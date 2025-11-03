/*
 * main.c
 *
 *  Author: MrReinsch
 *
 * This firmware drives a 5-LED warning panel using an ATtiny202.
 * It implements 8-bit Bit-Angle-Modulation (BAM) for smooth brightness control,
 * producing ~27 FPS sequences stored in PROGMEM. A single button cycles between
 * left, mid, and right light patterns (short press) or opens a timeout
 * configuration menu (long press). Timeout settings are saved to EEPROM, and
 * the whole system runs from the internal 20 MHz RC oscillator using a single
 * TCA0 timer for PWM and timing.
 *
 */

#include "avr/eeprom.h"
#include "avr/interrupt.h"
#include "avr/io.h"
#include "avr/pgmspace.h"
#include <xc.h>

// BSP
#define WARN_PA0_MASK (0x01u)
#define WARN_PA1_MASK (0x02u) // lef
#define WARN_PA2_MASK (0x04u) // left mid
#define WARN_PA3_MASK (0x08u) // right
#define WARN_PA6_MASK (0x40u) // right mid
#define WARN_PA7_MASK (0x80u) // Top

// Timer / BAM
#define WARN_BAM_BIT_0_DURATION (20u)
#define WARN_BAM_BIT_1_DURATION (60u)
#define WARN_BAM_BIT_2_DURATION (108u)
#define WARN_BAM_BIT_3_DURATION (202u)
#define WARN_BAM_BIT_4_DURATION (392u)
#define WARN_BAM_BIT_5_DURATION (768u)
#define WARN_BAM_BIT_6_DURATION (1700u) 
#define WARN_BAM_BIT_7_DURATION (3500u) 

#define WARN_TIMER_MAX_VALUE (0xFFFFu)
#define WARN_BAM_BIT_0_VALUE (WARN_TIMER_MAX_VALUE - WARN_BAM_BIT_0_DURATION)
#define WARN_BAM_BIT_1_VALUE (WARN_TIMER_MAX_VALUE - WARN_BAM_BIT_1_DURATION)
#define WARN_BAM_BIT_2_VALUE (WARN_TIMER_MAX_VALUE - WARN_BAM_BIT_2_DURATION)
#define WARN_BAM_BIT_3_VALUE (WARN_TIMER_MAX_VALUE - WARN_BAM_BIT_3_DURATION)
#define WARN_BAM_BIT_4_VALUE (WARN_TIMER_MAX_VALUE - WARN_BAM_BIT_4_DURATION)
#define WARN_BAM_BIT_5_VALUE (WARN_TIMER_MAX_VALUE - WARN_BAM_BIT_5_DURATION)
#define WARN_BAM_BIT_6_VALUE (WARN_TIMER_MAX_VALUE - WARN_BAM_BIT_6_DURATION)
#define WARN_BAM_BIT_7_VALUE (WARN_TIMER_MAX_VALUE - WARN_BAM_BIT_7_DURATION)

#define WARN_BAM_RESOLUTION_MAX (8u)
#define WARN_BAM_OUTPUT_MASK_BUFFERS_MAX (2u)

// Convert to GPIO Masks
#define WARN_NUM_OF_LEDS (5u)
#define WARN_LED_RIGHT_BIT_MASK (WARN_PA1_MASK)
#define WARN_LED_LEFT_BIT_MASK (WARN_PA2_MASK)
#define WARN_LED_LEFT_MID_BIT_MASK (WARN_PA6_MASK)
#define WARN_LED_RIGHT_MID_BIT_MASK (WARN_PA3_MASK)
#define WARN_LED_TOP_BIT_MASK (WARN_PA7_MASK)

#define WARN_LED_LEFT_ARROW_BIT_MASK (WARN_LED_LEFT_BIT_MASK | WARN_LED_LEFT_MID_BIT_MASK)
#define WARN_LED_RIGHT_ARROW_BIT_MASK (WARN_LED_RIGHT_BIT_MASK | WARN_LED_RIGHT_MID_BIT_MASK)
#define WARN_LED_MID_X_BIT_MASK (WARN_LED_LEFT_MID_BIT_MASK | WARN_LED_RIGHT_MID_BIT_MASK)

#define WARN_FRAMES_PER_SEQUENCE (256u)
#define WARN_FRAMES_SWITCH_TOP_MAX (64u)

typedef enum {
  WARN_SEQUENCE_OFF = 0,
  WARN_SEQUENCE_LEFT,
  WARN_SEQUENCE_MID,
  WARN_SEQUENCE_RIGHT,
  WARN_SEQUENCE_MAX,
  WARN_SEQUENCE_MASK = WARN_SEQUENCE_MAX - 1u,
  WARN_SEQUENCE_TIMEOUT = WARN_SEQUENCE_MASK + 1u,
} WARN_sequence_state_t;

typedef enum {
  WARN_TIMEOUT_OFF = 0u,
  WARN_TIMEOUT_120S = 1u,
  WARN_TIMEOUT_MAX = 2u,
} WARN_timeout_setting_t;

typedef enum {
  WARN_TIMEOUT_LIMIT_OFF = 0u,
  WARN_TIMEOUT_LIMIT_120S = 120u,
  WARN_TIMEOUT_LIMIT_MAX = 240u,
} WARN_timeout_limit_t;

typedef enum {
  WARN_PREPARE_BLOCKED = 0u,
  WARN_PREPARE_REQUEST = 1u,
  WARN_PREPARE_READY = 2u
} WARN_prepare_state_t;

typedef enum {
  WARN_BUTTON_IO_UNPRESSED = 0u,
  WARN_BUTTON_IO_DOWN = 1u,
  WARN_BUTTON_IO_LOCKED = 2u,
} WARN_button_io_state_t;

typedef enum {
  WARN_BUTTON_UNPRESSED = 0u,
  WARN_BUTTON_SHORT_PRESS = 1u,
  WARN_BUTTON_LONG_PRESS = 2u,
} WARN_button_event_t;

typedef enum {
  WARN_BUTTON_LONG_PRESS_VALUE = 200u,
  WARN_BUTTON_SHORT_PRESS_VALUE = 3u,
} WARN_button_press_values_t;

// clang-format off

const uint8_t WARN_sequenceLedValues[WARN_FRAMES_PER_SEQUENCE] PROGMEM = {
  128u, 128u, 128u, 128u, 255u, 255u, 255u, 255u, 
  255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u,
  255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u,
  255u, 255u, 255u, 255u, 128u, 128u, 128u, 128u,
  0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,
  0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,
  0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,
  0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,
  // 64 WARN_FRAMES_SWITCH_TOP_MAX
  0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,
  0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,
  3u,   3u,   4u,   4u,   5u,   5u,   6u,   6u,
  7u,   8u,   8u,   9u,   10u,  10u,  11u,  12u,
  13u,  14u,  15u,  16u,  17u,  18u,  19u,  20u,
  21u,  22u,  23u,  24u,  26u,  27u,  28u,  29u,
  31u,  32u,  34u,  35u,  37u,  38u,  40u,  41u,
  43u,  45u,  46u,  48u,  50u,  52u,  54u,  55u,
  // 128
  57u,  59u,  61u,  63u,  65u,  68u,  70u,  72u,
  74u,  76u,  79u,  81u,  83u,  86u,  88u,  91u,
  93u,  96u,  98u,  101u, 104u, 106u, 109u, 112u,
  115u, 117u, 120u, 123u, 126u, 127u, 127u, 127u,
  127u, 127u, 127u, 127u, 127u, 127u, 127u, 127u, 
  127u, 127u, 127u, 127u, 127u, 127u, 127u, 127u, 
  127u, 127u, 127u, 127u, 127u, 127u, 127u, 127u, 
  127u, 127u, 127u, 127u, 127u, 127u, 127u, 127u,
  //192
  127u, 127u, 127u, 127u, 127u, 127u, 127u, 127u, 
  127u, 127u, 127u, 127u, 127u, 127u, 127u, 127u, 
  127u, 127u, 127u, 127u, 127u, 127u, 127u, 127u, 
  127u, 127u, 127u, 127u, 127u, 127u, 127u, 127u, 
  0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,
  0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,
  0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,
  0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,
};

static const uint16_t WARN_bamTimerValueList[] = {
  (uint16_t)WARN_BAM_BIT_7_VALUE,
  (uint16_t)WARN_BAM_BIT_6_VALUE,
  (uint16_t)WARN_BAM_BIT_5_VALUE,
  (uint16_t)WARN_BAM_BIT_4_VALUE,
  (uint16_t)WARN_BAM_BIT_3_VALUE,
  (uint16_t)WARN_BAM_BIT_2_VALUE,
  (uint16_t)WARN_BAM_BIT_1_VALUE,
  (uint16_t)WARN_BAM_BIT_0_VALUE,
};
// clang-format on

static const uint8_t WARN_sequenceMaskLut[] = {
    (uint8_t)WARN_LED_LEFT_ARROW_BIT_MASK,
    (uint8_t)WARN_LED_LEFT_ARROW_BIT_MASK,
    (uint8_t)WARN_LED_MID_X_BIT_MASK,
    (uint8_t)WARN_LED_RIGHT_ARROW_BIT_MASK};

static uint8_t WARN_bamOutputMaskActiveBuffer[WARN_BAM_RESOLUTION_MAX];
static uint8_t WARN_bamOutputMaskPrepareBuffer[WARN_BAM_RESOLUTION_MAX];
static volatile WARN_prepare_state_t WARN_prepareState = WARN_PREPARE_REQUEST;
static volatile uint8_t WARN_bamStepTimerValueIndex;
static WARN_button_io_state_t WARN_buttonIoState;
static volatile uint8_t WARN_buttonFrameCounter;
static uint8_t WARN_buttonFrameCounterSnapShot;
static uint16_t WARN_timeoutCounter;
static WARN_timeout_setting_t WARN_timeoutSetting;
static uint8_t WARN_timeoutLimit;

WARN_timeout_setting_t WARN_eeTimeoutSetting EEMEM = WARN_TIMEOUT_MAX;

static WARN_timeout_limit_t getTimeoutCounterLimit(WARN_timeout_setting_t setting) {
  if (setting == WARN_TIMEOUT_OFF) {
    return WARN_TIMEOUT_LIMIT_OFF;
  }
  if (setting == WARN_TIMEOUT_120S) {
    return WARN_TIMEOUT_LIMIT_120S;
  }
  return WARN_TIMEOUT_LIMIT_MAX;
}

static inline void init(void) {
  PORTA_DIR = (uint8_t)(WARN_PA1_MASK | WARN_PA2_MASK | WARN_PA3_MASK | WARN_PA6_MASK | WARN_PA7_MASK);
  PORTA_PIN0CTRL = PORT_PULLUPEN_bm;
  VPORTA.OUT = 0u;
  TCA0_SINGLE_CTRLB = 0u;
  TCA0_SINGLE_CTRLC = 0u;
  TCA0_SINGLE_CTRLD = 0u;
  TCA0_SINGLE_INTCTRL = TCA_SINGLE_OVF_bm;
  TCA0_SINGLE_CNT = (uint16_t)WARN_BAM_BIT_0_VALUE;
  TCA0_SINGLE_PER = (uint16_t)WARN_TIMER_MAX_VALUE;
  TCA0_SINGLE_CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV4_gc;
  WARN_timeoutSetting = (WARN_timeout_setting_t)eeprom_read_byte(&WARN_eeTimeoutSetting);
  WARN_timeoutLimit = getTimeoutCounterLimit(WARN_timeoutSetting);
}

static inline void setPrepareStateReadySafe(void) {
  uint8_t s = SREG;
  cli();
  WARN_prepareState = WARN_PREPARE_READY;
  SREG = s;
}

static inline void setPrepareStatePrepareRequestSafe(void) {
  uint8_t s = SREG;
  cli();
  WARN_prepareState = WARN_PREPARE_REQUEST;
  SREG = s;
}

static inline void buildFrame(uint8_t value, uint8_t mask, uint8_t forceOff) {
  for (uint8_t b = 0u; b < WARN_BAM_RESOLUTION_MAX; b++) {
    uint8_t m = 0u;
    if (value & (1u << b)) {
      m = mask;
    }
    if (forceOff == WARN_SEQUENCE_OFF) {
      m = 0u;
    }
    WARN_bamOutputMaskPrepareBuffer[(WARN_BAM_RESOLUTION_MAX - 1u) - b] = m;
  }
}

static inline uint8_t getAbsoluteDelta(uint8_t a, uint8_t b) {
  return (uint8_t)a - b;
}

static inline uint8_t isButtonPressed(void) {
  return (VPORTA.IN & WARN_PA0_MASK) == 0u;
}

static WARN_button_event_t buttonHandler(void) {
  if (WARN_buttonIoState == WARN_BUTTON_IO_UNPRESSED) {
    if (isButtonPressed()) {
      WARN_buttonIoState = WARN_BUTTON_IO_DOWN;
      WARN_buttonFrameCounterSnapShot = WARN_buttonFrameCounter;
    }
  } else if (WARN_buttonIoState == WARN_BUTTON_IO_LOCKED) {
    if (!isButtonPressed()) {
      WARN_buttonIoState = WARN_BUTTON_IO_UNPRESSED;
    }
  } else {
    uint8_t delta = getAbsoluteDelta(WARN_buttonFrameCounter, WARN_buttonFrameCounterSnapShot);
    if (isButtonPressed()) {
      if (delta > WARN_BUTTON_LONG_PRESS_VALUE) {
        WARN_buttonIoState = WARN_BUTTON_IO_LOCKED;
        return WARN_BUTTON_LONG_PRESS;
      }
    } else {
      WARN_buttonIoState = WARN_BUTTON_IO_UNPRESSED;
      if (delta > WARN_BUTTON_SHORT_PRESS_VALUE) {
        return WARN_BUTTON_SHORT_PRESS;
      }
    }
  }
  return WARN_BUTTON_UNPRESSED;
}

static inline void setTimeoutLedPatternOff(void) {
  WARN_bamOutputMaskPrepareBuffer[0] = (uint8_t)(WARN_LED_TOP_BIT_MASK | WARN_LED_RIGHT_MID_BIT_MASK | WARN_LED_LEFT_MID_BIT_MASK);
}

static inline void setTimeoutLedPattern120s(void) {
  WARN_bamOutputMaskPrepareBuffer[0] = (uint8_t)(WARN_LED_TOP_BIT_MASK | WARN_LED_LEFT_BIT_MASK | WARN_LED_RIGHT_BIT_MASK);
}

static inline void setTimeoutLedPatternMax(void) {
  WARN_bamOutputMaskPrepareBuffer[0] = (uint8_t)(WARN_LED_TOP_BIT_MASK | WARN_LED_LEFT_BIT_MASK | WARN_LED_RIGHT_BIT_MASK | WARN_LED_LEFT_MID_BIT_MASK | WARN_LED_RIGHT_MID_BIT_MASK);
}

static inline void timeoutSettingMenu(void) {
  WARN_button_event_t event = WARN_BUTTON_UNPRESSED;
  for (uint8_t i = 1; i < WARN_BAM_RESOLUTION_MAX; i++) {
    WARN_bamOutputMaskPrepareBuffer[i] = 0u;
  }
  uint8_t forceUpdate = 1u;
  do {
    if (WARN_timeoutSetting == WARN_TIMEOUT_OFF) {
      setTimeoutLedPatternOff();
    } else if (WARN_timeoutSetting == WARN_TIMEOUT_120S) {
      setTimeoutLedPattern120s();
    } else {
      setTimeoutLedPatternMax();
    }
    if (event == WARN_BUTTON_SHORT_PRESS) {
      WARN_timeoutSetting++;
      if (WARN_timeoutSetting > WARN_TIMEOUT_MAX) {
        WARN_timeoutSetting = WARN_TIMEOUT_OFF;
      }
      forceUpdate = 1u;
    }
    if (forceUpdate > 0u) {
      setPrepareStateReadySafe();
      forceUpdate = 0u;
    }
    event = buttonHandler();
  } while (event != WARN_BUTTON_LONG_PRESS);
  eeprom_update_byte(&WARN_eeTimeoutSetting, (uint8_t)WARN_timeoutSetting);
}

int main(void) {
  uint8_t frameCounter = 0u;
  uint8_t isTimeoutActive = 0u;
  WARN_sequence_state_t sequence = WARN_SEQUENCE_LEFT;
  WARN_sequence_state_t sequenceBeforeTimeout = WARN_SEQUENCE_LEFT;
  init();
  sei();
  while (1) {
    WARN_button_event_t event = buttonHandler();
    if (event == WARN_BUTTON_SHORT_PRESS) {
      if (isTimeoutActive == 0u) {
        sequence = (++sequence) & WARN_SEQUENCE_MASK;
      } else {
        sequence = sequenceBeforeTimeout;
        isTimeoutActive = 0u;
      }
      setPrepareStateReadySafe();
      frameCounter = 0u;
      WARN_timeoutCounter = 0u;
    } else if (event == WARN_BUTTON_LONG_PRESS) {
      sequenceBeforeTimeout = sequence;
      timeoutSettingMenu();
      WARN_timeoutLimit = getTimeoutCounterLimit(WARN_timeoutSetting);
      sequence = sequenceBeforeTimeout;
      frameCounter = 0u;
    }
    if (WARN_prepareState == WARN_PREPARE_REQUEST) {
      frameCounter++;
      if (frameCounter >= (WARN_FRAMES_PER_SEQUENCE)) {
        WARN_timeoutCounter++;
      }
      uint8_t mask = (uint8_t)WARN_LED_TOP_BIT_MASK;
      if (frameCounter > (uint8_t)WARN_FRAMES_SWITCH_TOP_MAX) {
        mask = WARN_sequenceMaskLut[sequence];
      }
      uint8_t value = pgm_read_byte(&WARN_sequenceLedValues[frameCounter]);
      buildFrame(value, mask, sequence);
      setPrepareStateReadySafe();
    }
    if ((WARN_timeoutSetting != WARN_TIMEOUT_OFF) && (WARN_timeoutCounter >= WARN_timeoutLimit) && (isTimeoutActive == 0u)) {
      sequenceBeforeTimeout = sequence;
      sequence = WARN_SEQUENCE_OFF;
      isTimeoutActive = 1u;
      frameCounter = 0u;
    }
  }
}

ISR(TCA0_OVF_vect) {
  if (WARN_bamStepTimerValueIndex >= (uint8_t)WARN_BAM_RESOLUTION_MAX) {
    WARN_bamStepTimerValueIndex = 0u;
    WARN_buttonFrameCounter++;
    if (WARN_prepareState == WARN_PREPARE_READY) {
      VPORTA.OUT = WARN_bamOutputMaskPrepareBuffer[WARN_bamStepTimerValueIndex];
      for (uint8_t i = 0u; i < (uint8_t)WARN_BAM_RESOLUTION_MAX; i++) {
        WARN_bamOutputMaskActiveBuffer[i] = WARN_bamOutputMaskPrepareBuffer[i];
      }
      WARN_prepareState = WARN_PREPARE_REQUEST;
    }
  }
  TCA0_SINGLE_CNT = WARN_bamTimerValueList[WARN_bamStepTimerValueIndex];
  VPORTA.OUT = WARN_bamOutputMaskActiveBuffer[WARN_bamStepTimerValueIndex];
  WARN_bamStepTimerValueIndex++;
  TCA0_SINGLE_INTFLAGS = TCA_SINGLE_OVF_bm;
}