/*
 * main.c
 *
 *  Author: MrReinsch
 * 
 * This firmware drives a 5-LED warning panel using an ATtiny202.  
 * It implements 8-bit Bit-Angle-Modulation (BAM) for smooth brightness control, producing ~27 FPS sequences stored in PROGMEM.  
 * A single button cycles between left, mid, and right light patterns (short press) or opens a timeout configuration menu (long press).  
 * Timeout settings are saved to EEPROM, and the whole system runs from the internal 20 MHz RC oscillator using a single TCA0 timer for PWM and timing.
 * 
 */

#include "avr/eeprom.h"
#include "avr/interrupt.h"
#include "avr/io.h"
#include "avr/pgmspace.h"
#include <xc.h>

// BSP
#define PA0_MASK (0x01u)
#define PORT_IN_MASK (PA0_MASK)
#define PA1_MASK (0x02u) // lef
#define PA2_MASK (0x04u) // left mid
#define PA3_MASK (0x08u) // right
#define PA6_MASK (0x40u) // right mid
#define PA7_MASK (0x80u) // Top

// Timer / BAM
#define FRAMES_PER_SECOND (28u)
#define BAM_BIT_0_DURATION (20u) // ~1.2us*20
#define BAM_BIT_1_DURATION (BAM_BIT_0_DURATION * 2u)
#define BAM_BIT_2_DURATION (BAM_BIT_0_DURATION * 4u)
#define BAM_BIT_3_DURATION (BAM_BIT_0_DURATION * 8u)
#define BAM_BIT_4_DURATION (BAM_BIT_0_DURATION * 16u)
#define BAM_BIT_5_DURATION (BAM_BIT_0_DURATION * 32u)
#define BAM_BIT_6_DURATION (BAM_BIT_0_DURATION * 64u)
#define BAM_BIT_7_DURATION (BAM_BIT_0_DURATION * 128u)

#define TIMER_MAX_VALUE (0xFFFFu)
#define BAM_BIT_0_VALUE (TIMER_MAX_VALUE - BAM_BIT_0_DURATION)
#define BAM_BIT_1_VALUE (TIMER_MAX_VALUE - BAM_BIT_1_DURATION)
#define BAM_BIT_2_VALUE (TIMER_MAX_VALUE - BAM_BIT_2_DURATION)
#define BAM_BIT_3_VALUE (TIMER_MAX_VALUE - BAM_BIT_3_DURATION)
#define BAM_BIT_4_VALUE (TIMER_MAX_VALUE - BAM_BIT_4_DURATION)
#define BAM_BIT_5_VALUE (TIMER_MAX_VALUE - BAM_BIT_5_DURATION)
#define BAM_BIT_6_VALUE (TIMER_MAX_VALUE - BAM_BIT_6_DURATION)
#define BAM_BIT_7_VALUE (TIMER_MAX_VALUE - BAM_BIT_7_DURATION)

#define BAM_RESOLUTION_MAX (8u)
#define BAM_OUTPUT_MASK_BUFFERS_MAX (2u)
#define BAM_REPETITION_CYCLES_PER_FRAME (6u)

// Convert to GPIO Masks
#define NUM_OF_LEDS (5u)
#define LED_RIGHT_BIT_MASK (PA1_MASK)
#define LED_LEFT_BIT_MASK (PA2_MASK)
#define LED_LEFT_MID_BIT_MASK (PA6_MASK)
#define LED_RIGHT_MID_BIT_MASK (PA3_MASK)
#define LED_TOP_BIT_MASK (PA7_MASK)

#define FRAMES_PER_SEQUENCE (56u)
#define FRAMES_PER_SEQUENCE_MASK (FRAMES_PER_SEQUENCE - 1u)

#define BUTTON_LONG_PRESS_COUNTER_VALUE (200u)
#define BUTTON_SHORT_PRESS_COUNTER_VALUE (3u)

#define TIMEOUT_LIMIT_OFF (0u)
#define TIMEOUT_LIMIT_120S (60u)
#define TIMEOUT_LIMIT_MAX (240u)

typedef enum {
  SEQUENCE_OFF = 0,
  SEQUENCE_LEFT,
  SEQUENCE_MID,
  SEQUENCE_RIGHT,
  SEQUENCE_MAX,
  SEQUENCE_MASK = SEQUENCE_MAX - 1u,
  SEQUENCE_TIMEOUT = SEQUENCE_MASK + 1u,
} sequence_state_t;

typedef enum {
  TIMEOUT_OFF = 0u,
  TIMEOUT_120S = 1u,
  TIMEOUT_MAX = 2u,
} timeout_setting_t;

typedef enum {
  PREPARE_BLOCKED = 0,
  PREPARE_REQUEST = 1,
  PREPARE_READY = 2
} prepare_state_t;

typedef enum {
  BUTTON_IO_UNPRESSED = 0,
  BUTTON_IO_DOWN = 1,
  BUTTON_IO_LOCKED = 2,
} button_io_state_t;

typedef enum {
  BUTTON_UNPRESSED = 0,
  BUTTON_SHORT_PRESS = 1,
  BUTTON_LONG_PRESS = 2,
} button_event_t;

// TOP, LEFT, MID LEFT, RIGHT, RIGHT MID
const uint8_t sequenceLeftLedValues[FRAMES_PER_SEQUENCE][NUM_OF_LEDS] PROGMEM =
    {
        {128u, 0u, 0u, 0u, 0u}, 
        {128u, 0u, 0u, 0u, 0u}, 
        {255u, 0u, 0u, 0u, 0u}, 
        {255u, 0u, 0u, 0u, 0u}, 
        {255u, 0u, 0u, 0u, 0u}, 
        {255u, 0u, 0u, 0u, 0u}, 
        {255u, 0u, 0u, 0u, 0u}, 
        {128u, 0u, 0u, 0u, 0u}, 
        //
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        //
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        //
        {0u, 16u, 16u, 0u, 0u},
        {0u, 16u, 16u, 0u, 0u},
        {0u, 32u, 34u, 0u, 0u},
        {0u, 32u, 32u, 0u, 0u},
        {0u, 48u, 48u, 0u, 0u},
        {0u, 64u, 64u, 0u, 0u},
        {0u, 64u, 64u, 0u, 0u},
        {0u, 96u, 96u, 0u, 0u},
        // 32
        {0u, 96u, 96u, 0u, 0u},  
        {0u, 128u, 128u, 0u, 0u},
        {0u, 128u, 128u, 0u, 0u},
        {0u, 144u, 144u, 0u, 0u},
        {0u, 164u, 164u, 0u, 0u},
        {0u, 164u, 164u, 0u, 0u},
        {0u, 196u, 196u, 0u, 0u},
        {0u, 196u, 196u, 0u, 0u},
        //
        {0u, 255u, 255u, 0u, 0u},
        {0u, 255u, 255u, 0u, 0u},
        {0u, 255u, 255u, 0u, 0u},
        {0u, 255u, 255u, 0u, 0u},
        {0u, 255u, 255u, 0u, 0u},
        {0u, 255u, 255u, 0u, 0u},
        {0u, 255u, 255u, 0u, 0u},
        {0u, 255u, 255u, 0u, 0u},
        //
        {0u, 255u, 255u, 0u, 0u},
        {0u, 255u, 255u, 0u, 0u},
        {0u, 255u, 255u, 0u, 0u},
        {0u, 255u, 255u, 0u, 0u},
        {0u, 255u, 255u, 0u, 0u},
        {0u, 255u, 255u, 0u, 0u},
        {0u, 255u, 255u, 0u, 0u},
        {0u, 255u, 255u, 0u, 0u},
};

const uint8_t sequenceMidLedValues[FRAMES_PER_SEQUENCE][NUM_OF_LEDS] PROGMEM = {
    {128u, 0u, 0u, 0u, 0u},
    {128u, 0u, 0u, 0u, 0u},
    {255u, 0u, 0u, 0u, 0u},
    {255u, 0u, 0u, 0u, 0u},
    {255u, 0u, 0u, 0u, 0u},
    {255u, 0u, 0u, 0u, 0u},
    {255u, 0u, 0u, 0u, 0u},
    {128u, 0u, 0u, 0u, 0u},
    //
    {0u, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u},
    //
    {0u, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u},
    //
    {0u, 0u, 16u, 0u, 16u},
    {0u, 0u, 16u, 0u, 16u},
    {0u, 0u, 32u, 0u, 34u},
    {0u, 0u, 32u, 0u, 32u},
    {0u, 0u, 48u, 0u, 48u},
    {0u, 0u, 64u, 0u, 64u},
    {0u, 0u, 64u, 0u, 64u},
    {0u, 0u, 96u, 0u, 96u},
    //
    {0u, 0u, 96u, 0u, 96u},  
    {0u, 0u, 128u, 0u, 128u},
    {0u, 0u, 128u, 0u, 128u},
    {0u, 0u, 144u, 0u, 144u},
    {0u, 0u, 164u, 0u, 164u},
    {0u, 0u, 164u, 0u, 164u},
    {0u, 0u, 196u, 0u, 196u},
    {0u, 0u, 196u, 0u, 196u},
    //
    {0u, 0u, 255u, 0u, 255u},
    {0u, 0u, 255u, 0u, 255u},
    {0u, 0u, 255u, 0u, 255u},
    {0u, 0u, 255u, 0u, 255u},
    {0u, 0u, 255u, 0u, 255u},
    {0u, 0u, 255u, 0u, 255u},
    {0u, 0u, 255u, 0u, 255u},
    {0u, 0u, 255u, 0u, 255u},
    //
    {0u, 0u, 255u, 0u, 255u},
    {0u, 0u, 255u, 0u, 255u},
    {0u, 0u, 255u, 0u, 255u},
    {0u, 0u, 255u, 0u, 255u},
    {0u, 0u, 255u, 0u, 255u},
    {0u, 0u, 255u, 0u, 255u},
    {0u, 0u, 255u, 0u, 255u},
    {0u, 0u, 255u, 0u, 255u},
};

const uint8_t sequenceRightLedValues[FRAMES_PER_SEQUENCE][NUM_OF_LEDS] PROGMEM =
    {
        {128u, 0u, 0u, 0u, 0u},
        {128u, 0u, 0u, 0u, 0u},
        {255u, 0u, 0u, 0u, 0u},
        {255u, 0u, 0u, 0u, 0u},
        {255u, 0u, 0u, 0u, 0u},
        {255u, 0u, 0u, 0u, 0u},
        {255u, 0u, 0u, 0u, 0u},
        {128u, 0u, 0u, 0u, 0u},
        //
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        //
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        {0u, 0u, 0u, 0u, 0u},
        //
        {0u, 0u, 0u, 16u, 16u},
        {0u, 0u, 0u, 16u, 16u},
        {0u, 0u, 0u, 32u, 34u},
        {0u, 0u, 0u, 32u, 32u},
        {0u, 0u, 0u, 48u, 48u},
        {0u, 0u, 0u, 64u, 64u},
        {0u, 0u, 0u, 64u, 64u},
        {0u, 0u, 0u, 96u, 96u},
        //
        {0u, 0u, 0u, 96u, 96u},  
        {0u, 0u, 0u, 128u, 128u},
        {0u, 0u, 0u, 128u, 128u},
        {0u, 0u, 0u, 144u, 144u},
        {0u, 0u, 0u, 164u, 164u},
        {0u, 0u, 0u, 164u, 164u},
        {0u, 0u, 0u, 196u, 196u},
        {0u, 0u, 0u, 196u, 196u},
        //
        {0u, 0u, 0u, 255u, 255u},
        {0u, 0u, 0u, 255u, 255u},
        {0u, 0u, 0u, 255u, 255u},
        {0u, 0u, 0u, 255u, 255u},
        {0u, 0u, 0u, 255u, 255u},
        {0u, 0u, 0u, 255u, 255u},
        {0u, 0u, 0u, 255u, 255u},
        {0u, 0u, 0u, 255u, 255u},
        //
        {0u, 0u, 0u, 255u, 255u},
        {0u, 0u, 0u, 255u, 255u},
        {0u, 0u, 0u, 255u, 255u},
        {0u, 0u, 0u, 255u, 255u},
        {0u, 0u, 0u, 255u, 255u},
        {0u, 0u, 0u, 255u, 255u},
        {0u, 0u, 0u, 255u, 255u},
        {0u, 0u, 0u, 255u, 255u},
};

static const uint16_t bamTimerValueList[] = {
    (uint16_t)BAM_BIT_7_VALUE, (uint16_t)BAM_BIT_6_VALUE,
    (uint16_t)BAM_BIT_5_VALUE, (uint16_t)BAM_BIT_4_VALUE,
    (uint16_t)BAM_BIT_3_VALUE, (uint16_t)BAM_BIT_2_VALUE,
    (uint16_t)BAM_BIT_1_VALUE, (uint16_t)BAM_BIT_0_VALUE,
};

typedef const uint8_t (*sequence_t)[NUM_OF_LEDS];

static const sequence_t sequencePointerLut[] = {
    sequenceLeftLedValues, sequenceLeftLedValues, sequenceMidLedValues,
    sequenceRightLedValues};

static uint8_t bamOutputMaskActiveBuffer[BAM_RESOLUTION_MAX] = {0};
static uint8_t bamOutputMaskPrepareBuffer[BAM_RESOLUTION_MAX] = {0};
static volatile prepare_state_t prepareState = PREPARE_REQUEST;
static volatile uint8_t bamStepTimerValueIndex = 0u;
static button_io_state_t buttonIoState = BUTTON_IO_UNPRESSED;
static volatile uint8_t buttonFrameCounter = 0u;
static uint8_t buttonFrameCounterSnapShot = 0u;
static uint8_t bamRepetionCycleCounter = 0u;
static uint16_t timeoutCounter = 0u;
static timeout_setting_t timeoutSetting;
static uint8_t timeoutCounterLimit = 0u;
timeout_setting_t eeTimeoutSetting EEMEM = TIMEOUT_MAX;

static uint8_t getTimeoutCounterLimit(timeout_setting_t setting) {
  if (setting == (uint8_t)TIMEOUT_OFF) {
    return TIMEOUT_LIMIT_OFF;
  }
  if (setting == (uint8_t)TIMEOUT_120S) {
    return (uint8_t)TIMEOUT_LIMIT_120S;
  }
  return (uint8_t)TIMEOUT_LIMIT_MAX;
}

static inline void init(void) {
  PORTA_DIR = (uint8_t)(PA1_MASK | PA2_MASK | PA3_MASK | PA6_MASK | PA7_MASK);
  PORTA_PIN0CTRL = PORT_PULLUPEN_bm;
  VPORTA.OUT = 0u;
  TCA0_SINGLE_CTRLB = 0u;
  TCA0_SINGLE_CTRLC = 0u;
  TCA0_SINGLE_CTRLD = 0u;
  TCA0_SINGLE_INTCTRL = TCA_SINGLE_OVF_bm;
  TCA0_SINGLE_CNT = (uint16_t)BAM_BIT_0_VALUE;
  TCA0_SINGLE_PER = (uint16_t)TIMER_MAX_VALUE;
  TCA0_SINGLE_CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV4_gc;
  timeoutSetting = (timeout_setting_t)eeprom_read_byte(&eeTimeoutSetting);
  timeoutCounterLimit = getTimeoutCounterLimit(timeoutSetting);
}

static inline void setPrepareStateReadySafe(void) {
  uint8_t s = SREG;
  cli();
  prepareState = PREPARE_READY;
  SREG = s;
}

static inline void setPrepareStatePrepareRequestSafe(void) {
  uint8_t s = SREG;
  cli();
  prepareState = PREPARE_REQUEST;
  SREG = s;
}

static inline void buildFrame(const uint8_t *value, uint8_t forceOff) {
  for (uint8_t b = 0u; b < BAM_RESOLUTION_MAX; b++) {
    uint8_t m = 0u;
    uint8_t buffer = pgm_read_byte(&value[0]);
    if (buffer & (1u << b)) {
      m |= LED_TOP_BIT_MASK;
    }
    buffer = pgm_read_byte(&value[1]);
    if (buffer & (1u << b)) {
      m |= LED_LEFT_BIT_MASK;
    }
    buffer = pgm_read_byte(&value[2]);
    if (buffer & (1u << b)) {
      m |= LED_LEFT_MID_BIT_MASK;
    }
    buffer = pgm_read_byte(&value[3]);
    if (buffer & (1u << b)) {
      m |= LED_RIGHT_BIT_MASK;
    }
    buffer = pgm_read_byte(&value[4]);
    if (buffer & (1u << b)) {
      m |= LED_RIGHT_MID_BIT_MASK;
    }
    if (forceOff == SEQUENCE_OFF) {
      m = 0u;
    }
    bamOutputMaskPrepareBuffer[(BAM_RESOLUTION_MAX - 1u) - b] = m;
  }
}

static inline uint8_t getAbsoluteDelta(uint8_t a, uint8_t b) {
  return (uint8_t)a - b;
}

static inline uint8_t isButtonPressed(void) {
  return (VPORTA.IN & PA0_MASK) == 0u;
}

static button_event_t buttonHandler(void) {
  if (buttonIoState == BUTTON_IO_UNPRESSED) {
    if (isButtonPressed()) {
      buttonIoState = BUTTON_IO_DOWN;
      buttonFrameCounterSnapShot = buttonFrameCounter;
    }
  } else if (buttonIoState == BUTTON_IO_LOCKED) {
    if (!isButtonPressed()) {
      buttonIoState = BUTTON_IO_UNPRESSED;
    }
  } else {
    uint8_t delta =
        getAbsoluteDelta(buttonFrameCounter, buttonFrameCounterSnapShot);
    if (isButtonPressed()) {
      if (delta > BUTTON_LONG_PRESS_COUNTER_VALUE) {
        buttonIoState = BUTTON_IO_LOCKED;
        return BUTTON_LONG_PRESS;
      }
    } else {
      buttonIoState = BUTTON_IO_UNPRESSED;
      if (delta > BUTTON_SHORT_PRESS_COUNTER_VALUE) {
        return BUTTON_SHORT_PRESS;
      }
    }
  }
  return BUTTON_UNPRESSED;
}

static inline void timeoutSettingMenu() {
  button_event_t event = BUTTON_UNPRESSED;
  for (uint8_t i = 1; i < BAM_RESOLUTION_MAX; i++) {
    bamOutputMaskPrepareBuffer[i] = 0u;
  }
  uint8_t forceUpdate = 1u;
  do {
    if (timeoutSetting == (uint8_t)TIMEOUT_OFF) {
      bamOutputMaskPrepareBuffer[0u] =
          LED_TOP_BIT_MASK | LED_RIGHT_MID_BIT_MASK | LED_LEFT_MID_BIT_MASK;
    } else if (timeoutSetting == (uint8_t)TIMEOUT_120S) {
      bamOutputMaskPrepareBuffer[0u] =
          LED_TOP_BIT_MASK | LED_LEFT_BIT_MASK | LED_RIGHT_BIT_MASK;
    } else {
      bamOutputMaskPrepareBuffer[0u] =
          LED_TOP_BIT_MASK | LED_LEFT_BIT_MASK | LED_RIGHT_BIT_MASK |
          LED_LEFT_MID_BIT_MASK | LED_RIGHT_MID_BIT_MASK;
    }
    if (event == BUTTON_SHORT_PRESS) {
      timeoutSetting++;
      if (timeoutSetting > TIMEOUT_MAX) {
        timeoutSetting = TIMEOUT_OFF;
      }
      forceUpdate = 1u;
    }
    if (forceUpdate > 0u) {
      setPrepareStateReadySafe();
      forceUpdate = 0u;
    }
    event = buttonHandler();
  } while (event != BUTTON_LONG_PRESS);
  eeprom_update_byte(&eeTimeoutSetting, (uint8_t)timeoutSetting);
}

int main(void) {
  uint8_t frameCounter = 0u;
  uint8_t isTimeoutActive = 0u;
  sequence_state_t sequence = SEQUENCE_LEFT;
  sequence_state_t sequenceBeforeTimeout = SEQUENCE_LEFT;
  init();
  sei();
  while (1) {
    button_event_t event = buttonHandler();
    if (event == BUTTON_SHORT_PRESS) {
      if (isTimeoutActive == 0u) {
        sequence = (++sequence) & SEQUENCE_MASK;
      } else {
        sequence = sequenceBeforeTimeout;
        isTimeoutActive = 0u;
      }
      setPrepareStateReadySafe();
      frameCounter = 0u;
      timeoutCounter = 0u;
    } else if (event == BUTTON_LONG_PRESS) {
      sequenceBeforeTimeout = sequence;
      timeoutSettingMenu();
      timeoutCounterLimit = getTimeoutCounterLimit(timeoutSetting);
      sequence = sequenceBeforeTimeout;
      frameCounter = 0u;
    }
    if (prepareState == PREPARE_REQUEST) {
      frameCounter++;
      if (frameCounter >= FRAMES_PER_SEQUENCE) {
        frameCounter = 0u;
        timeoutCounter++;
      }
      sequence_t seq = sequencePointerLut[sequence];
      buildFrame(seq[frameCounter], sequence);
      setPrepareStateReadySafe();
    }
    if ((timeoutSetting != (uint8_t)TIMEOUT_OFF) &&
        (timeoutCounter >= timeoutCounterLimit) && (isTimeoutActive == 0u)) {
      sequenceBeforeTimeout = sequence;
      sequence = (uint8_t)SEQUENCE_OFF;
      isTimeoutActive = 1u;
      frameCounter = 0u;
    }
  }
}

ISR(TCA0_OVF_vect) {
  if (bamStepTimerValueIndex >= (uint8_t)BAM_RESOLUTION_MAX) {
    bamStepTimerValueIndex = 0u;
    if (++bamRepetionCycleCounter >= BAM_REPETITION_CYCLES_PER_FRAME) {
      bamRepetionCycleCounter = 0;
      buttonFrameCounter++;
      if (prepareState == PREPARE_READY) {
        VPORTA.OUT = bamOutputMaskPrepareBuffer[bamStepTimerValueIndex];
        for (uint8_t i = 0u; i < (uint8_t)BAM_RESOLUTION_MAX; i++) {
          bamOutputMaskActiveBuffer[i] = bamOutputMaskPrepareBuffer[i];
        }
        prepareState = PREPARE_REQUEST;
      }
    }
  }
  TCA0_SINGLE_CNT = bamTimerValueList[bamStepTimerValueIndex];
  VPORTA.OUT = bamOutputMaskActiveBuffer[bamStepTimerValueIndex];
  bamStepTimerValueIndex++;
  TCA0_SINGLE_INTFLAGS = TCA_SINGLE_OVF_bm;
}