# LED Warning Panel – ATtiny202

Firmware for a 5-LED warning panel driven by an **ATtiny202** (compiled as ATtiny402 for larger image size in AVR Studio).  
Implements smooth light effects using **8-bit Bit-Angle-Modulation (BAM)** with frame-based animation control and a single push-button interface.

---

## Overview

- **Microcontroller:** ATtiny202 (internal 20 MHz RC oscillator)  
- **Timer:** TCA0 used for both PWM (BAM) and frame timing  
- **LEDs:** 5 channels on Port A (PA1, PA2, PA3, PA6, PA7)  
- **Button:** PA0 (active-low with internal pull-up)  
- **Storage:** EEPROM used for persistent timeout configuration  

---

## Timing & Modulation

- CPU clock: 20 MHz / 6 ≈ 3.33 MHz  
- Timer A prescaler: /4 → 0.833 MHz (≈ 1.2 µs per tick)  
- BAM: 8 bit, LSB = 20 ticks (≈ 24 µs) -> 36us (measuered) 
- Bit durations: 20 – 3860 ticks (Bit 7 → Bit 0)  
- ~100 frames per second  
- Double-buffered bitplanes for flicker-free output  

---

## Sequences

- **3 patterns:** Left, Mid, Right 
- Data stored in **PROGMEM**  
- “Off” state implemented via forced zero mask  

---

## Button Logic

- Short press → cycle between sequences  
- Long press → open timeout-setting menu  
- Timing derived from frame counter (1 frame ≈ 36.7 ms)  
- Short ≥ 3 frames (~110 ms), Long ≥ 200 frames (~7.3 s)

---

## Timeout Feature

- Configurable timeout: **OFF / 120 s / MAX (240 s)**    
- Setting saved to EEPROM (restored on boot)

---

## Timer ISR Summary

```c
ISR(TCA0_OVF_vect)
