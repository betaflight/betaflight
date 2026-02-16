# RP2350 PWM Motor Output Bug Analysis

**Date:** February 16, 2026  
**Status:** DEFERRED (DSHOT600 works, PWM not needed currently)  
**GitHub Issue:** https://github.com/betaflight/betaflight/issues/14743  
**Affected Targets:** PICO2_RP2350A, MADFLIGHT_FC3, all RP2350-based boards  

---

## Problem Description

PWM-based motor protocols (PWM, Oneshot125, Oneshot42, Multishot) produce **no output** on RP2350 targets. DSHOT600 works correctly.

**Symptoms:**
- No PWM signal visible on oscilloscope
- Motor sliders in Configurator show values but motors don't spin
- DSHOT600 protocol works perfectly on same hardware

---

## Root Cause Analysis

### Bug Location
File: `src/platform/PICO/pwm_motor_pico.c`, function `motorPwmDevInit()`

### Bug 1: Clock Divider Integer Overflow (CRITICAL)

```c
// Line ~188-189 in pwm_motor_pico.c:
const uint8_t interval = (uint8_t)(clock / period);
const uint8_t fraction = (uint8_t)(((clock / period) - interval) * (0x01 << 4));
```

**Problem:** 
- `clock` = 150,000,000 Hz (RP2350 @ 150MHz)
- `period` = 65535 (0xFFFF for non-continuous mode)
- `clock / period` = **2289**
- Cast to `uint8_t` causes **overflow** → result is 241 (2289 % 256)

**Impact:** PWM divider is set to wrong value, resulting in incorrect or no PWM output.

### Bug 2: Prescaler Calculated But Not Used

```c
// Line ~171-173:
const unsigned prescaler = ceilf(clock / pwmRateHz);  // Calculated but NEVER USED!
const uint32_t hz = clock / prescaler;
const unsigned period = useContinuousUpdate ? hz / pwmRateHz : 0xffff;

// Line ~183-189: Uses completely different formula!
pwm_config config = pwm_get_default_config();
const uint8_t interval = (uint8_t)(clock / period);  // Wrong! Should use prescaler
```

**Problem:** The `prescaler` variable is calculated correctly but then ignored when configuring the PWM hardware.

### Bug 3: Missing PWM Enable (Potential)

Compare with working servo code:
```c
// pwm_servo_pico.c (WORKS):
pwm_set_enabled(slice, true);  // Explicit enable

// pwm_motor_pico.c (BROKEN):
pwm_init(slice, &config, true);  // Relies on pwm_init's start parameter
```

While `pwm_init(..., true)` should start the PWM, explicit `pwm_set_enabled()` is more reliable.

---

## Comparison: Working vs Broken Code

### Working: Servo PWM (`pwm_servo_pico.c`)
```c
#define PWM_PRESCALER 64.0f 
#define PWM_TOP_COUNT ((uint16_t)roundf((SYS_CLK_HZ / SERVO_PWM_FREQUENCY_HZ) / PWM_PRESCALER))

// Simple, correct approach:
pwm_set_clkdiv(slice, PWM_PRESCALER);
pwm_set_wrap(slice, PWM_TOP_COUNT);
pwm_set_enabled(slice, true);
```

### Working: Beeper PWM (`pwm_beeper_pico.c`)
```c
uint32_t clock_divide = (SystemCoreClock / frequency + 0xfffe) / 0xffff;
if (clock_divide > 255) {
    clock_divide = 255;  // Clamped to valid range!
    wrap = 0xffff;
}
pwm_config_set_clkdiv_int(&cfg, clock_divide);
```

### Working: STM32 Motor PWM (`pwm_output_hw.c`)
```c
const unsigned prescaler = ((clock / pwmRateHz) + 0xffff) / 0x10000;  // Proper calc
const uint32_t hz = clock / prescaler;
const unsigned period = useContinuousUpdate ? hz / pwmRateHz : 0xffff;
// Uses prescaler correctly in timer configuration
```

### Broken: PICO Motor PWM (`pwm_motor_pico.c`)
```c
const unsigned prescaler = ceilf(clock / pwmRateHz);  // = 150000 (way too big!)
const uint32_t hz = clock / prescaler;                // = 1000
const unsigned period = useContinuousUpdate ? hz / pwmRateHz : 0xffff;

// Then ignores prescaler and uses wrong formula:
const uint8_t interval = (uint8_t)(clock / period);   // OVERFLOW!
```

---

## Proposed Fix

Replace the PWM configuration code in `motorPwmDevInit()`:

```c
// BEFORE (broken):
const uint8_t interval = (uint8_t)(clock / period);
const uint8_t fraction = (uint8_t)(((clock / period) - interval) * (0x01 << 4));
pwm_config_set_clkdiv_int_frac(&config, interval, fraction);

// AFTER (fixed):
// RP2350 PWM divider is 8.4 fixed point (max ~255.9375)
// Need: divider = clock / (period * pwmRateHz)
// For Oneshot125 @ 4kHz: divider = 150MHz / (65535 * 4000) = 0.57 (use 1.0 min)
// For continuous PWM @ 490Hz: divider = 150MHz / (period * 490)

float divider;
if (useContinuousUpdate) {
    // Continuous mode: fixed pwmRateHz, calculate period
    divider = (float)clock / ((float)period * (float)pwmRateHz);
} else {
    // Oneshot mode: fixed period (0xFFFF), high update rate
    // Need divider that allows pulse widths of 125-250us at this clock
    // pulse_counts = pulse_time * clock / divider
    // For 125us pulse: counts = 125e-6 * 150e6 / divider = 18750 / divider
    // Want counts < 65535, so divider > 0.286 (use 1.0 minimum)
    divider = 1.0f;
}

// Clamp divider to valid range (1.0 to 255.9375)
if (divider < 1.0f) divider = 1.0f;
if (divider > 255.0f) divider = 255.0f;

pwm_config_set_clkdiv(&config, divider);
pwm_config_set_wrap(&config, period);

gpio_set_function(pin, GPIO_FUNC_PWM);
pwm_init(slice, &config, true);
pwm_set_enabled(slice, true);  // Explicit enable for safety
```

---

## Testing Plan

1. **Build with fix:**
   ```bash
   make CONFIG=MADFLIGHT_FC3
   ```

2. **Test Oneshot125:**
   - Set `motor_pwm_protocol = ONESHOT125` in CLI
   - Connect oscilloscope to motor pin
   - Expected: 125-250µs pulses at ~4kHz when motor slider moved

3. **Test Standard PWM:**
   - Set `motor_pwm_protocol = PWM`
   - Expected: 1000-2000µs pulses at configured rate (default 490Hz)

4. **Test Brushed:**
   - Set `motor_pwm_protocol = BRUSHED`
   - Expected: 0-100% duty cycle PWM

---

## Workaround (Current)

Use **DSHOT600** protocol instead of PWM-based protocols:

```
# In CLI:
set motor_pwm_protocol = DSHOT600
save
```

DSHOT600 uses PIO (Programmable I/O) instead of hardware PWM, so it's unaffected by this bug.

---

## Files to Modify

| File | Change |
|------|--------|
| `src/platform/PICO/pwm_motor_pico.c` | Fix clock divider calculation, add explicit enable |

---

## Priority

**LOW** - DSHOT600 is the preferred protocol for modern ESCs and works correctly. This bug only affects users with older analog ESCs requiring PWM/Oneshot protocols.

---

## Related Issues

- GitHub Issue #14743: "No PWM motor output on target PICO2_2350A"
- Affects: All RP2350-based Betaflight targets
- Does NOT affect: DSHOT protocols (use PIO, not PWM hardware)
