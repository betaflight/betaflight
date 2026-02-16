# PICO Platform I2C Error Handling Fix

## Overview

This PR fixes I2C bus error handling on RP2040/RP2350 PICO platforms, enabling proper detection of transfer errors (NACK, bus errors).

## Problem

The PICO I2C implementation was not properly detecting or reporting I2C transfer errors:
- Previous error state was not cleared before new transfers
- `i2cBusy()` did not properly report error status after completion
- `i2cRead()`/`i2cWrite()` always returned success even when errors occurred

This caused I2C sensor drivers to receive stale or incorrect data without knowing a transfer failed.

## Solution

- Clear `intr_error_stat` before starting new transfers
- Properly propagate error status through `i2cBusy()`
- Check `intr_error_stat` after transfer completion
- Return correct error status from `i2cRead()`/`i2cWrite()`

## Code Changes

### i2cWrite() Fix

```c
bool i2cWrite(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t data)
{
    // ADDED: Clear any previous error state
    if (device < I2CDEV_COUNT) {
        i2c_contexts[device].intr_error_stat = 0;
    }

    if (!i2cWriteBuffer(device, addr, reg, 1, &data)) {
        return false;
    }

    // FIXED: Check error status after completion
    bool error = false;
    while (i2cBusy(device, &error)) {
        // Wait until transfer is complete
    }

    return !error;  // FIXED: Return error status
}
```

### i2cRead() Fix

```c
bool i2cRead(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
    // ADDED: Clear any previous error state
    if (device < I2CDEV_COUNT) {
        i2c_contexts[device].intr_error_stat = 0;
    }

    if (!i2cReadBuffer(device, addr, reg, len, buf)) {
        return false;
    }

    // FIXED: Check error status after completion
    bool error = false;
    while (i2cBusy(device, &error)) {
        // Wait until transfer is complete
    }

    return !error;  // FIXED: Return error status
}
```

### i2cBusy() Fix

```c
bool i2cBusy(i2cDevice_e device, bool *error)
{
    // ... validation ...
    
    i2c_context_t *context = &i2c_contexts[device];
    if (context->state != I2C_STATE_IDLE) {
        if (error) {
            *error = false;  // Transfer still in progress
        }
        return true;
    }

    // FIXED: Report error status after transfer complete
    if (error) {
        *error = (context->intr_error_stat != 0);
    }

    return false;
}
```

## Files Modified

| File | Change |
|------|--------|
| `src/platform/PICO/bus_i2c_pico.c` | Fix error handling in i2cRead/i2cWrite/i2cBusy |

## Impact

All I2C sensors on PICO platform benefit from proper error detection:
- BMP580 barometer
- MMC5603 magnetometer  
- INA226 power monitor
- Any other I2C sensor

## Testing

**Test Hardware:**
- Board: MADFLIGHT FC3 (https://madflight.com/Board-FC3/)
- MCU: RP2350B (Raspberry Pi Pico 2)
- Sensors: BMP580, MMC5603, INA226

**Test Results:**
| Test | Result |
|------|--------|
| I2C error detection | ✅ Errors properly detected |
| Device detection | ✅ All sensors detect correctly |
| Read operations | ✅ Correct data returned |
| Write operations | ✅ Configuration applied |

## Commit Message

```
fix(pico): improve I2C error handling and detection

Fix I2C bus error handling on RP2040/RP2350 PICO platform:
- Clear previous error state before starting new transfer
- Properly propagate error status through i2cBusy()
- Check intr_error_stat for transfer completion errors
- Return correct error status from i2cRead/i2cWrite

Previously, I2C errors (NACK, bus errors) were not properly 
detected, causing drivers to receive stale/incorrect data.
This fix enables proper error detection for all I2C sensors.

Tested on RP2350B (Raspberry Pi Pico 2).
```

## PR Checklist

- [x] Code follows Betaflight coding style guidelines
- [x] Single fix: PICO I2C error handling
- [x] Build passes for PICO targets
- [x] Tested on real hardware
- [x] All I2C sensors working correctly
