# MMC5603 Magnetometer Driver for Betaflight

## PR Summary

Add support for MEMSIC MMC5603NJ 3-axis magnetometer sensor.

## Description

This PR adds a new compass driver for the MEMSIC MMC5603NJ magnetometer, a high-performance 3-axis magnetic sensor commonly used in flight controllers. The MMC5603 offers 18-bit resolution, ±32 Gauss range, and built-in automatic SET/RESET for temperature compensation.

### Features
- Full 18-bit resolution support (262144 counts per axis)
- Continuous measurement mode at 100Hz ODR
- Automatic SET/RESET for temperature drift compensation
- High power mode for lower noise
- Standard Betaflight magnetometer interface

## Hardware Specifications

| Parameter | Value |
|-----------|-------|
| Manufacturer | MEMSIC |
| Part Number | MMC5603NJ |
| Interface | I2C |
| I2C Address | 0x30 |
| Resolution | 18-bit per axis |
| Range | ±32 Gauss |
| Sensitivity | 16,384 counts/Gauss |
| Zero Field | 131,072 counts |
| Max ODR | 255 Hz |
| Product ID | 0x10 |

## Files Added

| File | Lines | Description |
|------|-------|-------------|
| `src/main/drivers/compass/compass_mmc5603.c` | ~230 | Main driver implementation |
| `src/main/drivers/compass/compass_mmc5603.h` | ~25 | Header file with function prototypes |

## Files Modified

| File | Change |
|------|--------|
| `src/main/sensors/compass.c` | Add MMC5603 to detection list |
| `src/main/sensors/compass.h` | Add MAG_MMC5603 enum |
| `src/main/target/common_post.h` | Add `USE_MAG_MMC5603` define |
| `mk/source.mk` | Add compass_mmc5603.c to build |

## Implementation Details

### Detection (`mmc5603Detect`)
```c
bool mmc5603Detect(magDev_t *magDev)
{
    // Read Product ID register (0x39)
    // Expected value: 0x10
    // Sets up init/read callbacks on success
}
```

### Initialization (`mmc5603Init`)
```c
static bool mmc5603Init(magDev_t *magDev)
{
    // 1. Software reset
    // 2. Perform SET operation (eliminates offset)
    // 3. Configure bandwidth (2.0ms)
    // 4. Set ODR to 100Hz
    // 5. Enable continuous mode with auto SET/RESET
}
```

### Data Reading (`mmc5603Read`)
```c
static bool mmc5603Read(magDev_t *magDev, int16_t *magData)
{
    // Read 9 bytes (registers 0x00-0x08)
    // Reconstruct 18-bit values from 3 bytes each axis
    // Convert to signed (subtract 131072 zero offset)
    // Scale to 16-bit for Betaflight compatibility
}
```

### Register Map

| Register | Address | Purpose |
|----------|---------|---------|
| XOUT0-ZOUT2 | 0x00-0x08 | 18-bit magnetic data (9 bytes) |
| STATUS1 | 0x18 | Measurement status flags |
| ODR | 0x1A | Output data rate setting |
| CTRL0 | 0x1B | Control register 0 (mode, SET/RESET) |
| CTRL1 | 0x1C | Control register 1 (bandwidth, reset) |
| CTRL2 | 0x1D | Control register 2 (continuous mode) |
| PRODUCT_ID | 0x39 | Device identification (0x10) |

## Board Configuration Example

```c
// In target config.h:
#define USE_MAG
#define USE_MAG_MMC5603
#define MAG_I2C_INSTANCE I2CDEV_0
```

## Testing

Tested on:
- **Board:** MADFLIGHT FC3 (https://madflight.com/Board-FC3/)
- **MCU:** RP2350B (Raspberry Pi Pico 2) @ 150MHz
- **I2C Speed:** 100kHz with internal pull-ups

### Test Results
- [x] Device detection at 0x30
- [x] Product ID read (0x10)
- [x] Continuous mode operation
- [x] X/Y/Z magnetic field readings
- [x] Compass visible in Betaflight Configurator
- [x] Compass calibration completed successfully

## Platform Compatibility

| Platform | Status | Notes |
|----------|--------|-------|
| RP2350 (PICO2) | ✅ Tested | MADFLIGHT_FC3 board |
| RP2040 (PICO) | ⚠️ Should work | Not tested |
| STM32F4 | ⚠️ Should work | Not tested (CI build only) |
| STM32F7 | ⚠️ Should work | Not tested (CI build only) |
| STM32H7 | ⚠️ Should work | Not tested (CI build only) |

**Note:** Only RP2350B (MADFLIGHT_FC3) was tested on actual hardware. STM32 platforms should work but require community testing.

## References

- [MMC5603NJ Datasheet](https://www.memsic.com/Public/Uploads/uploadfile/files/20220119/MMC5603NJDatasheetRev.B.pdf)

---

## Commit Message Template

```
feat(driver): add MMC5603 magnetometer driver

Add support for MEMSIC MMC5603NJ 3-axis magnetometer sensor.

Features:
- 18-bit resolution (262144 counts per axis)
- Continuous measurement mode at 100Hz ODR
- Automatic SET/RESET for temperature compensation
- High power mode for lower noise

New files:
- src/main/drivers/compass/compass_mmc5603.c
- src/main/drivers/compass/compass_mmc5603.h

Tested on: MADFLIGHT FC3 (RP2350B)
```

---

## Checklist Before PR

- [ ] Code follows Betaflight coding style guidelines
- [ ] Single commit with descriptive message
- [ ] PR opened against `master` branch
- [ ] Build passes for all targets (`make pre-push`)
- [ ] Test firmware built (`make unified_zip`)
- [ ] No changes to unrelated features
