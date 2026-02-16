# BMP580/BMP581 Barometer Driver for Betaflight

## Overview

This PR adds support for the Bosch BMP580/BMP581 next-generation barometer sensor to Betaflight.

## Features

- Full I2C driver for BMP580/BMP581 barometer
- 128X pressure oversampling (lowest noise)
- 4X temperature oversampling  
- IIR filter coefficient 4
- Normal power mode continuous measurement
- Compatible with both BMP580 and BMP581 variants

## Hardware Specifications

| Parameter | Value |
|-----------|-------|
| Chip | Bosch BMP580/BMP581 |
| Interface | I2C (up to 3.4MHz) |
| I2C Address | 0x46 or 0x47 |
| Pressure Range | 300-1250 hPa |
| Relative Accuracy | ±0.06 hPa (±0.5m) |
| Temperature Range | -40 to +85°C |
| Power Consumption | ~3.9µA @ 1Hz |

## Files Modified

| File | Change |
|------|--------|
| `src/main/drivers/barometer/barometer_bmp580.c` | New BMP580 driver |
| `src/main/drivers/barometer/barometer_bmp580.h` | Driver header |
| `src/main/sensors/barometer.c` | Add BMP580 detection |
| `src/main/sensors/barometer.h` | Add BARO_BMP580 enum |
| `mk/source.mk` | Add bmp580 to build |

## Oversampling Configuration

Default configuration uses 128X pressure / 4X temperature oversampling:

| Pressure OSR | Temp OSR | Noise (Pa RMS) | Update Rate | Use Case |
|--------------|----------|----------------|-------------|----------|
| 8X | 1X | ~0.5 Pa | ~80 Hz | Fast response |
| 64X | 2X | ~0.1 Pa | ~15 Hz | Balanced |
| 128X | 4X | ~0.06 Pa | ~10 Hz | Lowest noise (default) |

## Testing

**Test Hardware:**
- Board: MADFLIGHT FC3 (https://madflight.com/Board-FC3/)
- MCU: RP2350B (Raspberry Pi Pico 2) @ 150MHz
- Barometer: BMP580 at I2C address 0x47
- I2C Speed: 100kHz

**Test Results:**
| Test | Result |
|------|--------|
| Device detection | ✅ Pass at 0x47 |
| Chip ID read | 0x50 (BMP580) |
| Temperature | Stable readings |
| Pressure | Stable readings |
| Altitude | Displayed in OSD |
| Configurator | ✅ Baro visible |

## Platform Compatibility

| Platform | Status | Notes |
|----------|--------|-------|
| RP2350 (PICO2) | ✅ Tested | MADFLIGHT_FC3 board |
| RP2040 (PICO) | ⚠️ Should work | Not tested |
| STM32F4 | ⚠️ Should work | Not tested (CI build only) |
| STM32F7 | ⚠️ Should work | Not tested (CI build only) |
| STM32H7 | ⚠️ Should work | Not tested (CI build only) |

**Note:** Only RP2350B (MADFLIGHT_FC3) was tested on actual hardware. STM32 platforms should work but require community testing.

## Commit Message

```
feat(driver): add BMP580/BMP581 barometer driver

Add support for Bosch BMP580/BMP581 next-gen barometer:
- New driver barometer_bmp580.c/h with I2C protocol
- 128X oversampling for pressure (lowest noise)
- 4X oversampling for temperature  
- IIR filter coefficient 4
- Normal power mode continuous measurement
- Compatible with BMP580 and BMP581 variants

Specifications:
- Pressure range: 300-1250 hPa
- Relative accuracy: ±0.06 hPa (±0.5m)
- Temperature range: -40 to +85°C
- I2C address: 0x46 or 0x47

Tested on RP2350B (Raspberry Pi Pico 2) at I2C address 0x47.
```

## PR Checklist

- [x] Code follows Betaflight coding style guidelines
- [x] Single feature: BMP580 barometer driver
- [x] Build passes for PICO targets
- [x] Tested on real hardware (RP2350B)
- [ ] STM32 testing (needs community verification)
