# PR: MADFLIGHT_FC3 - Add internal I2C sensor configuration

**Repository:** betaflight/config  
**Branch:** feature/madflight-fc3-sensors  
**Target:** master  

## Title
`MADFLIGHT_FC3: Add internal I2C sensor configuration`

## Description

### Summary
Update MADFLIGHT_FC3 board configuration to enable internal I2C sensors.

### Changes
- **I2C0 Configuration:** Added 100kHz clock speed and internal pull-ups for reliable sensor communication
- **BMP5xx Barometer:** Configured internal BMP580/BMP581 on I2C0 (replaces external baro on I2C1)
- **MMC5603 Magnetometer:** Configured internal compass on I2C0 (replaces external mag on I2C1)
- **INA226 Power Monitor:** Added current/voltage sensor configuration on I2C0
- **Motor Protocol:** Use default DSHOT600 (Oneshot125 commented out for analog ESC reference)
- **MSP Displayport:** Enabled OSD via PIOUART0 for digital VTX systems
- **Cleanup:** Removed unused gyro defines (LSM6DSO, LSM6DSV16X)

### Related PRs (betaflight/betaflight)
- #14925 - BMP580/BMP581 barometer driver
- #14924 - MMC560x magnetometer driver  
- #14927 - INA226 power monitor driver

### Hardware
- Board: [MADFLIGHT FC3](https://madflight.com/Board-FC3/)
- MCU: RP2350B (Raspberry Pi Pico 2)
- Internal sensors: BMP580/BMP581, MMC5603, INA226

### Testing
- Compiled locally with `make MADFLIGHT_FC3`
- Hardware testing pending sensor driver merges

---

## PR Link
Create PR: https://github.com/gintaris/betaflight_config/pull/new/feature/madflight-fc3-sensors

Set:
- **Base repository:** `betaflight/config`
- **Base:** `master`
- **Head repository:** `gintaris/betaflight_config`
- **Compare:** `feature/madflight-fc3-sensors`
