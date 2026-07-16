# CLI Parameters Reference

> **Auto-generated** — do not edit manually.
> Source: `src/main/cli/settings.c` | Generated: 2026-07-16 | Commit: `45c3501877` | Firmware: `2026.6.0` | MSP: `0.1.48`

---

## Table of Contents

- [Gyro Config](#gyro-config)
- [Dyn Notch Config](#dyn-notch-config)
- [Accelerometer Config](#accelerometer-config)
- [Compass Config](#compass-config)
- [Barometer Config](#barometer-config)
- [RX Config](#rx-config)
- [RX SPI Config](#rx-spi-config)
- [ADC Config](#adc-config)
- [PWM Config](#pwm-config)
- [Blackbox Config](#blackbox-config)
- [Motor Config](#motor-config)
- [Throttle Correction Config](#throttle-correction-config)
- [Failsafe Config](#failsafe-config)
- [Board Alignment](#board-alignment)
- [Gimbal Config](#gimbal-config)
- [Battery Profiles](#battery-profiles)
- [Battery Config](#battery-config)
- [Voltage Sensor ADC Config](#voltage-sensor-adc-config)
- [Current Sensor ADC Config](#current-sensor-adc-config)
- [Current Sensor Virtual Config](#current-sensor-virtual-config)
- [Beeper Dev Config](#beeper-dev-config)
- [Beeper Config](#beeper-config)
- [Mixer Config](#mixer-config)
- [Motor 3D Config](#motor-3d-config)
- [Servo Config](#servo-config)
- [Control Rate Profiles](#control-rate-profiles)
- [Serial Config](#serial-config)
- [IMU Config](#imu-config)
- [Arming Config](#arming-config)
- [GPS Config](#gps-config)
- [GPS Rescue](#gps-rescue)
- [GPS Lap Timer](#gps-lap-timer)
- [RC Controls Config](#rc-controls-config)
- [Althold Config](#althold-config)
- [Poshold Config](#poshold-config)
- [PID Config](#pid-config)
- [PID Profile](#pid-profile)
- [Telemetry Config](#telemetry-config)
- [LED Strip Config](#led-strip-config)
- [Transponder Config](#transponder-config)
- [SDCARD Config](#sdcard-config)
- [SDIO Config](#sdio-config)
- [OSD Config](#osd-config)
- [OSD Element Config](#osd-element-config)
- [OSD Custom Text Config](#osd-custom-text-config)
- [System Config](#system-config)
- [VTX Settings Config](#vtx-settings-config)
- [VTX Config](#vtx-config)
- [VTX Io Config](#vtx-io-config)
- [VCD Config](#vcd-config)
- [MAX7456 Config](#max7456-config)
- [Display Port MSP Config](#display-port-msp-config)
- [Display Port MAX7456 Config](#display-port-max7456-config)
- [ESC Sensor Config](#esc-sensor-config)
- [RX CC2500 SPI Config](#rx-cc2500-spi-config)
- [Status LED Config](#status-led-config)
- [Dashboard Config](#dashboard-config)
- [Camera Control Config](#camera-control-config)
- [Rangefinder Config](#rangefinder-config)
- [Opticalflow Config](#opticalflow-config)
- [Pinio Config](#pinio-config)
- [Piniobox Config](#piniobox-config)
- [USB Config](#usb-config)
- [Flash Config](#flash-config)
- [RCDEVICE Config](#rcdevice-config)
- [Gyro Device Config](#gyro-device-config)
- [I2C Config](#i2c-config)
- [CAN Config](#can-config)
- [DRONECAN Config](#dronecan-config)
- [Mco Config](#mco-config)
- [RX Spektrum SPI Config](#rx-spektrum-spi-config)
- [RX Expresslrs SPI Config](#rx-expresslrs-spi-config)
- [Scheduler Config](#scheduler-config)
- [MSP Config](#msp-config)
- [Time Config](#time-config)
- [RPM Filter Config](#rpm-filter-config)
- [Flysky Config](#flysky-config)
- [Stats Config](#stats-config)
- [Pilot Config](#pilot-config)
- [Position](#position)
- [Autopilot](#autopilot)
- [Mode Activation Config](#mode-activation-config)
- [Gimbal Track Config](#gimbal-track-config)

---

## Gyro Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `gyro_hardware_lpf` | uint8 | master | `NORMAL`, `OPTION_1`, `OPTION_2`, `EXPERIMENTAL` |  |
| `gyro_high_range` | uint8 | master | `OFF`, `ON` | `USE_GYRO_SPI_ICM20649` |
| `gyro_lpf1_type` | uint8 | master | `STAGE1`, `KILL`, `STAGE2` |  |
| `gyro_lpf1_static_hz` | uint16 | master | `0` – `LPF_MAX_HZ` |  |
| `gyro_lpf2_type` | uint8 | master | `STAGE1`, `KILL`, `STAGE2` |  |
| `gyro_lpf2_static_hz` | uint16 | master | `0` – `LPF_MAX_HZ` |  |
| `gyro_notch1_hz` | uint16 | master | `0` – `LPF_MAX_HZ` |  |
| `gyro_notch1_cutoff` | uint16 | master | `0` – `LPF_MAX_HZ` |  |
| `gyro_notch2_hz` | uint16 | master | `0` – `LPF_MAX_HZ` |  |
| `gyro_notch2_cutoff` | uint16 | master | `0` – `LPF_MAX_HZ` |  |
| `gyro_calib_duration` | uint16 | master | `50` – `3000` |  |
| `gyro_calib_noise_limit` | uint8 | master | `0` – `200` |  |
| `gyro_offset_yaw` | int16 | master | `-1000` – `1000` |  |
| `gyro_overflow_detect` | uint8 | master | `AUTO`, `PAL`, `NTSC`, `HD` | `USE_GYRO_OVERFLOW_CHECK` |
| `yaw_spin_recovery` | uint8 | master | `NULL`, `LF` | `USE_YAW_SPIN_RECOVERY` |
| `yaw_spin_threshold` | uint16 | master | `YAW_SPIN_RECOVERY_THRESHOLD_MIN` – `YAW_SPIN_RECOVERY_THRESHOLD_MAX` | `USE_YAW_SPIN_RECOVERY` |
| `gyro_enabled_bitmask` | uint8 | hardware | `0` – `(1 << GYRO_COUNT) - 1` |  |
| `gyro_1_enabled` | uint8 | hardware | bitflag |  |
| `gyro_2_enabled` | uint8 | hardware | bitflag |  |
| `gyro_3_enabled` | uint8 | hardware | bitflag |  |
| `gyro_4_enabled` | uint8 | hardware | bitflag |  |
| `gyro_5_enabled` | uint8 | hardware | bitflag |  |
| `gyro_6_enabled` | uint8 | hardware | bitflag |  |
| `gyro_7_enabled` | uint8 | hardware | bitflag |  |
| `gyro_8_enabled` | uint8 | hardware | bitflag |  |
| `gyro_lpf1_dyn_min_hz` | uint16 | master | `0` – `DYN_LPF_MAX_HZ` | `USE_DYN_LPF` |
| `gyro_lpf1_dyn_max_hz` | uint16 | master | `0` – `DYN_LPF_MAX_HZ` | `USE_DYN_LPF` |
| `gyro_lpf1_dyn_expo` | uint8 | master | `0` – `10` | `USE_DYN_LPF` |
| `gyro_filter_debug_axis` | uint8 | master | `NONE`, `AUTO`, `MAX7456`, `MSP`, `FRSKYOSD`, `FBOSD` |  |
| `simplified_gyro_filter` | uint8 | master | `OFF`, `ON` | `USE_SIMPLIFIED_TUNING` |
| `simplified_gyro_filter_multiplier` | uint8 | master | `SIMPLIFIED_TUNING_FILTERS_MIN` – `SIMPLIFIED_TUNING_MAX` | `USE_SIMPLIFIED_TUNING` |

## Dyn Notch Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `dyn_notch_count` | uint8 | master | `0` – `DYN_NOTCH_COUNT_MAX` | `USE_DYN_NOTCH_FILTER` |
| `dyn_notch_q` | uint16 | master | `1` – `1000` | `USE_DYN_NOTCH_FILTER` |
| `dyn_notch_min_hz` | uint16 | master | `20` – `250` | `USE_DYN_NOTCH_FILTER` |
| `dyn_notch_max_hz` | uint16 | master | `200` – `1000` | `USE_DYN_NOTCH_FILTER` |

## Accelerometer Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `acc_hardware` | uint8 | master | `NONE`, `CYCLETIME`, `BATTERY`, `GYRO_FILTERED`, `ACCELEROMETER`, `PIDLOOP`, `RC_INTERPOLATION`, `ANGLERATE`, `ESC_SENSOR`, `SCHEDULER`, `STACK`, `ESC_SENSOR_RPM`, `ESC_SENSOR_TMP`, `ALTITUDE`, `FFT`, `FFT_TIME`, `FFT_FREQ`, `RX_FRSKY_SPI`, `RX_SFHSS_SPI`, `GYRO_RAW`, `MULTI_GYRO_RAW`, `MULTI_GYRO_DIFF`, `MAX7456_SIGNAL`, `MAX7456_SPICLOCK`, `SBUS`, `FPORT`, `RANGEFINDER`, `RANGEFINDER_QUALITY`, `OPTICALFLOW`, `LIDAR_TF`, `ADC_INTERNAL`, `RUNAWAY_TAKEOFF`, `SDIO`, `CURRENT_SENSOR`, `USB`, `SMARTAUDIO`, `RTH`, `ITERM_RELAX`, `ACRO_TRAINER`, `RC_SMOOTHING`, `RX_SIGNAL_LOSS`, `RC_SMOOTHING_RATE`, `ANTI_GRAVITY`, `DYN_LPF`, `RX_SPEKTRUM_SPI`, `DSHOT_RPM_TELEMETRY`, `RPM_FILTER`, `D_MAX`, `AC_CORRECTION`, `AC_ERROR`, `MULTI_GYRO_SCALED`, `DSHOT_RPM_ERRORS`, `CRSF_LINK_STATISTICS_UPLINK`, `CRSF_LINK_STATISTICS_PWR`, `CRSF_LINK_STATISTICS_DOWN`, `BARO`, `AUTOPILOT_ALTITUDE`, `DYN_IDLE`, `FEEDFORWARD_LIMIT`, `FEEDFORWARD`, `BLACKBOX_OUTPUT`, `GYRO_SAMPLE`, `RX_TIMING`, `D_LPF`, `VTX_TRAMP`, `GHST`, `GHST_MSP`, `SCHEDULER_DETERMINISM`, `TIMING_ACCURACY`, `RX_EXPRESSLRS_SPI`, `RX_EXPRESSLRS_PHASELOCK`, `RX_STATE_TIME`, `GPS_RESCUE_VELOCITY`, `GPS_RESCUE_HEADING`, `GPS_RESCUE_TRACKING`, `GPS_CONNECTION`, `ATTITUDE`, `VTX_MSP`, `GPS_DOP`, `FAILSAFE`, `GYRO_CALIBRATION`, `ANGLE_MODE`, `ANGLE_TARGET`, `CURRENT_ANGLE`, `DSHOT_TELEMETRY_COUNTS`, `RPM_LIMIT`, `RC_STATS`, `MAG_CALIB`, `MAG_TASK_RATE`, `EZLANDING`, `TPA`, `S_TERM`, `SPA`, `TASK`, `GIMBAL`, `WING_SETPOINT`, `CHIRP`, `FLASH_TEST_PRBS`, `MAVLINK_TELEMETRY`, `AUTOPILOT_PID`, `AUTOPILOT_STOP` | `USE_ACC` |
| `acc_high_range` | uint8 | master | `OFF`, `ON` | `USE_ACC`, `USE_GYRO_SPI_ICM20649` |
| `acc_lpf_hz` | uint16 | master | `0` – `500` | `USE_ACC` |
| `acc_trim_pitch` | int16 | master | `-300` – `300` | `USE_ACC` |
| `acc_trim_roll` | int16 | master | `-300` – `300` | `USE_ACC` |
| `acc_calibration` | int16 | master | array\[4\] | `USE_ACC` |

## Compass Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `align_mag` | uint8 | hardware | `DEFAULT`, `CW0`, `CW90`, `CW180`, `CW270`, `CW0FLIP`, `CW90FLIP`, `CW180FLIP`, `CW270FLIP`, `CUSTOM` | `USE_MAG` |
| `mag_align_roll` | int16 | hardware | `-3600` – `3600` | `USE_MAG` |
| `mag_align_pitch` | int16 | hardware | `-3600` – `3600` | `USE_MAG` |
| `mag_align_yaw` | int16 | hardware | `-3600` – `3600` | `USE_MAG` |
| `mag_bustype` | uint8 | hardware | `AUTO`, `GPS_ONLY`, `OPTICALFLOW_ONLY` | `USE_MAG` |
| `mag_i2c_device` | uint8 | hardware | `0` – `I2CDEV_COUNT` | `USE_MAG` |
| `mag_i2c_address` | uint8 | hardware | `0` – `I2C_ADDR7_MAX` | `USE_MAG` |
| `mag_spi_device` | uint8 | hardware | `0` – `SPIDEV_COUNT` | `USE_MAG` |
| `mag_hardware` | uint8 | master | `OFF`, `ON`, `FORCE` | `USE_MAG` |
| `mag_calibration` | int16 | master | array\[XYZ_AXIS_COUNT\] | `USE_MAG` |

## Barometer Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `baro_bustype` | uint8 | hardware | `AUTO`, `GPS_ONLY`, `OPTICALFLOW_ONLY` | `USE_BARO` |
| `baro_spi_device` | uint8 | hardware | `0` – `5` | `USE_BARO` |
| `baro_i2c_device` | uint8 | hardware | `0` – `5` | `USE_BARO` |
| `baro_i2c_address` | uint8 | hardware | `0` – `I2C_ADDR7_MAX` | `USE_BARO` |
| `baro_hardware` | uint8 | master | `PWM`, `ONESHOT125`, `ONESHOT42`, `MULTISHOT`, `BRUSHED`, `DSHOT150`, `DSHOT300`, `DSHOT600`, `PROSHOT1000`, `DISABLED` | `USE_BARO` |

## RX Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `mid_rc` | uint16 | master | `1200` – `1700` |  |
| `min_check` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |  |
| `max_check` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |  |
| `rssi_channel` | int8 | master | `0` – `MAX_SUPPORTED_RC_CHANNEL_COUNT` |  |
| `rssi_src_frame_errors` | int8 | master | `OFF`, `ON` |  |
| `rssi_scale` | uint8 | master | `RSSI_SCALE_MIN` – `RSSI_SCALE_MAX` |  |
| `rssi_offset` | int8 | master | `-100` – `100` |  |
| `rssi_invert` | int8 | master | `OFF`, `ON` |  |
| `rssi_src_frame_lpf_period` | uint8 | master | `0` – `UINT8_MAX` |  |
| `rssi_smoothing` | uint8 | master | `0` – `UINT8_MAX` |  |
| `rc_smoothing` | uint8 | master | `OFF`, `ON` | `USE_RC_SMOOTHING_FILTER` |
| `rc_smoothing_auto_factor` | uint8 | master | `RC_SMOOTHING_AUTO_FACTOR_MIN` – `RC_SMOOTHING_AUTO_FACTOR_MAX` | `USE_RC_SMOOTHING_FILTER` |
| `rc_smoothing_auto_factor_throttle` | uint8 | master | `RC_SMOOTHING_AUTO_FACTOR_MIN` – `RC_SMOOTHING_AUTO_FACTOR_MAX` | `USE_RC_SMOOTHING_FILTER` |
| `rc_smoothing_setpoint_cutoff` | uint8 | master | `0` – `UINT8_MAX` | `USE_RC_SMOOTHING_FILTER` |
| `rc_smoothing_throttle_cutoff` | uint8 | master | `0` – `UINT8_MAX` | `USE_RC_SMOOTHING_FILTER` |
| `rc_smoothing_debug_axis` | uint8 | master | `OFF`, `I_FREEZE`, `I`, `PID`, `PD_I_FREEZE` | `USE_RC_SMOOTHING_FILTER` |
| `fpv_mix_degrees` | uint8 | master | `0` – `90` |  |
| `max_aux_channels` | uint8 | master | `0` – `MAX_AUX_CHANNEL_COUNT` |  |
| `serialrx_provider` | uint8 | master | `NONE`, `SPEK2048`, `SBUS`, `SUMD`, `SUMH`, `XB-B`, `XB-B-RJ01`, `IBUS`, `JETIEXBUS`, `CRSF`, `SRXL`, `CUSTOM`, `FPORT`, `SRXL2`, `GHST`, `SPEK1024`, `MAVLINK` | `USE_SERIALRX` |
| `serialrx_inverted` | uint8 | master | `OFF`, `ON` | `USE_SERIALRX` |
| `spektrum_sat_bind` | uint8 | master | `SPEKTRUM_SAT_BIND_DISABLED` – `SPEKTRUM_SAT_BIND_MAX` | `USE_SPEKTRUM_BIND` |
| `spektrum_sat_bind_autoreset` | uint8 | master | `OFF`, `ON` | `USE_SPEKTRUM_BIND` |
| `srxl2_unit_id` | uint8 | master | `0` – `0xf` | `USE_SERIALRX_SRXL2` |
| `srxl2_baud_fast` | uint8 | master | `OFF`, `ON` | `USE_SERIALRX_SRXL2` |
| `sbus_baud_fast` | uint8 | master | `OFF`, `ON` | `USE_SERIALRX_SBUS` |
| `crsf_use_negotiated_baud` | uint8 | master | `OFF`, `ON` | `USE_CRSF_V3` |
| `airmode_start_throttle_percent` | uint8 | master | `0` – `100` |  |
| `rx_min_usec` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |  |
| `rx_max_usec` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |  |
| `serialrx_halfduplex` | uint8 | master | `OFF`, `ON` |  |
| `msp_override_channels_mask` | uint32 | master | `0` – `(1 << MAX_SUPPORTED_RC_CHANNEL_COUNT) - 1` | `USE_RX_MSP_OVERRIDE` |
| `msp_override_failsafe` | uint8 | hardware | `OFF`, `ON` | `USE_RX_MSP_OVERRIDE` |

## RX SPI Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `rx_spi_protocol` | uint8 | hardware | `V202_250K`, `V202_1M`, `SYMA_X`, `SYMA_X5C`, `CX10`, `CX10A`, `H8_3D`, `INAV`, `FRSKY_D`, `FRSKY_X`, `FLYSKY`, `FLYSKY_2A`, `KN`, `SFHSS`, `SPEKTRUM`, `FRSKY_X_LBT`, `REDPINE`, `FRSKY_X_V2`, `FRSKY_X_LBT_V2`, `EXPRESSLRS` | `USE_RX_SPI` |
| `rx_spi_bus` | uint8 | hardware | `0` – `SPIDEV_COUNT` | `USE_RX_SPI` |
| `rx_spi_led_inversion` | uint8 | hardware | `OFF`, `ON` | `USE_RX_SPI` |

## ADC Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `adc_device` | int8 | hardware | `0` – `ADCDEV_COUNT` | `USE_ADC` |
| `adc_vrefint_calibration` | uint16 | master | `0` – `2000` | `USE_ADC` |
| `adc_tempsensor_calibration30` | uint16 | master | `0` – `2000` | `USE_ADC` |
| `adc_tempsensor_calibration110` | uint16 | master | `0` – `2000` | `USE_ADC` |

## PWM Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `input_filtering_mode` | int8 | master | `OFF`, `ON` | `USE_RX_PWM` |

## Blackbox Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `blackbox_sample_rate` | uint8 | master | `1/1`, `1/2`, `1/4`, `1/8`, `1/16` | `USE_BLACKBOX` |
| `blackbox_device` | uint8 | hardware | `NONE`, `SPIFLASH`, `SDCARD`, `SERIAL`, `VIRTUAL` | `USE_BLACKBOX` |
| `blackbox_disable_pids` | uint32 | master | bitflag | `USE_BLACKBOX` |
| `blackbox_disable_rc` | uint32 | master | bitflag | `USE_BLACKBOX` |
| `blackbox_disable_setpoint` | uint32 | master | bitflag | `USE_BLACKBOX` |
| `blackbox_disable_bat` | uint32 | master | bitflag | `USE_BLACKBOX` |
| `blackbox_disable_mag` | uint32 | master | bitflag | `USE_BLACKBOX`, `USE_MAG` |
| `blackbox_disable_alt` | uint32 | master | bitflag | `USE_BLACKBOX`, `USE_BARO`, `USE_RANGEFINDER` |
| `blackbox_disable_rssi` | uint32 | master | bitflag | `USE_BLACKBOX` |
| `blackbox_disable_gyro` | uint32 | master | bitflag | `USE_BLACKBOX` |
| `blackbox_disable_gyrounfilt` | uint32 | master | bitflag | `USE_BLACKBOX` |
| `blackbox_disable_acc` | uint32 | master | bitflag | `USE_BLACKBOX`, `USE_ACC` |
| `blackbox_disable_attitude` | uint32 | master | bitflag | `USE_BLACKBOX`, `USE_ACC` |
| `blackbox_disable_debug` | uint32 | master | bitflag | `USE_BLACKBOX` |
| `blackbox_disable_motors` | uint32 | master | bitflag | `USE_BLACKBOX` |
| `blackbox_disable_servos` | uint32 | master | bitflag | `USE_BLACKBOX`, `USE_SERVOS` |
| `blackbox_disable_rpm` | uint32 | master | bitflag | `USE_BLACKBOX`, `USE_DSHOT_TELEMETRY` |
| `blackbox_disable_gps` | uint32 | master | bitflag | `USE_BLACKBOX`, `USE_GPS` |
| `blackbox_mode` | uint8 | master | `NORMAL`, `MOTOR_TEST`, `ALWAYS` | `USE_BLACKBOX` |
| `blackbox_high_resolution` | uint8 | master | `OFF`, `ON` | `USE_BLACKBOX` |

## Motor Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `max_throttle` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |  |
| `min_command` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |  |
| `motor_kv` | uint16 | hardware | `1` – `40000` |  |
| `motor_idle` | uint16 | master | `0` – `2000` |  |
| `dshot_burst` | uint8 | hardware | `NULL`, `LF` | `USE_DSHOT`, `USE_DSHOT_DMAR` |
| `dshot_bidir` | uint8 | master | `OFF`, `ON` | `USE_DSHOT`, `USE_DSHOT_TELEMETRY` |
| `dshot_edt` | uint8 | master | `AUTO-LAND`, `DROP`, `GPS-RESCUE` | `USE_DSHOT`, `USE_DSHOT_TELEMETRY` |
| `dshot_bitbang` | uint8 | hardware | `NULL`, `LF` | `USE_DSHOT`, `USE_DSHOT_BITBANG` |
| `dshot_bitbang_timer` | uint8 | hardware | `OFF`, `RP`, `RPY` | `USE_DSHOT`, `USE_DSHOT_BITBANG` |
| `use_unsynced_pwm` | uint8 | master | `OFF`, `ON` |  |
| `motor_pwm_protocol` | uint8 | master | `PT1`, `BIQUAD`, `PT2`, `PT3` |  |
| `motor_pwm_rate` | uint16 | master | `200` – `32000` |  |
| `motor_pwm_inversion` | uint8 | master | `OFF`, `ON` |  |
| `motor_poles` | uint8 | master | `4` – `UINT8_MAX` |  |
| `motor_output_reordering` | uint8 | master | array\[MAX_SUPPORTED_MOTORS\] |  |

## Throttle Correction Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `thr_corr_value` | uint8 | master | `0` – `150` |
| `thr_corr_angle` | uint16 | master | `1` – `900` |

## Failsafe Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `failsafe_delay` | uint8 | master | `PERIOD_RXDATA_RECOVERY / MILLIS_PER_TENTH_SECOND` – `200` |
| `failsafe_landing_time` | uint8 | master | `0` – `250` |
| `failsafe_throttle` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |
| `failsafe_switch_mode` | uint8 | master | `NONE`, `I2C`, `SPI`, `SLAVE`, `GYROAUTO` |
| `failsafe_throttle_low_delay` | uint16 | master | `0` – `300` |
| `failsafe_procedure` | uint8 | master | `HARDWARE_PWM`, `SOFTWARE_PWM`, `DAC` |
| `failsafe_recovery_delay` | uint16 | master | `1` – `200` |
| `failsafe_stick_threshold` | uint8 | master | `0` – `50` |

## Board Alignment

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `align_board_roll` | int16 | master | `-180` – `360` |
| `align_board_pitch` | int16 | master | `-180` – `360` |
| `align_board_yaw` | int16 | master | `-180` – `360` |

## Gimbal Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `gimbal_mode` | uint8 | master | `NORMAL`, `MIXTILT` | `USE_SERVOS` |

## Battery Profiles

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `battery_profile_name` | uint8 | battery | string\[1-MAX_BATTERY_PROFILE_NAME_LENGTH\] | `USE_PROFILE_NAMES` |
| `bat_capacity` | uint16 | battery | `0` – `20000` |  |
| `vbat_max_cell_voltage` | uint16 | battery | `VBAT_CELL_VOTAGE_RANGE_MIN` – `VBAT_CELL_VOTAGE_RANGE_MAX` |  |
| `vbat_full_cell_voltage` | uint16 | battery | `VBAT_CELL_VOTAGE_RANGE_MIN` – `VBAT_CELL_VOTAGE_RANGE_MAX` |  |
| `vbat_min_cell_voltage` | uint16 | battery | `VBAT_CELL_VOTAGE_RANGE_MIN` – `VBAT_CELL_VOTAGE_RANGE_MAX` |  |
| `vbat_warning_cell_voltage` | uint16 | battery | `VBAT_CELL_VOTAGE_RANGE_MIN` – `VBAT_CELL_VOTAGE_RANGE_MAX` |  |
| `cbat_alert_percent` | uint8 | battery | `0` – `100` |  |
| `force_battery_cell_count` | uint8 | battery | `0` – `24` |  |

## Battery Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `vbat_hysteresis` | uint8 | master | `0` – `250` |  |
| `current_meter` | uint8 | hardware | `NONE`, `ADC`, `VIRTUAL`, `ESC`, `MSP` |  |
| `battery_meter` | uint8 | hardware | `NONE`, `ADC`, `ESC` |  |
| `vbat_detect_cell_voltage` | uint16 | master | `0` – `2000` |  |
| `use_vbat_alerts` | uint8 | master | `OFF`, `ON` |  |
| `use_cbat_alerts` | uint8 | master | `OFF`, `ON` |  |
| `vbat_cutoff_percent` | uint8 | master | `0` – `100` |  |
| `vbat_display_lpf_period` | uint8 | master | `1` – `UINT8_MAX` |  |
| `vbat_sag_lpf_period` | uint8 | master | `1` – `UINT8_MAX` | `USE_BATTERY_VOLTAGE_SAG_COMPENSATION` |
| `ibat_lpf_period` | uint8 | master | `0` – `UINT8_MAX` |  |
| `vbat_duration_for_warning` | uint8 | master | `0` – `150` |  |
| `vbat_duration_for_critical` | uint8 | master | `0` – `150` |  |
| `battery_continue` | uint8 | master | `OFF`, `ON` | `USE_BATTERY_CONTINUE` |

## Voltage Sensor ADC Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `vbat_scale` | uint8 | hardware | `VBAT_SCALE_MIN` – `VBAT_SCALE_MAX` |
| `vbat_divider` | uint8 | master | `VBAT_DIVIDER_MIN` – `VBAT_DIVIDER_MAX` |
| `vbat_multiplier` | uint8 | master | `VBAT_MULTIPLIER_MIN` – `VBAT_MULTIPLIER_MAX` |

## Current Sensor ADC Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `ibata_scale` | int16 | hardware | `-16000` – `16000` |
| `ibata_offset` | int16 | master | `-32000` – `32000` |

## Current Sensor Virtual Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `ibatv_scale` | int16 | master | `-16000` – `16000` | `USE_VIRTUAL_CURRENT_METER` |
| `ibatv_offset` | uint16 | master | `0` – `16000` | `USE_VIRTUAL_CURRENT_METER` |

## Beeper Dev Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `beeper_inversion` | uint8 | hardware | `OFF`, `ON` | `USE_BEEPER` |
| `beeper_od` | uint8 | hardware | `OFF`, `ON` | `USE_BEEPER` |
| `beeper_frequency` | int16 | hardware | `0` – `16000` | `USE_BEEPER` |

## Beeper Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `beeper_dshot_beacon_tone` | uint8 | master | `1` – `DSHOT_CMD_BEACON5` | `USE_BEEPER`, `USE_DSHOT` |

## Mixer Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `yaw_motors_reversed` | int8 | master | `OFF`, `ON` |  |
| `mixer_type` | uint8 | master | `BASIC`, `ADVANCED` |  |
| `crashflip_motor_percent` | uint8 | master | `0` – `100` |  |
| `crashflip_rate` | uint8 | master | `0` – `250` |  |
| `crashflip_auto_rearm` | int8 | master | `OFF`, `ON` |  |
| `rpm_limit` | int8 | master | `OFF`, `ON` | `USE_RPM_LIMIT` |
| `rpm_limit_p` | uint16 | master | `0` – `100` | `USE_RPM_LIMIT` |
| `rpm_limit_i` | uint16 | master | `0` – `1000` | `USE_RPM_LIMIT` |
| `rpm_limit_d` | uint16 | master | `0` – `100` | `USE_RPM_LIMIT` |
| `rpm_limit_value` | uint16 | master | `1` – `UINT16_MAX` | `USE_RPM_LIMIT` |

## Motor 3D Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `3d_deadband_low` | uint16 | master | `PWM_PULSE_MIN` – `PWM_RANGE_MIDDLE` |
| `3d_deadband_high` | uint16 | master | `PWM_RANGE_MIDDLE` – `PWM_PULSE_MAX` |
| `3d_neutral` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` |
| `3d_deadband_throttle` | uint16 | master | `1` – `100` |
| `3d_limit_low` | uint16 | master | `PWM_PULSE_MIN` – `PWM_RANGE_MIDDLE` |
| `3d_limit_high` | uint16 | master | `PWM_RANGE_MIDDLE` – `PWM_PULSE_MAX` |
| `3d_switched_mode` | uint8 | master | `OFF`, `ON` |

## Servo Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `servo_center_pulse` | uint16 | master | `PWM_PULSE_MIN` – `PWM_PULSE_MAX` | `USE_SERVOS` |
| `servo_pwm_rate` | uint16 | master | `50` – `498` | `USE_SERVOS` |
| `servo_lowpass_hz` | uint16 | master | `0` – `400` | `USE_SERVOS` |
| `tri_unarmed_servo` | int8 | master | `OFF`, `ON` | `USE_SERVOS` |
| `channel_forwarding_start` | uint8 | master | `AUX1` – `MAX_SUPPORTED_RC_CHANNEL_COUNT` | `USE_SERVOS` |

## Control Rate Profiles

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `rateprofile_name` | uint8 | rate | string\[1-MAX_RATE_PROFILE_NAME_LENGTH\] | `USE_PROFILE_NAMES` |
| `thr_mid` | uint8 | rate | `0` – `100` |  |
| `thr_expo` | uint8 | rate | `0` – `100` |  |
| `thr_hover` | uint8 | rate | `0` – `100` |  |
| `rates_type` | uint8 | rate | `OFF`, `RP`, `RPY`, `RP_INC`, `RPY_INC` |  |
| `quickrates_rc_expo` | uint8 | rate | `OFF`, `ON` |  |
| `roll_rc_rate` | uint8 | rate | `1` – `CONTROL_RATE_CONFIG_RC_RATES_MAX` |  |
| `pitch_rc_rate` | uint8 | rate | `1` – `CONTROL_RATE_CONFIG_RC_RATES_MAX` |  |
| `yaw_rc_rate` | uint8 | rate | `1` – `CONTROL_RATE_CONFIG_RC_RATES_MAX` |  |
| `roll_expo` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_RC_EXPO_MAX` |  |
| `pitch_expo` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_RC_EXPO_MAX` |  |
| `yaw_expo` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_RC_EXPO_MAX` |  |
| `roll_srate` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_RATE_MAX` |  |
| `pitch_srate` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_RATE_MAX` |  |
| `yaw_srate` | uint8 | rate | `0` – `CONTROL_RATE_CONFIG_RATE_MAX` |  |
| `throttle_limit_type` | uint8 | rate | `ROLL`, `PITCH`, `YAW`, `THROTTLE` |  |
| `throttle_limit_percent` | uint8 | rate | `25` – `100` |  |
| `roll_rate_limit` | uint16 | rate | `CONTROL_RATE_CONFIG_RATE_LIMIT_MIN` – `CONTROL_RATE_CONFIG_RATE_LIMIT_MAX` |  |
| `pitch_rate_limit` | uint16 | rate | `CONTROL_RATE_CONFIG_RATE_LIMIT_MIN` – `CONTROL_RATE_CONFIG_RATE_LIMIT_MAX` |  |
| `yaw_rate_limit` | uint16 | rate | `CONTROL_RATE_CONFIG_RATE_LIMIT_MIN` – `CONTROL_RATE_CONFIG_RATE_LIMIT_MAX` |  |

## Serial Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `reboot_character` | uint8 | master | `48` – `126` |
| `serial_update_rate_hz` | uint16 | master | `100` – `2000` |

## IMU Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `imu_dcm_kp` | uint16 | master | `0` – `32000` |  |
| `imu_dcm_ki` | uint16 | master | `0` – `32000` |  |
| `small_angle` | uint8 | master | `0` – `180` |  |
| `imu_process_denom` | uint8 | master | `1` – `4` |  |
| `mag_declination` | int16 | master | `-300` – `300` | `USE_MAG` |
| `trust_mag` | uint8 | master | `OFF`, `ON` | `USE_MAG` |

## Arming Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `auto_disarm_delay` | uint8 | master | `0` – `60` |
| `gyro_cal_on_first_arm` | uint8 | master | `OFF`, `ON` |
| `prearm_allow_rearm` | uint8 | master | `OFF`, `ON` |

## GPS Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `gps_provider` | uint8 | master | `NMEA`, `UBLOX`, `MSP`, `VIRTUAL`, `DRONECAN` | `USE_GPS` |
| `gps_sbas_mode` | uint8 | master | `AUTO`, `EGNOS`, `WAAS`, `MSAS`, `GAGAN`, `NONE` | `USE_GPS` |
| `gps_auto_config` | uint8 | master | `OFF`, `ON` | `USE_GPS` |
| `gps_auto_baud` | uint8 | master | `OFF`, `ON` | `USE_GPS` |
| `gps_ublox_acquire_model` | uint8 | master | `PORTABLE`, `STATIONARY`, `PEDESTRIAN`, `AUTOMOTIVE`, `AT_SEA`, `AIRBORNE_1G`, `AIRBORNE_2G`, `AIRBORNE_4G` | `USE_GPS` |
| `gps_ublox_flight_model` | uint8 | master | `PORTABLE`, `STATIONARY`, `PEDESTRIAN`, `AUTOMOTIVE`, `AT_SEA`, `AIRBORNE_1G`, `AIRBORNE_2G`, `AIRBORNE_4G` | `USE_GPS` |
| `gps_update_rate_hz` | uint8 | master | `1` – `20` | `USE_GPS` |
| `gps_ublox_utc_standard` | uint8 | master | `AUTO`, `USNO`, `EU`, `SU`, `NTSC` | `USE_GPS` |
| `gps_ublox_use_galileo` | uint8 | master | `OFF`, `ON` | `USE_GPS` |
| `gps_ublox_enable_ana` | uint8 | master | `OFF`, `ON` | `USE_GPS` |
| `gps_set_home_point_once` | uint8 | master | `OFF`, `ON` | `USE_GPS` |
| `gps_use_3d_speed` | uint8 | master | `OFF`, `ON` | `USE_GPS` |
| `gps_sbas_integrity` | uint8 | master | `OFF`, `ON` | `USE_GPS` |
| `gps_nmea_custom_commands` | uint8 | master | string\[1-NMEA_CUSTOM_COMMANDS_MAX_LENGTH\] | `USE_GPS` |

## GPS Rescue

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `gps_rescue_min_start_dist` | uint16 | master | `5` – `30` | `USE_GPS`, `USE_GPS_RESCUE`, `!USE_WING` |
| `gps_rescue_alt_mode` | uint8 | master | `MAX_ALT`, `FIXED_ALT`, `CURRENT_ALT` | `USE_GPS`, `USE_GPS_RESCUE`, `!USE_WING` |
| `gps_rescue_initial_climb` | uint16 | master | `0` – `100` | `USE_GPS`, `USE_GPS_RESCUE`, `!USE_WING` |
| `gps_rescue_ascend_rate` | uint16 | master | `50` – `2500` | `USE_GPS`, `USE_GPS_RESCUE`, `!USE_WING` |
| `gps_rescue_return_alt` | uint16 | master | `5` – `1000` | `USE_GPS`, `USE_GPS_RESCUE`, `!USE_WING` |
| `gps_rescue_ground_speed` | uint16 | master | `0` – `3000` | `USE_GPS`, `USE_GPS_RESCUE`, `!USE_WING` |
| `gps_rescue_descent_dist` | uint16 | master | `5` – `500` | `USE_GPS`, `USE_GPS_RESCUE`, `!USE_WING` |
| `gps_rescue_descend_rate` | uint16 | master | `25` – `500` | `USE_GPS`, `USE_GPS_RESCUE`, `!USE_WING` |
| `gps_rescue_sanity_checks` | uint8 | master | `RESCUE_SANITY_OFF`, `RESCUE_SANITY_ON`, `RESCUE_SANITY_FS_ONLY` | `USE_GPS`, `USE_GPS_RESCUE`, `!USE_WING` |
| `gps_rescue_min_sats` | uint8 | master | `5` – `50` | `USE_GPS`, `USE_GPS_RESCUE`, `!USE_WING` |
| `gps_rescue_allow_arming_without_fix` | uint8 | master | `OFF`, `ON` | `USE_GPS`, `USE_GPS_RESCUE`, `!USE_WING` |
| `gps_rescue_yaw_p` | uint8 | master | `0` – `200` | `USE_GPS`, `USE_GPS_RESCUE`, `!USE_WING` |

## GPS Lap Timer

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `gps_lap_timer_gate_lat` | int32 | master | `-900000000` – `900000000` | `USE_GPS`, `USE_GPS_LAP_TIMER` |
| `gps_lap_timer_gate_lon` | int32 | master | `-1800000000` – `1800000000` | `USE_GPS`, `USE_GPS_LAP_TIMER` |
| `gps_lap_timer_min_lap_time_s` | uint16 | master | `0` – `3000` | `USE_GPS`, `USE_GPS_LAP_TIMER` |
| `gps_lap_timer_gate_tolerance_m` | uint8 | master | `1` – `100` | `USE_GPS`, `USE_GPS_LAP_TIMER` |

## RC Controls Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `deadband` | uint8 | master | `0` – `32` |
| `yaw_deadband` | uint8 | master | `0` – `100` |
| `yaw_control_reversed` | int8 | master | `OFF`, `ON` |

## Althold Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `alt_hold_climb_rate` | uint8 | master | `0` – `200` | `USE_ALTITUDE_HOLD`, `!USE_WING` |
| `alt_hold_deadband` | uint8 | master | `0` – `70` | `USE_ALTITUDE_HOLD`, `!USE_WING` |

## Poshold Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `pos_hold_deadband` | uint8 | master | `0` – `50` | `USE_POSITION_HOLD`, `!USE_WING` |
| `poshold_position_source` | uint8 | master | `OFF`, `SCALE`, `CLIP` | `USE_POSITION_HOLD`, `!USE_WING` |
| `poshold_opticalflow_quality_min` | uint8 | master | `0` – `100` | `USE_POSITION_HOLD`, `!USE_WING` |
| `poshold_opticalflow_max_range` | uint16 | master | `50` – `1000` | `USE_POSITION_HOLD`, `!USE_WING` |

## PID Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `pid_process_denom` | uint8 | master | `1` – `MAX_PID_PROCESS_DENOM` |  |
| `runaway_takeoff_prevention` | uint8 | master | `OFF`, `ON` | `USE_RUNAWAY_TAKEOFF` |
| `runaway_takeoff_deactivate_delay` | uint16 | master | `100` – `1000` | `USE_RUNAWAY_TAKEOFF` |
| `runaway_takeoff_deactivate_throttle_percent` | uint8 | master | `0` – `100` | `USE_RUNAWAY_TAKEOFF` |

## PID Profile

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `profile_name` | uint8 | profile | string\[1-MAX_PROFILE_NAME_LENGTH\] | `USE_PROFILE_NAMES` |
| `dterm_lpf1_dyn_min_hz` | uint16 | profile | `0` – `DYN_LPF_MAX_HZ` | `USE_DYN_LPF` |
| `dterm_lpf1_dyn_max_hz` | uint16 | profile | `0` – `DYN_LPF_MAX_HZ` | `USE_DYN_LPF` |
| `dterm_lpf1_dyn_expo` | uint8 | profile | `0` – `10` | `USE_DYN_LPF` |
| `dterm_lpf1_type` | uint8 | profile | `OFF`, `ON`, `BEEP`, `DISARM` |  |
| `dterm_lpf1_static_hz` | int16 | profile | `0` – `LPF_MAX_HZ` |  |
| `dterm_lpf2_type` | uint8 | profile | `OFF`, `ON`, `BEEP`, `DISARM` |  |
| `dterm_lpf2_static_hz` | int16 | profile | `0` – `LPF_MAX_HZ` |  |
| `dterm_notch_hz` | uint16 | profile | `0` – `LPF_MAX_HZ` |  |
| `dterm_notch_cutoff` | uint16 | profile | `0` – `LPF_MAX_HZ` |  |
| `vbat_sag_compensation` | uint8 | profile | `0` – `150` | `USE_BATTERY_VOLTAGE_SAG_COMPENSATION` |
| `pid_at_min_throttle` | uint8 | profile | `OFF`, `ON` |  |
| `anti_gravity_gain` | uint8 | profile | `ITERM_ACCELERATOR_GAIN_OFF` – `ITERM_ACCELERATOR_GAIN_MAX` |  |
| `anti_gravity_cutoff_hz` | uint8 | profile | `2` – `50` |  |
| `anti_gravity_p_gain` | uint8 | profile | `0` – `250` |  |
| `acc_limit_yaw` | uint16 | profile | `0` – `500` |  |
| `acc_limit` | uint16 | profile | `0` – `500` |  |
| `crash_dthreshold` | uint16 | profile | `10` – `2000` |  |
| `crash_gthreshold` | uint16 | profile | `100` – `2000` |  |
| `crash_setpoint_threshold` | uint16 | profile | `50` – `2000` |  |
| `crash_time` | uint16 | profile | `100` – `5000` |  |
| `crash_delay` | uint16 | profile | `0` – `500` |  |
| `crash_recovery_angle` | uint8 | profile | `5` – `30` |  |
| `crash_recovery_rate` | uint8 | profile | `50` – `255` |  |
| `crash_limit_yaw` | uint16 | profile | `0` – `1000` |  |
| `crash_recovery` | uint8 | profile | `HALF`, `NOMINAL`, `DOUBLE` |  |
| `iterm_rotation` | uint8 | profile | `OFF`, `ON` |  |
| `iterm_relax` | uint8 | profile | `OFF`, `SPI`, `SDIO` | `USE_ITERM_RELAX` |
| `iterm_relax_type` | uint8 | profile | `NORMAL`, `PITCHONLY`, `FULL` | `USE_ITERM_RELAX` |
| `iterm_relax_cutoff` | uint8 | profile | `1` – `50` | `USE_ITERM_RELAX` |
| `iterm_windup` | uint8 | profile | `20` – `100` |  |
| `pidsum_limit` | uint16 | profile | `PIDSUM_LIMIT_MIN` – `PIDSUM_LIMIT_MAX` |  |
| `pidsum_limit_yaw` | uint16 | profile | `PIDSUM_LIMIT_MIN` – `PIDSUM_LIMIT_MAX` |  |
| `yaw_lowpass_hz` | uint16 | profile | `0` – `500` |  |
| `throttle_boost` | uint8 | profile | `0` – `100` | `USE_THROTTLE_BOOST` |
| `throttle_boost_cutoff` | uint8 | profile | `5` – `50` | `USE_THROTTLE_BOOST` |
| `acro_trainer_angle_limit` | uint8 | profile | `10` – `80` | `USE_ACRO_TRAINER` |
| `acro_trainer_lookahead_ms` | uint16 | profile | `10` – `200` | `USE_ACRO_TRAINER` |
| `acro_trainer_debug_axis` | uint8 | profile | `PD`, `D`, `PDS` | `USE_ACRO_TRAINER` |
| `acro_trainer_gain` | uint8 | profile | `25` – `255` | `USE_ACRO_TRAINER` |
| `p_pitch` | uint8 | profile | `0` – `PID_GAIN_MAX` |  |
| `i_pitch` | uint8 | profile | `0` – `PID_GAIN_MAX` |  |
| `d_pitch` | uint8 | profile | `0` – `PID_GAIN_MAX` |  |
| `f_pitch` | uint16 | profile | `0` – `F_GAIN_MAX` |  |
| `p_roll` | uint8 | profile | `0` – `PID_GAIN_MAX` |  |
| `i_roll` | uint8 | profile | `0` – `PID_GAIN_MAX` |  |
| `d_roll` | uint8 | profile | `0` – `PID_GAIN_MAX` |  |
| `f_roll` | uint16 | profile | `0` – `F_GAIN_MAX` |  |
| `p_yaw` | uint8 | profile | `0` – `PID_GAIN_MAX` |  |
| `i_yaw` | uint8 | profile | `0` – `PID_GAIN_MAX` |  |
| `d_yaw` | uint8 | profile | `0` – `PID_GAIN_MAX` |  |
| `f_yaw` | uint16 | profile | `0` – `F_GAIN_MAX` |  |
| `s_pitch` | uint8 | profile | `0` – `PID_GAIN_MAX` | `USE_WING` |
| `s_roll` | uint8 | profile | `0` – `PID_GAIN_MAX` | `USE_WING` |
| `s_yaw` | uint8 | profile | `0` – `PID_GAIN_MAX` | `USE_WING` |
| `angle_p_gain` | uint8 | profile | `0` – `200` |  |
| `angle_feedforward` | uint8 | profile | `0` – `200` |  |
| `angle_feedforward_smoothing_ms` | uint8 | profile | `10` – `250` |  |
| `angle_limit` | uint8 | profile | `10` – `80` |  |
| `angle_earth_ref` | uint8 | profile | `0` – `100` |  |
| `horizon_level_strength` | uint8 | profile | `0` – `100` |  |
| `horizon_limit_sticks` | uint8 | profile | `10` – `200` |  |
| `horizon_limit_degrees` | uint8 | profile | `10` – `250` |  |
| `horizon_ignore_sticks` | uint8 | profile | `OFF`, `ON` |  |
| `horizon_delay_ms` | uint16 | profile | `10` – `5000` |  |
| `chirp_lag_freq_hz` | uint8 | profile | `0` – `255` | `USE_CHIRP` |
| `chirp_lead_freq_hz` | uint8 | profile | `0` – `255` | `USE_CHIRP` |
| `chirp_amplitude_roll` | uint16 | profile | `0` – `500` | `USE_CHIRP` |
| `chirp_amplitude_pitch` | uint16 | profile | `0` – `500` | `USE_CHIRP` |
| `chirp_amplitude_yaw` | uint16 | profile | `0` – `500` | `USE_CHIRP` |
| `chirp_frequency_start_deci_hz` | uint16 | profile | `1` – `1000` | `USE_CHIRP` |
| `chirp_frequency_end_deci_hz` | uint16 | profile | `1` – `10000` | `USE_CHIRP` |
| `chirp_time_seconds` | uint8 | profile | `1` – `255` | `USE_CHIRP` |
| `use_integrated_yaw` | uint8 | profile | `OFF`, `ON` | `USE_INTEGRATED_YAW_CONTROL` |
| `integrated_yaw_relax` | uint8 | profile | `0` – `255` | `USE_INTEGRATED_YAW_CONTROL` |
| `d_max_roll` | uint8 | profile | `0` – `PID_GAIN_MAX` | `USE_D_MAX` |
| `d_max_pitch` | uint8 | profile | `0` – `PID_GAIN_MAX` | `USE_D_MAX` |
| `d_max_yaw` | uint8 | profile | `0` – `PID_GAIN_MAX` | `USE_D_MAX` |
| `d_max_gain` | uint8 | profile | `0` – `100` | `USE_D_MAX` |
| `d_max_advance` | uint8 | profile | `0` – `200` | `USE_D_MAX` |
| `motor_output_limit` | uint8 | profile | `MOTOR_OUTPUT_LIMIT_PERCENT_MIN` – `MOTOR_OUTPUT_LIMIT_PERCENT_MAX` |  |
| `auto_profile_cell_count` | int8 | profile | `AUTO_PROFILE_CELL_COUNT_CHANGE` – `MAX_AUTO_DETECT_CELL_COUNT` |  |
| `launch_control_mode` | uint8 | profile | `ROLL`, `PITCH`, `YAW` | `USE_LAUNCH_CONTROL` |
| `launch_trigger_allow_reset` | uint8 | profile | `OFF`, `ON` | `USE_LAUNCH_CONTROL` |
| `launch_trigger_throttle_percent` | uint8 | profile | `0` – `LAUNCH_CONTROL_THROTTLE_TRIGGER_MAX` | `USE_LAUNCH_CONTROL` |
| `launch_angle_limit` | uint8 | profile | `0` – `80` | `USE_LAUNCH_CONTROL` |
| `launch_control_gain` | uint8 | profile | `0` – `200` | `USE_LAUNCH_CONTROL` |
| `thrust_linear` | uint8 | profile | `0` – `150` | `USE_THRUST_LINEARIZATION` |
| `feedforward_transition` | uint8 | profile | `0` – `100` | `USE_FEEDFORWARD` |
| `feedforward_averaging` | uint8 | profile | `LEGACY`, `LINEAR`, `DYNAMIC`, `EZLANDING` | `USE_FEEDFORWARD` |
| `feedforward_smooth_factor` | uint8 | profile | `0` – `95` | `USE_FEEDFORWARD` |
| `feedforward_jitter_factor` | uint8 | profile | `0` – `20` | `USE_FEEDFORWARD` |
| `feedforward_boost` | uint8 | profile | `0` – `50` | `USE_FEEDFORWARD` |
| `feedforward_max_rate_limit` | uint8 | profile | `0` – `200` | `USE_FEEDFORWARD` |
| `feedforward_yaw_hold_gain` | uint8 | profile | `0` – `100` | `USE_FEEDFORWARD` |
| `feedforward_yaw_hold_time` | uint8 | profile | `10` – `250` | `USE_FEEDFORWARD` |
| `dyn_idle_min_rpm` | uint8 | profile | `0` – `200` | `USE_DYN_IDLE` |
| `dyn_idle_p_gain` | uint8 | profile | `1` – `250` | `USE_DYN_IDLE` |
| `dyn_idle_i_gain` | uint8 | profile | `1` – `250` | `USE_DYN_IDLE` |
| `dyn_idle_d_gain` | uint8 | profile | `0` – `250` | `USE_DYN_IDLE` |
| `dyn_idle_max_increase` | uint8 | profile | `10` – `255` | `USE_DYN_IDLE` |
| `level_race_mode` | uint8 | profile | `OFF`, `ON` |  |
| `simplified_pids_mode` | uint8 | profile | `RUDDER`, `DIFF_THRUST` | `USE_SIMPLIFIED_TUNING` |
| `simplified_master_multiplier` | uint8 | profile | `SIMPLIFIED_TUNING_PIDS_MIN` – `SIMPLIFIED_TUNING_MAX` | `USE_SIMPLIFIED_TUNING` |
| `simplified_i_gain` | uint8 | profile | `SIMPLIFIED_TUNING_PIDS_MIN` – `SIMPLIFIED_TUNING_MAX` | `USE_SIMPLIFIED_TUNING` |
| `simplified_d_gain` | uint8 | profile | `SIMPLIFIED_TUNING_PIDS_MIN` – `SIMPLIFIED_TUNING_MAX` | `USE_SIMPLIFIED_TUNING` |
| `simplified_pi_gain` | uint8 | profile | `SIMPLIFIED_TUNING_PIDS_MIN` – `SIMPLIFIED_TUNING_MAX` | `USE_SIMPLIFIED_TUNING` |
| `simplified_d_max_gain` | uint8 | profile | `0` – `SIMPLIFIED_TUNING_MAX` | `USE_SIMPLIFIED_TUNING` |
| `simplified_feedforward_gain` | uint8 | profile | `0` – `SIMPLIFIED_TUNING_MAX` | `USE_SIMPLIFIED_TUNING` |
| `simplified_pitch_d_gain` | uint8 | profile | `SIMPLIFIED_TUNING_PIDS_MIN` – `SIMPLIFIED_TUNING_MAX` | `USE_SIMPLIFIED_TUNING` |
| `simplified_pitch_pi_gain` | uint8 | profile | `SIMPLIFIED_TUNING_PIDS_MIN` – `SIMPLIFIED_TUNING_MAX` | `USE_SIMPLIFIED_TUNING` |
| `simplified_dterm_filter` | uint8 | profile | `OFF`, `ON` | `USE_SIMPLIFIED_TUNING` |
| `simplified_dterm_filter_multiplier` | uint8 | profile | `SIMPLIFIED_TUNING_FILTERS_MIN` – `SIMPLIFIED_TUNING_MAX` | `USE_SIMPLIFIED_TUNING` |
| `tpa_mode` | uint8 | profile | `DEFAULT`, `BARO_ONLY`, `GPS_ONLY`, `RANGEFINDER_PREFER`, `RANGEFINDER_ONLY` |  |
| `tpa_rate` | uint8 | profile | `0` – `TPA_MAX` |  |
| `tpa_breakpoint` | uint16 | profile | `PWM_RANGE_MIN` – `PWM_RANGE_MAX` |  |
| `tpa_low_rate` | int8 | profile | `TPA_LOW_RATE_MIN` – `TPA_MAX` |  |
| `tpa_low_breakpoint` | uint16 | profile | `PWM_RANGE_MIN` – `PWM_RANGE_MAX` |  |
| `tpa_low_always` | uint8 | profile | `OFF`, `ON` |  |
| `tpa_speed_type` | uint8 | profile | *TABLE_TPA_SPEED_TYPE* | `USE_WING` |
| `tpa_speed_basic_delay` | uint16 | profile | `1` – `UINT16_MAX` | `USE_WING` |
| `tpa_speed_basic_gravity` | uint16 | profile | `1` – `UINT16_MAX` | `USE_WING` |
| `tpa_speed_adv_prop_pitch` | uint16 | profile | `0` – `UINT16_MAX` | `USE_WING` |
| `tpa_speed_adv_mass` | uint16 | profile | `1` – `UINT16_MAX` | `USE_WING` |
| `tpa_speed_adv_drag_k` | uint16 | profile | `1` – `UINT16_MAX` | `USE_WING` |
| `tpa_speed_adv_thrust` | uint16 | profile | `1` – `UINT16_MAX` | `USE_WING` |
| `tpa_speed_max_voltage` | uint16 | profile | `0` – `UINT16_MAX` | `USE_WING` |
| `tpa_speed_pitch_offset` | int16 | profile | `INT16_MIN` – `INT16_MAX` | `USE_WING` |
| `tpa_curve_type` | uint8 | profile | *TABLE_TPA_CURVE_TYPE* | `USE_ADVANCED_TPA` |
| `tpa_curve_stall_throttle` | uint8 | profile | `0` – `TPA_CURVE_STALL_THROTTLE_MAX` | `USE_ADVANCED_TPA` |
| `tpa_curve_pid_thr0` | uint16 | profile | `0` – `TPA_CURVE_PID_MAX` | `USE_ADVANCED_TPA` |
| `tpa_curve_pid_thr100` | uint16 | profile | `0` – `TPA_CURVE_PID_MAX` | `USE_ADVANCED_TPA` |
| `tpa_curve_expo` | int8 | profile | `TPA_CURVE_EXPO_MIN` – `TPA_CURVE_EXPO_MAX` | `USE_ADVANCED_TPA` |
| `ez_landing_threshold` | uint8 | profile | `0` – `200` |  |
| `ez_landing_limit` | uint8 | profile | `0` – `75` |  |
| `ez_landing_speed` | uint8 | profile | `0` – `250` |  |
| `landing_disarm_threshold` | uint8 | profile | `0` – `250` |  |
| `spa_roll_center` | uint16 | profile | `0` – `UINT16_MAX` | `USE_WING` |
| `spa_roll_width` | uint16 | profile | `0` – `UINT16_MAX` | `USE_WING` |
| `spa_roll_mode` | uint8 | master | `OFF`, `ON`, `AUTO` | `USE_WING` |
| `spa_pitch_center` | uint16 | profile | `0` – `UINT16_MAX` | `USE_WING` |
| `spa_pitch_width` | uint16 | profile | `0` – `UINT16_MAX` | `USE_WING` |
| `spa_pitch_mode` | uint8 | master | `OFF`, `ON`, `AUTO` | `USE_WING` |
| `spa_yaw_center` | uint16 | profile | `0` – `UINT16_MAX` | `USE_WING` |
| `spa_yaw_width` | uint16 | profile | `0` – `UINT16_MAX` | `USE_WING` |
| `spa_yaw_mode` | uint8 | master | `OFF`, `ON`, `AUTO` | `USE_WING` |
| `yaw_type` | uint8 | master | *TABLE_YAW_TYPE* | `USE_WING` |
| `angle_pitch_offset` | int16 | profile | `-ANGLE_PITCH_OFFSET_MAX` – `ANGLE_PITCH_OFFSET_MAX` | `USE_WING` |

## Telemetry Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `tlm_inverted` | uint8 | master | `OFF`, `ON` | `USE_TELEMETRY` |
| `tlm_halfduplex` | uint8 | master | `OFF`, `ON` | `USE_TELEMETRY` |
| `crsf_tlm_accgyro` | uint8 | master | `OFF`, `ON` | `USE_TELEMETRY`, `USE_CRSF_ACCGYRO_TELEMETRY` |
| `frsky_default_lat` | int16 | master | `-9000` – `9000` | `USE_TELEMETRY`, `USE_TELEMETRY_FRSKY_HUB`, `USE_GPS` |
| `frsky_default_long` | int16 | master | `-18000` – `18000` | `USE_TELEMETRY`, `USE_TELEMETRY_FRSKY_HUB`, `USE_GPS` |
| `frsky_gps_format` | uint8 | master | `0` – `FRSKY_FORMAT_NMEA` | `USE_TELEMETRY`, `USE_TELEMETRY_FRSKY_HUB`, `USE_GPS` |
| `frsky_unit` | uint8 | master | `IMPERIAL`, `METRIC`, `BRITISH` | `USE_TELEMETRY`, `USE_TELEMETRY_FRSKY_HUB`, `USE_GPS` |
| `frsky_vfas_precision` | uint8 | master | `FRSKY_VFAS_PRECISION_LOW` – `FRSKY_VFAS_PRECISION_HIGH` | `USE_TELEMETRY`, `USE_TELEMETRY_FRSKY_HUB` |
| `hott_alarm_int` | uint8 | master | `0` – `120` | `USE_TELEMETRY` |
| `pid_in_tlm` | uint8 | master | `OFF`, `ON` | `USE_TELEMETRY` |
| `report_cell_voltage` | uint8 | master | `OFF`, `ON` | `USE_TELEMETRY` |
| `ibus_sensor` | uint8 | master | array\[IBUS_SENSOR_COUNT\] | `USE_TELEMETRY`, `USE_TELEMETRY_IBUS` |
| `mavlink_mah_as_heading_divisor` | uint16 | master | `0` – `30000` | `USE_TELEMETRY`, `USE_TELEMETRY_MAVLINK` |
| `mavlink_min_txbuff` | uint8 | master | `1` – `100` | `USE_TELEMETRY`, `USE_TELEMETRY_MAVLINK` |
| `mavlink_ext_status_rate` | uint8 | master | `0` – `50` | `USE_TELEMETRY`, `USE_TELEMETRY_MAVLINK` |
| `mavlink_rc_chan_rate` | uint8 | master | `0` – `50` | `USE_TELEMETRY`, `USE_TELEMETRY_MAVLINK` |
| `mavlink_pos_rate` | uint8 | master | `0` – `50` | `USE_TELEMETRY`, `USE_TELEMETRY_MAVLINK` |
| `mavlink_extra1_rate` | uint8 | master | `0` – `50` | `USE_TELEMETRY`, `USE_TELEMETRY_MAVLINK` |
| `mavlink_extra2_rate` | uint8 | master | `0` – `50` | `USE_TELEMETRY`, `USE_TELEMETRY_MAVLINK` |
| `mavlink_extra3_rate` | uint8 | master | `0` – `50` | `USE_TELEMETRY`, `USE_TELEMETRY_MAVLINK` |
| `telemetry_disabled_voltage` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_current` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_fuel` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_mode` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_acc_x` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_acc_y` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_acc_z` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_pitch` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_roll` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_heading` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_altitude` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_vario` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_lat_long` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_ground_speed` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_distance` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_esc_current` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_esc_voltage` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_esc_rpm` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_esc_temperature` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_temperature` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_cap_used` | uint32 | master | bitflag | `USE_TELEMETRY`, `USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |
| `telemetry_disabled_sensors` | uint32 | master | `0` – `SENSOR_ALL` | `USE_TELEMETRY`, `!USE_TELEMETRY_SENSORS_DISABLED_DETAILS` |

## LED Strip Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `ledstrip_visual_beeper` | uint8 | master | `OFF`, `ON` | `USE_LED_STRIP` |
| `ledstrip_visual_beeper_color` | uint8 | master | `AUTO`, `TIM1`, `TIM8` | `USE_LED_STRIP` |
| `ledstrip_grb_rgb` | uint8 | master | `ROLL`, `PITCH` | `USE_LED_STRIP` |
| `ledstrip_profile` | uint8 | master | `OFF`, `2_POINT`, `3_POINT`, `4_POINT` | `USE_LED_STRIP` |
| `ledstrip_race_color` | uint8 | master | `AUTO`, `TIM1`, `TIM8` | `USE_LED_STRIP` |
| `ledstrip_beacon_color` | uint8 | master | `AUTO`, `TIM1`, `TIM8` | `USE_LED_STRIP` |
| `ledstrip_beacon_period_ms` | uint16 | master | `50` – `10000` | `USE_LED_STRIP` |
| `ledstrip_beacon_percent` | uint8 | master | `0` – `100` | `USE_LED_STRIP` |
| `ledstrip_beacon_armed_only` | uint8 | master | `OFF`, `ON` | `USE_LED_STRIP` |
| `ledstrip_brightness` | uint8 | master | `5` – `100` | `USE_LED_STRIP` |
| `ledstrip_rainbow_delta` | uint16 | master | `0` – `HSV_HUE_MAX` | `USE_LED_STRIP` |
| `ledstrip_rainbow_freq` | uint16 | master | `1` – `2000` | `USE_LED_STRIP` |

## Transponder Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `transponder_provider` | uint8 | master | *TABLE_TRANSPONDER_PROVIDER* | `USE_TRANSPONDER` |
| `transponder_data` | uint8 | master | array\[TRANSPONDER_DATA_LENGTH\] | `USE_TRANSPONDER` |

## SDCARD Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `sdcard_detect_inverted` | uint8 | hardware | `OFF`, `ON` | `USE_SDCARD` |
| `sdcard_mode` | uint8 | hardware | `BLACK`, `WHITE`, `RED`, `ORANGE`, `YELLOW`, `LIME_GREEN`, `GREEN`, `MINT_GREEN`, `CYAN`, `LIGHT_BLUE`, `BLUE`, `DARK_VIOLET`, `MAGENTA`, `DEEP_PINK` | `USE_SDCARD` |
| `sdcard_spi_bus` | uint8 | hardware | `0` – `SPIDEV_COUNT` | `USE_SDCARD_SPI` |

## SDIO Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `sdio_clk_bypass` | uint8 | hardware | `OFF`, `ON` | `USE_SDCARD_SDIO` |
| `sdio_use_cache` | uint8 | hardware | `OFF`, `ON` | `USE_SDCARD_SDIO` |
| `sdio_use_4bit_width` | uint8 | hardware | `OFF`, `ON` | `USE_SDCARD_SDIO` |
| `sdio_device` | uint8 | hardware | `0` – `SDIODEV_COUNT` | `USE_SDCARD_SDIO` |

## OSD Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `osd_units` | uint8 | master | `IMPERIAL`, `METRIC`, `BRITISH` | `USE_OSD` |
| `osd_warn_bitmask` | uint32 | master | `0` – `UINT32_MAX` | `USE_OSD` |
| `osd_rssi_alarm` | uint8 | master | `0` – `100` | `USE_OSD` |
| `osd_link_quality_alarm` | uint16 | master | `0` – `100` | `USE_OSD`, `USE_RX_LINK_QUALITY_INFO` |
| `osd_rssi_dbm_alarm` | int16 | master | `CRSF_RSSI_MIN` – `CRSF_RSSI_MAX` | `USE_OSD`, `USE_RX_RSSI_DBM` |
| `osd_rsnr_alarm` | int16 | master | `CRSF_SNR_MIN` – `CRSF_SNR_MAX` | `USE_OSD`, `USE_RX_RSNR` |
| `osd_cap_alarm` | uint16 | master | `0` – `20000` | `USE_OSD` |
| `osd_alt_alarm` | uint16 | master | `0` – `10000` | `USE_OSD` |
| `osd_distance_alarm` | uint16 | master | `0` – `UINT16_MAX` | `USE_OSD` |
| `osd_esc_temp_alarm` | uint8 | master | `0` – `UINT8_MAX` | `USE_OSD` |
| `osd_esc_rpm_alarm` | int16 | master | `ESC_RPM_ALARM_OFF` – `INT16_MAX` | `USE_OSD` |
| `osd_esc_current_alarm` | int16 | master | `ESC_CURRENT_ALARM_OFF` – `INT16_MAX` | `USE_OSD` |
| `osd_core_temp_alarm` | uint8 | master | `0` – `UINT8_MAX` | `USE_OSD`, `USE_ADC_INTERNAL` |
| `osd_ah_max_pit` | uint8 | master | `0` – `90` | `USE_OSD` |
| `osd_ah_max_rol` | uint8 | master | `0` – `90` | `USE_OSD` |
| `osd_ah_invert` | uint8 | master | `OFF`, `ON` | `USE_OSD` |
| `osd_logo_on_arming` | uint8 | master | `AU433`, `AU915`, `EU433`, `EU868`, `IN866`, `FCC915`, `ISM2400`, `CE2400`, `NONE` | `USE_OSD` |
| `osd_logo_on_arming_duration` | uint8 | master | `5` – `50` | `USE_OSD` |
| `osd_arming_logo` | uint8 | master | `0` – `DISPLAYPORT_SEVERITY_COUNT - 1` | `USE_OSD` |
| `osd_use_quick_menu` | uint8 | master | `OFF`, `ON` | `USE_OSD`, `USE_OSD_QUICK_MENU` |
| `osd_show_spec_prearm` | uint8 | master | `OFF`, `ON` | `USE_OSD`, `USE_SPEC_PREARM_SCREEN` |
| `osd_tim1` | uint16 | master | `0` – `INT16_MAX` | `USE_OSD` |
| `osd_tim2` | uint16 | master | `0` – `INT16_MAX` | `USE_OSD` |
| `osd_stick_overlay_radio_mode` | uint8 | master | `1` – `4` | `USE_OSD`, `USE_OSD_STICK_OVERLAY` |
| `osd_stat_bitmask` | uint32 | master | `0` – `UINT32_MAX` | `USE_OSD` |
| `osd_profile` | uint8 | master | `1` – `OSD_PROFILE_COUNT` | `USE_OSD`, `USE_OSD_PROFILES` |
| `osd_profile_1_name` | uint8 | master | string\[1-OSD_PROFILE_NAME_LENGTH\] | `USE_OSD`, `USE_OSD_PROFILES` |
| `osd_profile_2_name` | uint8 | master | string\[1-OSD_PROFILE_NAME_LENGTH\] | `USE_OSD`, `USE_OSD_PROFILES` |
| `osd_profile_3_name` | uint8 | master | string\[1-OSD_PROFILE_NAME_LENGTH\] | `USE_OSD`, `USE_OSD_PROFILES` |
| `osd_gps_sats_show_pdop` | uint8 | master | `OFF`, `ON` | `USE_OSD` |
| `osd_displayport_device` | uint8 | master | `TRANSPARENT`, `BLACK`, `GRAY`, `LIGHT_GRAY` | `USE_OSD` |
| `osd_rcchannels` | int8 | master | array\[OSD_RCCHANNELS_COUNT\] | `USE_OSD` |
| `osd_camera_frame_width` | uint8 | master | `OSD_CAMERA_FRAME_MIN_WIDTH` – `OSD_CAMERA_FRAME_MAX_WIDTH` | `USE_OSD` |
| `osd_camera_frame_height` | uint8 | master | `OSD_CAMERA_FRAME_MIN_HEIGHT` – `OSD_CAMERA_FRAME_MAX_HEIGHT` | `USE_OSD` |
| `osd_stat_avg_cell_value` | uint8 | master | `OFF`, `ON` | `USE_OSD` |
| `osd_framerate_hz` | uint16 | master | `OSD_FRAMERATE_MIN_HZ` – `OSD_FRAMERATE_MAX_HZ` | `USE_OSD` |
| `osd_menu_background` | uint8 | master | `NONE`, `ILAP`, `ARCITIMER`, `ERLT` | `USE_OSD` |
| `osd_aux_channel` | uint8 | master | `1` – `MAX_SUPPORTED_RC_CHANNEL_COUNT` | `USE_OSD` |
| `osd_aux_scale` | uint16 | master | `1` – `1000` | `USE_OSD` |
| `osd_aux_symbol` | uint8 | master | `0` – `255` | `USE_OSD` |
| `osd_canvas_width` | uint8 | master | `0` – `63` | `USE_OSD` |
| `osd_canvas_height` | uint8 | master | `0` – `31` | `USE_OSD` |
| `osd_craftname_msgs` | uint8 | master | `OFF`, `ON` | `USE_OSD`, `USE_CRAFTNAME_MSGS` |

## OSD Element Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `osd_vbat_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_rssi_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_link_quality_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_RX_LINK_QUALITY_INFO` |
| `osd_link_tx_power_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_RX_LINK_UPLINK_POWER` |
| `osd_rssi_dbm_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_RX_RSSI_DBM` |
| `osd_rsnr_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_RX_RSNR` |
| `osd_tim_1_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_tim_2_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_remaining_time_estimate_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_flymode_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_anti_gravity_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_g_force_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_throttle_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_vtx_channel_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_crosshairs_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_ah_sbar_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_ah_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_current_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_mah_drawn_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_wh_drawn_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_motor_diag_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_craft_name_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_pilot_name_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_gps_speed_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_gps_lon_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_gps_lat_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_gps_lap_curr_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_GPS_LAP_TIMER` |
| `osd_gps_lap_prev_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_GPS_LAP_TIMER` |
| `osd_gps_lap_best3_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_GPS_LAP_TIMER` |
| `osd_gps_sats_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_home_dir_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_home_dist_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_flight_dist_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_compass_bar_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_altitude_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_pid_roll_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_pid_pitch_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_pid_yaw_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_debug_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_debug2_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_power_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_pidrate_profile_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_warnings_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_avg_cell_voltage_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_pit_ang_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_rol_ang_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_battery_usage_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_disarmed_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_nheading_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_up_down_reference_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_ready_mode_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_nvario_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_VARIO` |
| `osd_esc_tmp_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_esc_rpm_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_esc_rpm_freq_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_rtc_date_time_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_adjustment_range_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_flip_arrow_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_core_temp_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_ADC_INTERNAL` |
| `osd_log_status_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_BLACKBOX` |
| `osd_stick_overlay_left_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_OSD_STICK_OVERLAY` |
| `osd_stick_overlay_right_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_OSD_STICK_OVERLAY` |
| `osd_rate_profile_name_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_PROFILE_NAMES` |
| `osd_pid_profile_name_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_PROFILE_NAMES` |
| `osd_battery_profile_name_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_PROFILE_NAMES` |
| `osd_profile_name_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_OSD_PROFILES` |
| `osd_rcchannels_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_camera_frame_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_efficiency_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_total_flights_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_aux_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_sys_goggle_voltage_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_MSP_DISPLAYPORT` |
| `osd_sys_vtx_voltage_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_MSP_DISPLAYPORT` |
| `osd_sys_bitrate_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_MSP_DISPLAYPORT` |
| `osd_sys_delay_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_MSP_DISPLAYPORT` |
| `osd_sys_distance_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_MSP_DISPLAYPORT` |
| `osd_sys_lq_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_MSP_DISPLAYPORT` |
| `osd_sys_goggle_dvr_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_MSP_DISPLAYPORT` |
| `osd_sys_vtx_dvr_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_MSP_DISPLAYPORT` |
| `osd_sys_warnings_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_MSP_DISPLAYPORT` |
| `osd_sys_vtx_temp_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_MSP_DISPLAYPORT` |
| `osd_sys_fan_speed_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_MSP_DISPLAYPORT` |
| `osd_lidar_dist_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_RANGEFINDER` |
| `osd_custom_serial_text_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD` |
| `osd_wp_number_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_GPS` |
| `osd_wp_current_lat_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_GPS` |
| `osd_wp_current_lon_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_GPS` |
| `osd_wp_current_alt_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_GPS` |
| `osd_wp_distance_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_GPS` |
| `osd_wp_direction_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_GPS` |
| `osd_wp_next_number_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_GPS` |
| `osd_wp_eta_pos` | uint16 | master | `0` – `OSD_POSCFG_MAX` | `USE_OSD`, `USE_GPS` |

## OSD Custom Text Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `osd_custom_serial_text_terminator` | uint8 | master | `CLASSIC`, `HYPERBOLIC` | `USE_OSD` |

## System Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `system_hse_mhz` | uint8 | hardware | `0` – `30` |  |
| `task_statistics` | int8 | master | `OFF`, `ON` |  |
| `debug_mode` | uint8 | master | `PT1`, `BIQUAD`, `PT2`, `PT3` |  |
| `rate_6pos_switch` | uint8 | master | `OFF`, `ON` |  |
| `cpu_overclock` | uint8 | master | `GYRO`, `SETPOINT` | `USE_OVERCLOCK` |
| `pwr_on_arm_grace` | uint8 | master | `0` – `30` |  |
| `enable_stick_arming` | uint8 | master | `OFF`, `ON` |  |

## VTX Settings Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `vtx_band` | uint8 | master | `0` – `VTX_TABLE_MAX_BANDS` | `USE_VTX_COMMON` |
| `vtx_channel` | uint8 | master | `0` – `VTX_TABLE_MAX_CHANNELS` | `USE_VTX_COMMON` |
| `vtx_power` | uint8 | master | `0` – `VTX_TABLE_MAX_POWER_LEVELS - 1` | `USE_VTX_COMMON` |
| `vtx_low_power_disarm` | uint8 | master | `RACE`, `BEACON` | `USE_VTX_COMMON` |
| `vtx_softserial_alt` | uint8 | master | `OFF`, `ON` | `USE_VTX_COMMON` |
| `vtx_freq` | uint16 | master | `0` – `VTX_SETTINGS_MAX_FREQUENCY_MHZ` | `USE_VTX_COMMON` |
| `vtx_pit_mode_freq` | uint16 | master | `0` – `VTX_SETTINGS_MAX_FREQUENCY_MHZ` | `USE_VTX_COMMON` |

## VTX Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `vtx_halfduplex` | uint8 | master | `OFF`, `ON` | `USE_VTX_CONTROL`, `USE_VTX_COMMON` |

## VTX Io Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `vtx_spi_bus` | uint8 | hardware | `0` – `SPIDEV_COUNT` | `USE_VTX_RTC6705` |

## VCD Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `vcd_video_system` | uint8 | master | `OFF`, `ON`, `UNTIL_FIRST_ARM` | `USE_VIDEO_SYSTEM` |
| `vcd_h_offset` | int8 | master | `-32` – `31` | `USE_MAX7456` |
| `vcd_v_offset` | int8 | master | `-15` – `16` | `USE_MAX7456` |

## MAX7456 Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `max7456_clock` | uint8 | hardware | `OFF`, `YAW`, `ALL` | `USE_MAX7456` |
| `max7456_spi_bus` | uint8 | hardware | `0` – `SPIDEV_COUNT` | `USE_MAX7456` |
| `max7456_preinit_opu` | uint8 | hardware | `OFF`, `ON` | `USE_MAX7456` |

## Display Port MSP Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `displayport_msp_col_adjust` | int8 | master | `-6` – `0` | `USE_MSP_DISPLAYPORT` |
| `displayport_msp_row_adjust` | int8 | master | `-3` – `0` | `USE_MSP_DISPLAYPORT` |
| `displayport_msp_fonts` | uint8 | master | array\[4\] | `USE_MSP_DISPLAYPORT` |
| `displayport_msp_use_device_blink` | uint8 | master | `OFF`, `ON` | `USE_MSP_DISPLAYPORT` |

## Display Port MAX7456 Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `displayport_max7456_col_adjust` | int8 | master | `-6` – `0` | `USE_MAX7456` |
| `displayport_max7456_row_adjust` | int8 | master | `-3` – `0` | `USE_MAX7456` |
| `displayport_max7456_inv` | uint8 | master | `OFF`, `ON` | `USE_MAX7456` |
| `displayport_max7456_blk` | uint8 | master | `0` – `3` | `USE_MAX7456` |
| `displayport_max7456_wht` | uint8 | master | `0` – `3` | `USE_MAX7456` |

## ESC Sensor Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `esc_sensor_halfduplex` | uint8 | master | `OFF`, `ON` | `USE_ESC_SENSOR` |
| `esc_sensor_current_offset` | uint16 | master | `0` – `16000` | `USE_ESC_SENSOR` |

## RX CC2500 SPI Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `frsky_spi_autobind` | uint8 | master | `OFF`, `ON` | `USE_RX_FRSKY_SPI` |
| `frsky_spi_tx_id` | uint8 | master | array\[3\] | `USE_RX_FRSKY_SPI` |
| `frsky_spi_offset` | int8 | master | `-127` – `127` | `USE_RX_FRSKY_SPI` |
| `frsky_spi_bind_hop_data` | uint8 | master | array\[50\] | `USE_RX_FRSKY_SPI` |
| `frsky_x_rx_num` | uint8 | master | `0` – `UINT8_MAX` | `USE_RX_FRSKY_SPI` |
| `frsky_spi_a1_source` | uint8 | master | `BETAFLIGHT`, `RACEFLIGHT`, `KISS`, `ACTUAL`, `QUICK` | `USE_RX_FRSKY_SPI` |
| `cc2500_spi_chip_detect` | uint8 | hardware | `OFF`, `ON` | `USE_RX_FRSKY_SPI` |

## Status LED Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `led_inversion` | uint8 | hardware | `0` – `((1 << STATUS_LED_COUNT) - 1)` | `!USE_VIRTUAL_LED` |

## Dashboard Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `dashboard_i2c_bus` | uint8 | hardware | `0` – `I2CDEV_COUNT` | `USE_DASHBOARD` |
| `dashboard_i2c_addr` | uint8 | hardware | `I2C_ADDR7_MIN` – `I2C_ADDR7_MAX` | `USE_DASHBOARD` |

## Camera Control Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `camera_control_mode` | uint8 | master | `VBAT`, `EXTADC`, `CONST` | `USE_CAMERA_CONTROL` |
| `camera_control_ref_voltage` | uint16 | master | `200` – `400` | `USE_CAMERA_CONTROL` |
| `camera_control_key_delay` | uint16 | master | `100` – `500` | `USE_CAMERA_CONTROL` |
| `camera_control_internal_resistance` | uint16 | master | `10` – `1000` | `USE_CAMERA_CONTROL` |
| `camera_control_button_resistance` | uint16 | master | array\[CAMERA_CONTROL_KEYS_COUNT\] | `USE_CAMERA_CONTROL` |
| `camera_control_inverted` | uint8 | master | `OFF`, `ON` | `USE_CAMERA_CONTROL` |

## Rangefinder Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `rangefinder_hardware` | uint8 | master | `OFF`, `108MHZ`, `120MHZ`, `192MHZ`, `216MHZ`, `240MHZ` | `USE_RANGEFINDER` |

## Opticalflow Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `opticalflow_hardware` | uint8 | master | `GRB`, `RGB`, `GRBW` | `USE_OPTICALFLOW` |
| `opticalflow_rotation` | int16 | master | `0` – `359` | `USE_OPTICALFLOW` |
| `opticalflow_lpf` | uint16 | master | `0` – `10000` | `USE_OPTICALFLOW` |
| `opticalflow_flip_x` | uint8 | master | `OFF`, `ON` | `USE_OPTICALFLOW` |

## Pinio Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `pinio_config` | uint8 | hardware | array\[PINIO_COUNT\] | `USE_PINIO` |

## Piniobox Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `pinio_box` | uint8 | hardware | array\[PINIO_COUNT\] | `USE_PINIO`, `USE_PINIOBOX` |

## USB Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `usb_hid_cdc` | uint8 | master | `OFF`, `ON` | `USE_USB_CDC_HID` |
| `usb_msc_pin_pullup` | uint8 | hardware | `OFF`, `ON` | `USE_USB_MSC` |

## Flash Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `flash_spi_bus` | uint8 | hardware | `0` – `SPIDEV_COUNT` | `USE_FLASH_SPI` |

## RCDEVICE Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `rcdevice_init_dev_attempts` | uint8 | master | `0` – `10` | `USE_RCDEVICE` |
| `rcdevice_init_dev_attempt_interval` | uint32 | master | `0` – `5000` | `USE_RCDEVICE` |
| `rcdevice_protocol_version` | uint8 | master | `0` – `1` | `USE_RCDEVICE` |
| `rcdevice_feature` | uint16 | master | `0` – `65535` | `USE_RCDEVICE` |

## Gyro Device Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `gyro_1_bustype` | uint8 | hardware | `AUTO`, `GPS_ONLY`, `OPTICALFLOW_ONLY` |
| `gyro_1_spibus` | uint8 | hardware | `0` – `SPIDEV_COUNT` |
| `gyro_1_i2cBus` | uint8 | hardware | `0` – `I2CDEV_COUNT` |
| `gyro_1_i2c_address` | uint8 | hardware | `0` – `I2C_ADDR7_MAX` |
| `gyro_2_bustype` | uint8 | hardware | `AUTO`, `GPS_ONLY`, `OPTICALFLOW_ONLY` |
| `gyro_2_spibus` | uint8 | hardware | `0` – `SPIDEV_COUNT` |
| `gyro_2_i2cBus` | uint8 | hardware | `0` – `I2CDEV_COUNT` |
| `gyro_2_i2c_address` | uint8 | hardware | `0` – `I2C_ADDR7_MAX` |
| `gyro_3_bustype` | uint8 | hardware | `AUTO`, `GPS_ONLY`, `OPTICALFLOW_ONLY` |
| `gyro_3_spibus` | uint8 | hardware | `0` – `SPIDEV_COUNT` |
| `gyro_3_i2cBus` | uint8 | hardware | `0` – `I2CDEV_COUNT` |
| `gyro_3_i2c_address` | uint8 | hardware | `0` – `I2C_ADDR7_MAX` |
| `gyro_4_bustype` | uint8 | hardware | `AUTO`, `GPS_ONLY`, `OPTICALFLOW_ONLY` |
| `gyro_4_spibus` | uint8 | hardware | `0` – `SPIDEV_COUNT` |
| `gyro_4_i2cBus` | uint8 | hardware | `0` – `I2CDEV_COUNT` |
| `gyro_4_i2c_address` | uint8 | hardware | `0` – `I2C_ADDR7_MAX` |
| `gyro_5_bustype` | uint8 | hardware | `AUTO`, `GPS_ONLY`, `OPTICALFLOW_ONLY` |
| `gyro_5_spibus` | uint8 | hardware | `0` – `SPIDEV_COUNT` |
| `gyro_5_i2cBus` | uint8 | hardware | `0` – `I2CDEV_COUNT` |
| `gyro_5_i2c_address` | uint8 | hardware | `0` – `I2C_ADDR7_MAX` |
| `gyro_6_bustype` | uint8 | hardware | `AUTO`, `GPS_ONLY`, `OPTICALFLOW_ONLY` |
| `gyro_6_spibus` | uint8 | hardware | `0` – `SPIDEV_COUNT` |
| `gyro_6_i2cBus` | uint8 | hardware | `0` – `I2CDEV_COUNT` |
| `gyro_6_i2c_address` | uint8 | hardware | `0` – `I2C_ADDR7_MAX` |
| `gyro_7_bustype` | uint8 | hardware | `AUTO`, `GPS_ONLY`, `OPTICALFLOW_ONLY` |
| `gyro_7_spibus` | uint8 | hardware | `0` – `SPIDEV_COUNT` |
| `gyro_7_i2cBus` | uint8 | hardware | `0` – `I2CDEV_COUNT` |
| `gyro_7_i2c_address` | uint8 | hardware | `0` – `I2C_ADDR7_MAX` |
| `gyro_8_bustype` | uint8 | hardware | `AUTO`, `GPS_ONLY`, `OPTICALFLOW_ONLY` |
| `gyro_8_spibus` | uint8 | hardware | `0` – `SPIDEV_COUNT` |
| `gyro_8_i2cBus` | uint8 | hardware | `0` – `I2CDEV_COUNT` |
| `gyro_8_i2c_address` | uint8 | hardware | `0` – `I2C_ADDR7_MAX` |

## I2C Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `i2c0_pullup` | uint8 | hardware | `OFF`, `ON` | `USE_I2C_DEVICE_0` |
| `i2c0_clockspeed_khz` | uint16 | hardware | `I2C_CLOCKSPEED_MIN_KHZ` – `I2C_CLOCKSPEED_MAX_KHZ` | `USE_I2C_DEVICE_0` |
| `i2c1_pullup` | uint8 | hardware | `OFF`, `ON` | `USE_I2C_DEVICE_1` |
| `i2c1_clockspeed_khz` | uint16 | hardware | `I2C_CLOCKSPEED_MIN_KHZ` – `I2C_CLOCKSPEED_MAX_KHZ` | `USE_I2C_DEVICE_1` |
| `i2c2_pullup` | uint8 | hardware | `OFF`, `ON` | `USE_I2C_DEVICE_2` |
| `i2c2_clockspeed_khz` | uint16 | hardware | `I2C_CLOCKSPEED_MIN_KHZ` – `I2C_CLOCKSPEED_MAX_KHZ` | `USE_I2C_DEVICE_2` |
| `i2c3_pullup` | uint8 | hardware | `OFF`, `ON` | `USE_I2C_DEVICE_3` |
| `i2c3_clockspeed_khz` | uint16 | hardware | `I2C_CLOCKSPEED_MIN_KHZ` – `I2C_CLOCKSPEED_MAX_KHZ` | `USE_I2C_DEVICE_3` |
| `i2c4_pullup` | uint8 | hardware | `OFF`, `ON` | `USE_I2C_DEVICE_4` |
| `i2c4_clockspeed_khz` | uint16 | hardware | `I2C_CLOCKSPEED_MIN_KHZ` – `I2C_CLOCKSPEED_MAX_KHZ` | `USE_I2C_DEVICE_4` |

## CAN Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `can_bitrate_khz` | uint16 | hardware | `125` – `1000` |

## DRONECAN Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `dronecan_enabled` | uint8 | hardware | `OFF`, `ON` |
| `dronecan_node_id` | uint8 | hardware | `0` – `127` |
| `dronecan_device` | uint8 | hardware | `1` – `CANDEV_COUNT` |

## Mco Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `mco_on_pa8` | uint8 | hardware | `OFF`, `ON` | `USE_MCO`, `USE_MCO_DEVICE1` |
| `mco_source` | uint8 | hardware | `0` – `MCO_SOURCE_COUNT - 1` | `USE_MCO`, `USE_MCO_DEVICE1` |
| `mco_divider` | uint8 | hardware | `0` – `MCO_DIVIDER_COUNT - 1` | `USE_MCO`, `USE_MCO_DEVICE1` |
| `mco2_on_pc9` | uint8 | hardware | `OFF`, `ON` | `USE_MCO`, `USE_MCO_DEVICE2` |

## RX Spektrum SPI Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `spektrum_spi_protocol` | uint8 | master | `0` – `UINT8_MAX` | `USE_RX_SPEKTRUM` |
| `spektrum_spi_mfg_id` | uint8 | master | array\[4\] | `USE_RX_SPEKTRUM` |
| `spektrum_spi_num_channels` | uint8 | master | `0` – `DSM_MAX_CHANNEL_COUNT` | `USE_RX_SPEKTRUM` |

## RX Expresslrs SPI Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `expresslrs_uid` | uint8 | master | array\[6\] | `USE_RX_EXPRESSLRS` |
| `expresslrs_domain` | uint8 | master | *TABLE_FREQ_DOMAIN* | `USE_RX_EXPRESSLRS` |
| `expresslrs_model_id` | uint8 | master | `0` – `UINT8_MAX` | `USE_RX_EXPRESSLRS` |

## Scheduler Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `scheduler_relax_rx` | uint16 | hardware | `0` – `500` |  |
| `scheduler_relax_osd` | uint16 | hardware | `0` – `500` |  |
| `scheduler_debug_task` | uint16 | hardware | `0` – `TASK_COUNT` |  |
| `cpu_late_limit_permille` | uint8 | master | `0` – `100` | `USE_LATE_TASK_STATISTICS` |

## MSP Config

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `serialmsp_halfduplex` | uint8 | master | `OFF`, `ON` |

## Time Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `timezone_offset_minutes` | int16 | master | `TIMEZONE_OFFSET_MINUTES_MIN` – `TIMEZONE_OFFSET_MINUTES_MAX` | `USE_RTC_TIME` |

## RPM Filter Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `rpm_filter_harmonics` | uint8 | master | `0` – `3` | `USE_RPM_FILTER` |
| `rpm_filter_weights` | uint8 | master | array\[RPM_FILTER_HARMONICS_MAX\] | `USE_RPM_FILTER` |
| `rpm_filter_q` | uint16 | master | `250` – `3000` | `USE_RPM_FILTER` |
| `rpm_filter_min_hz` | uint8 | master | `30` – `200` | `USE_RPM_FILTER` |
| `rpm_filter_fade_range_hz` | uint16 | master | `0` – `1000` | `USE_RPM_FILTER` |
| `rpm_filter_lpf_hz` | uint16 | master | `100` – `500` | `USE_RPM_FILTER` |

## Flysky Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `flysky_spi_tx_id` | uint32 | master | `0` – `UINT32_MAX` | `USE_RX_FLYSKY` |
| `flysky_spi_rf_channels` | uint8 | master | array\[16\] | `USE_RX_FLYSKY` |

## Stats Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `stats_min_armed_time_s` | int8 | master | `STATS_OFF` – `INT8_MAX` | `USE_PERSISTENT_STATS` |
| `stats_total_flights` | uint32 | master | `0` – `UINT32_MAX` | `USE_PERSISTENT_STATS` |
| `stats_total_time_s` | uint32 | master | `0` – `UINT32_MAX` | `USE_PERSISTENT_STATS` |
| `stats_total_dist_m` | uint32 | master | `0` – `UINT32_MAX` | `USE_PERSISTENT_STATS` |
| `stats_save_move_limit` | uint8 | master | `0` – `UINT8_MAX` | `USE_PERSISTENT_STATS` |
| `stats_mah_used` | uint32 | master | `0` – `UINT32_MAX` | `USE_PERSISTENT_STATS`, `USE_BATTERY_CONTINUE` |

## Pilot Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `craft_name` | uint8 | master | string\[1-MAX_NAME_LENGTH\] |  |
| `pilot_name` | uint8 | master | string\[1-MAX_NAME_LENGTH\] | `USE_OSD` |

## Position

| Parameter | Type | Scope | Range / Values |
|-----------|------|-------|----------------|
| `altitude_source` | int8 | master | `OFF`, `ON`, `FIRST_ARMING` |
| `altitude_prefer_baro` | int8 | master | `0` – `100` |
| `altitude_lpf` | uint16 | master | `10` – `5000` |
| `rangefinder_max_range_cm` | uint16 | master | `50` – `1000` |
| `altitude_d_lpf` | uint16 | master | `10` – `5000` |

## Autopilot

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `ap_landing_altitude_m` | uint8 | master | `0` – `200` | `!USE_WING` |
| `ap_hover_throttle` | uint16 | master | `0` – `1700` | `!USE_WING` |
| `ap_throttle_min` | uint16 | master | `1050` – `1400` | `!USE_WING` |
| `ap_throttle_max` | uint16 | master | `1400` – `2000` | `!USE_WING` |
| `ap_altitude_p` | uint8 | master | `0` – `200` | `!USE_WING` |
| `ap_altitude_i` | uint8 | master | `0` – `200` | `!USE_WING` |
| `ap_altitude_d` | uint8 | master | `0` – `200` | `!USE_WING` |
| `ap_altitude_f` | uint8 | master | `0` – `200` | `!USE_WING` |
| `ap_position_p` | uint8 | master | `0` – `200` | `!USE_WING` |
| `ap_position_i` | uint8 | master | `0` – `200` | `!USE_WING` |
| `ap_position_d` | uint8 | master | `0` – `200` | `!USE_WING` |
| `ap_position_a` | uint8 | master | `0` – `200` | `!USE_WING` |
| `ap_position_cutoff` | uint8 | master | `10` – `250` | `!USE_WING` |
| `ap_stop_threshold` | uint8 | master | `0` – `100` | `!USE_WING` |
| `ap_max_angle` | uint8 | master | `10` – `70` | `!USE_WING` |
| `ap_velocity_control_enable` | uint8 | master | `OFF`, `ON` | `!USE_WING` |
| `ap_velocity_p` | uint8 | master | `0` – `200` | `!USE_WING` |
| `ap_velocity_i` | uint8 | master | `0` – `200` | `!USE_WING` |
| `ap_velocity_d` | uint8 | master | `0` – `100` | `!USE_WING` |
| `ap_velocity_drag_coeff` | uint16 | master | `0` – `1000` | `!USE_WING` |
| `ap_max_velocity` | uint16 | master | `100` – `5000` | `!USE_WING` |
| `ap_waypoint_arrival_radius` | uint16 | master | `100` – `5000` | `!USE_WING` |
| `ap_waypoint_hold_radius` | uint16 | master | `50` – `1000` | `!USE_WING` |
| `ap_stick_deadband` | uint16 | master | `0` – `500` | `!USE_WING` |
| `ap_throttle_deadband` | uint16 | master | `0` – `500` | `!USE_WING` |
| `ap_yaw_mode` | uint8 | master | `VELOCITY`, `BEARING`, `HYBRID`, `FIXED`, `DAMPENER` | `!USE_WING` |
| `ap_yaw_p` | uint16 | master | `0` – `500` | `!USE_WING` |
| `ap_yaw_d` | uint16 | master | `0` – `200` | `!USE_WING` |
| `ap_max_yaw_rate` | uint16 | master | `10` – `360` | `!USE_WING` |
| `ap_min_forward_velocity` | uint16 | master | `10` – `500` | `!USE_WING` |
| `ap_velocity_buildup_max_pitch` | uint8 | master | `0` – `20` | `!USE_WING` |
| `ap_max_turn_rate` | uint8 | master | `1` – `90` | `!USE_WING` |
| `ap_hold_orbit_radius` | uint16 | master | `300` – `5000` | `!USE_WING` |
| `ap_hold_figure8_width` | uint16 | master | `500` – `10000` | `!USE_WING` |
| `ap_landing_descent_rate` | uint8 | master | `10` – `200` | `!USE_WING` |
| `ap_landing_detection_time` | uint8 | master | `5` – `50` | `!USE_WING` |
| `ap_landing_spiral_enable` | uint8 | master | `OFF`, `ON` | `!USE_WING` |
| `ap_landing_spiral_radius` | uint16 | master | `50` – `2000` | `!USE_WING` |
| `ap_landing_spiral_rate` | uint8 | master | `1` – `45` | `!USE_WING` |
| `ap_landing_velocity_threshold` | uint8 | master | `10` – `200` | `!USE_WING` |
| `ap_landing_throttle_threshold` | uint16 | master | `10` – `500` | `!USE_WING` |
| `ap_l1_enable` | uint8 | master | `OFF`, `ON` | `!USE_WING` |
| `ap_l1_period` | uint16 | master | `5` – `100` | `!USE_WING` |
| `ap_l1_min_lookahead` | uint16 | master | `100` – `5000` | `!USE_WING` |
| `ap_l1_max_lookahead` | uint16 | master | `1000` – `50000` | `!USE_WING` |
| `ap_l1_max_cross_track_error` | uint16 | master | `1000` – `50000` | `!USE_WING` |
| `ap_l1_turn_rate` | uint8 | master | `0` – `90` | `!USE_WING` |
| `ap_min_nav_altitude_m` | uint8 | master | `0` – `50` | `!USE_WING` |
| `ap_rx_loss_policy` | uint8 | master | `DISABLE`, `CONTINUE`, `LAND` | `!USE_WING` |
| `ap_max_distance_from_home` | uint16 | master | `0` – `10000` | `!USE_WING` |
| `ap_geofence_action` | uint8 | master | `LAND`, `RTH` | `!USE_WING` |

## Mode Activation Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `box_user_1_name` | uint8 | hardware | string\[1-MAX_BOX_USER_NAME_LENGTH\] | `USE_CUSTOM_BOX_NAMES` |
| `box_user_2_name` | uint8 | hardware | string\[1-MAX_BOX_USER_NAME_LENGTH\] | `USE_CUSTOM_BOX_NAMES` |
| `box_user_3_name` | uint8 | hardware | string\[1-MAX_BOX_USER_NAME_LENGTH\] | `USE_CUSTOM_BOX_NAMES` |
| `box_user_4_name` | uint8 | hardware | string\[1-MAX_BOX_USER_NAME_LENGTH\] | `USE_CUSTOM_BOX_NAMES` |

## Gimbal Track Config

| Parameter | Type | Scope | Range / Values | Requires |
|-----------|------|-------|----------------|----------|
| `gimbal_roll_rc_gain` | int8 | master | `-100` – `100` | `USE_GIMBAL` |
| `gimbal_pitch_rc_thr_gain` | int8 | master | `-100` – `100` | `USE_GIMBAL` |
| `gimbal_pitch_rc_low_gain` | int8 | master | `-100` – `100` | `USE_GIMBAL` |
| `gimbal_pitch_rc_high_gain` | int8 | master | `-100` – `100` | `USE_GIMBAL` |
| `gimbal_yaw_rc_gain` | int8 | master | `-100` – `100` | `USE_GIMBAL` |
| `gimbal_roll_gain` | int8 | master | `-100` – `100` | `USE_GIMBAL` |
| `gimbal_roll_offset` | int8 | master | `-100` – `100` | `USE_GIMBAL` |
| `gimbal_roll_limit` | int8 | master | `-100` – `100` | `USE_GIMBAL` |
| `gimbal_pitch_gain` | int8 | master | `-100` – `100` | `USE_GIMBAL` |
| `gimbal_pitch_offset` | int8 | master | `-100` – `100` | `USE_GIMBAL` |
| `gimbal_pitch_low_limit` | int8 | master | `-100` – `100` | `USE_GIMBAL` |
| `gimbal_pitch_high_limit` | int8 | master | `-100` – `100` | `USE_GIMBAL` |
| `gimbal_yaw_gain` | int8 | master | `-100` – `100` | `USE_GIMBAL` |
| `gimbal_yaw_offset` | int8 | master | `-100` – `100` | `USE_GIMBAL` |
| `gimbal_yaw_limit` | int8 | master | `-100` – `100` | `USE_GIMBAL` |
| `gimbal_stabilisation` | int8 | master | `0` – `7` | `USE_GIMBAL` |
| `gimbal_sensitivity` | int8 | master | `-16` – `15` | `USE_GIMBAL` |

---
*Generated by `docs/gen_cli_docs.py` from `src/main/cli/settings.c`*
