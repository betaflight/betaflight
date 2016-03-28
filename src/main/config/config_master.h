/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// System-wide
typedef struct master_t {
    // Profile configuration
    uint8_t current_profile_index;

    // System Configuration
    uint32_t enabledFeatures;
    uint8_t emf_avoidance;                   // change pll settings to avoid noise in the uhf band
    uint8_t i2c_highspeed;                   // Overclock i2c Bus for faster IMU readings

    serialConfig_t serialConfig;

#ifdef BLACKBOX
    uint8_t blackbox_rate_num;
    uint8_t blackbox_rate_denom;
    uint8_t blackbox_device;
#endif

    // IMU configuration
    uint16_t looptime;                      // imu loop time in us
    uint16_t dcm_kp;                        // DCM filter proportional gain ( x 10000)
    uint16_t dcm_ki;                        // DCM filter integral gain ( x 10000)
    uint8_t gyroSync;                       // Enable interrupt based loop
    uint8_t gyroSyncDenominator;            // Gyro sync Denominator

    // Mixer configuration
    uint8_t mixerMode;
    mixerConfig_t mixerConfig;
#ifdef USE_SERVOS
    servoMixer_t customServoMixer[MAX_SERVO_RULES];
#endif

    // Motor/ESC/Servo configuration
    escAndServoConfig_t escAndServoConfig;
    flight3DConfig_t flight3DConfig;
    // FIXME move these into escAndServoConfig_t
    uint16_t motor_pwm_rate;                // The update rate of motor outputs (50-498Hz)
    uint16_t servo_pwm_rate;                // The update rate of servo outputs (50-498Hz)

    // Sensor configuration
    sensorAlignmentConfig_t sensorAlignmentConfig;

    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
    uint8_t mag_hardware;                   // Which mag hardware to use on boards with more than one device
    uint8_t baro_hardware;                  // Barometer hardware to use
    flightDynamicsTrims_t accZero;
    flightDynamicsTrims_t magZero;


    uint8_t gyro_lpf;                       // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
    float soft_gyro_lpf_hz;                 // Software based gyro filter in hz
    gyroConfig_t gyroConfig;

    batteryConfig_t batteryConfig;

    rxConfig_t rxConfig;
    inputFilteringMode_e inputFilteringMode;  // Use hardware input filtering, e.g. for OrangeRX PPM/PWM receivers.

    // Arming configuration
    uint8_t retarded_arm;                   // allow disarm/arm on throttle down + roll left/right
    uint8_t disarm_kill_switch;             // allow disarm via AUX switch regardless of throttle value
    uint8_t auto_disarm_delay;              // allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
    uint8_t small_angle;

    // Flight config
    airplaneConfig_t airplaneConfig;

    controlRateConfig_t controlRateProfiles[MAX_CONTROL_RATE_PROFILE_COUNT];
    uint16_t max_angle_inclination;         // max inclination allowed in angle (level) mode. default 500 (50 degrees).
    int8_t yaw_control_direction;           // change control direction of yaw (inverted, normal)

#ifdef GPS
    gpsConfig_t gpsConfig;
#endif

#ifdef TELEMETRY
    telemetryConfig_t telemetryConfig;
#endif

#ifdef LED_STRIP
    ledConfig_t ledConfigs[MAX_LED_STRIP_LENGTH];
    hsvColor_t colors[CONFIGURABLE_COLOR_COUNT];
#endif

#ifdef TRANSPONDER
    uint8_t transponderData[6];
#endif
} master_t;

extern master_t masterConfig;
extern controlRateConfig_t *currentControlRateProfile;
