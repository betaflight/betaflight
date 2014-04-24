#pragma once

// System-wide
typedef struct master_t {
    uint8_t version;
    uint16_t size;
    uint8_t magic_be;                       // magic number, should be 0xBE

    uint8_t mixerConfiguration;
    uint32_t enabledFeatures;
    uint16_t looptime;                      // imu loop time in us
    uint8_t emf_avoidance;                   // change pll settings to avoid noise in the uhf band

    motorMixer_t customMixer[MAX_SUPPORTED_MOTORS]; // custom mixtable

    // motor/esc/servo related stuff
    escAndServoConfig_t escAndServoConfig;
    flight3DConfig_t flight3DConfig;

    uint16_t motor_pwm_rate;                // The update rate of motor outputs (50-498Hz)
    uint16_t servo_pwm_rate;                // The update rate of servo outputs (50-498Hz)

    // global sensor-related stuff

    sensorAlignmentConfig_t sensorAlignmentConfig;
    boardAlignment_t boardAlignment;

    int8_t yaw_control_direction;           // change control direction of yaw (inverted, normal)
    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
    uint16_t gyro_lpf;                      // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
    uint16_t gyro_cmpf_factor;              // Set the Gyro Weight for Gyro/Acc complementary filter. Increasing this value would reduce and delay Acc influence on the output of the filter.
    uint16_t gyro_cmpfm_factor;             // Set the Gyro Weight for Gyro/Magnetometer complementary filter. Increasing this value would reduce and delay Magnetometer influence on the output of the filter

    gyroConfig_t gyroConfig;

    uint16_t max_angle_inclination;         // max inclination allowed in angle (level) mode. default 500 (50 degrees).
    int16_flightDynamicsTrims_t accZero;
    int16_flightDynamicsTrims_t magZero;

    batteryConfig_t batteryConfig;

    uint8_t power_adc_channel;              // which channel is used for current sensor. Right now, only 2 places are supported: RC_CH2 (unused when in CPPM mode, = 1), RC_CH8 (last channel in PWM mode, = 9)

    rxConfig_t rxConfig;

    uint8_t retarded_arm;                   // allow disarsm/arm on throttle down + roll left/right

    airplaneConfig_t airplaneConfig;
    int8_t fixedwing_althold_dir;           // +1 or -1 for pitch/althold gain. later check if need more than just sign

    uint8_t rssi_aux_channel;               // Read rssi from channel. 1+ = AUX1+, 0 to disable.

    // gps-related stuff
    uint8_t gps_type;                       // See GPSHardware enum.
    int8_t gps_baudrate;                    // See GPSBaudRates enum.

    serialConfig_t serialConfig;

    telemetryConfig_t telemetryConfig;
    profile_t profile[3];                   // 3 separate profiles
    uint8_t current_profile_index;          // currently loaded profile

    uint8_t magic_ef;                       // magic number, should be 0xEF
    uint8_t chk;                            // XOR checksum
} master_t;

extern master_t masterConfig;
extern profile_t currentProfile;
