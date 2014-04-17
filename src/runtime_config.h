#pragma once

typedef enum {
    FEATURE_PPM = 1 << 0,
    FEATURE_VBAT = 1 << 1,
    FEATURE_INFLIGHT_ACC_CAL = 1 << 2,
    FEATURE_SERIALRX = 1 << 3,
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_SERVO_TILT = 1 << 5,
    FEATURE_SOFTSERIAL = 1 << 6,
    FEATURE_LED_RING = 1 << 7,
    FEATURE_GPS = 1 << 8,
    FEATURE_FAILSAFE = 1 << 9,
    FEATURE_SONAR = 1 << 10,
    FEATURE_TELEMETRY = 1 << 11,
    FEATURE_POWERMETER = 1 << 12,
    FEATURE_VARIO = 1 << 13,
    FEATURE_3D = 1 << 14,
} AvailableFeatures;

// Core runtime settings
typedef struct core_t {
    serialPort_t *mainport;
    serialPort_t *gpsport;
    serialPort_t *telemport;
    serialPort_t *rcvrport;
    uint8_t mpu6050_scale;                  // es/non-es variance between MPU6050 sensors, half my boards are mpu6000ES, need this to be dynamic. automatically set by mpu6050 driver.
    uint8_t numRCChannels;                  // number of rc channels as reported by current input driver
    bool useServo;                          // feature SERVO_TILT or wing/airplane mixers will enable this
} core_t;

typedef uint16_t (* rcReadRawDataPtr)(uint8_t chan);        // used by receiver driver to return channel data
typedef void (* pidControllerFuncPtr)(void);                // pid controller function prototype

extern core_t core;
