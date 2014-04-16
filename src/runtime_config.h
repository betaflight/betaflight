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

extern core_t core;
