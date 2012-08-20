#pragma once

#define MAX_MOTORS  12
#define MAX_SERVOS  8
#define MAX_INPUTS  8

typedef struct drv_pwm_config_t {
    bool enableInput;
    bool usePPM;
    bool useServos;
    bool extraServos;    // configure additional 4 channels in PPM mode as servos, not motors
    bool airplane;       // fixed wing hardware config, lots of servos etc
    uint16_t motorPwmRate;
    uint16_t servoPwmRate;
} drv_pwm_config_t;

bool pwmInit(drv_pwm_config_t *init); // returns whether driver is asking to calibrate throttle or not
void pwmWriteMotor(uint8_t index, uint16_t value);
void pwmWriteServo(uint8_t index, uint16_t value);
uint16_t pwmRead(uint8_t channel);

// void pwmWrite(uint8_t channel, uint16_t value);
