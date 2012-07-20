#pragma once

typedef struct drv_pwm_config_t {
    bool enableInput;
    bool usePPM;
    bool useServos;
    bool extraServos;    // configure additional 4 channels in PPM mode as servos, not motors
    uint16_t motorPwmRate;
    uint16_t servoPwmRate;
} drv_pwm_config_t;

bool pwmInit(drv_pwm_config_t *init); // returns whether driver is asking to calibrate throttle or not
void pwmWrite(uint8_t channel, uint16_t value);
uint16_t pwmRead(uint8_t channel);
uint8_t pwmGetNumOutputChannels(void);
