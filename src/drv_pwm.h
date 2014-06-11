/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#pragma once

#define MAX_MOTORS  12
#define MAX_SERVOS  8
#define MAX_INPUTS  8
#define PULSE_1MS   (1000)      // 1ms pulse width
#define PULSE_MIN   (750)       // minimum PWM pulse width which is considered valid
#define PULSE_MAX   (2250)      // maximum PWM pulse width which is considered valid

typedef struct drv_pwm_config_t {
    bool enableInput;
    bool usePPM;
    bool useUART;
    bool useSoftSerial;
    bool useServos;
    bool extraServos;    // configure additional 4 channels in PPM mode as servos, not motors
    bool airplane;       // fixed wing hardware config, lots of servos etc
    uint8_t adcChannel;  // steal one RC input for current sensor
    uint16_t motorPwmRate;
    uint16_t servoPwmRate;
    uint16_t idlePulse;  // PWM value to use when initializing the driver. set this to either PULSE_1MS (regular pwm), 
                         // some higher value (used by 3d mode), or 0, for brushed pwm drivers.
    uint16_t servoCenterPulse;
    uint16_t failsafeThreshold;
    
    // OUT parameters, filled by driver
    uint8_t numServos;
} drv_pwm_config_t;

// This indexes into the read-only hardware definition structure in drv_pwm.c, as well as into pwmPorts[] structure with dynamic data.
enum {
    PWM1 = 0,
    PWM2,
    PWM3,
    PWM4,
    PWM5,
    PWM6,
    PWM7,
    PWM8,
    PWM9,
    PWM10,
    PWM11,
    PWM12,
    PWM13,
    PWM14,
    MAX_PORTS
};

void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity);

bool pwmInit(drv_pwm_config_t *init); // returns whether driver is asking to calibrate throttle or not
void pwmWriteMotor(uint8_t index, uint16_t value);
void pwmWriteServo(uint8_t index, uint16_t value);
uint16_t pwmRead(uint8_t channel);

// void pwmWrite(uint8_t channel, uint16_t value);
