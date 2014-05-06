#pragma once

#define MAX_PWM_MOTORS  12
#define MAX_PWM_SERVOS  8

#define MAX_PWM_INPUT_PORTS 8

#define MAX_MOTORS  12
#define MAX_SERVOS  8
#define MAX_PWM_OUTPUT_PORTS MAX_PWM_MOTORS // must be set to the largest of either MAX_MOTORS or MAX_SERVOS

#if MAX_PWM_OUTPUT_PORTS < MAX_MOTORS || MAX_PWM_OUTPUT_PORTS < MAX_SERVOS
#error Invalid motor/servo/port configuration
#endif


#define PULSE_1MS   (1000)      // 1ms pulse width
#define PULSE_MIN   (750)       // minimum PWM pulse width which is considered valid
#define PULSE_MAX   (2250)      // maximum PWM pulse width which is considered valid




#define MAX_INPUTS  8

#define PWM_TIMER_MHZ 1

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
} drv_pwm_config_t;

// This indexes into the read-only hardware definition structure, timerHardware_t, as well as into pwmPorts structure with dynamic data.
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
    PWM14
};
