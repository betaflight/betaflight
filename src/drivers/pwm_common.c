
#include <stdbool.h>
#include <stdint.h>

#include <stdlib.h>

#include "platform.h"

#include "gpio_common.h"
#include "timer_common.h"

#include "failsafe.h" // FIXME dependency into the main code from a driver

#include "pwm_common.h"
/*
    Configuration maps:

    1) multirotor PPM input
    PWM1 used for PPM
    PWM5..8 used for motors
    PWM9..10 used for servo or else motors
    PWM11..14 used for motors

    2) multirotor PPM input with more servos
    PWM1 used for PPM
    PWM5..8 used for motors
    PWM9..10 used for servo or else motors
    PWM11..14 used for servos

    2) multirotor PWM input
    PWM1..8 used for input
    PWM9..10 used for servo or else motors
    PWM11..14 used for motors

    3) airplane / flying wing w/PWM
    PWM1..8 used for input
    PWM9 used for motor throttle +PWM10 for 2nd motor
    PWM11.14 used for servos

    4) airplane / flying wing with PPM
    PWM1 used for PPM
    PWM5..8 used for servos
    PWM9 used for motor throttle +PWM10 for 2nd motor
    PWM11.14 used for servos
*/

typedef struct {
    volatile uint16_t *ccr;
    uint16_t period;

    // for input only
    uint8_t channel;
    uint8_t state;
    uint16_t rise;
    uint16_t fall;
    uint16_t capture;
} pwmPortData_t;

enum {
    TYPE_IP = 0x10,
    TYPE_IW = 0x20,
    TYPE_M = 0x40,
    TYPE_S = 0x80
};

typedef void (* pwmWriteFuncPtr)(uint8_t index, uint16_t value);  // function pointer used to write motors

static pwmPortData_t pwmPorts[MAX_PORTS];
static uint16_t captures[MAX_INPUTS];
static pwmPortData_t *motors[MAX_MOTORS];
static pwmPortData_t *servos[MAX_SERVOS];
static pwmWriteFuncPtr pwmWritePtr = NULL;
static uint8_t numMotors = 0;
static uint8_t numServos = 0;
static uint8_t  numInputs = 0;
static uint16_t failsafeThreshold = 985;

static const uint8_t multiPPM[] = {
    PWM1 | TYPE_IP,     // PPM input
    PWM9 | TYPE_M,      // Swap to servo if needed
    PWM10 | TYPE_M,     // Swap to servo if needed
    PWM11 | TYPE_M,
    PWM12 | TYPE_M,
    PWM13 | TYPE_M,
    PWM14 | TYPE_M,
    PWM5 | TYPE_M,      // Swap to servo if needed
    PWM6 | TYPE_M,      // Swap to servo if needed
    PWM7 | TYPE_M,      // Swap to servo if needed
    PWM8 | TYPE_M,      // Swap to servo if needed
    0xFF
};

static const uint8_t multiPWM[] = {
    PWM1 | TYPE_IW,     // input #1
    PWM2 | TYPE_IW,
    PWM3 | TYPE_IW,
    PWM4 | TYPE_IW,
    PWM5 | TYPE_IW,
    PWM6 | TYPE_IW,
    PWM7 | TYPE_IW,
    PWM8 | TYPE_IW,     // input #8
    PWM9 | TYPE_M,      // motor #1 or servo #1 (swap to servo if needed)
    PWM10 | TYPE_M,     // motor #2 or servo #2 (swap to servo if needed)
    PWM11 | TYPE_M,     // motor #1 or #3
    PWM12 | TYPE_M,
    PWM13 | TYPE_M,
    PWM14 | TYPE_M,     // motor #4 or #6
    0xFF
};

static const uint8_t airPPM[] = {
    PWM1 | TYPE_IP,     // PPM input
    PWM9 | TYPE_M,      // motor #1
    PWM10 | TYPE_M,     // motor #2
    PWM11 | TYPE_S,     // servo #1
    PWM12 | TYPE_S,
    PWM13 | TYPE_S,
    PWM14 | TYPE_S,     // servo #4
    PWM5 | TYPE_S,      // servo #5
    PWM6 | TYPE_S,
    PWM7 | TYPE_S,
    PWM8 | TYPE_S,      // servo #8
    0xFF
};

static const uint8_t airPWM[] = {
    PWM1 | TYPE_IW,     // input #1
    PWM2 | TYPE_IW,
    PWM3 | TYPE_IW,
    PWM4 | TYPE_IW,
    PWM5 | TYPE_IW,
    PWM6 | TYPE_IW,
    PWM7 | TYPE_IW,
    PWM8 | TYPE_IW,     // input #8
    PWM9 | TYPE_M,      // motor #1
    PWM10 | TYPE_M,     // motor #2
    PWM11 | TYPE_S,     // servo #1
    PWM12 | TYPE_S,
    PWM13 | TYPE_S,
    PWM14 | TYPE_S,     // servo #4
    0xFF
};

static const uint8_t * const hardwareMaps[] = {
    multiPWM,
    multiPPM,
    airPWM,
    airPPM,
};

#define PWM_TIMER_MHZ 1
#define PWM_BRUSHED_TIMER_MHZ 8

failsafe_t *failsafe;

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = value;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    switch (channel) {
        case TIM_Channel_1:
            TIM_OC1Init(tim, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(tim, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(tim, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(tim, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
    }
}

void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
    TIM_ICInitTypeDef  TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;

    TIM_ICInit(tim, &TIM_ICInitStructure);
}

static void pwmGPIOConfig(GPIO_TypeDef *gpio, uint32_t pin, GPIO_Mode mode)
{
    gpio_config_t cfg;

    cfg.pin = pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(gpio, &cfg);
}

static pwmPortData_t *pwmOutConfig(uint8_t port, uint8_t mhz, uint16_t period, uint16_t value)
{
    pwmPortData_t *p = &pwmPorts[port];
    configTimeBase(timerHardware[port].tim, period, mhz);
    pwmGPIOConfig(timerHardware[port].gpio, timerHardware[port].pin, Mode_AF_PP);
    pwmOCConfig(timerHardware[port].tim, timerHardware[port].channel, value);
    // Needed only on TIM1
    if (timerHardware[port].outputEnable)
        TIM_CtrlPWMOutputs(timerHardware[port].tim, ENABLE);
    TIM_Cmd(timerHardware[port].tim, ENABLE);

    switch (timerHardware[port].channel) {
        case TIM_Channel_1:
            p->ccr = &timerHardware[port].tim->CCR1;
            break;
        case TIM_Channel_2:
            p->ccr = &timerHardware[port].tim->CCR2;
            break;
        case TIM_Channel_3:
            p->ccr = &timerHardware[port].tim->CCR3;
            break;
        case TIM_Channel_4:
            p->ccr = &timerHardware[port].tim->CCR4;
            break;
    }
    p->period = period;
    return p;
}

static pwmPortData_t *pwmInConfig(uint8_t port, timerCCCallbackPtr callback, uint8_t channel)
{
    pwmPortData_t *p = &pwmPorts[port];
    const timerHardware_t *timerHardwarePtr = &(timerHardware[port]);

    p->channel = channel;

    pwmGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_IPD);
    pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Rising);

    timerConfigure(timerHardwarePtr, 0xFFFF, PWM_TIMER_MHZ);
    configureTimerCaptureCompareInterrupt(timerHardwarePtr, port, callback);

    return p;
}

static void ppmCallback(uint8_t port, uint16_t capture)
{
    uint16_t diff;
    static uint16_t now;
    static uint16_t last = 0;
    static uint8_t chan = 0;
    static uint8_t GoodPulses;

    last = now;
    now = capture;
    diff = now - last;

    if (diff > 2700) { // Per http://www.rcgroups.com/forums/showpost.php?p=21996147&postcount=3960 "So, if you use 2.5ms or higher as being the reset for the PPM stream start, you will be fine. I use 2.7ms just to be safe."
        chan = 0;
    } else {
        if (diff > 750 && diff < 2250 && chan < MAX_INPUTS) {   // 750 to 2250 ms is our 'valid' channel range
            captures[chan] = diff;
            if (chan < 4 && diff > failsafeThreshold)
                GoodPulses |= (1 << chan);      // if signal is valid - mark channel as OK
            if (GoodPulses == 0x0F) {           // If first four channels have good pulses, clear FailSafe counter
                GoodPulses = 0;
                failsafe->vTable->validDataReceived();
            }
        }
        chan++;
        failsafe->vTable->reset();
    }
}

static void pwmCallback(uint8_t port, uint16_t capture)
{
    if (pwmPorts[port].state == 0) {
        pwmPorts[port].rise = capture;
        pwmPorts[port].state = 1;
        pwmICConfig(timerHardware[port].tim, timerHardware[port].channel, TIM_ICPolarity_Falling);
    } else {
        pwmPorts[port].fall = capture;
        // compute capture
        pwmPorts[port].capture = pwmPorts[port].fall - pwmPorts[port].rise;
        captures[pwmPorts[port].channel] = pwmPorts[port].capture;
        // switch state
        pwmPorts[port].state = 0;
        pwmICConfig(timerHardware[port].tim, timerHardware[port].channel, TIM_ICPolarity_Rising);
        // reset failsafe
        failsafe->vTable->reset();
    }
}

static void pwmWriteBrushed(uint8_t index, uint16_t value)
{
    *motors[index]->ccr = (value - 1000) * motors[index]->period / 1000;
}

static void pwmWriteStandard(uint8_t index, uint16_t value)
{
    *motors[index]->ccr = value;
}

void pwmInit(drv_pwm_config_t *init, failsafe_t *initialFailsafe)
{
    int i = 0;
    const uint8_t *setup;

    failsafe = initialFailsafe;

    // to avoid importing cfg/mcfg
    failsafeThreshold = init->failsafeThreshold;

    // this is pretty hacky shit, but it will do for now. array of 4 config maps, [ multiPWM multiPPM airPWM airPPM ]
    if (init->airplane)
        i = 2; // switch to air hardware config
    if (init->usePPM)
        i++; // next index is for PPM

    setup = hardwareMaps[i];

    for (i = 0; i < MAX_PORTS; i++) {
        uint8_t port = setup[i] & 0x0F;
        uint8_t mask = setup[i] & 0xF0;

        if (setup[i] == 0xFF) // terminator
            break;

#ifdef OLIMEXINO_UNCUT_LED2_E_JUMPER
        // PWM2 is connected to LED2 on the board and cannot be connected unless you cut LED2_E
        if (port == PWM2)
            continue;
#endif

        // skip UART ports for GPS
        if (init->useUART && (port == PWM3 || port == PWM4))
            continue;

        // skip softSerial ports
        if (init->useSoftSerial && (port == PWM5 || port == PWM6 || port == PWM7 || port == PWM8))
            continue;

        // skip ADC for powerMeter if configured
        if (init->adcChannel && (init->adcChannel == port))
            continue;

        // hacks to allow current functionality
        if (mask & (TYPE_IP | TYPE_IW) && !init->enableInput)
            mask = 0;

        if (init->useServos && !init->airplane) {
            // remap PWM9+10 as servos (but not in airplane mode LOL)
            if (port == PWM9 || port == PWM10)
                mask = TYPE_S;
        }

        if (init->extraServos && !init->airplane) {
            // remap PWM5..8 as servos when used in extended servo mode
            if (port >= PWM5 && port <= PWM8)
                mask = TYPE_S;
        }

        if (mask & TYPE_IP) {
            pwmInConfig(port, ppmCallback, 0);
            numInputs = 8;
        } else if (mask & TYPE_IW) {
            pwmInConfig(port, pwmCallback, numInputs);
            numInputs++;
        } else if (mask & TYPE_M) {
            uint32_t hz, mhz;
            if (init->motorPwmRate > 500)
                mhz = PWM_BRUSHED_TIMER_MHZ;
            else
                mhz = PWM_TIMER_MHZ;
            hz = mhz * 1000000;

            motors[numMotors++] = pwmOutConfig(port, mhz, hz / init->motorPwmRate, init->idlePulse);
        } else if (mask & TYPE_S) {
            servos[numServos++] = pwmOutConfig(port, PWM_TIMER_MHZ, 1000000 / init->servoPwmRate, init->servoCenterPulse);
        }
    }

    // determine motor writer function
    pwmWritePtr = pwmWriteStandard;
    if (init->motorPwmRate > 500)
        pwmWritePtr = pwmWriteBrushed;
}

void pwmWriteMotor(uint8_t index, uint16_t value)
{
    if (index < numMotors)
        pwmWritePtr(index, value);
}

void pwmWriteServo(uint8_t index, uint16_t value)
{
    if (index < numServos)
        *servos[index]->ccr = value;
}

uint16_t pwmRead(uint8_t channel)
{
    return captures[channel];
}
