#include "board.h"

#define PULSE_1MS       (1000) // 1ms pulse width

/* FreeFlight/Naze32 timer layout
    TIM2_CH1    RC1             PWM1
    TIM2_CH2    RC2             PWM2
    TIM2_CH3    RC3/UA2_TX      PWM3
    TIM2_CH4    RC4/UA2_RX      PWM4
    TIM3_CH1    RC5             PWM5
    TIM3_CH2    RC6             PWM6
    TIM3_CH3    RC7             PWM7
    TIM3_CH4    RC8             PWM8
    TIM1_CH1    PWM1            PWM9
    TIM1_CH4    PWM2            PWM10
    TIM4_CH1    PWM3            PWM11
    TIM4_CH2    PWM4            PWM12
    TIM4_CH3    PWM5            PWM13
    TIM4_CH4    PWM6            PWM14

    // RX1  TIM2_CH1 PA0 [also PPM] [also used for throttle calibration]
    // RX2  TIM2_CH2 PA1
    // RX3  TIM2_CH3 PA2 [also UART2_TX]
    // RX4  TIM2_CH4 PA3 [also UART2_RX]
    // RX5  TIM3_CH1 PA6 [also ADC_IN6]
    // RX6  TIM3_CH2 PA7 [also ADC_IN7]
    // RX7  TIM3_CH3 PB0 [also ADC_IN8]
    // RX8  TIM3_CH4 PB1 [also ADC_IN9]

    // Outputs
    // PWM1 TIM1_CH1 PA8
    // PWM2 TIM1_CH4 PA11
    // PWM3 TIM4_CH1 PB6? [also I2C1_SCL]
    // PWM4 TIM4_CH2 PB7 [also I2C1_SDA]
    // PWM5 TIM4_CH3 PB8
    // PWM6 TIM4_CH4 PB9

    Groups that allow running different period (ex 50Hz servos + 400Hz throttle + etc):
    TIM2 4 channels
    TIM3 4 channels
    TIM1 2 channels
    TIM4 4 channels

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

typedef void pwmCallbackPtr(uint8_t port, uint16_t capture);

static pwmHardware_t timerHardware[] = {
    { TIM2, GPIOA, GPIO_Pin_0, TIM_Channel_1, TIM2_IRQn, 0, },          // PWM1
    { TIM2, GPIOA, GPIO_Pin_1, TIM_Channel_2, TIM2_IRQn, 0, },          // PWM2
    { TIM2, GPIOA, GPIO_Pin_2, TIM_Channel_3, TIM2_IRQn, 0, },          // PWM3
    { TIM2, GPIOA, GPIO_Pin_3, TIM_Channel_4, TIM2_IRQn, 0, },          // PWM4
    { TIM3, GPIOA, GPIO_Pin_6, TIM_Channel_1, TIM3_IRQn, 0, },          // PWM5
    { TIM3, GPIOA, GPIO_Pin_7, TIM_Channel_2, TIM3_IRQn, 0, },          // PWM6
    { TIM3, GPIOB, GPIO_Pin_0, TIM_Channel_3, TIM3_IRQn, 0, },          // PWM7
    { TIM3, GPIOB, GPIO_Pin_1, TIM_Channel_4, TIM3_IRQn, 0, },          // PWM8
    { TIM1, GPIOA, GPIO_Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, },       // PWM9
    { TIM1, GPIOA, GPIO_Pin_11, TIM_Channel_4, TIM1_CC_IRQn, 1, },      // PWM10
    { TIM4, GPIOB, GPIO_Pin_6, TIM_Channel_1, TIM4_IRQn, 0, },          // PWM11
    { TIM4, GPIOB, GPIO_Pin_7, TIM_Channel_2, TIM4_IRQn, 0, },          // PWM12
    { TIM4, GPIOB, GPIO_Pin_8, TIM_Channel_3, TIM4_IRQn, 0, },          // PWM13
    { TIM4, GPIOB, GPIO_Pin_9, TIM_Channel_4, TIM4_IRQn, 0, },          // PWM14
};

typedef struct {
    pwmCallbackPtr *callback;
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

static pwmPortData_t pwmPorts[MAX_PORTS];
static uint16_t captures[MAX_INPUTS];
static pwmPortData_t *motors[MAX_MOTORS];
static pwmPortData_t *servos[MAX_SERVOS];
static uint8_t numMotors = 0;
static uint8_t numServos = 0;
static uint8_t  numInputs = 0;
static uint16_t failsafeThreshold = 985;
// external vars (ugh)
extern int16_t failsafeCnt;

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

static const uint8_t *hardwareMaps[] = {
    multiPWM,
    multiPPM,
    airPWM,
    airPPM,
};

static void pwmTimeBase(TIM_TypeDef *tim, uint32_t period)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = period - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1; // all timers run at 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

static void pwmNVICConfig(uint8_t irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

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

static void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
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

static void pwmGPIOConfig(GPIO_TypeDef *gpio, uint32_t pin, uint8_t input)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pin;
    if (input)
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    else
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(gpio, &GPIO_InitStructure);
}

static pwmPortData_t *pwmOutConfig(uint8_t port, uint16_t period, uint16_t value)
{
    pwmPortData_t *p = &pwmPorts[port];
    pwmTimeBase(timerHardware[port].tim, period);
    pwmGPIOConfig(timerHardware[port].gpio, timerHardware[port].pin, 0);
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
    return p;
}

static pwmPortData_t *pwmInConfig(uint8_t port, pwmCallbackPtr callback, uint8_t channel)
{
    pwmPortData_t *p = &pwmPorts[port];
    pwmTimeBase(timerHardware[port].tim, 0xFFFF);
    pwmGPIOConfig(timerHardware[port].gpio, timerHardware[port].pin, 1);
    pwmICConfig(timerHardware[port].tim, timerHardware[port].channel, TIM_ICPolarity_Rising);
    TIM_Cmd(timerHardware[port].tim, ENABLE);
    pwmNVICConfig(timerHardware[port].irq);
    // set callback before configuring interrupts
    p->callback = callback;
    p->channel = channel;

    switch (timerHardware[port].channel) {
        case TIM_Channel_1:
            TIM_ITConfig(timerHardware[port].tim, TIM_IT_CC1, ENABLE);
            break;
        case TIM_Channel_2:
            TIM_ITConfig(timerHardware[port].tim, TIM_IT_CC2, ENABLE);
            break;
        case TIM_Channel_3:
            TIM_ITConfig(timerHardware[port].tim, TIM_IT_CC3, ENABLE);
            break;
        case TIM_Channel_4:
            TIM_ITConfig(timerHardware[port].tim, TIM_IT_CC4, ENABLE);
            break;
    }
    return p;
}

void TIM1_CC_IRQHandler(void)
{
    uint8_t port;

    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET) {
        port = PWM9;
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
        pwmPorts[port].callback(port, TIM_GetCapture1(TIM1));
    } else if (TIM_GetITStatus(TIM1, TIM_IT_CC4) == SET) {
        port = PWM10;
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
        pwmPorts[port].callback(port, TIM_GetCapture4(TIM1));
    }
}

static void pwmTIMxHandler(TIM_TypeDef *tim, uint8_t portBase)
{
    int8_t port;

    // Generic CC handler for TIM2,3,4
    if (TIM_GetITStatus(tim, TIM_IT_CC1) == SET) {
        port = portBase + 0;
        TIM_ClearITPendingBit(tim, TIM_IT_CC1);
        pwmPorts[port].callback(port, TIM_GetCapture1(tim));
    } else if (TIM_GetITStatus(tim, TIM_IT_CC2) == SET) {
        port = portBase + 1;
        TIM_ClearITPendingBit(tim, TIM_IT_CC2);
        pwmPorts[port].callback(port, TIM_GetCapture2(tim));
    } else if (TIM_GetITStatus(tim, TIM_IT_CC3) == SET) {
        port = portBase + 2;
        TIM_ClearITPendingBit(tim, TIM_IT_CC3);
        pwmPorts[port].callback(port, TIM_GetCapture3(tim));
    } else if (TIM_GetITStatus(tim, TIM_IT_CC4) == SET) {
        port = portBase + 3;
        TIM_ClearITPendingBit(tim, TIM_IT_CC4);
        pwmPorts[port].callback(port, TIM_GetCapture4(tim));
    }
}

void TIM2_IRQHandler(void)
{
    pwmTIMxHandler(TIM2, PWM1); // PWM1..4
}

void TIM3_IRQHandler(void)
{
    pwmTIMxHandler(TIM3, PWM5); // PWM5..8
}

void TIM4_IRQHandler(void)
{
    pwmTIMxHandler(TIM4, PWM11); // PWM11..14
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
        if (diff > 750 && diff < 2250 && chan < 8) {   // 750 to 2250 ms is our 'valid' channel range
            captures[chan] = diff;
            if (chan < 4 && diff > failsafeThreshold)
                GoodPulses |= (1 << chan);      // if signal is valid - mark channel as OK
            if (GoodPulses == 0x0F) {           // If first four chanells have good pulses, clear FailSafe counter
                GoodPulses = 0;
                if (failsafeCnt > 20)
                    failsafeCnt -= 20;
                else
                    failsafeCnt = 0;
            }
        }
        chan++;
        failsafeCnt = 0;
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
        failsafeCnt = 0;
    }
}

bool pwmInit(drv_pwm_config_t *init)
{
    int i = 0;
    const uint8_t *setup;

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

#ifdef OLIMEXINO
        // PWM2 is connected to LED2 on the board and cannot be connected.
        if (port == PWM2)
            continue;
#endif

        // skip UART ports for GPS
        if (init->useUART && (port == PWM3 || port == PWM4))
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
            motors[numMotors++] = pwmOutConfig(port, 1000000 / init->motorPwmRate, PULSE_1MS);
        } else if (mask & TYPE_S) {
            servos[numServos++] = pwmOutConfig(port, 1000000 / init->servoPwmRate, PULSE_1MS);
        }
    }

    return false;
}

void pwmWriteMotor(uint8_t index, uint16_t value)
{
    if (index < numMotors)
        *motors[index]->ccr = value;
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
