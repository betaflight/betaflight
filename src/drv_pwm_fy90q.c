#ifdef FY90Q
#include "board.h"

#define PULSE_1MS       (1000) // 1ms pulse width
// #define PULSE_PERIOD    (2500) // pulse period (400Hz)
// #define PULSE_PERIOD_SERVO_DIGITAL  (5000) // pulse period for digital servo (200Hz)
// #define PULSE_PERIOD_SERVO_ANALOG  (20000) // pulse period for analog servo (50Hz)

// Forward declaration
static void pwmIRQHandler(TIM_TypeDef *tim);
static void ppmIRQHandler(TIM_TypeDef *tim);

// external vars (ugh)
extern int16_t failsafeCnt;

// local vars
static struct TIM_Channel {
    TIM_TypeDef *tim;
    uint16_t channel;
    uint16_t cc;
} Channels[] = {
    { TIM2, TIM_Channel_1, TIM_IT_CC1 },
    { TIM2, TIM_Channel_2, TIM_IT_CC2 },
    { TIM2, TIM_Channel_3, TIM_IT_CC3 },
    { TIM2, TIM_Channel_4, TIM_IT_CC4 },
    { TIM3, TIM_Channel_1, TIM_IT_CC1 },
    { TIM3, TIM_Channel_2, TIM_IT_CC2 },
    { TIM3, TIM_Channel_3, TIM_IT_CC3 },
    { TIM3, TIM_Channel_4, TIM_IT_CC4 },
};

static volatile uint16_t *OutputChannels[] = {
    &(TIM4->CCR1),
    &(TIM4->CCR2),
    &(TIM4->CCR3),
    &(TIM4->CCR4),
    // Extended use during CPPM input (TODO)
    &(TIM3->CCR1),
    &(TIM3->CCR2),
    &(TIM3->CCR3),
    &(TIM3->CCR4),
};

static struct PWM_State {
    uint8_t state;
    uint16_t rise;
    uint16_t fall;
    uint16_t capture;
} Inputs[8] = { { 0, } };

static TIM_ICInitTypeDef TIM_ICInitStructure = { 0, };
static bool usePPMFlag = false;
static uint8_t numOutputChannels = 0;

void TIM2_IRQHandler(void)
{
    if (usePPMFlag)
        ppmIRQHandler(TIM2);
    else
        pwmIRQHandler(TIM2);
}

static void ppmIRQHandler(TIM_TypeDef *tim)
{
    uint16_t diff;
    static uint16_t now;
    static uint16_t last = 0;
    static uint8_t chan = 0;
    static uint8_t GoodPulses;

    if (TIM_GetITStatus(tim, TIM_IT_CC1) == SET) {
        last = now;
        now = TIM_GetCapture1(tim);
        rcActive = true;
    }

    TIM_ClearITPendingBit(tim, TIM_IT_CC1);

    if (now > last) {
        diff = (now - last);
    } else {
        diff = ((0xFFFF - last) + now);
    }

    if (diff > 4000) {
        chan = 0;
    } else {
        if (diff > PULSE_MIN && diff < PULSE_MAX && chan < 8) {   // 750 to 2250 ms is our 'valid' channel range
            Inputs[chan].capture = diff;
            if (chan < 4 && diff > FAILSAFE_DETECT_TRESHOLD)
                GoodPulses |= (1 << chan);      // if signal is valid - mark channel as OK
            if (GoodPulses == 0x0F) {   // If first four chanells have good pulses, clear FailSafe counter
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

static void pwmIRQHandler(TIM_TypeDef *tim)
{
    uint8_t i;
    uint16_t val = 0;

    for (i = 0; i < 8; i++) {
        struct TIM_Channel channel = Channels[i];
        struct PWM_State *state = &Inputs[i];

        if (channel.tim == tim && (TIM_GetITStatus(tim, channel.cc) == SET)) {
            TIM_ClearITPendingBit(channel.tim, channel.cc);

            switch (channel.channel) {
                case TIM_Channel_1:
                    val = TIM_GetCapture1(channel.tim);
                    break;
                case TIM_Channel_2:
                    val = TIM_GetCapture2(channel.tim);
                    break;
                case TIM_Channel_3:
                    val = TIM_GetCapture3(channel.tim);
                    break;
                case TIM_Channel_4:
                    val = TIM_GetCapture4(channel.tim);
                    break;
            }

            if (state->state == 0)
                state->rise = val;
            else
                state->fall = val;

            if (state->state == 0) {
                // switch states
                state->state = 1;

                TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
                TIM_ICInitStructure.TIM_Channel = channel.channel;
                TIM_ICInit(channel.tim, &TIM_ICInitStructure);
            } else {
                // compute capture
                if (state->fall > state->rise)
                    state->capture = (state->fall - state->rise);
                else
                    state->capture = ((0xffff - state->rise) + state->fall);

                // switch state
                state->state = 0;

                // ping failsafe
                failsafeCnt = 0;

                TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
                TIM_ICInitStructure.TIM_Channel = channel.channel;
                TIM_ICInit(channel.tim, &TIM_ICInitStructure);
            }
        }
    }
}

static void pwmInitializeInput(bool usePPM)
{
    GPIO_InitTypeDef GPIO_InitStructure = { 0, };
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = { 0, };
    NVIC_InitTypeDef NVIC_InitStructure = { 0, };
    uint8_t i;

    // Input pins
    if (usePPM) {
        // Configure TIM2_CH1 for PPM input
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        // Input timer on TIM2 only for PPM
        NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        // TIM2 timebase
        TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
        TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);
        TIM_TimeBaseStructure.TIM_Period = 0xffff;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

        // Input capture on TIM2_CH1 for PPM
        TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
        TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
        TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
        TIM_ICInitStructure.TIM_ICFilter = 0x0;
        TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
        TIM_ICInit(TIM2, &TIM_ICInitStructure);

        // TIM2_CH1 capture compare interrupt enable
        TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
        TIM_Cmd(TIM2, ENABLE);

        // configure number of PWM outputs, in PPM mode, we use bottom 4 channels more more motors
        numOutputChannels = 10;
    } else {
        // Configure TIM2 all 4 channels
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        // TODO Configure EXTI4 1 channel

        // Input timers on TIM2 for PWM
        NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        // TIM2 timebase
        TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
        TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);
        TIM_TimeBaseStructure.TIM_Period = 0xffff;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

        // PWM Input capture
        TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
        TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
        TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
        TIM_ICInitStructure.TIM_ICFilter = 0x0;

        for (i = 0; i < 4; i++) {
            TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
            TIM_ICInit(Channels[i].tim, &TIM_ICInitStructure);
        }

        // TODO EXTI4

        TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
        // TODO EXTI4
        TIM_Cmd(TIM2, ENABLE);

        // In PWM input mode, all 4 channels are wasted
        numOutputChannels = 4;
    }
}

bool pwmInit(drv_pwm_config_t *init)
{
    GPIO_InitTypeDef GPIO_InitStructure = { 0, };
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = { 0, };
    TIM_OCInitTypeDef TIM_OCInitStructure = { 0, };

    uint8_t i;

    // Inputs

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
    // PWM3 TIM4_CH1 PB6 [also I2C1_SCL]
    // PWM4 TIM4_CH2 PB7 [also I2C1_SDA]
    // PWM5 TIM4_CH3 PB8
    // PWM6 TIM4_CH4 PB9

    // use PPM or PWM input
    usePPMFlag = init->usePPM;

    // preset channels to center
    for (i = 0; i < 8; i++)
        Inputs[i].capture = 1500;

    // Timers run at 1mhz.
    // TODO: clean this shit up. Make it all dynamic etc.
    if (init->enableInput)
        pwmInitializeInput(usePPMFlag);

    // Output pins (4x)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Output timer
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);

    TIM_TimeBaseStructure.TIM_Period = (1000000 / init->motorPwmRate) - 1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = PULSE_1MS;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    // PWM1,2,3,4
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_Cmd(TIM4, ENABLE);
    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_OC1PreloadConfig (TIM4, TIM_OCPreload_Enable);

    // turn on more motor outputs if we're using ppm / not using pwm input
    if (!init->enableInput || init->usePPM) {
        // TODO
    }

    return false;
}

void pwmWrite(uint8_t channel, uint16_t value)
{
    if (channel < numOutputChannels)
        *OutputChannels[channel] = value;
}

uint16_t pwmRead(uint8_t channel)
{
    return Inputs[channel].capture;
}
#endif
