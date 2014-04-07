#include "board.h"

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

    RX1  TIM2_CH1 PA0 [also PPM] [also used for throttle calibration]
    RX2  TIM2_CH2 PA1
    RX3  TIM2_CH3 PA2 [also UART2_TX]
    RX4  TIM2_CH4 PA3 [also UART2_RX]
    RX5  TIM3_CH1 PA6 [also ADC_IN6]
    RX6  TIM3_CH2 PA7 [also ADC_IN7]
    RX7  TIM3_CH3 PB0 [also ADC_IN8]
    RX8  TIM3_CH4 PB1 [also ADC_IN9]

    Outputs
    PWM1 TIM1_CH1 PA8
    PWM2 TIM1_CH4 PA11
    PWM3 TIM4_CH1 PB6? [also I2C1_SCL]
    PWM4 TIM4_CH2 PB7 [also I2C1_SDA]
    PWM5 TIM4_CH3 PB8
    PWM6 TIM4_CH4 PB9

    Groups that allow running different period (ex 50Hz servos + 400Hz throttle + etc):
    TIM2 4 channels
    TIM3 4 channels
    TIM1 2 channels
    TIM4 4 channels
*/

const timerHardware_t timerHardware[] = {
    { TIM2, GPIOA, Pin_0, TIM_Channel_1, TIM2_IRQn, 0, },          // PWM1
    { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 0, },          // PWM2
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 0, },          // PWM3
    { TIM2, GPIOA, Pin_3, TIM_Channel_4, TIM2_IRQn, 0, },          // PWM4
    { TIM3, GPIOA, Pin_6, TIM_Channel_1, TIM3_IRQn, 0, },          // PWM5
    { TIM3, GPIOA, Pin_7, TIM_Channel_2, TIM3_IRQn, 0, },          // PWM6
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 0, },          // PWM7
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 0, },          // PWM8
    { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, },       // PWM9
    { TIM1, GPIOA, Pin_11, TIM_Channel_4, TIM1_CC_IRQn, 1, },      // PWM10
    { TIM4, GPIOB, Pin_6, TIM_Channel_1, TIM4_IRQn, 0, },          // PWM11
    { TIM4, GPIOB, Pin_7, TIM_Channel_2, TIM4_IRQn, 0, },          // PWM12
    { TIM4, GPIOB, Pin_8, TIM_Channel_3, TIM4_IRQn, 0, },          // PWM13
    { TIM4, GPIOB, Pin_9, TIM_Channel_4, TIM4_IRQn, 0, },          // PWM14
};

#define MAX_TIMERS 4 // TIM1..TIM4
#define CC_CHANNELS_PER_TIMER 4 // TIM_Channel_1..4

static const TIM_TypeDef *timers[MAX_TIMERS] = {
    TIM1, TIM2, TIM3, TIM4
};

static const uint8_t channels[CC_CHANNELS_PER_TIMER] = {
    TIM_Channel_1, TIM_Channel_2, TIM_Channel_3, TIM_Channel_4
};

typedef struct timerConfig_s {
    TIM_TypeDef *tim;
    uint8_t channel;
    timerCCCallbackPtr *callback;
    uint8_t reference;
} timerConfig_t;

timerConfig_t timerConfig[MAX_TIMERS * CC_CHANNELS_PER_TIMER];

static uint8_t lookupTimerIndex(const TIM_TypeDef *tim)
{
    uint8_t timerIndex = 0;
    while (timers[timerIndex] != tim) {
        timerIndex++;
    }
    return timerIndex;
}

static uint8_t lookupChannelIndex(const uint8_t channel)
{
    uint8_t channelIndex = 0;
    while (channels[channelIndex] != channel) {
        channelIndex++;
    }
    return channelIndex;
}

void configureTimerChannelCallback(TIM_TypeDef *tim, uint8_t channel, uint8_t reference, timerCCCallbackPtr *callback)
{
    assert_param(IS_TIM_CHANNEL(channel));

    uint8_t timerConfigIndex = (lookupTimerIndex(tim) * MAX_TIMERS) + lookupChannelIndex(channel);

    if (timerConfigIndex >= MAX_TIMERS * CC_CHANNELS_PER_TIMER) {
        return;
    }

    timerConfig[timerConfigIndex].callback = callback;
    timerConfig[timerConfigIndex].channel = channel;
    timerConfig[timerConfigIndex].reference = reference;
}

void configureTimerInputCaptureCompareChannel(TIM_TypeDef *tim, const uint8_t channel)
{
    switch (channel) {
        case TIM_Channel_1:
            TIM_ITConfig(tim, TIM_IT_CC1, ENABLE);
            break;
        case TIM_Channel_2:
            TIM_ITConfig(tim, TIM_IT_CC2, ENABLE);
            break;
        case TIM_Channel_3:
            TIM_ITConfig(tim, TIM_IT_CC3, ENABLE);
            break;
        case TIM_Channel_4:
            TIM_ITConfig(tim, TIM_IT_CC4, ENABLE);
            break;
    }
}

void configureTimerCaptureCompareInterrupt(const timerHardware_t *timerHardwarePtr, uint8_t reference, timerCCCallbackPtr *callback)
{
    configureTimerChannelCallback(timerHardwarePtr->tim, timerHardwarePtr->channel, reference, callback);
    configureTimerInputCaptureCompareChannel(timerHardwarePtr->tim, timerHardwarePtr->channel);
}

void timerNVICConfigure(uint8_t irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = period - 1; // AKA TIMx_ARR

    // "The counter clock frequency (CK_CNT) is equal to f CK_PSC / (PSC[15:0] + 1)." - STM32F10x Reference Manual 14.4.11
    // Thus for 1Mhz: 72000000 / 1000000 = 72, 72 - 1 = 71 = TIM_Prescaler
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / ((uint32_t)mhz * 1000000)) - 1;


    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz)
{
    configTimeBase(timerHardwarePtr->tim, period, mhz);
    TIM_Cmd(timerHardwarePtr->tim, ENABLE);
    timerNVICConfigure(timerHardwarePtr->irq);
}


timerConfig_t *findTimerConfig(TIM_TypeDef *tim, uint8_t channel)
{
    uint8_t timerConfigIndex = (lookupTimerIndex(tim) * MAX_TIMERS) + lookupChannelIndex(channel);
    return &(timerConfig[timerConfigIndex]);
}

static void timCCxHandler(TIM_TypeDef *tim)
{
    uint16_t capture;
    timerConfig_t *timerConfig;

    uint8_t channelIndex = 0;
    for (channelIndex = 0; channelIndex < CC_CHANNELS_PER_TIMER; channelIndex++) {
        uint8_t channel = channels[channelIndex];

        if (channel == TIM_Channel_1 && TIM_GetITStatus(tim, TIM_IT_CC1) == SET) {
            TIM_ClearITPendingBit(tim, TIM_IT_CC1);

            timerConfig = findTimerConfig(tim, TIM_Channel_1);
            capture = TIM_GetCapture1(tim);
        } else if (channel == TIM_Channel_2 && TIM_GetITStatus(tim, TIM_IT_CC2) == SET) {
            TIM_ClearITPendingBit(tim, TIM_IT_CC2);

            timerConfig = findTimerConfig(tim, TIM_Channel_2);
            capture = TIM_GetCapture2(tim);
        } else if (channel == TIM_Channel_3 && TIM_GetITStatus(tim, TIM_IT_CC3) == SET) {
            TIM_ClearITPendingBit(tim, TIM_IT_CC3);

            timerConfig = findTimerConfig(tim, TIM_Channel_3);
            capture = TIM_GetCapture3(tim);
        } else if (channel == TIM_Channel_4 && TIM_GetITStatus(tim, TIM_IT_CC4) == SET) {
            TIM_ClearITPendingBit(tim, TIM_IT_CC4);

            timerConfig = findTimerConfig(tim, TIM_Channel_4);
            capture = TIM_GetCapture4(tim);
        } else {
            continue; // avoid uninitialised variable dereference
        }

        if (!timerConfig->callback) {
            continue;
        }
        timerConfig->callback(timerConfig->reference, capture);
    }
}
void TIM1_CC_IRQHandler(void)
{
    timCCxHandler(TIM1);
}

void TIM2_IRQHandler(void)
{
    timCCxHandler(TIM2);
}

void TIM3_IRQHandler(void)
{
    timCCxHandler(TIM3);
}

void TIM4_IRQHandler(void)
{
    timCCxHandler(TIM4);
}
