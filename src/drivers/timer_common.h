#pragma once

#ifdef CHEBUZZF3
#define USABLE_TIMER_CHANNEL_COUNT 18
#else
#define USABLE_TIMER_CHANNEL_COUNT 14
#endif

#ifdef STM32F303xC
typedef uint32_t captureCompare_t;
#endif
#ifdef STM32F10X_MD
typedef uint32_t captureCompare_t;
#endif

typedef void timerCCCallbackPtr(uint8_t port, captureCompare_t capture);

typedef struct {
    TIM_TypeDef *tim;
    GPIO_TypeDef *gpio;
    uint32_t pin;
    uint8_t channel;
    uint8_t irq;
    uint8_t outputEnable;
    GPIO_Mode gpioInputMode;
} timerHardware_t;

extern const timerHardware_t timerHardware[];

void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz);
void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz);
void timerNVICConfigure(uint8_t irq);

void configureTimerInputCaptureCompareChannel(TIM_TypeDef *tim, const uint8_t channel);
void configureTimerCaptureCompareInterrupt(const timerHardware_t *timerHardwarePtr, uint8_t reference, timerCCCallbackPtr *callback);
void configureTimerChannelCallback(TIM_TypeDef *tim, uint8_t channel, uint8_t reference, timerCCCallbackPtr *callback);
