#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "gpio_common.h"
#include "timer_common.h"

#include "pwm_mapping.h"

#include "pwm_rssi.h"

void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity); // from pwm_output.c

typedef struct rssiInputPort_s {
    uint8_t state;
    captureCompare_t rise;
    captureCompare_t fall;
    captureCompare_t capture;

    uint16_t raw;

    const timerHardware_t *timerHardware;
} rssiInputPort_t;

static rssiInputPort_t rssiInputPort;

static void pwmRssiCallback(uint8_t reference, captureCompare_t capture)
{
    const timerHardware_t *timerHardware = rssiInputPort.timerHardware;

    if (rssiInputPort.state == 0) {
        rssiInputPort.rise = capture;
        rssiInputPort.state = 1;
        pwmICConfig(timerHardware->tim, timerHardware->channel, TIM_ICPolarity_Falling);
    } else {
        rssiInputPort.fall = capture;

        // compute and store capture
        rssiInputPort.raw = rssiInputPort.fall - rssiInputPort.rise;

        // switch state
        rssiInputPort.state = 0;
        pwmICConfig(timerHardware->tim, timerHardware->channel, TIM_ICPolarity_Rising);
    }
}

static void pwmRSSIGPIOConfig(GPIO_TypeDef *gpio, uint32_t pin, GPIO_Mode mode)
{
    gpio_config_t cfg;

    cfg.pin = pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(gpio, &cfg);
}

void pwmRSSIICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;

    TIM_ICInit(tim, &TIM_ICInitStructure);
}

#define UNUSED_CALLBACK_REFERENCE 0

void pwmRSSIInConfig(uint8_t timerIndex)
{
    const timerHardware_t *timerHardwarePtr = &(timerHardware[timerIndex]);

    memset(&rssiInputPort, 0, sizeof(rssiInputPort));

    rssiInputPort.timerHardware = timerHardwarePtr;

    pwmRSSIGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, timerHardwarePtr->gpioInputMode);
    pwmRSSIICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Rising);

    timerConfigure(timerHardwarePtr, 0xFFFF, 1);
    configureTimerCaptureCompareInterrupt(timerHardwarePtr, UNUSED_CALLBACK_REFERENCE, pwmRssiCallback);
}

uint16_t pwmRSSIRead(void)
{
    return rssiInputPort.raw;
}

