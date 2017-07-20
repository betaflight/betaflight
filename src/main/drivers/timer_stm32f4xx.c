/*
  modified version of StdPeriph function is located here.
  TODO - what license does apply here?
  original file was lincesed under MCD-ST Liberty SW License Agreement V2
  http://www.st.com/software_license_agreement_liberty_v2
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "stm32f4xx.h"
#include "rcc.h"
#include "timer.h"

#define CCMR_Offset                 ((uint16_t)0x0018)

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1),  .irq = TIM1_CC_IRQn},
    { .TIMx = TIM2,  .rcc = RCC_APB1(TIM2),  .irq = TIM2_IRQn},
    { .TIMx = TIM3,  .rcc = RCC_APB1(TIM3),  .irq = TIM3_IRQn},
    { .TIMx = TIM4,  .rcc = RCC_APB1(TIM4),  .irq = TIM4_IRQn},
    { .TIMx = TIM5,  .rcc = RCC_APB1(TIM5),  .irq = TIM5_IRQn},
    { .TIMx = TIM6,  .rcc = RCC_APB1(TIM6),  .irq = 0},
    { .TIMx = TIM7,  .rcc = RCC_APB1(TIM7),  .irq = 0},
#if !defined(STM32F411xE) && !defined(STM32F446xx)
    { .TIMx = TIM8,  .rcc = RCC_APB2(TIM8),  .irq = TIM8_CC_IRQn},
#endif
    { .TIMx = TIM9,  .rcc = RCC_APB2(TIM9),  .irq = TIM1_BRK_TIM9_IRQn},
    { .TIMx = TIM10, .rcc = RCC_APB2(TIM10), .irq = TIM1_UP_TIM10_IRQn},
    { .TIMx = TIM11, .rcc = RCC_APB2(TIM11), .irq = TIM1_TRG_COM_TIM11_IRQn},
#ifndef STM32F411xE
    { .TIMx = TIM12, .rcc = RCC_APB1(TIM12), .irq = TIM8_BRK_TIM12_IRQn},
    { .TIMx = TIM13, .rcc = RCC_APB1(TIM13), .irq = TIM8_UP_TIM13_IRQn},
    { .TIMx = TIM14, .rcc = RCC_APB1(TIM14), .irq = TIM8_TRG_COM_TIM14_IRQn},
#endif
};

void TIM_SelectOCxM_NoDisable(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode)
{
    uint32_t tmp = 0;

    /* Check the parameters */
    assert_param(IS_TIM_LIST8_PERIPH(TIMx));
    assert_param(IS_TIM_CHANNEL(TIM_Channel));
    assert_param(IS_TIM_OCM(TIM_OCMode));

    tmp = (uint32_t) TIMx;
    tmp += CCMR_Offset;

    if ((TIM_Channel == TIM_Channel_1) ||(TIM_Channel == TIM_Channel_3)) {
        tmp += (TIM_Channel>>1);

        /* Reset the OCxM bits in the CCMRx register */
        *(__IO uint32_t *) tmp &= (uint32_t)~((uint32_t)TIM_CCMR1_OC1M);

        /* Configure the OCxM bits in the CCMRx register */
        *(__IO uint32_t *) tmp |= TIM_OCMode;
    } else {
        tmp += (uint16_t)(TIM_Channel - (uint16_t)4)>> (uint16_t)1;

        /* Reset the OCxM bits in the CCMRx register */
        *(__IO uint32_t *) tmp &= (uint32_t)~((uint32_t)TIM_CCMR1_OC2M);

        /* Configure the OCxM bits in the CCMRx register */
        *(__IO uint32_t *) tmp |= (uint16_t)(TIM_OCMode << 8);
    }
}

uint8_t timerClockDivisor(TIM_TypeDef *tim)
{
#if defined (STM32F40_41xxx)
    if (tim == TIM8) return 1;
#endif
    if (tim == TIM1 || tim == TIM9 || tim == TIM10 || tim == TIM11) {
        return 1;
    } else {
        return 2;
    }
}
