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

#include "stm32f30x.h"
#include "rcc.h"
#include "timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1),  .irq = TIM1_CC_IRQn },
    { .TIMx = TIM2,  .rcc = RCC_APB1(TIM2),  .irq = TIM2_IRQn },
    { .TIMx = TIM3,  .rcc = RCC_APB1(TIM3),  .irq = TIM3_IRQn },
    { .TIMx = TIM4,  .rcc = RCC_APB1(TIM4),  .irq = TIM4_IRQn },
    { .TIMx = TIM6,  .rcc = RCC_APB1(TIM6),  .irq = 0 },
    { .TIMx = TIM7,  .rcc = RCC_APB1(TIM7),  .irq = 0 },
    { .TIMx = TIM8,  .rcc = RCC_APB2(TIM8),  .irq = TIM8_CC_IRQn },
    { .TIMx = TIM15, .rcc = RCC_APB2(TIM15), .irq = TIM1_BRK_TIM15_IRQn },
    { .TIMx = TIM16, .rcc = RCC_APB2(TIM16), .irq = TIM1_UP_TIM16_IRQn },
    { .TIMx = TIM17, .rcc = RCC_APB2(TIM17), .irq = TIM1_TRG_COM_TIM17_IRQn },
};

#define CCMR_OFFSET        ((uint16_t)0x0018)
#define CCMR_OC13M_MASK    ((uint32_t)0xFFFEFF8F)
#define CCMR_OC24M_MASK    ((uint32_t)0xFEFF8FFF)

void TIM_SelectOCxM_NoDisable(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint32_t TIM_OCMode)
{
    uint32_t tmp = 0;

    /* Check the parameters */
    assert_param(IS_TIM_LIST1_PERIPH(TIMx));
    assert_param(IS_TIM_CHANNEL(TIM_Channel));
    assert_param(IS_TIM_OCM(TIM_OCMode));

    tmp = (uint32_t) TIMx;
    tmp += CCMR_OFFSET;

    if ((TIM_Channel == TIM_Channel_1) || (TIM_Channel == TIM_Channel_3)) {
        tmp += (TIM_Channel>>1);

        /* Reset the OCxM bits in the CCMRx register */
        *(__IO uint32_t *) tmp &= CCMR_OC13M_MASK;

        /* Configure the OCxM bits in the CCMRx register */
        *(__IO uint32_t *) tmp |= TIM_OCMode;
    } else {
        tmp += (uint32_t)(TIM_Channel - (uint32_t)4) >> (uint32_t)1;

        /* Reset the OCxM bits in the CCMRx register */
        *(__IO uint32_t *) tmp &= CCMR_OC24M_MASK;

        /* Configure the OCxM bits in the CCMRx register */
        *(__IO uint32_t *) tmp |= (uint32_t)(TIM_OCMode << 8);
    }
}

uint8_t timerClockDivisor(TIM_TypeDef *tim)
{
    UNUSED(tim);
    return 1;
}
