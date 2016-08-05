/*
  modified version of StdPeriph function is located here.
  TODO - what license does apply here?
  original file was lincesed under MCD-ST Liberty SW License Agreement V2
  http://www.st.com/software_license_agreement_liberty_v2
*/

#include "stm32f30x.h"
#include "rcc.h"
#include "timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1),  GPIO_AF_6  },
    { .TIMx = TIM2,  .rcc = RCC_APB1(TIM2),  GPIO_AF_1  },
    { .TIMx = TIM3,  .rcc = RCC_APB1(TIM3),  GPIO_AF_2  },
    { .TIMx = TIM4,  .rcc = RCC_APB1(TIM4),  GPIO_AF_10 },
    { .TIMx = TIM6,  .rcc = RCC_APB1(TIM6),  0          },
    { .TIMx = TIM7,  .rcc = RCC_APB1(TIM7),  0          },
    { .TIMx = TIM8,  .rcc = RCC_APB2(TIM8),  GPIO_AF_5  },
    { .TIMx = TIM15, .rcc = RCC_APB2(TIM15), GPIO_AF_9  },
    { .TIMx = TIM16, .rcc = RCC_APB2(TIM16), GPIO_AF_1  },
    { .TIMx = TIM17, .rcc = RCC_APB2(TIM17), GPIO_AF_1  },
};


/**
  * @brief  Selects the TIM Output Compare Mode.
  * @note   This function does NOT disable the selected channel before changing the Output
  *         Compare Mode. If needed, user has to enable this channel using
  *         TIM_CCxCmd() and TIM_CCxNCmd() functions.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_Channel: specifies the TIM Channel
  *          This parameter can be one of the following values:
  *            @arg TIM_Channel_1: TIM Channel 1
  *            @arg TIM_Channel_2: TIM Channel 2
  *            @arg TIM_Channel_3: TIM Channel 3
  *            @arg TIM_Channel_4: TIM Channel 4
  * @param  TIM_OCMode: specifies the TIM Output Compare Mode.
  *           This parameter can be one of the following values:
  *            @arg TIM_OCMode_Timing
  *            @arg TIM_OCMode_Active
  *            @arg TIM_OCMode_Toggle
  *            @arg TIM_OCMode_PWM1
  *            @arg TIM_OCMode_PWM2
  *            @arg TIM_ForcedAction_Active
  *            @arg TIM_ForcedAction_InActive
  *            @arg TIM_OCMode_Retrigerrable_OPM1
  *            @arg TIM_OCMode_Retrigerrable_OPM2
  *            @arg TIM_OCMode_Combined_PWM1
  *            @arg TIM_OCMode_Combined_PWM2
  *            @arg TIM_OCMode_Asymmetric_PWM1
  *            @arg TIM_OCMode_Asymmetric_PWM2
  * @retval None
  */
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
