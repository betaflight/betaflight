/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"
#include "drivers/timer.h"
#include "timer_hw_ext.h"
#include "hpm_clock_drv.h"
#include "hpm_soc.h"
#include "hpm_iomux.h"

typedef struct timerDef_s {
    TIM_TypeDef *TIMx;
    rccPeriphTag_t rcc;
    uint8_t inputIrq;
} timerDef_t;
typedef uint32_t timCCR_t;
typedef uint32_t timCCER_t;
typedef uint32_t timSR_t;
typedef uint32_t timCNT_t;

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    {.TIMx = HPM_PWM0, .rcc = clock_mot0, .inputIrq = IRQn_PWM0 },
    {.TIMx = HPM_PWM1, .rcc = clock_mot1, .inputIrq = IRQn_PWM1 },
    {.TIMx = HPM_PWM2, .rcc = clock_mot2, .inputIrq = IRQn_PWM2 },
    {.TIMx = HPM_PWM3, .rcc = clock_mot3, .inputIrq = IRQn_PWM3 },
};

const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {
    {
        .tim = (timerResource_t *) HPM_PWM1,
        .tag = TIMER_GET_IO_TAG(PB7),
        .channel = DEF_TIM_CHANNEL(CH_CH8),
        .output = DEF_TIM_OUTPUT(CH_CH4) | 0,
        .alternateFunction = IOC_PB07_FUNC_CTL_TRGM1_P_02,
    },
    {
        .tim = (timerResource_t *) HPM_PWM1,
        .tag = TIMER_GET_IO_TAG(PB13),
        .channel = DEF_TIM_CHANNEL(CH_CH9),
        .output = DEF_TIM_OUTPUT(CH_CH5) | 0,
        .alternateFunction = IOC_PB13_FUNC_CTL_TRGM1_P_03,
    },
    {
        .tim = (timerResource_t *) HPM_PWM2,
        .tag = TIMER_GET_IO_TAG(PC16),
        .channel = DEF_TIM_CHANNEL(CH_CH10),
        .output = DEF_TIM_OUTPUT(CH_CH6) | 0,
        .alternateFunction = IOC_PC16_FUNC_CTL_TRGM2_P_02,
    },
    {
        .tim = (timerResource_t *) HPM_PWM2,
        .tag = TIMER_GET_IO_TAG(PC21),
        .channel = DEF_TIM_CHANNEL(CH_CH11),
        .output = DEF_TIM_OUTPUT(CH_CH7) | 0,
        .alternateFunction = IOC_PC21_FUNC_CTL_TRGM2_P_03,
    }
};

const hpmicroTimerHwExt_t hpmicroTimerHwExt[FULL_TIMER_CHANNEL_COUNT] = {
    {
        .timerHw = &fullTimerHardware[0],
        .tag = TIMER_GET_IO_TAG(PB7),
        .channel_ref = DEF_TIM_CHANNEL(CH_CH4),
        .cmp_index = 0,
        .dma_req_cmp_index = 10,
        .pwm_ref_src = HPM_TRGM1_INPUT_SRC_PWM1_CH8REF,
        .trgm_dma_group = 0,
        .pwm_trgm_index = 1,
        .pwm_trgm_dma_src = HPM_TRGM1_DMA_SRC_PWM1_CMP10,
        .pwm_dmamux_src = HPM_DMA_SRC_MOT1_0,
#ifdef USE_DSHOT_TELEMETRY
        .gptmr = HPM_GPTMR2,
#endif
    },
    {
        .timerHw = &fullTimerHardware[1],
        .tag = TIMER_GET_IO_TAG(PB13),
        .channel_ref = DEF_TIM_CHANNEL(CH_CH5),
        .cmp_index = 2,
        .dma_req_cmp_index = 11,
        .pwm_ref_src = HPM_TRGM1_INPUT_SRC_PWM1_CH9REF,
        .trgm_dma_group = 1,
        .pwm_trgm_index = 1,
        .pwm_trgm_dma_src = HPM_TRGM1_DMA_SRC_PWM1_CMP11,
        .pwm_dmamux_src = HPM_DMA_SRC_MOT1_1,
#ifdef USE_DSHOT_TELEMETRY
        .gptmr = HPM_GPTMR3,
#endif
    },
    {
        .timerHw = &fullTimerHardware[2],
        .tag = TIMER_GET_IO_TAG(PC16),
        .channel_ref = DEF_TIM_CHANNEL(CH_CH6),
        .cmp_index = 4,
        .dma_req_cmp_index = 12,
        .pwm_ref_src = HPM_TRGM2_INPUT_SRC_PWM2_CH10REF,
        .trgm_dma_group = 2,
        .pwm_trgm_index = 2,
        .pwm_trgm_dma_src = HPM_TRGM2_DMA_SRC_PWM2_CMP12,
        .pwm_dmamux_src = HPM_DMA_SRC_MOT2_2,
#ifdef USE_DSHOT_TELEMETRY
        .gptmr = HPM_GPTMR4,
#endif
    },
    {
        .timerHw = &fullTimerHardware[3],
        .tag = TIMER_GET_IO_TAG(PC21),
        .channel_ref = DEF_TIM_CHANNEL(CH_CH7),
        .cmp_index = 6,
        .dma_req_cmp_index = 13,
        .pwm_ref_src = HPM_TRGM2_INPUT_SRC_PWM2_CH11REF,
        .trgm_dma_group = 3,
        .pwm_trgm_index = 2,
        .pwm_trgm_dma_src = HPM_TRGM2_DMA_SRC_PWM2_CMP13,
        .pwm_dmamux_src = HPM_DMA_SRC_MOT2_3,
#ifdef USE_DSHOT_TELEMETRY
        .gptmr = HPM_GPTMR5,
#endif
    }
};

static int8_t timerNumbers[HARDWARE_TIMER_DEFINITION_COUNT] = { 1, 2, 3, 4 };

int8_t timerGetNumberByIndex(uint8_t index)
{
    if (index < HARDWARE_TIMER_DEFINITION_COUNT) {
        return timerNumbers[index];
    } else {
        return 0;
    }
}

int8_t timerGetIndexByNumber(uint8_t number)
{
    for (uint8_t index = 0; index < HARDWARE_TIMER_DEFINITION_COUNT; index++) {
        if (timerNumbers[index] == (int8_t) number) {
            return index;
        }
    }

    return -1;
}

rccPeriphTag_t timerRCC(TIM_TypeDef *tim)
{
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].TIMx == tim) {
            return timerDefinitions[i].rcc;
        }
    }
    return 0;
}

void timerInit(void)
{
    /* enable the timer peripherals */
    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        clock_add_to_group(timerRCC((PWM_Type *) TIMER_HARDWARE[i].tim), 0);
    }
}

FAST_CODE volatile timCCR_t *timerChCCR(const timerHardware_t *timHw)
{
    const hpmicroTimerHwExt_t *ext = hpmicroTimerHwExtByTimer(timHw);
    uint8_t cmpIndex = (ext) ? ext->cmp_index : 0;

    PWM_Type *ptr = (PWM_Type *) timHw->tim;
    return (volatile timCCR_t *) ((volatile char *) &ptr->CMP[cmpIndex]);
}
