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

/*
 * HPMicro timer hardware extension table: maps generic timer hardware entries
 * to PWM channels, TRGM DMA sources and GPTMR capture instances.
 */

#pragma once

#include "drivers/timer.h"
#include "hpm_soc.h"

#define HPM_PWM_REF_SRC_NONE 0U

typedef struct hpmicroTimerHwExt_s {
    const timerHardware_t *timerHw; /**< Corresponding generic timer hardware entry. */
    ioTag_t tag;                    /**< Output pin tag (same as timerHw->tag). */

    uint8_t channel_ref;            /**< PWM channel used for reference / DMA compare. */
    uint8_t cmp_index;              /**< PWM compare index. */
    uint8_t dma_req_cmp_index;      /**< PWM DMA request compare index. */
    uint32_t pwm_ref_src;           /**< HPM_TRGMx_INPUT_SRC_PWMx_CHyREF, or HPM_PWM_REF_SRC_NONE. */
    uint8_t trgm_dma_group;         /**< TRGM DMA request group for PWM (TRGM_DMACFG_x). */
    uint8_t pwm_trgm_index;         /**< TRGM instance index used for PWM DMA request. */
    uint32_t pwm_trgm_dma_src;      /**< HPM_TRGMx_DMA_SRC_PWMx_CMPy for PWM DMA request. */
    uint32_t pwm_dmamux_src;        /**< HPM_DMA_SRC_MOTx_y for PWM output DMA. */
#ifdef USE_DSHOT_TELEMETRY
    GPTMR_Type *gptmr;              /**< GPTMR peripheral base. */
#endif
} hpmicroTimerHwExt_t;

/* Number of entries matches FULL_TIMER_CHANNEL_COUNT. */
extern const hpmicroTimerHwExt_t hpmicroTimerHwExt[];

/**
 * @brief Look up the HPMicro timer hardware extension by pin tag.
 */
const hpmicroTimerHwExt_t *hpmicroTimerHwExtByTag(ioTag_t tag);

/**
 * @brief Look up the HPMicro timer hardware extension by generic timerHardware pointer.
 */
const hpmicroTimerHwExt_t *hpmicroTimerHwExtByTimer(const timerHardware_t *timHw);
