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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/dma_reqmap.h"
#include "hpm_soc.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * HPMicro-specific dmaPeripheral_e values used locally in dma_reqmap_mcu.c.
 * They are kept outside the generic enum so platform-external headers are
 * not modified.
 */
typedef enum {
    HPM_DMA_PERIPH_PWM_OUT,
    HPM_DMA_PERIPH_GPTMR_CAP_NEG,
    HPM_DMA_PERIPH_GPTMR_CAP_POS,
} hpmDmaPeripheral_e;

/**
 * @brief Return the DMA channel spec for a motor PWM output.
 */
const dmaChannelSpec_t *hpmDmaGetPwmOutChannelSpec(uint8_t motorIndex);

/**
 * @brief Return the DMA channel spec for a GPTMR capture channel.
 * @param gptmrIndex  GPTMR instance index (0..5)
 * @param posEdge     true for positive-edge (CH2), false for negative-edge (CH3)
 */
const dmaChannelSpec_t *hpmDmaGetGptmrCapChannelSpec(uint8_t gptmrIndex, bool posEdge);


#ifdef __cplusplus
}
#endif
