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
 * TRGM pin resource allocation for DShot output and telemetry.
 * Maps TRGM pins to PWM reference sources and GPTMR input capture channels.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/io_types.h"
#include "hpm_soc.h"
#include "hpm_trgm_drv.h"

typedef struct trgmDshotResource_s {
    /* Static fields copied from the map. */
    ioTag_t pin;             /**< Pin used as TRGMx_Py. */
    uint32_t ioc_function;   /**< IOC function for the TRGMx_Py pin. */
    uint8_t trgm_index;      /**< TRGM instance index (0, 1, 2, ...). */
    uint8_t port_index;      /**< TRGM port index within the instance. */
    uint32_t trgm_p_dst;     /**< HPM_TRGMx_OUTPUT_SRC_TRGMx_Py (output to pin). */
    uint32_t trgm_p_src;     /**< HPM_TRGMx_INPUT_SRC_TRGMx_Py (input from pin). */

    /* Runtime fields filled at alloc time. */
    uint32_t pwm_ref_src;    /**< HPM_TRGMx_INPUT_SRC_PWMx_CHyREF (input to TRGM for output). */

    /* Runtime fields filled by trgmDshotResourceAllocInput(). */
    GPTMR_Type *gptmr;       /**< GPTMR bound for telemetry input capture. */
    uint32_t gptmr_clock;    /**< Clock name for the GPTMR. */
    uint32_t gptmr_in2_dst;  /**< HPM_TRGMx_OUTPUT_SRC_GPTMRx_IN2. */
    uint32_t gptmr_in3_dst;  /**< HPM_TRGMx_OUTPUT_SRC_GPTMRx_IN3. */
} trgmDshotResource_t;

/**
 * @brief Allocate the TRGM pin associated with @p pin.
 *
 * On success the static fields of @p res are filled and the pin is marked as
 * in-use. Returns false if the pin is not in the map or already allocated.
 */
bool trgmDshotResourceAlloc(ioTag_t pin, trgmDshotResource_t *res);

/**
 * @brief Bind a PWM reference source to an allocated pin.
 *
 * The source must be a PWM CH8REF..CH23REF value that matches the TRGM
 * instance (TRGM0-&gt;PWM0, TRGM1-&gt;PWM1, TRGM2-&gt;PWM2). Returns false if the
 * source is invalid or already allocated.
 */
bool trgmDshotResourceAllocPwmRef(trgmDshotResource_t *res, uint32_t pwm_ref_src);

/**
 * @brief Bind an allocated pin to a GPTMR for DShot telemetry input capture.
 *
 * The TRGM input (TRGMx_Py) is routed to the selected GPTMR's IN2/3 channels.
 * Returns false if the GPTMR is not reachable from this TRGM instance or is
 * already allocated.
 */
bool trgmDshotResourceAllocInput(trgmDshotResource_t *res, GPTMR_Type *gptmr);

/**
 * @brief Release the PWM reference source binding.
 */
void trgmDshotResourceFreePwmRef(trgmDshotResource_t *res);

/**
 * @brief Release the GPTMR input binding created by trgmDshotResourceAllocInput().
 */
void trgmDshotResourceFreeInput(trgmDshotResource_t *res);

/**
 * @brief Release a TRGM pin allocated by trgmDshotResourceAlloc().
 *
 * Also releases any input binding still attached to it.
 */
void trgmDshotResourceFree(trgmDshotResource_t *res);

/**
 * @brief Return the TRGM base register pointer for a resource.
 */
TRGM_Type *trgmDshotResourceTrgm(const trgmDshotResource_t *res);

/**
 * @brief Return the TRGM base register pointer for an instance index.
 */
TRGM_Type *trgmDshotTrgmByIndex(uint8_t trgm_index);

/**
 * @brief Return the GPTMR instance index for a GPTMR base pointer.
 */
int trgmDshotGptmrIndexByGptmr(GPTMR_Type *gptmr);
