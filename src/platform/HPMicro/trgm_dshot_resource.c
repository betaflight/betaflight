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

// TRGM resource allocation for DShot: maps pins to TRGMx_Py outputs and
// GPTMR edge-capture inputs, tracking allocations in bitmaps.

#include "platform.h"

#include "trgm_dshot_resource.h"
#include "timer_hw_ext.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/utils.h"
#include "drivers/io.h"
#ifdef USE_DSHOT_TELEMETRY
#include "hpm_clock_drv.h"
#endif
#include "hpm_iomux.h"
#include "hpm_soc.h"
#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"
#include "hpm_dmamux_src.h"

/*
 * Compact map entry describing a pin that can be used as TRGMx_Py for DShot.
 * The output side (PWMx_CHyREF -> TRGMx_Py -> pin) is configured at runtime
 * by trgmDshotResourceAllocPwmRef().
 * The input side (TRGMx_Py -> GPTMRx_IN2/3) is configured at runtime by
 * trgmDshotResourceAllocInput().
 */
typedef struct trgmDshotPinMapEntry_s {
    ioTag_t pin;
    uint32_t ioc_function;   /**< IOC function for the TRGMx_Py pin. */
    uint8_t trgm_index;
    uint8_t port_index;
    uint32_t trgm_p_dst;     /**< HPM_TRGMx_OUTPUT_SRC_TRGMx_Py (output to pin). */
#ifdef USE_DSHOT_TELEMETRY
    uint32_t trgm_p_src;     /**< HPM_TRGMx_INPUT_SRC_TRGMx_Py (input from pin). */
#endif
} trgmDshotPinMapEntry_t;

#ifdef USE_DSHOT_TELEMETRY
#define TRGM_DSHOT_PIN_ENTRY(pin, iocFunction, trgmIndex, portIndex, outputDst, inputSrc) \
    { pin, iocFunction, trgmIndex, portIndex, outputDst, inputSrc }
#else
#define TRGM_DSHOT_PIN_ENTRY(pin, iocFunction, trgmIndex, portIndex, outputDst, inputSrc) \
    { pin, iocFunction, trgmIndex, portIndex, outputDst }
#endif

static const trgmDshotPinMapEntry_t trgmDshotPinMap[] = {
#ifdef HPM6360
    /* TRGM1 pins. */
    TRGM_DSHOT_PIN_ENTRY(IO_TAG(PA22), IOC_PA22_FUNC_CTL_TRGM1_P_02, 1, 2, HPM_TRGM1_OUTPUT_SRC_TRGM1_P2, HPM_TRGM1_INPUT_SRC_TRGM1_P2),
    TRGM_DSHOT_PIN_ENTRY(IO_TAG(PA27), IOC_PA27_FUNC_CTL_TRGM1_P_07, 1, 7, HPM_TRGM1_OUTPUT_SRC_TRGM1_P7, HPM_TRGM1_INPUT_SRC_TRGM1_P7),

    /* TRGM0 pins. */
    TRGM_DSHOT_PIN_ENTRY(IO_TAG(PB20), IOC_PB20_FUNC_CTL_TRGM0_P_00, 0, 0, HPM_TRGM0_OUTPUT_SRC_TRGM0_P0, HPM_TRGM0_INPUT_SRC_TRGM0_P0),
    TRGM_DSHOT_PIN_ENTRY(IO_TAG(PB21), IOC_PB21_FUNC_CTL_TRGM0_P_01, 0, 1, HPM_TRGM0_OUTPUT_SRC_TRGM0_P1, HPM_TRGM0_INPUT_SRC_TRGM0_P1),

    /* Additional TRGM0 pins that can be swapped in at runtime. */
    TRGM_DSHOT_PIN_ENTRY(IO_TAG(PB22), IOC_PB22_FUNC_CTL_TRGM0_P_02, 0, 2, HPM_TRGM0_OUTPUT_SRC_TRGM0_P2, HPM_TRGM0_INPUT_SRC_TRGM0_P2),
    TRGM_DSHOT_PIN_ENTRY(IO_TAG(PB23), IOC_PB23_FUNC_CTL_TRGM0_P_03, 0, 3, HPM_TRGM0_OUTPUT_SRC_TRGM0_P3, HPM_TRGM0_INPUT_SRC_TRGM0_P3),
#endif

#ifdef HPM6750
    /* TRGM1 motor pins. */
    TRGM_DSHOT_PIN_ENTRY(IO_TAG(PB7), IOC_PB07_FUNC_CTL_TRGM1_P_02, 1, 2, HPM_TRGM1_OUTPUT_SRC_TRGM1_P2, HPM_TRGM1_INPUT_SRC_TRGM1_P2),
    TRGM_DSHOT_PIN_ENTRY(IO_TAG(PB13), IOC_PB13_FUNC_CTL_TRGM1_P_03, 1, 3, HPM_TRGM1_OUTPUT_SRC_TRGM1_P3, HPM_TRGM1_INPUT_SRC_TRGM1_P3),

    /* TRGM2 motor pins. */
    TRGM_DSHOT_PIN_ENTRY(IO_TAG(PC16), IOC_PC16_FUNC_CTL_TRGM2_P_02, 2, 2, HPM_TRGM2_OUTPUT_SRC_TRGM2_P2, HPM_TRGM2_INPUT_SRC_TRGM2_P2),
    TRGM_DSHOT_PIN_ENTRY(IO_TAG(PC21), IOC_PC21_FUNC_CTL_TRGM2_P_03, 2, 3, HPM_TRGM2_OUTPUT_SRC_TRGM2_P3, HPM_TRGM2_INPUT_SRC_TRGM2_P3),
#endif
};

#undef TRGM_DSHOT_PIN_ENTRY

#define TRGM_DSHOT_PIN_MAP_COUNT (sizeof(trgmDshotPinMap) / sizeof(trgmDshotPinMap[0]))
STATIC_ASSERT(TRGM_DSHOT_PIN_MAP_COUNT <= 32, trgmDshotPinMapFitsAllocationBitmap);

#ifdef USE_DSHOT_TELEMETRY
/*
 * Map entry describing a GPTMR that can be used for DShot telemetry input.
 * For each GPTMR this records the clock, the TRGM instance that can reach it,
 * and the TRGM output destinations for GPTMRx_IN2/3.
 */
typedef struct trgmDshotGptmrMapEntry_s {
    GPTMR_Type *gptmr;
    uint8_t gptmr_index;        /**< GPTMR instance index (used for allocation bitmap). */
    uint8_t trgm_index;         /**< TRGM instance that can route to this GPTMR. */
    uint32_t gptmr_clock;       /**< Clock name for the GPTMR. */
    uint32_t trgm_in2_dst;      /**< HPM_TRGMx_OUTPUT_SRC_GPTMRx_IN2. */
    uint32_t trgm_in3_dst;      /**< HPM_TRGMx_OUTPUT_SRC_GPTMRx_IN3. */
} trgmDshotGptmrMapEntry_t;

static const trgmDshotGptmrMapEntry_t trgmDshotGptmrMap[] = {
#ifdef HPM6360
    { HPM_GPTMR0, 0, 0, clock_gptmr0, HPM_TRGM0_OUTPUT_SRC_GPTMR0_IN2, HPM_TRGM0_OUTPUT_SRC_GPTMR0_IN3 },
    { HPM_GPTMR1, 1, 0, clock_gptmr1, HPM_TRGM0_OUTPUT_SRC_GPTMR1_IN2, HPM_TRGM0_OUTPUT_SRC_GPTMR1_IN3 },
    { HPM_GPTMR2, 2, 1, clock_gptmr2, HPM_TRGM1_OUTPUT_SRC_GPTMR2_IN2, HPM_TRGM1_OUTPUT_SRC_GPTMR2_IN3 },
#endif
#ifdef HPM6750
    { HPM_GPTMR0, 0, 0, clock_gptmr0, HPM_TRGM0_OUTPUT_SRC_GPTMR0_IN2, HPM_TRGM0_OUTPUT_SRC_GPTMR0_IN3 },
    { HPM_GPTMR1, 1, 0, clock_gptmr1, HPM_TRGM0_OUTPUT_SRC_GPTMR1_IN2, HPM_TRGM0_OUTPUT_SRC_GPTMR1_IN3 },
    { HPM_GPTMR2, 2, 1, clock_gptmr2, HPM_TRGM1_OUTPUT_SRC_GPTMR2_IN2, HPM_TRGM1_OUTPUT_SRC_GPTMR2_IN3 },
    { HPM_GPTMR3, 3, 1, clock_gptmr3, HPM_TRGM1_OUTPUT_SRC_GPTMR3_IN2, HPM_TRGM1_OUTPUT_SRC_GPTMR3_IN3 },
    { HPM_GPTMR4, 4, 2, clock_gptmr4, HPM_TRGM2_OUTPUT_SRC_GPTMR4_IN2, HPM_TRGM2_OUTPUT_SRC_GPTMR4_IN3 },
    { HPM_GPTMR5, 5, 2, clock_gptmr5, HPM_TRGM2_OUTPUT_SRC_GPTMR5_IN2, HPM_TRGM2_OUTPUT_SRC_GPTMR5_IN3 },
#endif
};

#define TRGM_DSHOT_GPTMR_MAP_COUNT (sizeof(trgmDshotGptmrMap) / sizeof(trgmDshotGptmrMap[0]))
#endif

/* Bitmaps tracking allocated resources. */
static uint32_t allocatedPins;  /* one bit per pin map entry */
static uint16_t allocatedPwmRef[3];     /* one 16-bit bitmap per PWM timer (CH8..23) */
#ifdef USE_DSHOT_TELEMETRY
static uint32_t allocatedGptmr; /* one bit per GPTMR index */
#endif

/*
 * Return the TRGM base register pointer for an instance index.
 */
FAST_CODE TRGM_Type *trgmDshotTrgmByIndex(uint8_t trgm_index)
{
    switch (trgm_index) {
    case 0:
        return HPM_TRGM0;
    case 1:
        return HPM_TRGM1;
#ifdef HPM_TRGM2
    case 2:
        return HPM_TRGM2;
#endif
    default:
        return NULL;
    }
}

#ifdef USE_DSHOT_TELEMETRY
/*
 * Return the GPTMR instance index for a GPTMR base pointer.
 */
FAST_CODE int trgmDshotGptmrIndexByGptmr(GPTMR_Type *gptmr)
{
    if (gptmr == NULL) {
        return -1;
    }
    for (uint32_t i = 0; i < TRGM_DSHOT_GPTMR_MAP_COUNT; i++) {
        if (trgmDshotGptmrMap[i].gptmr == gptmr) {
            return (int) trgmDshotGptmrMap[i].gptmr_index;
        }
    }
    return -1;
}
#endif

FAST_CODE TRGM_Type *trgmDshotResourceTrgm(const trgmDshotResource_t *res)
{
    if (res == NULL) {
        return NULL;
    }
    return trgmDshotTrgmByIndex(res->trgm_index);
}

/*
 * Sanity-check that the requested source is in the PWM CHyREF range.
 * On HPM6xxx the PWM CH8REF..CH23REF inputs occupy 0x14-0x23 in every
 * TRGM instance (the same numeric value selects PWM0_CHyREF on TRGM0,
 * PWM1_CHyREF on TRGM1, PWM2_CHyREF on TRGM2, etc.).
 */
#define TRGM_DSHOT_PWM_REF_MIN  (0x14U)
#define TRGM_DSHOT_PWM_REF_MAX  (0x23U)

static bool trgmDshotIsPwmRefSrcValid(uint32_t pwm_ref_src)
{
    return pwm_ref_src >= TRGM_DSHOT_PWM_REF_MIN && pwm_ref_src <= TRGM_DSHOT_PWM_REF_MAX;
}

static const trgmDshotPinMapEntry_t *trgmDshotPinMapEntryByPin(ioTag_t pin, uint32_t *entryIndex)
{
    for (uint32_t i = 0; i < TRGM_DSHOT_PIN_MAP_COUNT; i++) {
        const trgmDshotPinMapEntry_t *entry = &trgmDshotPinMap[i];
        if (entry->pin == pin) {
            if (entryIndex != NULL) {
                *entryIndex = i;
            }
            return entry;
        }
    }

    return NULL;
}

bool trgmDshotResourceAlloc(ioTag_t pin, trgmDshotResource_t *res)
{
    uint32_t entryIndex;
    const trgmDshotPinMapEntry_t *entry = trgmDshotPinMapEntryByPin(pin, &entryIndex);
    if (res == NULL || entry == NULL) {
        return false;
    }
    if (allocatedPins & (1U << entryIndex)) {
        return false;
    }

    memset(res, 0, sizeof(*res));
    res->pin = entry->pin;
    res->ioc_function = entry->ioc_function;
    res->trgm_index = entry->trgm_index;
    res->port_index = entry->port_index;
    res->trgm_p_dst = entry->trgm_p_dst;
#ifdef USE_DSHOT_TELEMETRY
    res->trgm_p_src = entry->trgm_p_src;
#endif

    allocatedPins |= (1U << entryIndex);
    return true;
}

bool trgmDshotResourceAllocPwmRef(trgmDshotResource_t *res, uint32_t pwm_ref_src)
{
    if (res == NULL || res->pin == IO_TAG_NONE) {
        return false;
    }
    if (!trgmDshotIsPwmRefSrcValid(pwm_ref_src)) {
        return false;
    }
    if (res->trgm_index >= ARRAYLEN(allocatedPwmRef)) {
        return false;
    }

    int bit = (int) (pwm_ref_src - TRGM_DSHOT_PWM_REF_MIN);
    if ((allocatedPwmRef[res->trgm_index] & (1U << bit)) && res->pwm_ref_src != pwm_ref_src) {
        return false;
    }

    /* Release any previous PWM reference source before taking the new one. */
    trgmDshotResourceFreePwmRef(res);

    res->pwm_ref_src = pwm_ref_src;
    allocatedPwmRef[res->trgm_index] |= (1U << bit);
    return true;
}

#ifdef USE_DSHOT_TELEMETRY
bool trgmDshotResourceAllocInput(trgmDshotResource_t *res, GPTMR_Type *gptmr)
{
    if (res == NULL || res->pin == IO_TAG_NONE || gptmr == NULL) {
        return false;
    }

    const trgmDshotGptmrMapEntry_t *gptmrEntry = NULL;

    for (uint32_t i = 0; i < TRGM_DSHOT_GPTMR_MAP_COUNT; i++) {
        if (trgmDshotGptmrMap[i].gptmr == gptmr) {
            gptmrEntry = &trgmDshotGptmrMap[i];
            break;
        }
    }
    if (gptmrEntry == NULL) {
        return false;
    }
    if (gptmrEntry->trgm_index != res->trgm_index) {
        return false;
    }
    if ((allocatedGptmr & (1U << gptmrEntry->gptmr_index)) && res->gptmr != gptmr) {
        return false;
    }

    /* Release any previous input binding before taking the new one. */
    trgmDshotResourceFreeInput(res);

    res->gptmr = gptmrEntry->gptmr;
    res->gptmr_clock = gptmrEntry->gptmr_clock;
    res->gptmr_in2_dst = gptmrEntry->trgm_in2_dst;
    res->gptmr_in3_dst = gptmrEntry->trgm_in3_dst;

    allocatedGptmr |= (1U << gptmrEntry->gptmr_index);
    return true;
}

void trgmDshotResourceFreeInput(trgmDshotResource_t *res)
{
    if (res == NULL || res->gptmr == NULL) {
        return;
    }

    for (uint32_t i = 0; i < TRGM_DSHOT_GPTMR_MAP_COUNT; i++) {
        if (trgmDshotGptmrMap[i].gptmr == res->gptmr) {
            allocatedGptmr &= ~(1U << trgmDshotGptmrMap[i].gptmr_index);
            break;
        }
    }

    res->gptmr = NULL;
    res->gptmr_clock = 0;
    res->gptmr_in2_dst = 0;
    res->gptmr_in3_dst = 0;
}
#endif

void trgmDshotResourceFreePwmRef(trgmDshotResource_t *res)
{
    if (res == NULL || res->pwm_ref_src == HPM_PWM_REF_SRC_NONE) {
        return;
    }
    if (res->trgm_index >= ARRAYLEN(allocatedPwmRef)) {
        return;
    }

    int bit = (int) (res->pwm_ref_src - TRGM_DSHOT_PWM_REF_MIN);
    allocatedPwmRef[res->trgm_index] &= ~(1U << bit);
    res->pwm_ref_src = HPM_PWM_REF_SRC_NONE;
}

void trgmDshotResourceFree(trgmDshotResource_t *res)
{
    if (res == NULL) {
        return;
    }

#ifdef USE_DSHOT_TELEMETRY
    trgmDshotResourceFreeInput(res);
#endif
    trgmDshotResourceFreePwmRef(res);

    for (uint32_t i = 0; i < TRGM_DSHOT_PIN_MAP_COUNT; i++) {
        if (trgmDshotPinMap[i].pin == res->pin) {
            allocatedPins &= ~(1U << i);
            break;
        }
    }

    memset(res, 0, sizeof(*res));
}
