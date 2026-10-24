/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * The System Manager owns the PLLs, so roots are enabled and read over SCMI. The DShot FlexIO root is moved
 * at init and each LPSPI root at its first enable, as their default rates are too low.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "platform/nxp_clock.h"

#include "scmi_imx95.h"

// CCM clock roots, the index the SCMI clock id is built from.
#define IMX95_ROOT_M7           54

// SCMI clock id of the 400 MHz SYS_PLL1_DFS1_DIV2 source.
#define IMX95_CLK_SYS_PLL1_DFS1_DIV2 10

// The enable answer of a root not asked yet; SCMI answers are zero or negative.
#define CLOCK_NOT_ASKED              1

STATIC_ASSERT(NXP_DSHOT_FLEXIO_INSTANCE == 1 || NXP_DSHOT_FLEXIO_INSTANCE == 2, dshot_flexio_instance_must_be_1_or_2);

static const uint8_t nxpClockRoot[NXP_CLK_COUNT] = {
    [NXP_CLK_LPUART1] = 11,
    [NXP_CLK_LPUART2] = 12,
    [NXP_CLK_LPUART3] = 101,
    [NXP_CLK_LPUART4] = 102,
    [NXP_CLK_LPUART5] = 103,
    [NXP_CLK_LPUART6] = 104,
    [NXP_CLK_LPUART7] = 105,
    [NXP_CLK_LPUART8] = 106,
    [NXP_CLK_LPSPI1]  = 8,
    [NXP_CLK_LPSPI2]  = 9,
    [NXP_CLK_LPSPI3]  = 94,
    [NXP_CLK_LPSPI4]  = 95,
    [NXP_CLK_LPSPI5]  = 96,
    [NXP_CLK_LPSPI6]  = 97,
    [NXP_CLK_LPSPI7]  = 98,
    [NXP_CLK_LPSPI8]  = 99,
    [NXP_CLK_LPI2C1]  = 6,
    [NXP_CLK_LPI2C2]  = 7,
    [NXP_CLK_LPI2C3]  = 88,
    [NXP_CLK_LPI2C4]  = 89,
    [NXP_CLK_LPI2C5]  = 90,
    [NXP_CLK_LPI2C6]  = 91,
    [NXP_CLK_LPI2C7]  = 92,
    [NXP_CLK_LPI2C8]  = 93,
    [NXP_CLK_FLEXIO1] = 83,
    [NXP_CLK_FLEXIO2] = 84,
    // TPM1 and TPM3 have no root of their own and count the AONMIX and WAKEUPMIX bus clocks.
    [NXP_CLK_TPM1]    = 2,
    [NXP_CLK_TPM2]    = 19,
    [NXP_CLK_TPM3]    = 78,
    [NXP_CLK_TPM4]    = 112,
    [NXP_CLK_TPM5]    = 113,
    [NXP_CLK_TPM6]    = 114,
    [NXP_CLK_ADC1]    = 0,
    [NXP_CLK_CAN1]    = 3,
    [NXP_CLK_CAN2]    = 79,
    [NXP_CLK_CAN3]    = 80,
    [NXP_CLK_CAN4]    = 81,
    [NXP_CLK_CAN5]    = 82,
    [NXP_CLK_TSTMR2]  = 115,
};

static uint32_t nxpClockRate[NXP_CLK_COUNT];

// SCMI answer to the enable of each root.
static volatile int32_t nxpClockStatus[NXP_CLK_COUNT] = { [0 ... NXP_CLK_COUNT - 1] = CLOCK_NOT_ASKED };

uint32_t SystemCoreClock = 800000000u;

// A refused move leaves the root as it was; users work from its reported rate.
static void clockMove(nxpClock_e clock, uint32_t parent, uint32_t hz)
{
    const uint32_t id = SCMI_CLOCK_ROOT(nxpClockRoot[clock]);
    if (scmiClockParentSet(id, parent) == SCMI_SUCCESS) {
        scmiClockRateSet(id, hz);
    }
    nxpClockRate[clock] = 0;
}

void nxpClockInit(void)
{
    uint64_t hz;
    if (scmiClockRateGet(SCMI_CLOCK_ROOT(IMX95_ROOT_M7), &hz) == SCMI_SUCCESS && hz && hz <= UINT32_MAX) {
        SystemCoreClock = (uint32_t)hz;
    }

#if NXP_DSHOT_FLEXIO_CLOCK_HZ
    clockMove((nxpClock_e)(NXP_CLK_FLEXIO1 + NXP_DSHOT_FLEXIO_INSTANCE - 1), IMX95_CLK_SYS_PLL1_DFS1_DIV2,
              NXP_DSHOT_FLEXIO_CLOCK_HZ);
#endif

    // Read once so a debugger can check the 24 MHz system_imx95.c assumes.
    nxpClockHz(NXP_CLK_TSTMR2);
}

bool nxpClockGateEnable(nxpClock_e peripheral)
{
    if ((unsigned)peripheral >= NXP_CLK_COUNT) {
        return false;
    }

    // TPM1 and TPM3 count bus clocks and the ADC root is the System Manager's.
    const bool readOnly = peripheral == NXP_CLK_TPM1 || peripheral == NXP_CLK_TPM3 || peripheral == NXP_CLK_ADC1;

    // A granted or refused enable stands until reset, so each root is asked once.
    const int32_t status = nxpClockStatus[peripheral];
    if (!readOnly && status != SCMI_SUCCESS && status != SCMI_DENIED) {
#if NXP_LPSPI_CLOCK_HZ
        if (peripheral >= NXP_CLK_LPSPI1 && peripheral <= NXP_CLK_LPSPI8) {
            clockMove(peripheral, IMX95_CLK_SYS_PLL1_DFS1_DIV2, NXP_LPSPI_CLOCK_HZ);
        }
#endif
        nxpClockStatus[peripheral] = scmiClockEnable(SCMI_CLOCK_ROOT(nxpClockRoot[peripheral]));
    }

    // A root another agent runs refuses the enable, so the rate decides.
    return nxpClockHz(peripheral) != 0;
}

// Asked once per root; the System Manager does not change a root under a running agent.
uint32_t nxpClockHz(nxpClock_e peripheral)
{
    if ((unsigned)peripheral >= NXP_CLK_COUNT) {
        return 0;
    }

    if (!nxpClockRate[peripheral]) {
        uint64_t hz;
        if (scmiClockRateGet(SCMI_CLOCK_ROOT(nxpClockRoot[peripheral]), &hz) == SCMI_SUCCESS
            && hz <= UINT32_MAX) {
            nxpClockRate[peripheral] = (uint32_t)hz;
        }
    }

    return nxpClockRate[peripheral];
}
