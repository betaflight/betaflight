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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

#include "platform.h"

#ifdef USE_TRANSPONDER
#include "build/build_config.h"

#include "config/config_reset.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "common/utils.h"

#include "drivers/timer.h"
#include "drivers/transponder_ir.h"
#include "drivers/system.h"
#include "drivers/usb_io.h"

#include "config/config.h"

#include "io/transponder_ir.h"

PG_REGISTER_WITH_RESET_FN(transponderConfig_t, transponderConfig, PG_TRANSPONDER_CONFIG, 0);

void pgResetFn_transponderConfig(transponderConfig_t *transponderConfig)
{
    RESET_CONFIG_2(transponderConfig_t, transponderConfig,
        .provider = TRANSPONDER_ILAP,
        .reserved = 0,
        .data = { 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0x0, 0x0, 0x0 }, // Note, this is NOT a valid transponder code, it's just for testing production hardware
        .ioTag = IO_TAG_NONE
    );
    transponderConfig->ioTag = timerioTagGetByUsage(TIM_USE_TRANSPONDER, 0);
}

static bool transponderInitialised = false;
static bool transponderRepeat = false;

// timers
static timeUs_t nextUpdateAtUs = 0;

#define JITTER_DURATION_COUNT ARRAYLEN(jitterDurations)
static uint8_t jitterDurations[] = {0,9,4,8,3,9,6,7,1,6,9,7,8,2,6};

const transponderRequirement_t transponderRequirements[TRANSPONDER_PROVIDER_COUNT] = {
    {TRANSPONDER_ILAP, TRANSPONDER_DATA_LENGTH_ILAP, TRANSPONDER_TRANSMIT_DELAY_ILAP, TRANSPONDER_TRANSMIT_JITTER_ILAP},
    {TRANSPONDER_ARCITIMER, TRANSPONDER_DATA_LENGTH_ARCITIMER, TRANSPONDER_TRANSMIT_DELAY_ARCITIMER, TRANSPONDER_TRANSMIT_JITTER_ARCITIMER},
    {TRANSPONDER_ERLT, TRANSPONDER_DATA_LENGTH_ERLT, TRANSPONDER_TRANSMIT_DELAY_ERLT, TRANSPONDER_TRANSMIT_JITTER_ERLT}
};

void transponderUpdate(timeUs_t currentTimeUs)
{
    static uint32_t jitterIndex = 0;

    if (!(transponderInitialised && transponderRepeat && isTransponderIrReady())) {
        return;
    }

    const bool updateNow = (timeDelta_t)(currentTimeUs - nextUpdateAtUs) >= 0L;
    if (!updateNow) {
        return;
    }

    uint8_t provider = transponderConfig()->provider;

    // TODO use a random number generator for random jitter?  The idea here is to avoid multiple transmitters transmitting at the same time.
    uint32_t jitter = (transponderRequirements[provider - 1].transmitJitter / 10 * jitterDurations[jitterIndex++]);
    if (jitterIndex >= JITTER_DURATION_COUNT) {
        jitterIndex = 0;
    }

    nextUpdateAtUs = currentTimeUs + transponderRequirements[provider - 1].transmitDelay + jitter;

#ifdef REDUCE_TRANSPONDER_CURRENT_DRAW_WHEN_USB_CABLE_PRESENT
    // reduce current draw when USB cable is plugged in by decreasing the transponder transmit rate.
    if (usbCableIsInserted()) {
        nextUpdateAtUs = currentTimeUs + (1000 * 1000) / 10; // 10 hz.
    }
#endif

    transponderIrTransmit();
}

void transponderInit(void)
{
    transponderInitialised = transponderIrInit(transponderConfig()->ioTag, transponderConfig()->provider);
    if (!transponderInitialised) {
        return;
    }

    transponderIrUpdateData(transponderConfig()->data);
}

void transponderStopRepeating(void)
{
    transponderRepeat = false;
}

void transponderStartRepeating(void)
{
    if (!transponderInitialised) {
        return;
    }

    transponderRepeat = true;
}

void transponderUpdateData(void)
{
    if (!transponderInitialised) {
        return;
    }

    transponderIrUpdateData(transponderConfig()->data);
}

void transponderTransmitOnce(void)
{

    if (!transponderInitialised) {
        return;
    }
    transponderIrTransmit();
}
#endif
