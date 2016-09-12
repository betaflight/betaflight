/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "stdbool.h"
#include "stdint.h"
#include "string.h"

#include <platform.h>
#include "build/build_config.h"

#include "common/maths.h"

#include "drivers/adc.h"
#include "drivers/system.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/config_reset.h"

#include "sensors/amperage.h"

amperageMeter_t amperageMeters[MAX_AMPERAGE_METERS];

PG_REGISTER_ARR_WITH_RESET_FN(amperageMeterConfig_t, MAX_AMPERAGE_METERS, amperageMeterConfig, PG_AMPERAGE_METER_CONFIG, 0);

void pgResetFn_amperageMeterConfig(amperageMeterConfig_t *instance)
{
    for (int i = 0; i < MAX_AMPERAGE_METERS; i++) {
        if (i == AMPERAGE_METER_ADC) {
            RESET_CONFIG(amperageMeterConfig_t, &instance[i],
                .scale = 400, // for Allegro ACS758LCB-100U (40mV/A)
            );
        }
    }
}

#define ADCVREF 3300   // in mV
int32_t amperageSensorToCentiamps(const uint16_t src, amperageMeterConfig_t *config)
{
    int32_t millivolts;

    millivolts = ((uint32_t)src * ADCVREF) / 4096;
    millivolts -= config->offset;

    return (millivolts * 1000) / (int32_t)config->scale; // amperage in 0.01A steps
}

void amperageUpdateMeter(int32_t lastUpdateAt)
{
#ifndef ADC_AMPERAGE
    UNUSED(lastUpdateAt);
#else
    static int32_t amperageRaw = 0;
    static int64_t mAhdrawnRaw = 0;
    static uint16_t amperageLatestADC;     // most recent raw reading from current ADC

    amperageMeter_t *state = &amperageMeters[AMPERAGE_METER_ADC];

    amperageRaw -= amperageRaw / 8;
    amperageRaw += amperageLatestADC = adcGetChannel(ADC_AMPERAGE);
    state->amperage = amperageSensorToCentiamps(amperageRaw / 8, amperageMeterConfig(AMPERAGE_METER_ADC));

    mAhdrawnRaw += (MAX(0, state->amperage) * lastUpdateAt) / 1000;
    state->mAhDrawn = mAhdrawnRaw / (3600 * 100);
#endif
}


void amperageUpdateVirtualMeter(int32_t lastUpdateAt, bool armed, bool throttleLowAndMotorStop, int32_t throttleOffset)
{
    static int64_t mAhdrawnRaw = 0;
    int32_t throttleFactor = 0;

    amperageMeter_t *state = &amperageMeters[AMPERAGE_METER_VIRTUAL];

    state->amperage = (int32_t)amperageMeterConfig(AMPERAGE_METER_VIRTUAL)->offset;
    if (armed) {
        if (throttleLowAndMotorStop) {
            throttleOffset = 0;
        }
        throttleFactor = throttleOffset + (throttleOffset * throttleOffset / 50);
        state->amperage += throttleFactor * (int32_t)amperageMeterConfig(AMPERAGE_METER_VIRTUAL)->scale  / 1000;
    }

    mAhdrawnRaw += (MAX(0, state->amperage) * lastUpdateAt) / 1000;
    state->mAhDrawn = mAhdrawnRaw / (3600 * 100);
}


amperageMeter_t *getAmperageMeter(amperageMeter_e  index)
{
    return &amperageMeters[index];
}

void amperageMeterInit(void)
{
}
