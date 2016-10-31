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
#include "common/filter.h"

#include "drivers/adc.h"
#include "drivers/system.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/config_reset.h"
#include "config/feature.h"

#include "fc/runtime_config.h"
#include "fc/config.h"

#include "io/beeper.h"

#include "sensors/voltage.h"
#include "sensors/battery.h"
#include "amperage.h"

#define VBATT_PRESENT_THRESHOLD_MV    10

uint16_t vbat;

// Battery monitoring stuff
uint8_t batteryCellCount = 3;       // cell count
uint16_t batteryWarningVoltage;
uint16_t batteryCriticalVoltage;

static batteryState_e batteryState;

PG_REGISTER_WITH_RESET_TEMPLATE(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 1);

PG_RESET_TEMPLATE(batteryConfig_t, batteryConfig,
    .vbatmaxcellvoltage = 43,
    .vbatmincellvoltage = 33,
    .vbatwarningcellvoltage = 35,
    .vbathysteresis = 1,
);

#define VBATTERY_STABLE_DELAY 40

void batteryUpdate(void)
{
#ifdef ADC_BATTERY
    vbat = getVoltageForADCChannel(ADC_BATTERY);

    /* battery has just been connected*/
    if (batteryState == BATTERY_NOT_PRESENT && vbat > VBATT_PRESENT_THRESHOLD_MV)
    {
        /* Actual battery state is calculated below, this is really BATTERY_PRESENT */
        batteryState = BATTERY_OK;
        /* wait for VBatt to stabilise then we can calc number of cells
        (using the filtered value takes a long time to ramp up) 
        We only do this on the ground so don't care if we do block, not
        worse than original code anyway*/
        delay(VBATTERY_STABLE_DELAY);
        voltageMeterUpdate();

        uint16_t vbatLatestVoltage = getLatestVoltageForADCChannel(ADC_BATTERY);

        unsigned cells = (vbatLatestVoltage / batteryConfig()->vbatmaxcellvoltage) + 1;
        if (cells > 8) {
            // something is wrong, we expect 8 cells maximum (and autodetection will be problematic at 6+ cells)
            cells = 8;
        }
        batteryCellCount = cells;
        batteryWarningVoltage = batteryCellCount * batteryConfig()->vbatwarningcellvoltage;
        batteryCriticalVoltage = batteryCellCount * batteryConfig()->vbatmincellvoltage;
    }
    /* battery has been disconnected - can take a while for filter cap to disharge so we use a threshold of VBATT_PRESENT_THRESHOLD_MV */
    else if (batteryState != BATTERY_NOT_PRESENT && vbat <= VBATT_PRESENT_THRESHOLD_MV)
    {
        batteryState = BATTERY_NOT_PRESENT;
        batteryCellCount = 0;
        batteryWarningVoltage = 0;
        batteryCriticalVoltage = 0;
    }

    switch(batteryState)
    {
        case BATTERY_OK:
            if (vbat <= (batteryWarningVoltage - batteryConfig()->vbathysteresis)) {
                batteryState = BATTERY_WARNING;
                beeper(BEEPER_BAT_LOW);
            }
            break;
        case BATTERY_WARNING:
            if (vbat <= (batteryCriticalVoltage - batteryConfig()->vbathysteresis)) {
                batteryState = BATTERY_CRITICAL;
                beeper(BEEPER_BAT_CRIT_LOW);
            } else if (vbat > batteryWarningVoltage) {
                batteryState = BATTERY_OK;
            } else {
                beeper(BEEPER_BAT_LOW);
            }
            break;
        case BATTERY_CRITICAL:
            if (vbat > batteryCriticalVoltage) {
                batteryState = BATTERY_WARNING;
                beeper(BEEPER_BAT_LOW);
            } else {
                beeper(BEEPER_BAT_CRIT_LOW);
            }
            break;
        case BATTERY_NOT_PRESENT:
            break;
    }
#endif
}

batteryState_e getBatteryState(void)
{
    return batteryState;
}

const char * const batteryStateStrings[] = {"OK", "WARNING", "CRITICAL", "NOT PRESENT"};

const char * getBatteryStateString(void)
{
    return batteryStateStrings[batteryState];
}

uint8_t batteryVoltagePercentage(void)
{
    return constrain((((uint32_t)vbat - (batteryConfig()->vbatmincellvoltage * batteryCellCount)) * 100) / ((batteryConfig()->vbatmaxcellvoltage - batteryConfig()->vbatmincellvoltage) * batteryCellCount), 0, 100);
}

uint8_t batteryCapacityRemainingPercentage(void)
{
    uint16_t batteryCapacity = batteryConfig()->batteryCapacity;

    amperageMeter_t *state = getAmperageMeter(batteryConfig()->amperageMeterSource);

    return constrain((batteryCapacity - constrain(state->mAhDrawn, 0, 0xFFFF)) * 100.0f / batteryCapacity , 0, 100);
}


void batteryInit(void)
{
    batteryState = BATTERY_NOT_PRESENT;
    batteryCellCount = 1;
    batteryWarningVoltage = 0;
    batteryCriticalVoltage = 0;
}
