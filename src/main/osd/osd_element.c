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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <platform.h>
#include "build/debug.h"

// only required for data providers
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/video_textscreen.h"
#include "drivers/system.h"  // only required for data providers
#include "drivers/adc.h"  // only required for data providers
#include "fc/rc_controls.h" // only required for data providers // FIXME dependency on FC code for throttle status
#include "osd/fc_state.h" // only required for data providers
#include "sensors/battery.h" // only required for data providers
#include "common/utils.h"


#include "osd/osd_element.h"
#include "osd/osd_element_render.h"

intptr_t osdElementData_onDuration(void)
{
    return (intptr_t)millis();
}

intptr_t osdElementData_armedDuration(void)
{
    return (intptr_t)fcStatus.armedDuration;
}

intptr_t osdElementData_mAhDrawn(void)
{
    return (intptr_t)mAhDrawn;
}

intptr_t osdElementData_amperage(void)
{
    return (intptr_t)amperage;
}

static voltageAndName_t voltageAndName;

intptr_t osdElementData_voltage5V(void)
{
    voltageAndName = (voltageAndName_t){
        .name = "5V",
        .voltage = batteryAdcToVoltage(adcGetChannel(ADC_POWER_5V))
    };
    return (intptr_t) &voltageAndName;
}

intptr_t osdElementData_voltage12V(void)
{
    voltageAndName = (voltageAndName_t){
        .name = "12V",
        .voltage = batteryAdcToVoltage(adcGetChannel(ADC_POWER_12V))
    };
    return (intptr_t) &voltageAndName;
}

intptr_t osdElementData_voltageBattery(void)
{
    voltageAndName = (voltageAndName_t){
        .name = "BAT",
        .voltage = vbat
    };
    return (intptr_t) &voltageAndName;
}


intptr_t osdElementData_voltageBatteryFC(void)
{
    voltageAndName = (voltageAndName_t){
        .name = " FC",
        .voltage = fcStatus.vbat
    };
    return (intptr_t) &voltageAndName;
}

intptr_t osdElementData_flightModeFC(void)
{
    // translate FC state into the mode flags accepted by the renderer
    uint8_t modes = 0;
    if (fcStatus.fcState & (1 << FC_STATE_HORIZON)) {
        modes |= OSD_FLIGHT_MODE_HORIZON;
    }
    if (fcStatus.fcState & (1 << FC_STATE_ANGLE)) {
        modes |= OSD_FLIGHT_MODE_ANGLE;
    }

    if (modes == 0) {
        modes |= OSD_FLIGHT_MODE_ACRO;
    }

    return (intptr_t) modes;
}

intptr_t osdElementData_indicatorBaroFC(void)
{
    return (intptr_t) fcStatus.fcState & (1 << FC_STATE_BARO);
}

intptr_t osdElementData_indicatorMagFC(void)
{
    return (intptr_t) fcStatus.fcState & (1 << FC_STATE_MAG);
}

intptr_t osdElementData_rssiFC(void)
{
    return (intptr_t) fcStatus.rssi;
}

elementHandlerConfig_t elementHandlers[] = {
    {OSD_ELEMENT_ON_DURATION, osdElementRender_duration, osdElementData_onDuration},
    {OSD_ELEMENT_ARMED_DURATION, osdElementRender_duration, osdElementData_armedDuration},
    {OSD_ELEMENT_MAH_DRAWN, osdElementRender_mahDrawn, osdElementData_mAhDrawn},
    {OSD_ELEMENT_AMPERAGE, osdElementRender_amperage, osdElementData_amperage},
    {OSD_ELEMENT_VOLTAGE_5V, osdElementRender_voltage, osdElementData_voltage5V},
    {OSD_ELEMENT_VOLTAGE_12V, osdElementRender_voltage, osdElementData_voltage12V},
    {OSD_ELEMENT_VOLTAGE_BATTERY, osdElementRender_voltage, osdElementData_voltageBattery},
    {OSD_ELEMENT_VOLTAGE_BATTERY_FC, osdElementRender_voltage, osdElementData_voltageBatteryFC},
    {OSD_ELEMENT_FLIGHT_MODE, osdElementRender_flightMode, osdElementData_flightModeFC},
    {OSD_ELEMENT_INDICATOR_MAG, osdElementRender_indicatorMag, osdElementData_indicatorMagFC},
    {OSD_ELEMENT_INDICATOR_BARO, osdElementRender_indicatorBaro, osdElementData_indicatorBaroFC},
    {OSD_ELEMENT_RSSI_FC, osdElementRender_rssi, osdElementData_rssiFC},
};

static elementHandlerConfig_t *osdFindElementHandler(uint8_t id)
{
    for (unsigned int i = 0; i < ARRAYLEN(elementHandlers); i++) {
        elementHandlerConfig_t *candidate = &elementHandlers[i];
        if (candidate->id == id) {
            return candidate;
        }
    }
    return NULL;
}

typedef struct osdElementState_s {
    bool flashWhenDisconnected;
} osdElementState_t;

static osdElementState_t osdElementState;

// set showNow parameter to true when the element should be drawn, flash frequency and state defined by caller.
void osdSetElementFlashOnDisconnectState(bool showNow) {
    osdElementState.flashWhenDisconnected = showNow;
}

void osdDrawTextElement(const element_t *element)
{
    if (!element->flags & EF_ENABLED) {
        return;
    }
    if (element->flags & EF_FLASH_ON_DISCONNECT && !osdElementState.flashWhenDisconnected) {
        return;
    }

    elementHandlerConfig_t *elementHandlerConfig = osdFindElementHandler(element->id);

    if (!elementHandlerConfig) {
        return;
    }

    elementHandlerConfig->renderFn(element, elementHandlerConfig->dataFn);
}
