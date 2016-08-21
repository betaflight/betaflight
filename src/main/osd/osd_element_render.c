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

#include "common/printf.h"
#include "drivers/system.h"
#include "drivers/video_textscreen.h"

#include "osd/osd_element.h"
#include "osd/osd_element_render.h"
#include "osd/osd_screen.h"

static char elementAsciiBuffer[31];

void osdElementRender_duration(const element_t *element, elementDataProviderFn dataFn)
{
    uint32_t millis = (uint32_t) dataFn();

    uint32_t totalSeconds = (millis / 1000);
    uint8_t minutes = totalSeconds / 60;
    uint8_t seconds = totalSeconds % 60;

    sprintf(elementAsciiBuffer, "%02d:%02d", minutes, seconds);
    osdPrintAt(element->x, element->y, elementAsciiBuffer);
}

void osdElementRender_mahDrawn(const element_t *element, elementDataProviderFn dataFn)
{
    int32_t mAhDrawn = (int32_t) dataFn();

    tfp_sprintf(elementAsciiBuffer, "mAh: %5d", mAhDrawn);
    osdPrintAt(element->x, element->y, elementAsciiBuffer);
}

void osdElementRender_amperage(const element_t *element, elementDataProviderFn dataFn)
{
    int32_t amperage = (int32_t) dataFn();

    tfp_sprintf(elementAsciiBuffer, "AMP:%2d.%02dA", amperage / 100, amperage % 100);
    osdPrintAt(element->x, element->y, elementAsciiBuffer);
}

void osdElementRender_voltage(const element_t *element, elementDataProviderFn dataFn)
{
    voltageAndName_t *voltageAndName= (voltageAndName_t *) dataFn();

    tfp_sprintf(elementAsciiBuffer, "%3s:%3d.%dV", voltageAndName->name, voltageAndName->voltage / 10, voltageAndName->voltage % 10);
    osdPrintAt(element->x, element->y, elementAsciiBuffer);
}

void osdElementRender_indicatorMag(const element_t *element, elementDataProviderFn dataFn)
{
   bool on = (bool) dataFn();
   if (!on)
       return;

    osdSetCharacterAtPosition(element->x, element->y, 'M');
}

void osdElementRender_indicatorBaro(const element_t *element, elementDataProviderFn dataFn)
{
   bool on = (bool) dataFn();
   if (!on)
       return;

    osdSetCharacterAtPosition(element->x, element->y, 'B');
}

void osdElementRender_flightMode(const element_t *element, elementDataProviderFn dataFn)
{
    uint8_t modes = (int8_t) dataFn();

    char *flightMode = "";
    if (modes & OSD_FLIGHT_MODE_ACRO) {
        flightMode = "ACRO";
    } else if (modes & OSD_FLIGHT_MODE_ANGLE) {
        flightMode = "ANGL";
    } else if (modes & OSD_FLIGHT_MODE_HORIZON) {
        flightMode = "HRZN";
    }

    osdPrintAt(element->x, element->y, flightMode);
}

void osdElementRender_rssi(const element_t *element, elementDataProviderFn dataFn)
{
    uint16_t rssi = (uint16_t) dataFn();

    tfp_sprintf(elementAsciiBuffer, "RSSI:%3d%%", rssi / 10);
    osdPrintAt(element->x, element->y, elementAsciiBuffer);
}

