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
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/video_textscreen.h"
#include "drivers/video.h"

#include "osd/osd_element.h"
#include "osd/osd_element_render.h"
#include "osd/osd_screen.h"
#include "osd/fonts/font.h"

// from osd hardware implementation
void osdHardwareDisplayMotor(uint8_t x, uint8_t y, uint8_t percent);

static char elementAsciiBuffer[31];

void osdElementRender_duration(const element_t *element, elementDataProviderFn dataFn)
{
    uint32_t millis = (uint32_t) dataFn();

    uint32_t totalSeconds = (millis / 1000);
    uint8_t minutes = totalSeconds / 60;
    uint8_t seconds = totalSeconds % 60;

    sprintf(elementAsciiBuffer, "%3d:%02d", minutes, seconds);
    osdPrintAt(element->x, element->y, elementAsciiBuffer);
}

void osdElementRender_mahDrawn(const element_t *element, elementDataProviderFn dataFn)
{
    int32_t mAhDrawn = (int32_t) dataFn();

    tfp_sprintf(elementAsciiBuffer, "%5d", mAhDrawn);
    osdPrintAt(element->x, element->y, elementAsciiBuffer);
    osdSetRawCharacterAtPosition(element->x + 5, element->y, FONT_CHARACTER_MAH);
}

void osdElementRender_amperage(const element_t *element, elementDataProviderFn dataFn)
{
    int32_t amperage = (int32_t) dataFn();

    tfp_sprintf(elementAsciiBuffer, "%2d.%02d", amperage / 100, amperage % 100);
    osdPrintAt(element->x, element->y, elementAsciiBuffer);
    osdSetRawCharacterAtPosition(element->x + 5, element->y, FONT_CHARACTER_AMP);
}

void osdElementRender_voltage(const element_t *element, elementDataProviderFn dataFn)
{
    voltageAndName_t *voltageAndName= (voltageAndName_t *) dataFn();

    tfp_sprintf(elementAsciiBuffer, "%s:%2d.%dV", voltageAndName->name, voltageAndName->voltage / 10, voltageAndName->voltage % 10);
    osdPrintAt(element->x, element->y, elementAsciiBuffer);
}

void osdElementRender_voltageBattery(const element_t *element, elementDataProviderFn dataFn)
{
    voltageAndName_t *voltageAndName= (voltageAndName_t *) dataFn();

    tfp_sprintf(elementAsciiBuffer, "%2d.%dV", voltageAndName->voltage / 10, voltageAndName->voltage % 10);
    osdPrintAt(element->x +1, element->y, elementAsciiBuffer);
    osdSetRawCharacterAtPosition(element->x, element->y, voltageAndName->symbol);
}

void osdElementRender_indicatorMag(const element_t *element, elementDataProviderFn dataFn)
{
    bool on = (bool) dataFn();
    if (!on)
        return;

    osdSetRawCharacterAtPosition(element->x, element->y, FONT_CHARACTER_MAG);
}

void osdElementRender_indicatorBaro(const element_t *element, elementDataProviderFn dataFn)
{
   bool on = (bool) dataFn();
   if (!on)
       return;

    osdSetRawCharacterAtPosition(element->x, element->y, FONT_CHARACTER_BARO);
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

    tfp_sprintf(elementAsciiBuffer, "%3d", (rssi / 1023) * 100);
    osdPrintAt(element->x, element->y, elementAsciiBuffer);
    osdSetRawCharacterAtPosition(element->x + 3, element->y, FONT_CHARACTER_RSSI);
}

void osdElementRender_callsign(const element_t *element, elementDataProviderFn dataFn)
{
    uint8_t *callsign = (uint8_t *) dataFn();

    osdPrintAt(element->x, element->y, (char *)callsign);
}


// 4x4 grid
struct quadMotorCoordinateOffset_s {
    uint8_t x;
    uint8_t y;
} quadMotorCoordinateOffsets[4] = {
    // FIXME assumes standard motor order
    {3, 3},
    {3, 1},
    {0, 3},
    {0, 1}
};

void osdElementRender_motors(const element_t *element, elementDataProviderFn dataFn)
{
    uint16_t *motors = (uint16_t *) dataFn();

    const int maxMotors = 4; // just quad for now
    for (int i = 0; i < maxMotors; i++) {
        if (!motors[i]) {
            continue; // skip unused/uninitialsed motors.
        }
        int percent = scaleRange(motors[i], 1000, 2000, 0, 100); // FIXME should use min/max command as used by the FC.

        osdHardwareDisplayMotor(quadMotorCoordinateOffsets[i].x + element->x, quadMotorCoordinateOffsets[i].y + element->y, percent);
    }
}
