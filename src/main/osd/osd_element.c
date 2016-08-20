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

#include "drivers/system.h"  // only required for data providers
#include "drivers/video_textscreen.h"
#include "fc/rc_controls.h" // only required for data providers
#include "sensors/battery.h" // only required for data providers
#include "common/utils.h"


#include "osd/osd_element.h"
#include "osd/osd_element_render.h"

intptr_t osdElementData_onTime(void)
{
    return (intptr_t)millis();
}

intptr_t osdElementData_mAhDrawn(void)
{
    return (intptr_t)mAhDrawn;
}

elementHandlerConfig_t elementHandlers[] = {
    {OSD_ELEMENT_ON_TIME, osdElementRender_onTime, osdElementData_onTime},
    {OSD_ELEMENT_MAH_DRAWN, osdElementRender_mahDrawn, osdElementData_mAhDrawn}
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

void osdDrawTextElement(const element_t *element)
{
    if (!element->enabled) {
        return;
    }

    elementHandlerConfig_t *elementHandlerConfig = osdFindElementHandler(element->id);

    if (!elementHandlerConfig) {
        return;
    }

    elementHandlerConfig->renderFn(element, elementHandlerConfig->dataFn);
}
