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

#include "platform.h"

#ifdef USE_CAMERA_CONTROL

#ifndef CAMERA_CONTROL_PIN
#define CAMERA_CONTROL_PIN NONE
#endif

#include "drivers/camera_control.h"
#include "drivers/camera_control_impl.h"

#include "drivers/io.h"
#include "drivers/time.h"
#include "pg/pg_ids.h"

#ifdef USE_OSD
#include "osd/osd.h"
#endif

PG_REGISTER_WITH_RESET_FN(cameraControlConfig_t, cameraControlConfig, PG_CAMERA_CONTROL_CONFIG, 0);

void pgResetFn_cameraControlConfig(cameraControlConfig_t *cameraControlConfig)
{
    cameraControlConfig->mode = CAMERA_CONTROL_MODE_HARDWARE_PWM;
    cameraControlConfig->refVoltage = 330;
    cameraControlConfig->keyDelayMs = 180;
    cameraControlConfig->internalResistance = 470;
    cameraControlConfig->ioTag = IO_TAG(CAMERA_CONTROL_PIN);
    cameraControlConfig->inverted = 0;   // Output is inverted externally
    cameraControlConfig->buttonResistanceValues[CAMERA_CONTROL_KEY_ENTER] = 450;
    cameraControlConfig->buttonResistanceValues[CAMERA_CONTROL_KEY_LEFT]  = 270;
    cameraControlConfig->buttonResistanceValues[CAMERA_CONTROL_KEY_UP]    = 150;
    cameraControlConfig->buttonResistanceValues[CAMERA_CONTROL_KEY_RIGHT] = 68;
    cameraControlConfig->buttonResistanceValues[CAMERA_CONTROL_KEY_DOWN]  = 0;
}

static cameraControlRuntime_t cameraControlRuntime;

void cameraControlInit(void)
{
    if (cameraControlConfig()->ioTag == IO_TAG_NONE) {
        return;
    }

    cameraControlRuntime.inverted = cameraControlConfig()->inverted;
    cameraControlRuntime.io = IOGetByTag(cameraControlConfig()->ioTag);
    IOInit(cameraControlRuntime.io, OWNER_CAMERA_CONTROL, 0);

    cameraControlInitImpl(&cameraControlRuntime);
}

void cameraControlProcess(timeUs_t currentTimeUs)
{
    if (!cameraControlRuntime.enabled) {
        return;
    }

    if (cameraControlRuntime.endTimeMillis && currentTimeUs / 1000 >= cameraControlRuntime.endTimeMillis) {
        // Key press duration ended, return to idle state
        cameraControlProcessImpl();
        cameraControlRuntime.endTimeMillis = 0;
    }
}

void cameraControlKeyPress(cameraControlKey_e key, uint32_t holdDurationMs)
{
    if (!cameraControlRuntime.enabled) {
        return;
    }

    if (key >= CAMERA_CONTROL_KEYS_COUNT) {
        return;
    }

#ifdef USE_OSD
    // Force OSD timeout so we are alone on the display.
    resumeRefreshAt = 0;
#endif

    cameraControlKeyPressImpl(key, holdDurationMs);
}

#endif // USE_CAMERA_CONTROL
