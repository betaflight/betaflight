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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "platform.h"

#include "common/axis.h"

#include "drivers/io.h"

#include "fc/rc_controls.h"

#include "pg/headtracker.h"

#include "rx/rx.h"
#include "rx/sbus.h"

#include "sensors/gyro.h"

#include "flight/imu.h"

#if defined(USE_HEADTRACKER)
static int16_t yawOffset = 1800;
static IO_t headtrackerIO;

#define SHIMMY_PERIOD 500000 // 500ms

void headtrackerYawReset(void)
{
    // Capture the offset to express current yaw as 0 degrees
    yawOffset = (1800 - attitude.values.yaw + 3600) % 3600;
}

bool headtrackerInit(void)
{
    headtrackerIO = IOGetByTag(headtrackerConfig()->headtracker_ioTag);

    // Initialise the resource pin used for yaw reset
    if (headtrackerIO) {
        IOInit(headtrackerIO, OWNER_HEADTRACKER, 0);
        IOConfigGPIO(headtrackerIO, IOCFG_IPU);
    }

    return true;
}

void taskHeadtracker(uint32_t currentTime)
{
    UNUSED(currentTime);
    int16_t angles[3];
    uint16_t channels[3];
    float yawGyro = gyroGetFilteredDownsampled(YAW);
    static int8_t shimmyCount = 0;
    static uint32_t shimmyStartTime = 0;

    // Check for a head shimmy to reset yaw
    if (headtrackerConfig()->headtracker_yaw_shimmy_enable) {
        uint16_t shimmyAmplitude = headtrackerConfig()->headtracker_yaw_shimmy_amplitude;

        // Check for a head shimmy to reset the yaw
        if (cmpTimeUs(currentTime, shimmyStartTime) > SHIMMY_PERIOD) {
            // Shimmy phase timed out so reset
            shimmyCount = 0;
        }

        // Look for alternate yaw directions in rapid succession exceeding a given amplitude
        bool odd = shimmyCount % 2;

        if ((odd && (yawGyro > shimmyAmplitude)) || (!odd && (yawGyro < -shimmyAmplitude))) {
            shimmyStartTime = currentTime;
            shimmyCount++;
        }

        // Check if the required number of shakes have been seen
        if (shimmyCount == headtrackerConfig()->headtracker_yaw_shimmy_count) {
            shimmyCount = 0;
            headtrackerYawReset();
        }
    }

    // Check for headtracker reset switch activation
    if (headtrackerIO && !IORead(headtrackerIO)) {
        shimmyCount = 0;
        headtrackerYawReset();
    }

    // Express a level attitude as 0 degrees RPY
    angles[ROLL] = attitude.values.roll + 1800;
    angles[PITCH] = attitude.values.pitch + 1800;
    angles[YAW] = (attitude.values.yaw + yawOffset) % 3600;

    // Limit the max angle of delection
    if (headtrackerConfig()->headtracker_max_angle) {
        const int newValueMin = 1800 - headtrackerConfig()->headtracker_max_angle * 10;
        const int newValueMax = 1800 + headtrackerConfig()->headtracker_max_angle * 10;
        for (int channel = 0; channel < 3; channel++) {
            angles[channel] = scaleRange(angles[channel], newValueMin, newValueMax, 0, 3600);
            angles[channel] = constrain(angles[channel], 0, 3600);
        }
    }

    // Convert the decidegrees value to the 11 bit channel value
    for (int channel = 0; channel < 3; channel++) {
        channels[channel] = angles[channel] * 2048 / 3600;
    }

    // Use the SBus output to send attitude information for headtracking
    sbusTx(channels, 3);
}
#endif



