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

#include "dshot_bitbang_impl.h"

void bbDshotRequestTelemetry(unsigned motorIndex)
{
    if (motorIndex >= ARRAYLEN(bbMotors)) {
        return;
    }
    bbMotor_t *const bbmotor = &bbMotors[motorIndex];

    if (!bbmotor->configured) {
        return;
    }
    bbmotor->protocolControl.requestTelemetry = true;
}

bool bbDshotIsMotorIdle(unsigned motorIndex)
{
    if (motorIndex >= ARRAYLEN(bbMotors)) {
        return false;
    }

    bbMotor_t *const bbmotor = &bbMotors[motorIndex];
    return bbmotor->protocolControl.value == 0;
}

#ifdef USE_DSHOT_BITBANG
bool isDshotBitbangActive(const motorDevConfig_t *motorDevConfig)
{
#if defined(STM32F4) || defined(APM32F4)
    return motorDevConfig->useDshotBitbang == DSHOT_BITBANG_ON ||
        (motorDevConfig->useDshotBitbang == DSHOT_BITBANG_AUTO && motorDevConfig->useDshotTelemetry && motorDevConfig->motorProtocol != MOTOR_PROTOCOL_PROSHOT1000);
#else
    return motorDevConfig->useDshotBitbang == DSHOT_BITBANG_ON ||
        (motorDevConfig->useDshotBitbang == DSHOT_BITBANG_AUTO && motorDevConfig->motorProtocol != MOTOR_PROTOCOL_PROSHOT1000);
#endif
}
#endif
