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

#pragma once

#define MAVLINK_COMM_NUM_BUFFERS 1

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

void mavlinkRxHandleMessage(const mavlink_rc_channels_override_t *msg);
bool mavlinkRxInit(const rxConfig_t *initialRxConfig, rxRuntimeState_t *rxRuntimeState);
bool isValidMavlinkTxBuffer (void);
bool shouldSendMavlinkTelemetry(void);
