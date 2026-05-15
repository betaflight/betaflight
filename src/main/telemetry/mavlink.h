/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/time.h"

void initMAVLinkTelemetry(void);
void handleMAVLinkTelemetry(void);
void checkMAVLinkTelemetryState(void);

void freeMAVLinkTelemetryPort(void);
void configureMAVLinkTelemetryPort(void);

// Forward-typedef so consumers can hold pointers without dragging in
// common/mavlink.h (which carries -Wpedantic-noisy unnamed unions).
typedef struct __mavlink_message mavlink_message_t;

// Pack a fully-formed mavlink_message_t into the shared TX buffer and write it
// to the open MAVLink serial port. Implemented in telemetry/mavlink.c; shared
// with telemetry/mavlink_mission.c so the mission module reuses the same
// buffer/port path.
void mavlinkSendMessage(mavlink_message_t *msg);

typedef struct mavlinkTelemetryStream_s {
    uint8_t rate;
    timeMs_t updateTime;
    void (*const streamFunc)(void);
} mavlinkTelemetryStream_t;
