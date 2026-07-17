/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef USE_NAV_MISSION

#include "common/time.h"

typedef enum {
    NAV_MISSION_IDLE = 0,     // mission switch off
    NAV_MISSION_FLYING,       // flying the waypoint list nose-first
    NAV_MISSION_DONE,         // final waypoint reached; holding there
    NAV_MISSION_ABORTED,      // refused or interrupted; requires a switch cycle to re-engage
} navMissionPhase_e;

// called every position-hold task cycle (armed or not); owns all engagement logic
void navMissionUpdate(timeUs_t currentTimeUs);

// true while the mission owns the aircraft (FLYING or holding at DONE):
// gates the autopilot angle/throttle/yaw injections exactly like a hold mode
bool navMissionIsControlling(void);
// nose-first steering: yaw rate toward the active waypoint, rescue-style
float navMissionGetYawRateDps(void);
// 3D missions: the mission owns the altitude-hold target while these are active
bool navMissionAltitudeActive(void);
bool navMissionIsDescending(void);
float navMissionGetTargetAltitudeCm(void);
float navMissionGetTargetClimbRateCmS(void);
// latched: position control failed twice within the retry window; the
// mission aborted to wings-level and the pilot must take over
bool navMissionFailed(void);

navMissionPhase_e navMissionGetPhase(void);
unsigned navMissionActiveIndex(void);

#else

#define navMissionIsControlling() false
#define navMissionFailed() false

#endif // USE_NAV_MISSION
