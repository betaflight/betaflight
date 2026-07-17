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

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "common/vector.h"

#ifdef USE_OSD_NAV_HUD

// Build-time cap for trail storage; navHudConfig()->breadcrumbCount selects the
// active count up to this limit. 240 points * 6 bytes = 1440 bytes of RAM.
#define NAV_HUD_TRAIL_CAPACITY 240

// Trail points recorded while GPS Rescue was flying, so the renderer can
// distinguish the pre-rescue path from the actual rescue track.
#define NAV_HUD_TRAIL_FLAG_RESCUE 0x01

// Waypoints dropped in flight with the waypoint switch (BOXUSER2 rising edge)
#define NAV_HUD_WAYPOINT_MAX 8

typedef enum {
    NAV_HUD_PHASE_IDLE = 0,      // no rescue in progress
    NAV_HUD_PHASE_INIT,          // RESCUE_INITIALIZE
    NAV_HUD_PHASE_CLIMB,         // RESCUE_ATTAIN_ALT
    NAV_HUD_PHASE_ORIENT,        // RESCUE_PITCH_FORWARD (IMU heading recovery)
    NAV_HUD_PHASE_TURN,          // RESCUE_ROTATE
    NAV_HUD_PHASE_RETURN,        // RESCUE_FLY_HOME
    NAV_HUD_PHASE_DESCEND,       // RESCUE_DESCENT
    NAV_HUD_PHASE_LAND,          // RESCUE_LANDING
    NAV_HUD_PHASE_HOLD,          // RESCUE_DO_NOTHING (engaged at/near home)
    NAV_HUD_PHASE_ABORTED,       // RESCUE_EMERG_DESCENT (sanity failure)
    NAV_HUD_PHASE_UNAVAILABLE,   // rescue configured but not currently possible
    NAV_HUD_PHASE_COUNT
} navHudPhase_e;

typedef struct navHudTrailPoint_s {
    int16_t eastM;               // metres East of home
    int16_t northM;              // metres North of home
    uint8_t flags;               // NAV_HUD_TRAIL_FLAG_*
} navHudTrailPoint_t;

// Snapshot of everything the renderer needs. All positions are local ENU
// centimetres relative to the home point (home is the origin). All headings
// and bearings are compass degrees (0 = North, clockwise positive).
typedef struct navHudState_s {
    // validity / health
    bool gpsValid;               // current 3D fix
    bool homeValid;              // home position captured
    bool positionValid;          // craft position relative to home is usable
    bool headingValid;           // IMU heading trustworthy
    bool headingFromMag;         // a calibrated compass is feeding the heading (valid at hover)
    bool cogValid;               // ground course usable (sufficient ground speed)
    bool headingSuspect;         // heading fights the GPS course while moving: mag misaligned or interfered
    bool gpsGlitch;              // last fix rejected as implausible
    bool stale;                  // no accepted fix for NAV_HUD_STALE_US

    // hold modes
    bool posHoldActive;          // POS_HOLD flight mode engaged
    bool altHoldActive;          // ALT_HOLD flight mode engaged

    // rescue
    bool rescueActive;
    bool rescueHealthy;          // sanity checks passing while rescue is active
    uint8_t rescueFailure;       // rescueFailureState_e value; 0 = healthy
    navHudPhase_e phase;
    bool routeValid;             // rescueStartCm -> home line is usable
    vector2_t rescueStartCm;     // where rescue engaged
    vector2_t targetPosCm;       // autopilot position target ("carrot")
    bool targetPosValid;
    int32_t descentDistanceCm;   // rescue starts descending at this distance from home

    // geometry
    vector2_t craftPosCm;
    uint32_t distanceToHomeCm;
    int16_t bearingToHomeDeg;
    int16_t headingDeg;
    int16_t cogDeg;
    int16_t desiredHeadingDeg;
    int16_t headingErrorDeg;     // wrapped to -180..180; desired - actual
    int32_t crossTrackCm;        // signed, positive right of planned track

    // altitude / speed
    int32_t altitudeCm;
    int32_t targetAltitudeCm;    // valid while rescue is active
    int16_t verticalVelCmS;
    uint16_t groundSpeedCmS;
    uint16_t targetSpeedCmS;     // valid while rescue is active

    // progress
    uint16_t etaSeconds;
    bool etaValid;
    int8_t trackTrend;           // +1 closing on home, -1 opening, 0 steady

    // gps health
    uint8_t satellites;
    uint16_t hdop;               // scaled by 100, i.e. 120 = 1.20
} navHudState_t;

void navHudInit(void);
void navHudOnGpsNewData(void);                 // called from the GPS driver once per accepted fix
void navHudUpdate(timeUs_t currentTimeUs);     // refreshes the state snapshot; cheap, rate-limited internally
const navHudState_t *navHudGetState(void);
void navHudResetTrail(void);

// breadcrumb trail access, index 0 = oldest point
unsigned navHudTrailCount(void);
const navHudTrailPoint_t *navHudTrailPointAt(unsigned index);

// waypoint access, index 0 = first dropped
unsigned navHudWaypointCount(void);
const vector2_t *navHudWaypointAt(unsigned index);
int32_t navHudWaypointAltCm(unsigned index);   // altitude flown when the waypoint was dropped
bool navHudWaypointHintActive(void);           // brief one-time hint after the first drop
uint8_t navHudWaypointDeleteMsg(void);         // waypoint number just deleted (0 = no confirmation)

// pure helpers shared with the renderer and unit tests
int16_t navHudWrapDeg180(int32_t deg);
uint16_t navHudWrapDeg360(int32_t deg);
int16_t navHudBearingDeg(const vector2_t *fromCm, const vector2_t *toCm);
int32_t navHudCrossTrackCm(const vector2_t *lineStartCm, const vector2_t *craftCm);
uint32_t navHudSelectAutoScaleM(uint32_t requiredWidthM);

#endif // USE_OSD_NAV_HUD
