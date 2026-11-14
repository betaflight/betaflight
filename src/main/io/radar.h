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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

#include "io/gps.h"

/*
 * Peer aircraft reported by an external radar module (FormationFlight or the
 * older ESP32 INAV-Radar) over MSP2_COMMON_SET_RADAR_POS. The message layout is
 * the one INAV defined, so the same modules work unchanged with Betaflight.
 */

enum {
    RADAR_MAX_PEERS = 8,            // wire ids 1..RADAR_MAX_PEERS are kept, others are ignored
    RADAR_PEER_TIMEOUT_MS = 5000,   // FormationFlight v2 stops sending a lost peer instead of flagging it
    RADAR_POS_PAYLOAD_SIZE = 19,
};

/* Peer state as sent by the module */
typedef enum {
    RADAR_PEER_STATE_UNDEFINED = 0,
    RADAR_PEER_STATE_ARMED = 1,
    RADAR_PEER_STATE_LOST = 2,
} radarPeerState_e;

/* Last reported position of one peer */
typedef struct radarPeer_s {
    gpsLocation_t llh;          // lat/lon * 1e7, altitude MSL in cm
    uint16_t heading;           // degrees
    uint16_t speed;             // cm/s
    uint8_t state;              // radarPeerState_e
    uint8_t lq;                 // link quality 0..4
    timeMs_t lastUpdateMs;      // 0 = never received
} radarPeer_t;

/* Camera and canvas geometry used to place peers on the HUD */
typedef struct radarHudGeometry_s {
    uint8_t cols;
    uint8_t rows;
    uint8_t cameraFovH;         // degrees
    uint8_t cameraFovV;         // degrees
    int8_t cameraUptilt;        // degrees, camera tilted up relative to the frame
    uint8_t marginH;            // columns kept clear at each side
    uint8_t marginV;            // rows kept clear at top and bottom
} radarHudGeometry_t;

/*
 * Stores one MSP2_COMMON_SET_RADAR_POS payload. Returns false if the payload is
 * too short; a complete payload for an id outside 1..RADAR_MAX_PEERS is ignored.
 */
bool radarReceivePos(const uint8_t *payload, unsigned len, timeMs_t nowMs);

/* Returns the peer with a 1-based id if it has a position and is neither lost nor stale, else NULL */
const radarPeer_t *radarFindPeer(uint8_t id, timeMs_t nowMs);

/* Returns the next usable peer id after currentId (wrapping), currentId if it is the only one, 0 if none */
uint8_t radarGetNextPeerId(uint8_t currentId, timeMs_t nowMs);

/* Returns the letter shown on the OSD for a peer id: 1 = 'A', 2 = 'B', ... */
char radarGetPeerLetter(uint8_t id);

/*
 * Horizontal distance (cm) and bearing (centidegrees, clockwise from north) from one
 * position to another. Longitude is scaled by the latitude of the first position rather
 * than by GPS_cosLat, which is only set once a home position is recorded.
 */
void radarCalcDistanceBearing(uint32_t *distanceCm, int32_t *bearingCentiDeg,
                              const gpsLocation_t *from, const gpsLocation_t *to);

/*
 * Places a peer on the HUD the way INAV's osdHudDrawPoi does. relBearingDeg is the
 * direction to the peer relative to the nose (-180..180, positive = right), altDiffM
 * is the peer's altitude above ours and pitchDeg is positive nose down. Returns true
 * when the peer is inside both the camera view and the HUD area; otherwise the peer
 * is parked on the left or right HUD edge, one row above the centre.
 */
bool radarHudProject(int *col, int *row, const radarHudGeometry_t *geometry,
                     int relBearingDeg, uint32_t distanceM, int32_t altDiffM, int pitchDeg);

/* Wraps an angle in degrees to -180..180 */
int radarWrapAngle180(int angleDeg);

#ifdef UNIT_TEST
void radarResetForTest(void);
#endif
