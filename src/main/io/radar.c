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

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RADAR

#include "common/maths.h"

#include "io/radar.h"

static radarPeer_t radarPeers[RADAR_MAX_PEERS];

static uint16_t readU16(const uint8_t *p)
{
    return p[0] | (p[1] << 8);
}

static uint32_t readU32(const uint8_t *p)
{
    return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

static bool isPeerIdValid(uint8_t id)
{
    return id >= 1 && id <= RADAR_MAX_PEERS;
}

static bool isPeerUsable(const radarPeer_t *peer, timeMs_t nowMs)
{
    return peer->lastUpdateMs != 0
        && cmpTimeMs(nowMs, peer->lastUpdateMs) <= RADAR_PEER_TIMEOUT_MS
        && peer->state != RADAR_PEER_STATE_LOST
        && (peer->llh.lat != 0 || peer->llh.lon != 0);
}

static bool isPositionValid(int32_t lat, int32_t lon)
{
    return lat >= -90 * GPS_DEGREES_DIVIDER && lat <= 90 * GPS_DEGREES_DIVIDER
        && lon >= -180 * GPS_DEGREES_DIVIDER && lon <= 180 * GPS_DEGREES_DIVIDER;
}

/* Payload: id(u8) state(u8) lat(i32) lon(i32) alt_cm(i32) heading_deg(u16) speed_cms(u16) lq(u8) */
bool radarReceivePos(const uint8_t *payload, unsigned len, timeMs_t nowMs)
{
    const bool isComplete = (len >= RADAR_POS_PAYLOAD_SIZE);

    if (isComplete && isPeerIdValid(payload[0])) {
        const int32_t lat = (int32_t)readU32(&payload[2]);
        const int32_t lon = (int32_t)readU32(&payload[6]);

        if (!isPositionValid(lat, lon)) {
            return isComplete;
        }

        radarPeer_t *peer = &radarPeers[payload[0] - 1];

        peer->state = payload[1];
        peer->llh.lat = lat;
        peer->llh.lon = lon;
        peer->llh.altCm = (int32_t)readU32(&payload[10]);
        peer->heading = readU16(&payload[14]);
        peer->speed = readU16(&payload[16]);
        peer->lq = payload[18];
        // 0 is reserved for "never received"
        peer->lastUpdateMs = (nowMs != 0) ? nowMs : 1;
    }

    return isComplete;
}

const radarPeer_t *radarFindPeer(uint8_t id, timeMs_t nowMs)
{
    const radarPeer_t *peer = NULL;

    if (isPeerIdValid(id) && isPeerUsable(&radarPeers[id - 1], nowMs)) {
        peer = &radarPeers[id - 1];
    }

    return peer;
}

uint8_t radarGetNextPeerId(uint8_t currentId, timeMs_t nowMs)
{
    uint8_t nextId = 0;

    for (unsigned i = 1; i <= RADAR_MAX_PEERS && nextId == 0; i++) {
        const uint8_t id = (currentId + i - 1) % RADAR_MAX_PEERS + 1;
        if (radarFindPeer(id, nowMs)) {
            nextId = id;
        }
    }

    return nextId;
}

char radarGetPeerLetter(uint8_t id)
{
    return (isPeerIdValid(id)) ? 'A' + id - 1 : '-';
}

void radarCalcDistanceBearing(uint32_t *distanceCm, int32_t *bearingCentiDeg,
                              const gpsLocation_t *from, const gpsLocation_t *to)
{
    const float cosLat = cos_approx(DEGREES_TO_RADIANS((float)from->lat / GPS_DEGREES_DIVIDER));
    // 64-bit: a longitude difference can reach 360 degrees, which overflows int32_t
    int64_t deltaLon = (int64_t)to->lon - from->lon;
    // Take the short way round when the two positions are on either side of the 180 degree meridian
    if (deltaLon > 180LL * GPS_DEGREES_DIVIDER) {
        deltaLon -= 360LL * GPS_DEGREES_DIVIDER;
    } else if (deltaLon < -180LL * GPS_DEGREES_DIVIDER) {
        deltaLon += 360LL * GPS_DEGREES_DIVIDER;
    }
    const float dLat = (float)((int64_t)to->lat - from->lat) * EARTH_ANGLE_TO_CM;
    const float dLon = (float)deltaLon * cosLat * EARTH_ANGLE_TO_CM;
    int32_t bearing = lrintf(9000.0f - RADIANS_TO_DEGREES(atan2_approx(dLat, dLon)) * 100.0f);

    if (bearing < 0) {
        bearing += 36000;
    }

    *distanceCm = sqrtf(sq(dLat) + sq(dLon));
    *bearingCentiDeg = bearing % 36000;
}

bool radarHudProject(int *col, int *row, const radarHudGeometry_t *geometry,
                     int relBearingDeg, uint32_t distanceM, int32_t altDiffM, int pitchDeg)
{
    const int minX = geometry->marginH + 2;
    const int maxX = geometry->cols - geometry->marginH - 3;
    const int minY = geometry->marginV;
    const int maxY = geometry->rows - geometry->marginV - 2;
    const int centreX = geometry->cols / 2;
    const int centreY = geometry->rows / 2;
    const int halfFovH = geometry->cameraFovH / 2;
    const int halfFovV = geometry->cameraFovV / 2;
    bool isInView = false;

    *col = (relBearingDeg > 0) ? maxX : minX;
    *row = centreY - 1;

    if (relBearingDeg > -halfFovH && relBearingDeg < halfFovH) {
        const float scaledX = sin_approx(DEGREES_TO_RADIANS(relBearingDeg)) / sin_approx(DEGREES_TO_RADIANS(halfFovH));
        // INAV scales by a fixed 15 columns (a 30 column SD canvas); half the canvas keeps HD in proportion
        const int x = centreX + lrintf((geometry->cols / 2) * scaledX);

        if (x >= minX && x <= maxX) {
            const float peerAngleDeg = RADIANS_TO_DEGREES(atan2_approx(-altDiffM, distanceM));
            const float errorY = peerAngleDeg - pitchDeg + geometry->cameraUptilt;
            const float scaledY = sin_approx(DEGREES_TO_RADIANS(errorY)) / sin_approx(DEGREES_TO_RADIANS(halfFovV));

            *col = x;
            *row = constrain(centreY + lrintf((geometry->rows / 2) * scaledY), minY, maxY - 1);
            isInView = true;
        }
    }

    return isInView;
}

int radarWrapAngle180(int angleDeg)
{
    int wrapped = angleDeg % 360;

    if (wrapped > 180) {
        wrapped -= 360;
    } else if (wrapped < -180) {
        wrapped += 360;
    }

    return wrapped;
}

#ifdef UNIT_TEST
void radarResetForTest(void)
{
    memset(radarPeers, 0, sizeof(radarPeers));
}
#endif

#endif // USE_RADAR
