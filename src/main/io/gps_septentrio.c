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

#ifdef USE_GPS_SEPTENTRIO

#include <math.h>
#include <string.h>

#include "common/maths.h"
#include "io/gps.h"
#include "io/gps_septentrio.h"

static sbfParserState_t sbfState;
septentrioPortDetector_t portDetector;

static uint16_t sbfBlockId(const sbfHeader_t *header)
{
    return (uint16_t)(header->id & 0x1FFF);
}

// CRC calculation is performed incrementally as each byte is received,
// allowing early detection of corrupted frames and avoiding the need to store the entire frame before validation.

// This allows handling frames exceeding SBF_MAX_FRAME_SIZE. Until this limit is reached
// (or for smaller frames), the payload is stored in the frame buffer while the CRC is computed progressively. 
// If the frame exceeds SBF_MAX_FRAME_SIZE, additional payload bytes are no longer stored, 
// but the CRC calculation continues until the expected frame length is reached.
// Without this, handling frames larger than SBF_MAX_FRAME_SIZE would require skipping
// CRC validation to store the truncated payload, which could lead to accepting corrupted frames.
static uint16_t sbfAccumulateCrc16(uint16_t crc, uint8_t data)
{
    uint8_t x = (uint8_t)((crc >> 8) ^ data);
    x ^= x >> 4;
    return (uint16_t)((crc << 8) ^ ((uint16_t)x << 12) ^ ((uint16_t)x << 5) ^ x);
}

static uint8_t sbfSvidToGnssId(uint16_t svid)
{
    if (svid >= 1   && svid <= 37)  return SEPTENTRIO_GNSS_GPS;
    if (svid >= 38  && svid <= 61)  return SEPTENTRIO_GNSS_GLONASS;
    if (svid == 62)                 return SEPTENTRIO_GNSS_GLONASS; // GLONASS unknown slot
    if (svid >= 63  && svid <= 68)  return SEPTENTRIO_GNSS_GLONASS;
    if (svid >= 71  && svid <= 106) return SEPTENTRIO_GNSS_GALILEO;
    // 107-119: L-Band MSS, no standard GNSS ID, skip
    if (svid >= 120 && svid <= 140) return SEPTENTRIO_GNSS_SBAS;
    if (svid >= 141 && svid <= 180) return SEPTENTRIO_GNSS_BEIDOU;
    if (svid >= 181 && svid <= 190) return SEPTENTRIO_GNSS_QZSS;
    if (svid >= 191 && svid <= 197) return SEPTENTRIO_GNSS_NAVIC;
    if (svid >= 198 && svid <= 215) return SEPTENTRIO_GNSS_SBAS;
    if (svid >= 216 && svid <= 222) return SEPTENTRIO_GNSS_NAVIC;
    if (svid >= 223 && svid <= 245) return SEPTENTRIO_GNSS_BEIDOU;
    if (svid >= 250 && svid <= 251) return SEPTENTRIO_GNSS_GPS;
    return SEPTENTRIO_GNSS_UNKNOWN; // unknown, same sentinel as unused slot
}

static uint8_t sbfSvidToSatId(uint16_t svid)
{
    if (svid >= 1   && svid <= 37)  return svid;       // GPS G01-G37
    if (svid >= 38  && svid <= 61)  return svid - 37;  // GLONASS R01-R24
    if (svid == 62)                 return SEPTENTRIO_SATID_UNKNOWN; // GLONASS unknown slot
    if (svid >= 63  && svid <= 68)  return svid - 38;  // GLONASS R25-R30
    if (svid >= 71  && svid <= 106) return svid - 70;  // Galileo E01-E36
    // 107-119: L-Band MSS, no standard satellite ID, skip
    if (svid >= 120 && svid <= 140) return svid - 100; // SBAS S20-S40
    if (svid >= 141 && svid <= 180) return svid - 140; // BeiDou C01-C40
    if (svid >= 181 && svid <= 190) return svid - 180; // QZSS J01-J10
    if (svid >= 191 && svid <= 197) return svid - 190; // NavIC I01-I07
    if (svid >= 198 && svid <= 215) return svid - 157; // SBAS S41-S58
    if (svid >= 216 && svid <= 222) return svid - 208; // NavIC I08-I14
    if (svid >= 223 && svid <= 245) return svid - 182; // BeiDou C41-C63
    if (svid >= 250 && svid <= 251) return svid - 212; // GPS G38-G39
    return SEPTENTRIO_SATID_UNKNOWN;
}

void gpsSeptentrioPortDetectorReset(void)
{
    memset(portDetector.rxBuf, 0, sizeof(portDetector.rxBuf));
    portDetector.rxIdx = 0;
    portDetector.isDetected = false;
    portDetector.portName[0] = '\0'; // strictly empty port name (no fallback)
}

static void sbfResetFrame(void)
{
    sbfState.index = 0;
    sbfState.expectedLength = 0;
    sbfState.synced = false;
    sbfState.calculatedCrc = 0;
}

static void sbfResetNavEpoch(void)
{
    sbfState.currentNavTow = 0;
    sbfState.currentNavWnc = 0;
    sbfState.haveNavEpoch = false;
    sbfState.havePvt = false;
    sbfState.haveDop = false;
    sbfState.haveVelCov = false;
    memset(&sbfState.pvt, 0, sizeof(sbfState.pvt));
    memset(&sbfState.dop, 0, sizeof(sbfState.dop));
    memset(&sbfState.velCov, 0, sizeof(sbfState.velCov));
}

static void sbfResetChannelStatus(void)
{
    sbfState.haveChannelStatus = false;
    memset(sbfState.channelStatusPayload, 0, sizeof(sbfState.channelStatusPayload));
    sbfState.channelStatusPayloadLength = 0;
}

void gpsSeptentrioReset(void)
{
    memset(&sbfState, 0, sizeof(sbfState));
    sbfResetFrame();
    sbfResetNavEpoch();
    sbfResetChannelStatus();
}

static void sbfStartNavEpochIfNeeded(uint32_t tow)
{
    // If we have a new TOW, reset the epoch state to start accumulating new data for this epoch
    if (sbfState.haveNavEpoch && tow != sbfState.currentNavTow) {
        sbfState.havePvt = false;
        sbfState.haveDop = false;
        sbfState.haveVelCov = false;
    }
    sbfState.currentNavTow = tow;
    sbfState.haveNavEpoch = true;
}

static bool sbfCommitNavEpoch(void)
{
    if (!sbfState.havePvt) { // PVTGeodetic block is required to commit a navigation epoch to gpsSol
        return false;
    }

    const sbfPvtGeodetic_t *pvt = &sbfState.pvt;
    const uint8_t modeType = (uint8_t)(pvt->mode & 0x0F); // 0: no GNSS PVT available, 1: stand-alone PVT, 2: differential PVT, 3: fixed solution...

    gpsSol.time = sbfState.currentNavTow;
    // A WNc value of 65535 means that the receiver has not determined a valid week number
    if (sbfState.currentNavWnc != UINT16_MAX) {
        gpsWeekTimeToDateTime(&gpsSol.dateTime, (int16_t)sbfState.currentNavWnc, sbfState.currentNavTow, 0);
        gpsSol.dateTime.valid = true;
    } else {
        gpsSol.dateTime.valid = false;
    }
    gpsSol.llh.lat = (int32_t)lround(RADIANS_TO_DEGREES_D(pvt->latitude) * GPS_DEGREES_DIVIDER);
    gpsSol.llh.lon = (int32_t)lround(RADIANS_TO_DEGREES_D(pvt->longitude) * GPS_DEGREES_DIVIDER);
    gpsSol.llh.altCm = (int32_t)lround((pvt->height - (double)pvt->undulation) * 100.0); // subtract the geoid undulation to get height above mean sea level
    gpsSol.numSat = (pvt->nrSv == 255U) ? 0U : pvt->nrSv;

    if (sbfState.haveDop) {
        gpsSol.dop.pdop = sbfState.dop.pDop;
        gpsSol.dop.hdop = sbfState.dop.hDop;
        gpsSol.dop.vdop = sbfState.dop.vDop;
    }

	// Ground speed in cm/s, calculated from the north and east velocity components
    gpsSol.groundSpeed = (uint16_t)lround(sqrt(sq(pvt->vn) + sq(pvt->ve)) * 100.0);
	// 3D speed in cm/s, calculated from the north, east, and up velocity components
    gpsSol.speed3d = (uint16_t)lround(sqrt(sq(pvt->vn) + sq(pvt->ve) + sq(pvt->vu)) * 100.0);

    // Normalize course-over-ground to be within [0, 360) degrees
    if (isnan(pvt->cog) || isinf(pvt->cog) || pvt->cog < -1e9f) {
		// Invalid course value, set to UINT16_MAX as a sentinel (0 being a valid angle).
        // Septentrio marks course-over-ground as invalid when the speed is lower than 0.1 m/s.
        gpsSol.groundCourse = UINT16_MAX;
    } else {
        float courseDeg = fmodf(pvt->cog, 360.0f);
        if (courseDeg < 0.0f) { // ensure course is non-negative
            courseDeg += 360.0f;
        }
        gpsSol.groundCourse = (uint16_t)lroundf(courseDeg * 10.0f);
    }

    gpsSol.velned.velN = (int16_t)lroundf(pvt->vn * 100.0f);
    gpsSol.velned.velE = (int16_t)lroundf(pvt->ve * 100.0f);
    gpsSol.velned.velD = (int16_t)lroundf(-pvt->vu * 100.0f);

    gpsSol.acc.hAcc = (uint32_t)pvt->hAccuracy * 5U;
    gpsSol.acc.vAcc = (uint32_t)pvt->vAccuracy * 5U;
    gpsSol.acc.sAcc = UINT32_MAX;
    if (sbfState.haveVelCov) {
        // SBF does not provide a direct speed accuracy value, but it can be estimated from the velocity covariance matrix.
        // The diagonal elements of the covariance matrix represent the variance of the respective velocity components (vn, ve, vu).
        // The speed accuracy can be approximated as the square root of the maximum variance among these components.
        const float maxVariance = fmaxf(fmaxf(sbfState.velCov.covVnVn, sbfState.velCov.covVeVe), sbfState.velCov.covVuVu);
        if (maxVariance > 0.0f) {
			// The square root of the variance gives the standard deviation (accuracy)
            gpsSol.acc.sAcc = (uint32_t)lroundf(sqrtf(maxVariance) * 1000.0f);
        }
    }
    gpsSol.acc.headAcc = UINT32_MAX; // not provided by SBF with this set of blocks (as only course over ground is available)

    // Calculate the navigation interval based on the current and last epoch timestamps
    const uint64_t weekDurationMs = 7ULL * 24ULL * 3600ULL * 1000ULL;
    const uint64_t currentNavEpochMs = ((uint64_t)sbfState.currentNavWnc * weekDurationMs) + sbfState.currentNavTow;
    if (sbfState.lastNavEpochMs == 0U) {
        gpsSol.navIntervalMs = 100; // default to 100 ms for the first epoch
    } else {
        const uint64_t navDeltaMs = currentNavEpochMs - sbfState.lastNavEpochMs;
        gpsSol.navIntervalMs = (uint32_t)constrain((uint32_t)navDeltaMs, 50, 2500); // see calculateNavInterval() function (gps.c)
    }
    sbfState.lastNavEpochMs = currentNavEpochMs;

    bool hasFix = (modeType != 0U && pvt->error == 0U); // true if a valid GNSS fix is available
    gpsSetFixState(hasFix);

    return true;
}

static void sbfProcessChannelStatus(void)
{
    const sbfChannelStatusHeader_t *header = (const sbfChannelStatusHeader_t *)sbfState.channelStatusPayload;

    // Check that only the actual stored payload may be processed
    const uint16_t availableLength = MIN(sbfState.channelStatusPayloadLength, (uint16_t)sizeof(sbfState.channelStatusPayload));
    if (availableLength < sizeof(*header)
        || header->sb1Length < sizeof(sbfChannelSatInfo_t)
        || header->sb2Length < sizeof(sbfChannelStateInfo_t)) {
        return; // inconsistent block
    }
    const uint8_t *const payloadStart = sbfState.channelStatusPayload;
    const uint8_t *const payloadEnd = sbfState.channelStatusPayload + availableLength;

    const uint8_t *const firstSat = payloadStart + sizeof(*header);
    const uint32_t reportedN = header->n; // number of ChannelSatInfo sub-blocks in this ChannelStatus block (32-bit to avoid truncation)
    const size_t bytesForSats = (size_t)(payloadEnd - firstSat); // remaining bytes available for ChannelSatInfo sub-blocks
    const size_t reportedSatSize = header->sb1Length ? header->sb1Length : 1; // avoid division by zero (should not happen with header checks)

    size_t channelCount = MIN((size_t)reportedN, bytesForSats / reportedSatSize);
    channelCount = MIN(channelCount, GPS_SV_MAXSATS * 2U); // safety cap

    uint8_t svCount = 0; // count of valid satellites processed
    const uint8_t *sat = firstSat; // pointer to the first ChannelSatInfo sub-block

    for (size_t i = 0; i < channelCount; i++) { // loop over the number of satellites reported by the receiver 
        if ((size_t)(payloadEnd - sat) < header->sb1Length) {
            break; // truncated or inconsistent block
        }
        sbfChannelSatInfo_t s1;
        memcpy(&s1, sat, sizeof(s1));

        const uint32_t reportedN2 = (uint32_t)s1.n2; // number of ChannelStateInfo sub-blocks for this satellite
        const size_t remaining = (size_t)(payloadEnd - sat); // remaining bytes available for this satellite's sub-blocks

        // Compute in 64-bit to avoid overflow during multiplication
        const uint64_t satBlockLen64 = (uint64_t)header->sb1Length + (uint64_t)reportedN2 * (uint64_t)header->sb2Length;
        if (satBlockLen64 > remaining) {
            break; // sub-block size exceeds the received payload
        }
        const size_t satBlockLength = (size_t)satBlockLen64; // safe because satBlockLen64 <= remaining <= SBF_MAX_FRAME_SIZE

        // Resolve SVID and system constellation ID
        const uint16_t svid = (s1.svID != 0) ? s1.svID : s1.svidFull;
        const uint8_t gnssId = sbfSvidToGnssId(svid);

        // Health, tracking, and PVT statuses are structured as 2-bit field sequences per frequency band
        // (allowing up to 8 signals per satellite within a uint16_t representation)
        uint8_t maxTrack = 0;   // highest tracking state across all signals
        bool usedInPvt = false; // true if at least one signal is included in the PVT solution
        const uint8_t *state = sat + header->sb1Length;

        // Walk N2 ChannelStateInfo sub-blocks for main antenna
        for (uint32_t j = 0; j < reportedN2; j++) {
            if ((size_t)(payloadEnd - state) < sizeof(sbfChannelStateInfo_t)) {
                break; // truncated or inconsistent block
            }
            sbfChannelStateInfo_t s2;
            memcpy(&s2, state, sizeof(s2));

            if (s2.antenna == 0) { // only evaluate main antenna for quality assessment
                // Iterate through all 8 2-bit signal fields in the 16-bit word
                for (uint8_t shift = 0; shift < 16; shift += 2) {
                    const uint8_t sigTrack = (s2.trackingStatus >> shift) & 0x03; // 0 for idle or not applicable
                    if (sigTrack > maxTrack) { // update maxTrack if this signal has a higher tracking state
                        maxTrack = sigTrack;
                    }
                    const uint8_t sigPvt = (s2.pvtStatus >> shift) & 0x03; // 0 if not used in PVT solution 
                    if (sigPvt == 2) {
                        usedInPvt = true;
                    }
                }
                break;
            }
            state += header->sb2Length; // advance to the next ChannelStateInfo sub-block
        }

        if (gnssId == SEPTENTRIO_GNSS_UNKNOWN || maxTrack == 0) { // unknown constellation or idle/not applicable antenna tracking status
            sat += satBlockLength; // skip to the next ChannelSatInfo sub-block
            continue;
        }

        if (svCount < GPS_SV_MAXSATS) { // only process up to the maximum number of satellites we can store
            GPS_svinfo[svCount].chn = gnssId;
            GPS_svinfo[svCount].svid = sbfSvidToSatId(svid);
            GPS_svinfo[svCount].cno = 0; // not provided in ChannelStatus

            // Build overall satellite quality byte
            uint8_t quality = 0;
            if (maxTrack == 1) {
                quality |= 1; // search
            } else if (maxTrack == 2) {
                quality |= 2; // sync
            } else if (maxTrack == 3) {
                quality |= 5; // tracking
            }
            if (usedInPvt) {
                quality |= (1 << 3);
            }

            // Flags the satellite as healthy if at least one active signal is healthy in s1.health_status
            uint8_t health = 0; // 0 for unknown
            for (uint8_t shift = 0; shift < 16; shift += 2) {
                const uint8_t sigHealth = (s1.healthStatus >> shift) & 0x03;
                if (sigHealth == 1) {
                    health = 1;
                    break;
                } else if (sigHealth == 3) {
                    health = 3;
                }
            }
            if (health == 1) {
                quality |= (1 << 4); // healthy
            } else if (health == 3) {
                quality |= (2 << 4); // unhealthy
            }
            GPS_svinfo[svCount].quality = quality;
            svCount++;
        }
        sat += satBlockLength; // advance to the next ChannelSatInfo sub-block
    }
    GPS_numCh = svCount; // assign final active satellite count 

    // Fill the rest of the array using standard sentinel value (as UBLOX does)
    for (uint8_t i = svCount; i < GPS_SV_MAXSATS; i++) {
        GPS_svinfo[i] = (GPS_svinfo_t){ .chn = 255 };
    }
}

static void sbfProcessBlock(void)
{
    const sbfHeader_t *header = &sbfState.header;
    const uint16_t blockId = sbfBlockId(header);
    const uint8_t *payload = &sbfState.frame[SBF_HEADER_SIZE];
    const uint16_t payloadLength = (uint16_t)(sbfState.expectedLength - SBF_HEADER_SIZE);

    switch (blockId) {
    case SBF_BLOCK_PVTGEODETIC:
        sbfStartNavEpochIfNeeded(header->tow);
        sbfState.currentNavWnc = header->wnc;
        if (payloadLength >= sizeof(sbfPvtGeodetic_t)) {
            memcpy(&sbfState.pvt, payload, sizeof(sbfPvtGeodetic_t));
            sbfState.havePvt = true;
        }
        break;

    case SBF_BLOCK_DOP:
        sbfStartNavEpochIfNeeded(header->tow);
        sbfState.currentNavWnc = header->wnc;
        if (payloadLength >= sizeof(sbfDop_t)) {
            memcpy(&sbfState.dop, payload, sizeof(sbfDop_t));
            sbfState.haveDop = true;
        }
        break;

    case SBF_BLOCK_VELCOVGEODETIC:
        sbfStartNavEpochIfNeeded(header->tow);
        sbfState.currentNavWnc = header->wnc;
        if (payloadLength >= sizeof(sbfVelCovGeodetic_t)) {
            memcpy(&sbfState.velCov, payload, sizeof(sbfVelCovGeodetic_t));
            sbfState.haveVelCov = true;
        }
        break;

    case SBF_BLOCK_ENDOFPVT:
        // Navigation epoch commit handled in gpsNewFrameSeptentrio(uint8_t)
        break;

    case SBF_BLOCK_CHANNELSTATUS:
        if (payloadLength >= sizeof(sbfChannelStatusHeader_t)) {
            const uint16_t availableLength = (uint16_t)MIN((uint32_t)payloadLength, (uint32_t)(SBF_MAX_FRAME_SIZE - SBF_HEADER_SIZE));
            memcpy(sbfState.channelStatusPayload, payload, availableLength);
			// Store the actual copied bytes length for parsing
            sbfState.channelStatusPayloadLength = availableLength;
            sbfState.haveChannelStatus = true;
            sbfProcessChannelStatus();
        }
        break;

    default:
        break;
    }
}

static void gpsSeptentrioProcessAck(uint8_t data)
{
    static uint8_t ackBuf[4]; // circular buffer to hold the last 4 bytes of the ACK/NACK response
    static uint8_t ackIdx = 0;

    ackBuf[ackIdx & 0x3] = data; // & 0x3 equivalent to modulo 4, keeps the index within the bounds of the buffer
    if (ackIdx < 4) {
        ackIdx++; // increment index until we have at least 4 bytes (& 0x3 mask keeps the write position correct)
    } else {
        ackIdx = 4 + ((ackIdx + 1) & 0x3); // keep ackIdx >= 3 while preserving the low two bits
    }

    // The reply to a valid command is the command itself, preceded by "$R: ".
    // The reply to an invalid command starts with "$R? ".
    if (ackIdx >= 3) {
        const uint8_t prev2 = ackBuf[(ackIdx - 3) & 0x3]; // 2 bytes ago
        const uint8_t prev1 = ackBuf[(ackIdx - 2) & 0x3]; // 1 byte ago
        const uint8_t prev0 = ackBuf[(ackIdx - 1) & 0x3]; // current byte

        if (prev2 == '$' && prev1 == 'R' && prev0 == ':') {
            gpsData.ackState = GPS_ACK_GOT_ACK;
            ackIdx = 0;
        } else if (prev2 == '$' && prev1 == 'R' && prev0 == '?') {
            gpsData.ackState = GPS_ACK_GOT_NACK;
            ackIdx = 0;
        } // otherwise, continue scanning for the ACK/NACK sequence in the incoming data stream
    }
}

bool gpsSeptentrioProcessPort(uint8_t data)
{
    if (portDetector.isDetected) { // port already detected, no further processing needed
        return true;
    }
    if (data == 0 || data == '\r') {
        return false;
    }

    // Append byte to sliding buffer
    if (portDetector.rxIdx < SEPTENTRIO_RX_BUF_SIZE - 1) { // ensure space for null terminator
        portDetector.rxBuf[portDetector.rxIdx++] = (char)data;
        portDetector.rxBuf[portDetector.rxIdx] = '\0'; // null-terminate the string 
    } else { // sliding buffer is full, shift left and append new byte
        memmove(portDetector.rxBuf, portDetector.rxBuf + 1, SEPTENTRIO_RX_BUF_SIZE - 2);
        portDetector.rxBuf[SEPTENTRIO_RX_BUF_SIZE - 2] = (char)data;
        portDetector.rxBuf[SEPTENTRIO_RX_BUF_SIZE - 1] = '\0';
    }

    // Match serial and USB port names directly preceding the '>' prompt character
    char *promptPtr = strchr(portDetector.rxBuf, '>');
    // Only search if there is at least one byte before '>' to search through,
    // otherwise promptPtr - 1 would form a pointer before the start of rxBuf (undefined behavior)
    if (promptPtr != NULL && promptPtr > portDetector.rxBuf) {
        // Reverse-search from '>' back to the start of rxBuf to find "COM" or "USB"
        // Future-proofs against the 1-digit port limit of 4-character matching (promptPtr - 4)
        // (e.g., "COM10" or "USB10" will be detected correctly)
        char *searchPtr = promptPtr - 1;
        while (true) { // search backwards until we reach the start of the buffer
            // Check if searchPtr currently points to the start of "COM" or "USB"
            if (strncmp(searchPtr, "COM", 3) == 0 || strncmp(searchPtr, "USB", 3) == 0) {
                size_t nameLen = promptPtr - searchPtr; // length of the port string

                // Ensure the parsed name fits inside the destination buffer
                if (nameLen < SEPTENTRIO_PORT_NAME_LENGTH) {
                    strncpy(portDetector.portName, searchPtr, nameLen);
                    portDetector.portName[nameLen] = '\0';
                    portDetector.isDetected = true;
                    gpsData.ackState = GPS_ACK_GOT_ACK; // port detected, move to next configuration step
                    return true;
                }
            }
            // Stop once we've checked rxBuf itself,
            // decrementing further would form a pointer before the start of rxBuf
            if (searchPtr == portDetector.rxBuf) {
                break;
            }
            searchPtr--;
        }
    }
    return false; // continue accumulating bytes until a valid port name is detected
}

bool gpsNewFrameSeptentrio(uint8_t data)
{
    // Non-SBF data processing (port detection and ACK handling)
    if (gpsData.state == GPS_STATE_CONFIGURE && gpsData.state_position == SEPTENTRIO_CFG_DETECT_PORT) {
        gpsSeptentrioProcessPort(data);
        return false; // continue processing until the port is detected and configuration can proceed
    }
    if (gpsData.state == GPS_STATE_CONFIGURE && gpsData.ackState == GPS_ACK_WAITING) {
        gpsSeptentrioProcessAck(data); // process ACK/NACK responses for configuration commands
        return false;
    }

    // SBF frame processing
    if (!sbfState.synced) { // we are not yet synced, check for the sync sequence
        if (sbfState.index == 0) {
            if (data != SBF_SYNC1) {
                return false;
            }
            sbfState.frame[sbfState.index++] = data; // copy the first sync byte into the frame buffer 
            return false; // wait for the second sync byte
        }

        if (sbfState.index == 1) { // we have received the first sync byte, now check for the second
            if (data != SBF_SYNC2) {
                sbfResetFrame();
                if (data == SBF_SYNC1) { // if the second byte is the first sync byte, start a new frame
                    sbfState.frame[sbfState.index++] = data;
                }
                return false;
            }
            sbfState.frame[sbfState.index++] = data;
            sbfState.synced = true;     // valid sync sequence received, we are now synced
            sbfState.calculatedCrc = 0; // initialize the accumulated CRC for the frame (excluding the sync bytes)
            return false; // wait for the rest of the frame
        }
    }

    if (sbfState.index >= 4) { // CRC accumulation starts after sync and CRC fields (first 4 bytes of the frame)
        sbfState.calculatedCrc = sbfAccumulateCrc16(sbfState.calculatedCrc, data);
    }

    if (sbfState.index < SBF_MAX_FRAME_SIZE) { // only store bytes if we haven't exceeded the allowed maximum size
        sbfState.frame[sbfState.index] = data;
    }
    sbfState.index++; // keep updating the index to count up to the true frame length 

    if (sbfState.index == SBF_HEADER_SIZE) { // get packet length from the header 
        memcpy(&sbfState.header, sbfState.frame, sizeof(sbfHeader_t));
        sbfState.expectedLength = sbfState.header.length;

        if (sbfState.expectedLength < SBF_HEADER_SIZE || sbfState.expectedLength > SBF_MAX_FRAME_SANITY_SIZE) {
            sbfResetFrame();
            return false;
        }
    }

    // Once we have received the expected length of the frame, we can process it
    if (sbfState.expectedLength != 0 && sbfState.index >= sbfState.expectedLength) {
        memcpy(&sbfState.header, sbfState.frame, sizeof(sbfHeader_t));
        
        if (sbfState.calculatedCrc == sbfState.header.crc) { // accumulated CRC matches the expected CRC in the header
            const uint16_t blockId = sbfBlockId(&sbfState.header);
            sbfProcessBlock(); // only the bytes up to MAX_FRAME_SIZE are processed, 
            // any excess bytes are not included in the frame buffer and are ignored
            
            // Detect boundary block to commit the navigation epoch to gpsSol
            if (blockId == SBF_BLOCK_ENDOFPVT) {
                const bool updated = sbfCommitNavEpoch();
                sbfResetNavEpoch();
                sbfResetFrame();
                return updated;
            }
        } // Skip until the next sync sequence is detected, reset the frame state
        sbfResetFrame();
    }
    return false;
}

#endif // USE_GPS_SEPTENTRIO
