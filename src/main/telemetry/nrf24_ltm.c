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


#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(TELEMETRY_NRF24_LTM)

#include "common/utils.h"
#include "common/streambuf.h"
#include "telemetry/ltm.h"

static uint8_t sequenceNumber = 0;
static uint8_t ltmPayload[LTM_MAX_MESSAGE_SIZE];

/*
 * Returns a the length of the datagram which forms the NRF24L01 ACK PAYLOAD
 *
 * The datagram consists of 1 byte of header and up to 31 bytes of data
 * The header is a sequence number.
 * The sequence number allows the receiver to detect
 * missing datagrams.
 *
 * The data sent is of the form  <frametype><payload>, that is
 * neither the LTM '$T' header nor the checksum is sent.
 * (LTM messages are of the form $T<frametype><payload><checksum>.)
 *
 */

int getNrf24LtmDatagram(uint8_t *payload, ltm_frame_e ltmFrameType)
{
    sbuf_t ltmPayloadBuf = { .ptr = ltmPayload, .end =ARRAYEND(ltmPayload) };
    sbuf_t * const sbuf = &ltmPayloadBuf;

    switch (ltmFrameType) {
    default:
    case LTM_AFRAME:
        ltm_aframe(sbuf);
        break;
    case LTM_SFRAME:
        ltm_sframe(sbuf);
        break;
#if defined(GPS)
    case LTM_GFRAME:
        ltm_gframe(sbuf);
        break;
    case LTM_OFRAME:
        ltm_oframe(sbuf);
        break;
    case LTM_XFRAME:
        ltm_xframe(sbuf);
        break;
#endif
#if defined(NAV)
    case LTM_NFRAME:
        ltm_nframe(sbuf);
        break;
#endif
    }
    payload[0] =  sequenceNumber++;
    sbufSwitchToReader(sbuf, ltmPayload);
    int ii = 1;
    while (sbufBytesRemaining(sbuf)) {
        payload[ii++] = sbufReadU8(sbuf);
    }
    return ii;
}
#endif
