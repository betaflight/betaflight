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

#include <platform.h>
#include "build_config.h"

#ifdef USE_RX_NRF24
#include "drivers/rx_nrf24l01.h"
#include "rx/rx.h"
#include "rx/nrf24.h"
#include "rx/nrf24_cx10.h"
#include "rx/nrf24_syma.h"
#include "rx/nrf24_v202.h"
#include "rx/nrf24_h8_3d.h"


uint16_t nrf24RcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
STATIC_UNIT_TESTED uint8_t nrf24Payload[NRF24L01_MAX_PAYLOAD_SIZE];
STATIC_UNIT_TESTED uint8_t nrf24NewPacketAvailable; // set true when a new packet is received

typedef void (*protocolInitPtr)(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);
typedef nrf24_received_t (*protocolDataReceivedPtr)(uint8_t *payload);
typedef void (*protocolSetRcDataFromPayloadPtr)(uint16_t *rcData, const uint8_t *payload);

static protocolInitPtr protocolInit;
static protocolDataReceivedPtr protocolDataReceived;
static protocolSetRcDataFromPayloadPtr protocolSetRcDataFromPayload;

STATIC_UNIT_TESTED uint16_t rxNrf24ReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel)
{
    if (channel >= rxRuntimeConfig->channelCount) {
        return 0;
    }
    if (nrf24NewPacketAvailable) {
        protocolSetRcDataFromPayload(nrf24RcData, nrf24Payload);
        nrf24NewPacketAvailable = false;
    }
    return nrf24RcData[channel];
}

STATIC_UNIT_TESTED bool rxNrf24SetProtocol(nrf24_protocol_t protocol)
{
    switch (protocol) {
#ifdef USE_RX_V202
    case NRF24RX_V202_250K:
    case NRF24RX_V202_1M:
        protocolInit = v202Init;
        protocolDataReceived = v202DataReceived;
        protocolSetRcDataFromPayload = v202SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_SYMA
    case NRF24RX_SYMA_X:
    case NRF24RX_SYMA_X5C:
        protocolInit = symaInit;
        protocolDataReceived = symaDataReceived;
        protocolSetRcDataFromPayload = symaSetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_CX10
    case NRF24RX_CX10:
    case NRF24RX_CX10A:
        protocolInit = cx10Init;
        protocolDataReceived = cx10DataReceived;
        protocolSetRcDataFromPayload = cx10SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_H8_3D
    case NRF24RX_H8_3D:
        protocolInit = h8_3dInit;
        protocolDataReceived = h8_3dDataReceived;
        protocolSetRcDataFromPayload = h8_3dSetRcDataFromPayload;
        break;
#endif
    default:
        return false;
        break;
    }
    return true;
}

/*
 * Returns true if the NRF24L01 has received new data.
 * Called from updateRx in rx.c, updateRx called from taskUpdateRxCheck.
 * If taskUpdateRxCheck returns true, then taskUpdateRxMain will shortly be called.
 */
bool rxNrf24DataReceived(void)
{
    if (protocolDataReceived(nrf24Payload) == NRF24_RECEIVED_DATA) {
        nrf24NewPacketAvailable = true;
        return true;
    }
    return false;
}

/*
 * Set and initialize the NRF24 protocol
 */
bool rxNrf24Init(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    bool ret = false;
    NRF24L01_SpiInit();
    if (rxNrf24SetProtocol(rxConfig->nrf24rx_protocol)) {
        protocolInit(rxConfig, rxRuntimeConfig);
        ret = true;
    }
    nrf24NewPacketAvailable = false;
    if (callback) {
        *callback = rxNrf24ReadRawRC;
    }
    return ret;
}
#endif
