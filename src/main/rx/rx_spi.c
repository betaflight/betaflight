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

#ifdef USE_RX_SPI

#include "build/build_config.h"

#include "common/utils.h"

#include "config/feature.h"

#include "drivers/rx/rx_spi.h"
#include "drivers/rx/rx_nrf24l01.h"

#include "fc/config.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/cc2500_frsky_common.h"
#include "rx/nrf24_cx10.h"
#include "rx/nrf24_syma.h"
#include "rx/nrf24_v202.h"
#include "rx/nrf24_h8_3d.h"
#include "rx/nrf24_inav.h"
#include "rx/nrf24_kn.h"
#include "rx/flysky.h"


uint16_t rxSpiRcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
STATIC_UNIT_TESTED uint8_t rxSpiPayload[RX_SPI_MAX_PAYLOAD_SIZE];
STATIC_UNIT_TESTED uint8_t rxSpiNewPacketAvailable; // set true when a new packet is received

typedef bool (*protocolInitFnPtr)(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);
typedef rx_spi_received_e (*protocolDataReceivedFnPtr)(uint8_t *payload);
typedef void (*protocolSetRcDataFromPayloadFnPtr)(uint16_t *rcData, const uint8_t *payload);

static protocolInitFnPtr protocolInit;
static protocolDataReceivedFnPtr protocolDataReceived;
static protocolSetRcDataFromPayloadFnPtr protocolSetRcDataFromPayload;

STATIC_UNIT_TESTED uint16_t rxSpiReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel)
{
    BUILD_BUG_ON(NRF24L01_MAX_PAYLOAD_SIZE > RX_SPI_MAX_PAYLOAD_SIZE);

    if (channel >= rxRuntimeConfig->channelCount) {
        return 0;
    }
    if (rxSpiNewPacketAvailable) {
        protocolSetRcDataFromPayload(rxSpiRcData, rxSpiPayload);
        rxSpiNewPacketAvailable = false;
    }
    return rxSpiRcData[channel];
}

STATIC_UNIT_TESTED bool rxSpiSetProtocol(rx_spi_protocol_e protocol)
{
    switch (protocol) {
    default:
#ifdef USE_RX_V202
    case RX_SPI_NRF24_V202_250K:
    case RX_SPI_NRF24_V202_1M:
        protocolInit = v202Nrf24Init;
        protocolDataReceived = v202Nrf24DataReceived;
        protocolSetRcDataFromPayload = v202Nrf24SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_SYMA
    case RX_SPI_NRF24_SYMA_X:
    case RX_SPI_NRF24_SYMA_X5C:
        protocolInit = symaNrf24Init;
        protocolDataReceived = symaNrf24DataReceived;
        protocolSetRcDataFromPayload = symaNrf24SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_CX10
    case RX_SPI_NRF24_CX10:
    case RX_SPI_NRF24_CX10A:
        protocolInit = cx10Nrf24Init;
        protocolDataReceived = cx10Nrf24DataReceived;
        protocolSetRcDataFromPayload = cx10Nrf24SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_H8_3D
    case RX_SPI_NRF24_H8_3D:
        protocolInit = h8_3dNrf24Init;
        protocolDataReceived = h8_3dNrf24DataReceived;
        protocolSetRcDataFromPayload = h8_3dNrf24SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_KN
    case RX_SPI_NRF24_KN:
        protocolInit = knNrf24Init;
        protocolDataReceived = knNrf24DataReceived;
        protocolSetRcDataFromPayload = knNrf24SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_INAV
    case RX_SPI_NRF24_INAV:
        protocolInit = inavNrf24Init;
        protocolDataReceived = inavNrf24DataReceived;
        protocolSetRcDataFromPayload = inavNrf24SetRcDataFromPayload;
        break;
#endif
#if defined(USE_RX_FRSKY_SPI)
#if defined(USE_RX_FRSKY_SPI_D)
    case RX_SPI_FRSKY_D:
#endif
#if defined(USE_RX_FRSKY_SPI_X)
    case RX_SPI_FRSKY_X:
#endif
        protocolInit = frSkySpiInit;
        protocolDataReceived = frSkySpiDataReceived;
        protocolSetRcDataFromPayload = frSkySpiSetRcData;

        break;
#endif // USE_RX_FRSKY_SPI
#ifdef USE_RX_FLYSKY
    case RX_SPI_A7105_FLYSKY:
    case RX_SPI_A7105_FLYSKY_2A:
        protocolInit = flySkyInit;
        protocolDataReceived = flySkyDataReceived;
        protocolSetRcDataFromPayload = flySkySetRcDataFromPayload;
        break;
#endif
    }
    return true;
}

/*
 * Returns true if the RX has received new data.
 * Called from updateRx in rx.c, updateRx called from taskUpdateRxCheck.
 * If taskUpdateRxCheck returns true, then taskUpdateRxMain will shortly be called.
 */
static uint8_t rxSpiFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    if (protocolDataReceived(rxSpiPayload) == RX_SPI_RECEIVED_DATA) {
        rxSpiNewPacketAvailable = true;
        return RX_FRAME_COMPLETE;
    }
    return RX_FRAME_PENDING;
}

/*
 * Set and initialize the RX protocol
 */
bool rxSpiInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    bool ret = false;

    const rx_spi_type_e spiType = feature(FEATURE_SOFTSPI) ? RX_SPI_SOFTSPI : RX_SPI_HARDSPI;
    rxSpiDeviceInit(spiType);
    if (rxSpiSetProtocol(rxConfig->rx_spi_protocol)) {
        ret = protocolInit(rxConfig, rxRuntimeConfig);
    }
    rxSpiNewPacketAvailable = false;
    rxRuntimeConfig->rxRefreshRate = 20000;

    rxRuntimeConfig->rcReadRawFn = rxSpiReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = rxSpiFrameStatus;

    return ret;
}
#endif
