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

#include "drivers/rx_nrf24l01.h"

#include "config/feature.h"

#include "fc/config.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/eleres.h"
#include "rx/nrf24_cx10.h"
#include "rx/nrf24_syma.h"
#include "rx/nrf24_v202.h"
#include "rx/nrf24_h8_3d.h"
#include "rx/nrf24_inav.h"


uint16_t rxSpiRcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
STATIC_UNIT_TESTED uint8_t rxSpiPayload[RX_SPI_MAX_PAYLOAD_SIZE];
STATIC_UNIT_TESTED uint8_t rxSpiNewPacketAvailable; // set true when a new packet is received

typedef void (*protocolInitPtr)(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);
typedef rx_spi_received_e (*protocolDataReceivedPtr)(uint8_t *payload);
typedef void (*protocolSetRcDataFromPayloadPtr)(uint16_t *rcData, const uint8_t *payload);

static protocolInitPtr protocolInit;
static protocolDataReceivedPtr protocolDataReceived;
static protocolSetRcDataFromPayloadPtr protocolSetRcDataFromPayload;

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
    case NRF24RX_V202_250K:
    case NRF24RX_V202_1M:
        protocolInit = v202Nrf24Init;
        protocolDataReceived = v202Nrf24DataReceived;
        protocolSetRcDataFromPayload = v202Nrf24SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_SYMA
    case NRF24RX_SYMA_X:
    case NRF24RX_SYMA_X5C:
        protocolInit = symaNrf24Init;
        protocolDataReceived = symaNrf24DataReceived;
        protocolSetRcDataFromPayload = symaNrf24SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_CX10
    case NRF24RX_CX10:
    case NRF24RX_CX10A:
        protocolInit = cx10Nrf24Init;
        protocolDataReceived = cx10Nrf24DataReceived;
        protocolSetRcDataFromPayload = cx10Nrf24SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_H8_3D
    case NRF24RX_H8_3D:
        protocolInit = h8_3dNrf24Init;
        protocolDataReceived = h8_3dNrf24DataReceived;
        protocolSetRcDataFromPayload = h8_3dNrf24SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_INAV
    case NRF24RX_INAV:
        protocolInit = inavNrf24Init;
        protocolDataReceived = inavNrf24DataReceived;
        protocolSetRcDataFromPayload = inavNrf24SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_ELERES
    case RFM22_ELERES:
        protocolInit = eleresInit;
        protocolDataReceived = eleresDataReceived;
        protocolSetRcDataFromPayload = eleresSetRcDataFromPayload;
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
uint8_t rxSpiFrameStatus(void)
{
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
        protocolInit(rxConfig, rxRuntimeConfig);
        ret = true;
    }
    rxRuntimeConfig->rxRefreshRate = 10000;
    rxSpiNewPacketAvailable = false;
    rxRuntimeConfig->rcReadRawFn = rxSpiReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = rxSpiFrameStatus;
    return ret;
}
#endif
