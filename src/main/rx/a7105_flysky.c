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

#include <string.h>

#include "platform.h"

#ifdef USE_RX_FLYSKY

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/rx/rx_a7105.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "config/config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx_spi.h"

#include "rx/a7105_flysky_defs.h"
#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/rx_spi_common.h"

#include "sensors/battery.h"

#include "a7105_flysky.h"

#if FLYSKY_CHANNEL_COUNT > MAX_FLYSKY_CHANNEL_COUNT
#error "FlySky AFHDS protocol support 8 channel max"
#endif

#if FLYSKY_2A_CHANNEL_COUNT > MAX_FLYSKY_2A_CHANNEL_COUNT
#error "FlySky AFHDS 2A protocol support 14 channel max"
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(flySkyConfig_t, flySkyConfig, PG_FLYSKY_CONFIG, 1);
PG_RESET_TEMPLATE(flySkyConfig_t, flySkyConfig, .txId = 0, .rfChannelMap = {0});

static const uint8_t flySkyRegs[] = {
    0xff, 0x42, 0x00, 0x14, 0x00, 0xff, 0xff, 0x00,
    0x00, 0x00, 0x00, 0x03, 0x19, 0x05, 0x00, 0x50,
    0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00,
    0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,
    0x13, 0xc3, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00,
    0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
    0x01, 0x0f
};

static const uint8_t flySky2ARegs[] = {
    0xff, 0x62, 0x00, 0x25, 0x00, 0xff, 0xff, 0x00,
    0x00, 0x00, 0x00, 0x03, 0x19, 0x05, 0x00, 0x50,
    0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x4f,
    0x62, 0x80, 0xff, 0xff, 0x2a, 0x32, 0xc3, 0x1f,
    0x1e, 0xff, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00,
    0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
    0x01, 0x0f
};

static const uint8_t flySky2ABindChannels[] = {
    0x0D, 0x8C
};

static const uint8_t flySkyRfChannels[16][16] = {
    {   0x0a, 0x5a, 0x14, 0x64, 0x1e, 0x6e, 0x28, 0x78, 0x32, 0x82, 0x3c, 0x8c, 0x46, 0x96, 0x50, 0xa0},
    {   0xa0, 0x50, 0x96, 0x46, 0x8c, 0x3c, 0x82, 0x32, 0x78, 0x28, 0x6e, 0x1e, 0x64, 0x14, 0x5a, 0x0a},
    {   0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x46, 0x96, 0x1e, 0x6e, 0x3c, 0x8c, 0x28, 0x78, 0x32, 0x82},
    {   0x82, 0x32, 0x78, 0x28, 0x8c, 0x3c, 0x6e, 0x1e, 0x96, 0x46, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a},
    {   0x28, 0x78, 0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96},
    {   0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a, 0x78, 0x28},
    {   0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96, 0x14, 0x64},
    {   0x64, 0x14, 0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50},
    {   0x50, 0xa0, 0x46, 0x96, 0x3c, 0x8c, 0x28, 0x78, 0x0a, 0x5a, 0x32, 0x82, 0x1e, 0x6e, 0x14, 0x64},
    {   0x64, 0x14, 0x6e, 0x1e, 0x82, 0x32, 0x5a, 0x0a, 0x78, 0x28, 0x8c, 0x3c, 0x96, 0x46, 0xa0, 0x50},
    {   0x46, 0x96, 0x3c, 0x8c, 0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
    {   0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50, 0x8c, 0x3c, 0x96, 0x46},
    {   0x46, 0x96, 0x0a, 0x5a, 0x3c, 0x8c, 0x14, 0x64, 0x50, 0xa0, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82},
    {   0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0xa0, 0x50, 0x64, 0x14, 0x8c, 0x3c, 0x5a, 0x0a, 0x96, 0x46},
    {   0x46, 0x96, 0x0a, 0x5a, 0x50, 0xa0, 0x3c, 0x8c, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
    {   0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0x8c, 0x3c, 0xa0, 0x50, 0x5a, 0x0a, 0x96, 0x46}
};

const timings_t flySkyTimings = {.packet = 1500, .firstPacket = 1900, .syncPacket = 2250, .telemetry = 0xFFFFFFFF};
const timings_t flySky2ATimings = {.packet = 3850, .firstPacket = 4850, .syncPacket = 5775, .telemetry = 57000};

static rx_spi_protocol_e protocol = RX_SPI_A7105_FLYSKY_2A;
static const timings_t *timings = &flySky2ATimings;
static uint32_t timeout = 0;
static uint32_t timeLastPacket = 0;
static uint32_t timeLastBind = 0;
static uint32_t timeTxRequest = 0;
static uint32_t countTimeout = 0;
static uint32_t countPacket = 0;
static uint32_t txId = 0;
static uint32_t rxId = 0;
static bool bound = false;
static bool sendTelemetry = false;
static bool waitTx = false;
static uint16_t errorRate = 0;
static uint16_t rssi_dBm = 0;
static uint8_t rfChannelMap[FLYSKY_FREQUENCY_COUNT] = {0};

static uint8_t getNextChannel(uint8_t step)
{
    static uint8_t channel = 0;
    channel = (channel + step) & 0x0F;
    return rfChannelMap[channel];
}

static void flySkyCalculateRfChannels(void)
{
    uint32_t channelRow = txId & 0x0F;
    uint32_t channelOffset = ((txId & 0xF0) >> 4) + 1;

    if (channelOffset > 9) {
        channelOffset = 9; // from sloped soarer findings, bug in flysky protocol
    }

    for (unsigned i = 0; i < FLYSKY_FREQUENCY_COUNT; i++) {
        rfChannelMap[i] = flySkyRfChannels[channelRow][i] - channelOffset;
    }
}

static void resetTimeout(const uint32_t timeStamp)
{
    timeLastPacket = timeStamp;
    timeout = timings->firstPacket;
    countTimeout = 0;
    countPacket++;
}

static void checkTimeout(void)
{
    static uint32_t timeMeasuareErrRate = 0;
    static uint32_t timeLastTelemetry = 0;
    uint32_t time = micros();

    if ((time - timeMeasuareErrRate) > (101 * timings->packet)) {
        errorRate = (countPacket >= 100) ? (0) : (100 - countPacket);
        countPacket = 0;
        timeMeasuareErrRate = time;
    }

    if ((time - timeLastTelemetry) > timings->telemetry) {
        timeLastTelemetry = time;
        sendTelemetry = true;
    }

    if ((time - timeLastPacket) > timeout) {
        uint32_t stepOver = (time - timeLastPacket) / timings->packet;

        timeLastPacket = (stepOver > 1) ? (time) : (timeLastPacket + timeout);

        A7105Strobe(A7105_STANDBY);
        A7105WriteReg(A7105_0F_CHANNEL, getNextChannel(stepOver % FLYSKY_FREQUENCY_COUNT));
        A7105Strobe(A7105_RX);

        if(countTimeout > 31) {
            timeout = timings->syncPacket;
            setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
        } else {
            timeout = timings->packet;
            countTimeout++;
        }
    }
}

static void checkRSSI(void)
{
    static uint8_t buf[FLYSKY_RSSI_SAMPLE_COUNT] = {0};
    static int16_t sum = 0;
    static uint16_t currentIndex = 0;

    uint8_t adcValue = A7105ReadReg(A7105_1D_RSSI_THOLD);

    sum += adcValue;
    sum -= buf[currentIndex];
    buf[currentIndex] = adcValue;
    currentIndex = (currentIndex + 1) % FLYSKY_RSSI_SAMPLE_COUNT;

    rssi_dBm = 50 + sum / (3 * FLYSKY_RSSI_SAMPLE_COUNT); // range about [95...52], -dBm

    int16_t tmp = 2280 - 24 * rssi_dBm; // convert to [0...1023]
    setRssiDirect(tmp, RSSI_SOURCE_RX_PROTOCOL);
}

static bool isValidPacket(const uint8_t *packet)
{
    const flySky2ARcDataPkt_t *rcPacket = (const flySky2ARcDataPkt_t*) packet;
    return (rcPacket->rxId == rxId && rcPacket->txId == txId);
}

static void buildAndWriteTelemetry(uint8_t *packet)
{
    if (packet) {
        static uint8_t bytesToWrite = FLYSKY_2A_PAYLOAD_SIZE; // first time write full packet to buffer a7105
        flySky2ATelemetryPkt_t *telemertyPacket = (flySky2ATelemetryPkt_t*) packet;
        uint16_t voltage = getBatteryVoltage();

        telemertyPacket->type = FLYSKY_2A_PACKET_TELEMETRY;

        telemertyPacket->sens[0].type = SENSOR_INT_V;
        telemertyPacket->sens[0].number = 0;
        telemertyPacket->sens[0].valueL = voltage & 0xFF;
        telemertyPacket->sens[0].valueH = (voltage >> 8) & 0xFF;

        telemertyPacket->sens[1].type = SENSOR_RSSI;
        telemertyPacket->sens[1].number = 0;
        telemertyPacket->sens[1].valueL = rssi_dBm & 0xFF;
        telemertyPacket->sens[1].valueH = (rssi_dBm >> 8) & 0xFF;

        telemertyPacket->sens[2].type = SENSOR_ERR_RATE;
        telemertyPacket->sens[2].number = 0;
        telemertyPacket->sens[2].valueL = errorRate & 0xFF;
        telemertyPacket->sens[2].valueH = (errorRate >> 8) & 0xFF;

        memset (&telemertyPacket->sens[3], 0xFF, 4 * sizeof(flySky2ASens_t));

        A7105WriteFIFO(packet, bytesToWrite);

        bytesToWrite = 9 + 3 * sizeof(flySky2ASens_t);// next time write only bytes that could change, the others are already set as 0xFF in buffer a7105
    }
}

static rx_spi_received_e flySky2AReadAndProcess(uint8_t *payload, const uint32_t timeStamp)
{
    rx_spi_received_e result = RX_SPI_RECEIVED_NONE;
    uint8_t packet[FLYSKY_2A_PAYLOAD_SIZE];

    uint8_t bytesToRead = (bound) ? (9 + 2*FLYSKY_2A_CHANNEL_COUNT) : (11 + FLYSKY_FREQUENCY_COUNT);
    A7105ReadFIFO(packet, bytesToRead);

    switch (packet[0]) {
    case FLYSKY_2A_PACKET_RC_DATA:
    case FLYSKY_2A_PACKET_FS_SETTINGS: // failsafe settings
    case FLYSKY_2A_PACKET_SETTINGS: // receiver settings
        if (isValidPacket(packet)) {
            checkRSSI();
            resetTimeout(timeStamp);

            const flySky2ARcDataPkt_t *rcPacket = (const flySky2ARcDataPkt_t*) packet;

            if (rcPacket->type == FLYSKY_2A_PACKET_RC_DATA) {
                if (payload) {
                    memcpy(payload, rcPacket->data, 2*FLYSKY_2A_CHANNEL_COUNT);
                }

                if (sendTelemetry) {
                    buildAndWriteTelemetry(packet);
                    sendTelemetry = false;
                    timeTxRequest = timeStamp;
                    waitTx = true;
                }

                result = RX_SPI_RECEIVED_DATA;
            }

            if (!waitTx) {
                A7105WriteReg(A7105_0F_CHANNEL, getNextChannel(1));
            }
        }
        break;

    case FLYSKY_2A_PACKET_BIND1:
    case FLYSKY_2A_PACKET_BIND2:
        if (!bound) {
            resetTimeout(timeStamp);

            flySky2ABindPkt_t *bindPacket = (flySky2ABindPkt_t*) packet;

            if (bindPacket->rfChannelMap[0] != 0xFF) {
                memcpy(rfChannelMap, bindPacket->rfChannelMap, FLYSKY_FREQUENCY_COUNT); // get TX channels
            }

            txId = bindPacket->txId;
            bindPacket->rxId = rxId;
            memset(bindPacket->rfChannelMap, 0xFF, 26); // erase channelMap and 10 bytes after it

            timeTxRequest = timeLastBind = timeStamp;
            waitTx = true;

            A7105WriteFIFO(packet, FLYSKY_2A_PAYLOAD_SIZE);
        }
        break;

    default:
        break;
    }

    if (!waitTx) {
        A7105Strobe(A7105_RX);
    }
    return result;
}

static rx_spi_received_e flySkyReadAndProcess(uint8_t *payload, const uint32_t timeStamp)
{
    rx_spi_received_e result = RX_SPI_RECEIVED_NONE;
    uint8_t packet[FLYSKY_PAYLOAD_SIZE];

    uint8_t bytesToRead = (bound) ? (5 + 2*FLYSKY_CHANNEL_COUNT) : (5);
    A7105ReadFIFO(packet, bytesToRead);

    const flySkyRcDataPkt_t *rcPacket = (const flySkyRcDataPkt_t*) packet;

    if (bound && rcPacket->type == FLYSKY_PACKET_RC_DATA && rcPacket->txId == txId) {
        checkRSSI();
        resetTimeout(timeStamp);

        if (payload) {
            memcpy(payload, rcPacket->data, 2*FLYSKY_CHANNEL_COUNT);
        }

        A7105WriteReg(A7105_0F_CHANNEL, getNextChannel(1));

        result = RX_SPI_RECEIVED_DATA;
    }

    if (!bound && rcPacket->type == FLYSKY_PACKET_BIND) {
        resetTimeout(timeStamp);

        txId = rcPacket->txId;
        flySkyCalculateRfChannels();

        A7105WriteReg(A7105_0F_CHANNEL, getNextChannel(0));

        timeLastBind = timeStamp;
    }

    A7105Strobe(A7105_RX);
    return result;
}

bool flySkyInit(const rxSpiConfig_t *rxSpiConfig, struct rxRuntimeState_s *rxRuntimeState, rxSpiExtiConfig_t *extiConfig)
{
    if (!rxSpiExtiConfigured()) {
        return false;
    }

    protocol = rxSpiConfig->rx_spi_protocol;

    rxSpiCommonIOInit(rxSpiConfig);

    extiConfig->ioConfig = IOCFG_IPD;
    extiConfig->trigger = BETAFLIGHT_EXTI_TRIGGER_RISING;

    uint8_t startRxChannel;

    if (protocol == RX_SPI_A7105_FLYSKY_2A) {
        rxRuntimeState->channelCount = FLYSKY_2A_CHANNEL_COUNT;
        timings = &flySky2ATimings;
        rxId = U_ID_0 ^ U_ID_1 ^ U_ID_2;
        startRxChannel = flySky2ABindChannels[0];
        A7105Init(0x5475c52A, IO_NONE);
        A7105Config(flySky2ARegs, sizeof(flySky2ARegs));
    } else {
        rxRuntimeState->channelCount = FLYSKY_CHANNEL_COUNT;
        timings = &flySkyTimings;
        startRxChannel = 0;
        A7105Init(0x5475c52A, IO_NONE);
        A7105Config(flySkyRegs, sizeof(flySkyRegs));
    }

    if (flySkyConfig()->txId == 0) {
        bound = false;
    } else {
        bound = true;
        txId = flySkyConfig()->txId; // load TXID
        memcpy (rfChannelMap, flySkyConfig()->rfChannelMap, FLYSKY_FREQUENCY_COUNT);// load channel map
        startRxChannel = getNextChannel(0);
    }

    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }

    A7105WriteReg(A7105_0F_CHANNEL, startRxChannel);
    A7105Strobe(A7105_RX); // start listening

    resetTimeout(micros());

    return true;
}

void flySkySetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload)
{
    if (rcData && payload) {
        uint32_t channelCount;
        channelCount = (protocol == RX_SPI_A7105_FLYSKY_2A) ? (FLYSKY_2A_CHANNEL_COUNT) : (FLYSKY_CHANNEL_COUNT);

        for (unsigned i = 0; i < channelCount; i++) {
            rcData[i] = payload[2 * i + 1] << 8 | payload [2 * i + 0];
        }
    }
}

rx_spi_received_e flySkyDataReceived(uint8_t *payload)
{
    rx_spi_received_e result = RX_SPI_RECEIVED_NONE;
    uint32_t timeStamp;

    if (A7105RxTxFinished(&timeStamp)) {
        uint8_t modeReg = A7105ReadReg(A7105_00_MODE);

        if (((modeReg & A7105_MODE_TRSR) != 0) && ((modeReg & A7105_MODE_TRER) == 0)) { // TX complete
            if (bound) {
                A7105WriteReg(A7105_0F_CHANNEL, getNextChannel(1));
            }
            A7105Strobe(A7105_RX);
        } else if ((modeReg & (A7105_MODE_CRCF|A7105_MODE_TRER)) == 0) { // RX complete, CRC pass
            if (protocol == RX_SPI_A7105_FLYSKY_2A) {
                result = flySky2AReadAndProcess(payload, timeStamp);
            } else {
                result = flySkyReadAndProcess(payload, timeStamp);
            }
        } else {
            A7105Strobe(A7105_RX);
        }
    }

    if (waitTx && (micros() - timeTxRequest) > TX_DELAY) {
        A7105Strobe(A7105_TX);
        waitTx = false;
    }

    if (rxSpiCheckBindRequested(true)) {
        bound = false;
        txId = 0;
        memset(rfChannelMap, 0, FLYSKY_FREQUENCY_COUNT);
        uint8_t bindChannel = (protocol == RX_SPI_A7105_FLYSKY_2A) ? flySky2ABindChannels[0] : 0;
        A7105WriteReg(A7105_0F_CHANNEL, bindChannel);
    }

    if (bound) {
        checkTimeout();
        rxSpiLedBlinkRxLoss(result);
    } else {
        if ((micros() - timeLastBind) > BIND_TIMEOUT && rfChannelMap[0] != 0 && txId != 0) {
            result = RX_SPI_RECEIVED_BIND;
            bound = true;
            flySkyConfigMutable()->txId = txId; // store TXID
            memcpy (flySkyConfigMutable()->rfChannelMap, rfChannelMap, FLYSKY_FREQUENCY_COUNT);// store channel map
            writeEEPROM();
        }
        rxSpiLedBlinkBind();
    }

    return result;
}

#endif /* USE_RX_FLYSKY */
