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

#ifdef USE_RX_SPEKTRUM

#include "build/debug.h"

#include "drivers/io.h"
#include "drivers/rx/rx_cyrf6936.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/battery.h"

#include "fc/config.h"
#include "config/feature.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx_spi.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/rx_spi_common.h"

#include "rx/cyrf6936_spektrum.h"

static const uint8_t pnCodes[5][9][8] = {
{ /* Row 0 */
  /* Col 0 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8},
  /* Col 1 */ {0x88, 0x17, 0x13, 0x3B, 0x2D, 0xBF, 0x06, 0xD6},
  /* Col 2 */ {0xF1, 0x94, 0x30, 0x21, 0xA1, 0x1C, 0x88, 0xA9},
  /* Col 3 */ {0xD0, 0xD2, 0x8E, 0xBC, 0x82, 0x2F, 0xE3, 0xB4},
  /* Col 4 */ {0x8C, 0xFA, 0x47, 0x9B, 0x83, 0xA5, 0x66, 0xD0},
  /* Col 5 */ {0x07, 0xBD, 0x9F, 0x26, 0xC8, 0x31, 0x0F, 0xB8},
  /* Col 6 */ {0xEF, 0x03, 0x95, 0x89, 0xB4, 0x71, 0x61, 0x9D},
  /* Col 7 */ {0x40, 0xBA, 0x97, 0xD5, 0x86, 0x4F, 0xCC, 0xD1},
  /* Col 8 */ {0xD7, 0xA1, 0x54, 0xB1, 0x5E, 0x89, 0xAE, 0x86}
},
{ /* Row 1 */
  /* Col 0 */ {0x83, 0xF7, 0xA8, 0x2D, 0x7A, 0x44, 0x64, 0xD3},
  /* Col 1 */ {0x3F, 0x2C, 0x4E, 0xAA, 0x71, 0x48, 0x7A, 0xC9},
  /* Col 2 */ {0x17, 0xFF, 0x9E, 0x21, 0x36, 0x90, 0xC7, 0x82},
  /* Col 3 */ {0xBC, 0x5D, 0x9A, 0x5B, 0xEE, 0x7F, 0x42, 0xEB},
  /* Col 4 */ {0x24, 0xF5, 0xDD, 0xF8, 0x7A, 0x77, 0x74, 0xE7},
  /* Col 5 */ {0x3D, 0x70, 0x7C, 0x94, 0xDC, 0x84, 0xAD, 0x95},
  /* Col 6 */ {0x1E, 0x6A, 0xF0, 0x37, 0x52, 0x7B, 0x11, 0xD4},
  /* Col 7 */ {0x62, 0xF5, 0x2B, 0xAA, 0xFC, 0x33, 0xBF, 0xAF},
  /* Col 8 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97}
},
{ /* Row 2 */
  /* Col 0 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97},
  /* Col 1 */ {0x8E, 0x4A, 0xD0, 0xA9, 0xA7, 0xFF, 0x20, 0xCA},
  /* Col 2 */ {0x4C, 0x97, 0x9D, 0xBF, 0xB8, 0x3D, 0xB5, 0xBE},
  /* Col 3 */ {0x0C, 0x5D, 0x24, 0x30, 0x9F, 0xCA, 0x6D, 0xBD},
  /* Col 4 */ {0x50, 0x14, 0x33, 0xDE, 0xF1, 0x78, 0x95, 0xAD},
  /* Col 5 */ {0x0C, 0x3C, 0xFA, 0xF9, 0xF0, 0xF2, 0x10, 0xC9},
  /* Col 6 */ {0xF4, 0xDA, 0x06, 0xDB, 0xBF, 0x4E, 0x6F, 0xB3},
  /* Col 7 */ {0x9E, 0x08, 0xD1, 0xAE, 0x59, 0x5E, 0xE8, 0xF0},
  /* Col 8 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E}
},
{ /* Row 3 */
  /* Col 0 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E},
  /* Col 1 */ {0x80, 0x69, 0x26, 0x80, 0x08, 0xF8, 0x49, 0xE7},
  /* Col 2 */ {0x7D, 0x2D, 0x49, 0x54, 0xD0, 0x80, 0x40, 0xC1},
  /* Col 3 */ {0xB6, 0xF2, 0xE6, 0x1B, 0x80, 0x5A, 0x36, 0xB4},
  /* Col 4 */ {0x42, 0xAE, 0x9C, 0x1C, 0xDA, 0x67, 0x05, 0xF6},
  /* Col 5 */ {0x9B, 0x75, 0xF7, 0xE0, 0x14, 0x8D, 0xB5, 0x80},
  /* Col 6 */ {0xBF, 0x54, 0x98, 0xB9, 0xB7, 0x30, 0x5A, 0x88},
  /* Col 7 */ {0x35, 0xD1, 0xFC, 0x97, 0x23, 0xD4, 0xC9, 0x88},
  /* Col 8 */ {0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40, 0x93}
},
{ /* Row 4 */
  /* Col 0 */ {0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40, 0x93},
  /* Col 1 */ {0xDC, 0x68, 0x08, 0x99, 0x97, 0xAE, 0xAF, 0x8C},
  /* Col 2 */ {0xC3, 0x0E, 0x01, 0x16, 0x0E, 0x32, 0x06, 0xBA},
  /* Col 3 */ {0xE0, 0x83, 0x01, 0xFA, 0xAB, 0x3E, 0x8F, 0xAC},
  /* Col 4 */ {0x5C, 0xD5, 0x9C, 0xB8, 0x46, 0x9C, 0x7D, 0x84},
  /* Col 5 */ {0xF1, 0xC6, 0xFE, 0x5C, 0x9D, 0xA5, 0x4F, 0xB7},
  /* Col 6 */ {0x58, 0xB5, 0xB3, 0xDD, 0x0E, 0x28, 0xF1, 0xB0},
  /* Col 7 */ {0x5F, 0x30, 0x3B, 0x56, 0x96, 0x45, 0xF4, 0xA1},
  /* Col 8 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8}
},
};

static const uint8_t cyrf6936Config[][2] = {
    {CYRF6936_CLK_EN,          CYRF6936_RXF}, // Enable the clock
    {CYRF6936_AUTO_CAL_TIME,   0x3C}, // From manual, needed for initialization
    {CYRF6936_AUTO_CAL_OFFSET, 0x14}, // From manual, needed for initialization
    {CYRF6936_RX_CFG,          CYRF6936_LNA | CYRF6936_FAST_TURN_EN}, // Enable low noise amplifier and fast turning
    {CYRF6936_TX_OFFSET_LSB,   0x55}, // From manual, typical configuration
    {CYRF6936_TX_OFFSET_MSB,   0x05}, // From manual, typical configuration
    {CYRF6936_XACT_CFG,        CYRF6936_MODE_SYNTH_RX | CYRF6936_FRC_END}, // Force in Synth RX mode
    {CYRF6936_TX_CFG,          CYRF6936_DATA_CODE_LENGTH | CYRF6936_DATA_MODE_SDR | CYRF6936_PA_4}, // Enable 64 chip codes, SDR mode and amplifier +4dBm
    {CYRF6936_DATA64_THOLD,    0x0E}, // From manual, typical configuration
};
static const uint8_t cyrf6936BindConfig[][2] = {
    {CYRF6936_TX_CFG,      CYRF6936_DATA_CODE_LENGTH | CYRF6936_DATA_MODE_SDR | CYRF6936_PA_4}, // Enable 64 chip codes, SDR mode and amplifier +4dBm
    {CYRF6936_FRAMING_CFG, CYRF6936_SOP_LEN | 0xE}, // Set SOP CODE to 64 chips and SOP Correlator Threshold to 0xE
    {CYRF6936_RX_OVERRIDE, CYRF6936_FRC_RXDR | CYRF6936_DIS_RXCRC}, // Force receive data rate and disable receive CRC checker
    {CYRF6936_EOP_CTRL,    0x02}, // Only enable EOP symbol count of 2
    {CYRF6936_TX_OVERRIDE, CYRF6936_DIS_TXCRC}, // Disable transmit CRC generate
};
static const uint8_t cyrf6936TransferConfig[][2] = {
    {CYRF6936_TX_CFG,      CYRF6936_DATA_CODE_LENGTH | CYRF6936_DATA_MODE_8DR | CYRF6936_PA_4}, // Enable 64 chip codes, 8DR mode and amplifier +4dBm
    {CYRF6936_FRAMING_CFG, CYRF6936_SOP_EN | CYRF6936_SOP_LEN | CYRF6936_LEN_EN | 0xE}, // Set SOP CODE enable, SOP CODE to 64 chips, Packet length enable, and SOP Correlator Threshold to 0xE
    {CYRF6936_TX_OVERRIDE, 0x00}, // Reset TX overrides
    {CYRF6936_RX_OVERRIDE, 0x00}, // Reset RX overrides
};

typedef enum {
    DSM2_22     = 0x01,
    DSM2_11     = 0x02,
    DSM2_11_DX8 = 0x12,
    DSMX_22     = 0xA2,
    DSMX_11     = 0xB2,
} dsm_protocol_e;

#define IS_DSM2(x) (x == DSM2_22 || x == DSM2_11 || x == DSM2_11_DX8)
#define IS_DSMX(x) (!IS_DSM2(x))

#define CHECK_MFG_ID(protocol, packet, id) ((IS_DSM2(protocol) && packet[0] == (~id[2]&0xFF) && packet[1] == (~id[3]&0xFF)) || \
        (IS_DSMX(protocol) && packet[0] == id[2] && packet[1] == id[3]))

typedef enum {
    DSM_RECEIVER_BIND = 0x0,
    DSM_RECEIVER_SYNC_A = 0x1,
    DSM_RECEIVER_SYNC_B = 0x2,
    DSM_RECEIVER_RECV = 0x3,
#ifdef USE_RX_SPEKTRUM_TELEMETRY
    DSM_RECEIVER_TLM = 0x4,
#endif
} dsm_receiver_status_e;

typedef struct dsmReceiver_s {
    dsm_receiver_status_e status;
    dsm_protocol_e protocol;

    uint8_t mfgId[4];

    uint8_t rfChannel;
    uint8_t rfChannelIdx;
    uint8_t rfChannels[23];

    uint8_t sopCol;
    uint8_t dataCol;
    uint16_t crcSeed;

    uint8_t missedPackets;
    uint8_t numChannels;

    bool bound;

    uint32_t timeout;
    uint32_t timeLastPacket;

#ifdef USE_RX_SPEKTRUM_TELEMETRY
    uint32_t timeLastTelemetry;
    bool sendTelemetry;
#endif
} dsmReceiver_t;

STATIC_UNIT_TESTED dsmReceiver_t dsmReceiver;

PG_REGISTER_WITH_RESET_TEMPLATE(spektrumConfig_t, spektrumConfig, PG_RX_SPEKTRUM_SPI_CONFIG, 0);
PG_RESET_TEMPLATE(spektrumConfig_t, spektrumConfig, .protocol = 0, .mfgId = {0, 0, 0, 0}, .numChannels = 0);

static void dsmGenerateDsmxChannels(void)
{
    unsigned idx = 0;
    const uint32_t id = ~((dsmReceiver.mfgId[0] << 24) | (dsmReceiver.mfgId[1] << 16) | (dsmReceiver.mfgId[2] << 8) | (dsmReceiver.mfgId[3] << 0));
    uint32_t idTmp = id;

    while (idx < 23) {
        unsigned i;
        unsigned count3To27 = 0, count28To51 = 0, count52To76 = 0;

        idTmp = idTmp * 0x0019660D + 0x3C6EF35F; // Randomization
        const uint8_t nextCh = ((idTmp >> 8) % 0x49) + 3;
        if (((nextCh ^ id) & 0x01) == 0) {
            continue;
        }

        for (i = 0; i < idx; i++) {
            if (dsmReceiver.rfChannels[i] == nextCh) {
                break;
            }

            if (dsmReceiver.rfChannels[i] <= 27) {
                count3To27++;
            } else if (dsmReceiver.rfChannels[i] <= 51) {
                count28To51++;
            } else {
                count52To76++;
            }
        }

        if (i != idx) {
            continue;
        }

        if ((nextCh < 28 && count3To27 < 8)
            || (nextCh >= 28 && nextCh < 52 && count28To51 < 7)
            || (nextCh >= 52 && count52To76 < 8)) {
            dsmReceiver.rfChannels[idx++] = nextCh;
        }
    }
}

static void dsmSetChannel(const uint8_t channel, const uint8_t sopCol, const uint8_t dataCol, const uint16_t crcSeed)
{
    const uint8_t pnRow = IS_DSM2(dsmReceiver.protocol) ? channel % 5 : (channel - 2) % 5;

    cyrf6936SetCrcSeed(crcSeed);
    cyrf6936SetSopCode(pnCodes[pnRow][sopCol]);
    cyrf6936SetDataCode(pnCodes[pnRow][dataCol]);

    cyrf6936SetChannel(channel);
}

static void dsmReceiverSetNextSyncChannel(void)
{
    dsmReceiver.crcSeed = ~dsmReceiver.crcSeed;
    dsmReceiver.rfChannel = (dsmReceiver.rfChannel + 1) % DSM_MAX_RF_CHANNEL;
    dsmSetChannel(dsmReceiver.rfChannel, dsmReceiver.sopCol, dsmReceiver.dataCol, dsmReceiver.crcSeed);
}

static void dsmReceiverSetNextChannel(void)
{
    dsmReceiver.rfChannelIdx = IS_DSM2(dsmReceiver.protocol) ? (dsmReceiver.rfChannelIdx + 1) % 2 : (dsmReceiver.rfChannelIdx + 1) % 23;
    dsmReceiver.crcSeed = ~dsmReceiver.crcSeed;
    dsmReceiver.rfChannel = dsmReceiver.rfChannels[dsmReceiver.rfChannelIdx];
    dsmSetChannel(dsmReceiver.rfChannel, dsmReceiver.sopCol, dsmReceiver.dataCol, dsmReceiver.crcSeed);
}

static void resetReceiveTimeout(const uint32_t timeStamp)
{
    dsmReceiver.timeLastPacket = timeStamp;
    if (dsmReceiver.crcSeed == ((dsmReceiver.mfgId[0] << 8) + dsmReceiver.mfgId[1])) {
        dsmReceiver.timeout = DSM_RECV_SHORT_TIMEOUT_US;
    } else {
        dsmReceiver.timeout = (dsmReceiver.numChannels < 8 ? DSM_RECV_LONG_TIMEOUT_US : DSM_RECV_MID_TIMEOUT_US);
    }
}

static void checkTimeout(void)
{
    const uint32_t time = micros();

#ifdef USE_RX_SPEKTRUM_TELEMETRY
    if (featureIsEnabled(FEATURE_TELEMETRY) && (time - dsmReceiver.timeLastTelemetry) > DSM_TELEMETRY_TIME_US) {
        dsmReceiver.timeLastTelemetry = time;
        dsmReceiver.sendTelemetry = true;
    }
#endif

    if ((time - dsmReceiver.timeLastPacket) > dsmReceiver.timeout) {
        cyrf6936SetMode(CYRF6936_MODE_SYNTH_RX, true);
        cyrf6936WriteRegister(CYRF6936_RX_ABORT, 0x00);

        dsmReceiver.timeLastPacket += dsmReceiver.timeout;

        switch (dsmReceiver.status) {
        case DSM_RECEIVER_BIND:
            dsmReceiver.rfChannel = (dsmReceiver.rfChannel + 2) % DSM_MAX_RF_CHANNEL;
            cyrf6936SetChannel(dsmReceiver.rfChannel);
            dsmReceiver.timeout = DSM_BIND_TIMEOUT_US;
            cyrf6936StartRecv();
            break;
        case DSM_RECEIVER_SYNC_A:
        case DSM_RECEIVER_SYNC_B:
            IS_DSM2(dsmReceiver.protocol) ? dsmReceiverSetNextSyncChannel() : dsmReceiverSetNextChannel();
            dsmReceiver.timeout = DSM_SYNC_TIMEOUT_US;
            cyrf6936StartRecv();
            break;
        case DSM_RECEIVER_RECV:
            dsmReceiver.missedPackets++;
            DEBUG_SET(DEBUG_RX_SPEKTRUM_SPI, 0, dsmReceiver.missedPackets);
            if (dsmReceiver.missedPackets < DSM_MAX_MISSED_PACKETS) {
                dsmReceiverSetNextChannel();
                if (dsmReceiver.crcSeed == ((dsmReceiver.mfgId[0] << 8) + dsmReceiver.mfgId[1])) {
                    dsmReceiver.timeout = DSM_RECV_SHORT_TIMEOUT_US;
                } else {
                    dsmReceiver.timeout = DSM_RECV_TIMEOUT_OFFSET_US + (dsmReceiver.numChannels < 8 ? DSM_RECV_LONG_TIMEOUT_US : DSM_RECV_MID_TIMEOUT_US);
                }
                cyrf6936StartRecv();
            } else {
                setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
                dsmReceiver.status = DSM_RECEIVER_SYNC_A;
                dsmReceiver.timeout = DSM_SYNC_TIMEOUT_US;
            }
            break;
#ifdef USE_RX_SPEKTRUM_TELEMETRY
        case DSM_RECEIVER_TLM:
            DEBUG_SET(DEBUG_RX_SPEKTRUM_SPI, 2, (cyrf6936ReadRegister(CYRF6936_TX_IRQ_STATUS) & CYRF6936_TXC_IRQ) == 0);
            dsmReceiverSetNextChannel();
            dsmReceiver.status = DSM_RECEIVER_RECV;
            dsmReceiver.timeout = (dsmReceiver.numChannels < 8 ? DSM_RECV_LONG_TIMEOUT_US : DSM_RECV_MID_TIMEOUT_US) - DSM_TELEMETRY_TIMEOUT_US;
            cyrf6936StartRecv();
            break;
#endif
        default:
            break;
        }
    }
}

static void dsmReceiverStartBind(void)
{
    uint8_t dataCode[16];
    dsmReceiver.status = DSM_RECEIVER_BIND;

    cyrf6936SetConfigLen(cyrf6936BindConfig, ARRAYLEN(cyrf6936BindConfig));

    memcpy(dataCode, pnCodes[0][8], 8);
    memcpy(dataCode + 8, pnCodes[0][8], 8);
    cyrf6936SetDataCode(dataCode);

    dsmReceiver.rfChannel = DSM_INITIAL_BIND_CHANNEL;
    cyrf6936SetChannel(dsmReceiver.rfChannel);
    dsmReceiver.timeLastPacket = micros();
    dsmReceiver.timeout = DSM_BIND_TIMEOUT_US;
    cyrf6936StartRecv();
}

static void dsmReceiverStartTransfer(void)
{
    dsmReceiver.status = DSM_RECEIVER_SYNC_A;
    dsmReceiver.rfChannelIdx = 0;
    dsmReceiver.missedPackets = 0;

    cyrf6936SetConfigLen(cyrf6936TransferConfig, ARRAYLEN(cyrf6936TransferConfig));

    dsmReceiver.numChannels = spektrumConfig()->numChannels;
    dsmReceiver.protocol = spektrumConfig()->protocol;

    dsmReceiver.crcSeed = ~((dsmReceiver.mfgId[0] << 8) + dsmReceiver.mfgId[1]);
    dsmReceiver.sopCol = (dsmReceiver.mfgId[0] + dsmReceiver.mfgId[1] + dsmReceiver.mfgId[2] + 2) & 0x07;
    dsmReceiver.dataCol = 7 - dsmReceiver.sopCol;

    if (IS_DSMX(dsmReceiver.protocol)) {
        dsmGenerateDsmxChannels();
        dsmReceiver.rfChannelIdx = 22;
        dsmReceiverSetNextChannel();
    } else {
        memset(dsmReceiver.rfChannels, 0, 23);
        dsmReceiverSetNextSyncChannel();
    }

    dsmReceiver.timeLastPacket = micros();
    dsmReceiver.timeout = DSM_SYNC_TIMEOUT_US << 2;
    cyrf6936StartRecv();
}

bool spektrumSpiInit(const struct rxSpiConfig_s *rxConfig, struct rxRuntimeConfig_s *rxRuntimeConfig)
{
    IO_t extiPin = IOGetByTag(rxConfig->extiIoTag);
    if (!extiPin) {
        return false;
    }

    rxSpiCommonIOInit(rxConfig);

    rxRuntimeConfig->channelCount = DSM_MAX_CHANNEL_COUNT;

    if (!cyrf6936Init(extiPin)) {
        return false;
    }

    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }

    cyrf6936SetConfigLen(cyrf6936Config, ARRAYLEN(cyrf6936Config));

#ifdef USE_RX_SPEKTRUM_TELEMETRY
    dsmReceiver.timeLastTelemetry = micros() + DSM_TELEMETRY_TIME_US;
    dsmReceiver.sendTelemetry = false;
#endif

    if (spektrumConfig()->mfgId[0] || spektrumConfig()->mfgId[1]
        || spektrumConfig()->mfgId[2] || spektrumConfig()->mfgId[3]) {
        dsmReceiver.bound = true;
        memcpy(dsmReceiver.mfgId, spektrumConfig()->mfgId, 4);
        dsmReceiverStartTransfer();
    } else {
        dsmReceiver.bound = false;
        dsmReceiverStartBind();
    }

    return true;
}

#ifdef USE_RX_SPEKTRUM_TELEMETRY
static void dsmSendTelemetryPacket(void)
{
    uint8_t packet[9];
    const uint16_t voltage = getBatteryVoltage();

    packet[0] = DSM_TELEMETRY_FRAME_RPM;
    packet[1] = 0xFF; //sid
    packet[2] = 0xFF; //rpm
    packet[3] = 0xFF; //rpm
    packet[4] = voltage >> 8;
    packet[5] = voltage & 0xFF;
    packet[6] = 0x7F; //temperature
    packet[7] = 0xFF; //temperature
    packet[8] = getRssiPercent();
    cyrf6936SetMode(CYRF6936_MODE_IDLE, true);
    cyrf6936SendLen(packet, 9);
}
#endif

void spektrumSpiSetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload)
{
    if (rcData && payload) {
        const uint8_t divider = (uint8_t) IS_DSMX(dsmReceiver.protocol);
        const uint8_t bitShift = 10 + divider;
        const uint16_t valueMax = IS_DSMX(dsmReceiver.protocol) ? 0x7FF : 0x3FF;

        for (unsigned i = 0; i < 7; i++) {
            const uint16_t tmp = (payload[2 * i] << 8) + payload[2 * i + 1];
            const uint8_t chan = (tmp >> bitShift) & 0x0F;
            const int16_t val = (tmp & valueMax) >> divider;

            if (chan < dsmReceiver.numChannels) {
                rcData[chan] = 988 + val;
            }
        }
    }
}

static bool isValidPacket(const uint8_t *packet)
{
    DEBUG_SET(DEBUG_RX_SPEKTRUM_SPI, 1, isError);
    if (isError) {
        if (dsmReceiver.status != DSM_RECEIVER_RECV && (cyrf6936GetRxStatus() & CYRF6936_BAD_CRC)) {
            dsmReceiver.crcSeed = ~dsmReceiver.crcSeed;
        } else {
            return false;
        }
    }
    if (!CHECK_MFG_ID(dsmReceiver.protocol, packet, dsmReceiver.mfgId)) {
        return false;
    }
    return true;
}

rx_spi_received_e spektrumReadPacket(uint8_t *payload, const uint32_t timeStamp)
{
    rx_spi_received_e result = RX_SPI_RECEIVED_NONE;

    uint8_t packetLength, packet[16];

    packetLength = cyrf6936ReadRegister(CYRF6936_RX_COUNT);
    cyrf6936RecvLen(packet, packetLength);

    cyrf6936WriteRegister(CYRF6936_XACT_CFG, CYRF6936_MODE_SYNTH_RX | CYRF6936_FRC_END);
    cyrf6936WriteRegister(CYRF6936_RX_ABORT, 0x00);

    if (packetLength < 2) {
        return result;
    }

    switch (dsmReceiver.status) {
    case DSM_RECEIVER_BIND:
        if (packet[0] != packet[4] || packet[1] != packet[5]
            || packet[2] != packet[6] || packet[3] != packet[7]) {

            dsmReceiver.timeLastPacket = timeStamp;
            dsmReceiver.timeout = DSM_BIND_TIMEOUT_US;
            break;
        }

        unsigned i;
        uint16_t bindSum = 384 - 0x10;
        for (i = 0; i < 8; i++) {
            bindSum += packet[i];
        }

        if (packet[8] != bindSum >> 8 || packet[9] != (bindSum & 0xFF)) {
            break;
        }

        for (i = 8; i < 14; i++) {
            bindSum += packet[i];
        }

        if (packet[14] != bindSum >> 8 || packet[15] != (bindSum & 0xFF)) {
            break;
        }

        dsmReceiver.mfgId[0] = ~packet[0];
        dsmReceiver.mfgId[1] = ~packet[1];
        dsmReceiver.mfgId[2] = ~packet[2];
        dsmReceiver.mfgId[3] = ~packet[3];
        dsmReceiver.numChannels = packet[11];
        dsmReceiver.protocol = packet[12];
        memcpy(spektrumConfigMutable()->mfgId, dsmReceiver.mfgId, 4);
        spektrumConfigMutable()->numChannels = dsmReceiver.numChannels;
        spektrumConfigMutable()->protocol = dsmReceiver.protocol;
        writeEEPROM();

        dsmReceiverStartTransfer();

        dsmReceiver.bound = true;
        result = RX_SPI_RECEIVED_BIND;
        break;
    case DSM_RECEIVER_SYNC_A:
        if (isValidPacket(packet)) {
            if (IS_DSM2(dsmReceiver.protocol)) {
                dsmReceiver.rfChannels[0] = dsmReceiver.rfChannel;
                dsmReceiver.rfChannels[1] = dsmReceiver.rfChannel;

                dsmReceiver.status = DSM_RECEIVER_SYNC_B;
            } else {
                dsmReceiver.status = DSM_RECEIVER_RECV;
                dsmReceiver.missedPackets = 0;
            }

            IS_DSM2(dsmReceiver.protocol) ? dsmReceiverSetNextSyncChannel() : dsmReceiverSetNextChannel();
            resetReceiveTimeout(timeStamp);
            cyrf6936StartRecv();
        }
        break;
    case DSM_RECEIVER_SYNC_B:
        if (isValidPacket(packet)) {
            if (dsmReceiver.crcSeed != ((dsmReceiver.mfgId[0] << 8) + dsmReceiver.mfgId[1])) {
                dsmReceiver.rfChannels[0] = dsmReceiver.rfChannel;
            } else {
                dsmReceiver.rfChannels[1] = dsmReceiver.rfChannel;
            }

            if (dsmReceiver.rfChannels[0] != dsmReceiver.rfChannels[1]) {
                dsmReceiver.status = DSM_RECEIVER_RECV;
                dsmReceiver.missedPackets = 0;
                dsmReceiverSetNextChannel();
                resetReceiveTimeout(timeStamp);
                cyrf6936StartRecv();
            }
        }
        break;
    case DSM_RECEIVER_RECV:
        if (isValidPacket(packet)) {
            setRssi(403 + cyrf6936GetRssi() * 20, RSSI_SOURCE_RX_PROTOCOL);
            dsmReceiver.missedPackets = 0;
            DEBUG_SET(DEBUG_RX_SPEKTRUM_SPI, 0, dsmReceiver.missedPackets);
            memcpy(payload, &packet[2], 14);
#ifdef USE_RX_SPEKTRUM_TELEMETRY
            if (dsmReceiver.sendTelemetry && dsmReceiver.crcSeed == ((dsmReceiver.mfgId[0] << 8) + dsmReceiver.mfgId[1])) {
                dsmSendTelemetryPacket();
                dsmReceiver.sendTelemetry = false;
                dsmReceiver.timeLastPacket = timeStamp;
                dsmReceiver.timeout = DSM_TELEMETRY_TIMEOUT_US;
                dsmReceiver.status = DSM_RECEIVER_TLM;
            } else {
#endif
                dsmReceiverSetNextChannel();
                resetReceiveTimeout(timeStamp);
                cyrf6936StartRecv();
#ifdef USE_RX_SPEKTRUM_TELEMETRY
            }
#endif
            result = RX_SPI_RECEIVED_DATA;
        }
        break;
    default:
        break;
    }

    return result;
}

rx_spi_received_e spektrumSpiDataReceived(uint8_t *payload)
{
    rx_spi_received_e result = RX_SPI_RECEIVED_NONE;
    uint32_t timeStamp;

    if (rxSpiCheckBindRequested(true)) {
        dsmReceiver.bound = false;
        dsmReceiverStartBind();
    }

    if (cyrf6936RxFinished(&timeStamp)) {
        result = spektrumReadPacket(payload, timeStamp);
    }

    checkTimeout();

    dsmReceiver.bound ? rxSpiLedBlinkRxLoss(result) : rxSpiLedBlinkBind();

    return result;
}

#endif /* USE_RX_SPEKTRUM */
