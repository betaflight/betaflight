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

#include <stdbool.h>

#include "platform.h"

#ifdef USE_RX_FRSKY_SPI

#include "common/maths.h"

#include "drivers/io.h"
#include "drivers/rx/rx_cc2500.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/time.h"

#include "config/config.h"

#include "pg/rx.h"
#include "pg/rx_spi.h"
#include "pg/rx_spi_cc2500.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/rx_spi_common.h"

#include "rx/cc2500_common.h"
#include "rx/cc2500_frsky_common.h"
#include "rx/cc2500_frsky_d.h"
#include "rx/cc2500_frsky_x.h"

#include "cc2500_frsky_shared.h"

rx_spi_protocol_e spiProtocol;

static timeMs_t start_time;
static uint8_t packetLength;
static uint8_t protocolState;

uint32_t missingPackets;
timeDelta_t timeoutUs;

static uint8_t calData[255][3];
static timeMs_t timeTunedMs;
uint8_t listLength;
static uint16_t bindState;
static int8_t bindOffset, bindOffset_min, bindOffset_max;

typedef uint8_t handlePacketFn(uint8_t * const packet, uint8_t * const protocolState);
typedef rx_spi_received_e processFrameFn(uint8_t * const packet);
typedef void setRcDataFn(uint16_t *rcData, const uint8_t *payload);

static handlePacketFn *handlePacket;
static processFrameFn *processFrame;
static setRcDataFn *setRcData;

const cc2500RegisterConfigElement_t cc2500FrskyBaseConfig[] =
{
    { CC2500_02_IOCFG0,   0x01 },
    { CC2500_18_MCSM0,    0x18 },
    { CC2500_07_PKTCTRL1, 0x05 },
    { CC2500_3E_PATABLE,  0xFF },
    { CC2500_0C_FSCTRL0,  0x00 },
    { CC2500_0D_FREQ2,    0x5C },
    { CC2500_13_MDMCFG1,  0x23 },
    { CC2500_14_MDMCFG0,  0x7A },
    { CC2500_19_FOCCFG,   0x16 },
    { CC2500_1A_BSCFG,    0x6C },
    { CC2500_1B_AGCCTRL2, 0x03 },
    { CC2500_1C_AGCCTRL1, 0x40 },
    { CC2500_1D_AGCCTRL0, 0x91 },
    { CC2500_21_FREND1,   0x56 },
    { CC2500_22_FREND0,   0x10 },
    { CC2500_23_FSCAL3,   0xA9 },
    { CC2500_24_FSCAL2,   0x0A },
    { CC2500_25_FSCAL1,   0x00 },
    { CC2500_26_FSCAL0,   0x11 },
    { CC2500_29_FSTEST,   0x59 },
    { CC2500_2C_TEST2,    0x88 },
    { CC2500_2D_TEST1,    0x31 },
    { CC2500_2E_TEST0,    0x0B },
    { CC2500_03_FIFOTHR,  0x07 },
    { CC2500_09_ADDR,     0x00 }
};

const cc2500RegisterConfigElement_t cc2500FrskyDConfig[] =
{
    { CC2500_17_MCSM1,    0x0C },
    { CC2500_0E_FREQ1,    0x76 },
    { CC2500_0F_FREQ0,    0x27 },
    { CC2500_06_PKTLEN,   0x19 },
    { CC2500_08_PKTCTRL0, 0x05 },
    { CC2500_0B_FSCTRL1,  0x08 },
    { CC2500_10_MDMCFG4,  0xAA },
    { CC2500_11_MDMCFG3,  0x39 },
    { CC2500_12_MDMCFG2,  0x11 },
    { CC2500_15_DEVIATN,  0x42 }
};

const cc2500RegisterConfigElement_t cc2500FrskyXConfig[] =
{
    { CC2500_17_MCSM1,    0x0C },
    { CC2500_0E_FREQ1,    0x76 },
    { CC2500_0F_FREQ0,    0x27 },
    { CC2500_06_PKTLEN,   0x1E },
    { CC2500_08_PKTCTRL0, 0x01 },
    { CC2500_0B_FSCTRL1,  0x0A },
    { CC2500_10_MDMCFG4,  0x7B },
    { CC2500_11_MDMCFG3,  0x61 },
    { CC2500_12_MDMCFG2,  0x13 },
    { CC2500_15_DEVIATN,  0x51 }
};

const cc2500RegisterConfigElement_t cc2500FrskyXLbtConfig[] =
{
    { CC2500_17_MCSM1,    0x0E },
    { CC2500_0E_FREQ1,    0x80 },
    { CC2500_0F_FREQ0,    0x00 },
    { CC2500_06_PKTLEN,   0x23 },
    { CC2500_08_PKTCTRL0, 0x01 },
    { CC2500_0B_FSCTRL1,  0x08 },
    { CC2500_10_MDMCFG4,  0x7B },
    { CC2500_11_MDMCFG3,  0xF8 },
    { CC2500_12_MDMCFG2,  0x03 },
    { CC2500_15_DEVIATN,  0x53 }
};

const cc2500RegisterConfigElement_t cc2500FrskyXV2Config[] =
{
    { CC2500_17_MCSM1,    0x0E },
    { CC2500_08_PKTCTRL0, 0x05 },
    { CC2500_11_MDMCFG3,  0x84 },
};

const cc2500RegisterConfigElement_t cc2500FrskyXLbtV2Config[] =
{
    { CC2500_08_PKTCTRL0, 0x05 },
};

static void initialise(void)
{
    rxSpiStartupSpeed();

    cc2500Reset();

    cc2500ApplyRegisterConfig(cc2500FrskyBaseConfig, sizeof(cc2500FrskyBaseConfig));

    switch (spiProtocol) {
    case RX_SPI_FRSKY_D:
        cc2500ApplyRegisterConfig(cc2500FrskyDConfig, sizeof(cc2500FrskyDConfig));

        break;
    case RX_SPI_FRSKY_X:
        cc2500ApplyRegisterConfig(cc2500FrskyXConfig, sizeof(cc2500FrskyXConfig));

        break;
    case RX_SPI_FRSKY_X_LBT:
        cc2500ApplyRegisterConfig(cc2500FrskyXLbtConfig, sizeof(cc2500FrskyXLbtConfig));

        break;
    case RX_SPI_FRSKY_X_V2:
        cc2500ApplyRegisterConfig(cc2500FrskyXConfig, sizeof(cc2500FrskyXConfig));
        cc2500ApplyRegisterConfig(cc2500FrskyXV2Config, sizeof(cc2500FrskyXV2Config));

        break;
    case RX_SPI_FRSKY_X_LBT_V2:
        cc2500ApplyRegisterConfig(cc2500FrskyXLbtConfig, sizeof(cc2500FrskyXLbtConfig));
        cc2500ApplyRegisterConfig(cc2500FrskyXLbtV2Config, sizeof(cc2500FrskyXLbtV2Config));

        break;
    default:

        break;
    }

    for(unsigned c = 0;c < 0xFF; c++)
    { //calibrate all channels
        cc2500Strobe(CC2500_SIDLE);
        cc2500WriteReg(CC2500_0A_CHANNR, c);
        cc2500Strobe(CC2500_SCAL);
        delayMicroseconds(900); //
        calData[c][0] = cc2500ReadReg(CC2500_23_FSCAL3);
        calData[c][1] = cc2500ReadReg(CC2500_24_FSCAL2);
        calData[c][2] = cc2500ReadReg(CC2500_25_FSCAL1);
    }

    rxSpiNormalSpeed();
}

void initialiseData(bool inBindState)
{
    cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)rxCc2500SpiConfig()->bindOffset);
    cc2500WriteReg(CC2500_18_MCSM0, 0x8);
    cc2500WriteReg(CC2500_09_ADDR, inBindState ? 0x03 : rxCc2500SpiConfig()->bindTxId[0]);
    cc2500WriteReg(CC2500_07_PKTCTRL1, 0x0D);
    cc2500WriteReg(CC2500_19_FOCCFG, 0x16);
    if (!inBindState) {
        cc2500WriteReg(CC2500_03_FIFOTHR,  0x0E);
    }
    delay(10);
}

static void initTuneRx(void)
{
    cc2500WriteReg(CC2500_19_FOCCFG, 0x14);
    timeTunedMs = millis();
    bindOffset = -126;
    cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset);
    cc2500WriteReg(CC2500_07_PKTCTRL1, 0x0C);
    cc2500WriteReg(CC2500_18_MCSM0, 0x8);

    cc2500Strobe(CC2500_SIDLE);
    cc2500WriteReg(CC2500_23_FSCAL3, calData[0][0]);
    cc2500WriteReg(CC2500_24_FSCAL2, calData[0][1]);
    cc2500WriteReg(CC2500_25_FSCAL1, calData[0][2]);
    cc2500WriteReg(CC2500_0A_CHANNR, 0);
    cc2500Strobe(CC2500_SFRX);
    cc2500Strobe(CC2500_SRX);
}

static bool isValidBindPacket(uint8_t *packet)
{
    if (spiProtocol == RX_SPI_FRSKY_D || spiProtocol == RX_SPI_FRSKY_X_V2 || spiProtocol == RX_SPI_FRSKY_X_LBT_V2) {
        if (!(packet[packetLength - 1] & 0x80)) {
            return false;
        }
    }
    if ((packet[0] == packetLength - 3) && (packet[1] == 0x03) && (packet[2] == 0x01)) {
        return true;
    }

    return false;
}

static bool tuneRx(uint8_t *packet, int8_t inc)
{
    if ((millis() - timeTunedMs) > 50 || bindOffset == 126 || bindOffset == -126) {
        timeTunedMs = millis();
        bindOffset += inc;
        cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset);
        cc2500Strobe(CC2500_SRX);
    }
    if (rxSpiGetExtiState()) {
        uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
        if (ccLen >= packetLength) {
            cc2500ReadFifo(packet, packetLength);
            if (isValidBindPacket(packet)) {
                return true;
            }
        }
    }

    return false;
}

static void initGetBind(void)
{
    cc2500Strobe(CC2500_SIDLE);
    cc2500WriteReg(CC2500_23_FSCAL3, calData[0][0]);
    cc2500WriteReg(CC2500_24_FSCAL2, calData[0][1]);
    cc2500WriteReg(CC2500_25_FSCAL1, calData[0][2]);
    cc2500WriteReg(CC2500_0A_CHANNR, 0);
    cc2500Strobe(CC2500_SFRX);
    delayMicroseconds(20); // waiting flush FIFO

    cc2500Strobe(CC2500_SRX);
    listLength = 0;
    bindState = 0;
}

static void generateV2HopData(uint16_t id)
{
    uint8_t inc = (id % 46) + 1;                                // Increment
    if (inc == 12 || inc == 35) inc++;                          // Exception list from dumps
    uint8_t offset = id % 5;                                    // Start offset

    uint8_t channel;
    for (uint8_t i = 0; i < 47; i++) {
        channel = 5 * ((uint16_t)(inc * i) % 47) + offset;      // Exception list from dumps
        if (spiProtocol == RX_SPI_FRSKY_X_LBT_V2) {             // LBT or FCC
            // LBT
            if (channel <=1 || channel == 43 || channel == 44 || channel == 87 || channel == 88 || channel == 129 || channel == 130 || channel == 173 || channel == 174) {
                channel += 2;
            }
            else if (channel == 216 || channel == 217 || channel == 218) {
                channel += 3;
            }
        } else {
            // FCC
            if (channel == 3 || channel == 4 || channel == 46 || channel == 47 || channel == 90 || channel == 91  || channel == 133 || channel == 134 || channel == 176 || channel == 177 || channel == 220 || channel == 221) {
                channel += 2;
            }
        }
        rxCc2500SpiConfigMutable()->bindHopData[i] = channel;   // Store
    }
    rxCc2500SpiConfigMutable()->bindHopData[47] = 0;            //Bind freq
    rxCc2500SpiConfigMutable()->bindHopData[48] = 0;
    rxCc2500SpiConfigMutable()->bindHopData[49] = 0;
}

static bool getBind(uint8_t *packet)
{
    // len|bind |tx
    // id|03|01|idx|h0|h1|h2|h3|h4|00|00|00|00|00|00|00|00|00|00|00|00|00|00|00|CHK1|CHK2|RSSI|LQI/CRC|
    if (rxSpiGetExtiState()) {
        uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
        if (ccLen >= packetLength) {
            cc2500ReadFifo(packet, packetLength);
            if (isValidBindPacket(packet)) {
                if (spiProtocol == RX_SPI_FRSKY_X_V2 || spiProtocol == RX_SPI_FRSKY_X_LBT_V2) {
                    for (uint8_t i = 3; i < packetLength - 4; i++) {
                        packet[i] ^= 0xA7;
                    }
                    rxCc2500SpiConfigMutable()->bindTxId[0] = packet[3];
                    rxCc2500SpiConfigMutable()->bindTxId[1] = packet[4];
                    rxCc2500SpiConfigMutable()->bindTxId[2] = packet[5];
                    rxCc2500SpiConfigMutable()->rxNum = packet[6];
                    generateV2HopData((packet[4] << 8) + packet[3]);
                    listLength = 47;

                    return true;
                } else {
                    if (packet[5] == 0x00) {
                        rxCc2500SpiConfigMutable()->bindTxId[0] = packet[3];
                        rxCc2500SpiConfigMutable()->bindTxId[1] = packet[4];
                        if (spiProtocol == RX_SPI_FRSKY_D) {
                            rxCc2500SpiConfigMutable()->bindTxId[2] = packet[17];
                            rxCc2500SpiConfigMutable()->rxNum = 0;
                        } else {
                            rxCc2500SpiConfigMutable()->bindTxId[2] = packet[11];
                            rxCc2500SpiConfigMutable()->rxNum = packet[12];
                        }
                    }
                    for (uint8_t n = 0; n < 5; n++) {
                         rxCc2500SpiConfigMutable()->bindHopData[packet[5] + n] = (packet[5] + n) >= 47 ? 0 : packet[6 + n];
                    }
                    bindState |= 1 << (packet[5] / 5);
                    if (bindState == 0x3FF) {
                        listLength = 47;

                        return true;
                    }
                }
            }
        }
    }

    return false;
}

rx_spi_received_e frSkySpiDataReceived(uint8_t *packet)
{
    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;

    switch (protocolState) {
    case STATE_INIT:
        if ((millis() - start_time) > 10) {
            initialise();

            protocolState = STATE_BIND;
        }

        break;
    case STATE_BIND:
        if (rxSpiCheckBindRequested(true) || rxCc2500SpiConfig()->autoBind) {
            rxSpiLedOn();
            initTuneRx();

            protocolState = STATE_BIND_TUNING_LOW;
        } else {
            protocolState = STATE_STARTING;
        }

        break;
    case STATE_BIND_TUNING_LOW:
       if (tuneRx(packet, 2)) {
            bindOffset_min = bindOffset;
            bindOffset = 126;

            protocolState = STATE_BIND_TUNING_HIGH;
        }

        break;
    case STATE_BIND_TUNING_HIGH:
        if (tuneRx(packet, -2)) {
            bindOffset_max = bindOffset;
            bindOffset = ((int16_t)bindOffset_max + (int16_t)bindOffset_min) / 2;
            rxCc2500SpiConfigMutable()->bindOffset = bindOffset;
            initGetBind();
            initialiseData(true);

            if(bindOffset_min < bindOffset_max)
                protocolState = STATE_BIND_BINDING;
            else
                protocolState = STATE_BIND;
        }

        break;
    case STATE_BIND_BINDING:
        if (getBind(packet)) {
            cc2500Strobe(CC2500_SIDLE);

            protocolState = STATE_BIND_COMPLETE;
        }

        break;
    case STATE_BIND_COMPLETE:
        if (!rxCc2500SpiConfig()->autoBind) {
            writeEEPROM();
        } else {
            uint8_t ctr = 80;
            while (ctr--) {
                rxSpiLedToggle();
                delay(50);
            }
        }

        ret = RX_SPI_RECEIVED_BIND;
        protocolState = STATE_STARTING;

        break;

    case STATE_STARTING:
    default:
        ret = handlePacket(packet, &protocolState);

        break;
    }

    return ret;
}

rx_spi_received_e frSkySpiProcessFrame(uint8_t *packet)
{
    if (processFrame) {
        return processFrame(packet);
    }

    return RX_SPI_RECEIVED_NONE;
}

void frSkySpiSetRcData(uint16_t *rcData, const uint8_t *payload)
{
    setRcData(rcData, payload);
}

void nextChannel(uint8_t skip)
{
    static uint8_t channr = 0;

    channr += skip;
    while (channr >= listLength) {
        channr -= listLength;
    }
    cc2500Strobe(CC2500_SIDLE);
    cc2500WriteReg(CC2500_23_FSCAL3,
                    calData[rxCc2500SpiConfig()->bindHopData[channr]][0]);
    cc2500WriteReg(CC2500_24_FSCAL2,
                    calData[rxCc2500SpiConfig()->bindHopData[channr]][1]);
    cc2500WriteReg(CC2500_25_FSCAL1,
                    calData[rxCc2500SpiConfig()->bindHopData[channr]][2]);
    cc2500WriteReg(CC2500_0A_CHANNR, rxCc2500SpiConfig()->bindHopData[channr]);
    if (spiProtocol == RX_SPI_FRSKY_D) {
        cc2500Strobe(CC2500_SFRX);
    }
}

bool frSkySpiInit(const rxSpiConfig_t *rxSpiConfig, rxRuntimeState_t *rxRuntimeState, rxSpiExtiConfig_t *extiConfig)
{
    UNUSED(extiConfig);

    rxSpiCommonIOInit(rxSpiConfig);
    if (!cc2500SpiInit()) {
        return false;
    }

    spiProtocol = rxSpiConfig->rx_spi_protocol;

    switch (spiProtocol) {
    case RX_SPI_FRSKY_D:
        rxRuntimeState->channelCount = RC_CHANNEL_COUNT_FRSKY_D;

        handlePacket = frSkyDHandlePacket;
        setRcData = frSkyDSetRcData;
        packetLength = FRSKY_RX_D8_LENGTH;
        frSkyDInit();

        break;
    case RX_SPI_FRSKY_X:
    case RX_SPI_FRSKY_X_LBT:
    case RX_SPI_FRSKY_X_V2:
    case RX_SPI_FRSKY_X_LBT_V2:
        rxRuntimeState->channelCount = RC_CHANNEL_COUNT_FRSKY_X;

        handlePacket = frSkyXHandlePacket;
#if defined(USE_RX_FRSKY_SPI_TELEMETRY) && defined(USE_TELEMETRY_SMARTPORT)
        processFrame = frSkyXProcessFrame;
#endif
        setRcData = frSkyXSetRcData;
        packetLength = frSkyXInit();

        break;
    default:

        break;
    }

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }
#endif

    missingPackets = 0;
    timeoutUs = 50;

    start_time = millis();
    protocolState = STATE_INIT;

    return true;
}
#endif
