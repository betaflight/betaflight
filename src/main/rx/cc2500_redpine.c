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

#ifdef USE_RX_REDPINE_SPI

#include "cc2500_redpine.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "common/maths.h"
#include "common/utils.h"
#include "config/config.h"
#include "config/feature.h"
#include "drivers/adc.h"
#include "drivers/io.h"
#include "drivers/io_def.h"
#include "drivers/io_types.h"
#include "drivers/resource.h"
#include "drivers/rx/rx_cc2500.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "io/vtx.h"
#include "pg/rx.h"
#include "pg/rx_spi.h"
#include "pg/rx_spi_cc2500.h"
#include "rx/cc2500_common.h"
#include "rx/rx_spi_common.h"
#include "sensors/battery.h"

enum {
    STATE_INIT = 0,
    STATE_BIND,
    STATE_BIND_TUNING1,
    STATE_BIND_TUNING2,
    STATE_BIND_TUNING3,
    STATE_BIND_COMPLETE,
    STATE_STARTING,
    STATE_UPDATE,
    STATE_DATA,
    STATE_TELEMETRY,
    STATE_RESUME,
};

bool redpineFast = true;

#define VTX_STATUS_FRAME 1
#define SCALE_REDPINE(channelValue) ((2 * channelValue + 2452) / 3)
#define SWITCH_REDPINE_SPEED_US 2000000
#define DEFAULT_PACKET_TIME_US 50000
#define REDPINE_HOP_CHANNELS 49
#define BIND_TUNE_STEP 2

static uint8_t calData[255][3];
static timeMs_t start_time;
static uint8_t protocolState;
static uint32_t missingPackets;
static timeDelta_t timeoutUs;
static timeMs_t timeTunedMs;
static int8_t bindOffset_max = 0;
static int8_t bindOffset_min = 0;

static void initialise(void);
static void initBindTuneRx(void);
static bool tuneRx1(uint8_t *packet);
static bool tuneRx2(uint8_t *packet);
static bool tuneRx3(uint8_t *packet);
static void nextChannel();
static bool redpineRxPacketBind(uint8_t *packet);
static bool isRedpineFast(void);

const cc2500RegisterConfigElement_t cc2500RedPineBaseConfig[] =
{
    { CC2500_02_IOCFG0, 0x01 },
    { CC2500_03_FIFOTHR, 0x07 },
    { CC2500_06_PKTLEN, REDPINE_PACKET_SIZE },
    { CC2500_07_PKTCTRL1, 0x0C },
    { CC2500_08_PKTCTRL0, 0x05 },
    { CC2500_09_ADDR, 0x00 }
};

const cc2500RegisterConfigElement_t cc2500RedPineFastConfig[] =
{
    { CC2500_0B_FSCTRL1, 0x0A },
    { CC2500_0C_FSCTRL0, 0x00 },
    { CC2500_0D_FREQ2, 0x5D },
    { CC2500_0E_FREQ1, 0x93 },
    { CC2500_0F_FREQ0, 0xB1 },
    { CC2500_10_MDMCFG4, 0x2D },
    { CC2500_11_MDMCFG3, 0x3B },
    { CC2500_12_MDMCFG2, 0x73 },
    { CC2500_13_MDMCFG1, 0x23 },
    { CC2500_14_MDMCFG0, 0x56 },
    { CC2500_15_DEVIATN, 0x00 },
    { CC2500_17_MCSM1, 0x0C },
    { CC2500_18_MCSM0, 0x08 },
    { CC2500_19_FOCCFG, 0x1D },
    { CC2500_1A_BSCFG, 0x1C },
    { CC2500_1B_AGCCTRL2, 0xC7 },
    { CC2500_1C_AGCCTRL1, 0x00 },
    { CC2500_1D_AGCCTRL0, 0xB0 },
    { CC2500_21_FREND1, 0xB6 },
    { CC2500_22_FREND0, 0x10 },
    { CC2500_23_FSCAL3, 0xEA },
    { CC2500_24_FSCAL2, 0x0A },
    { CC2500_25_FSCAL1, 0x00 },
    { CC2500_26_FSCAL0, 0x11 },
    { CC2500_29_FSTEST, 0x59 },
    { CC2500_2C_TEST2, 0x88 },
    { CC2500_2D_TEST1, 0x31 },
    { CC2500_2E_TEST0, 0x0B },
    { CC2500_3E_PATABLE, 0xFF }
};

const cc2500RegisterConfigElement_t cc2500RedPineConfig[] =
{
    { CC2500_0B_FSCTRL1, 0x06 },
    { CC2500_0C_FSCTRL0, 0x00 },
    { CC2500_0D_FREQ2, 0x5D },
    { CC2500_0E_FREQ1, 0x93 },
    { CC2500_0F_FREQ0, 0xB1 },
    { CC2500_10_MDMCFG4, 0x78 },
    { CC2500_11_MDMCFG3, 0x93 },
    { CC2500_12_MDMCFG2, 0x03 },
    { CC2500_13_MDMCFG1, 0x22 },
    { CC2500_14_MDMCFG0, 0xF8 },
    { CC2500_15_DEVIATN, 0x44 },
    { CC2500_17_MCSM1, 0x0C },
    { CC2500_18_MCSM0, 0x08 },
    { CC2500_19_FOCCFG, 0x16 },
    { CC2500_1A_BSCFG, 0x6C },
    { CC2500_1B_AGCCTRL2, 0x43 },
    { CC2500_1C_AGCCTRL1, 0x40 },
    { CC2500_1D_AGCCTRL0, 0x91 },
    { CC2500_21_FREND1, 0x56 },
    { CC2500_22_FREND0, 0x10 },
    { CC2500_23_FSCAL3, 0xA9 },
    { CC2500_24_FSCAL2, 0x0A },
    { CC2500_25_FSCAL1, 0x00 },
    { CC2500_26_FSCAL0, 0x11 },
    { CC2500_29_FSTEST, 0x59 },
    { CC2500_2C_TEST2, 0x88 },
    { CC2500_2D_TEST1, 0x31 },
    { CC2500_2E_TEST0, 0x0B },
    { CC2500_3E_PATABLE, 0xFF }
};

static void initialise(void)
{
    cc2500Reset();

    cc2500ApplyRegisterConfig(cc2500RedPineBaseConfig, sizeof(cc2500RedPineBaseConfig));

    if (isRedpineFast()) {
        cc2500ApplyRegisterConfig(cc2500RedPineFastConfig, sizeof(cc2500RedPineFastConfig));
    } else {
        cc2500ApplyRegisterConfig(cc2500RedPineConfig, sizeof(cc2500RedPineConfig));
    }

    for (unsigned c = 0; c < 0xFF; c++) {  // calibrate all channels
        cc2500Strobe(CC2500_SIDLE);
        cc2500WriteReg(CC2500_0A_CHANNR, c);
        cc2500Strobe(CC2500_SCAL);
        delayMicroseconds(900);  //
        calData[c][0] = cc2500ReadReg(CC2500_23_FSCAL3);
        calData[c][1] = cc2500ReadReg(CC2500_24_FSCAL2);
        calData[c][2] = cc2500ReadReg(CC2500_25_FSCAL1);
    }
}

rx_spi_received_e redpineSpiDataReceived(uint8_t *packet)
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
                redpineFast = true;
                initialise();
                rxSpiLedOn();
                initBindTuneRx();

                protocolState = STATE_BIND_TUNING1;
            } else {
                protocolState = STATE_STARTING;
            }

            break;
        case STATE_BIND_TUNING1:
            if (tuneRx1(packet)) {
                protocolState = STATE_BIND_TUNING2;
            }
            break;
        case STATE_BIND_TUNING2:
            if (tuneRx2(packet)) {
                protocolState = STATE_BIND_TUNING3;
            }
            break;
        case STATE_BIND_TUNING3:
            if (tuneRx3(packet)) {
                if (((int16_t)bindOffset_max - (int16_t)bindOffset_min) <= 10) {
                    initBindTuneRx();
                    protocolState = STATE_BIND_TUNING1;  // retry
                } else {
                    rxCc2500SpiConfigMutable()->bindOffset = ((int16_t)bindOffset_max + (int16_t)bindOffset_min) / 2;
                    protocolState = STATE_BIND_COMPLETE;
                    cc2500Strobe(CC2500_SIDLE);

                    for (uint8_t i = 0; i < REDPINE_HOP_CHANNELS; i++) {
                        if (rxCc2500SpiConfigMutable()->bindHopData[i] == 0) {
                            protocolState = STATE_BIND_TUNING1;  // retry
                            break;
                        }
                    }
                }
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
        default:
            ret = redpineHandlePacket(packet, &protocolState);

            break;
    }
    DEBUG_SET(DEBUG_RX_FRSKY_SPI, 3, protocolState);

    return ret;
}

static void initBindTuneRx(void)
{
    timeTunedMs = millis();

    bindOffset_min = -64;
    DEBUG_SET(DEBUG_RX_FRSKY_SPI, 1, bindOffset_min);
    cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset_min);
    cc2500WriteReg(CC2500_07_PKTCTRL1, 0x0C);
    cc2500WriteReg(CC2500_18_MCSM0, 0x8);

    cc2500Strobe(CC2500_SIDLE);
    cc2500WriteReg(CC2500_23_FSCAL3, calData[0][0]);
    cc2500WriteReg(CC2500_24_FSCAL2, calData[0][1]);
    cc2500WriteReg(CC2500_25_FSCAL1, calData[0][2]);
    cc2500WriteReg(CC2500_0A_CHANNR, 0);
    cc2500Strobe(CC2500_SFRX);
    cc2500Strobe(CC2500_SRX);

    for (uint8_t i = 0; i < REDPINE_HOP_CHANNELS; i++) {
        rxCc2500SpiConfigMutable()->bindHopData[i] = 0;
    }
}

static bool tuneRx1(uint8_t *packet)
{
    if (bindOffset_min >= 126) {
        bindOffset_min = -126;
        DEBUG_SET(DEBUG_RX_FRSKY_SPI, 1, bindOffset_min);
    }
    if ((millis() - timeTunedMs) > 220) {  // 220ms
        timeTunedMs = millis();
        bindOffset_min += BIND_TUNE_STEP << 3;
        DEBUG_SET(DEBUG_RX_FRSKY_SPI, 1, bindOffset_min);
        cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset_min);
        cc2500Strobe(CC2500_SRX);
    }
    if (redpineRxPacketBind(packet)) {
        bindOffset_max = bindOffset_min;
        DEBUG_SET(DEBUG_RX_FRSKY_SPI, 2, bindOffset_max);
        cc2500Strobe(CC2500_SRX);
        timeTunedMs = millis();
        return true;
    }
    return false;
}

static bool tuneRx2(uint8_t *packet)
{
    rxSpiLedBlink(100);
    if (((millis() - timeTunedMs) > 880) || bindOffset_max > (126 - BIND_TUNE_STEP)) {  // 220ms *4
        timeTunedMs = millis();
        cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset_min);
        cc2500Strobe(CC2500_SRX);
        return true;
    }
    if (redpineRxPacketBind(packet)) {
        timeTunedMs = millis();
        bindOffset_max += BIND_TUNE_STEP;
        DEBUG_SET(DEBUG_RX_FRSKY_SPI, 2, bindOffset_max);
        cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset_max);
    }
    return false;
}

static bool tuneRx3(uint8_t *packet)
{
    rxSpiLedBlink(100);
    if (((millis() - timeTunedMs) > 880) || bindOffset_min < (-126 + BIND_TUNE_STEP)) {  // 220ms *4
        return true;
    }
    if (redpineRxPacketBind(packet)) {
        timeTunedMs = millis();
        bindOffset_min -= BIND_TUNE_STEP;
        DEBUG_SET(DEBUG_RX_FRSKY_SPI, 1, bindOffset_min);
        cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset_min);
    }
    return false;
}

static bool redpineRxPacketBind(uint8_t *packet)
{
    if (rxSpiGetExtiState()) {
        uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
        if (ccLen != REDPINE_PACKET_SIZE_W_ADDONS) {
            cc2500Strobe(CC2500_SFRX);
        } else {
            cc2500ReadFifo(packet, ccLen);
            if (packet[1] == 0x03 && packet[2] == 0x01) {
                rxCc2500SpiConfigMutable()->bindTxId[0] = packet[3];
                rxCc2500SpiConfigMutable()->bindTxId[1] = packet[4];
                for (uint8_t n = 0; n < 5; n++) {
                    rxCc2500SpiConfigMutable()->bindHopData[packet[5] + n] = packet[6 + n];
                }
                return true;
            }
        }
    }
    return false;
}

static void nextChannel(void)
{
    static uint8_t channr = 0;

    channr += 1;
    while (channr >= REDPINE_HOP_CHANNELS) {
        channr -= REDPINE_HOP_CHANNELS;
    }
    cc2500Strobe(CC2500_SIDLE);
    cc2500WriteReg(CC2500_23_FSCAL3, calData[rxCc2500SpiConfig()->bindHopData[channr]][0]);
    cc2500WriteReg(CC2500_24_FSCAL2, calData[rxCc2500SpiConfig()->bindHopData[channr]][1]);
    cc2500WriteReg(CC2500_25_FSCAL1, calData[rxCc2500SpiConfig()->bindHopData[channr]][2]);
    cc2500WriteReg(CC2500_0A_CHANNR, rxCc2500SpiConfig()->bindHopData[channr]);
}

static bool isRedpineFast(void)
{
    return (redpineFast);
}

void switchRedpineMode(void)
{
    redpineFast = !redpineFast;
}

#define CHANNEL_START 3
void redpineSetRcData(uint16_t *rcData, const uint8_t *packet)
{
    if (packet[CHANNEL_START] == VTX_STATUS_FRAME && packet[CHANNEL_START + 1] == 0) {
        if (!ARMING_FLAG(ARMED)) {
#ifdef USE_VTX
            vtxSettingsConfigMutable()->band = packet[5] + 1;
            vtxSettingsConfigMutable()->channel = packet[6];
            vtxSettingsConfigMutable()->power = packet[7];
            saveConfigAndNotify();
#endif
        }
    } else {
        uint16_t channelValue;
        // 4 stick channels (11-bit)
        channelValue = (uint16_t)((packet[CHANNEL_START + 1] << 8) & 0x700) | packet[CHANNEL_START];
        rcData[0] = SCALE_REDPINE(channelValue);

        channelValue = (uint16_t)((packet[CHANNEL_START + 2] << 4) & 0x7F0) | ((packet[CHANNEL_START + 1] >> 4) & 0xF);
        rcData[1] = SCALE_REDPINE(channelValue);

        channelValue = (uint16_t)((packet[CHANNEL_START + 4] << 8) & 0x700) | packet[CHANNEL_START + 3];
        rcData[2] = SCALE_REDPINE(channelValue);

        channelValue = (uint16_t)((packet[CHANNEL_START + 5] << 4) & 0x7F0) | ((packet[CHANNEL_START + 4] >> 4) & 0xF);
        rcData[3] = SCALE_REDPINE(channelValue);

        // 12 1-bit aux channels - first 4 are interleaved with stick channels
        rcData[4] = (packet[CHANNEL_START + 1] & 0x08) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[5] = (packet[CHANNEL_START + 2] & 0x80) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[6] = (packet[CHANNEL_START + 4] & 0x08) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[7] = (packet[CHANNEL_START + 5] & 0x80) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[8] = (packet[CHANNEL_START + 6] & 0x01) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[9] = (packet[CHANNEL_START + 6] & 0x02) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[10] = (packet[CHANNEL_START + 6] & 0x04) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[11] = (packet[CHANNEL_START + 6] & 0x08) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[12] = (packet[CHANNEL_START + 6] & 0x10) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[13] = (packet[CHANNEL_START + 6] & 0x20) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[14] = (packet[CHANNEL_START + 6] & 0x40) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[15] = (packet[CHANNEL_START + 6] & 0x80) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    }
}

rx_spi_received_e redpineHandlePacket(uint8_t *const packet, uint8_t *const protocolState)
{
    static int32_t looptime = DEFAULT_PACKET_TIME_US;
    static timeUs_t packetTimerUs;
    static timeUs_t totalTimerUs;
    static timeUs_t protocolTimerUs;

    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;

    switch (*protocolState) {
        case STATE_STARTING:
            *protocolState = STATE_UPDATE;
            nextChannel();
            cc2500Strobe(CC2500_SRX);
#ifdef USE_RX_CC2500_SPI_PA_LNA
            cc2500TxDisable();
#endif  // USE_RX_CC2500_SPI_PA_LNA
            protocolTimerUs = micros();
            break;
        case STATE_UPDATE:
            packetTimerUs = 0;
            totalTimerUs = micros();

            *protocolState = STATE_DATA;
            if (rxSpiCheckBindRequested(false)) {
                packetTimerUs = 0;
                missingPackets = 0;
                *protocolState = STATE_INIT;
                break;
            }

            FALLTHROUGH;
            // here FS code could be
        case STATE_DATA:
            if (rxSpiGetExtiState()) {
                uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
                if (ccLen == REDPINE_PACKET_SIZE_W_ADDONS) {
                    cc2500ReadFifo(packet, ccLen);

                    if ((packet[1] == rxCc2500SpiConfig()->bindTxId[0]) && (packet[2] == rxCc2500SpiConfig()->bindTxId[1])) {
                        if (isRedpineFast()) {
                            looptime = packet[CHANNEL_START + 7] * 100;
                        } else {
                            looptime = packet[CHANNEL_START + 7] * 1000;
                        }
                        DEBUG_SET(DEBUG_RX_FRSKY_SPI, 0, looptime);
                        DEBUG_SET(DEBUG_RX_FRSKY_SPI, 1, packet[ccLen - 2]);

                        packetTimerUs = micros() + looptime / 8;  // add a buffer on the packet time incase tx and  rx clocks are different
                        totalTimerUs = micros();
                        protocolTimerUs = micros();
                        missingPackets = 0;
                        DEBUG_SET(DEBUG_RX_FRSKY_SPI, 2, missingPackets);

                        rxSpiLedOn();

                        cc2500setRssiDbm(packet[ccLen - 2]);

                        ret = RX_SPI_RECEIVED_DATA;
                        nextChannel();
                        cc2500Strobe(CC2500_SRX);
                    }
                } else {
                    cc2500Strobe(CC2500_SFRX);
                }
            }

            if (cmpTimeUs(micros(), totalTimerUs) > 50 * looptime) {
                // out of sync with packets - do a complete resysnc
                rxSpiLedToggle();
                setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
                nextChannel();
                cc2500Strobe(CC2500_SRX);
                *protocolState = STATE_UPDATE;
            } else if ((cmpTimeUs(micros(), packetTimerUs) > looptime) && packetTimerUs) {
                // missed a packet
                packetTimerUs = micros();
                nextChannel();
                cc2500Strobe(CC2500_SRX);
                missingPackets++;
                DEBUG_SET(DEBUG_RX_FRSKY_SPI, 2, missingPackets);
#if defined(USE_RX_CC2500_SPI_DIVERSITY)
                if (missingPackets >= 2) {
                    cc2500switchAntennae();
                }
#endif
            } else if (cmpTimeUs(micros(), protocolTimerUs) > SWITCH_REDPINE_SPEED_US) {
                switchRedpineMode();
                looptime = DEFAULT_PACKET_TIME_US;
                protocolTimerUs = micros();
                *protocolState = STATE_INIT;
            }
            break;
    }
    return ret;
}

bool redpineSpiInit(const rxSpiConfig_t *rxSpiConfig, rxRuntimeState_t *rxRuntimeState, rxSpiExtiConfig_t *extiConfig)
{
    UNUSED(extiConfig);

    rxSpiCommonIOInit(rxSpiConfig);
    if (!cc2500SpiInit()) {
        return false;
    }

    rxRuntimeState->channelCount = RC_CHANNEL_COUNT_REDPINE;

    missingPackets = 0;
    timeoutUs = 50;

    start_time = millis();
    protocolState = STATE_INIT;

    return true;
}

#endif
