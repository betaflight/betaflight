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
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RX_FRSKY_D

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/cc2500.h"
#include "drivers/io.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "fc/config.h"

#include "config/feature.h"
#include "config/parameter_group_ids.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/frsky_d.h"

#include "sensors/battery.h"

#include "telemetry/frsky.h"

#define RC_CHANNEL_COUNT 8
#define MAX_MISSING_PKT 100

#define DEBUG_DATA_ERROR_COUNT 0

#define SYNC 9000
#define FS_THR 960

static uint32_t missingPackets;
static uint8_t calData[255][3];
static uint32_t time_tune;
static uint8_t listLength;
static uint8_t bindIdx;
static uint8_t cnt;
static int32_t t_out;
static uint8_t Lqi;
static timeUs_t lastPacketReceivedTime;
static uint8_t protocolState;
static uint32_t start_time;
static int8_t bindOffset;
static uint16_t dataErrorCount = 0;

static IO_t gdoPin;
static IO_t bindPin = DEFIO_IO(NONE);
static bool lastBindPinStatus;
static IO_t frSkyLedPin;
#if defined(USE_FRSKY_RX_PA_LNA)
static IO_t txEnPin;
static IO_t rxEnPin;
static IO_t antSelPin;
static uint8_t pass;
#endif
bool bindRequested = false;

#ifdef USE_FRSKY_D_TELEMETRY
static uint8_t frame[20];
static int16_t RSSI_dBm;
static uint8_t telemetry_id;
static uint32_t telemetryTime;

#if defined(TELEMETRY_FRSKY)
#define MAX_SERIAL_BYTES 64
static uint8_t hub_index;
static uint8_t idxx = 0;
static uint8_t idx_ok = 0;
static uint8_t telemetry_expected_id = 0;
static uint8_t srx_data[MAX_SERIAL_BYTES]; // buffer for telemetry serial data
#endif
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(frSkyDConfig_t, frSkyDConfig, PG_FRSKY_D_CONFIG,
                                0);

PG_RESET_TEMPLATE(frSkyDConfig_t, frSkyDConfig,
    .autoBind = false,
    .bindTxId = {0, 0},
    .bindOffset = 0,
    .bindHopData = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
);

enum {
    STATE_INIT = 0,
    STATE_BIND,
    STATE_BIND_TUNING,
    STATE_BIND_BINDING1,
    STATE_BIND_BINDING2,
    STATE_BIND_COMPLETE,
    STATE_STARTING,
    STATE_UPDATE,
    STATE_DATA,
    STATE_TELEMETRY
};

#if defined(USE_FRSKY_D_TELEMETRY)
static void compute_RSSIdbm(uint8_t *packet)
{
    if (packet[18] >= 128) {
        RSSI_dBm = ((((uint16_t)packet[18]) * 18) >> 5) - 82;
    } else {
        RSSI_dBm = ((((uint16_t)packet[18]) * 18) >> 5) + 65;
    }

    processRssi(constrain((RSSI_dBm << 3) / 10, 0, 100));
}

#if defined(TELEMETRY_FRSKY)
static uint8_t frsky_append_hub_data(uint8_t *buf)
{
    if (telemetry_id == telemetry_expected_id)
        idx_ok = idxx;
    else // rx re-requests last packet
        idxx = idx_ok;

    telemetry_expected_id = (telemetry_id + 1) & 0x1F;
    uint8_t index = 0;
    for (uint8_t i = 0; i < 10; i++) {
        if (idxx == hub_index) {
            break;
        }
        buf[i] = srx_data[idxx];
        idxx = (idxx + 1) & (MAX_SERIAL_BYTES - 1);
        index++;
    }
    return index;
}

static void frSkyTelemetryInitFrameSpi(void)
{
    hub_index = 0;
    idxx = 0;
}

static void frSkyTelemetryWriteSpi(uint8_t ch)
{
    if (hub_index < MAX_SERIAL_BYTES) {
        srx_data[hub_index++] = ch;
    }
}
#endif

static void telemetry_build_frame(uint8_t *packet)
{
    const uint16_t adcExternal1Sample = adcGetChannel(ADC_EXTERNAL1);
    const uint16_t adcRssiSample = adcGetChannel(ADC_RSSI);
    uint8_t bytes_used = 0;
    telemetry_id = packet[4];
    frame[0] = 0x11; // length
    frame[1] = frSkyDConfig()->bindTxId[0];
    frame[2] = frSkyDConfig()->bindTxId[1];
    frame[3] = (uint8_t)((adcExternal1Sample & 0xff0) >> 4); // A1
    frame[4] = (uint8_t)((adcRssiSample & 0xff0) >> 4);      // A2
    frame[5] = (uint8_t)RSSI_dBm;
#if defined(TELEMETRY_FRSKY)
    bytes_used = frsky_append_hub_data(&frame[8]);
#endif
    frame[6] = bytes_used;
    frame[7] = telemetry_id;
}

#endif

#if defined(USE_FRSKY_RX_PA_LNA)
static void RX_enable()
{
    IOLo(txEnPin);
    IOHi(rxEnPin);
}

static void TX_enable()
{
    IOLo(rxEnPin);
    IOHi(txEnPin);
}
#endif

void frSkyDBind()
{
    bindRequested = true;
}

static void initialize()
{
    cc2500Reset();
    cc2500WriteReg(CC2500_02_IOCFG0, 0x01);
    cc2500WriteReg(CC2500_17_MCSM1, 0x0C);
    cc2500WriteReg(CC2500_18_MCSM0, 0x18);
    cc2500WriteReg(CC2500_06_PKTLEN, 0x19);
    cc2500WriteReg(CC2500_08_PKTCTRL0, 0x05);
    cc2500WriteReg(CC2500_3E_PATABLE, 0xFF);
    cc2500WriteReg(CC2500_0B_FSCTRL1, 0x08);
    cc2500WriteReg(CC2500_0C_FSCTRL0, 0x00);
    cc2500WriteReg(CC2500_0D_FREQ2, 0x5C);
    cc2500WriteReg(CC2500_0E_FREQ1, 0x76);
    cc2500WriteReg(CC2500_0F_FREQ0, 0x27);
    cc2500WriteReg(CC2500_10_MDMCFG4, 0xAA);
    cc2500WriteReg(CC2500_11_MDMCFG3, 0x39);
    cc2500WriteReg(CC2500_12_MDMCFG2, 0x11);
    cc2500WriteReg(CC2500_13_MDMCFG1, 0x23);
    cc2500WriteReg(CC2500_14_MDMCFG0, 0x7A);
    cc2500WriteReg(CC2500_15_DEVIATN, 0x42);
    cc2500WriteReg(CC2500_19_FOCCFG, 0x16);
    cc2500WriteReg(CC2500_1A_BSCFG, 0x6C);
    cc2500WriteReg(CC2500_1B_AGCCTRL2, 0x03);
    cc2500WriteReg(CC2500_1C_AGCCTRL1, 0x40);
    cc2500WriteReg(CC2500_1D_AGCCTRL0, 0x91);
    cc2500WriteReg(CC2500_21_FREND1, 0x56);
    cc2500WriteReg(CC2500_22_FREND0, 0x10);
    cc2500WriteReg(CC2500_23_FSCAL3, 0xA9);
    cc2500WriteReg(CC2500_24_FSCAL2, 0x0A);
    cc2500WriteReg(CC2500_25_FSCAL1, 0x00);
    cc2500WriteReg(CC2500_26_FSCAL0, 0x11);
    cc2500WriteReg(CC2500_29_FSTEST, 0x59);
    cc2500WriteReg(CC2500_2C_TEST2, 0x88);
    cc2500WriteReg(CC2500_2D_TEST1, 0x31);
    cc2500WriteReg(CC2500_2E_TEST0, 0x0B);
    cc2500WriteReg(CC2500_03_FIFOTHR, 0x07);
    cc2500WriteReg(CC2500_09_ADDR, 0x00);
    cc2500Strobe(CC2500_SIDLE);

    cc2500WriteReg(CC2500_07_PKTCTRL1, 0x04);
    cc2500WriteReg(CC2500_0C_FSCTRL0, 0);
    for (uint8_t c = 0; c < 0xFF; c++) {
        cc2500Strobe(CC2500_SIDLE);
        cc2500WriteReg(CC2500_0A_CHANNR, c);
        cc2500Strobe(CC2500_SCAL);
        delayMicroseconds(900);
        calData[c][0] = cc2500ReadReg(CC2500_23_FSCAL3);
        calData[c][1] = cc2500ReadReg(CC2500_24_FSCAL2);
        calData[c][2] = cc2500ReadReg(CC2500_25_FSCAL1);
    }
}

static void initialize_data(uint8_t adr)
{
    cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)frSkyDConfig()->bindOffset);
    cc2500WriteReg(CC2500_18_MCSM0, 0x8);
    cc2500WriteReg(CC2500_09_ADDR, adr ? 0x03 : frSkyDConfig()->bindTxId[0]);
    cc2500WriteReg(CC2500_07_PKTCTRL1, 0x0D);
    cc2500WriteReg(CC2500_19_FOCCFG, 0x16);
    delay(10);
}

static void initTuneRx(void)
{
    cc2500WriteReg(CC2500_19_FOCCFG, 0x14);
    time_tune = millis();
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

static bool tuneRx(uint8_t *packet)
{
    if (bindOffset >= 126) {
        bindOffset = -126;
    }
    if ((millis() - time_tune) > 50) {
        time_tune = millis();
        bindOffset += 5;
        cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset);
    }
    if (IORead(gdoPin)) {
        uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
        if (ccLen) {
            cc2500ReadFifo(packet, ccLen);
            if (packet[ccLen - 1] & 0x80) {
                if (packet[2] == 0x01) {
                    Lqi = packet[ccLen - 1] & 0x7F;
                    if (Lqi < 50) {
                        frSkyDConfigMutable()->bindOffset = bindOffset;

                        return true;
                    }
                }
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
    bindIdx = 0x05;
}

static bool getBind1(uint8_t *packet)
{
    // len|bind |tx
    // id|03|01|idx|h0|h1|h2|h3|h4|00|00|00|00|00|00|00|00|00|00|00|00|00|00|00|CHK1|CHK2|RSSI|LQI/CRC|
    // Start by getting bind packet 0 and the txid
    if (IORead(gdoPin)) {
        uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
        if (ccLen) {
            cc2500ReadFifo(packet, ccLen);
            if (packet[ccLen - 1] & 0x80) {
                if (packet[2] == 0x01) {
                    if (packet[5] == 0x00) {
                        frSkyDConfigMutable()->bindTxId[0] = packet[3];
                        frSkyDConfigMutable()->bindTxId[1] = packet[4];
                        for (uint8_t n = 0; n < 5; n++) {
                            frSkyDConfigMutable()->bindHopData[packet[5] + n] =
                                packet[6 + n];
                        }

                        return true;
                    }
                }
            }
        }
    }

    return false;
}

static bool getBind2(uint8_t *packet)
{
    if (bindIdx <= 120) {
        if (IORead(gdoPin)) {
            uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            if (ccLen) {
                cc2500ReadFifo(packet, ccLen);
                if (packet[ccLen - 1] & 0x80) {
                    if (packet[2] == 0x01) {
                        if ((packet[3] == frSkyDConfig()->bindTxId[0]) &&
                            (packet[4] == frSkyDConfig()->bindTxId[1])) {
                            if (packet[5] == bindIdx) {
#if defined(DJTS)
                                if (packet[5] == 0x2D) {
                                    for (uint8_t i = 0; i < 2; i++) {
                                        frSkyDConfigMutable()
                                            ->bindHopData[packet[5] + i] =
                                            packet[6 + i];
                                    }
                                    listLength = 47;

                                    return true;
                                }
#endif
                                for (uint8_t n = 0; n < 5; n++) {
                                    if (packet[6 + n] == packet[ccLen - 3] ||
                                        (packet[6 + n] == 0)) {
                                        if (bindIdx >= 0x2D) {
                                            listLength = packet[5] + n;

                                            return true;
                                        }
                                    }

                                    frSkyDConfigMutable()->bindHopData[packet[5] + n] = packet[6 + n];
                                }

                                bindIdx = bindIdx + 5;

                                return false;
                            }
                        }
                    }
                }
            }
        }

        return false;
    } else {
        return true;
    }

}

static void nextChannel(uint8_t skip)
{
    static uint8_t channr = 0;

    channr += skip;
    while (channr >= listLength) {
        channr -= listLength;
    }
    cc2500Strobe(CC2500_SIDLE);
    cc2500WriteReg(CC2500_23_FSCAL3,
                    calData[frSkyDConfig()->bindHopData[channr]][0]);
    cc2500WriteReg(CC2500_24_FSCAL2,
                    calData[frSkyDConfig()->bindHopData[channr]][1]);
    cc2500WriteReg(CC2500_25_FSCAL1,
                    calData[frSkyDConfig()->bindHopData[channr]][2]);
    cc2500WriteReg(CC2500_0A_CHANNR, frSkyDConfig()->bindHopData[channr]);
    cc2500Strobe(CC2500_SFRX);
}

static bool checkBindRequested(bool reset)
{
    if (bindPin) {
        bool bindPinStatus = IORead(bindPin);
        if (lastBindPinStatus && !bindPinStatus) {
            bindRequested = true;
        }
        lastBindPinStatus = bindPinStatus;
    }

    if (!bindRequested) {
        return false;
    } else {
        if (reset) {
            bindRequested = false;
        }

        return true;
    }
}

#define FRSKY_D_CHANNEL_SCALING (2.0f / 3)

static void decodeChannelPair(uint16_t *channels, const uint8_t *packet, const uint8_t highNibbleOffset) {
    channels[0] = FRSKY_D_CHANNEL_SCALING * (uint16_t)((packet[highNibbleOffset] & 0xf) << 8 | packet[0]);
    channels[1] = FRSKY_D_CHANNEL_SCALING * (uint16_t)((packet[highNibbleOffset] & 0xf0) << 4 | packet[1]);
}

void frSkyDSetRcData(uint16_t *rcData, const uint8_t *packet)
{
    uint16_t channels[RC_CHANNEL_COUNT];
    bool dataError = false;

    decodeChannelPair(channels, packet + 6, 4);
    decodeChannelPair(channels + 2, packet + 8, 3);
    decodeChannelPair(channels + 4, packet + 12, 4);
    decodeChannelPair(channels + 6, packet + 14, 3);

    for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
        if ((channels[i] < 800) || (channels[i] > 2200)) {
            dataError = true;

            break;
        }
    }

    if (!dataError) {
        for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
            rcData[i] = channels[i];
        }
    } else {
        DEBUG_SET(DEBUG_FRSKY_D_RX, DEBUG_DATA_ERROR_COUNT, ++dataErrorCount);
    }
}

rx_spi_received_e frSkyDDataReceived(uint8_t *packet)
{
    const timeUs_t currentPacketReceivedTime = micros();

    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;

    switch (protocolState) {
    case STATE_INIT:
        if ((millis() - start_time) > 10) {
            initialize();

            protocolState = STATE_BIND;
        }

        break;
    case STATE_BIND:
        if (checkBindRequested(true) || frSkyDConfig()->autoBind) {
            IOHi(frSkyLedPin);
            initTuneRx();

            protocolState = STATE_BIND_TUNING;
        } else {
            protocolState = STATE_STARTING;
        }

        break;
    case STATE_BIND_TUNING:
        if (tuneRx(packet)) {
            initGetBind();
            initialize_data(1);

            protocolState = STATE_BIND_BINDING1;
        }

        break;
    case STATE_BIND_BINDING1:
        if (getBind1(packet)) {
            protocolState = STATE_BIND_BINDING2;
        }

        break;
    case STATE_BIND_BINDING2:
        if (getBind2(packet)) {
            cc2500Strobe(CC2500_SIDLE);

            protocolState = STATE_BIND_COMPLETE;
        }

        break;
    case STATE_BIND_COMPLETE:
        if (!frSkyDConfig()->autoBind) {
            writeEEPROM();
        } else {
            uint8_t ctr = 40;
            while (ctr--) {
                IOHi(frSkyLedPin);
                delay(50);
                IOLo(frSkyLedPin);
                delay(50);
            }
        }

        protocolState = STATE_STARTING;

        break;
    case STATE_STARTING:
        listLength = 47;
        initialize_data(0);
        protocolState = STATE_UPDATE;
        nextChannel(1); //
        cc2500Strobe(CC2500_SRX);
        ret = RX_SPI_RECEIVED_BIND;

        break;
    case STATE_UPDATE:
        lastPacketReceivedTime = currentPacketReceivedTime;
        protocolState = STATE_DATA;

        if (checkBindRequested(false)) {
            lastPacketReceivedTime = 0;
            t_out = 50;
            missingPackets = 0;

            protocolState = STATE_INIT;

            break;
        }
    // here FS code could be
    case STATE_DATA:
        if (IORead(gdoPin)) {
            uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            if (ccLen >= 20) {
                cc2500ReadFifo(packet, 20);
                if (packet[19] & 0x80) {
                    missingPackets = 0;
                    t_out = 1;
                    if (packet[0] == 0x11) {
                        if ((packet[1] == frSkyDConfig()->bindTxId[0]) &&
                            (packet[2] == frSkyDConfig()->bindTxId[1])) {
                            IOHi(frSkyLedPin);
                            nextChannel(1);
#ifdef USE_FRSKY_D_TELEMETRY
                            if ((packet[3] % 4) == 2) {
                                telemetryTime = micros();
                                compute_RSSIdbm(packet);
                                telemetry_build_frame(packet);
                                protocolState = STATE_TELEMETRY;
                            } else
#endif
                            {
                                cc2500Strobe(CC2500_SRX);
                                protocolState = STATE_UPDATE;
                            }
                            ret = RX_SPI_RECEIVED_DATA;
                            lastPacketReceivedTime = currentPacketReceivedTime;
                        }
                    }
                }
            }
        }

        if (cmp32(currentPacketReceivedTime, lastPacketReceivedTime) > (t_out * SYNC)) {
#ifdef USE_FRSKY_RX_PA_LNA
            RX_enable();
#endif
            if (t_out == 1) {
#ifdef USE_FRSKY_RX_DIVERSITY // SE4311 chip
                if (missingPackets >= 2) {
                    if (pass & 0x01) {
                        IOHi(antSelPin);
                    } else {
                        IOLo(antSelPin);
                    }
                    pass++;
                }
#endif

                if (missingPackets > MAX_MISSING_PKT)
                    t_out = 50;

                missingPackets++;
                nextChannel(1);
            } else {
                if (cnt++ & 0x01) {
                    IOLo(frSkyLedPin);
                } else
                    IOHi(frSkyLedPin);

                nextChannel(13);
            }

            cc2500Strobe(CC2500_SRX);
            protocolState = STATE_UPDATE;
        }
        break;
#ifdef USE_FRSKY_D_TELEMETRY
    case STATE_TELEMETRY:
        if ((micros() - telemetryTime) >= 1380) {
            cc2500Strobe(CC2500_SIDLE);
            cc2500SetPower(6);
            cc2500Strobe(CC2500_SFRX);
#if defined(USE_FRSKY_RX_PA_LNA)
            TX_enable();
#endif
            cc2500Strobe(CC2500_SIDLE);
            cc2500WriteFifo(frame, frame[0] + 1);
            protocolState = STATE_DATA;
            ret = RX_SPI_RECEIVED_DATA;
            lastPacketReceivedTime = currentPacketReceivedTime;
        }

        break;

#endif
    }
    return ret;
}

static void frskyD_Rx_Setup(rx_spi_protocol_e protocol)
{
    UNUSED(protocol);
    // gpio init here
    gdoPin = IOGetByTag(IO_TAG(FRSKY_RX_GDO_0_PIN));
    IOInit(gdoPin, OWNER_RX_BIND, 0);
    IOConfigGPIO(gdoPin, IOCFG_IN_FLOATING);
    frSkyLedPin = IOGetByTag(IO_TAG(FRSKY_RX_LED_PIN));
    IOInit(frSkyLedPin, OWNER_LED, 0);
    IOConfigGPIO(frSkyLedPin, IOCFG_OUT_PP);
#if defined(USE_FRSKY_RX_PA_LNA)
    rxEnPin = IOGetByTag(IO_TAG(FRSKY_RX_RX_EN_PIN));
    IOInit(rxEnPin, OWNER_RX_BIND, 0);
    IOConfigGPIO(rxEnPin, IOCFG_OUT_PP);
    txEnPin = IOGetByTag(IO_TAG(FRSKY_RX_TX_EN_PIN));
    IOInit(txEnPin, OWNER_RX_BIND, 0);
    IOConfigGPIO(txEnPin, IOCFG_OUT_PP);
#endif
#if defined(USE_FRSKY_RX_DIVERSITY)
    antSelPin = IOGetByTag(IO_TAG(FRSKY_RX_ANT_SEL_PIN));
    IOInit(antSelPin, OWNER_RX_BIND, 0);
    IOConfigGPIO(antSelPin, IOCFG_OUT_PP);
#endif
#if defined(BINDPLUG_PIN)
    bindPin = IOGetByTag(IO_TAG(BINDPLUG_PIN));
    IOInit(bindPin, OWNER_RX_BIND, 0);
    IOConfigGPIO(bindPin, IOCFG_IPU);

    lastBindPinStatus = IORead(bindPin);
#endif

    start_time = millis();
    lastPacketReceivedTime = 0;
    t_out = 50;
    missingPackets = 0;
    protocolState = STATE_INIT;
#if defined(USE_FRSKY_RX_DIVERSITY)
    IOHi(antSelPin);
#endif
#if defined(USE_FRSKY_RX_PA_LNA)
    RX_enable();
#endif

#if defined(TELEMETRY_FRSKY)
    initFrSkyExternalTelemetry(&frSkyTelemetryInitFrameSpi,
                               &frSkyTelemetryWriteSpi);
#endif

    // if(!frSkySpiDetect())//detect spi working routine
    // return;
}

void frSkyDInit(const rxConfig_t *rxConfig,
                    rxRuntimeConfig_t *rxRuntimeConfig)
{
    rxRuntimeConfig->channelCount = RC_CHANNEL_COUNT;
    frskyD_Rx_Setup((rx_spi_protocol_e)rxConfig->rx_spi_protocol);
}
#endif
