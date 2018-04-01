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

#include "platform.h"

#ifdef USE_RX_FRSKY_SPI

#include "common/maths.h"

#include "drivers/rx/rx_cc2500.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "fc/config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"

#include "rx/cc2500_frsky_common.h"
#include "rx/cc2500_frsky_d.h"
#include "rx/cc2500_frsky_x.h"

#include "cc2500_frsky_shared.h"

static rx_spi_protocol_e spiProtocol;

static timeMs_t start_time;
static uint8_t protocolState;

uint32_t missingPackets;
timeDelta_t timeoutUs;

static uint8_t calData[255][3];
static timeMs_t timeTunedMs;
uint8_t listLength;
static uint8_t bindIdx;
static int8_t bindOffset;
static bool lastBindPinStatus;

static bool bindRequested;

typedef uint8_t handlePacketFn(uint8_t * const packet, uint8_t * const protocolState);
typedef void setRcDataFn(uint16_t *rcData, const uint8_t *payload);

static handlePacketFn *handlePacket;
static setRcDataFn *setRcData;

IO_t gdoPin;
static IO_t bindPin = DEFIO_IO(NONE);
static IO_t frSkyLedPin;

#if defined(USE_RX_FRSKY_SPI_PA_LNA)
static IO_t txEnPin;
static IO_t rxLnaEnPin;
static IO_t antSelPin;
#endif

#ifdef USE_RX_FRSKY_SPI_TELEMETRY
int16_t rssiDbm;
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(rxFrSkySpiConfig_t, rxFrSkySpiConfig, PG_RX_FRSKY_SPI_CONFIG, 0);

PG_RESET_TEMPLATE(rxFrSkySpiConfig_t, rxFrSkySpiConfig,
    .autoBind = false,
    .bindTxId = {0, 0},
    .bindOffset = 0,
    .bindHopData = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    .rxNum = 0,
);

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
void setRssiDbm(uint8_t value)
{
    if (value >= 128) {
        rssiDbm = ((((uint16_t)value) * 18) >> 5) - 82;
    } else {
        rssiDbm = ((((uint16_t)value) * 18) >> 5) + 65;
    }

    setRssiUnfiltered(constrain(rssiDbm << 3, 0, 1023), RSSI_SOURCE_RX_PROTOCOL);
}
#endif // USE_RX_FRSKY_SPI_TELEMETRY

#if defined(USE_RX_FRSKY_SPI_PA_LNA)
void TxEnable(void)
{
    IOHi(txEnPin);
}

void TxDisable(void)
{
    IOLo(txEnPin);
}
#endif

void LedOn(void)
{
#if defined(RX_FRSKY_SPI_LED_PIN_INVERTED)
    IOLo(frSkyLedPin);
#else
    IOHi(frSkyLedPin);
#endif
}

void LedOff(void)
{
#if defined(RX_FRSKY_SPI_LED_PIN_INVERTED)
    IOHi(frSkyLedPin);
#else
    IOLo(frSkyLedPin);
#endif
}

void frSkySpiBind(void)
{
    bindRequested = true;
}

static void initialise() {
    cc2500Reset();
    cc2500WriteReg(CC2500_02_IOCFG0,   0x01);
    cc2500WriteReg(CC2500_17_MCSM1,    0x0C);
    cc2500WriteReg(CC2500_18_MCSM0,    0x18);
    cc2500WriteReg(CC2500_07_PKTCTRL1, 0x04);
    cc2500WriteReg(CC2500_3E_PATABLE,  0xFF);
    cc2500WriteReg(CC2500_0C_FSCTRL0,  0x00);
    cc2500WriteReg(CC2500_0D_FREQ2,    0x5C);
    cc2500WriteReg(CC2500_0E_FREQ1,    0x76);
    cc2500WriteReg(CC2500_0F_FREQ0,    0x27);
    cc2500WriteReg(CC2500_13_MDMCFG1,  0x23);
    cc2500WriteReg(CC2500_14_MDMCFG0,  0x7A);
    cc2500WriteReg(CC2500_19_FOCCFG,   0x16);
    cc2500WriteReg(CC2500_1A_BSCFG,    0x6C);
    cc2500WriteReg(CC2500_1B_AGCCTRL2, 0x03);
    cc2500WriteReg(CC2500_1C_AGCCTRL1, 0x40);
    cc2500WriteReg(CC2500_1D_AGCCTRL0, 0x91);
    cc2500WriteReg(CC2500_21_FREND1,   0x56);
    cc2500WriteReg(CC2500_22_FREND0,   0x10);
    cc2500WriteReg(CC2500_23_FSCAL3,   0xA9);
    cc2500WriteReg(CC2500_24_FSCAL2,   0x0A);
    cc2500WriteReg(CC2500_25_FSCAL1,   0x00);
    cc2500WriteReg(CC2500_26_FSCAL0,   0x11);
    cc2500WriteReg(CC2500_29_FSTEST,   0x59);
    cc2500WriteReg(CC2500_2C_TEST2,    0x88);
    cc2500WriteReg(CC2500_2D_TEST1,    0x31);
    cc2500WriteReg(CC2500_2E_TEST0,    0x0B);
    cc2500WriteReg(CC2500_03_FIFOTHR,  0x07);
    cc2500WriteReg(CC2500_09_ADDR,     0x00);

    switch (spiProtocol) {
    case RX_SPI_FRSKY_D:
        cc2500WriteReg(CC2500_06_PKTLEN,   0x19);
        cc2500WriteReg(CC2500_08_PKTCTRL0, 0x05);
        cc2500WriteReg(CC2500_0B_FSCTRL1,  0x08);
        cc2500WriteReg(CC2500_10_MDMCFG4,  0xAA);
        cc2500WriteReg(CC2500_11_MDMCFG3,  0x39);
        cc2500WriteReg(CC2500_12_MDMCFG2,  0x11);
        cc2500WriteReg(CC2500_15_DEVIATN,  0x42);

        break;
    case RX_SPI_FRSKY_X:
        cc2500WriteReg(CC2500_06_PKTLEN,   0x1E);
        cc2500WriteReg(CC2500_08_PKTCTRL0, 0x01);
        cc2500WriteReg(CC2500_0B_FSCTRL1,  0x0A);
        cc2500WriteReg(CC2500_10_MDMCFG4,  0x7B);
        cc2500WriteReg(CC2500_11_MDMCFG3,  0x61);
        cc2500WriteReg(CC2500_12_MDMCFG2,  0x13);
        cc2500WriteReg(CC2500_15_DEVIATN,  0x51);

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
}

void initialiseData(uint8_t adr)
{
    cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)rxFrSkySpiConfig()->bindOffset);
    cc2500WriteReg(CC2500_18_MCSM0, 0x8);
    cc2500WriteReg(CC2500_09_ADDR, adr ? 0x03 : rxFrSkySpiConfig()->bindTxId[0]);
    cc2500WriteReg(CC2500_07_PKTCTRL1, 0x0D);
    cc2500WriteReg(CC2500_19_FOCCFG, 0x16);
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

static bool tuneRx(uint8_t *packet)
{
    if (bindOffset >= 126) {
        bindOffset = -126;
    }
    if ((millis() - timeTunedMs) > 50) {
        timeTunedMs = millis();
        bindOffset += 5;
        cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset);
    }
    if (IORead(gdoPin)) {
        uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
        if (ccLen) {
            cc2500ReadFifo(packet, ccLen);
            if (packet[ccLen - 1] & 0x80) {
                if (packet[2] == 0x01) {
                    uint8_t Lqi = packet[ccLen - 1] & 0x7F;
                    if (Lqi < 50) {
                        rxFrSkySpiConfigMutable()->bindOffset = bindOffset;

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
                        rxFrSkySpiConfigMutable()->bindTxId[0] = packet[3];
                        rxFrSkySpiConfigMutable()->bindTxId[1] = packet[4];
                        for (uint8_t n = 0; n < 5; n++) {
                            rxFrSkySpiConfigMutable()->bindHopData[packet[5] + n] =
                                packet[6 + n];
                        }

                        rxFrSkySpiConfigMutable()->rxNum = packet[12];

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
                        if ((packet[3] == rxFrSkySpiConfig()->bindTxId[0]) &&
                            (packet[4] == rxFrSkySpiConfig()->bindTxId[1])) {
                            if (packet[5] == bindIdx) {
#if defined(DJTS)
                                if (packet[5] == 0x2D) {
                                    for (uint8_t i = 0; i < 2; i++) {
                                        rxFrSkySpiConfigMutable()->bindHopData[packet[5] + i] = packet[6 + i];
                                    }
                                    listLength = 47;

                                    return true;
                                }
#endif

                                for (uint8_t n = 0; n < 5; n++) {
                                    if (packet[6 + n] == packet[ccLen - 3] || (packet[6 + n] == 0)) {
                                        if (bindIdx >= 0x2D) {
                                            listLength = packet[5] + n;

                                            return true;
                                        }
                                    }

                                    rxFrSkySpiConfigMutable()->bindHopData[packet[5] + n] = packet[6 + n];
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

bool checkBindRequested(bool reset)
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
        if (checkBindRequested(true) || rxFrSkySpiConfig()->autoBind) {
            LedOn();
            initTuneRx();

            protocolState = STATE_BIND_TUNING;
        } else {
            protocolState = STATE_STARTING;
        }

        break;
    case STATE_BIND_TUNING:
       if (tuneRx(packet)) {
            initGetBind();
            initialiseData(1);

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
        if (!rxFrSkySpiConfig()->autoBind) {
            writeEEPROM();
        } else {
            uint8_t ctr = 40;
            while (ctr--) {
                LedOn();
                delay(50);
                LedOff();
                delay(50);
            }
        }

        ret = RX_SPI_RECEIVED_BIND;
        protocolState = STATE_STARTING;

        break;
    default:
        ret = handlePacket(packet, &protocolState);

        break;
    }

    return ret;
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
                    calData[rxFrSkySpiConfig()->bindHopData[channr]][0]);
    cc2500WriteReg(CC2500_24_FSCAL2,
                    calData[rxFrSkySpiConfig()->bindHopData[channr]][1]);
    cc2500WriteReg(CC2500_25_FSCAL1,
                    calData[rxFrSkySpiConfig()->bindHopData[channr]][2]);
    cc2500WriteReg(CC2500_0A_CHANNR, rxFrSkySpiConfig()->bindHopData[channr]);
    if (spiProtocol == RX_SPI_FRSKY_D) {
        cc2500Strobe(CC2500_SFRX);
    }
}

#if defined(USE_RX_FRSKY_SPI_PA_LNA) && defined(USE_RX_FRSKY_SPI_DIVERSITY)
void switchAntennae(void)
{
    static bool alternativeAntennaSelected = true;

    if (alternativeAntennaSelected) {
        IOLo(antSelPin);
    } else {
        IOHi(antSelPin);
    }
    alternativeAntennaSelected = !alternativeAntennaSelected;
}
#endif

static bool frSkySpiDetect(void)
{
    const uint8_t chipPartNum = cc2500ReadReg(CC2500_30_PARTNUM | CC2500_READ_BURST); //CC2500 read registers chip part num
    const uint8_t chipVersion = cc2500ReadReg(CC2500_31_VERSION | CC2500_READ_BURST); //CC2500 read registers chip version
    if (chipPartNum == 0x80 && chipVersion == 0x03) {
        return true;
    }

    return false;
}

bool frSkySpiInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
#if !defined(RX_FRSKY_SPI_DISABLE_CHIP_DETECTION)
    if (!frSkySpiDetect()) {
        return false;
    }
#else
    UNUSED(frSkySpiDetect);
#endif

    spiProtocol = rxConfig->rx_spi_protocol;

    switch (spiProtocol) {
    case RX_SPI_FRSKY_D:
        rxRuntimeConfig->channelCount = RC_CHANNEL_COUNT_FRSKY_D;

        handlePacket = frSkyDHandlePacket;
        setRcData = frSkyDSetRcData;
        frSkyDInit();

        break;
    case RX_SPI_FRSKY_X:
        rxRuntimeConfig->channelCount = RC_CHANNEL_COUNT_FRSKY_X;

        handlePacket = frSkyXHandlePacket;
        setRcData = frSkyXSetRcData;
        frSkyXInit();

        break;
    default:

        break;
    }

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }
#endif

    // gpio init here
    gdoPin = IOGetByTag(IO_TAG(RX_FRSKY_SPI_GDO_0_PIN));
    IOInit(gdoPin, OWNER_RX_SPI, 0);
    IOConfigGPIO(gdoPin, IOCFG_IN_FLOATING);
    frSkyLedPin = IOGetByTag(IO_TAG(RX_FRSKY_SPI_LED_PIN));
    IOInit(frSkyLedPin, OWNER_LED, 0);
    IOConfigGPIO(frSkyLedPin, IOCFG_OUT_PP);
#if defined(USE_RX_FRSKY_SPI_PA_LNA)
    rxLnaEnPin = IOGetByTag(IO_TAG(RX_FRSKY_SPI_LNA_EN_PIN));
    IOInit(rxLnaEnPin, OWNER_RX_SPI, 0);
    IOConfigGPIO(rxLnaEnPin, IOCFG_OUT_PP);
    IOHi(rxLnaEnPin); // always on at the moment
    txEnPin = IOGetByTag(IO_TAG(RX_FRSKY_SPI_TX_EN_PIN));
    IOInit(txEnPin, OWNER_RX_SPI, 0);
    IOConfigGPIO(txEnPin, IOCFG_OUT_PP);
#if defined(USE_RX_FRSKY_SPI_DIVERSITY)
    antSelPin = IOGetByTag(IO_TAG(RX_FRSKY_SPI_ANT_SEL_PIN));
    IOInit(antSelPin, OWNER_RX_SPI, 0);
    IOConfigGPIO(antSelPin, IOCFG_OUT_PP);
#endif
#endif // USE_RX_FRSKY_SPI_PA_LNA
#if defined(BINDPLUG_PIN)
    bindPin = IOGetByTag(IO_TAG(BINDPLUG_PIN));
    IOInit(bindPin, OWNER_RX_BIND, 0);
    IOConfigGPIO(bindPin, IOCFG_IPU);

    lastBindPinStatus = IORead(bindPin);
#endif

#if defined(USE_RX_FRSKY_SPI_PA_LNA)
#if defined(USE_RX_FRSKY_SPI_DIVERSITY)
    IOHi(antSelPin);
#endif
    TxDisable();
#endif // USE_RX_FRSKY_SPI_PA_LNA

    missingPackets = 0;
    timeoutUs = 50;

    start_time = millis();
    protocolState = STATE_INIT;

    return true;
}
#endif
