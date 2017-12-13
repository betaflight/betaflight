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

#include "drivers/cc2500.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "fc/config.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "rx/rx.h"

#include "rx/cc2500_frsky_common.h"

#include "cc2500_frsky_shared.h"

static uint32_t missingPackets;
static uint8_t calData[255][3];
static timeMs_t timeTunedMs;
uint8_t listLength;
static uint8_t bindIdx;
static uint8_t Lqi;
static uint8_t protocolState;
static timeMs_t timeStartedMs;
static int8_t bindOffset;
static bool lastBindPinStatus;
bool bindRequested = false;

IO_t gdoPin;
static IO_t bindPin = DEFIO_IO(NONE);
IO_t frSkyLedPin;

#if defined(USE_RX_FRSKY_SPI_PA_LNA)
static IO_t txEnPin;
static IO_t rxLnaEnPin;
IO_t antSelPin;
#endif

#ifdef USE_RX_FRSKY_SPI_TELEMETRY
int16_t RSSI_dBm;
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
        RSSI_dBm = ((((uint16_t)value) * 18) >> 5) - 82;
    } else {
        RSSI_dBm = ((((uint16_t)value) * 18) >> 5) + 65;
    }

    setRssiUnfiltered(constrain(RSSI_dBm << 3, 0, 1023), RSSI_SOURCE_RX_PROTOCOL);
}
#endif // USE_RX_FRSKY_SPI_TELEMETRY

#if defined(USE_RX_FRSKY_SPI_PA_LNA)
void RxEnable(void)
{
    IOLo(txEnPin);
}

void TxEnable(void)
{
    IOHi(txEnPin);
}
#endif

void frSkyBind(void)
{
    bindRequested = true;
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
                    Lqi = packet[ccLen - 1] & 0x7F;
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

void handleBinding(uint8_t protocolState, uint8_t *packet)
{
    switch (protocolState) {
    case STATE_BIND:
        if (checkBindRequested(true) || rxFrSkySpiConfig()->autoBind) {
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
                IOHi(frSkyLedPin);
                delay(50);
                IOLo(frSkyLedPin);
                delay(50);
            }
        }

        protocolState = STATE_STARTING;

        break;
    }
}

void nextChannel(uint8_t skip, bool sendStrobe)
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
    if (sendStrobe) {
        cc2500Strobe(CC2500_SFRX);
    }
}

void frskySpiRxSetup()
{
    // gpio init here
    gdoPin = IOGetByTag(IO_TAG(RX_FRSKY_SPI_GDO_0_PIN));
    IOInit(gdoPin, OWNER_RX_BIND, 0);
    IOConfigGPIO(gdoPin, IOCFG_IN_FLOATING);
    frSkyLedPin = IOGetByTag(IO_TAG(RX_FRSKY_SPI_LED_PIN));
    IOInit(frSkyLedPin, OWNER_LED, 0);
    IOConfigGPIO(frSkyLedPin, IOCFG_OUT_PP);
#if defined(USE_RX_FRSKY_SPI_PA_LNA)
    rxLnaEnPin = IOGetByTag(IO_TAG(RX_FRSKY_SPI_LNA_EN_PIN));
    IOInit(rxLnaEnPin, OWNER_RX_BIND, 0);
    IOConfigGPIO(rxLnaEnPin, IOCFG_OUT_PP);
    IOHi(rxLnaEnPin); // always on at the moment
    txEnPin = IOGetByTag(IO_TAG(RX_FRSKY_SPI_TX_EN_PIN));
    IOInit(txEnPin, OWNER_RX_BIND, 0);
    IOConfigGPIO(txEnPin, IOCFG_OUT_PP);
#if defined(USE_RX_FRSKY_SPI_DIVERSITY)
    antSelPin = IOGetByTag(IO_TAG(RX_FRSKY_SPI_ANT_SEL_PIN));
    IOInit(antSelPin, OWNER_RX_BIND, 0);
    IOConfigGPIO(antSelPin, IOCFG_OUT_PP);
#endif
#endif // USE_RX_FRSKY_SPI_PA_LNA
#if defined(BINDPLUG_PIN)
    bindPin = IOGetByTag(IO_TAG(BINDPLUG_PIN));
    IOInit(bindPin, OWNER_RX_BIND, 0);
    IOConfigGPIO(bindPin, IOCFG_IPU);

    lastBindPinStatus = IORead(bindPin);
#endif

    timeStartedMs = millis();
    missingPackets = 0;
    protocolState = STATE_INIT;
#if defined(USE_RX_FRSKY_SPI_PA_LNA)
#if defined(USE_RX_FRSKY_SPI_DIVERSITY)
    IOHi(antSelPin);
#endif
    RxEnable();
#endif // USE_RX_FRSKY_SPI_PA_LNA

    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }

    // if(!frSkySpiDetect())//detect spi working routine
    // return;
}
#endif
