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

// This file is copied with modifications from project Deviation,
// see http://deviationtx.com

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#ifdef USE_RX_NRF24

#include "build/build_config.h"

#include "drivers/time.h"
#include "drivers/gpio.h"
#include "drivers/io.h"
#include "io_impl.h"
#include "rcc.h"
#include "rx_spi.h"
#include "rx_nrf24l01.h"
#include "drivers/bus_spi.h"

#define NRF24_CE_HI()       {IOHi(IOGetByTag(IO_TAG(RX_CE_PIN)));}
#define NRF24_CE_LO()       {IOLo(IOGetByTag(IO_TAG(RX_CE_PIN)));}

// Instruction Mnemonics
// nRF24L01:  Table 16. Command set for the nRF24L01 SPI. Product Specification, p46
// nRF24L01+: Table 20. Command set for the nRF24L01+ SPI. Product Specification, p51
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

uint8_t NRF24L01_WriteReg(uint8_t reg, uint8_t data)
{
    return rxSpiWriteCommand(W_REGISTER | (REGISTER_MASK & reg), data);
}

uint8_t NRF24L01_WriteRegisterMulti(uint8_t reg, const uint8_t *data, uint8_t length)
{
    return rxSpiWriteCommandMulti(W_REGISTER | ( REGISTER_MASK & reg), data, length);
}

/*
 * Transfer the payload to the nRF24L01 TX FIFO
 * Packets in the TX FIFO are transmitted when the
 * nRF24L01 next enters TX mode
 */
uint8_t NRF24L01_WritePayload(const uint8_t *data, uint8_t length)
{
    return rxSpiWriteCommandMulti(W_TX_PAYLOAD, data, length);
}

uint8_t NRF24L01_WriteAckPayload(const uint8_t *data, uint8_t length, uint8_t pipe)
{
    return rxSpiWriteCommandMulti(W_ACK_PAYLOAD | (pipe & 0x07), data, length);
}

uint8_t NRF24L01_ReadReg(uint8_t reg)
{
    return rxSpiReadCommand(R_REGISTER | (REGISTER_MASK & reg), NOP);
}

uint8_t NRF24L01_ReadRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length)
{
    return rxSpiReadCommandMulti(R_REGISTER | (REGISTER_MASK & reg), NOP, data, length);
}

/*
 * Read a packet from the nRF24L01 RX FIFO.
 */
uint8_t NRF24L01_ReadPayload(uint8_t *data, uint8_t length)
{
    return rxSpiReadCommandMulti(R_RX_PAYLOAD, NOP, data, length);
}

/*
 * Empty the transmit FIFO buffer.
 */
void NRF24L01_FlushTx()
{
    rxSpiWriteByte(FLUSH_TX);
}

/*
 * Empty the receive FIFO buffer.
 */
void NRF24L01_FlushRx()
{
    rxSpiWriteByte(FLUSH_RX);
}

uint8_t NRF24L01_Activate(uint8_t code)
{
    return rxSpiWriteCommand(ACTIVATE, code);
}

// standby configuration, used to simplify switching between RX, TX, and Standby modes
static uint8_t standbyConfig;

void NRF24L01_Initialize(uint8_t baseConfig)
{
    standbyConfig = BV(NRF24L01_00_CONFIG_PWR_UP) | baseConfig;
    NRF24_CE_LO();
    // nRF24L01+ needs 100 milliseconds settling time from PowerOnReset to PowerDown mode
    static const timeUs_t settlingTimeUs = 100000;
    const timeUs_t currentTimeUs = micros();
    if (currentTimeUs < settlingTimeUs) {
        delayMicroseconds(settlingTimeUs - currentTimeUs);
    }
    // now in PowerDown mode
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, standbyConfig); // set PWR_UP to enter Standby mode
    // nRF24L01+ needs 4500 microseconds from PowerDown mode to Standby mode, for crystal oscillator startup
    delayMicroseconds(4500);
    // now in Standby mode
}

/*
 * Common setup of registers
 */
void NRF24L01_SetupBasic(void)
{
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00); // No auto acknowledgment
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, BV(NRF24L01_02_EN_RXADDR_ERX_P0));
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, NRF24L01_03_SETUP_AW_5BYTES); // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00); // Disable dynamic payload length on all pipes
}

/*
 * Enter standby mode
 */
void NRF24L01_SetStandbyMode(void)
{
    // set CE low and clear the PRIM_RX bit to enter standby mode
    NRF24_CE_LO();
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, standbyConfig);
}

/*
 * Enter receive mode
 */
void NRF24L01_SetRxMode(void)
{
    NRF24_CE_LO(); // drop into standby mode
    // set the PRIM_RX bit
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, standbyConfig | BV(NRF24L01_00_CONFIG_PRIM_RX));
    NRF24L01_ClearAllInterrupts();
    // finally set CE high to start enter RX mode
    NRF24_CE_HI();
    // nRF24L01+ will now transition from Standby mode to RX mode after 130 microseconds settling time
}

/*
 * Enter transmit mode. Anything in the transmit FIFO will be transmitted.
 */
void NRF24L01_SetTxMode(void)
{
    // Ensure in standby mode, since can only enter TX mode from standby mode
    NRF24L01_SetStandbyMode();
    NRF24L01_ClearAllInterrupts();
    // pulse CE for 10 microseconds to enter TX mode
    NRF24_CE_HI();
    delayMicroseconds(10);
    NRF24_CE_LO();
    // nRF24L01+ will now transition from Standby mode to TX mode after 130 microseconds settling time.
    // Transmission will then begin and continue until TX FIFO is empty.
}

void NRF24L01_ClearAllInterrupts(void)
{
    // Writing to the STATUS register clears the specified interrupt bits
    NRF24L01_WriteReg(NRF24L01_07_STATUS, BV(NRF24L01_07_STATUS_RX_DR) | BV(NRF24L01_07_STATUS_TX_DS) | BV(NRF24L01_07_STATUS_MAX_RT));
}

void NRF24L01_SetChannel(uint8_t channel)
{
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, channel);
}

bool NRF24L01_ReadPayloadIfAvailable(uint8_t *data, uint8_t length)
{
    if (NRF24L01_ReadReg(NRF24L01_17_FIFO_STATUS) & BV(NRF24L01_17_FIFO_STATUS_RX_EMPTY)) {
        return false;
    }
    NRF24L01_ReadPayload(data, length);
    return true;
}

#ifndef UNIT_TEST
#define DISABLE_RX()    {IOHi(IOGetByTag(IO_TAG(RX_NSS_PIN)));}
#define ENABLE_RX()     {IOLo(IOGetByTag(IO_TAG(RX_NSS_PIN)));}
/*
 * Fast read of payload, for use in interrupt service routine
 */
bool NRF24L01_ReadPayloadIfAvailableFast(uint8_t *data, uint8_t length)
{
    // number of bits transferred = 8 * (3 + length)
    // for 16 byte payload, that is 8*19 = 152
    // at 50MHz clock rate that is approximately 3 microseconds
    bool ret = false;
    ENABLE_RX();
    rxSpiTransferByte(R_REGISTER | (REGISTER_MASK & NRF24L01_07_STATUS));
    const uint8_t status = rxSpiTransferByte(NOP);
    if ((status & BV(NRF24L01_07_STATUS_RX_DR)) == 0) {
        ret = true;
        // clear RX_DR flag
        rxSpiTransferByte(W_REGISTER | (REGISTER_MASK & NRF24L01_07_STATUS));
        rxSpiTransferByte(BV(NRF24L01_07_STATUS_RX_DR));
        rxSpiTransferByte(R_RX_PAYLOAD);
        for (uint8_t i = 0; i < length; i++) {
            data[i] = rxSpiTransferByte(NOP);
        }
    }
    DISABLE_RX();
    return ret;
}
#endif // UNIT_TEST
#endif // USE_RX_NRF24
