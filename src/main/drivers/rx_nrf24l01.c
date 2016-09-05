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

#include "system.h"
#include "gpio.h"
#include "io.h"
#include "io_impl.h"
#include "rcc.h"
#include "rx_nrf24l01.h"

#ifdef UNIT_TEST

#define NRF24_CE_HI() {}
#define NRF24_CE_LO() {}
void NRF24L01_SpiInit(void) {}

#else

#include "bus_spi.h"
#include "bus_spi_soft.h"

#define DISABLE_NRF24()     {IOHi(IOGetByTag(IO_TAG(NRF24_CSN_PIN)));}
#define ENABLE_NRF24()      {IOLo(IOGetByTag(IO_TAG(NRF24_CSN_PIN)));}
#define NRF24_CE_HI()       {IOHi(IOGetByTag(IO_TAG(NRF24_CE_PIN)));}
#define NRF24_CE_LO()       {IOLo(IOGetByTag(IO_TAG(NRF24_CE_PIN)));}

#ifdef USE_NRF24_SOFTSPI
static const softSPIDevice_t softSPIDevice = {
    .sckTag = IO_TAG(NRF24_SCK_PIN),
    .mosiTag = IO_TAG(NRF24_MOSI_PIN),
    .misoTag = IO_TAG(NRF24_MISO_PIN),
    // Note: Nordic Semiconductor uses 'CSN', STM uses 'NSS'
    .nssTag = IO_TAG(NRF24_CSN_PIN),
};
#endif // USE_NRF24_SOFTSPI

#ifdef USE_NRF24_SOFTSPI
static bool useSoftSPI = false;
#endif
void NRF24L01_SpiInit(nfr24l01_spi_type_e spiType)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }

#ifdef USE_NRF24_SOFTSPI
    if (spiType == NFR24L01_SOFTSPI) {
        useSoftSPI = true;
        softSpiInit(&softSPIDevice);
    }
    const SPIDevice nrf24SPIDevice = SOFT_SPIDEV_1;
#else
    UNUSED(spiType);
    const SPIDevice nrf24SPIDevice = spiDeviceByInstance(NRF24_SPI_INSTANCE);
    IOInit(IOGetByTag(IO_TAG(NRF24_CSN_PIN)), OWNER_SPI, RESOURCE_SPI_CS, nrf24SPIDevice + 1);
#endif // USE_NRF24_SOFTSPI

#if defined(STM32F10X)
    RCC_AHBPeriphClockCmd(NRF24_CSN_GPIO_CLK_PERIPHERAL, ENABLE);
    RCC_AHBPeriphClockCmd(NRF24_CE_GPIO_CLK_PERIPHERAL, ENABLE);
#endif

    // CE as OUTPUT
    IOInit(IOGetByTag(IO_TAG(NRF24_CE_PIN)),  OWNER_NRF24, RESOURCE_NRF24_CE,  nrf24SPIDevice + 1);
#if defined(STM32F10X)
    IOConfigGPIO(IOGetByTag(IO_TAG(NRF24_CE_PIN)), SPI_IO_CS_CFG);
#elif defined(STM32F3) || defined(STM32F4)
    IOConfigGPIOAF(IOGetByTag(IO_TAG(NRF24_CE_PIN)), SPI_IO_CS_CFG, 0);
#endif

    DISABLE_NRF24();
    NRF24_CE_LO();

#ifdef NRF24_SPI_INSTANCE
    spiSetDivisor(NRF24_SPI_INSTANCE, SPI_CLOCK_STANDARD);
#endif
    hardwareInitialised = true;
}

uint8_t nrf24TransferByte(uint8_t data)
{
#ifdef USE_NRF24_SOFTSPI
    if (useSoftSPI) {
        return softSpiTransferByte(&softSPIDevice, data);
    } else
#endif
    {
#ifdef NRF24_SPI_INSTANCE
        return spiTransferByte(NRF24_SPI_INSTANCE, data);
#else
        return 0;
#endif
    }
}

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
    ENABLE_NRF24();
    nrf24TransferByte(W_REGISTER | (REGISTER_MASK & reg));
    nrf24TransferByte(data);
    DISABLE_NRF24();
    return true;
}

static uint8_t NRF24L01_WriteMulti(uint8_t type, const uint8_t *data, uint8_t length)
{
    ENABLE_NRF24();
    const uint8_t ret = nrf24TransferByte(type);
    for (uint8_t i = 0; i < length; i++) {
        nrf24TransferByte(data[i]);
    }
    DISABLE_NRF24();
    return ret;
}

uint8_t NRF24L01_WriteRegisterMulti(uint8_t reg, const uint8_t *data, uint8_t length)
{
    return NRF24L01_WriteMulti(W_REGISTER | ( REGISTER_MASK & reg), data, length);
}

/*
 * Transfer the payload to the nRF24L01 TX FIFO
 * Packets in the TX FIFO are transmitted when the
 * nRF24L01 next enters TX mode
 */
uint8_t NRF24L01_WritePayload(const uint8_t *data, uint8_t length)
{
    return NRF24L01_WriteMulti(W_TX_PAYLOAD, data, length);
}

uint8_t NRF24L01_WriteAckPayload(const uint8_t *data, uint8_t length, uint8_t pipe)
{
    return NRF24L01_WriteMulti(W_ACK_PAYLOAD | (pipe & 0x07), data, length);
}

uint8_t NRF24L01_ReadReg(uint8_t reg)
{
    ENABLE_NRF24();
    nrf24TransferByte(R_REGISTER | (REGISTER_MASK & reg));
    const uint8_t ret = nrf24TransferByte(NOP);
    DISABLE_NRF24();
    return ret;
}

static uint8_t NRF24L01_ReadMulti(uint8_t type, uint8_t *data, uint8_t length)
{
    ENABLE_NRF24();
    const uint8_t ret = nrf24TransferByte(type);
    for (uint8_t i = 0; i < length; i++) {
        data[i] = nrf24TransferByte(NOP);
    }
    DISABLE_NRF24();
    return ret;
}

uint8_t NRF24L01_ReadRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length)
{
    return NRF24L01_ReadMulti(R_REGISTER | (REGISTER_MASK & reg), data, length);
}

/*
 * Read a packet from the nRF24L01 RX FIFO.
 */
uint8_t NRF24L01_ReadPayload(uint8_t *data, uint8_t length)
{
    return NRF24L01_ReadMulti(R_RX_PAYLOAD, data, length);
}

/*
 * Empty the transmit FIFO buffer.
 */
void NRF24L01_FlushTx()
{
    ENABLE_NRF24();
    nrf24TransferByte(FLUSH_TX);
    DISABLE_NRF24();
}

/*
 * Empty the receive FIFO buffer.
 */
void NRF24L01_FlushRx()
{
    ENABLE_NRF24();
    nrf24TransferByte(FLUSH_RX);
    DISABLE_NRF24();
}

#endif // UNIT_TEST

// standby configuration, used to simplify switching between RX, TX, and Standby modes
static uint8_t standbyConfig;

void NRF24L01_Initialize(uint8_t baseConfig)
{
    standbyConfig = BV(NRF24L01_00_CONFIG_PWR_UP) | baseConfig;
    NRF24_CE_LO();
    // nRF24L01+ needs 100 milliseconds settling time from PowerOnReset to PowerDown mode
    static const uint32_t settlingTimeUs = 100000;
    const uint32_t currentTimeUs = micros();
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
/*
 * Fast read of payload, for use in interrupt service routine
 */
bool NRF24L01_ReadPayloadIfAvailableFast(uint8_t *data, uint8_t length)
{
    // number of bits transferred = 8 * (3 + length)
    // for 16 byte payload, that is 8*19 = 152
    // at 50MHz clock rate that is approximately 3 microseconds
    bool ret = false;
    ENABLE_NRF24();
    nrf24TransferByte(R_REGISTER | (REGISTER_MASK & NRF24L01_07_STATUS));
    const uint8_t status = nrf24TransferByte(NOP);
    if ((status & BV(NRF24L01_07_STATUS_RX_DR)) == 0) {
        ret = true;
        // clear RX_DR flag
        nrf24TransferByte(W_REGISTER | (REGISTER_MASK & NRF24L01_07_STATUS));
        nrf24TransferByte(BV(NRF24L01_07_STATUS_RX_DR));
        nrf24TransferByte(R_RX_PAYLOAD);
        for (uint8_t i = 0; i < length; i++) {
            data[i] = nrf24TransferByte(NOP);
        }
    }
    DISABLE_NRF24();
    return ret;
}
#endif
#endif
