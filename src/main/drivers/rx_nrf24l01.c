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

#include "system.h"
#include "gpio.h"
#include "rx_nrf24l01.h"

#ifdef UNIT_TEST

#define NRF24_CE_HI() {}
#define NRF24_CE_LO() {}
void NRF24L01_SpiInit(void) {}

#else

#include "bus_spi.h"
#include "bus_spi_soft.h"

#define DISABLE_NRF24()     {GPIO_SetBits(NRF24_CSN_GPIO, NRF24_CSN_PIN);}
#define ENABLE_NRF24()      {GPIO_ResetBits(NRF24_CSN_GPIO, NRF24_CSN_PIN);}
#define NRF24_CE_HI()       {GPIO_SetBits(NRF24_CE_GPIO, NRF24_CE_PIN);}
#define NRF24_CE_LO()       {GPIO_ResetBits(NRF24_CE_GPIO, NRF24_CE_PIN);}

#ifdef USE_NRF24_SOFTSPI
static const softSPIDevice_t softSPI = {
    .sck_gpio = NRF24_SCK_GPIO,
    .mosi_gpio = NRF24_MOSI_GPIO,
    .miso_gpio = NRF24_MISO_GPIO,
    .sck_pin = NRF24_SCK_PIN,
    .mosi_pin = NRF24_MOSI_PIN,
    .miso_pin = NRF24_MISO_PIN
};
#endif

void NRF24L01_SpiInit(void)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }
#ifdef USE_NRF24_SOFTSPI
    softSpiInit(&softSPI);
#endif
    // Note: Nordic Semiconductor uses 'CSN', STM uses 'NSS'
#ifdef STM32F303xC
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
#ifndef USE_NRF24_SOFTSPI
    // CSN as output
    RCC_AHBPeriphClockCmd(NRF24_CSN_GPIO_CLK_PERIPHERAL, ENABLE);
    GPIO_InitStructure.GPIO_Pin = NRF24_CSN_PIN;
    GPIO_Init(NRF24_CSN_GPIO, &GPIO_InitStructure);
#endif
    // CE as OUTPUT
    RCC_AHBPeriphClockCmd(NRF24_CE_GPIO_CLK_PERIPHERAL, ENABLE);
    GPIO_InitStructure.GPIO_Pin = NRF24_CE_PIN;
    GPIO_Init(NRF24_CE_GPIO, &GPIO_InitStructure);
#endif // STM32F303xC

#ifdef STM32F10X
    gpio_config_t gpio;
    gpio.speed = Speed_50MHz;
    gpio.mode = Mode_Out_PP;
#ifndef USE_NRF24_SOFTSPI
    // CSN as output
    RCC_APB2PeriphClockCmd(NRF24_CSN_GPIO_CLK_PERIPHERAL, ENABLE);
    gpio.pin = NRF24_CSN_PIN;
    gpioInit(NRF24_CSN_GPIO, &gpio);
#endif
    // CE as output
    RCC_APB2PeriphClockCmd(NRF24_CE_GPIO_CLK_PERIPHERAL, ENABLE);
    gpio.pin = NRF24_CE_PIN;
    gpioInit(NRF24_CE_GPIO, &gpio);
    // TODO: NRF24_IRQ as input
#endif // STM32F10X

    DISABLE_NRF24();
    NRF24_CE_LO();

#ifdef NRF24_SPI_INSTANCE
    spiSetDivisor(NRF24_SPI_INSTANCE, SPI_9MHZ_CLOCK_DIVIDER);
#endif
    hardwareInitialised = true;
}

uint8_t nrf24TransferByte(uint8_t data)
{
#ifdef USE_NRF24_SOFTSPI
    return softSpiTransferByte(&softSPI, data);
#else
    return spiTransferByte(NRF24_SPI_INSTANCE, data);
#endif
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

uint8_t NRF24L01_WriteRegisterMulti(uint8_t reg, const uint8_t *data, uint8_t length)
{
    ENABLE_NRF24();
    const uint8_t ret = nrf24TransferByte(W_REGISTER | ( REGISTER_MASK & reg));
    for (uint8_t i = 0; i < length; i++) {
        nrf24TransferByte(data[i]);
    }
    DISABLE_NRF24();
    return ret;
}

/*
 * Transfer the payload to the nRF24L01 TX FIFO
 * Packets in the TX FIFO are transmitted when the
 * nRF24L01 next enters TX mode
 */
uint8_t NRF24L01_WritePayload(const uint8_t *data, uint8_t length)
{
    ENABLE_NRF24();
    const uint8_t ret = nrf24TransferByte(W_TX_PAYLOAD);
    for (uint8_t i = 0; i < length; i++) {
        nrf24TransferByte(data[i]);
    }
    DISABLE_NRF24();
    return ret;
}

uint8_t NRF24L01_ReadReg(uint8_t reg)
{
    ENABLE_NRF24();
    nrf24TransferByte(R_REGISTER | (REGISTER_MASK & reg));
    const uint8_t ret = nrf24TransferByte(NOP);
    DISABLE_NRF24();
    return ret;
}

uint8_t NRF24L01_ReadRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length)
{
    ENABLE_NRF24();
    const uint8_t ret = nrf24TransferByte(R_REGISTER | (REGISTER_MASK & reg));
    for (uint8_t i = 0; i < length; i++) {
        data[i] = nrf24TransferByte(NOP);
    }
    DISABLE_NRF24();
    return ret;
}

/*
 * Read a packet from the nRF24L01 RX FIFO.
 */
uint8_t NRF24L01_ReadPayload(uint8_t *data, uint8_t length)
{
    ENABLE_NRF24();
    const uint8_t ret = nrf24TransferByte(R_RX_PAYLOAD);
    for (uint8_t i = 0; i < length; i++) {
        data[i] = nrf24TransferByte(NOP);
    }
    DISABLE_NRF24();
    return ret;
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
bool NRF24L01_ReadPayloadIfAvailableFast(uint8_t *data, uint8_t length)
{
    // number of bits transferred = 8 * (3 + length)
    // for 16 byte payload, that is 8*19 = 152
    // at 50MHz clock rate that is approximately 3 microseconds
    bool ret = false;
    ENABLE_NRF24();
    nrf24TransferByte(R_REGISTER | (REGISTER_MASK & NRF24L01_17_FIFO_STATUS));
    const uint8_t fifoStatus = nrf24TransferByte(NOP);
    if ((fifoStatus & BV(NRF24L01_17_FIFO_STATUS_RX_EMPTY)) == 0) {
        ret = true;
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
