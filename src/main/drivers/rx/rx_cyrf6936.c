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
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_RX_SPEKTRUM

#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/rx/rx_cyrf6936.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/time.h"

static IO_t rxIntIO = IO_NONE;
static extiCallbackRec_t cyrf6936extiCallbackRec;
static volatile uint32_t timeEvent = 0;
static volatile bool occurEvent = false;
volatile bool isError = false;

void cyrf6936ExtiHandler(extiCallbackRec_t *cb)
{
    UNUSED(cb);

    if (IORead(rxIntIO) == 0) {
        timeEvent = micros();
        occurEvent = true;
    }
}

bool cyrf6936RxFinished(uint32_t *timeStamp)
{
    if (occurEvent) {
        if (timeStamp) {
            *timeStamp = timeEvent;
        }

        uint8_t rxIrqStatus = cyrf6936ReadRegister(CYRF6936_RX_IRQ_STATUS);
        if ((rxIrqStatus & CYRF6936_RXC_IRQ) || (rxIrqStatus & CYRF6936_RXE_IRQ)) {
            isError = (rxIrqStatus & CYRF6936_RXE_IRQ) > 0x0;
        }

        occurEvent = false;
        return true;
    }
    return false;
}

bool cyrf6936Init(IO_t extiPin)
{
    spiDeviceByInstance(RX_SPI_INSTANCE);
    rxIntIO = extiPin;
    IOInit(rxIntIO, OWNER_RX_SPI_EXTI, 0);
    EXTIHandlerInit(&cyrf6936extiCallbackRec, cyrf6936ExtiHandler);
    EXTIConfig(rxIntIO, &cyrf6936extiCallbackRec, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IPD, EXTI_TRIGGER_FALLING);
    EXTIEnable(rxIntIO, false);

    uint16_t timeout = 1000;
    do { // Check if chip has waken up
        cyrf6936WriteRegister(CYRF6936_XACT_CFG, 0x82);
    } while ((cyrf6936ReadRegister(CYRF6936_XACT_CFG) != 0x82) && timeout--);

    // Soft reset
    cyrf6936WriteRegister(CYRF6936_MODE_OVERRIDE, CYRF6936_RST);

    // Verify the CYRF chip is responding
    return cyrf6936ReadRegister(CYRF6936_FRAMING_CFG) == 0xA5;
}

void cyrf6936WriteRegister(const uint8_t address, const uint8_t data)
{
    rxSpiWriteCommand(CYRF6936_DIR | address, data);
}

void cyrf6936WriteBlock(const uint8_t address, const uint8_t *data, const uint8_t length)
{
    rxSpiWriteCommandMulti(CYRF6936_DIR | address, &data[0], length);
}

uint8_t cyrf6936ReadRegister(const uint8_t address)
{
    return rxSpiReadCommand(address, 0xFF);
}

void cyrf6936ReadBlock(const uint8_t address, uint8_t data[], const uint8_t length)
{
    rxSpiReadCommandMulti(address, 0xFF, &data[0], length);
}

uint8_t cyrf6936GetRssi(void)
{
    return cyrf6936ReadRegister(CYRF6936_RSSI) & 0x1F;  //5 bit value 0 - 31
}

uint8_t cyrf6936GetRxStatus(void)
{
    return cyrf6936ReadRegister(CYRF6936_RX_STATUS);
}

void cyrf6936SetConfigLen(const uint8_t config[][2], const uint8_t length)
{
    for (unsigned i = 0; i < length; i++) {
        cyrf6936WriteRegister(config[i][0], config[i][1]);
    }
}

void cyrf6936SetChannel(const uint8_t chan)
{
    cyrf6936WriteRegister(CYRF6936_CHANNEL, chan);
}

void cyrf6936SetMode(const uint8_t mode, const bool force)
{
    if (force) {
        cyrf6936WriteRegister(CYRF6936_XACT_CFG, mode | CYRF6936_FRC_END);
    } else {
        cyrf6936WriteRegister(CYRF6936_XACT_CFG, mode);
    }
}

void cyrf6936SetCrcSeed(const uint16_t crc)
{
    cyrf6936WriteRegister(CYRF6936_CRC_SEED_LSB, crc & 0xff);
    cyrf6936WriteRegister(CYRF6936_CRC_SEED_MSB, crc >> 8);
}

void cyrf6936SetSopCode(const uint8_t *sopcode)
{
    cyrf6936WriteBlock(CYRF6936_SOP_CODE, sopcode, 8);
}

void cyrf6936SetDataCode(const uint8_t *datacode)
{
    cyrf6936WriteBlock(CYRF6936_DATA_CODE, datacode, 16);
}

void cyrf6936SendLen(const uint8_t *data, const uint8_t length)
{
    cyrf6936WriteRegister(CYRF6936_TX_LENGTH, length);
    cyrf6936WriteRegister(CYRF6936_TX_CTRL, CYRF6936_TX_CLR);
    cyrf6936WriteBlock(CYRF6936_TX_BUFFER, data, length);
    cyrf6936WriteRegister(CYRF6936_TX_CTRL, CYRF6936_TX_GO);
}

void cyrf6936StartRecv(void)
{
    cyrf6936WriteRegister(CYRF6936_RX_IRQ_STATUS, CYRF6936_RXOW_IRQ);
    cyrf6936WriteRegister(CYRF6936_RX_CTRL, CYRF6936_RX_GO | CYRF6936_RXC_IRQEN | CYRF6936_RXE_IRQEN);
    EXTIEnable(rxIntIO, true);
}

void cyrf6936RecvLen(uint8_t *data, const uint8_t length)
{
    cyrf6936ReadBlock(CYRF6936_RX_BUFFER, data, length);
}

#endif /* USE_RX_SPEKTRUM */
