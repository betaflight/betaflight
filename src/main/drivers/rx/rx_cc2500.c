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

/*
* CC2500 SPI drivers
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_RX_CC2500

#include "build/build_config.h"

#include "drivers/io.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "rx_cc2500.h"

#define NOP 0xFF

uint8_t cc2500ReadFifo(uint8_t *dpbuffer, uint8_t len)
{
    return rxSpiReadCommandMulti(CC2500_3F_RXFIFO | CC2500_READ_BURST, NOP, dpbuffer, len);
}

uint8_t cc2500WriteFifo(uint8_t *dpbuffer, uint8_t len)
{
    uint8_t ret;
    cc2500Strobe(CC2500_SFTX); // 0x3B SFTX
    ret = rxSpiWriteCommandMulti(CC2500_3F_TXFIFO | CC2500_WRITE_BURST,
                                 dpbuffer, len);
    cc2500Strobe(CC2500_STX); // 0x35
    return ret;
}

uint8_t cc2500ReadRegisterMulti(uint8_t address, uint8_t *data, uint8_t length)
{
    return rxSpiReadCommandMulti(address, NOP, data, length);
}

uint8_t cc2500WriteRegisterMulti(uint8_t address, uint8_t *data,
                                  uint8_t length)
{
    return rxSpiWriteCommandMulti(address, data, length);
}

uint8_t cc2500ReadReg(uint8_t reg)
{
    return rxSpiReadCommand(reg | 0x80, NOP);
}

void cc2500Strobe(uint8_t address) { rxSpiWriteByte(address); }

uint8_t cc2500WriteReg(uint8_t address, uint8_t data)
{
    return rxSpiWriteCommand(address, data);
}

void cc2500SetPower(uint8_t power)
{
    const uint8_t patable[8] = {
        0xC5, // -12dbm
        0x97, // -10dbm
        0x6E, // -8dbm
        0x7F, // -6dbm
        0xA9, // -4dbm
        0xBB, // -2dbm
        0xFE, // 0dbm
        0xFF  // 1.5dbm
    };
    if (power > 7)
        power = 7;
    cc2500WriteReg(CC2500_3E_PATABLE, patable[power]);
}

uint8_t cc2500Reset(void)
{
    cc2500Strobe(CC2500_SRES);
    delayMicroseconds(1000); // 1000us
    // CC2500_SetTxRxMode(TXRX_OFF);
    // RX_EN_off;//off tx
    // TX_EN_off;//off rx
    return cc2500ReadReg(CC2500_0E_FREQ1) == 0xC4; // check if reset
}
#endif
