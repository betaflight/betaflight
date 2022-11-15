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

#ifdef USE_RX_FLYSKY

#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rx/rx_a7105.h"
#include "drivers/rx/rx_spi.h"

static IO_t txEnIO = IO_NONE;

static bool consumeExti = true;

void A7105Init(uint32_t id, IO_t txEnPin)
{
    if (txEnPin) {
        txEnIO = txEnPin;
        //TODO: Create resource for this if it ever gets used
        IOInit(txEnIO, OWNER_RX_SPI_CC2500_TX_EN, 0);
        IOConfigGPIO(txEnIO, IOCFG_OUT_PP);
    } else {
        txEnIO = IO_NONE;
    }

    A7105SoftReset();
    A7105WriteID(id);
}

void A7105Config(const uint8_t *regsTable, uint8_t size)
{
    if (regsTable) {
        unsigned timeout = 1000;

        for (unsigned i = 0; i < size; i++) {
            if (regsTable[i] != 0xFF) {
                A7105WriteReg ((A7105Reg_t)i, regsTable[i]);
            }
        }

        A7105Strobe(A7105_STANDBY);

        A7105WriteReg(A7105_02_CALC, 0x01);

        while ((A7105ReadReg(A7105_02_CALC) != 0) && timeout--) {}

        A7105ReadReg(A7105_22_IF_CALIB_I);

        A7105WriteReg(A7105_24_VCO_CURCAL, 0x13);
        A7105WriteReg(A7105_25_VCO_SBCAL_I, 0x09);
        A7105Strobe(A7105_STANDBY);
    }
}

bool A7105RxTxFinished(timeUs_t *timeStamp)
{
    bool result = false;

    if (consumeExti && rxSpiPollExti()) {
        if (rxSpiGetLastExtiTimeUs()) {
            *timeStamp = rxSpiGetLastExtiTimeUs();
        }

        rxSpiResetExti();

        result = true;
    }
    return result;
}

void A7105SoftReset(void)
{
    rxSpiWriteCommand((uint8_t)A7105_00_MODE, 0x00);
}

uint8_t A7105ReadReg(A7105Reg_t reg)
{
    return rxSpiReadCommand((uint8_t)reg | 0x40, 0xFF);
}

void A7105WriteReg(A7105Reg_t reg, uint8_t data)
{
    rxSpiWriteCommand((uint8_t)reg, data);
}

void A7105Strobe(A7105State_t state)
{
    if (A7105_TX == state || A7105_RX == state) {
        consumeExti = true;
        rxSpiResetExti();
    } else {
        consumeExti = false;
    }

    if (txEnIO) {
        if (A7105_TX == state) {
            IOHi(txEnIO); /* enable PA */
        } else {
            IOLo(txEnIO); /* disable PA */
        }
    }

    rxSpiWriteByte((uint8_t)state);
}

void A7105WriteID(uint32_t id)
{
    uint8_t data[4];
    data[0] = (id >> 24) & 0xFF;
    data[1] = (id >> 16) & 0xFF;
    data[2] = (id >> 8) & 0xFF;
    data[3] = (id >> 0) & 0xFF;
    rxSpiWriteCommandMulti((uint8_t)A7105_06_ID_DATA, &data[0], sizeof(data));
}

uint32_t A7105ReadID(void)
{
    uint32_t id;
    uint8_t data[4];
    rxSpiReadCommandMulti((uint8_t)A7105_06_ID_DATA | 0x40, 0xFF, &data[0], sizeof(data));
    id = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3] << 0;
    return id;
}

void A7105ReadFIFO(uint8_t *data, uint8_t num)
{
    if (data) {
        if(num > 64) {
            num = 64;
        }

        A7105Strobe(A7105_RST_RDPTR); /* reset read pointer */
        rxSpiReadCommandMulti((uint8_t)A7105_05_FIFO_DATA | 0x40, 0xFF, data, num);
    }
}

void A7105WriteFIFO(uint8_t *data, uint8_t num)
{
    if (data) {
        if(num > 64) {
            num = 64;
        }

        A7105Strobe(A7105_RST_WRPTR); /* reset write pointer */
        rxSpiWriteCommandMulti((uint8_t)A7105_05_FIFO_DATA, data, num);
    }
}

#endif /* USE_RX_FLYSKY */
