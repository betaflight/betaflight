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

// This file is copied with modifications from OpenSky,
// see https://github.com/fishpepper/OpenSky

// See also in particular
// https://github.com/fishpepper/OpenSky/blob/master/arch/stm32f1/hal_cc25xx.c
// https://github.com/fishpepper/OpenSky/blob/master/cc25xx.h
// https://github.com/fishpepper/OpenSky/blob/master/cc25xx.c

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#ifdef USE_RX_CC2500

#include "build/build_config.h"

#include "system.h"
#include "bus_spi.h"
#include "io.h"
#include "io_impl.h"
#include "rx_spi.h"
#include "rx_cc2500.h"

static uint8_t cc25xx_current_antenna;

#define CC25XX_RX_HI()       {IOHi(DEFIO_IO(RX_SW_CRX_PIN));}
#define CC25XX_RX_LO()       {IOLo(DEFIO_IO(RX_SW_CRX_PIN));}
#define CC25XX_TX_HI()       {IOHi(DEFIO_IO(RX_SW_CTX_PIN));}
#define CC25XX_TX_LO()       {IOLo(DEFIO_IO(RX_SW_CTX_PIN));}

static void cc25xx_init_gpio(void)
{
    const SPIDevice rxSPIDevice = spiDeviceByInstance(RX_SPI_INSTANCE);
    IOInit(DEFIO_IO(RX_SW_CRX_PIN), OWNER_RX_SPI_CS, rxSPIDevice + 1);
    IOInit(DEFIO_IO(RX_SW_CTX_PIN), OWNER_RX_SPI_CS, rxSPIDevice + 1);
    IOInit(DEFIO_IO(RX_CC2500_GDO2_PIN), OWNER_RX_SPI_CS, rxSPIDevice + 1);
    // All pins SPI_IO_CS_CFG except GDO2 which is input, same as SPI_IO_AF_MISO_CFG
    IOConfigGPIO(DEFIO_IO(RX_SW_CRX_PIN), SPI_IO_CS_CFG);
    IOConfigGPIO(DEFIO_IO(RX_SW_CTX_PIN), SPI_IO_CS_CFG);
    IOConfigGPIO(DEFIO_IO(RX_CC2500_GDO2_PIN), SPI_IO_AF_MISO_CFG);
#ifdef RX_ANT_SW_CTX_PIN
    IOInit(DEFIO_IO(RX_ANT_SW_CTX_PIN), OWNER_RX_SPI_CS, rxSPIDevice + 1);
    IOConfigGPIO(DEFIO_IO(RX_ANT_SW_CTX_PIN), SPI_IO_CS_CFG);
#endif
#ifdef RX_ANT_SW_CRX_PIN
    IOInit(DEFIO_IO(RX_ANT_SW_CRX_PIN), OWNER_RX_SPI_CS, rxSPIDevice + 1);
    IOConfigGPIO(DEFIO_IO(RX_ANT_SW_CRX_PIN), SPI_IO_CS_CFG);
#endif
}

void cc25xx_init(void)
{
    cc25xx_init_gpio();
    cc25xx_set_antenna(0);
    cc25xx_enter_rxmode();
}

uint32_t cc25xx_set_antenna(uint8_t id)
{
#if 0
    if (id) {
        CC25XX_ANT_SW_CTX_GPIO->BRR  = (CC25XX_ANT_SW_CTX_PIN); //0
        CC25XX_ANT_SW_CRX_GPIO->BSRR = (CC25XX_ANT_SW_CRX_PIN); //1
    } else {
        CC25XX_ANT_SW_CTX_GPIO->BSRR = (CC25XX_ANT_SW_CTX_PIN); //1
        CC25XX_ANT_SW_CRX_GPIO->BRR  = (CC25XX_ANT_SW_CRX_PIN); //0
    }
#endif
    cc25xx_current_antenna = id;
    return id;
}

void cc25xx_set_gdo_mode(void)
{
    cc25xx_set_register(IOCFG0, 0x01); //6);
    //cc25xx_set_register(IOCFG1, ???);
    cc25xx_set_register(IOCFG2, 0x01); //6);
}

void cc25xx_set_register(uint8_t address, uint8_t data)
{
    rxSpiWriteCommand(address, data);
}

uint8_t cc25xx_get_register(uint8_t address)
{
    return rxSpiReadCommand(address | 0x80, 0xFF);
}

void cc25xx_strobe(uint8_t address)
{
    rxSpiWriteByte(address);
}

uint8_t cc25xx_get_status(void)
{
    return rxSpiWriteByte(0xFF);
}

uint8_t cc25xx_transmission_completed(void)
{
    //after tx cc25xx goes back to RX (configured by mcsm1 register)
    return ((cc25xx_get_status() & (0x70)) == CC2500_STATUS_STATE_RX);
}

void cc25xx_enter_rxmode(void)
{
    CC25XX_RX_HI();
    delayMicroseconds(20);
    CC25XX_TX_LO();
    delayMicroseconds(5);
}

void cc25xx_enter_txmode(void)
{
    CC25XX_RX_LO();
    delayMicroseconds(20);
    CC25XX_TX_HI();
    delayMicroseconds(5);
}

void cc25xx_enable_receive(void)
{
    //switch on rx again
    cc25xx_enter_rxmode();
}

uint8_t cc25xx_get_gdo_status(void)
{
    return IORead(DEFIO_IO(RX_CC2500_GDO2_PIN));
}

void cc25xx_register_read_multi(uint8_t address, uint8_t *data, uint8_t length)
{
    rxSpiReadCommandMulti(address, 0xFF, data, length);
}

void cc25xx_read_fifo(uint8_t *data, uint8_t length)
{
    cc25xx_register_read_multi(CC25XX_FIFO | READ_FLAG | BURST_FLAG, data, length);
}

void cc25xx_register_write_multi(uint8_t address, uint8_t *data, uint8_t length)
{
    rxSpiWriteCommandMulti(address | BURST_FLAG, data, length);
}

#define cc25xx_get_register_burst(x)  cc25xx_get_register(x | READ_FLAG | BURST_FLAG)

void cc25xx_process_packet(volatile uint8_t *packet_received, volatile uint8_t *buffer, uint8_t maxlen)
{
    if(cc25xx_get_gdo_status() == 1){
        //data received, fetch data
        //timeout_set_100us(5);

        *packet_received = 0;

        //there is a bug in the cc2500
        //see p3 http://www.ti.com/lit/er/swrz002e/swrz002e.pdf
        //workaround: read len register very quickly twice:
        uint8_t len1, len2, len, i;

        //try this 10 times befor giving up:
        for (i=0; i<10; i++){
            len1 = cc25xx_get_register_burst(RXBYTES) & 0x7F;
            len2 = cc25xx_get_register_burst(RXBYTES) & 0x7F;
            if (len1==len2) break;
        }

        //valid len found?
        if (len1==len2){
            len = len1;

            //packet received, grab data
            uint8_t tmp_buffer[128]; // fifo < 128 bytes
            cc25xx_read_fifo(tmp_buffer, len);

            //only accept valid packet lengths:
            if (len == maxlen){
                uint8_t i;
                for(i=0; i<maxlen; i++){
                    buffer[i] = tmp_buffer[i];
                }
                *packet_received = 1;
            }
        }else{
            //no, ignore this
            len = 0;
        }
    }
}

void cc25xx_transmit_packet(volatile uint8_t *buffer, uint8_t length)
{
    UNUSED(length);
    //flush tx fifo
    cc25xx_strobe(RFST_SFTX);
    //copy to fifo
    cc25xx_register_write_multi(CC25XX_FIFO, (uint8_t *)buffer, buffer[0]+1);
    //and send!
    cc25xx_strobe(RFST_STX);
}

void cc25xx_switch_antenna(void)
{
    // switch to next antenna
    if (cc25xx_current_antenna) {
        cc25xx_current_antenna = cc25xx_set_antenna(0);
    } else {
        cc25xx_current_antenna = cc25xx_set_antenna(1);
    }
}

void cc25xx_wait_for_transmission_complete(void)
{
#if 0
    //after STX we go back to RX state (see MCSM1 register)
    //so wait a maximum of 9ms for completion
    timeout2_set_100us(90);

    while (!timeout2_timed_out()) {
        if (cc25xx_transmission_completed()) {
            //done with tx, return
            return;
        }
    }

    //if we reach this point, tx timed out:
    debug("!TX");
#endif
}
#endif // USE_RX_CC2500
