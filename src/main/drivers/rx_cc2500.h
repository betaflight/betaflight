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
// see http://deviationtx.com, file iface_nrf24l01.h

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "rx_spi.h"

//adress checks
#define CC2500_PKTCTRL1_FLAG_ADR_CHECK_00 ((0<<1) | (0<<0))
#define CC2500_PKTCTRL1_FLAG_ADR_CHECK_01 ((0<<1) | (1<<0))
#define CC2500_PKTCTRL1_FLAG_ADR_CHECK_10 ((1<<1) | (0<<0))
#define CC2500_PKTCTRL1_FLAG_ADR_CHECK_11 ((1<<1) | (1<<0))
//append status bytes?
#define CC2500_PKTCTRL1_APPEND_STATUS     (1<<2)
//crc autoflush
#define CC2500_PKTCTRL1_CRC_AUTOFLUSH     (1<<3)

// Flags
#define BURST_FLAG   0x40 // 0b0100 0000
#define WRITE_FLAG   0x00 // 0b0000 0000
#define READ_FLAG    0x80 // 0b1000 0000

//Definitions for burst/single access to registers
#define CC2500_WRITE_SINGLE     0x00
#define CC2500_WRITE_BURST      0x40
#define CC2500_READ_SINGLE      0x80
#define CC2500_READ_BURST       0xC0

#define CC2500_STATUS_STATE_IDLE   (0<<4)
#define CC2500_STATUS_STATE_RX     (1<<4)
#define CC2500_STATUS_STATE_TX     (2<<4)
#define CC2500_STATUS_STATE_FSTXON (3<<4)
#define CC2500_STATUS_STATE_CALIBRATE  (4<<4)
#define CC2500_STATUS_STATE_SETTLING   (5<<4)
#define CC2500_STATUS_STATE_RXFIFO_OVF (6<<4)
#define CC2500_STATUS_STATE_TXFIFO_OVF (7<<4)



// strobes
#define RFST_SRES     0x30
#define RFST_SFSTXON  0x31
#define RFST_SXOFF    0x32
#define RFST_SCAL     0x33
#define RFST_SRX      0x34
#define RFST_STX      0x35
#define RFST_SIDLE    0x36
#define RFST_SWOR     0x38
#define RFST_SPWD     0x39
#define RFST_SFRX     0x3A
#define RFST_SFTX     0x3B
#define RFST_SWORRST  0x3C
#define RFST_SNOP     0x3D

// Status registers
#define PARTNUM        (0x30|BURST_FLAG)
#define VERSION        (0x31|BURST_FLAG)
#define FREQEST        (0x32|BURST_FLAG)
#define LQI            (0x33|BURST_FLAG)
#define RSSI           (0x34|BURST_FLAG)
#define MARCSTATE      (0x35|BURST_FLAG)
#define WORTIME1       (0x36|BURST_FLAG)
#define WORTIME0       (0x37|BURST_FLAG)
#define PKTSTATUS      (0x38|BURST_FLAG)
#define VCO_VC_DAC     (0x39|BURST_FLAG)
#define TXBYTES        (0x3A|BURST_FLAG)
#define RXBYTES        (0x3B|BURST_FLAG)
#define RCCTRL1_STATUS (0x3C|BURST_FLAG)
#define RCCTRL0_STATUS (0x3D|BURST_FLAG)

// Status byte states
#define STB_IDLE         0x00
#define STB_RX           0x10
#define STB_TX           0x20
#define STB_FSTXON       0x30
#define STB_CALIBRATE    0x40
#define STB_SETTLING     0x50
#define STB_RX_OVF       0x60
#define STB_TX_UNDF      0x70

// Config registers addresses
#define IOCFG2   0x00
#define IOCFG1   0x01
#define IOCFG0   0x02
#define FIFOTHR  0x03
#define SYNC1    0x04
#define SYNC0    0x05
#define PKTLEN   0x06
#define PKTCTRL1 0x07
#define PKTCTRL0 0x08
#define ADDR     0x09
#define CHANNR   0x0A
#define FSCTRL1  0x0B
#define FSCTRL0  0x0C
#define FREQ2    0x0D
#define FREQ1    0x0E
#define FREQ0    0x0F
#define MDMCFG4  0x10
#define MDMCFG3  0x11
#define MDMCFG2  0x12
#define MDMCFG1  0x13
#define MDMCFG0  0x14
#define DEVIATN  0x15
#define MCSM2    0x16
#define MCSM1    0x17
#define MCSM0    0x18
#define FOCCFG   0x19
#define BSCFG    0x1A
#define AGCCTRL2 0x1B
#define AGCCTRL1 0x1C
#define AGCCTRL0 0x1D
#define WOREVT1  0x1E
#define WOREVT0  0x1F
#define WORCTRL  0x20
#define FREND1   0x21
#define FREND0   0x22
#define FSCAL3   0x23
#define FSCAL2   0x24
#define FSCAL1   0x25
#define FSCAL0   0x26
#define RCCTRL1  0x27
#define RCCTRL0  0x28
#define FSTEST   0x29
#define PTEST    0x2A
#define AGCTEST  0x2B
#define TEST2    0x2C
#define TEST1    0x2D
#define TEST0    0x2E

#define PA_TABLE0  0x3E

// FIFO
#define CC25XX_FIFO     0x3F

void cc25xx_init(void);
void cc25xx_set_register(uint8_t reg, uint8_t val);
uint8_t cc25xx_get_register(uint8_t address);
void cc25xx_strobe(uint8_t val);

void cc25xx_enable_receive(void);
void cc25xx_enable_transmit(void);
void cc25xx_enter_rxmode(void);
void cc25xx_enter_txmode(void);

#define cc25xx_rx_sleep() { delayMicroseconds(1352); }
#define cc25xx_tx_sleep() { delayMicroseconds(1250); }

// not used on d4rii
#define cc25xx_disable_rf_interrupt() {}
#define cc25xx_setup_rf_dma(mode) {}
#define cc25xx_partnum_valid(p, v) ((p == 0x80) && (v = 0x03))

uint8_t cc25xx_get_status(void);
uint32_t cc25xx_set_antenna(uint8_t id);
void cc25xx_set_gdo_mode(void);
uint8_t cc25xx_get_gdo_status(void);
void cc25xx_process_packet(volatile uint8_t *packet_received, volatile uint8_t *buffer, uint8_t maxlen);
void cc25xx_transmit_packet(volatile uint8_t *buffer, uint8_t len);

void cc25xx_read_fifo(uint8_t *buf, uint8_t len);
void cc25xx_register_read_multi(uint8_t address, uint8_t *buffer, uint8_t len);
uint8_t cc25xx_transmission_completed(void);
#define cc25xx_get_register_burst(x)  cc25xx_get_register(x | READ_FLAG | BURST_FLAG)

void cc25xx_switch_antenna(void);
void cc25xx_wait_for_transmission_complete(void);

