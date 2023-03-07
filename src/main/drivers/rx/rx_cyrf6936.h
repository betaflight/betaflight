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

#pragma once

enum {
    CYRF6936_CHANNEL         = 0x00,
    CYRF6936_TX_LENGTH       = 0x01,
    CYRF6936_TX_CTRL         = 0x02,
    CYRF6936_TX_CFG          = 0x03,
    CYRF6936_TX_IRQ_STATUS   = 0x04,
    CYRF6936_RX_CTRL         = 0x05,
    CYRF6936_RX_CFG          = 0x06,
    CYRF6936_RX_IRQ_STATUS   = 0x07,
    CYRF6936_RX_STATUS       = 0x08,
    CYRF6936_RX_COUNT        = 0x09,
    CYRF6936_RX_LENGTH       = 0x0A,
    CYRF6936_PWR_CTRL        = 0x0B,
    CYRF6936_XTAL_CTRL       = 0x0C,
    CYRF6936_IO_CFG          = 0x0D,
    CYRF6936_GPIO_CTRL       = 0x0E,
    CYRF6936_XACT_CFG        = 0x0F,
    CYRF6936_FRAMING_CFG     = 0x10,
    CYRF6936_DATA32_THOLD    = 0x11,
    CYRF6936_DATA64_THOLD    = 0x12,
    CYRF6936_RSSI            = 0x13,
    CYRF6936_EOP_CTRL        = 0x14,
    CYRF6936_CRC_SEED_LSB    = 0x15,
    CYRF6936_CRC_SEED_MSB    = 0x16,
    CYRF6936_TX_CRC_LSB      = 0x17,
    CYRF6936_TX_CRC_MSB      = 0x18,
    CYRF6936_RX_CRC_LSB      = 0x19,
    CYRF6936_RX_CRC_MSB      = 0x1A,
    CYRF6936_TX_OFFSET_LSB   = 0x1B,
    CYRF6936_TX_OFFSET_MSB   = 0x1C,
    CYRF6936_MODE_OVERRIDE   = 0x1D,
    CYRF6936_RX_OVERRIDE     = 0x1E,
    CYRF6936_TX_OVERRIDE     = 0x1F,
    CYRF6936_TX_BUFFER       = 0x20,
    CYRF6936_RX_BUFFER       = 0x21,
    CYRF6936_SOP_CODE        = 0x22,
    CYRF6936_DATA_CODE       = 0x23,
    CYRF6936_PREAMBLE        = 0x24,
    CYRF6936_MFG_ID          = 0x25,
    CYRF6936_XTAL_CFG        = 0x26,
    CYRF6936_CLK_OFFSET      = 0x27,
    CYRF6936_CLK_EN          = 0x28,
    CYRF6936_RX_ABORT        = 0x29,
    CYRF6936_AUTO_CAL_TIME   = 0x32,
    CYRF6936_AUTO_CAL_OFFSET = 0x35,
    CYRF6936_ANALOG_CTRL     = 0x39,
};
// ENABLE WRITING
#define CYRF6936_DIR (1<<7)

// CYRF6936_MODE_OVERRIDE
#define CYRF6936_RST (1<<0)

// CYRF6936_CLK_EN
#define CYRF6936_RXF (1<<1)

// CYRF6936_XACT_CFG
enum {
    CYRF6936_MODE_SLEEP    = (0x0 << 2),
    CYRF6936_MODE_IDLE     = (0x1 << 2),
    CYRF6936_MODE_SYNTH_TX = (0x2 << 2),
    CYRF6936_MODE_SYNTH_RX = (0x3 << 2),
    CYRF6936_MODE_RX       = (0x4 << 2),
};
#define CYRF6936_FRC_END (1<<5)
#define CYRF6936_ACK_EN  (1<<7)

// CYRF6936_IO_CFG
#define CYRF6936_IRQ_GPIO   (1<<0)
#define CYRF6936_SPI_3PIN   (1<<1)
#define CYRF6936_PACTL_GPIO (1<<2)
#define CYRF6936_PACTL_OD   (1<<3)
#define CYRF6936_XOUT_OD    (1<<4)
#define CYRF6936_SDI_OD    (1<<5)
#define CYRF6936_IRQ_POL    (1<<6)
#define CYRF6936_IRQ_OD     (1<<7)

// CYRF6936_FRAMING_CFG
#define CYRF6936_LEN_EN  (1<<5)
#define CYRF6936_SOP_LEN (1<<6)
#define CYRF6936_SOP_EN  (1<<7)

// CYRF6936_RX_STATUS
enum {
    CYRF6936_RX_DATA_MODE_GFSK = 0x00,
    CYRF6936_RX_DATA_MODE_8DR  = 0x01,
    CYRF6936_RX_DATA_MODE_DDR  = 0x10,
    CYRF6936_RX_DATA_MODE_NV   = 0x11,
};
#define CYRF6936_RX_CODE (1<<2)
#define CYRF6936_BAD_CRC (1<<3)
#define CYRF6936_CRC0    (1<<4)
#define CYRF6936_EOP_ERR (1<<5)
#define CYRF6936_PKT_ERR (1<<6)
#define CYRF6936_RX_ACK  (1<<7)

// CYRF6936_TX_IRQ_STATUS
#define CYRF6936_TXE_IRQ    (1<<0)
#define CYRF6936_TXC_IRQ    (1<<1)
#define CYRF6936_TXBERR_IRQ (1<<2)
#define CYRF6936_TXB0_IRQ   (1<<3)
#define CYRF6936_TXB8_IRQ   (1<<4)
#define CYRF6936_TXB15_IRQ  (1<<5)
#define CYRF6936_LV_IRQ     (1<<6)
#define CYRF6936_OS_IRQ     (1<<7)

// CYRF6936_RX_IRQ_STATUS
#define CYRF6936_RXE_IRQ    (1<<0)
#define CYRF6936_RXC_IRQ    (1<<1)
#define CYRF6936_RXBERR_IRQ (1<<2)
#define CYRF6936_RXB1_IRQ   (1<<3)
#define CYRF6936_RXB8_IRQ   (1<<4)
#define CYRF6936_RXB16_IRQ  (1<<5)
#define CYRF6936_SOPDET_IRQ (1<<6)
#define CYRF6936_RXOW_IRQ   (1<<7)

// CYRF6936_TX_CTRL
#define CYRF6936_TXE_IRQEN    (1<<0)
#define CYRF6936_TXC_IRQEN    (1<<1)
#define CYRF6936_TXBERR_IRQEN (1<<2)
#define CYRF6936_TXB0_IRQEN   (1<<3)
#define CYRF6936_TXB8_IRQEN   (1<<4)
#define CYRF6936_TXB15_IRQEN  (1<<5)
#define CYRF6936_TX_CLR       (1<<6)
#define CYRF6936_TX_GO        (1<<7)

// CYRF6936_RX_CTRL
#define CYRF6936_RXE_IRQEN    (1<<0)
#define CYRF6936_RXC_IRQEN    (1<<1)
#define CYRF6936_RXBERR_IRQEN (1<<2)
#define CYRF6936_RXB1_IRQEN   (1<<3)
#define CYRF6936_RXB8_IRQEN   (1<<4)
#define CYRF6936_RXB16_IRQEN  (1<<5)
#define CYRF6936_RSVD         (1<<6)
#define CYRF6936_RX_GO        (1<<7)

// CYRF6936_RX_OVERRIDE
#define CYRF6936_ACE       (1<<1)
#define CYRF6936_DIS_RXCRC (1<<2)
#define CYRF6936_DIS_CRC0  (1<<3)
#define CYRF6936_FRC_RXDR  (1<<4)
#define CYRF6936_MAN_RXACK (1<<5)
#define CYRF6936_RXTX_DLY  (1<<6)
#define CYRF6936_ACK_RX    (1<<7)

// CYRF6936_TX_OVERRIDE
#define CYRF6936_TX_INV    (1<<0)
#define CYRF6936_DIS_TXCRC (1<<2)
#define CYRF6936_OVRD_ACK  (1<<3)
#define CYRF6936_MAN_TXACK (1<<4)
#define CYRF6936_FRC_PRE   (1<<6)
#define CYRF6936_ACK_TX    (1<<7)

// CYRF6936_RX_CFG
#define CYRF6936_VLD_EN       (1<<0)
#define CYRF6936_RXOW_EN      (1<<1)
#define CYRF6936_FAST_TURN_EN (1<<3)
#define CYRF6936_HILO         (1<<4)
#define CYRF6936_ATT          (1<<5)
#define CYRF6936_LNA          (1<<6)
#define CYRF6936_AGC_EN       (1<<7)

// CYRF6936_TX_CFG
enum {
    CYRF6936_PA_M35 = 0x0,
    CYRF6936_PA_M30 = 0x1,
    CYRF6936_PA_M24 = 0x2,
    CYRF6936_PA_M18 = 0x3,
    CYRF6936_PA_M13 = 0x4,
    CYRF6936_PA_M5  = 0x5,
    CYRF6936_PA_0   = 0x6,
    CYRF6936_PA_4   = 0x7,
};
enum {
    CYRF6936_DATA_MODE_GFSK = (0x0 << 3),
    CYRF6936_DATA_MODE_8DR  = (0x1 << 3),
    CYRF6936_DATA_MODE_DDR  = (0x2 << 3),
    CYRF6936_DATA_MODE_SDR  = (0x3 << 3),
};
#define CYRF6936_DATA_CODE_LENGTH (1<<5)

extern bool isError;

bool cyrf6936Init(void);

bool cyrf6936RxFinished(uint32_t *timeStamp);

void cyrf6936WriteRegister(const uint8_t address, const uint8_t data);
void cyrf6936WriteBlock(const uint8_t address, const uint8_t *data, const uint8_t length);
uint8_t cyrf6936ReadRegister(const uint8_t address);
void cyrf6936ReadBlock(const uint8_t address, uint8_t *data, const uint8_t length);

uint8_t cyrf6936GetRssi(void);
uint8_t cyrf6936GetRxStatus(void);
void cyrf6936SetConfigLen(const uint8_t config[][2], const uint8_t length);
void cyrf6936SetChannel(const uint8_t chan);
void cyrf6936SetMode(const uint8_t mode, const bool force);
void cyrf6936SetCrcSeed(const uint16_t crc);
void cyrf6936SetSopCode(const uint8_t *sopcode);
void cyrf6936SetDataCode(const uint8_t *datacode);

void cyrf6936SendLen(const uint8_t *data, const uint8_t length);
void cyrf6936StartRecv(void);
void cyrf6936RecvLen(uint8_t *data, const uint8_t length);
