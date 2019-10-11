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

/* A7105 states for strobe */
typedef enum {
    A7105_SLEEP = 0x80,
    A7105_IDLE = 0x90,
    A7105_STANDBY = 0xA0,
    A7105_PLL = 0xB0,
    A7105_RX = 0xC0,
    A7105_TX = 0xD0,
    A7105_RST_WRPTR = 0xE0,
    A7105_RST_RDPTR = 0xF0
} A7105State_t;

/* Register addresses */
typedef enum {
    A7105_00_MODE = 0x00,
    A7105_01_MODE_CONTROL = 0x01,
    A7105_02_CALC = 0x02,
    A7105_03_FIFOI = 0x03,
    A7105_04_FIFOII = 0x04,
    A7105_05_FIFO_DATA = 0x05,
    A7105_06_ID_DATA = 0x06,
    A7105_07_RC_OSC_I = 0x07,
    A7105_08_RC_OSC_II = 0x08,
    A7105_09_RC_OSC_III = 0x09,
    A7105_0A_CK0_PIN = 0x0A,
    A7105_0B_GPIO1_PIN_I = 0x0B,
    A7105_0C_GPIO2_PIN_II = 0x0C,
    A7105_0D_CLOCK = 0x0D,
    A7105_0E_DATA_RATE = 0x0E,
    A7105_0F_PLL_I = 0x0F,
    A7105_0F_CHANNEL = 0x0F,
    A7105_10_PLL_II = 0x10,
    A7105_11_PLL_III = 0x11,
    A7105_12_PLL_IV = 0x12,
    A7105_13_PLL_V = 0x13,
    A7105_14_TX_I = 0x14,
    A7105_15_TX_II = 0x15,
    A7105_16_DELAY_I = 0x16,
    A7105_17_DELAY_II = 0x17,
    A7105_18_RX = 0x18,
    A7105_19_RX_GAIN_I = 0x19,
    A7105_1A_RX_GAIN_II = 0x1A,
    A7105_1B_RX_GAIN_III = 0x1B,
    A7105_1C_RX_GAIN_IV = 0x1C,
    A7105_1D_RSSI_THOLD = 0x1D,
    A7105_1E_ADC = 0x1E,
    A7105_1F_CODE_I = 0x1F,
    A7105_20_CODE_II = 0x20,
    A7105_21_CODE_III = 0x21,
    A7105_22_IF_CALIB_I = 0x22,
    A7105_23_IF_CALIB_II = 0x23,
    A7105_24_VCO_CURCAL = 0x24,
    A7105_25_VCO_SBCAL_I = 0x25,
    A7105_26_VCO_SBCAL_II = 0x26,
    A7105_27_BATTERY_DET = 0x27,
    A7105_28_TX_TEST = 0x28,
    A7105_29_RX_DEM_TEST_I = 0x29,
    A7105_2A_RX_DEM_TEST_II = 0x2A,
    A7105_2B_CPC = 0x2B,
    A7105_2C_XTAL_TEST = 0x2C,
    A7105_2D_PLL_TEST = 0x2D,
    A7105_2E_VCO_TEST_I = 0x2E,
    A7105_2F_VCO_TEST_II = 0x2F,
    A7105_30_IFAT = 0x30,
    A7105_31_RSCALE = 0x31,
    A7105_32_FILTER_TEST = 0x32,
} A7105Reg_t;

/* Register: A7105_00_MODE */
#define A7105_MODE_FECF        0x40    // [0]: FEC pass. [1]: FEC error. (FECF is read only, it is updated internally while receiving every packet.)
#define A7105_MODE_CRCF        0x20    // [0]: CRC pass. [1]: CRC error. (CRCF is read only, it is updated internally while receiving every packet.)
#define A7105_MODE_CER         0x10    // [0]: RF chip is disabled. [1]: RF chip is enabled.
#define A7105_MODE_XER         0x08    // [0]: Crystal oscillator is disabled. [1]: Crystal oscillator is enabled.
#define A7105_MODE_PLLER       0x04    // [0]: PLL is disabled. [1]: PLL is enabled.
#define A7105_MODE_TRSR        0x02    // [0]: RX state. [1]: TX state. Serviceable if TRER=1 (TRX is enable).
#define A7105_MODE_TRER        0x01    // [0]: TRX is disabled. [1]: TRX is enabled.

void A7105Init(uint32_t id, IO_t extiPin, IO_t txEnPin);
void A7105SoftReset(void);
void A7105Config(const uint8_t *regsTable, uint8_t size);

uint8_t A7105ReadReg(A7105Reg_t reg);
void A7105WriteReg(A7105Reg_t reg, uint8_t data);
void A7105Strobe(A7105State_t state);

void A7105WriteID(uint32_t id);
uint32_t A7105ReadID(void);

void A7105ReadFIFO(uint8_t *data, uint8_t num);
void A7105WriteFIFO(uint8_t *data, uint8_t num);

bool A7105RxTxFinished(uint32_t *timeStamp);
