/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/bus.h"

// Analog Devices ADIS16575 / ADIS16576 / ADIS16577 precision MEMS IMU.

// f_SCLK: 15MHz for register access, 8MHz for stall-free burst reads.
#ifndef ADIS1657X_MAX_SPI_CLK_HZ
#define ADIS1657X_MAX_SPI_CLK_HZ    8000000
#endif

/*
 * Registers are 16 bits but byte-addressed: the low byte lives at reg and the
 * high byte at reg + 1. Bit 7 of the address byte set means write, so a write is
 * two frames and there is no read bit.
 *
 * Reads are pipelined: the frame carrying the address returns the *previous*
 * frame's answer, so a read is also two frames.
 */
#define ADIS1657X_WRITE_BIT         0x80

// t_STALL between frames, 5us minimum.
#define ADIS1657X_STALL_US          5

#define ADIS1657X_REG_FIFO_CNT              0x3C
#define ADIS1657X_REG_FIFO_CTRL             0x5A
#define ADIS1657X_REG_FILT_CTRL             0x5C
#define ADIS1657X_REG_RNG_MDL               0x5E
#define ADIS1657X_REG_MSC_CTRL              0x60
#define ADIS1657X_REG_DEC_RATE              0x64
#define ADIS1657X_REG_GLOB_CMD              0x68
#define ADIS1657X_REG_PROD_ID               0x72

// PROD_ID (0x72) is a real identity register, so the variant is detected at run time.
#define ADIS1657X_PROD_ID_16575             0x40BF
#define ADIS1657X_PROD_ID_16576             0x40C0
#define ADIS1657X_PROD_ID_16577             0x40C1

// RNG_MDL (0x5E) bits [3:2] report the fitted gyro range; [1:0] read 11 and
// [15:4] are unused, so mask before comparing.
#define ADIS1657X_RNG_MDL_RANGE_MASK        0x000F
#define ADIS1657X_RNG_MDL_LOW_RANGE         0x0007
#define ADIS1657X_RNG_MDL_HIGH_RANGE        0x000F

#define ADIS1657X_GLOB_CMD_SW_RESET         (1 << 7)

#define ADIS1657X_MSC_CTRL_BURST_32         (1 << 9)
#define ADIS1657X_MSC_CTRL_OUT_SEL          (1 << 8)
#define ADIS1657X_MSC_CTRL_GSEN_EN          (1 << 7)
#define ADIS1657X_MSC_CTRL_POP_EN           (1 << 6)
#define ADIS1657X_MSC_CTRL_DR_POL           (1 << 0)

/*
 * OUT_SEL clear selects rate and acceleration rather than delta angle and delta
 * velocity. BURST_32 keeps 32-bit outputs; readFn uses the high word of each pair.
 * POP_EN translates the accelerometer to the gyroscope origin. GSEN_EN only
 * affects the delta-angle path, so it stays clear. DR_POL set makes data ready
 * active high, matching the rising edge mpuIntExtiInit() arms.
 */
#define ADIS1657X_MSC_CTRL_CFG                              \
    (ADIS1657X_MSC_CTRL_BURST_32                            \
     | ADIS1657X_MSC_CTRL_POP_EN                            \
     | ADIS1657X_MSC_CTRL_DR_POL)

// FIFO_CTRL (0x5A). Zero disables the FIFO; the burst then reads live registers,
// so one data-ready pulse yields exactly one fresh sample.
#define ADIS1657X_FIFO_CTRL_DISABLED        0x0000

// FILT_CTRL (0x5C): Bartlett-window FIR of 2^n taps for n in bits [2:0], max 6.
// Zero is no filtering.
#define ADIS1657X_FILT_CTRL_OFF             0
#define ADIS1657X_FILT_CTRL_4_TAP           2
#define ADIS1657X_FILT_CTRL_16_TAP          4
#define ADIS1657X_FILT_CTRL_64_TAP          6

// DEC_RATE (0x64): f_ODR = 2000 / (DEC_RATE + 1). Zero for the full rate.
#define ADIS1657X_DEC_RATE_2000HZ           0

/*
 * Burst (DIN 0x6800): eighteen 16-bit big-endian words, no stall between frames.
 * FIFO_CNT, DIAG_STAT, a low/high pair per axis for gyro XYZ then accel XYZ,
 * temperature, sample counter, timestamp upper (zero unless TS_32 is set), and
 * the checksum. The payload is even-aligned, so no rxBuf[1] shim is needed.
 */
#define ADIS1657X_BURST_CMD                 ADIS1657X_REG_GLOB_CMD

#define ADIS1657X_BURST_WORD_FIFO_CNT       0
#define ADIS1657X_BURST_WORD_DIAG_STAT      1
#define ADIS1657X_BURST_WORD_GYRO_LOW       2
#define ADIS1657X_BURST_WORD_ACCEL_LOW      8
#define ADIS1657X_BURST_WORD_TEMPERATURE    14
#define ADIS1657X_BURST_WORD_DATA_CNTR      15
#define ADIS1657X_BURST_WORD_TIMESTAMP      16
#define ADIS1657X_BURST_WORD_CHECKSUM       17
#define ADIS1657X_BURST_WORD_COUNT          18
#define ADIS1657X_BURST_FRAME_LEN           (ADIS1657X_BURST_WORD_COUNT * sizeof(uint16_t))

// The high word of each 32-bit axis, which is the 16-bit output on its own.
#define ADIS1657X_BURST_GYRO_HIGH(axis)     (ADIS1657X_BURST_WORD_GYRO_LOW + 1 + 2 * (axis))
#define ADIS1657X_BURST_ACCEL_HIGH(axis)    (ADIS1657X_BURST_WORD_ACCEL_LOW + 1 + 2 * (axis))

// 16-bit byte-wise sum over the gyro word through the timestamp; FIFO_CNT and
// DIAG_STAT are excluded.
#define ADIS1657X_BURST_CHECKSUM_FIRST_WORD ADIS1657X_BURST_WORD_GYRO_LOW
#define ADIS1657X_BURST_CHECKSUM_END_WORD   ADIS1657X_BURST_WORD_CHECKSUM

// 16-bit gyro sensitivity, LSB per deg/s (datasheet table 20): the -2 parts are
// +-450 deg/s at 40, the -3 parts +-2000 deg/s at 10.
#ifndef ADIS1657X_GYRO_LSB_PER_DPS_LOW
#define ADIS1657X_GYRO_LSB_PER_DPS_LOW      40.0f
#endif
#ifndef ADIS1657X_GYRO_LSB_PER_DPS_HIGH
#define ADIS1657X_GYRO_LSB_PER_DPS_HIGH     10.0f
#endif

#define ADIS1657X_GYRO_SCALE_LOW            (1.0f / ADIS1657X_GYRO_LSB_PER_DPS_LOW)
#define ADIS1657X_GYRO_SCALE_HIGH           (1.0f / ADIS1657X_GYRO_LSB_PER_DPS_HIGH)

// 16-bit accel sensitivity, LSB per g (datasheet table 35): +-8g at 4000 for the
// ADIS16575, +-40g at 800 for the ADIS16576 and ADIS16577.
#define ADIS1657X_ACC_1G_16575              4000
#define ADIS1657X_ACC_1G_16576              800
#define ADIS1657X_ACC_1G_16577              800

// TEMP_OUT: 0.1 degC/LSB, no offset.
#define ADIS1657X_TEMP_SCALE                0.1f
#define ADIS1657X_TEMP_ZERO                 0.0f

uint8_t adis1657xSpiDetect(const extDevice_t *dev);

bool adis1657xSpiAccDetect(accDev_t *acc);
bool adis1657xSpiGyroDetect(gyroDev_t *gyro);
