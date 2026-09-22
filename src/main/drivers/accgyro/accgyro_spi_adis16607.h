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

// Analog Devices ADIS16607 precision MEMS IMU (gyro + accel).

// t_STALL is 10ns, so no inter-frame delay is needed.
#define ADIS16607_MAX_SPI_CLK_HZ    10000000

// Bit 7 set means read.
#define ADIS16607_READ_BIT          0x80

// Registers are 16 bits wide, big-endian on the wire.
#define ADIS16607_REG_DEV_ID                0x00
#define ADIS16607_REG_REV_ID                0x01
#define ADIS16607_REG_DIAG_STAT             0x05
#define ADIS16607_REG_X_ACCEL_HIGH          0x06
#define ADIS16607_REG_X_GYRO_HIGH           0x0C
#define ADIS16607_REG_TEMPERATURE           0x20
#define ADIS16607_REG_WRITE_LOCK            0x2D
#define ADIS16607_REG_USER_GPIO_CFG1        0x2F
#define ADIS16607_REG_SPI_FULLDUPLEX_KEY    0x31
#define ADIS16607_REG_SPI_HALFDUPLEX_KEY    0x32
#define ADIS16607_REG_USER_DATA_CFG         0x34
#define ADIS16607_REG_USER_FIFO_CFG         0x35
#define ADIS16607_REG_SOFT_RESET            0x36
#define ADIS16607_REG_MSC_CTRL              0x39
#define ADIS16607_REG_DEC_RATE              0x3A
#define ADIS16607_REG_DIGITAL_STATUS        0x4E

// A then B unlocks, B then A locks. The part will not leave its initialisation
// phase until write lock is set, so locking is mandatory.
#define ADIS16607_WRITE_LOCK_KEY_A          0xAAAA
#define ADIS16607_WRITE_LOCK_KEY_B          0x5555
#define ADIS16607_WRITE_LOCK_LOCKED         0x0001

// Pins the part to half duplex, where the response arrives in the same frame.
// Required: burst reads only work in half duplex.
#define ADIS16607_SPI_HALFDUPLEX_KEY_VAL    0xB4B4

// CFG_GPIO_3 is bits [11:9]; 001 is data ready.
#define ADIS16607_GPIO_CFG1_GPIO3_DATA_RDY  (1 << 9)

#define ADIS16607_DATA_CFG_X_ACCEL_EN       (1 << 0)
#define ADIS16607_DATA_CFG_Y_ACCEL_EN       (1 << 1)
#define ADIS16607_DATA_CFG_Z_ACCEL_EN       (1 << 2)
#define ADIS16607_DATA_CFG_X_GYRO_EN        (1 << 3)
#define ADIS16607_DATA_CFG_Y_GYRO_EN        (1 << 4)
#define ADIS16607_DATA_CFG_Z_GYRO_EN        (1 << 5)
#define ADIS16607_DATA_CFG_X_DELTVEL_EN     (1 << 6)
#define ADIS16607_DATA_CFG_Y_DELTVEL_EN     (1 << 7)
#define ADIS16607_DATA_CFG_Z_DELTVEL_EN     (1 << 8)
#define ADIS16607_DATA_CFG_X_DELTANG_EN     (1 << 9)
#define ADIS16607_DATA_CFG_Y_DELTANG_EN     (1 << 10)
#define ADIS16607_DATA_CFG_Z_DELTANG_EN     (1 << 11)
#define ADIS16607_DATA_CFG_TEMPERATURE_EN   (1 << 12)
#define ADIS16607_DATA_CFG_TIME_STAMP_EN    (1 << 13)
#define ADIS16607_DATA_CFG_DATA_CNTR_EN     (1 << 14)
#define ADIS16607_DATA_CFG_WORD_SIZE_16     (0 << 15)
#define ADIS16607_DATA_CFG_WORD_SIZE_32     (1 << 15)

// FIFO_THRESHOLD is bits [10:0]; zero disables the FIFO.
#define ADIS16607_FIFO_CFG_CLEAR_FIFOB      (1 << 15)
#define ADIS16607_FIFO_CFG_ST_GYRO_EN       (1 << 14)
#define ADIS16607_FIFO_CFG_ST_ACCEL_EN      (1 << 13)
#define ADIS16607_FIFO_CFG_DISABLED         0x0000

#define ADIS16607_SOFT_RESET_REG_RESET      (1 << 0)

// FILT_BW is bits [9:8].
#define ADIS16607_MSC_CTRL_FILT_BW_OFF      (0 << 8)
#define ADIS16607_MSC_CTRL_FILT_BW_500HZ    (1 << 8)
#define ADIS16607_MSC_CTRL_FILT_BW_100HZ    (2 << 8)
#define ADIS16607_MSC_CTRL_FILT_BW_20HZ     (3 << 8)

// f_ODR = 9560 / (DEC_RATE + 1).
#define ADIS16607_BASE_ODR_HZ               9560
#define ADIS16607_DEC_RATE_9560HZ           0
#define ADIS16607_DEC_RATE_4780HZ           1
#define ADIS16607_DEC_RATE_3186HZ           2
#define ADIS16607_DEC_RATE_2390HZ           3
#define ADIS16607_DEC_RATE_1195HZ           7

#define ADIS16607_DIGITAL_STATUS_BOOTLOAD_BUSY (1 << 0)

/*
 * Burst: a half-duplex read of DIAG_STAT with CS held low past the command
 * frame clocks out the USER_DATA_CFG-enabled registers in bit order plus a
 * checksum word - here eleven 16-bit words: status, DIAG_STAT, accel XYZ, gyro
 * XYZ, temperature, data counter, checksum. Word 0 is the command frame's first
 * half, so the payload starts at byte 0 of the frame.
 */
#define ADIS16607_BURST_CMD                 (ADIS16607_REG_DIAG_STAT | ADIS16607_READ_BIT)

#define ADIS16607_BURST_WORD_STATUS         0
#define ADIS16607_BURST_WORD_DIAG_STAT      1
#define ADIS16607_BURST_WORD_ACCEL          2
#define ADIS16607_BURST_WORD_GYRO           5
#define ADIS16607_BURST_WORD_TEMPERATURE    8
#define ADIS16607_BURST_WORD_DATA_CNTR      9
#define ADIS16607_BURST_WORD_CHECKSUM       10
#define ADIS16607_BURST_WORD_COUNT          11
#define ADIS16607_BURST_FRAME_LEN           (ADIS16607_BURST_WORD_COUNT * sizeof(uint16_t))

// 16-bit truncated sum of DIAG_STAT up to (excluding) the checksum word.
#define ADIS16607_BURST_CHECKSUM_FIRST_WORD ADIS16607_BURST_WORD_DIAG_STAT

/*
 * Delta velocity/angle and the time stamp are left out: Betaflight integrates
 * rate itself, and every extra word costs EXTI-to-sample bus time. The data
 * counter earns its two bytes - a truncated sum cannot tell a re-read of an
 * unchanged output register from a fresh sample, and the counter can, which is
 * what keeps the polled path from feeding duplicated samples into the PID loop.
 */
#define ADIS16607_USER_DATA_CFG                             \
    (ADIS16607_DATA_CFG_WORD_SIZE_16                        \
     | ADIS16607_DATA_CFG_X_ACCEL_EN                        \
     | ADIS16607_DATA_CFG_Y_ACCEL_EN                        \
     | ADIS16607_DATA_CFG_Z_ACCEL_EN                        \
     | ADIS16607_DATA_CFG_X_GYRO_EN                         \
     | ADIS16607_DATA_CFG_Y_GYRO_EN                         \
     | ADIS16607_DATA_CFG_Z_GYRO_EN                         \
     | ADIS16607_DATA_CFG_TEMPERATURE_EN                    \
     | ADIS16607_DATA_CFG_DATA_CNTR_EN)

/*
 * 16-bit format sensitivities:
 *   gyro -2   62.5   LSB/deg/s  over +/-450 deg/s (+/-480 max)
 *   gyro -3   15.625 LSB/deg/s  over +/-2000 deg/s
 *   accel    781.25  LSB/g      over +/-40 g, both variants
 *
 * DEV_ID reads 0x6000 on both variants and no register reports which part is
 * fitted, so the variant is a build-time choice, and getting it wrong scales
 * the gyro by 4x with nothing to show for it at runtime. There is therefore no
 * default: a board fitting the part states which one it fitted, in its config.h.
 */
#if defined(USE_ACCGYRO_ADIS16607)

#ifndef ADIS16607_VARIANT
#error "ADIS16607_VARIANT must be defined as 2 (ADIS16607-2, +/-450 deg/s) or 3 (ADIS16607-3, +/-2000 deg/s) - the two parts share a DEV_ID and differ 4x in gyro sensitivity"
#endif

#if ADIS16607_VARIANT == 2
#define ADIS16607_GYRO_LSB_PER_DPS          62.5f
#elif ADIS16607_VARIANT == 3
#define ADIS16607_GYRO_LSB_PER_DPS          15.625f
#else
#error "ADIS16607_VARIANT must be 2 or 3 - no other variant exists"
#endif

#define ADIS16607_GYRO_SCALE                (1.0f / ADIS16607_GYRO_LSB_PER_DPS)
#define ADIS16607_ACC_1G                    781

// 0.005 degC/LSB, offset 25 degC.
#define ADIS16607_TEMP_SCALE                0.005f
#define ADIS16607_TEMP_ZERO                 25.0f

#endif // USE_ACCGYRO_ADIS16607

uint8_t adis16607SpiDetect(const extDevice_t *dev);

bool adis16607SpiAccDetect(accDev_t *acc);
bool adis16607SpiGyroDetect(gyroDev_t *gyro);
