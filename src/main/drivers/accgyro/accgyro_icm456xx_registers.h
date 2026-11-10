/*
 * This file is part of Betaflight.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ICM45686_WHO_AM_I_CONST                 (0xE9)
#define ICM456XX_WHO_AM_REGISTER                0x72
#define ICM456XX_REG_MISC2                      0x7F
#define ICM456XX_INT1_CONFIG0                   0x16
#define ICM456XX_INT1_STATUS0                   0x19
#define ICM456XX_GYRO_CONFIG0                   0x1C
#define ICM456XX_ACCEL_CONFIG0                  0x1B
#define ICM456XX_PWR_MGMT0                      0x10
#define ICM456XX_REG_IREG_ADDR_15_8             0x7C
#define ICM456XX_REG_IREG_DATA                  0x7E
#define ICM456XX_RA_SREG_CTRL                   0xA267 // To access: 0xA200 + 0x67
#define ICM456XX_SREG_DATA_ENDIAN_SEL_BIG       (1 << 1) // Big endian (requires IREG write)
#define ICM456XX_GYRO_MODE_LN                   (0x03 << 2)  // Low Noise Mode
#define ICM456XX_ACCEL_MODE_LN                  (0x03) // Low Noise Mode
#define ICM456XX_INT1_STATUS_EN_DRDY            (1 << 2)
#define ICM456XX_INT1_STATUS_EN_FIFO_FULL       (1 << 0)
#define ICM456XX_INT1_STATUS_RESET_DONE         (1 << 7)
#define ICM456XX_INT1_STATUS_DRDY               (1 << 2)
#define ICM456XX_INT1_STATUS_FIFO_FULL          (1 << 0)
#define ICM456XX_SOFT_RESET                     (1 << 1)
#define ICM456XX_ACCEL_DATA_X1_UI               0x00
#define ICM456XX_ACCEL_FS_SEL_32G               (0x00 << 4)
#define ICM456XX_ACCEL_ODR_6K4_LN               0x03
#define ICM456XX_GYRO_FS_SEL_4000DPS            (0x00 << 4)
#define ICM456XX_GYRO_ODR_6K4_LN                0x03

static inline uint16_t icm456xxReadU16(const uint8_t *bytes, bool bigEndian)
{
    return bigEndian ? ((uint16_t)bytes[0] << 8) | bytes[1] : ((uint16_t)bytes[1] << 8) | bytes[0];
}

static inline int16_t icm456xxReadI16(const uint8_t *bytes, bool bigEndian)
{
    const uint16_t value = icm456xxReadU16(bytes, bigEndian);
    return value <= INT16_MAX ? (int16_t)value : (int16_t)((int32_t)value - 65536);
}
