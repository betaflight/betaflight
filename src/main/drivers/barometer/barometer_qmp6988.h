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

#pragma once

#define QMP6988_I2C_ADDR                      (0x70)
#define QMP6988_DEFAULT_CHIP_ID               (0x5c)

#define QMP6988_CHIP_ID_REG                   (0xD1)  /* Chip ID Register */
#define QMP6988_IO_SETUP_REG			(0xF5)
#define QMP6988_SET_IIR_REG			(0xF1)
#define QMP6988_CTRL_MEAS_REG		(0xF4)
#define QMP6988_COE_B00_1_REG		(0xA0)
#define QMP6988_PRESSURE_MSB_REG              (0xF7)  /* Pressure MSB Register */
#define QMP6988_PRESSURE_LSB_REG              (0xF8)  /* Pressure LSB Register */
#define QMP6988_PRESSURE_XLSB_REG             (0xF9)  /* Pressure XLSB Register */
#define QMP6988_TEMPERATURE_MSB_REG           (0xFA)  /* Temperature MSB Reg */
#define QMP6988_TEMPERATURE_LSB_REG           (0xFB)  /* Temperature LSB Reg */
#define QMP6988_TEMPERATURE_XLSB_REG          (0xFC)  /* Temperature XLSB Reg */
#define QMP6988_DATA_FRAME_SIZE			6
#define QMP6988_FORCED_MODE                   (0x01)



#define QMP6988_OVERSAMP_SKIPPED          (0x00)
#define QMP6988_OVERSAMP_1X               (0x01)
#define QMP6988_OVERSAMP_2X               (0x02)
#define QMP6988_OVERSAMP_4X               (0x03)
#define QMP6988_OVERSAMP_8X               (0x04)
#define QMP6988_OVERSAMP_16X              (0x05)

// configure pressure and temperature oversampling, forced sampling mode
#define QMP6988_PRESSURE_OSR              (QMP6988_OVERSAMP_8X)
#define QMP6988_TEMPERATURE_OSR           (QMP6988_OVERSAMP_1X)
#define QMP6988_MODE                      (QMP6988_PRESSURE_OSR << 2 | QMP6988_TEMPERATURE_OSR << 5 | QMP6988_FORCED_MODE)

#define T_INIT_MAX                       (20)
// 20/16 = 1.25 ms
#define T_MEASURE_PER_OSRS_MAX           (37)
// 37/16 = 2.3125 ms
#define T_SETUP_PRESSURE_MAX             (10)
// 10/16 = 0.625 ms

bool qmp6988Detect(baroDev_t *baro);
