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

// Flag any deprecated defines with compile errors so they
// can be cleaned up and not further propagated.

#pragma once

#ifdef ACC_1_ALIGN
#error "The ACC_1_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef ACC_2_ALIGN
#error "The ACC_2_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef ACC_ICM20689_ALIGN
#error "The ACC_ICM20689_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef ACC_MPU6000_1_ALIGN
#error "The ACC_MPU6000_1_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef ACC_MPU6000_2_ALIGN
#error "The ACC_MPU6000_2_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef ACC_MPU6000_ALIGN
#error "The ACC_MPU6000_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef ACC_MPU6500_1_ALIGN
#error "The ACC_MPU6500_1_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef ACC_MPU6500_2_ALIGN
#error "The ACC_MPU6500_2_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef ACC_MPU6500_ALIGN
#error "The ACC_MPU6500_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef GYRO_ICM20689_ALIGN
#error "The GYRO_ICM20689_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef GYRO_MPU6000_1_ALIGN
#error "The GYRO_MPU6000_1_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef GYRO_MPU6000_2_ALIGN
#error "The GYRO_MPU6000_2_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef GYRO_MPU6000_ALIGN
#error "The GYRO_MPU6000_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef GYRO_MPU6500_1_ALIGN
#error "The GYRO_MPU6500_1_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef GYRO_MPU6500_2_ALIGN
#error "The GYRO_MPU6500_2_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef GYRO_MPU6500_ALIGN
#error "The GYRO_MPU6000_ALIGN define has been deprecated - please remove from the target definition"
#endif

#ifdef ICM20689_CS_PIN
#error "The ICM20689_CS_PIN define has been deprecated - please remove from the target definition"
#endif

#ifdef ICM20689_SPI_INSTANCE
#error "The ICM20689_SPI_INSTANCE define has been deprecated - please remove from the target definition"
#endif

#ifdef MPU_INT_EXTI
#error "The MPU_INT_EXTI define has been deprecated - please remove from the target definition"
#endif

#ifdef MPU6000_CS_PIN
#error "The MPU6000_CS_PIN define has been deprecated - please remove from the target definition"
#endif

#ifdef MPU6000_SPI_INSTANCE
#error "The MPU6000_SPI_INSTANCE define has been deprecated - please remove from the target definition"
#endif

#ifdef MPU6500_CS_PIN
#error "The MPU6500_CS_PIN define has been deprecated - please remove from the target definition"
#endif

#ifdef MPU6500_SPI_INSTANCE
#error "The MPU6500_SPI_INSTANCE define has been deprecated - please remove from the target definition"
#endif

#ifdef SDCARD_DMA_CHANNEL
#error "The SDCARD_DMA_CHANNEL define has been deprecated - please remove from the target definition"
#endif

#ifdef SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER
#error "The SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER define should not be part of the target definition"
#endif

#ifdef SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER
#error "The SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER define should not be part of the target definition"
#endif

#ifdef USE_DUAL_GYRO
#error "The USE_DUAL_GYRO define has been deprecated - please remove from the target definition"
#endif

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
#error "The USE_SERIAL_4WAY_BLHELI_INTERFACE define should not be part of the target definition"
#endif

