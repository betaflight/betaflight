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
/*
 * porting for ch32h41x by Temperslee
 */
#pragma once

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "CH415"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight CH32H41x"
#endif

#ifndef CH32H41x
#define CH32H41x
#endif


#define USE_VCP
#define USE_USB_DETECT

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define USE_UART7
#define USE_UART8

// #define TARGET_IO_PORTA         0xfeff
// #define TARGET_IO_PORTB         0xffdb
// #define TARGET_IO_PORTC         0x1fdf
// #define TARGET_IO_PORTD         0x0008
// #define TARGET_IO_PORTE         0x7879
// #define TARGET_IO_PORTF         0x0038

#define TARGET_IO_PORTA         0xff1f
#define TARGET_IO_PORTB         0xff7c
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         0xffff
#define TARGET_IO_PORTF         0xffff


#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4
#define USE_SPI_DMA_ENABLE_EARLY

#define USE_EXTI
#define USE_GYRO_EXTI

#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3
#define USE_I2C_DEVICE_4


#define USE_PERSISTENT_MSC_RTC

#define USE_ADC

// Remove these undefines as support is added
#undef USE_TRANSPONDER
#undef USE_DSHOT_DMAR

// #undef USE_DSHOT
// #undef USE_DSHOT_TELEMETRY

// #define USE_DSHOT
// #define USE_DSHOT_DMAR
// #define USE_DSHOT_TELEMETRY


// #undef USE_DSHOT_BITBANG
#define USE_DSHOT_BITBAND

#define USE_BEEPER

// #define USE_VTX
#define USE_OSD
#define USE_OSD_HD
#define USE_MAG
#define USE_GPS
#define USE_GPS_UBLOX
#define USE_GPS_RESCUE


#define USE_ALTITUDE_HOLD
#define USE_POSITION_HOLD

#undef USE_RX_PPM
#undef USE_RX_PWM
#undef USE_RX_SPI
#undef USE_RX_CC2500
#undef USE_RX_EXPRESSLRS
// #undef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#undef USE_SERIAL_4WAY_SK_BOOTLOADER
#define USE_ESCSERIAL

#define FLASH_PAGE_SIZE ((uint32_t)0x2000) // 8K sectors
