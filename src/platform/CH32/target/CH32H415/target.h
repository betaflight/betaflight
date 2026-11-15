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
#define TARGET_BOARD_IDENTIFIER "H415"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight CH32H41x"
#endif

#ifndef CH32H41x
#define CH32H41x
#endif

#ifndef USE_VCP
#define USE_VCP
#endif
#ifndef USE_USB_DETECT
#define USE_USB_DETECT
#endif
#ifndef USE_USB_MSC
#define USE_USB_MSC
#endif

#ifndef USE_UART1
#define USE_UART1
#endif
#ifndef USE_UART2
#define USE_UART2
#endif
#ifndef USE_UART3
#define USE_UART3
#endif
#ifndef USE_UART4
#define USE_UART4
#endif
#ifndef USE_UART5
#define USE_UART5
#endif
#ifndef USE_UART6
#define USE_UART6
#endif
#ifndef USE_UART7
#define USE_UART7
#endif
#ifndef USE_UART8
#define USE_UART8
#endif

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         0xffff
#define TARGET_IO_PORTF         0xffff

#ifndef USE_SPI
#define USE_SPI
#endif
#ifndef USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_1
#endif
#ifndef USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_2
#endif
#ifndef USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_3
#endif
#ifndef USE_SPI_DEVICE_4
#define USE_SPI_DEVICE_4
#endif
#ifndef USE_SPI_DMA_ENABLE_EARLY
#define USE_SPI_DMA_ENABLE_EARLY
#endif

#ifndef USE_EXTI
#define USE_EXTI
#endif
#ifndef USE_GYRO_EXTI
#define USE_GYRO_EXTI
#endif

#ifndef USE_I2C
#define USE_I2C
#endif
#ifndef USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_1
#endif
#ifndef USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_2
#endif
#ifndef USE_I2C_DEVICE_3
#define USE_I2C_DEVICE_3
#endif
#ifndef USE_I2C_DEVICE_4
#define USE_I2C_DEVICE_4
#endif

#ifndef USE_PERSISTENT_MSC_RTC
#define USE_PERSISTENT_MSC_RTC
#endif

#ifndef USE_ADC
#define USE_ADC
#endif
#ifndef USE_PLATFORM_ADC_TEMPERATURE
#define USE_PLATFORM_ADC_TEMPERATURE
#endif

#undef USE_TRANSPONDER
#undef USE_DSHOT_DMAR

#ifndef USE_BEEPER
#define USE_BEEPER
#endif

#if defined(CAMERA_CONTROL_PIN) || (!defined(CLOUD_BUILD) && !defined(CORE_BUILD))
#ifndef USE_CAMERA_CONTROL
#define USE_CAMERA_CONTROL
#endif
#endif

#if !defined(CLOUD_BUILD) && !defined(CORE_BUILD)
#ifndef USE_DSHOT
#define USE_DSHOT
#endif
#ifndef USE_DSHOT_TELEMETRY
#define USE_DSHOT_TELEMETRY
#endif
#ifndef USE_LED_STRIP
#define USE_LED_STRIP
#endif
#ifndef USE_LED_STRIP_STATUS_MODE
#define USE_LED_STRIP_STATUS_MODE
#endif
#ifndef USE_OSD
#define USE_OSD
#endif
#ifndef USE_OSD_HD
#define USE_OSD_HD
#endif
#ifndef USE_MAG
#define USE_MAG
#endif
#ifndef USE_GPS
#define USE_GPS
#endif
#ifndef USE_GPS_UBLOX
#define USE_GPS_UBLOX
#endif
#ifndef USE_GPS_RESCUE
#define USE_GPS_RESCUE
#endif
#ifndef USE_ALTITUDE_HOLD
#define USE_ALTITUDE_HOLD
#endif
#ifndef USE_POSITION_HOLD
#define USE_POSITION_HOLD
#endif
#endif

#undef USE_RX_PPM
#undef USE_RX_PWM
#undef USE_RX_SPI
#undef USE_RX_CC2500
#undef USE_RX_EXPRESSLRS
#undef USE_SERIAL_4WAY_SK_BOOTLOADER
#ifndef USE_ESCSERIAL
#define USE_ESCSERIAL
#endif

#define FLASH_PAGE_SIZE ((uint32_t)0x2000) // 8K sectors
