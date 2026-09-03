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

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "4xF"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight - UM324xF"
#endif

#ifndef UM324xF
#define UM324xF
#endif

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0x0000
#define TARGET_IO_PORTG 0x0000
#define TARGET_IO_PORTH 0x0000

#define USE_I2C
#define USE_I2C_DEVICE_1
// #define USE_I2C_DEVICE_2
// #define USE_I2C_DEVICE_3

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4
#define USE_SPI_DMA_ENABLE_LATE
#define USE_SPI_GYRO

//#define USE_USB_DETECT

#define USE_ESCSERIAL

#define USE_ADC
#define USE_OPA
#define USE_EXTI

// DShot bit-band decode: reads telemetry samples through the Cortex-M
// bit-band alias region (0x22000000).
#define USE_DSHOT_BITBAND

#define USE_BEEPER

// #define USE_PID_DENOM_CHECK  //polo:UM324xF performance is enlough.
#define DEFAULT_PID_PROCESS_DENOM       1

#define STANDARD_INQUIRY_DATA_LEN       0x24U


#undef USE_DSHOT_DMAR

// 4way ESC flashing: keep the BLHeli bootloader (needed for BLHeli_S/32 ESC
// firmware updates), drop the legacy STK500 (old Atmel ESC) protocol.
#undef USE_SERIAL_4WAY_SK_BOOTLOADER

// Optional further trims (enabled by uncommenting) — needed if adding
// DroneCAN (~10KB) to the build:
#undef USE_TELEMETRY_HOTT
#undef USE_TELEMETRY_SRXL
#undef USE_TELEMETRY_SMARTPORT
#undef USE_TELEMETRY_JETIEXBUS
#undef USE_TELEMETRY_GHST
#undef USE_TELEMETRY_IBUS
#undef USE_TELEMETRY_LTM
#undef USE_RANGEFINDER
#undef USE_TRANSPONDER
// NOTE: do NOT undef USE_GPS if DroneCAN GNSS is planned — the DroneCAN
// GPS provider feeds gpsSol and depends on the GPS stack.



