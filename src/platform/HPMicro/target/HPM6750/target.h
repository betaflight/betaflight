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

/*
 * Target configuration and feature selection for the HPM6750 MCU.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "common/utils.h"
#include "hpm_dma_drv.h"
#include "hpm_otp_drv.h"
#include "hpm_soc.h"
#include "stdio.h"
#include "string.h"
#include "io_def.h"

/* ============================================================================
 * Feature Disables
 * ========================================================================== */

#undef TASK_GYROPID_DESIRED_PERIOD
#undef SCHEDULER_DELAY_LIMIT
#undef USE_DASHBOARD
#undef USE_TELEMETRY_LTM
#undef USE_VTX_RTC6705_SOFTSPI
#undef USE_RX_REDPINE_SPI
#undef USE_RX_CC2500
#undef USE_RX_SX1280
#undef USE_RX_SX127X
#undef USE_RANGEFINDER
#undef USE_RX_EXPRESSLRS
#undef USE_RX_PPM
#undef USE_RX_PWM
#undef USE_TELEMETRY_FRSKY_HUB
#undef USE_TELEMETRY_HOTT
#undef USE_TELEMETRY_SMARTPORT
#undef USE_TELEMETRY_MAVLINK
#undef USE_TELEMETRY_CRSF
#undef USE_TELEMETRY_GHST
#undef USE_TELEMETRY_IBUS
#undef USE_TELEMETRY_JETIEXBUS
#undef USE_TELEMETRY_SRXL
#undef USE_SERIALRX_JETIEXBUS
#undef USE_PINIO
#undef USE_SERVOS
#undef USE_MULTI_GYRO
#undef USE_DSHOT_DMAR
#undef USE_DSHOT_BITBANG
#undef USE_RPM_LIMIT
#undef USE_TRANSPONDER
#undef USE_FLASH_M25P16
#undef USE_FLASHFS
#undef USE_FLASH_TOOLS
#undef USE_FLASH_W25N01G
#undef USE_FLASH_W25N02K
#undef USE_FLASH_W25M
#undef USE_FLASH_W25M512
#undef USE_FLASH_W25M02G
#undef USE_FLASH_W25Q128FV
#undef USE_FLASH_PY25Q128HA
#undef USE_MAX7456
#undef USE_RX_SPI
#undef USE_OSD_HD
#undef USE_LED_STRIP

/* ============================================================================
 * Target Identification
 * ========================================================================== */

#define TARGET_BOARD_IDENTIFIER "HP67"

/* ============================================================================
 * Platform Configuration
 * ========================================================================== */

#define FAST_IRQ_HANDLER FAST_CODE
#define SERIAL_TRAIT_PIN_CONFIG  1
#define UART_TRAIT_AF_PORT       1
#define SPI_TRAIT_AF_PIN         1
#define I2C_TRAIT_AF_PIN         1

/* ============================================================================
 * Motor and Mixer Configuration
 * ========================================================================== */

#define MAX_SUPPORTED_MOTORS     4
#define FULL_TIMER_CHANNEL_COUNT 4
#define USE_QUAD_MIXER_ONLY      1
#define DEFAULT_CPU_OVERCLOCK    1

/* ============================================================================
 * DMA Memory Attributes
 * ========================================================================== */

#define DMA_DATA_ZERO_INIT ATTR_ALIGN (64) __attribute__((section(".noncacheable.bss")))
#define DMA_DATA          ATTR_ALIGN (64) __attribute__((section(".noncacheable.init")))
#define STATIC_DMA_DATA_AUTO static

/* ============================================================================
 * Storage Configuration
 * ========================================================================== */

#define EEPROM_SIZE 32768
#define USE_USB_MSC 1
#define USE_SDCARD_SDIO 1
#define SDIO_DEVICE SDIODEV_1

#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_SDCARD

/* ============================================================================
 * Device Unique ID
 * ========================================================================== */

#define U_ID_0 otp_read_from_shadow(OTP_SOC_UUID_IDX)
#define U_ID_1 otp_read_from_shadow(OTP_SOC_UUID_IDX + 1U)
#define U_ID_2 otp_read_from_shadow(OTP_SOC_UUID_IDX + 2U)

/* ============================================================================
 * Peripheral Features
 * ========================================================================== */

#define USE_EXTI
#define USE_ADC
#define USE_I2C
#define USE_I2C_PULLUP
#define USE_I2C_DEVICE_0
#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2

#define USE_PERSISTENT_OBJECTS 1
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define TASK_GYROPID_DESIRED_PERIOD TASK_PERIOD_HZ (1000)
#define SCHEDULER_DELAY_LIMIT 1
#define USE_TIMER_DMA

/* ============================================================================
 * UART Configuration
 * ========================================================================== */

#define USE_UART1
#define USE_UART6
#define USE_UART8

/* ============================================================================
 * Motor and Serial Configuration
 * ========================================================================== */

#define USE_MOTOR
#define UNIFIED_SERIAL_PORT_COUNT 4
#define REQUIRE_CC_ARM_PRINTF_SUPPORT
#define DEFAULT_RX_FEATURE FEATURE_RX_SERIAL
#define DEFAULT_FEATURES (FEATURE_TELEMETRY)
#define TARGET_NO_PWM_OUTPUT

/* ============================================================================
 * SPI Configuration
 * ========================================================================== */

#define USE_SPI
#define USE_SPI_DMA_ENABLE_LATE
#define USE_SPI_DEVICE_0
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define USE_PIN_AF
#define USE_DMA_SPEC
#define USE_RPM_FILTER 1
#define USE_OSD
#define USE_SPI_GYRO 1

#if !defined(USE_CONFIG)
#define USE_VIRTUAL_GYRO
#endif

/* ============================================================================
 * IO Port Definitions
 * ========================================================================== */

#define TARGET_IO_PORTA 0xffffffff
#define TARGET_IO_PORTB 0xffffffff
#define TARGET_IO_PORTC 0xffffffff
#define TARGET_IO_PORTD 0xffffffff
#define TARGET_IO_PORTE 0xffffffff
#define TARGET_IO_PORTF 0xffffffff

/* ============================================================================
 * Flash Configuration
 * ========================================================================== */

#define DEFIO_NO_PORTS          // suppress 'no pins defined' warning
#undef USE_FLASH
#define FLASH_PAGE_SIZE (0x400)

/* ============================================================================
 * USB Configuration
 * ========================================================================== */

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "HPM6700 Platform"
#endif

/* ============================================================================
 * UART Direction Switch Timer Configuration
 * ========================================================================== */

#ifndef UART_DIR_SWITCH_GPTMR
#define UART_DIR_SWITCH_GPTMR         HPM_GPTMR4
#endif

#ifndef UART_DIR_SWITCH_GPTMR_CLOCK
#define UART_DIR_SWITCH_GPTMR_CLOCK   clock_gptmr4
#endif

#ifndef UART_DIR_SWITCH_GPTMR_IRQ
#define UART_DIR_SWITCH_GPTMR_IRQ     IRQn_GPTMR4
#endif

#ifndef UART_DIR_SWITCH_GPTMR_CHANNEL
#define UART_DIR_SWITCH_GPTMR_CHANNEL 0
#endif

/* ============================================================================
 * Flash Configuration
 * ========================================================================== */

#ifndef BOARD_APP_XPI_NOR_XPI_BASE
#define BOARD_APP_XPI_NOR_XPI_BASE     (HPM_XPI0)
#endif

#ifndef BOARD_APP_XPI_NOR_CFG_OPT_HDR
#define BOARD_APP_XPI_NOR_CFG_OPT_HDR  (0xfcf90002U)
#endif

#ifndef BOARD_APP_XPI_NOR_CFG_OPT_OPT0
#define BOARD_APP_XPI_NOR_CFG_OPT_OPT0 (0x00000005U)
#endif

#ifndef BOARD_APP_XPI_NOR_CFG_OPT_OPT1
#define BOARD_APP_XPI_NOR_CFG_OPT_OPT1 (0x00001000U)
#endif

#ifndef BOARD_CPU_FREQ
#define BOARD_CPU_FREQ (816000000UL)
#endif

/* ============================================================================
 * Console UART Pin Configuration
 * ========================================================================== */

#ifndef CONSOLE_UART_RX_PIN
#define CONSOLE_UART_RX_PIN   DEFIO_TAG(PY7)
#endif

#ifndef CONSOLE_UART_TX_PIN
#define CONSOLE_UART_TX_PIN   DEFIO_TAG(PY6)
#endif

#ifndef CONSOLE_RX_AF
#define CONSOLE_RX_AF      IOC_PY07_FUNC_CTL_UART0_RXD
#endif

#ifndef CONSOLE_TX_AF
#define CONSOLE_TX_AF      IOC_PY06_FUNC_CTL_UART0_TXD
#endif

/* ============================================================================
 * SDXC Pin Configuration
 * ========================================================================== */

#ifndef SDXC_CD_PIN
#define SDXC_CD_PIN   DEFIO_TAG(PE18)
#endif

#ifndef SDXC_CD_PIN_IOCIDX
#define SDXC_CD_PIN_IOCIDX IOC_PAD_PE18
#endif

#ifndef SDXC_CD_PIN_AF
#define SDXC_CD_PIN_AF     IOC_PE18_FUNC_CTL_GPIO_E_18
#endif

#ifndef SDXC_CD_PIN_SDXC_AF
#define SDXC_CD_PIN_SDXC_AF IOC_PE18_FUNC_CTL_SDC0_CDN
#endif

#ifndef SDXC_CD_PIN_POL
#define SDXC_CD_PIN_POL    1
#endif

#ifndef SDXC_CMD_PIN
#define SDXC_CMD_PIN   DEFIO_TAG(PE22)
#endif

#ifndef SDXC_CMD_PIN_AF
#define SDXC_CMD_PIN_AF    IOC_PE22_FUNC_CTL_SDC0_CMD
#endif

#ifndef SDXC_CLK_PIN
#define SDXC_CLK_PIN   DEFIO_TAG(PE27)
#endif

#ifndef SDXC_CLK_PIN_AF
#define SDXC_CLK_PIN_AF    IOC_PE27_FUNC_CTL_SDC0_CLK
#endif

#ifndef SDXC_DATA0_PIN
#define SDXC_DATA0_PIN   DEFIO_TAG(PE26)
#endif

#ifndef SDXC_DATA0_PIN_AF
#define SDXC_DATA0_PIN_AF  IOC_PE26_FUNC_CTL_SDC0_DATA_0
#endif

#ifndef SDXC_DATA1_PIN
#define SDXC_DATA1_PIN   DEFIO_TAG(PE21)
#endif

#ifndef SDXC_DATA1_PIN_AF
#define SDXC_DATA1_PIN_AF  IOC_PE21_FUNC_CTL_SDC0_DATA_1
#endif

#ifndef SDXC_DATA2_PIN
#define SDXC_DATA2_PIN   DEFIO_TAG(PE28)
#endif

#ifndef SDXC_DATA2_PIN_AF
#define SDXC_DATA2_PIN_AF  IOC_PE28_FUNC_CTL_SDC0_DATA_2
#endif

#ifndef SDXC_DATA3_PIN
#define SDXC_DATA3_PIN   DEFIO_TAG(PE23)
#endif

#ifndef SDXC_DATA3_PIN_AF
#define SDXC_DATA3_PIN_AF  IOC_PE23_FUNC_CTL_SDC0_DATA_3
#endif
