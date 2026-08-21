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

/*
 * HPMicro board defaults: console UART, SDXC/eMMC configuration and SD card
 * pin initialisation helpers.
 */

#pragma once

#include "hpm_sdxc_drv.h"
#include "hpm_sdxc_soc_drv.h"
#ifndef BOARD_RUNNING_CORE
#define BOARD_RUNNING_CORE HPM_CORE0
#endif

#ifndef BOARD_CONSOLE_UART_BASE
#define BOARD_CONSOLE_UART_BASE       HPM_UART0
#endif

#ifndef BOARD_CONSOLE_UART_CLK_NAME
#define BOARD_CONSOLE_UART_CLK_NAME   clock_uart0
#endif

#ifndef BOARD_CONSOLE_UART_IRQ
#define BOARD_CONSOLE_UART_IRQ        IRQn_UART0
#endif

#ifndef BOARD_CONSOLE_UART_TX_DMA_REQ
#define BOARD_CONSOLE_UART_TX_DMA_REQ HPM_DMA_SRC_UART0_TX
#endif

#ifndef BOARD_CONSOLE_UART_RX_DMA_REQ
#define BOARD_CONSOLE_UART_RX_DMA_REQ HPM_DMA_SRC_UART0_RX
#endif

#ifndef BOARD_APP_SDCARD_SDXC_BASE
#define BOARD_APP_SDCARD_SDXC_BASE                 (HPM_SDXC0)
#endif

#ifndef BOARD_APP_SDCARD_SDXC_IRQ
#define BOARD_APP_SDCARD_SDXC_IRQ                  IRQn_SDXC0
#endif

#ifndef BOARD_APP_SDCARD_SUPPORT_3V3
#define BOARD_APP_SDCARD_SUPPORT_3V3               (1)
#endif

#ifndef BOARD_APP_SDCARD_SUPPORT_1V8
#define BOARD_APP_SDCARD_SUPPORT_1V8               (0)
#endif

#ifndef BOARD_APP_SDCARD_SUPPORT_4BIT
#define BOARD_APP_SDCARD_SUPPORT_4BIT              (1)
#endif

#ifndef BOARD_APP_SDCARD_SUPPORT_CARD_DETECTION
#define BOARD_APP_SDCARD_SUPPORT_CARD_DETECTION    (1)
#endif

#ifndef BOARD_APP_SDCARD_SUPPORT_POWER_SWITCH
#define BOARD_APP_SDCARD_SUPPORT_POWER_SWITCH      (0)
#endif

#ifndef BOARD_APP_SDCARD_POWER_SWITCH_USING_GPIO
#define BOARD_APP_SDCARD_POWER_SWITCH_USING_GPIO   (0)
#endif

#ifndef BOARD_APP_SDCARD_SUPPORT_VOLTAGE_SWITCH
#define BOARD_APP_SDCARD_SUPPORT_VOLTAGE_SWITCH    (0)
#endif

#ifndef BOARD_APP_SDCARD_CARD_DETECTION_USING_GPIO
#define BOARD_APP_SDCARD_CARD_DETECTION_USING_GPIO (1)
#endif

#ifndef BOARD_APP_EMMC_SDXC_BASE
#define BOARD_APP_EMMC_SDXC_BASE                (HPM_SDXC0)
#endif

#ifndef BOARD_APP_EMMC_SDXC_IRQ
#define BOARD_APP_EMMC_SDXC_IRQ                 IRQn_SDXC0
#endif

#ifndef BOARD_APP_EMMC_SUPPORT_3V3
#define BOARD_APP_EMMC_SUPPORT_3V3              (1)
#endif

#ifndef BOARD_APP_EMMC_SUPPORT_1V8
#define BOARD_APP_EMMC_SUPPORT_1V8              (0)
#endif

#ifndef BOARD_APP_EMMC_SUPPORT_4BIT
#define BOARD_APP_EMMC_SUPPORT_4BIT             (1)
#endif

#ifndef BOARD_APP_EMMC_HOST_USING_IRQ
#define BOARD_APP_EMMC_HOST_USING_IRQ           (0)
#endif

#ifndef BOARD_APP_SDCARD_CARD_DETECTION_PIN
#define BOARD_APP_SDCARD_CARD_DETECTION_PIN     IOC_PAD_PY05
#endif

#ifndef BOARD_APP_SDCARD_CARD_DETECTION_PIN_POL
#define BOARD_APP_SDCARD_CARD_DETECTION_PIN_POL 1       /* PIN value 0 means card is inserted */
#endif

void board_delay_ms(uint32_t ms);
void board_delay_us(uint32_t us);
uint32_t board_sd_configure_clock(SDXC_Type *ptr, uint32_t freq, bool need_inverse);
void init_sdxc_cmd_pin(SDXC_Type *ptr, bool open_drain, bool is_1v8);
void init_sdxc_cd_pin(SDXC_Type *ptr, bool as_gpio);
void init_sdxc_clk_data_pins(SDXC_Type *ptr, uint32_t width, bool is_1v8);
