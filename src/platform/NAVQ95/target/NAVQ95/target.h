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
 * target.h — NavQ95 (NXP i.MX 95, Cortex-M7) base target.
 */

#pragma once

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "N095"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight - NXP NavQ95 (i.MX95 M7)"
#endif

#ifndef IMX95_M7
#define IMX95_M7
#endif

/*
 * Configuration storage in RAM (config_streamer's own buffer). Selecting a
 * CONFIG_IN_* backend is required; CONFIG_IN_RAM needs no flash hooks. Config
 * is volatile across power cycles until a flash backend is added.
 */
#define CONFIG_IN_RAM
#define EEPROM_SIZE             8192

/*
 * Expose one logical serial port (USB VCP) so SERIAL_PORT_COUNT >= 1. The port
 * is backed by the LPUART console in platform_stubs.c and carries the CLI.
 */
#define USE_VCP

/*
 * Second logical serial port: UART7, backed by LPUART7 in
 * serial_uart_navq95.c via Betaflight's generic drivers/serial_uart.c core.
 * This is a real Betaflight UART port (SERIAL_PORT_UART7) so MSP can be assigned
 * to it from the Configurator's Ports tab while the LPUART2 console keeps the
 * CLI.
 *
 * Pads (NavQ95 board): TX = GPIO_IO08 = PA8 (RGPIO1 bit 8, LPUART7_TX ALT5),
 *                      RX = GPIO_IO09 = PA9 (RGPIO1 bit 9, LPUART7_RX ALT5).
 * These tags select the uartHardware[] pin rows; the actual LPUART alt mux +
 * RX SELECT_INPUT daisy are applied over SCMI by the driver.
 *
 * SERIAL_UART_FIRST_INDEX = 1: UART numbering starts at 1 (no UART0), matching
 * the generated serial_post.h default and the STM32-style targets.
 */
#define USE_UART
#define USE_UART7
/*
 * Also define USE_USART7: pg/serial_uart.c gates the UART7 entry of its
 * uartDmaopt[] table on USE_USART7 (an upstream spelling quirk — ports enable
 * USE_UART7). Without it the table is empty on this single-UART target, and its
 * `for (unsigned i = 0; i < ARRAYLEN(uartDmaopt); i++)` trips
 * -Werror=type-limits (i < 0 always false). Defining USE_USART7 adds the entry
 * (its DMA opts default to DMA_OPT_UNUSED) so the loop bound is non-zero. It is
 * referenced nowhere else, so this is inert beyond that table.
 */
#define USE_USART7
#define SERIAL_UART_FIRST_INDEX 1
#define UART7_TX_PIN            PA8
#define UART7_RX_PIN            PA9

/*
 * MSP over UART7. pg/msp.c claims serial ports for FUNCTION_MSP at config reset:
 * slot 0 is always the first port (the VCP console, so the CLI stays reachable),
 * and MSP_UART is claimed as an additional MSP slot. Defining it here makes the
 * Betaflight Configurator connect over UART7 (LPUART7, 115200 8N1) out of the
 * box, while the LPUART2 console keeps the CLI. Can still be reassigned from the
 * Configurator's Ports tab / `msp_2_uart` CLI setting.
 */
#define MSP_UART               SERIAL_PORT_UART7

/*
 * Beeper (buzzer). GPIO-driven (no PWM): sound_beeper.c toggles the pin via the
 * IO HAL when BEEPER_PWM_HZ is 0 (the default). The pad is GPIO_IO23 = GPIO2_IO23
 * = Betaflight tag PB23 (RGPIO2[23]); its IOMUXC mux row lives in io_navq95.c
 * (navq95_padMux[]). beeperInit() -> IOConfigGPIO(PB23, ...) brings the pin up.
 *
 * Polarity (see pg/beeper_dev.c): the NavQ95 buzzer is driven active-high /
 * push-pull, so BEEPER_INVERTED is defined (IS_INVERTED=true, IS_OPEN_DRAIN=
 * false) — systemBeep(true) then drives the pin HIGH to sound the buzzer.
 */
#define USE_BEEPER
#define BEEPER_PIN              PB23
#define BEEPER_INVERTED

/*
 * Sensors use the virtual drivers for now; the USE_VIRTUAL_ACC/GYRO/MAG/BARO
 * defines are set in mk/NAVQ95.mk (they must be seen before common_pre.h, which
 * platform.h includes ahead of this header).
 */

/* Pull in the platform abstraction (CMSIS includes, type shims, capabilities). */
#include "platform.h"

/*
 * 64-bit DWT CYCCNT time base helper. navq95_timebaseTick() folds the CYCCNT
 * wrap and must be called periodically (from SysTick, once configured) so no
 * wrap is lost. navq95_timebaseInit() configures the Cortex-M7 SysTick at 1 kHz
 * to drive that fold; call it after cycleCounterInit(). The 32-bit
 * micros()/millis()/microsISR() primitives (declared in drivers/time.h) are
 * built on the same fold.
 */
void navq95_timebaseTick(void);
void navq95_timebaseInit(void);
