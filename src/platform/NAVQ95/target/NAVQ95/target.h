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
