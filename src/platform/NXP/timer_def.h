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
 * timer_def.h — Timer definitions for the NavQ95 (NXP i.MX 95, Cortex-M7).
 *
 * drivers/timer.h includes this header unconditionally (pulled in transitively
 * via dshot.h and dma_reqmap.h). No generic timer channels are defined yet; the
 * i.MX 95 FlexPWM/GPT bring-up is handled by a dedicated driver, as on the
 * RT1176 (IMXRT) port.
 */

#pragma once

#include "platform.h"
#include "common/utils.h"
#include "drivers/timer_types.h"

/* i.MX 95 does not use the STM32-style DEF_TIM macro system. Provide only the
 * macro the generic timer code references so the header compiles. */
#if defined(USE_TIMER_MGMT)
#define TIMER_GET_IO_TAG(pin) DEFIO_TAG_E(pin)
#else
#define TIMER_GET_IO_TAG(pin) DEFIO_TAG(pin)
#endif
