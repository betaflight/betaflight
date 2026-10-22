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
 * navq95_scmi.h — internal NavQ95 (i.MX 95) System-Manager SCMI helpers.
 *
 * On the i.MX 95 the Cortex-M7 does NOT own the IOMUXC or the CCM: the System
 * Manager (running on the M33) owns them and the M7 asks it to mux pads and
 * ungate clocks over SCMI (via the MU5 shared-memory transport). These helpers
 * are implemented once in platform_stubs.c (the only place that drives the SMT
 * handshake) and are shared by the peripheral bring-up files (console, LPSPI1,
 * LPI2C1/LPI2C6, ...) through this header.
 *
 * IOMUXC_BASE = 0x443C0000. A pad "mux register" absolute address uniquely
 * identifies the pin (identifier = (muxReg - IOMUXC_BASE) / 4); the "config
 * register" (pad electrical) value is PRE-SHIFTED (the SM applies <<1, i.e.
 * send raw_value >> 1). See the pad-value comments in platform_stubs.c.
 */

#pragma once

#include <stdint.h>

/*
 * SCMI PINCTRL_CONFIG_SET for a DEDICATED pad (no SELECT_INPUT/daisy): program
 * the pad MUX (mode + optional SION) and CONFIG (pad electrical) value.
 *   muxReg  : absolute IOMUXC mux-control register (IOMUXC_BASE + offset)
 *   muxMode : ALT mux mode (0..7)
 *   sion    : 1 to force the input path (SION), else 0
 *   padReg  : absolute IOMUXC pad-config register (kept for clarity; unused)
 *   padVal  : pre-shifted pad electrical value (raw >> 1)
 * Returns the SCMI status (0 = success).
 */
int navq95_scmi_pinmux(uint32_t muxReg, uint32_t muxMode, uint32_t sion,
                       uint32_t padReg, uint32_t padVal);

/*
 * SCMI PINCTRL_CONFIG_SET for a SHARED pad that needs a SELECT_INPUT/daisy
 * register (alternate-function routing, ALT != 0). Emits the MUX, DAISY_ID,
 * DAISY_CFG and CONFIG config pairs.
 *   daisyReg : absolute SELECT_INPUT register (IOMUXC_BASE + 0x408 + 4*id)
 *   daisyVal : input mux selector written to the SELECT_INPUT register
 * Other arguments as navq95_scmi_pinmux(). Returns the SCMI status.
 */
int navq95_scmi_pinmux_daisy(uint32_t muxReg, uint32_t muxMode, uint32_t sion,
                             uint32_t daisyReg, uint32_t daisyVal,
                             uint32_t padVal);

/*
 * SCMI CLOCK_CONFIG_SET: ungate (enable) a clock root. clockRootId is the
 * i.MX 95 CCM root index (e.g. LPSPI1 = 8, LPI2C1 = 6, LPI2C6 = 91); the SM
 * clock id = root + 41. Returns the SCMI status (0 = success).
 */
int navq95_scmi_clock_enable(uint32_t clockRootId);

/*
 * SCMI CLOCK_RATE_SET: program a clock root's physical rate in Hz. Call BEFORE
 * navq95_scmi_clock_enable() for roots the boot firmware leaves un-rated (e.g.
 * the LPI2C roots) so the peripheral actually receives a functional clock.
 * clockRootId is the CCM root index. Returns the SCMI status (0 = success).
 */
int navq95_scmi_clock_rate_set(uint32_t clockRootId, uint32_t rateHz);

/*
 * Console debug helpers (LPUART5), for peripheral bring-up diagnostics. Write a
 * string / a 0x-prefixed 8-hex-digit word to the same console as the boot
 * banner. Safe to call once the console is up (after systemInit()).
 */
void navq95_dbg_puts(const char *s);
void navq95_dbg_hex32(uint32_t v);
