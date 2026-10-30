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
 * System Manager (SCMI) helpers. On the i.MX 95 the System Manager owns IOMUXC
 * and the clock controller, so the M7 asks it to mux pads and enable clocks.
 *
 * A pad is identified by its absolute IOMUXC mux register; padVal is the pad
 * control value passed to the System Manager.
 */

#pragma once

#include <stdint.h>

// Mux a pad without an input select (daisy) register. padReg is unused.
int navq95_scmi_pinmux(uint32_t muxReg, uint32_t muxMode, uint32_t sion,
                       uint32_t padReg, uint32_t padVal);

// Mux a pad that also needs its input select (daisy) register set.
int navq95_scmi_pinmux_daisy(uint32_t muxReg, uint32_t muxMode, uint32_t sion,
                             uint32_t daisyReg, uint32_t daisyVal,
                             uint32_t padVal);

// Enable a clock root, by CCM root index.
int navq95_scmi_clock_enable(uint32_t clockRootId);

// Set the rate of a clock root, by CCM root index.
int navq95_scmi_clock_rate_set(uint32_t clockRootId, uint32_t rateHz);
