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

#pragma once

#include <stdint.h>

#include "drivers/io_types.h"

// The 5 tuple matches the IOMUXC_<pad>_<signal> macros in the vendor header, so
// a table entry can be written as that macro. An output leaves the daisy zero.
typedef struct nxpPinMux_s {
    uint32_t muxRegister;
    uint32_t muxMode;
    uint32_t inputRegister;
    uint32_t inputDaisy;
    uint32_t configRegister;
} nxpPinMux_t;

// Per pad entry of the generated GPIO table, indexed by (port, pin). Offsets are
// relative to the IOMUXC base; gpioMuxMode differs between pad groups.
typedef struct nxpGpioPad_s {
    uint16_t muxRegOffset;
    uint16_t padCtlRegOffset;
    uint8_t gpioMuxMode;
    uint8_t padGroup;       // imxrtPadGroup_e, 11xx only
} nxpGpioPad_t;

// An open drain pad also gets SION set, so the peripheral can read the line back
// while driving it. A pad in NXP_RESERVED_PINS is left alone.
void nxpIomuxConfigPin(const nxpPinMux_t *mux, ioConfig_t cfg);

// A cfg asking for IOCFG_MODE_AF updates only the pad control register, leaving
// the mux where nxpIomuxConfigPin() put it.
void nxpIomuxConfigGpio(IO_t io, ioConfig_t cfg);

// Give every pad in NXP_RESERVED_PINS to OWNER_SYSTEM.
void nxpIomuxClaimReserved(void);
