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

// The GPIO pad table for the family being built. Only this file names the
// generated per family tables.
#if defined(IMX95)
#include "iomux_pad_def_imx95.h"
#elif defined(IMXRT11XX)
#include "iomux_pad_def_imxrt11xx.h"
#elif defined(IMXRT10XX)
#include "iomux_pad_def_imxrt10xx.h"
#else
#error "Unsupported i.MXRT family: no pad table (add the family include here)"
#endif
