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
 * Author: 4712
*/
#pragma once

#include "drivers/io_types.h"

typedef struct {
    IO_t io;
} escHardware_t;

extern uint8_t selected_esc;

bool isEscHi(uint8_t selEsc);
bool isEscLo(uint8_t selEsc);
void setEscHi(uint8_t selEsc);
void setEscLo(uint8_t selEsc);
void setEscInput(uint8_t selEsc);
void setEscOutput(uint8_t selEsc);

#define ESC_IS_HI  isEscHi(selected_esc)
#define ESC_IS_LO  isEscLo(selected_esc)
#define ESC_SET_HI setEscHi(selected_esc)
#define ESC_SET_LO setEscLo(selected_esc)
#define ESC_INPUT  setEscInput(selected_esc)
#define ESC_OUTPUT setEscOutput(selected_esc)

typedef struct ioMem_s {
    uint8_t D_NUM_BYTES;
    uint8_t D_FLASH_ADDR_H;
    uint8_t D_FLASH_ADDR_L;
    uint8_t *D_PTR_I;
} ioMem_t;

