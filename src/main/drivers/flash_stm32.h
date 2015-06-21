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

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct flash_stm32_state_s {
    uintptr_t address;
    int err;
    bool unlocked;
} flash_stm32_writer_t;

void flash_stm32_init(flash_stm32_writer_t *f);

int flash_stm32_start(flash_stm32_writer_t *f, uintptr_t base);
int flash_stm32_write(flash_stm32_writer_t *f, const void *p, uint32_t size);
int flash_stm32_finish(flash_stm32_writer_t *f);
