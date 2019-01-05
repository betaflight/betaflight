/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

#include "drivers/io_types.h"

typedef enum {
    EXTI_TRIGGER_RISING = 0,
    EXTI_TRIGGER_FALLING = 1,
    EXTI_TRIGGER_BOTH = 2
} extiTrigger_t;

typedef struct extiCallbackRec_s extiCallbackRec_t;
typedef void extiHandlerCallback(extiCallbackRec_t *self);

struct extiCallbackRec_s {
    extiHandlerCallback *fn;
};

void EXTIInit(void);

void EXTIHandlerInit(extiCallbackRec_t *cb, extiHandlerCallback *fn);
void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, ioConfig_t config, extiTrigger_t trigger);
void EXTIRelease(IO_t io);
void EXTIEnable(IO_t io, bool enable);
