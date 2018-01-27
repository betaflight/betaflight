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

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/io.h"
#include "drivers/timer_def.h"

#ifdef USE_TIMER_MGMT

typedef struct timerChannelConfig_s {
    ioTag_t ioTag;
#if defined(STM32F3) || defined(STM32F4) || defined(STM32F7)
    uint8_t pinAF;
#endif
    uint8_t dma;
#if defined(STM32F4) || defined(STM32F7)
    uint8_t dmaChannel;
#endif
    uint8_t inverted;
} timerChannelConfig_t;

PG_DECLARE_ARRAY(timerChannelConfig_t, TIMER_CHANNEL_COUNT, timerChannelConfig);

#endif