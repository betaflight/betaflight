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

#include <stdint.h>

#include <platform.h>

#ifdef USE_PINIO

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "build/debug.h"

#include "drivers/io.h"

#include "pinio.h"

#ifndef PINIO1_PIN
#define PINIO1_PIN NONE
#endif
#ifndef PINIO2_PIN
#define PINIO2_PIN NONE
#endif
#ifndef PINIO3_PIN
#define PINIO3_PIN NONE
#endif
#ifndef PINIO4_PIN
#define PINIO4_PIN NONE
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(pinioConfig_t, pinioConfig, PG_PINIO_CONFIG, 0);

PG_RESET_TEMPLATE(pinioConfig_t, pinioConfig,
    .ioTag = {
        IO_TAG(PINIO1_PIN),
        IO_TAG(PINIO2_PIN),
        IO_TAG(PINIO3_PIN),
        IO_TAG(PINIO4_PIN),
    },
    .config = {
        PINIO_CONFIG_OUT_PP,
        PINIO_CONFIG_OUT_PP,
        PINIO_CONFIG_OUT_PP,
        PINIO_CONFIG_OUT_PP
    },
);

typedef struct pinioRuntime_s {
    IO_t io;
    bool inverted;
} pinioRuntime_t;

static pinioRuntime_t pinioRuntime[MAX_PINIO];

void pinioInit(const pinioConfig_t *pinioConfig)
{
    for (int i = 0; i < MAX_PINIO; i++) {
        IO_t io = IOGetByTag(pinioConfig->ioTag[i]);

        if (!io) {
            continue;
        }

        IOInit(io, OWNER_PINIO, RESOURCE_INDEX(i));

        switch (pinioConfig->config[i] & 0x7f) {
        case PINIO_CONFIG_OUT_PP:
            IOConfigGPIO(io, IOCFG_OUT_PP);
            break;
        }

        if (pinioConfig->config[i] & PINIO_CONFIG_OUT_INVERTED)
        {
            pinioRuntime[i].inverted = true;
            IOHi(io);
        } else {
            pinioRuntime[i].inverted = false;
            IOLo(io);
        }
        pinioRuntime[i].io = io;
    }
}

void pinio(int index, bool on)
{
    if (on ^ pinioRuntime[index].inverted) {
        IOHi(pinioRuntime[index].io);
    } else {
        IOLo(pinioRuntime[index].io);
    }
}

void pinioON(int index)
{
    pinio(index, true);
}

void pinioOFF(int index)
{
    pinio(index, false);
}
#endif
