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

#include "platform.h"

#ifdef USE_PINIO

#include "pg/pg_ids.h"
#include "pinio.h"
#include "drivers/io.h"

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
        PINIO_CONFIG_MODE_OUT_PP,
        PINIO_CONFIG_MODE_OUT_PP,
        PINIO_CONFIG_MODE_OUT_PP,
        PINIO_CONFIG_MODE_OUT_PP
    },
);
#endif
