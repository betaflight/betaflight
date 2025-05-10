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
#ifndef PINIO5_PIN
#define PINIO5_PIN NONE
#endif
#ifndef PINIO6_PIN
#define PINIO6_PIN NONE
#endif
#ifndef PINIO7_PIN
#define PINIO7_PIN NONE
#endif
#ifndef PINIO8_PIN
#define PINIO8_PIN NONE
#endif

#ifndef PINIO1_CONFIG
#define PINIO1_CONFIG PINIO_CONFIG_MODE_OUT_PP
#endif

#ifndef PINIO2_CONFIG
#define PINIO2_CONFIG PINIO_CONFIG_MODE_OUT_PP
#endif

#ifndef PINIO3_CONFIG
#define PINIO3_CONFIG PINIO_CONFIG_MODE_OUT_PP
#endif

#ifndef PINIO4_CONFIG
#define PINIO4_CONFIG PINIO_CONFIG_MODE_OUT_PP
#endif

#ifndef PINIO5_CONFIG
#define PINIO5_CONFIG PINIO_CONFIG_MODE_OUT_PP
#endif

#ifndef PINIO6_CONFIG
#define PINIO6_CONFIG PINIO_CONFIG_MODE_OUT_PP
#endif

#ifndef PINIO7_CONFIG
#define PINIO7_CONFIG PINIO_CONFIG_MODE_OUT_PP
#endif

#ifndef PINIO8_CONFIG
#define PINIO8_CONFIG PINIO_CONFIG_MODE_OUT_PP
#endif

PG_REGISTER_WITH_RESET_FN(pinioConfig_t, pinioConfig, PG_PINIO_CONFIG, 0);

void pgResetFn_pinioConfig(pinioConfig_t *config)
{
    config->ioTag[0] = IO_TAG(PINIO1_PIN);
    config->ioTag[1] = IO_TAG(PINIO2_PIN);
    config->ioTag[2] = IO_TAG(PINIO3_PIN);
    config->ioTag[3] = IO_TAG(PINIO4_PIN);
    config->ioTag[4] = IO_TAG(PINIO5_PIN);
    config->ioTag[5] = IO_TAG(PINIO6_PIN);
    config->ioTag[6] = IO_TAG(PINIO7_PIN);
    config->ioTag[7] = IO_TAG(PINIO8_PIN);

    config->config[0] = PINIO1_CONFIG;
    config->config[1] = PINIO2_CONFIG;
    config->config[2] = PINIO3_CONFIG;
    config->config[3] = PINIO4_CONFIG;
    config->config[4] = PINIO5_CONFIG;
    config->config[5] = PINIO6_CONFIG;
    config->config[6] = PINIO7_CONFIG;
    config->config[7] = PINIO8_CONFIG;
}
#endif
