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

#ifdef USE_PINIOBOX

#include "drivers/io.h"

#include "msp/msp_box.h"

#include "pg/pg_ids.h"

#include "piniobox.h"

#ifndef PINIO1_BOX
#define PINIO1_BOX PERMANENT_ID_NONE
#endif

#ifndef PINIO2_BOX
#define PINIO2_BOX PERMANENT_ID_NONE
#endif

#ifndef PINIO3_BOX
#define PINIO3_BOX PERMANENT_ID_NONE
#endif

#ifndef PINIO4_BOX
#define PINIO4_BOX PERMANENT_ID_NONE
#endif

#ifndef PINIO5_BOX
#define PINIO5_BOX PERMANENT_ID_NONE
#endif

#ifndef PINIO6_BOX
#define PINIO6_BOX PERMANENT_ID_NONE
#endif

#ifndef PINIO7_BOX
#define PINIO7_BOX PERMANENT_ID_NONE
#endif

#ifndef PINIO8_BOX
#define PINIO8_BOX PERMANENT_ID_NONE
#endif

PG_REGISTER_WITH_RESET_FN(pinioBoxConfig_t, pinioBoxConfig, PG_PINIOBOX_CONFIG, 1);

void pgResetFn_pinioBoxConfig(pinioBoxConfig_t *config)
{
    config->permanentId[0] = PINIO1_BOX;
    config->permanentId[1] = PINIO2_BOX;
    config->permanentId[2] = PINIO3_BOX;
    config->permanentId[3] = PINIO4_BOX;
    config->permanentId[4] = PINIO5_BOX;
    config->permanentId[5] = PINIO6_BOX;
    config->permanentId[6] = PINIO7_BOX;
    config->permanentId[7] = PINIO8_BOX;
}

#endif
