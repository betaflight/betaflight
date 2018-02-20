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

#include "platform.h"

#ifdef USE_PINIOBOX

#include "pg/pg_ids.h"
#include "piniobox.h"
#include "drivers/io.h"
#include "interface/msp_box.h"

PG_REGISTER_WITH_RESET_TEMPLATE(pinioBoxConfig_t, pinioBoxConfig, PG_PINIOBOX_CONFIG, 1);

PG_RESET_TEMPLATE(pinioBoxConfig_t, pinioBoxConfig,
    { PERMANENT_ID_NONE, PERMANENT_ID_NONE, PERMANENT_ID_NONE, PERMANENT_ID_NONE }
);
#endif
