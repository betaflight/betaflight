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

 #include "platform.h"

 
 #ifdef USE_ALTITUDE_LIMIT
 
 #include "flight/alt_limit.h"
 
 #include "pg/pg.h"
 #include "pg/pg_ids.h"
 
 #include "alt_limit.h"


 PG_REGISTER_WITH_RESET_TEMPLATE(altLimitConfig_t, altLimitConfig, PG_ALTLIMIT_CONFIG, 0);
 
 PG_RESET_TEMPLATE(altLimitConfig_t, altLimitConfig,
     .ceiling = 120, // maximum allowed altitude in meters (above starting altitude)
     .buffer = 15, // hight under ceiling where throttle will be scaled down 
     .active = false, // set to true to enforce altitude limit
 );
 #endif
 