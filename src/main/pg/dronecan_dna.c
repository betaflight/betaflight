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

#if ENABLE_DRONECAN_DNA

#include "pg/dronecan_dna.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

// Reset to all-empty (every nodeId 0). Zero-init via the default template.
PG_REGISTER_WITH_RESET_TEMPLATE(dronecanDnaConfig_t, dronecanDnaConfig, PG_DRONECAN_DNA_CONFIG, 0);

PG_RESET_TEMPLATE(dronecanDnaConfig_t, dronecanDnaConfig, );

#endif // ENABLE_DRONECAN_DNA
