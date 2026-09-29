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

#ifdef USE_RADAR

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/radar.h"

// Defaults follow INAV's osd_hud_radar_* and osd_camera_* settings
PG_REGISTER_WITH_RESET_TEMPLATE(radarConfig_t, radarConfig, PG_RADAR_CONFIG, 0);

PG_RESET_TEMPLATE(radarConfig_t, radarConfig,
    .peerDisplayTimeS = 3,
    .hudMaxPeers = 4,
    .hudRangeMinM = 3,
    .hudRangeMaxM = 4000,
    .hudAltTimeS = 3,
    .hudDistTimeS = 3,
    .cameraFovH = 135,
    .cameraFovV = 85,
    .cameraUptilt = 0,
    .hudMarginH = 3,
    .hudMarginV = 3,
);

#endif // USE_RADAR
