/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_OSD_NAV_HUD

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "nav_hud.h"

PG_REGISTER_WITH_RESET_TEMPLATE(navHudConfig_t, navHudConfig, PG_NAV_HUD_CONFIG, 3);

PG_RESET_TEMPLATE(navHudConfig_t, navHudConfig,
    .mode = NAV_HUD_MODE_COMPACT,
    .orientation = NAV_HUD_ORIENTATION_NORTH_UP,
    .center = NAV_HUD_CENTER_HOME,
    .autoZoom = 1,
    .fixedScaleM = 400,
    .breadcrumbs = 1,
    .breadcrumbCount = 240,
    .breadcrumbSpacingM = 5,
    .projectedTrack = 0,
    .homeLine = 1,
    .rescueRoute = 1,
    .rescueExpand = 1,
    .showEta = 1,
    .showXtrack = 1,
    .showTargets = 1,
    .showGpsHealth = 0,
    .showSpeed = 1,
    .rangeRing = 1,
    .mission3D = 1,
);

#endif // USE_OSD_NAV_HUD
