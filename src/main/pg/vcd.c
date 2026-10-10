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

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/osd.h"

#include "vcd.h"

// Make it obvious on the configurator that the FC doesn't support HD
#ifndef DEFAULT_VIDEO_SYSTEM
#ifdef USE_OSD_HD
#define DEFAULT_VIDEO_SYSTEM VIDEO_SYSTEM_HD
#else
#define DEFAULT_VIDEO_SYSTEM VIDEO_SYSTEM_AUTO
#endif
#endif

// no template required since defaults are zero
PG_REGISTER_WITH_RESET_FN(vcdProfile_t, vcdProfile, PG_VCD_CONFIG, 0);

void pgResetFn_vcdProfile(vcdProfile_t *vcdProfile)
{
    vcdProfile->video_system = DEFAULT_VIDEO_SYSTEM;
}
