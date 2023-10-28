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

#include "drivers/osd.h"
#include "pg/vcd.h"

#include "locales/localisation.h"

const char STR_LOCALE[] = LOCALE;         // return locale en, da, etc

char * tr_hd(char * str_sd, char * str_hd) {
#ifdef USE_OSD_HD
    if( vcdProfile()->video_system == VIDEO_SYSTEM_HD) {
        return( str_hd);
    }
#endif
    return( str_sd);
}

