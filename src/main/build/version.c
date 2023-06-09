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

#include "version.h"

const char * const targetName = __TARGET__;
const char * const shortGitRevision = __REVISION__;
#if defined(__CONFIG_REVISION__)
const char * const shortConfigGitRevision = __CONFIG_REVISION__;
#endif
const char * const buildDate = __DATE__;
const char * const buildTime = __TIME__;

#ifdef BUILD_KEY
    const char * const buildKey = STR(BUILD_KEY);
#else
    const char * const buildKey = NULL;
#endif

#if defined(BUILD_KEY) && defined(RELEASE_NAME)
    const char * const releaseName = STR(RELEASE_NAME);
#else
    const char * const releaseName = NULL;
#endif
