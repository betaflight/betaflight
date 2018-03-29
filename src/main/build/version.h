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

#pragma once

#include "common/utils.h"

#define FC_FIRMWARE_NAME            "ButterFlight"
#define FC_VERSION_MAJOR            3  // increment when a major release is made (big new feature, etc)
#define FC_VERSION_MINOR            5  // increment when a minor release is made (small new feature, change etc)
#define FC_VERSION_PATCH_LEVEL      0  // increment when a bug is fixed

#if defined(USE_GYRO_IMUF9001)
    #define IMUF_VERSION_MAJOR       1
    #define IMUF_VERSION_MINOR       0
    #define IMUF_VERSION_PATCH_LEVEL 4
    #define FC_VERSION_STRING STR(FC_VERSION_MAJOR) "." STR(FC_VERSION_MINOR) "." STR(FC_VERSION_PATCH_LEVEL) " | IMUF: " STR(IMUF_VERSION_MAJOR) "." STR(IMUF_VERSION_MINOR) "." STR(IMUF_VERSION_PATCH_LEVEL)
#else
    #define FC_VERSION_STRING STR(FC_VERSION_MAJOR) "." STR(FC_VERSION_MINOR) "." STR(FC_VERSION_PATCH_LEVEL)
#endif //USE_GYRO_IMUF9001

// Uncomment this for release / maintenance branches, or use `OPTIONS=RELEASE_BUILD`
//#define RELEASE_BUILD

extern const char* const targetName;

#define GIT_SHORT_REVISION_LENGTH   7 // lower case hexadecimal digits.
extern const char* const shortGitRevision;

#define BUILD_DATE_LENGTH 11
extern const char* const buildDate;  // "MMM DD YYYY" MMM = Jan/Feb/...

#define BUILD_TIME_LENGTH 8
extern const char* const buildTime;  // "HH:MM:SS"

#define MSP_API_VERSION_STRING STR(API_VERSION_MAJOR) "." STR(API_VERSION_MINOR)
