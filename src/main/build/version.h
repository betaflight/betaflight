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

#pragma once

#include "common/utils.h"

#define FC_FIRMWARE_NAME            "Betaflight"
#define FC_FIRMWARE_IDENTIFIER      "BTFL"

#define FC_CALVER_BASE_YEAR 2000

// The year the release is made
#define FC_VERSION_YEAR             2025
// The month the release is made
#define FC_VERSION_MONTH            12
// Increment when a bug-fix release is made (0 for initial YYYY.MM.X release)
#define FC_VERSION_PATCH_LEVEL      0
// Optional suffix for pre-releases (alpha, beta, rc1, etc). Empty string for final releases.
#define FC_VERSION_SUFFIX           "beta"

/* load the version from the makefile if it is defined otherwise default */
#ifdef __FC_VERSION__
#define FC_VERSION_STRING __FC_VERSION__
#else
#define FC_VERSION_STRING STR(FC_VERSION_YEAR) "." STR(FC_VERSION_MONTH) "." STR(FC_VERSION_PATCH_LEVEL)
#endif

extern const char* const targetName;

#define GIT_SHORT_REVISION_LENGTH   7 // lower case hexadecimal digits.
extern const char* const shortGitRevision;

#define GIT_SHORT_CONFIG_REVISION_LENGTH   7 // lower case hexadecimal digits.
extern const char* const shortConfigGitRevision;

#define BUILD_DATE_LENGTH           11
extern const char* const buildDate;  // "MMM DD YYYY" MMM = Jan/Feb/...

#define BUILD_TIME_LENGTH           8
extern const char* const buildTime;  // "HH:MM:SS"

#define MSP_API_VERSION_STRING STR(API_VERSION_MAJOR) "." STR(API_VERSION_MINOR)

extern const char* const buildKey;
extern const char* const releaseName;

STATIC_ASSERT(FC_VERSION_YEAR >= FC_CALVER_BASE_YEAR && FC_VERSION_YEAR <= FC_CALVER_BASE_YEAR + 255, FC_VERSION_YEAR_range);
STATIC_ASSERT(FC_VERSION_MONTH >= 1 && FC_VERSION_MONTH <= 12, FC_VERSION_MONTH_range);
STATIC_ASSERT(FC_VERSION_PATCH_LEVEL >= 0 && FC_VERSION_PATCH_LEVEL <= 255, FC_VERSION_PATCH_LEVEL_range);
STATIC_ASSERT(sizeof(FC_VERSION_STRING) - 1 <= 255, FC_VERSION_STRING_too_long);
