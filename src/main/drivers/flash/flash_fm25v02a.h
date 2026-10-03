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
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct flashDevice_s;

#define FM25V02A_DEVICE_ID_LENGTH          9
#define FM25V02A_JEDEC_ID                  0xC22208
#define FM25V02A_JEDEC_ID_EXTENDED_TEMP    0xC22248

#define FM25V02A_TOTAL_SIZE                (32 * 1024)
#define FM25V02A_LOGICAL_PAGE_SIZE         256
#define FM25V02A_LOGICAL_SECTOR_SIZE       2048

bool fm25v02a_identify(struct flashDevice_s *fdevice, const uint8_t *deviceId, size_t deviceIdLength);
