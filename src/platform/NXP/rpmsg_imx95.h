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

#pragma once

#include <stdbool.h>
#include <stdint.h>

struct rpmsg_device;

// Fixed endpoint addresses, so a restarted image serves the channels Linux kept.
#define RPMSG_TTY_ADDR          0x400
#define RPMSGFS_ADDR            0x401

// Called from every rpmsgPoll(); rdev is NULL until Linux attaches and again once the link is lost.
typedef void rpmsgServiceFn(struct rpmsg_device *rdev);

void rpmsgInit(void);
void rpmsgCopy(void *dst, const void *src, uint32_t len);
bool rpmsgAddService(rpmsgServiceFn *service);
// Runs the link from thread mode; returns the rpmsg device once Linux has attached, else NULL.
struct rpmsg_device *rpmsgPoll(void);
