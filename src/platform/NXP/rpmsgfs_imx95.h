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

// Directory on the Linux side that holds this image's files.
#define RPMSGFS_ROOT            "/betaflight"

// Open flags as RpmsgFS protocol values.
#define RPMSGFS_O_RDONLY        (1 << 0)
#define RPMSGFS_O_WRONLY        (1 << 1)
#define RPMSGFS_O_CREAT         (1 << 2)
#define RPMSGFS_O_EXCL          (1 << 3)
#define RPMSGFS_O_APPEND        (1 << 4)
#define RPMSGFS_O_TRUNC         (1 << 5)

// Linux errno values the server returns, negated.
#define RPMSGFS_EBADF           (-9)
#define RPMSGFS_EEXIST          (-17)
#define RPMSGFS_ENOTCONN        (-107)
#define RPMSGFS_ETIMEDOUT       (-110)

// Starts the client; it is ready as soon as the link is up.
void rpmsgfsStart(void);
bool rpmsgfsReady(void);

// Non-blocking calls. rpmsgfsOpenResult() collects the fd or negative errno, RPMSGFS_ETIMEDOUT after a second.
// Closes get no answer; one write a second asks for one, and rpmsgfsWriteError() returns its error once, or 0.
bool rpmsgfsOpenStart(const char *path, int32_t flags, int32_t mode);
bool rpmsgfsOpenResult(int32_t *result);
uint32_t rpmsgfsWriteNoWait(int32_t fd, const void *buf, uint32_t count);
int32_t rpmsgfsWriteError(void);
bool rpmsgfsCloseNoWait(int32_t fd);
