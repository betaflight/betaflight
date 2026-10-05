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

/*
 * RpmsgFS client: files on the Linux side over the rpmsgfs protocol served by rpmsgfs-server.
 * Requests go to the server's usual address, as a restarted image hears nothing from a running server.
 * Answers are matched by cookie; a write with cookie zero gets no answer.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RPMSG

#include <openamp/rpmsg.h>

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"
#include "drivers/time.h"

#include "rpmsg_imx95.h"
#include "rpmsgfs_imx95.h"

#define RPMSGFS_OPEN            1
#define RPMSGFS_CLOSE           2
#define RPMSGFS_WRITE           4

typedef struct __attribute__((packed)) {
    uint32_t command;
    int32_t result;
    uint64_t cookie;
} rpmsgfsHeader_t;

typedef struct __attribute__((packed)) {
    rpmsgfsHeader_t header;
    int32_t flags;
    int32_t mode;
    char pathname[];
} rpmsgfsOpenMsg_t;

typedef struct __attribute__((packed)) {
    rpmsgfsHeader_t header;
    int32_t fd;
} rpmsgfsCloseMsg_t;

typedef struct __attribute__((packed)) {
    rpmsgfsHeader_t header;
    int32_t fd;
    uint32_t count;
    uint8_t buf[];
} rpmsgfsReadWriteMsg_t;

typedef struct {
    uint64_t cookie;            // zero when nothing is in flight
    bool done;
    int32_t result;
} rpmsgfsRequest_t;

// Linux exports any channel whose name starts with rpmsgfs- to rpmsgfs-server.
#define RPMSGFS_NAME            "rpmsgfs-0x401"
// The NavQ95 kernel's first dynamic address, used until an answer from the server gives its own.
#define RPMSGFS_SERVER_ADDR     0x401
#define RPMSGFS_ANSWER_MS       1000

static struct rpmsg_endpoint rpmsgfsEpt;
static uint64_t rpmsgfsLastCookie;
static rpmsgfsRequest_t rpmsgfsOpening;
static timeMs_t rpmsgfsOpenSentAt;
static uint64_t rpmsgfsWriteCookie;
static timeMs_t rpmsgfsWriteCheckAt;
static int32_t rpmsgfsWriteFailure;

static int rpmsgfsCallback(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv)
{
    UNUSED(priv);
    const rpmsgfsHeader_t *header = data;

    if (len < sizeof(*header) || !header->cookie) {
        return 0;
    }
    ept->dest_addr = src;
    if (header->cookie == rpmsgfsOpening.cookie && header->command == RPMSGFS_OPEN && !rpmsgfsOpening.done) {
        rpmsgfsOpening.result = header->result;
        rpmsgfsOpening.done = true;
    }
    if (header->cookie == rpmsgfsWriteCookie && header->command == RPMSGFS_WRITE) {
        rpmsgfsWriteCookie = 0;
        if (header->result < 0) {
            rpmsgfsWriteFailure = header->result;
        }
    }
    return 0;
}

static void rpmsgfsOpenFail(int32_t result)
{
    rpmsgfsOpening.result = result;
    rpmsgfsOpening.done = true;
}

// An open in flight when the link is lost fails, so its caller can try again.
static void rpmsgfsService(struct rpmsg_device *rdev)
{
    const bool waiting = rpmsgfsOpening.cookie && !rpmsgfsOpening.done;

    if (!rdev) {
        if (waiting) {
            rpmsgfsOpenFail(RPMSGFS_ENOTCONN);
        }
        rpmsgfsWriteCookie = 0;
        return;
    }
    // Created with no destination so that the name service announces it.
    if (!rpmsgfsEpt.rdev
        && rpmsg_create_ept(&rpmsgfsEpt, rdev, RPMSGFS_NAME, RPMSGFS_ADDR, RPMSG_ADDR_ANY,
                            rpmsgfsCallback, NULL) == RPMSG_SUCCESS) {
        rpmsgfsEpt.dest_addr = RPMSGFS_SERVER_ADDR;
        rpmsgfsWriteCheckAt = millis() + RPMSGFS_ANSWER_MS;
    }
    if (waiting && cmpTimeMs(millis(), rpmsgfsOpenSentAt) >= RPMSGFS_ANSWER_MS) {
        rpmsgfsOpenFail(RPMSGFS_ETIMEDOUT);
    }
}

bool rpmsgfsReady(void)
{
    return is_rpmsg_ept_ready(&rpmsgfsEpt);
}

void rpmsgfsStart(void)
{
    rpmsgInit();
    rpmsgAddService(rpmsgfsService);
}

static void *rpmsgfsBuffer(uint32_t *size)
{
    return rpmsgfsReady() ? rpmsg_get_tx_payload_buffer(&rpmsgfsEpt, size, 0) : NULL;
}

// Sends a message from rpmsgfsBuffer(); req is NULL for a message that wants no answer.
static bool rpmsgfsSend(void *msg, uint32_t command, uint32_t len, rpmsgfsRequest_t *req)
{
    rpmsgfsHeader_t *header = msg;

    header->command = command;
    header->result = 0;
    header->cookie = 0;
    if (req) {
        header->cookie = ++rpmsgfsLastCookie;
        req->cookie = header->cookie;
        req->done = false;
    }
    if (rpmsg_send_nocopy(&rpmsgfsEpt, msg, len) < 0) {
        rpmsg_release_tx_buffer(&rpmsgfsEpt, msg);
        if (req) {
            req->cookie = 0;
        }
        return false;
    }
    return true;
}

bool rpmsgfsOpenStart(const char *path, int32_t flags, int32_t mode)
{
    uint32_t size;
    rpmsgfsOpenMsg_t *msg;

    if (rpmsgfsOpening.cookie || !(msg = rpmsgfsBuffer(&size))) {
        return false;
    }

    const uint32_t len = sizeof(*msg) + strlen(path) + 1;
    if (len > size) {
        rpmsg_release_tx_buffer(&rpmsgfsEpt, msg);
        return false;
    }
    msg->flags = flags;
    msg->mode = mode;
    memcpy(msg->pathname, path, strlen(path) + 1);
    rpmsgfsOpenSentAt = millis();
    return rpmsgfsSend(msg, RPMSGFS_OPEN, len, &rpmsgfsOpening);
}

bool rpmsgfsOpenResult(int32_t *result)
{
    if (!rpmsgfsOpening.cookie || !rpmsgfsOpening.done) {
        return false;
    }
    rpmsgfsOpening.cookie = 0;
    *result = rpmsgfsOpening.result;
    return true;
}

uint32_t rpmsgfsWriteNoWait(int32_t fd, const void *buf, uint32_t count)
{
    uint32_t size;
    rpmsgfsReadWriteMsg_t *msg = rpmsgfsBuffer(&size);

    if (!msg) {
        return 0;
    }
    const uint32_t n = MIN(count, size - (uint32_t)sizeof(*msg));
    msg->fd = fd;
    msg->count = n;
    rpmsgCopy(msg->buf, buf, n);

    // An answer that never comes is given up when the next check is due.
    const timeMs_t now = millis();
    rpmsgfsRequest_t check = { 0 };
    const bool checking = cmpTimeMs(now, rpmsgfsWriteCheckAt) >= 0;
    if (!rpmsgfsSend(msg, RPMSGFS_WRITE, sizeof(*msg) + n, checking ? &check : NULL)) {
        return 0;
    }
    if (checking) {
        rpmsgfsWriteCookie = check.cookie;
        rpmsgfsWriteCheckAt = now + RPMSGFS_ANSWER_MS;
    }
    return n;
}

int32_t rpmsgfsWriteError(void)
{
    const int32_t result = rpmsgfsWriteFailure;
    rpmsgfsWriteFailure = 0;
    return result;
}

bool rpmsgfsCloseNoWait(int32_t fd)
{
    uint32_t size;
    rpmsgfsCloseMsg_t *msg = rpmsgfsBuffer(&size);

    if (!msg) {
        return false;
    }
    msg->fd = fd;
    return rpmsgfsSend(msg, RPMSGFS_CLOSE, sizeof(*msg), NULL);
}

#endif // USE_RPMSG
