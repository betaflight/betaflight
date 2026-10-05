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
 * VIRTUAL blackbox device: logs go to files on the Linux side over RpmsgFS.
 * The logger only fills a ring; the rpmsg poll in the serial task sends it and opens and closes the files.
 * Data that finds the ring full is dropped; a log lost by a restarted rpmsgfs-server is reopened for append.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_RPMSG) && defined(USE_BLACKBOX_VIRTUAL)

#include "blackbox/blackbox_virtual.h"
#include "build/debug.h"
#include "common/printf.h"
#include "common/utils.h"

#include "rpmsg_imx95.h"
#include "rpmsgfs_imx95.h"

#define BLACKBOX_FILE_RING_SIZE         (16 * 1024)
#define BLACKBOX_FILE_SENDS_PER_POLL    16
// One full RpmsgFS write: Linux's 512 B rpmsg buffer less the rpmsg and write headers.
#define BLACKBOX_FILE_SEND_MIN          472

typedef enum {
    BLACKBOX_FILE_IDLE,
    BLACKBOX_FILE_OPEN_WANTED,
    BLACKBOX_FILE_OPENING,
    BLACKBOX_FILE_OPEN,
} blackboxFileState_e;

static uint8_t blackboxFileRing[BLACKBOX_FILE_RING_SIZE];
static uint32_t blackboxFileHead;
static uint32_t blackboxFileTail;
static blackboxFileState_e blackboxFileState;
static int32_t blackboxFileFd = -1;
static bool blackboxFileCloseWanted;
static int32_t blackboxFileNumber;
static bool blackboxFileReopening;
static uint32_t blackboxFileDropped;
static uint32_t blackboxFileReopens;

static uint32_t blackboxFileUsed(void)
{
    return (blackboxFileHead - blackboxFileTail) % BLACKBOX_FILE_RING_SIZE;
}

static bool blackboxFileOpenStart(int32_t number, int32_t flags)
{
    char name[40];
    tfp_sprintf(name, RPMSGFS_ROOT "/LOG%05u.BFL", (unsigned)number);
    return rpmsgfsOpenStart(name, flags, 0644);
}

// The open log has no fd while it is being reopened.
static void blackboxFileReopen(void)
{
    int32_t result;

    if (blackboxFileFd >= 0 && rpmsgfsWriteError() == RPMSGFS_EBADF) {
        blackboxFileFd = -1;
        blackboxFileReopens++;
        DEBUG_SET(DEBUG_BLACKBOX_OUTPUT, 4, blackboxFileReopens);  //!< Log Reopens
    }
    if (blackboxFileFd < 0 && !blackboxFileReopening) {
        blackboxFileReopening = blackboxFileOpenStart(blackboxFileNumber, RPMSGFS_O_WRONLY | RPMSGFS_O_APPEND);
    }
    if (blackboxFileReopening && rpmsgfsOpenResult(&result)) {
        blackboxFileReopening = false;
        blackboxFileFd = result < 0 ? -1 : result;
    }
}

// Only full messages go out while logging, the rest once the log closes.
// A log open when the link is lost is given up until the next log starts.
static void blackboxFileService(struct rpmsg_device *rdev)
{
    if (rdev && blackboxFileState == BLACKBOX_FILE_OPEN_WANTED
        && blackboxFileOpenStart(blackboxFileNumber + 1, RPMSGFS_O_WRONLY | RPMSGFS_O_CREAT | RPMSGFS_O_EXCL)) {
        blackboxFileState = BLACKBOX_FILE_OPENING;
    }
    if (rdev && blackboxFileState == BLACKBOX_FILE_OPEN) {
        blackboxFileReopen();
    }
    if (!rdev && blackboxFileState == BLACKBOX_FILE_OPEN) {
        int32_t result;
        // The client fails a reopen in flight when the link goes.
        if (blackboxFileReopening) {
            rpmsgfsOpenResult(&result);
        }
        blackboxFileReopening = false;
        blackboxFileFd = -1;
        blackboxFileTail = blackboxFileHead;
        blackboxFileCloseWanted = false;
        blackboxFileState = BLACKBOX_FILE_IDLE;
    }
    if (!rdev || blackboxFileFd < 0) {
        return;
    }
    const uint32_t sendMin = blackboxFileCloseWanted ? 1 : BLACKBOX_FILE_SEND_MIN;
    for (unsigned i = 0; i < BLACKBOX_FILE_SENDS_PER_POLL && blackboxFileUsed() >= sendMin; i++) {
        const uint32_t run = (blackboxFileHead >= blackboxFileTail) ? blackboxFileHead - blackboxFileTail
                                                                    : BLACKBOX_FILE_RING_SIZE - blackboxFileTail;
        const uint32_t sent = rpmsgfsWriteNoWait(blackboxFileFd, &blackboxFileRing[blackboxFileTail], run);
        if (!sent) {
            return;
        }
        blackboxFileTail = (blackboxFileTail + sent) % BLACKBOX_FILE_RING_SIZE;
    }
    if (blackboxFileCloseWanted && !blackboxFileUsed() && rpmsgfsCloseNoWait(blackboxFileFd)) {
        blackboxFileFd = -1;
        blackboxFileState = BLACKBOX_FILE_IDLE;
    }
}

bool blackboxVirtualOpen(void)
{
    rpmsgfsStart();
    return rpmsgAddService(blackboxFileService);
}

FAST_CODE void blackboxVirtualWrite(const uint8_t *buffer, uint32_t len)
{
    if (blackboxFileState != BLACKBOX_FILE_OPEN || blackboxFileCloseWanted) {
        return;
    }
    while (len--) {
        const uint32_t next = (blackboxFileHead + 1) % BLACKBOX_FILE_RING_SIZE;
        if (next == blackboxFileTail) {
            blackboxFileDropped += len + 1;
            DEBUG_SET(DEBUG_BLACKBOX_OUTPUT, 2, blackboxFileDropped);  //!< Dropped Bytes
            return;
        }
        blackboxFileRing[blackboxFileHead] = *buffer++;
        blackboxFileHead = next;
    }
}

FAST_CODE void blackboxVirtualPutChar(uint8_t value)
{
    blackboxVirtualWrite(&value, 1);
}

bool blackboxVirtualFlush(void)
{
    return blackboxFileUsed() == 0;
}

// The open is sent from the poll, so the flight loop never sends on the link.
// Each existing log costs one open round trip to find the next number.
bool blackboxVirtualBeginLog(void)
{
    int32_t result;

    switch (blackboxFileState) {
    case BLACKBOX_FILE_IDLE:
        if (blackboxFileFd < 0) {
            blackboxFileState = BLACKBOX_FILE_OPEN_WANTED;
        }
        return false;

    case BLACKBOX_FILE_OPEN_WANTED:
        return false;

    case BLACKBOX_FILE_OPENING:
        if (!rpmsgfsOpenResult(&result)) {
            return false;
        }
        blackboxFileState = BLACKBOX_FILE_IDLE;
        if (result == RPMSGFS_EEXIST || result >= 0) {
            blackboxFileNumber++;
        }
        if (result < 0) {
            return false;
        }
        blackboxFileFd = result;
        // Discard a write error left from the previous log.
        rpmsgfsWriteError();
        blackboxFileHead = blackboxFileTail = 0;
        blackboxFileCloseWanted = false;
        blackboxFileState = BLACKBOX_FILE_OPEN;
        return true;

    case BLACKBOX_FILE_OPEN:
    default:
        // A log still closing is finished before the next one opens.
        return !blackboxFileCloseWanted;
    }
}

bool blackboxVirtualEndLog(void)
{
    if (blackboxFileState == BLACKBOX_FILE_OPEN) {
        blackboxFileCloseWanted = true;
    }
    return true;
}

void blackboxVirtualClose(void)
{
    blackboxVirtualEndLog();
}

int32_t blackboxVirtualLogFileNumber(void)
{
    return blackboxFileNumber;
}

#endif // USE_RPMSG && USE_BLACKBOX_VIRTUAL
