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

#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "common/maths.h"
#include "common/printf.h"

#include "io/asyncfatfs/asyncfatfs.h"

#include "osd/osd.h"

enum {
    FILE_STATE_NONE = 0,
    FILE_STATE_WRITING,
    FILE_STATE_READING,
    FILE_STATE_FAILED,
    FILE_STATE_COMPLETE,
};

uint8_t fileState = FILE_STATE_NONE;

union {
    int32_t i;
    unsigned char b[4];
} mahDrawn;

bool isBatteryContinueActive = false;

static void fileCloseContinue(void)
{
    if (fileState != FILE_STATE_FAILED) {
        fileState = FILE_STATE_COMPLETE;
    }
}

static void fileWriteContinue(afatfsFilePtr_t file)
{
    if (!file) {
        fileState = FILE_STATE_FAILED;
        return;
    }

    uint32_t bytesToWrite = 4;

    uint32_t totalBytesWritten = 0;
    uint32_t bytesWritten = 0;
    bool success;

    do {
        bytesWritten = afatfs_fwrite(file, &mahDrawn.b[totalBytesWritten], bytesToWrite - totalBytesWritten);
        totalBytesWritten += bytesWritten;
        success = (totalBytesWritten == bytesToWrite);

        afatfs_poll();
    } while (!success && afatfs_getLastError() == AFATFS_ERROR_NONE);

    if (!success) {
        fileState = FILE_STATE_FAILED;
    }

    while (!afatfs_fclose(file, fileCloseContinue)) {
        afatfs_poll();
    }
}

static void fileReadContinue(afatfsFilePtr_t file)
{
    if (!file) {
        fileState = FILE_STATE_FAILED;
        return;
    }

    uint32_t bytesToRead = 4;

    uint32_t totalBytesRead = 0;
    uint32_t bytesRead = 0;
    bool success;

    if (!afatfs_feof(file)) {

        do {
            bytesRead = afatfs_fread(file, &mahDrawn.b[totalBytesRead], bytesToRead - totalBytesRead);
            totalBytesRead += bytesRead;
            success = (totalBytesRead == bytesToRead);

            afatfs_poll();
        } while (!success && afatfs_getLastError() == AFATFS_ERROR_NONE);
    }

    if (!success) {
        fileState = FILE_STATE_FAILED;
    }

    while (!afatfs_fclose(file, fileCloseContinue)) {
        afatfs_poll();
    }

    return;
}

bool batContinueWriteMAh(int32_t mah)
{
    fileState = FILE_STATE_WRITING;

    mahDrawn.i = mah;

    // Go to the root directory
    if (!afatfs_chdir(NULL)) {
        return false;
    }

    bool result = afatfs_fopen("DRAWN.MAH", "w+", fileWriteContinue);
    if (!result) {
        return false;
    }

    while (fileState == FILE_STATE_WRITING) {
        afatfs_poll();
    }

    while (!afatfs_flush()) {
        afatfs_poll();
    };

    return (fileState == FILE_STATE_COMPLETE);
}

int32_t batContinueReadMAh()
{
    if (mahDrawn.i > 0) {
        return mahDrawn.i;
    }

    if (fileState == FILE_STATE_FAILED) {
        return 0;
    }

    fileState = FILE_STATE_READING;

    bool result = afatfs_fopen("DRAWN.MAH", "r", fileReadContinue);
    if (!result) {
        return 0;
    }

    while (fileState == FILE_STATE_READING) {
        afatfs_poll();
    }

    return mahDrawn.i;
}
