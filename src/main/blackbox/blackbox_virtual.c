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
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <dirent.h>
#include <common/maths.h>

#define LOGFILE_PREFIX "LOG"
#define LOGFILE_SUFFIX "BFL"

static FILE *blackboxVirtualFile = NULL;
static int32_t largestLogFileNumber = 0;
bool blackboxVirtualOpen(void)
{
    DIR *dir = opendir(".");
    struct dirent *file;
    while ((file = readdir(dir)) != NULL) {
        if (strncmp(file->d_name, LOGFILE_PREFIX, strlen(LOGFILE_PREFIX)) == 0
                    && strncmp(file->d_name + 8, LOGFILE_SUFFIX, strlen(LOGFILE_SUFFIX)) == 0) {

            char logSequenceNumberString[6];
            memcpy(logSequenceNumberString, file->d_name + 3, 5);
            logSequenceNumberString[5] = '\0';
            largestLogFileNumber = MAX((int32_t)atoi(logSequenceNumberString), largestLogFileNumber);
        }
    }
    closedir(dir);
    return true;
}

void blackboxVirtualPutChar(uint8_t value)
{
    if (blackboxVirtualFile != NULL) {
        fputc(value, blackboxVirtualFile);
    }
}

void blackboxVirtualWrite(const uint8_t *buffer, uint32_t len)
{
    if (blackboxVirtualFile != NULL) {
        fwrite(buffer, len, 1, blackboxVirtualFile);
    }
}

bool blackboxVirtualFlush(void)
{
    if (blackboxVirtualFile != NULL) {
        fflush(blackboxVirtualFile);
        return true;
    } else {
        return false;
    }

}

bool blackboxVirtualBeginLog(void)
{
    if (blackboxVirtualFile != NULL) {
        return false;
    }

    int32_t remainder = largestLogFileNumber + 1;
    char filename[] = LOGFILE_PREFIX "00000." LOGFILE_SUFFIX;

    for (int i = 7; i >= 3; i--) {
        filename[i] = (remainder % 10) + '0';
        remainder /= 10;
    }

    blackboxVirtualFile = fopen(filename, "w");
    if (blackboxVirtualFile != NULL) {
        largestLogFileNumber++;
    }
    return blackboxVirtualFile != NULL;
}

bool blackboxVirtualEndLog(void)
{
    if (blackboxVirtualFile != NULL) {
        fclose(blackboxVirtualFile);
        blackboxVirtualFile = NULL;
    }
    return true;
}

void blackboxVirtualClose(void)
{
    blackboxVirtualEndLog();
}

uint32_t blackboxVirtualLogFileNumber(void)
{
    return largestLogFileNumber;
}
