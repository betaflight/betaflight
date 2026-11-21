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

#include <stdint.h>
#include <string.h>

#include <string>
#include <vector>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"

    #include "blackbox/blackbox.h"
    #include "blackbox/blackbox_io.h"

    #include "drivers/sdcard.h"

    #include "io/asyncfatfs/asyncfatfs.h"
    #include "io/asyncfatfs/fat_standard.h"
    #include "io/serial.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// A pretend SD card: the log files in its "logs" directory, in 8.3 form ("LOG00001BFL")
static std::vector<std::string> cardLogs;
// Every file blackbox has asked to open, as it named it ("LOG00001.BFL")
static std::vector<std::string> opened;
static size_t findIndex;
static fatDirectoryEntry_t foundEntry;
static int fakeFile;

static bool beginLog(void)
{
    // Keep calling until the log is open, as blackbox does
    for (int i = 0; i < 20; i++) {
        if (blackboxDeviceBeginLog()) {
            return true;
        }
    }
    return false;
}

TEST(BlackboxSdcardTest, LogNumbering)
{
    blackboxConfigMutable()->device = BLACKBOX_DEVICE_SDCARD;

    // The usual numbering: an empty card starts at LOG00001
    blackboxConfigMutable()->log_number_start = 1;
    EXPECT_TRUE(beginLog());
    ASSERT_EQ(1u, opened.size());
    EXPECT_EQ("LOG00001.BFL", opened.back());
    EXPECT_EQ(1, blackboxGetLogFileNo());
    EXPECT_TRUE(blackboxDeviceEndLog(true));

    // Raising the start skips ahead, and the number isn't cut down to 8 bits
    blackboxConfigMutable()->log_number_start = 12300;
    EXPECT_TRUE(beginLog());
    EXPECT_EQ("LOG12300.BFL", opened.back());
    EXPECT_EQ(12300, blackboxGetLogFileNo());
    EXPECT_TRUE(blackboxDeviceEndLog(true));

    // The next log carries on from the last
    EXPECT_TRUE(beginLog());
    EXPECT_EQ("LOG12301.BFL", opened.back());
    EXPECT_TRUE(blackboxDeviceEndLog(true));

    // Lowering the start never takes the numbering backwards
    blackboxConfigMutable()->log_number_start = 1;
    EXPECT_TRUE(beginLog());
    EXPECT_EQ("LOG12302.BFL", opened.back());
    EXPECT_TRUE(blackboxDeviceEndLog(true));

    // The last name a log can have: still logging, so the card isn't full
    blackboxConfigMutable()->log_number_start = BLACKBOX_LOG_NUMBER_MAX;
    EXPECT_TRUE(beginLog());
    EXPECT_EQ("LOG99999.BFL", opened.back());
    EXPECT_EQ(BLACKBOX_LOG_NUMBER_MAX, blackboxGetLogFileNo());
    EXPECT_FALSE(isBlackboxDeviceFull());
    EXPECT_TRUE(blackboxDeviceEndLog(true));

    // With no name left the card counts as full, and no file is opened, so LOG99999.BFL is left alone
    EXPECT_TRUE(isBlackboxDeviceFull());
    const size_t openedBefore = opened.size();
    EXPECT_FALSE(beginLog());
    EXPECT_EQ(openedBefore, opened.size());
    EXPECT_EQ(-1, blackboxGetLogFileNo());
}

// STUBS

extern "C" {

PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);

uint8_t debugMode;
int16_t debug[DEBUG16_VALUE_COUNT];
uint32_t targetPidLooptime;

afatfsFilesystemState_e afatfs_getFilesystemState(void) { return AFATFS_FILESYSTEM_STATE_READY; }
bool afatfs_isFull(void) { return false; }
bool afatfs_sectorCacheInSync(void) { return true; }
uint32_t afatfs_getFreeBufferSpace(void) { return 4096; }
bool afatfs_flush(void) { return true; }
void afatfs_fputc(afatfsFilePtr_t, uint8_t) {}
uint32_t afatfs_fwrite(afatfsFilePtr_t, const uint8_t *, uint32_t len) { return len; }
bool afatfs_chdir(afatfsFilePtr_t) { return true; }
bool afatfs_fclose(afatfsFilePtr_t, afatfsCallback_t) { return true; }
bool afatfs_funlink(afatfsFilePtr_t, afatfsCallback_t) { return true; }

bool afatfs_mkdir(const char *, afatfsFileCallback_t complete)
{
    complete((afatfsFilePtr_t)&fakeFile);
    return true;
}

void afatfs_findFirst(afatfsFilePtr_t, afatfsFinder_t *)
{
    findIndex = 0;
}

afatfsOperationStatus_e afatfs_findNext(afatfsFilePtr_t, afatfsFinder_t *, fatDirectoryEntry_t **dirEntry)
{
    // Each log on the card in turn, then a zeroed entry: the end of the directory
    memset(&foundEntry, 0, sizeof(foundEntry));
    if (findIndex < cardLogs.size()) {
        memcpy(foundEntry.filename, cardLogs[findIndex].c_str(), FAT_FILENAME_LENGTH);
    }
    findIndex++;
    *dirEntry = &foundEntry;
    return AFATFS_OPERATION_SUCCESS;
}

void afatfs_findLast(afatfsFilePtr_t) {}

bool afatfs_fopen(const char *filename, const char *, afatfsFileCallback_t complete)
{
    opened.push_back(filename);

    std::string name(filename);
    name.erase(name.find('.'), 1);
    cardLogs.push_back(name);

    complete((afatfsFilePtr_t)&fakeFile);
    return true;
}

bool sdcard_isInserted(void) { return true; }
bool sdcard_isFunctional(void) { return true; }

const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000}; // see baudRate_e

void mspSerialAllocatePorts(void) {}
void mspSerialReleasePortIfAllocated(serialPort_t *) {}
void serialWrite(serialPort_t *, uint8_t) {}
uint32_t serialTxBytesFree(const serialPort_t *) { return 0; }
bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }
serialPort_t *findSharedSerialPort(uint16_t, serialPortFunction_e) { return NULL; }
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) { return NULL; }
void closeSerialPort(serialPort_t *) {}
portSharing_e determinePortSharing(serialPortIdentifier_e, serialPortFunction_e) { return PORTSHARING_UNUSED; }
uint32_t millis(void) { return 0; }

}
