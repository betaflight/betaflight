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

/*
 * Winbond W25M series stacked die flash driver.
 * Handles homogeneous stack of identical dies by calling die drivers.
 * 
 * Author: jflyper
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#ifdef USE_FLASH_W25M

#include "common/maths.h"
#include "drivers/bus_spi.h"
#include "drivers/flash.h"
#include "drivers/flash_impl.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "flash_m25p16.h"
#include "flash_w25m.h"

#include "pg/flash.h"

#define W25M_INSTRUCTION_SOFTWARE_DIE_SELECT         0xC2

#define JEDEC_ID_WINBOND_W25M512                     0xEF7119 // W25Q256 x 2

static const flashVTable_t w25m_vTable;

#define MAX_DIE_COUNT 2

static flashDevice_t dieDevice[MAX_DIE_COUNT];

static int dieCount;
static uint32_t dieSize;

static void w25m_dieSelect(busDevice_t *busdev, int die)
{
    static int activeDie = -1;

    if (activeDie == die) {
        return;
    }

    uint8_t command[2] = { W25M_INSTRUCTION_SOFTWARE_DIE_SELECT, die };

#ifdef SPI_BUS_TRANSACTION
    spiBusTransactionTransfer(busdev, command, NULL, 2);
#else
    spiBusTransfer(busdev, command, NULL, 2);
#endif

    activeDie = die;
}

static bool w25m_isReady(flashDevice_t *fdevice)
{
    UNUSED(fdevice);

    for (int die = 0 ; die < dieCount ; die++) {
        if (dieDevice[die].couldBeBusy) {
            w25m_dieSelect(fdevice->busdev, die);
            if (!dieDevice[die].vTable->isReady(&dieDevice[die])) {
                return false;
            }
        }
    }

    return true;
}

static bool w25m_waitForReady(flashDevice_t *fdevice, uint32_t timeoutMillis)
{
    uint32_t time = millis();
    while (!w25m_isReady(fdevice)) {
        if (millis() - time > timeoutMillis) {
            return false;
        }
    }

    return true;
}

bool w25m_detect(flashDevice_t *fdevice, uint32_t chipID)
{

    switch (chipID) {
    case JEDEC_ID_WINBOND_W25M512:
        // W25Q256 x 2
        dieCount = 2;

        for (int die = 0 ; die < dieCount ; die++) {
            w25m_dieSelect(fdevice->busdev, die);
            dieDevice[die].busdev = fdevice->busdev;
            m25p16_detect(&dieDevice[die], JEDEC_ID_WINBOND_W25Q256);
        }

        fdevice->geometry.flashType = FLASH_TYPE_NOR;
        break;

    default:
        // Not a valid W25M series device
        fdevice->geometry.sectors = 0;
        fdevice->geometry.pagesPerSector = 0;
        fdevice->geometry.sectorSize = 0;
        fdevice->geometry.totalSize = 0;
        return false;
    }

    fdevice->geometry.sectors = dieDevice[0].geometry.sectors;
    fdevice->geometry.sectorSize = dieDevice[0].geometry.sectorSize;
    fdevice->geometry.pagesPerSector = dieDevice[0].geometry.pagesPerSector;
    fdevice->geometry.pageSize = dieDevice[0].geometry.pageSize;
    dieSize = dieDevice[0].geometry.totalSize;
    fdevice->geometry.totalSize = dieSize * dieCount;
    fdevice->vTable = &w25m_vTable;

    return true;
}

void w25m_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    int dieNumber = address / dieSize;

    w25m_dieSelect(fdevice->busdev, dieNumber);

    dieDevice[dieNumber].vTable->eraseSector(&dieDevice[dieNumber], address % dieSize);
}

void w25m_eraseCompletely(flashDevice_t *fdevice)
{
    for (int dieNumber = 0 ; dieNumber < dieCount ; dieNumber++) {
        w25m_dieSelect(fdevice->busdev, dieNumber);
        dieDevice[dieNumber].vTable->eraseCompletely(&dieDevice[dieNumber]);
    }
}

static uint32_t currentWriteAddress;
static int currentWriteDie;

void w25m_pageProgramBegin(flashDevice_t *fdevice, uint32_t address)
{
    UNUSED(fdevice);

    currentWriteDie = address / dieSize;
    w25m_dieSelect(fdevice->busdev, currentWriteDie);
    currentWriteAddress = address % dieSize;
    dieDevice[currentWriteDie].vTable->pageProgramBegin(&dieDevice[currentWriteDie], currentWriteAddress);
}

void w25m_pageProgramContinue(flashDevice_t *fdevice, const uint8_t *data, int length)
{
    UNUSED(fdevice);

    dieDevice[currentWriteDie].vTable->pageProgramContinue(&dieDevice[currentWriteDie], data, length);
}

void w25m_pageProgramFinish(flashDevice_t *fdevice)
{
    UNUSED(fdevice);

    dieDevice[currentWriteDie].vTable->pageProgramFinish(&dieDevice[currentWriteDie]);
}

void w25m_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, int length)
{
    w25m_pageProgramBegin(fdevice, address);

    w25m_pageProgramContinue(fdevice, data, length);

    w25m_pageProgramFinish(fdevice);
}

int w25m_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, int length)
{
    int rlen; // remaining length
    int tlen; // transfer length for a round
    int rbytes;

    // Divide a read that spans multiple dies into two.
    // The loop is executed twice at the most for decent 'length'.

    for (rlen = length; rlen; rlen -= tlen) {
        int dieNumber = address / dieSize;
        uint32_t dieAddress = address % dieSize;
        tlen = MIN(dieAddress + rlen, dieSize) - dieAddress;

        w25m_dieSelect(fdevice->busdev, dieNumber);

        rbytes = dieDevice[dieNumber].vTable->readBytes(&dieDevice[dieNumber], dieAddress, buffer, tlen);

        if (!rbytes) {
            return 0;
        }

        address += tlen;
        buffer += tlen;
    }
    return length;
}

const flashGeometry_t* w25m_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

static const flashVTable_t w25m_vTable = {
    .isReady = w25m_isReady,
    .waitForReady = w25m_waitForReady,
    .eraseSector = w25m_eraseSector,
    .eraseCompletely = w25m_eraseCompletely,
    .pageProgramBegin = w25m_pageProgramBegin,
    .pageProgramContinue = w25m_pageProgramContinue,
    .pageProgramFinish = w25m_pageProgramFinish,
    .pageProgram = w25m_pageProgram,
    .readBytes = w25m_readBytes,
    .getGeometry = w25m_getGeometry,
};
#endif
