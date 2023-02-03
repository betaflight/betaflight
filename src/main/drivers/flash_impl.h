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
 * Author: jflyper
 */

/*
 * Each flash chip should:
 *
 * * expose a public `identify` method.
 *   - return true if the driver supports the passed JEDEC ID and false otherwise.
 *   - configure the `geometry` member of the flashDevice_t or set all `geometry` members to 0 if driver doesn't support the JEDEC ID.
 *   - configure the `vTable` member, with an appropriate API.
 *   - configure remaining flashDevice_t members, as appropriate.
 * * not reconfigure the bus or flash chip when in memory mapped mode.
 *   - the firmware is free to do whatever it wants when memory mapped mode is disabled
 *   - when memory mapped mode is restored, e.g. after saving config to external flash, it should be in
 *     the same state that firmware found it in before the firmware disabled memory mapped mode.
 *
 * When memory mapped mode is disabled the following applies to all flash chip drivers uses in a memory mapped system:
 *   - do not call any methods or use data from the flash chip.  i.e. memory mapped code/data is INACCESSIBLE.
 *     i.e. when saving the config, *ALL* the code to erase a block and write data should be in RAM,
 *     this includes any `delay` methods.
 *   - not require the use of use any ISRs - interrupts are disabled during flash access when memory mapped mode is disabled.
 *
 * When compiling a driver for use in a memory mapped flash system the following applies:
 *   - the vTable must live in RAM so it can be used when memory mapped mode is disabled.
 *   - other constant data structures that usually live in flash memory must be stored in RAM.
 *   - methods used to erase sectors, write data and read data much live in RAM.
 */
#pragma once

#include "drivers/bus.h"
#include "drivers/dma.h"

struct flashVTable_s;

typedef enum {
    FLASHIO_NONE = 0,
    FLASHIO_SPI,
    FLASHIO_QUADSPI,
    FLASHIO_OCTOSPI,
} flashDeviceIoMode_e;

typedef struct flashDeviceIO_s {
    union {
    #ifdef USE_FLASH_SPI
        extDevice_t *dev; // Device interface dependent handle (spi/i2c)
    #endif
    #ifdef USE_FLASH_QUADSPI
        QUADSPI_TypeDef *quadSpi;
    #endif
    #ifdef USE_FLASH_OCTOSPI
        OCTOSPI_TypeDef *octoSpi;
    #endif
    } handle;
    flashDeviceIoMode_e mode;
} flashDeviceIO_t;

typedef struct flashDevice_s {
    //
    // members to be configured by the flash chip implementation
    //

    const struct flashVTable_s *vTable;
    flashGeometry_t geometry;
    uint32_t currentWriteAddress;
    bool isLargeFlash;
    // Whether we've performed an action that could have made the device busy
    // for writes. This allows us to avoid polling for writable status
    // when it is definitely ready already.
    bool couldBeBusy;
    uint32_t timeoutAt;

    //
    // members configured by the flash detection system, read-only from the flash chip implementation's perspective.
    //

    flashDeviceIO_t io;
    void (*callback)(uint32_t arg);
    uint32_t callbackArg;
} flashDevice_t;

typedef struct flashVTable_s {
    void (*configure)(flashDevice_t *fdevice, uint32_t configurationFlags);

    bool (*isReady)(flashDevice_t *fdevice);
    bool (*waitForReady)(flashDevice_t *fdevice);

    void (*eraseSector)(flashDevice_t *fdevice, uint32_t address);
    void (*eraseCompletely)(flashDevice_t *fdevice);

    void (*pageProgramBegin)(flashDevice_t *fdevice, uint32_t address, void (*callback)(uint32_t length));
    uint32_t (*pageProgramContinue)(flashDevice_t *fdevice, uint8_t const **buffers, uint32_t *bufferSizes, uint32_t bufferCount);
    void (*pageProgramFinish)(flashDevice_t *fdevice);
    void (*pageProgram)(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uint32_t length));

    void (*flush)(flashDevice_t *fdevice);

    int (*readBytes)(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length);

    const flashGeometry_t *(*getGeometry)(flashDevice_t *fdevice);
} flashVTable_t;
