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
 *
 * Author: jflyper
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_FLASH_W25N02K

#include "flash.h"
#include "flash_impl.h"
#include "flash_w25n01g.h"
#include "flash_w25n02k.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_quadspi.h"
#include "drivers/io.h"
#include "drivers/time.h"

// Device size parameters
#define W25N02KV_PAGE_SIZE         2048
#define W25N02KV_PAGES_PER_BLOCK   64
#define W25N02KV_BLOCKS_PER_DIE    2048  // Updated based on W25N02KV specifications

// BB replacement area
#define W25N02KV_BB_MARKER_BLOCKS           1
#define W25N02KV_BB_REPLACEMENT_BLOCKS      20
#define W25N02KV_BB_MANAGEMENT_BLOCKS       (W25N02KV_BB_REPLACEMENT_BLOCKS + W25N02KV_BB_MARKER_BLOCKS)
// blocks are zero-based index
#define W25N02KV_BB_REPLACEMENT_START_BLOCK (W25N02KV_BLOCKS_PER_DIE - W25N02KV_BB_REPLACEMENT_BLOCKS)
#define W25N02KV_BB_MANAGEMENT_START_BLOCK  (W25N02KV_BLOCKS_PER_DIE - W25N02KV_BB_MANAGEMENT_BLOCKS)
#define W25N02KV_BB_MARKER_BLOCK            (W25N02KV_BB_REPLACEMENT_START_BLOCK - W25N02KV_BB_MARKER_BLOCKS)

// Instructions

#define W25N02KV_INSTRUCTION_RDID             0x9F
#define W25N02KV_INSTRUCTION_DEVICE_RESET     0xFF
#define W25N02KV_INSTRUCTION_READ_STATUS_REG  0x05
#define W25N02KV_INSTRUCTION_READ_STATUS_ALTERNATE_REG  0x0F
#define W25N02KV_INSTRUCTION_WRITE_STATUS_REG 0x01
#define W25N02KV_INSTRUCTION_WRITE_STATUS_ALTERNATE_REG 0x1F
#define W25N02KV_INSTRUCTION_WRITE_ENABLE     0x06
#define W25N02KV_INSTRUCTION_DIE_SELECT       0xC2
#define W25N02KV_INSTRUCTION_BLOCK_ERASE      0xD8
#define W25N02KV_INSTRUCTION_READ_BBM_LUT     0xA5
#define W25N02KV_INSTRUCTION_BB_MANAGEMENT    0xA1
#define W25N02KV_INSTRUCTION_PROGRAM_DATA_LOAD        0x02
#define W25N02KV_INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD 0x84
#define W25N02KV_INSTRUCTION_PROGRAM_EXECUTE  0x10
#define W25N02KV_INSTRUCTION_PAGE_DATA_READ   0x13
#define W25N02KV_INSTRUCTION_READ_DATA        0x03
#define W25N02KV_INSTRUCTION_FAST_READ        0x0B
#define W25N02KV_INSTRUCTION_FAST_READ_QUAD_OUTPUT 0x6B

// Config/status register addresses
#define W25N02KV_PROT_REG 0xA0
#define W25N02KV_CONF_REG 0xB0
#define W25N02KV_STAT_REG 0xC0

// Bits in config/status register 1 (W25N02K_PROT_REG)
#define W25N02KV_PROT_CLEAR                (0)
#define W25N02KV_PROT_SRP1_ENABLE          (1 << 0)
#define W25N02KV_PROT_WP_E_ENABLE          (1 << 1)
#define W25N02KV_PROT_TB_ENABLE            (1 << 2)
#define W25N02KV_PROT_PB0_ENABLE           (1 << 3)
#define W25N02KV_PROT_PB1_ENABLE           (1 << 4)
#define W25N02KV_PROT_PB2_ENABLE           (1 << 5)
#define W25N02KV_PROT_PB3_ENABLE           (1 << 6)
#define W25N02KV_PROT_SRP2_ENABLE          (1 << 7)

// Bits in config/status register 2 (W25N02K_CONF_REG)
#define W25N02KV_CONFIG_ECC_ENABLE         (1 << 4)
#define W25N02KV_CONFIG_BUFFER_READ_MODE   (1 << 3)

// Bits in config/status register 3 (W25N02K_STATREG)
#define W25N02KV_STATUS_BBM_LUT_FULL       (1 << 6)
#define W25N02KV_STATUS_FLAG_ECC_POS       4
#define W25N02KV_STATUS_FLAG_ECC_MASK      ((1 << 5)|(1 << 4))
#define W25N02KV_STATUS_FLAG_ECC(status)   (((status) & W25N02KV_STATUS_FLAG_ECC_MASK) >> 4)
#define W25N02KV_STATUS_PROGRAM_FAIL       (1 << 3)
#define W25N02KV_STATUS_ERASE_FAIL         (1 << 2)
#define W25N02KV_STATUS_FLAG_WRITE_ENABLED (1 << 1)
#define W25N02KV_STATUS_FLAG_BUSY          (1 << 0)

#define W25N02KV_BBLUT_TABLE_ENTRY_COUNT     20
#define W25N02KV_BBLUT_TABLE_ENTRY_SIZE      4  // in bytes

// Bits in LBA for BB LUT
#define W25N02KV_BBLUT_STATUS_ENABLED (1 << 15)
#define W25N02KV_BBLUT_STATUS_INVALID (1 << 14)
#define W25N02KV_BBLUT_STATUS_MASK    (W25N02KV_BBLUT_STATUS_ENABLED | W25N02KV_BBLUT_STATUS_INVALID)

// Some useful defs and macros
#define W25N02KV_LINEAR_TO_COLUMN(laddr) ((laddr) % W25N02KV_PAGE_SIZE)
#define W25N02KV_LINEAR_TO_PAGE(laddr) ((laddr) / W25N02KV_PAGE_SIZE)
#define W25N02KV_LINEAR_TO_BLOCK(laddr) (W25N02KV_LINEAR_TO_PAGE(laddr) / W25N02KV_PAGES_PER_BLOCK)
#define W25N02KV_BLOCK_TO_PAGE(block) ((block) * W25N02KV_PAGES_PER_BLOCK)
#define W25N02KV_BLOCK_TO_LINEAR(block) (W25N02KV_BLOCK_TO_PAGE(block) * W25N02KV_PAGE_SIZE)

// IMPORTANT: Timeout values are currently required to be set to the highest value required by any of the supported flash chips by this driver

// The timeout values (2ms minimum to avoid 1 tick advance in consecutive calls to millis).
#define W25N02KV_TIMEOUT_PAGE_READ_MS        2   // tREmax = 60us (ECC enabled)
#define W25N02KV_TIMEOUT_PAGE_PROGRAM_MS     2   // tPPmax = 700us
#define W25N02KV_TIMEOUT_BLOCK_ERASE_MS      15  // tBEmax = 10ms
#define W25N02KV_TIMEOUT_RESET_MS            500 // tRSTmax = 500ms

// Sizes (in bits)
#define W28N01G_STATUS_REGISTER_SIZE        8
#define W28N01G_STATUS_PAGE_ADDRESS_SIZE    16
#define W28N01G_STATUS_COLUMN_ADDRESS_SIZE  16

typedef struct bblut_s {
    uint16_t pba;
    uint16_t lba;
} bblut_t;

//**************************************下述内容更改**********************************************************************

/******************    等待设备就绪     ******************/
static bool w25n02kv_waitForReady(flashDevice_t *fdevice);

/******************    设置超时     ******************/
static void w25n02kv_setTimeout(flashDevice_t *fdevice, uint32_t timeoutMillis)
{
    uint32_t now = millis();
    fdevice->timeoutAt = now + timeoutMillis;
}

/**
 * Send the given command byte to the device.
 */
/******************    执行单字节命令     ******************/
static void w25n02kv_performOneByteCommand(flashDeviceIO_t *io, uint8_t command)
{
    if (io->mode == FLASHIO_SPI) {
        extDevice_t *dev = io->handle.dev;

        busSegment_t segments[] = {
                {.u.buffers = {&command, NULL}, sizeof(command), true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        spiSequence(dev, &segments[0]);

        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (io->mode == FLASHIO_QUADSPI) {
        QUADSPI_TypeDef *quadSpi = io->handle.quadSpi;
        quadSpiTransmit1LINE(quadSpi, command, 0, NULL, 0);
    }
#endif
}

/******************    执行带页面地址的命令     ******************/
static void w25n02kv_performCommandWithPageAddress(flashDeviceIO_t *io, uint8_t command, uint32_t pageAddress)
{
    if (io->mode == FLASHIO_SPI) {
        extDevice_t *dev = io->handle.dev;

        uint8_t cmd[] = { command, 0, (pageAddress >> 8) & 0xff, (pageAddress >> 0) & 0xff};

        busSegment_t segments[] = {
                {.u.buffers = {cmd, NULL}, sizeof(cmd), true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        spiSequence(dev, &segments[0]);

        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (io->mode is FLASHIO_QUADSPI) {
        QUADSPI_TypeDef *quadSpi = io->handle.quadSpi;

        quadSpiInstructionWithAddress1LINE(quadSpi, command, 0, pageAddress & 0xffff, W28N02KV_STATUS_PAGE_ADDRESS_SIZE + 8);
    }
#endif
}

/******************    读取寄存器函数     ******************/
static uint8_t w25n02kv_readRegister(flashDeviceIO_t *io, uint8_t reg)
{
    if (io->mode == FLASHIO_SPI) {
        extDevice_t *dev = io->handle.dev;

        // 修改为适用于W25N02KV的读状态寄存器指令
        uint8_t cmd[3] = { W25N02KV_INSTRUCTION_READ_STATUS_REG, reg, 0 };
        uint8_t in[3];

        busSegment_t segments[] = {
                {.u.buffers = {cmd, in}, sizeof(cmd), true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        // 确保DMA完成
        spiWait(dev);

        spiSequence(dev, &segments[0]);

        // 等待SPI访问完成
        spiWait(dev);

        return in[2];
    }
#ifdef USE_QUADSPI
    else if (io->mode == FLASHIO_QUADSPI) {

        QUADSPI_TypeDef *quadSpi = io->handle.quadSpi;

        uint8_t in[1];
        // 调整为W25N02KV特有的大小和命令
        quadSpiReceiveWithAddress1LINE(quadSpi, W25N02KV_INSTRUCTION_READ_STATUS_REG, 0, reg, W28N01G_STATUS_REGISTER_SIZE, in, sizeof(in));

        return in[0];
    }
#endif
    return 0;
}

/******************    写寄存器函数     ******************/
static void w25n02kv_writeRegister(flashDeviceIO_t *io, uint8_t reg, uint8_t data)
{
    if (io->mode == FLASHIO_SPI) {
        extDevice_t *dev = io->handle.dev;
        // 更新为适用于W25N02KV的写状态寄存器指令
        uint8_t cmd[3] = { W25N02KV_INSTRUCTION_WRITE_STATUS_REG, reg, data };

        busSegment_t segments[] = {
                {.u.buffers = {cmd, NULL}, sizeof(cmd), true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        // 确保任何先前的DMA已完成
        spiWait(dev);

        spiSequence(dev, &segments[0]);

        // 阻塞等待SPI访问完成
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (io->mode == FLASHIO_QUADSPI) {
        QUADSPI_TypeDef *quadSpi = io->handle.quadSpi;

        // 更新为适用于W25N02KV的指令和寄存器大小
        quadSpiTransmitWithAddress1LINE(quadSpi, W25N02KV_INSTRUCTION_WRITE_STATUS_REG, 0, reg, W28N01G_STATUS_REGISTER_SIZE, &data, 1);
    }
#endif
}

/*****************  W25N02KV 设备复位函数   *****************/
static void w25n02kv_deviceReset(flashDevice_t *fdevice)
{
    flashDeviceIO_t *io = &fdevice->io;

    // 使用适合W25N02KV的复位指令
    w25n02kv_performOneByteCommand(io, W25N02KV_INSTRUCTION_DEVICE_RESET);

    // 设置超时时间为W25N02KV定义的复位最大时间
    w25n02kv_setTimeout(fdevice, W25N02KV_TIMEOUT_RESET_MS);
    w25n02kv_waitForReady(fdevice);

    // 清除保护设置，WP-E关闭
    w25n02kv_writeRegister(io, W25N02KV_PROT_REG, W25N02KV_PROT_CLEAR);

    // 启用缓冲读模式和ECC
    w25n02kv_writeRegister(io, W25N02KV_CONF_REG, W25N02KV_CONFIG_ECC_ENABLE|W25N02KV_CONFIG_BUFFER_READ_MODE);
}

/******************    检查设备是否就绪     ******************/
bool w25n02kv_isReady(flashDevice_t *fdevice)
{
    uint8_t status = w25n02kv_readRegister(&fdevice->io, W25N02KV_STAT_REG);

    return ((status & W25N02KV_STATUS_FLAG_BUSY) == 0);
}

/******************    等待设备就绪     ******************/
static bool w25n02kv_waitForReady(flashDevice_t *fdevice)
{
    while (!w25n02kv_isReady(fdevice)) {
        uint32_t now = millis();
        if (cmp32(now, fdevice->timeoutAt) >= 0) {
            return false;
        }
    }
    fdevice->timeoutAt = 0;

    return true;
}



/**
 * The flash requires this write enable command to be sent before commands that would cause
 * a write like program and erase.
 */
/******************    写使能函数     ******************/
static void w25n02kv_writeEnable(flashDevice_t *fdevice)
{
    w25n02kv_performOneByteCommand(&fdevice->io, W25N02KV_INSTRUCTION_WRITE_ENABLE);

    // 假设我们即将进行写操作，因此设备即将忙碌
    fdevice->couldBeBusy = true;
}

/**
 * Read chip identification and geometry information (into global `geometry`).
 *
 * Returns true if we get valid ident, false if something bad happened like there is no M25P16.
 */
const flashVTable_t w25n02kv_vTable;

static void w25n02kv_deviceInit(flashDevice_t *flashdev);

/******************    设备检测和初始化函数     ******************/
bool w25n02kv_detect(flashDevice_t *fdevice, uint32_t chipID)
{
    switch (chipID) {
    case JEDEC_ID_WINBOND_W25N02KV:  // 更新为W25N02KV的JEDEC ID
        fdevice->geometry.sectors = 2048;      // W25N02KV有2048个块
        fdevice->geometry.pagesPerSector = 64; // 每个块64页
        fdevice->geometry.pageSize = 2048;     // 每页2048字节
        break;

    default:
        // 不支持的芯片
        fdevice->geometry.sectors = 0;
        fdevice->geometry.pagesPerSector = 0;

        fdevice->geometry.sectorSize = 0;
        fdevice->geometry.totalSize = 0;
        return false;
    }

    fdevice->geometry.flashType = FLASH_TYPE_NAND;
    fdevice->geometry.sectorSize = fdevice->geometry.pagesPerSector * fdevice->geometry.pageSize;
    fdevice->geometry.totalSize = fdevice->geometry.sectorSize * fdevice->geometry.sectors;

    flashPartitionSet(FLASH_PARTITION_TYPE_BADBLOCK_MANAGEMENT,
            W25N02KV_BB_MANAGEMENT_START_BLOCK,
            W25N02KV_BB_MANAGEMENT_START_BLOCK + W25N02KV_BB_MANAGEMENT_BLOCKS - 1);

    fdevice->couldBeBusy = true; // 假设芯片可能正忙

    w25n02kv_deviceReset(fdevice);

    // Upper 4MB (32 blocks * 128KB/block) will be used for bad block replacement area.

    // Blocks in this area are only written through bad block LUT,
    // and factory written bad block marker in unused blocks are retained.

    // When a replacement block is required,
    // (1) "Read BB LUT" command is used to obtain the last block mapped,
    // (2) blocks after the last block is scanned for a good block,
    // (3) the first good block is used for replacement, and the BB LUT is updated.

    // There are only 20 BB LUT entries, and there are 32 replacement blocks.
    // There will be a least chance of running out of replacement blocks.
    // If it ever run out, the device becomes unusable.


    w25n02kv_deviceInit(fdevice);

    fdevice->vTable = &w25n02kv_vTable; // 使用W25N02KV的虚拟表

    return true;
}


/**
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
/******************    擦除指定扇区     ******************/
void w25n02kv_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    // 等待设备就绪
    w25n02kv_waitForReady(fdevice);

    // 启用写操作
    w25n02kv_writeEnable(fdevice);

    // 执行擦除命令，转换线性地址到页面地址
    w25n02kv_performCommandWithPageAddress(&fdevice->io, W25N02KV_INSTRUCTION_BLOCK_ERASE, W25N02KV_LINEAR_TO_PAGE(address));

    // 设置擦除超时
    w25n02kv_setTimeout(fdevice, W25N02KV_TIMEOUT_BLOCK_ERASE_MS);
}

/******************    擦除整个芯片     ******************/
// W25N02KV does not support full chip erase.
// Call eraseSector repeatedly to erase the entire chip.
void w25n02kv_eraseCompletely(flashDevice_t *fdevice)
{
    // 循环遍历所有扇区
    for (uint32_t block = 0; block < fdevice->geometry.sectors; block++) {
        // 对每一个扇区调用擦除函数，将块地址转换成线性地址
        w25n02kv_eraseSector(fdevice, W25N02KV_BLOCK_TO_LINEAR(block));
    }
}

/******************    指定列地址上传输数据     ******************/
static void w25n02kv_programDataLoad(flashDevice_t *fdevice, uint16_t columnAddress, const uint8_t *data, int length)
{
    w25n02kv_waitForReady(fdevice);  // 确保设备就绪

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;
        uint8_t cmd[] = { W25N02KV_INSTRUCTION_PROGRAM_DATA_LOAD, columnAddress >> 8, columnAddress & 0xff };

        busSegment_t segments[] = {
                {.u.buffers = {cmd, NULL}, sizeof(cmd), false, NULL},
                {.u.buffers = {(uint8_t *)data, NULL}, length, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        QUADSPI_TypeDef *quadSpi = fdevice->io.handle.quadSpi;

        quadSpiTransmitWithAddress1LINE(quadSpi, W25N02KV_INSTRUCTION_PROGRAM_DATA_LOAD, 0, columnAddress, W28N01G_STATUS_COLUMN_ADDRESS_SIZE, data, length);
    }
#endif

    w25n02kv_setTimeout(fdevice, W25N02KV_TIMEOUT_PAGE_PROGRAM_MS);
}

/******************    指定列地址随机写入数据     ******************/
static void w25n02kv_randomProgramDataLoad(flashDevice_t *fdevice, uint16_t columnAddress, const uint8_t *data, int length)
{
    // 使用W25N02KV专用的随机数据加载指令
    uint8_t cmd[] = { W25N02KV_INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD, columnAddress >> 8, columnAddress & 0xff };

    // 确保W25N02KV设备就绪
    w25n02kv_waitForReady(fdevice);

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        busSegment_t segments[] = {
                {.u.buffers = {cmd, NULL}, sizeof(cmd), false, NULL},
                {.u.buffers = {(uint8_t *)data, NULL}, length, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        spiSequence(dev, &segments[0]);

        // 等待SPI传输完成
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        QUADSPI_TypeDef *quadSpi = fdevice->io.handle.quadSpi;

        // 在QUADSPI模式下发送随机数据加载指令
        quadSpiTransmitWithAddress1LINE(quadSpi, W25N02KV_INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD, 0, columnAddress, W28N01G_STATUS_COLUMN_ADDRESS_SIZE, data, length);
    }
#endif

    // 设置页面编程的超时时间
    w25n02kv_setTimeout(fdevice, W25N02KV_TIMEOUT_PAGE_PROGRAM_MS);
}

/****************** 触发Flash设备上的页面编程操作 ******************/
static void w25n02kv_programExecute(flashDevice_t *fdevice, uint32_t pageAddress)
{
    // 确保W25N02KV设备就绪
    w25n02kv_waitForReady(fdevice);

    // 执行程序操作指令，传递页地址
    w25n02kv_performCommandWithPageAddress(&fdevice->io, W25N02KV_INSTRUCTION_PROGRAM_EXECUTE, pageAddress);

    // 设置超时，保证程序执行完成
    w25n02kv_setTimeout(fdevice, W25N02KV_TIMEOUT_PAGE_PROGRAM_MS);
}


//
// Writes are done in three steps:
// (1) Load internal data buffer with data to write
//     - We use "Random Load Program Data", as "Load Program Data" resets unused data bytes in the buffer to 0xff.
//     - Each "Random Load Program Data" instruction must be accompanied by at least a single data.
//     - Each "Random Load Program Data" instruction terminates at the rising of CS.
// (2) Enable write
// (3) Issue "Execute Program"
//

/*
flashfs page program behavior
- Single program never crosses page boundary.
- Except for this characteristic, it program arbitral size.
- Write address is, naturally, not a page boundary.

To cope with this behavior.

pageProgramBegin:
If buffer is dirty and programLoadAddress != address, then the last page is a partial write;
issue PAGE_PROGRAM_EXECUTE to flash buffer contents, clear dirty and record the address as programLoadAddress and programStartAddress.
Else do nothing.

pageProgramContinue:
Mark buffer as dirty.
If programLoadAddress is on page boundary, then issue PROGRAM_LOAD_DATA, else issue RANDOM_PROGRAM_LOAD_DATA.
Update programLoadAddress.
Optionally observe the programLoadAddress, and if it's on page boundary, issue PAGE_PROGRAM_EXECUTE.

pageProgramFinish:
Observe programLoadAddress. If it's on page boundary, issue PAGE_PROGRAM_EXECUTE and clear dirty, else just return.
If pageProgramContinue observes the page boundary, then do nothing(?).
*/

static uint32_t programStartAddress;
static uint32_t programLoadAddress;
static bool bufferDirty = false;
static bool isProgramming = false;

/****************** 开始Flash设备上的页面编程过程 ******************/
void w25n02kv_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uint32_t length))
{
    fdevice->callback = callback;  // 保存回调函数，用于操作完成后调用

    if (bufferDirty) {
        if (address != programLoadAddress) {
            w25n02kv_waitForReady(fdevice);  // 等待设备就绪

            isProgramming = false;

            w25n02kv_writeEnable(fdevice);  // 启用写操作

            w25n02kv_programExecute(fdevice, W25N02KV_LINEAR_TO_PAGE(programStartAddress));  // 执行页面编程

            bufferDirty = false;
            isProgramming = true;
        }
    } else {
        programStartAddress = programLoadAddress = address;  // 设置开始和加载地址
    }
}

/****************** 继续W25N02KV设备上的页面编程过程 ******************/
uint32_t w25n02kv_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, uint32_t *bufferSizes, uint32_t bufferCount)
{
    if (bufferCount < 1) {
        fdevice->callback(0);
        return 0;
    }

    w25n02kv_waitForReady(fdevice);  // 等待设备就绪

    w25n02kv_writeEnable(fdevice);  // 启用写操作

    isProgramming = false;

    if (!bufferDirty) {
        w25n02kv_programDataLoad(fdevice, W25N02KV_LINEAR_TO_COLUMN(programLoadAddress), buffers[0], bufferSizes[0]);
    } else {
        w25n02kv_randomProgramDataLoad(fdevice, W25N02KV_LINEAR_TO_COLUMN(programLoadAddress), buffers[0], bufferSizes[0]);
    }

   // XXX Test if write enable is reset after each data loading.

    bufferDirty = true;
    programLoadAddress += bufferSizes[0];

    if (fdevice->callback) {
        fdevice->callback(bufferSizes[0]);
    }

    return bufferSizes[0];
}

/****************** 完成W25N02KV设备上的页面编程过程 ******************/
static uint32_t currentPage = UINT32_MAX;

void w25n02kv_pageProgramFinish(flashDevice_t *fdevice)
{
    if (bufferDirty && W25N02KV_LINEAR_TO_COLUMN(programLoadAddress) == 0) {

        currentPage = W25N02KV_LINEAR_TO_PAGE(programStartAddress); // reset page to the page being written

        w25n02kv_programExecute(fdevice, W25N02KV_LINEAR_TO_PAGE(programStartAddress));

        bufferDirty = false;
        isProgramming = true;

        programStartAddress = programLoadAddress;
    }
}


/**
 * Write bytes to a flash page. Address must not cross a page boundary.
 *
 * Bits can only be set to zero, not from zero back to one again. In order to set bits to 1, use the erase command.
 *
 * Length must be smaller than the page size.
 *
 * This will wait for the flash to become ready before writing begins.
 *
 * Datasheet indicates typical programming time is 0.8ms for 256 bytes, 0.2ms for 64 bytes, 0.05ms for 16 bytes.
 * (Although the maximum possible write time is noted as 5ms).
 *
 * If you want to write multiple buffers (whose sum of sizes is still not more than the page size) then you can
 * break this operation up into one beginProgram call, one or more continueProgram calls, and one finishProgram call.
 */

/****************** 开始到结束，执行W25N02KV设备上的完整页面编程过程 ******************/
void w25n02kv_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uint32_t length))
{
    w25n02kv_pageProgramBegin(fdevice, address, callback);          // 初始化页面编程并设置回调
    w25n02kv_pageProgramContinue(fdevice, &data, &length, 1);       // 加载数据并继续编程过程
    w25n02kv_pageProgramFinish(fdevice);                            // 完成编程，确保所有数据写入
}

/****************** 清除W25N02KV设备上挂起的页面编程操作 ******************/
void w25n02kv_flush(flashDevice_t *fdevice)
{
    if (bufferDirty) {
        currentPage = W25N02KV_LINEAR_TO_PAGE(programStartAddress);  // 重设当前页面

        w25n02kv_programExecute(fdevice, W25N02KV_LINEAR_TO_PAGE(programStartAddress));  // 强制执行页面编程

        bufferDirty = false;  // 标记缓冲区为已清理
        isProgramming = true;  // 设置正在编程状态
    } else {
        isProgramming = false;  // 无需清除，设置为非编程状态
    }
}

/****************** 添加错误记录 ******************/
void w25n02kv_addError(uint32_t address, uint8_t code)
{
    UNUSED(address);  // 当前未使用地址参数
    UNUSED(code);     // 当前未使用代码参数
}


/**
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * Waits up to W25N02K_TIMEOUT_PAGE_READ_MS milliseconds for the flash to become ready before reading.
 *
 * The number of bytes actually read is returned, which can be zero if an error or timeout occurred.
 */

// Continuous read mode (BUF = 0):
// (1) "Page Data Read" command is executed for the page pointed by address
// (2) "Read Data" command is executed for bytes not requested and data are discarded
// (3) "Read Data" command is executed and data are stored directly into caller's buffer
//
// Buffered read mode (BUF = 1), non-read ahead
// (1) If currentBufferPage != requested page, then issue PAGE_DATA_READ on requested page.
// (2) Compute transferLength as smaller of remaining length and requested length.
// (3) Issue READ_DATA on column address.
// (4) Return transferLength.


/****************** 从W25N02KV设备的指定地址读取数据 ******************/
int w25n02kv_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    uint32_t targetPage = W25N02KV_LINEAR_TO_PAGE(address);  // 计算目标页面

    if (currentPage != targetPage) {  // 检查当前页面是否已加载
        if (!w25n02kv_waitForReady(fdevice)) {  // 等待设备就绪
            return 0;
        }

        currentPage = UINT32_MAX;  // 重置当前页面

        w25n02kv_performCommandWithPageAddress(&fdevice->io, W25N02KV_INSTRUCTION_PAGE_DATA_READ, targetPage);  // 执行页面数据读取命令

        w25n02kv_setTimeout(fdevice, W25N02KV_TIMEOUT_PAGE_READ_MS);  // 设置超时
        if (!w25n02kv_waitForReady(fdevice)) {  // 再次等待设备就绪
            return 0;
        }

        currentPage = targetPage;  // 更新当前页面
    }

    uint32_t column = W25N02KV_LINEAR_TO_COLUMN(address);  // 计算列地址
    uint16_t transferLength;  // 计算实际传输长度

    if (length > W25N02KV_PAGE_SIZE - column) {
        transferLength = W25N02KV_PAGE_SIZE - column;  // 如果请求长度超过页面剩余长度，调整为最大可能长度
    } else {
        transferLength = length;  // 否则使用请求的长度
    }

    if (fdevice->io.mode == FLASHIO_SPI) {  // SPI模式
        extDevice_t *dev = fdevice->io.handle.dev;

        uint8_t cmd[4];
        cmd[0] = W25N02KV_INSTRUCTION_READ_DATA;
        cmd[1] = (column >> 8) & 0xff;
        cmd[2] = (column & 0xff);
        cmd[3] = 0;

        busSegment_t segments[] = {
                {.u.buffers = {cmd, NULL}, sizeof(cmd), false, NULL},
                {.u.buffers = {NULL, buffer}, transferLength, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        spiSequence(dev, &segments[0]);  // 发起SPI传输序列

        spiWait(dev);  // 等待SPI操作完成
    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {  // QUADSPI模式
        QUADSPI_TypeDef *quadSpi = fdevice->io.handle.quadSpi;

        quadSpiReceiveWithAddress4LINES(quadSpi, W25N02KV_INSTRUCTION_FAST_READ_QUAD_OUTPUT, 8, column, W28N01G_STATUS_COLUMN_ADDRESS_SIZE, buffer, transferLength);
    }
#endif

    w25n02kv_setTimeout(fdevice, W25N02KV_TIMEOUT_PAGE_READ_MS);  // 重新设置超时
    if (!w25n02kv_waitForReady(fdevice)) {  // 最后的设备就绪检查
        return 0;
    }

    // ECC状态检查
    uint8_t statReg = w25n02kv_readRegister(&fdevice->io, W25N02KV_STAT_REG);
    uint8_t eccCode = W25N02KV_STATUS_FLAG_ECC(statReg);  // 解析ECC状态码

    switch (eccCode) {
    case 0: // 成功读取，无ECC校正
        break;
    case 1: // 成功读取，有ECC校正
    case 2: // 单页无法纠正的ECC错误
    case 3: // 多页无法纠正的ECC错误
        w25n02kv_addError(address, eccCode);  // 记录错误
        w25n02kv_deviceReset(fdevice);  // 重置设备
        break;
    }

    return transferLength;  // 返回实际读取的长度
}

/****************** 从W25N02KV设备的扩展区读取数据 ******************/
int w25n02kv_readExtensionBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, int length)
{
    // 等待设备就绪
    if (!w25n02kv_waitForReady(fdevice)) {
        return 0;
    }

    // 执行页面数据读取命令，预读取数据到缓存
    w25n02kv_performCommandWithPageAddress(&fdevice->io, W25N02KV_INSTRUCTION_PAGE_DATA_READ, W25N02KV_LINEAR_TO_PAGE(address));

    // 设置读取扩展区数据的起始列地址（2048为W25N02KV的页面大小）
    uint32_t column = 2048;

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        // 准备读数据指令和地址
        uint8_t cmd[4];
        cmd[0] = W25N02KV_INSTRUCTION_READ_DATA;
        cmd[1] = (column >> 8) & 0xff;
        cmd[2] = (column & 0xff);
        cmd[3] = 0;

        busSegment_t segments[] = {
                {.u.buffers = {cmd, NULL}, sizeof(cmd), false, NULL},
                {.u.buffers = {NULL, buffer}, length, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        // 确保任何之前的DMA已完成
        spiWait(dev);

        // 启动SPI传输序列
        spiSequence(dev, &segments[0]);

        // 等待SPI访问完成
        spiWait(dev);

    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        QUADSPI_TypeDef *quadSpi = fdevice->io.handle.quadSpi;

        // 在QUADSPI模式下读取数据
        quadSpiReceiveWithAddress1LINE(quadSpi, W25N02KV_INSTRUCTION_READ_DATA, 8, column, W28N01G_STATUS_COLUMN_ADDRESS_SIZE, buffer, length);
    }
#endif

    // 设置读取超时
    w25n02kv_setTimeout(fdevice, W25N02KV_TIMEOUT_PAGE_READ_MS);

    return length;  // 返回读取的长度
}

/**
 * Fetch information about the detected flash chip layout.
 *
 * Can be called before calling W25N02K_init() (the result would have totalSize = 0).
 */

/****************** 获取检测到的Flash设备的布局信息 ******************/
const flashGeometry_t* w25n02kv_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;  // 返回指向设备几何结构的指针
}

/****************** 虚拟表（VTable）配置 ******************/
const flashVTable_t w25n02kv_vTable = {
    .isReady = w25n02kv_isReady,
    .waitForReady = w25n02kv_waitForReady,
    .eraseSector = w25n02kv_eraseSector,
    .eraseCompletely = w25n02kv_eraseCompletely,
    .pageProgramBegin = w25n02kv_pageProgramBegin,
    .pageProgramContinue = w25n02kv_pageProgramContinue,
    .pageProgramFinish = w25n02kv_pageProgramFinish,
    .pageProgram = w25n02kv_pageProgram,
    .flush = w25n02kv_flush,
    .readBytes = w25n02kv_readBytes,
    .getGeometry = w25n02kv_getGeometry,  
};

typedef volatile struct cb_context_s {
    flashDevice_t *fdevice;  // 指向Flash设备的指针
    bblut_t *bblut;  // 指向坏块查找表的指针
    int lutsize;  // 查找表的大小
    int lutindex;  // 当前查找表索引
} cb_context_t;

// Called in ISR context
// Read of BBLUT entry has just completed
/****************** 在中断服务例程中被调用，完成BBLUT条目读取后的回调处理 ******************/
busStatus_e w25n02kv_readBBLUTCallback(uint32_t arg)
{
    cb_context_t *cb_context = (cb_context_t *)arg;  // 从传递的参数中获取回调上下文
    flashDevice_t *fdevice = cb_context->fdevice;  // 从上下文中获取Flash设备指针
    uint8_t *rxData = fdevice->io.handle.dev->bus->curSegment->u.buffers.rxData;  // 获取接收数据缓冲区

    // 解析接收到的数据，设置物理块地址（PBA）和逻辑块地址（LBA）
    cb_context->bblut->pba = (rxData[0] << 16) | rxData[1];
    cb_context->bblut->lba = (rxData[2] << 16) | rxData[3];

    // 如果当前索引未达到查找表的大小，递增索引并移动到下一个查找表条目
    if (++cb_context->lutindex < cb_context->lutsize) {
        cb_context->bblut++;
        return BUS_BUSY; // 还有更多的条目要处理，返回忙状态
    }

    return BUS_READY; // 所有条目处理完成，返回就绪状态
}

/****************** 从W25N02KV设备读取坏块查找表 (BBLUT) ******************/
void w25n02kv_readBBLUT(flashDevice_t *fdevice, bblut_t *bblut, int lutsize)
{
    cb_context_t cb_context;
    uint8_t in[4];

    cb_context.fdevice = fdevice;  // 设置回调上下文中的设备指针
    fdevice->callbackArg = (uint32_t)&cb_context;  // 传递上下文为回调参数

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        uint8_t cmd[4];
        cmd[0] = W25N02KV_INSTRUCTION_READ_BBM_LUT;  // 设置读取BBM LUT的命令
        cmd[1] = 0;  // 命令的后续字节（如果有的话）

        cb_context.bblut = &bblut[0];  // 设置回调上下文中的坏块查找表指针
        cb_context.lutsize = lutsize;  // 设置查找表的大小
        cb_context.lutindex = 0;  // 初始化查找表索引

        busSegment_t segments[] = {
            {.u.buffers = {cmd, NULL}, sizeof(cmd), false, NULL},
            {.u.buffers = {NULL, in}, sizeof(in), true, w25n02kv_readBBLUTCallback},
            {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        spiSequence(dev, &segments[0]);  // 启动SPI传输序列

        spiWait(dev);  // 等待SPI访问完成
    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        QUADSPI_TypeDef *quadSpi = fdevice->io.handle.quadSpi;

        // Note: Using HAL QuadSPI there doesn't appear to be a way to send 2 bytes, then blocks of 4 bytes, while keeping the CS line LOW
        // thus, we have to read the entire BBLUT in one go and process the result.


        uint8_t bblutBuffer[W25N02KV_BBLUT_TABLE_ENTRY_COUNT * W25N02KV_BBLUT_TABLE_ENTRY_SIZE];
        quadSpiReceive1LINE(quadSpi, W25N02KV_INSTRUCTION_READ_BBM_LUT, 8, bblutBuffer, sizeof(bblutBuffer));

        for (int i = 0, offset = 0 ; i < lutsize ; i++, offset += 4) {
            if (i < W25N02KV_BBLUT_TABLE_ENTRY_COUNT) {
                bblut[i].pba = (in[offset + 0] << 16)|in[offset + 1];
                bblut[i].lba = (in[offset + 2] << 16)|in[offset + 3];
            }
        }
    }
#endif
}

/****************** 写入W25N02KV设备的坏块查找表 (BBLUT) ******************/
void w25n02kv_writeBBLUT(flashDevice_t *fdevice, uint16_t lba, uint16_t pba)
{
    w25n02kv_waitForReady(fdevice);  // 等待设备就绪

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        uint8_t cmd[5] = { W25N02KV_INSTRUCTION_BB_MANAGEMENT, lba >> 8, lba, pba >> 8, pba };

        busSegment_t segments[] = {
                {.u.buffers = {cmd, NULL}, sizeof(cmd), true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        // 确保任何之前的DMA已完成
        spiWait(dev);

        // 启动SPI传输序列
        spiSequence(dev, &segments[0]);

        // 等待SPI访问完成
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        QUADSPI_TypeDef *quadSpi = fdevice->io.handle.quadSpi;

        uint8_t data[4] = { lba >> 8, lba, pba >> 8, pba };
        quadSpiInstructionWithData1LINE(quadSpi, W25N02KV_INSTRUCTION_BB_MANAGEMENT, 0, data, sizeof(data));
    }
#endif

    w25n02kv_setTimeout(fdevice, W25N02KV_TIMEOUT_PAGE_PROGRAM_MS);  // 设置超时
}

static void w25n02kv_deviceInit(flashDevice_t *flashdev)
{
    UNUSED(flashdev);
}

#endif
