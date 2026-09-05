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

#include "platform.h"

#include "config/config_streamer.h"
#include "config/config_streamer_impl.h"
#include "drivers/flash/flash.h"
#include "drivers/system.h"
#include "hpm_interrupt.h"
#include "hpm_l1c_drv.h"
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_romapi.h"

#define HPMICRO_XPI0_BASE    0x80000000

/*
 * All functions in this file run while the XPI NOR flash is being
 * erased/programmed/reconfigured, during which the XIP fetch path must not
 * be used. Place them in ILM so no instruction fetch touches flash.
 */

static xpi_nor_config_t s_xpi_nor_config;
static uint32_t flash_size;
static uint32_t sector_size;
static uint32_t page_size;

FAST_CODE static void configFlashDcFlush(uint32_t address, uint32_t size)
{
    // l1c_dc_flush requires a cacheline-aligned address and a size that is
    // a multiple of the cacheline size, so align the range up.
    const uint32_t start = HPM_L1C_CACHELINE_ALIGN_DOWN(address);
    const uint32_t end = HPM_L1C_CACHELINE_ALIGN_UP(address + size);
    l1c_dc_flush(start, end - start);
}

FAST_CODE static void configFlashDcInvalidate(uint32_t address, uint32_t size)
{
    const uint32_t start = HPM_L1C_CACHELINE_ALIGN_DOWN(address);
    const uint32_t end = HPM_L1C_CACHELINE_ALIGN_UP(address + size);
    l1c_dc_invalidate(start, end - start);
}

FAST_CODE static uint32_t configFlashIrqDisable(void)
{
    const uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    __asm volatile ("fence":::"memory");
    return irq;
}

FAST_CODE static void configFlashIrqRestore(uint32_t irq)
{
    __asm volatile ("fence":::"memory");
    if (irq & CSR_MSTATUS_MIE_MASK) {
        enable_global_irq(CSR_MSTATUS_MIE_MASK);
    }
}

FAST_CODE configStreamerResult_e configWriteWord(uintptr_t address, config_streamer_buffer_type_t *buffer)
{
    hpm_stat_t status;

    // configUnlock() initializes the NOR geometry before the streamer writes.
    // Fail closed if initialization did not complete so the alignment check
    // below cannot evaluate address % 0.
    if (sector_size == 0) {
        return CONFIG_RESULT_FAILURE;
    }
    // The config is written through the memory-mapped window at
    // HPMICRO_XPI0_BASE. After the ROM API erases/programs flash, the CPU
    // D-cache may still hold stale data for this region, so any subsequent
    // read (e.g. isEEPROMVersionValid after write) would see the old
    // contents. Flush the source buffer into RAM before programming, and
    // invalidate the D-cache lines for the target region afterwards so the
    // freshly written data is re-fetched from flash.
    configFlashDcFlush((uint32_t) buffer, CONFIG_STREAMER_BUFFER_SIZE);

    // Interrupt handlers execute from XIP flash, so keep them disabled only
    // while the blocking ROM erase/program calls make XIP unavailable. The
    // streamer can run with interrupts enabled between buffer writes.
    const uint32_t irq = configFlashIrqDisable();

    if (address % sector_size == 0) {
        status = rom_xpi_nor_erase(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &s_xpi_nor_config,
                                   address - HPMICRO_XPI0_BASE, sector_size);

        // fence: ensure the erase is visible to subsequent data loads before
        // the D-cache invalidate (data ordering — this is config data, not
        // executable code, so fence is sufficient and cheaper than fence.i).
        __asm volatile ("fence":::"memory");
        if (status != status_success) {
            // Do not program into an un-erased sector; let the streamer abort
            configFlashIrqRestore(irq);
            return CONFIG_RESULT_FAILURE;
        }
    }
    STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE == sizeof(uint32_t) * 8,
                  "CONFIG_STREAMER_BUFFER_SIZE does not match written size");
    status = rom_xpi_nor_program(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &s_xpi_nor_config, buffer,
                                 address - HPMICRO_XPI0_BASE, CONFIG_STREAMER_BUFFER_SIZE);

    __asm volatile ("fence":::"memory");
    if (status != status_success) {
        configFlashIrqRestore(irq);
        return CONFIG_RESULT_FAILURE;
    }

    configFlashDcInvalidate((uint32_t) address, CONFIG_STREAMER_BUFFER_SIZE);
    configFlashIrqRestore(irq);

    return CONFIG_RESULT_SUCCESS;
}


FAST_CODE void configClearFlags(void)
{
    // NOOP — ROM API returns status directly; no persistent error flags to clear.
}

FAST_CODE void configLock(void)
{
    // The ROM operations restore the caller's IRQ state individually. Keep
    // the streamer-end hook as an ordering point for completed flash writes.
    __asm volatile ("fence":::"memory");
}

FAST_CODE void configUnlock(void)
{
    static bool flash_option_inited = false;
    xpi_nor_config_option_t option;

    if (flash_option_inited) {
        return;
    }
    option.header.U = BOARD_APP_XPI_NOR_CFG_OPT_HDR;
    option.option0.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT0;
    option.option1.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT1;

    XPI_Type *base = BOARD_APP_XPI_NOR_XPI_BASE;

    const uint32_t irq = configFlashIrqDisable();
    hpm_stat_t status = rom_xpi_nor_auto_config(base, &s_xpi_nor_config, &option);

    if (status != status_success) {
        configFlashIrqRestore(irq);
        failureMode(FAILURE_FLASH_WRITE_FAILED);
        return;
    }

    uint32_t detectedFlashSize = 0;
    uint32_t detectedPageSize = 0;
    uint32_t detectedSectorSize = 0;

    status = rom_xpi_nor_get_property(base, &s_xpi_nor_config, xpi_nor_property_total_size, &detectedFlashSize);
    if (status != status_success) {
        configFlashIrqRestore(irq);
        failureMode(FAILURE_FLASH_WRITE_FAILED);
        return;
    }

    status = rom_xpi_nor_get_property(base, &s_xpi_nor_config, xpi_nor_property_page_size, &detectedPageSize);
    if (status != status_success) {
        configFlashIrqRestore(irq);
        failureMode(FAILURE_FLASH_WRITE_FAILED);
        return;
    }

    status = rom_xpi_nor_get_property(base, &s_xpi_nor_config, xpi_nor_property_sector_size, &detectedSectorSize);
    configFlashIrqRestore(irq);
    if (status != status_success) {
        failureMode(FAILURE_FLASH_WRITE_FAILED);
        return;
    }
    // Reject invalid geometry even when the ROM reports success.  Besides
    // preventing divide-by-zero, these relationships are required by the
    // erase/program logic below and by the config-region linker layout.
    if (detectedFlashSize == 0 || detectedPageSize == 0 || detectedSectorSize == 0
        || detectedPageSize > detectedSectorSize || detectedSectorSize > detectedFlashSize
        || detectedSectorSize % detectedPageSize != 0 || detectedFlashSize % detectedSectorSize != 0) {
        failureMode(FAILURE_FLASH_WRITE_FAILED);
        return;
    }

    flash_size = detectedFlashSize;
    page_size = detectedPageSize;
    sector_size = detectedSectorSize;

    /*
     * The config region sits in the last 32K of the flash window that the
     * linker manages (XPI_CFG), starting at __config_start.  The actual
     * physical NOR flash may be larger than the linker window (e.g. the
     * linker books 4M out of a 16M chip); the config region is at the end
     * of that linker window, not necessarily at the end of the physical
     * chip.
     *
     * configWriteWord() only erases a sector when the write address falls
     * on a sector_size boundary, so __config_start must be sector-aligned
     * -- otherwise the first config write would go to un-erased flash and
     * the stored config would be corrupt.  A 4K/8K/16K/32K-sector flash
     * is aligned; a 64K+ sector flash is not (flash_size - 32K is never
     * 64K-aligned).
     */
    if ((uintptr_t) &__config_start % sector_size != 0) {
        // Linker script error: config region must be sector-aligned
        failureMode(FAILURE_DEVELOPER);
        return;
    }
    flash_option_inited = true;
}
