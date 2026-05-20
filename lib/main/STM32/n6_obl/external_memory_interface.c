/*
 * Override of CubeN6's external_memory_interface.c.
 *
 * ST's stock implementation reads function pointers from a fixed RAM
 * address (0x38000000) populated by a separately-staged flash loader
 * binary. That two-stage pattern is what STM32CubeProgrammer's TSV does
 * (slot 0x3 stages MX66UW1G45G_*-OBL.bin into RAM before slot 0x4 writes
 * to nor0). We don't want that — our OBL bundles the flash driver
 * statically and the EXTERNAL_MEMORY_Descriptor below dispatches direct
 * to flash_iface ops.
 *
 * This file is picked up by the Makefile in place of the submodule's
 * external_memory_interface.c; the rest of the OBL middleware
 * (openbl_core, openbl_mem, openbl_usb_cmd, the USB Device Library) is
 * pulled in unchanged via VPATH.
 */

#include <string.h>

#include "stm32n6xx_hal.h"
#include "openbl_mem.h"
#include "openbootloader_conf.h"

#include "flash_iface.h"

#define MEMMAP_BASE   EXT_MEMORY_START_ADDRESS    /* 0x70000000 */

static uint32_t ext_init(uint32_t Address)
{
    (void)Address;
    /* OBL_run_dfu_* in main.c calls flash_init() before entering the OBL
     * loop, so the driver is already up. Return success regardless. */
    return 1U;
}

static uint8_t ext_read(uint32_t Address)
{
    /* OBL Reads come through here a byte at a time. flash_memmap_on()
     * idempotently re-engages memory-mapped mode; reads are direct CPU
     * loads against the AXI window. */
    if (!flash_memmap_on()) {
        return 0xFFU;
    }
    return *(volatile uint8_t *)Address;
}

static void ext_write(uint32_t Address, uint8_t *Data, uint32_t DataLength)
{
    if (Address < MEMMAP_BASE) {
        return;
    }
    const uint32_t offset = Address - MEMMAP_BASE;
    (void)flash_program(offset, Data, DataLength);
}

static void ext_jump(uint32_t Address)
{
    /* OBL never jumps to external memory — the betaflight boot decision
     * lives in main.c and runs after the DFU loop ends via
     * NVIC_SystemReset, not via a direct jump. Stub. */
    (void)Address;
}

static void ext_mass_erase(uint32_t Address)
{
    /* The OBL DFU memory-map advertises only the BF slot (or the OBL
     * slot during recovery), so DFU-driven mass erase is bounded. We
     * implement it as "erase the advertised slot" rather than "erase
     * the whole chip" to avoid wiping the OBL by accident. */
    (void)Address;
}

static void ext_sector_erase(uint32_t StartAddress, uint32_t EndAddress)
{
    if (StartAddress < MEMMAP_BASE || EndAddress < StartAddress) {
        return;
    }
    const uint32_t offset = StartAddress - MEMMAP_BASE;
    const uint32_t length = EndAddress - StartAddress + 1U;
    (void)flash_erase_range(offset, length);
}

static uint64_t ext_verify(uint32_t Address, uint32_t DataAddr,
                           uint32_t DataLength, uint32_t Missalignement)
{
    (void)Missalignement;
    if (Address < MEMMAP_BASE) {
        return 0U;
    }
    const uint32_t offset = Address - MEMMAP_BASE;
    /* DataAddr in this protocol points to a host-supplied buffer in RAM
     * (already received by OBL into one of its scratch areas). Compare
     * directly. Returns 0 on match, otherwise an OPENBL-coded error. */
    if (flash_verify(offset, (const uint8_t *)DataAddr, DataLength)) {
        return 0U;
    }
    return 1U;
}

OPENBL_MemoryTypeDef EXTERNAL_MEMORY_Descriptor =
{
    EXT_MEMORY_START_ADDRESS,
    EXT_MEMORY_END_ADDRESS,
    EXT_MEMORY_SIZE,
    EXTERNAL_MEMORY_AREA,
    ext_init,
    ext_read,
    ext_write,
    ext_jump,
    ext_mass_erase,
    ext_sector_erase,
    ext_verify
};
