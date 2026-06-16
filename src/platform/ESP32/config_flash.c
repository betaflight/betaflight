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

#include <string.h>

#include "platform.h"

#include "drivers/system.h"
#include "config/config_streamer.h"

#if defined(CONFIG_IN_FLASH)

#include "esp_rom_spiflash.h"

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/extmem_reg.h"

// ESP32-S3 persistent config in the on-board SPI flash.
//
// Reads are memory-mapped: config_eeprom.c dereferences &__config_start directly,
// so the config region must appear as plain CPU-readable bytes. configFlashInit()
// maps it into the byte-accessible DROM window (0x3C000000) through the flash MMU
// (link/esp32s3.ld points __config_start/__config_end there). It MUST be the DROM
// window - byte reads from the IROM window (0x42000000) fault.
//
// Writes are the hard part. While the SPI-flash controller is erasing/programming
// it cannot service execute-in-place (XIP) fetches, so the operation runs with the
// CPU caches suspended; every instruction and datum touched in that window must
// live in IRAM or ROM (never flash). Core 1 (APP_CPU) also runs from flash XIP and
// is stalled for the duration, and interrupts are disabled. configWriteWord() is
// therefore placed in IRAM and, while the caches are down, calls only ROM routines.
//
// NOTE: the ROM SPI-flash routines use the chip descriptor g_rom_flashchip which the
// ROM keeps in the top of DRAM; link/esp32s3.ld reserves that region so the stack
// cannot clobber it - otherwise these routines hang in their status-poll.

#define DROM_WINDOW_BASE       0x3C000000u   // S3 data bus (byte-accessible) window
#define CONFIG_FLASH_OFFSET    0x00210000u   // flash offset, just past the 2 MB app
#define CONFIG_FLASH_VADDR     (DROM_WINDOW_BASE + CONFIG_FLASH_OFFSET)
#define CONFIG_FLASH_SIZE      0x10000u       // 64 KB region = one S3 flash MMU page
#define FLASH_SECTOR_SIZE      4096u

// Flash MMU table: maps 64 KB flash pages into the cache windows. It lives in the
// peripheral register space (always CPU-accessible, no flash/cache involved), so an
// entry is written directly - the same primitive as IDF's mmu_ll_write_entry().
// ESP32-S3 entry encoding: bits[13:0] = flash page number; VALID is bit14 *clear*
// and the flash (vs PSRAM) target is bit15 clear, so a bare page number is a valid
// flash mapping.
#define MMU_TABLE_BASE         0x600C5000u
#define MMU_VADDR_PAGE_MASK    0x01FFFFFFu   // 32 MB window: entry = (vaddr & mask) >> 16

// Autoload-enable bit passed to Cache_Resume_I/DCache (CACHE_LL_L1_*CACHE_AUTOLOAD).
#define CACHE_L1_AUTOLOAD      (1u << 2)

// Memory-mapped (DROM) config address -> raw flash offset.
#define FLASH_ADDR_TO_OFFSET(addr)  ((uint32_t)((uintptr_t)(addr) - DROM_WINDOW_BASE))

// IRAM placement. Literals follow the code into the section via the toolchain's
// -mtext-section-literals; noinline stops LTO folding this into a flash-resident
// caller (which would silently move it out of IRAM and fault with the cache down).
#define CONFIG_FLASH_IRAM  __attribute__((noinline, section(".iram1.config_flash")))

// ROM cache helpers. The plain Cache_Suspend_I/DCache symbols are IDF C wrappers
// (in an esp_rom patch we don't compile) around these rom_-prefixed ROM entries;
// the only thing the wrapper adds is the post-suspend "wait idle" poll, which we
// replicate below (the S3 ROM suspend returns before the cache is actually idle -
// ESP_ROM_HAS_CACHE_SUSPEND_WAITI_BUG). Resume/Invalidate are real ROM symbols
// provided by esp32s3.rom.ld.
extern uint32_t rom_Cache_Suspend_ICache(void);
extern uint32_t rom_Cache_Suspend_DCache(void);
extern void     Cache_Resume_ICache(uint32_t autoload);
extern void     Cache_Resume_DCache(uint32_t autoload);
extern int      Cache_Invalidate_Addr(uint32_t addr, uint32_t size);

// In the ROM (esp32s3.rom.ld) but not prototyped in esp_rom_spiflash.h: clears any
// flash block-protection bits so erase/program take effect.
extern esp_rom_spiflash_result_t esp_rom_spiflash_unlock(void);

static uint32_t lastErasedSector = UINT32_MAX;
static bool flashUnlocked = false;

void configUnlock(void)
{
    lastErasedSector = UINT32_MAX;
}

void configLock(void)
{
    // NOOP - the ROM flash API needs no locking.
}

void configClearFlags(void)
{
    lastErasedSector = UINT32_MAX;
}

// Map the config region (flash page CONFIG_FLASH_OFFSET>>16) into the DROM window at
// CONFIG_FLASH_VADDR so the memory-mapped reads in config_eeprom.c resolve to flash.
// Called once from initEEPROM() before the first config read. The target MMU entry is
// unused by the app (it occupies entries 0..6), so writing it disturbs no live mapping
// and needs no cache suspend; just invalidate the range afterwards so the first read
// fetches fresh flash rather than a stale cache line.
void configFlashInit(void)
{
    const uint32_t entry = (CONFIG_FLASH_VADDR & MMU_VADDR_PAGE_MASK) >> 16;
    *(volatile uint32_t *)(MMU_TABLE_BASE + entry * 4) = CONFIG_FLASH_OFFSET >> 16;
    __asm__ volatile ("memw" ::: "memory");
    Cache_Invalidate_Addr(CONFIG_FLASH_VADDR, CONFIG_FLASH_SIZE);
}

CONFIG_FLASH_IRAM configStreamerResult_e configWriteWord(uintptr_t address, config_streamer_buffer_type_t *buffer)
{
    const uint32_t flashOffset = FLASH_ADDR_TO_OFFSET(address);
    const uint32_t sector = flashOffset / FLASH_SECTOR_SIZE;
    const bool needErase = (sector != lastErasedSector);
    configStreamerResult_e result = CONFIG_RESULT_SUCCESS;

    uint32_t ps;
    __asm__ volatile ("rsil %0, 15" : "=r"(ps));

    // Stall core 1 (write the 0x86 stall token, split across the two RTC fields).
    SET_PERI_REG_BITS(RTC_CNTL_SW_CPU_STALL_REG, RTC_CNTL_SW_STALL_APPCPU_C1_V, 0x21, RTC_CNTL_SW_STALL_APPCPU_C1_S);
    SET_PERI_REG_BITS(RTC_CNTL_OPTIONS0_REG,     RTC_CNTL_SW_STALL_APPCPU_C0_V, 0x2,  RTC_CNTL_SW_STALL_APPCPU_C0_S);

    // Suspend the caches (XIP off) and wait for them to go idle (S3 ROM erratum).
    rom_Cache_Suspend_ICache();
    while (((READ_PERI_REG(EXTMEM_CACHE_STATE_REG) >> EXTMEM_ICACHE_STATE_S) & EXTMEM_ICACHE_STATE) != 1) { }
    rom_Cache_Suspend_DCache();
    while (((READ_PERI_REG(EXTMEM_CACHE_STATE_REG) >> EXTMEM_DCACHE_STATE_S) & EXTMEM_DCACHE_STATE) != 1) { }

    // Clear any flash block-protection bits once (persists in the status register).
    if (!flashUnlocked) {
        flashUnlocked = (esp_rom_spiflash_unlock() == ESP_ROM_SPIFLASH_RESULT_OK);
    }

    if (needErase) {
        if (esp_rom_spiflash_erase_sector(sector) != ESP_ROM_SPIFLASH_RESULT_OK) {
            result = CONFIG_RESULT_FAILURE;
        }
    }
    if (result == CONFIG_RESULT_SUCCESS) {
        if (esp_rom_spiflash_write(flashOffset, (const uint32_t *)buffer, CONFIG_STREAMER_BUFFER_SIZE) != ESP_ROM_SPIFLASH_RESULT_OK) {
            result = CONFIG_RESULT_FAILURE;
        }
    }

    Cache_Resume_DCache(CACHE_L1_AUTOLOAD);
    Cache_Resume_ICache(CACHE_L1_AUTOLOAD);

    // Un-stall core 1 (clear both stall fields).
    CLEAR_PERI_REG_MASK(RTC_CNTL_OPTIONS0_REG,     RTC_CNTL_SW_STALL_APPCPU_C0_M);
    CLEAR_PERI_REG_MASK(RTC_CNTL_SW_CPU_STALL_REG, RTC_CNTL_SW_STALL_APPCPU_C1_M);

    __asm__ volatile ("wsr.ps %0; rsync" :: "r"(ps) : "memory");

    if (result == CONFIG_RESULT_SUCCESS) {
        if (needErase) {
            lastErasedSector = sector;
        }
        // writeConfigToEEPROM() re-reads the config memory-mapped right after the
        // write (isEEPROMStructureValid); invalidate so it sees the new bytes.
        Cache_Invalidate_Addr(DROM_WINDOW_BASE + flashOffset, CONFIG_STREAMER_BUFFER_SIZE);
    }

    return result;
}

#endif // CONFIG_IN_FLASH
