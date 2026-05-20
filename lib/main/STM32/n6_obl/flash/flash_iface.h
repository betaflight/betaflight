/*
 * Common XSPI flash driver interface for the Betaflight N6 OpenBootloader.
 *
 * Implementations live alongside this header (mx66uw1g45g.c, etc.) and are
 * selected at build time via the OBL_FLASH_DRIVER make variable. The OBL
 * never indirects through the 0x38000000 function pointer table that ST's
 * stock OBL relies on — the driver is statically linked.
 *
 * Operations expected:
 *   init     : XSPI controller + GPIO + chip command set bring-up; leave
 *              the chip in 1S-1S-1S 4-byte addressing in indirect mode.
 *   geometry : page_size_bytes, sector_size_bytes, total_size_bytes for
 *              DFU descriptor advertisement.
 *   erase    : sector- or block-erase at byte-aligned offset.
 *   program  : page-aligned write (length <= page_size).
 *   memmap_on/off : engage / disengage XSPI memory-mapped mode for read.
 *                   Reads use plain CPU loads against 0x70xxxxxx after
 *                   memmap_on.
 *
 * No error reporting beyond bool — recovery image is single-purpose; if
 * anything fails the only sensible response is to halt and let the user
 * pull power and retry via @FSBL DFU again.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t page_size_bytes;
    uint32_t sector_size_bytes;
    uint32_t total_size_bytes;
} flash_geometry_t;

bool flash_init(void);
void flash_deinit(void);

const flash_geometry_t *flash_get_geometry(void);

bool flash_erase_sector(uint32_t offset);
bool flash_program_page(uint32_t offset, const uint8_t *data, uint32_t length);

bool flash_memmap_on(void);
bool flash_memmap_off(void);

/* Convenience: erase enough sectors to cover [offset, offset+length). */
bool flash_erase_range(uint32_t offset, uint32_t length);

/* Convenience: program a buffer larger than one page, splitting into
 * page-aligned writes. Caller must have erased first. */
bool flash_program(uint32_t offset, const uint8_t *data, uint32_t length);

/* Verify by re-reading via memory-mapped mode. Returns true on match. */
bool flash_verify(uint32_t offset, const uint8_t *data, uint32_t length);
