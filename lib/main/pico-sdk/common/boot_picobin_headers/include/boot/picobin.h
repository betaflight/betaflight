/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BOOT_PICOBIN_H
#define _BOOT_PICOBIN_H

#ifndef NO_PICO_PLATFORM
#include "pico/platform.h"
#else
#ifndef _u
#ifdef __ASSEMBLER__
#define _u(x) x
#else
#define _u(x) x ## u
#endif
#endif
#endif

/** \file picobin.h
*  \defgroup boot_picobin_headers boot_picobin_headers
*
* \brief Constants for PICOBIN format
*/

// these are designed to not look like (likely) 16/32-bit ARM or RISC-V instructions or look like valid pointers
#define PICOBIN_BLOCK_MARKER_START _u(0xffffded3)
#define PICOBIN_BLOCK_MARKER_END   _u(0xab123579)

#define PICOBIN_MAX_BLOCK_SIZE _u(0x280)
#define PICOBIN_MAX_IMAGE_DEF_BLOCK_SIZE _u(0x180)
#define PICOBIN_MAX_PARTITION_TABLE_BLOCK_SIZE _u(0x280)

// note bit 6 is used to make parity even
#define PICOBIN_BLOCK_ITEM_1BS_NEXT_BLOCK_OFFSET        _u(0x41)
#define PICOBIN_BLOCK_ITEM_1BS_IMAGE_TYPE               _u(0x42)
#define PICOBIN_BLOCK_ITEM_1BS_VECTOR_TABLE             _u(0x03)
#define PICOBIN_BLOCK_ITEM_1BS_ENTRY_POINT              _u(0x44)
#define PICOBIN_BLOCK_ITEM_1BS_ROLLING_WINDOW_DELTA     _u(0x05)
#define PICOBIN_BLOCK_ITEM_LOAD_MAP                     _u(0x06)
#define PICOBIN_BLOCK_ITEM_1BS_HASH_DEF                 _u(0x47)
#define PICOBIN_BLOCK_ITEM_1BS_VERSION                  _u(0x48)
#define PICOBIN_BLOCK_ITEM_SIGNATURE                    _u(0x09)
#define PICOBIN_BLOCK_ITEM_PARTITION_TABLE              _u(0x0a)
#define PICOBIN_BLOCK_ITEM_HASH_VALUE                   _u(0x4b)
#define PICOBIN_BLOCK_ITEM_SALT                         _u(0x0c)

#define PICOBIN_BLOCK_ITEM_2BS_IGNORED          (_u(0x80) | _u(0x7e))
#define PICOBIN_BLOCK_ITEM_2BS_LAST             (_u(0x80) | _u(0x7f))

// ----

#define PICOBIN_INDEX_TO_BITS(y, x) (y ## _ ## x << y ## _LSB)

#define PICOBIN_IMAGE_TYPE_IMAGE_TYPE_LSB            _u(0)
#define PICOBIN_IMAGE_TYPE_IMAGE_TYPE_BITS           _u(0x000f)
#define PICOBIN_IMAGE_TYPE_IMAGE_TYPE_INVALID        _u(0x0)
#define PICOBIN_IMAGE_TYPE_IMAGE_TYPE_EXE            _u(0x1)
#define PICOBIN_IMAGE_TYPE_IMAGE_TYPE_DATA           _u(0x2)
#define PICOBIN_IMAGE_TYPE_IMAGE_TYPE_AS_BITS(x) PICOBIN_INDEX_TO_BITS(PICOBIN_IMAGE_TYPE_IMAGE_TYPE, x)

#define PICOBIN_IMAGE_TYPE_EXE_SECURITY_LSB          _u(4)
#define PICOBIN_IMAGE_TYPE_EXE_SECURITY_BITS         _u(0x0030)
#define PICOBIN_IMAGE_TYPE_EXE_SECURITY_UNSPECIFIED  _u(0x0)
#define PICOBIN_IMAGE_TYPE_EXE_SECURITY_NS           _u(0x1)
#define PICOBIN_IMAGE_TYPE_EXE_SECURITY_S            _u(0x2)
#define PICOBIN_IMAGE_TYPE_EXE_SECURITY_AS_BITS(x) PICOBIN_INDEX_TO_BITS(PICOBIN_IMAGE_TYPE_EXE_SECURITY, x)

#define PICOBIN_IMAGE_TYPE_EXE_CPU_LSB               _u(8)
#define PICOBIN_IMAGE_TYPE_EXE_CPU_BITS              _u(0x0700)
#define PICOBIN_IMAGE_TYPE_EXE_CPU_ARM               _u(0)
#define PICOBIN_IMAGE_TYPE_EXE_CPU_RISCV             _u(1)
#define PICOBIN_IMAGE_TYPE_EXE_CPU_VARMULET          _u(2)
#define PICOBIN_IMAGE_TYPE_EXE_CPU_AS_BITS(x) PICOBIN_INDEX_TO_BITS(PICOBIN_IMAGE_TYPE_EXE_CPU, x)

#define PICOBIN_IMAGE_TYPE_EXE_CHIP_LSB              _u(12)
#define PICOBIN_IMAGE_TYPE_EXE_CHIP_BITS             _u(0x7000)
#define PICOBIN_IMAGE_TYPE_EXE_CHIP_RP2040           _u(0)
#define PICOBIN_IMAGE_TYPE_EXE_CHIP_RP2350           _u(1)
#define PICOBIN_IMAGE_TYPE_EXE_CHIP_AS_BITS(x) PICOBIN_INDEX_TO_BITS(PICOBIN_IMAGE_TYPE_EXE_CHIP, x)

#define PICOBIN_IMAGE_TYPE_EXE_TBYB_BITS             _u(0x8000)

// todo assert no overlap ^

#define PICOBIN_PARTITION_PERMISSIONS_LSB                                   _u(26)
#define PICOBIN_PARTITION_PERMISSIONS_BITS                                  _u(0xfc000000)

#define PICOBIN_PARTITION_PERMISSION_S_R_BITS                               _u(0x04000000)
#define PICOBIN_PARTITION_PERMISSION_S_W_BITS                               _u(0x08000000)
#define PICOBIN_PARTITION_PERMISSION_NS_R_BITS                              _u(0x10000000)
#define PICOBIN_PARTITION_PERMISSION_NS_W_BITS                              _u(0x20000000)
#define PICOBIN_PARTITION_PERMISSION_NSBOOT_R_BITS                          _u(0x40000000)
#define PICOBIN_PARTITION_PERMISSION_NSBOOT_W_BITS                          _u(0x80000000)

#define PICOBIN_PARTITION_LOCATION_FIRST_SECTOR_LSB                         _u(0)
#define PICOBIN_PARTITION_LOCATION_FIRST_SECTOR_BITS                        _u(0x00001fff)
#define PICOBIN_PARTITION_LOCATION_LAST_SECTOR_LSB                          _u(13)
#define PICOBIN_PARTITION_LOCATION_LAST_SECTOR_BITS                         _u(0x03ffe000)

#define PICOBIN_PARTITION_FLAGS_HAS_ID_BITS                                 _u(0x00000001)
#define PICOBIN_PARTITION_FLAGS_LINK_TYPE_LSB                               _u(1)
#define PICOBIN_PARTITION_FLAGS_LINK_TYPE_BITS                              _u(0x00000006)
#define PICOBIN_PARTITION_FLAGS_LINK_VALUE_LSB                              _u(3)
#define PICOBIN_PARTITION_FLAGS_LINK_VALUE_BITS                             _u(0x00000078)

#define PICOBIN_PARTITION_MAX_EXTRA_FAMILIES                                _u(3)
#define PICOBIN_PARTITION_FLAGS_ACCEPTS_NUM_EXTRA_FAMILIES_LSB              _u(7)
#define PICOBIN_PARTITION_FLAGS_ACCEPTS_NUM_EXTRA_FAMILIES_BITS             _u(0x00000180)
// these are an optimization when booting in either ARM or RISC-V, to avoid looking at partitions
// which are known not to contain the right sort of binary, OR as a way to prevent
// auto-architecture-switch. NOTE: the first partition that can be booted, will be,
// so if you have a RISC-V binary in the first partition, and auto-arhcitecture-switch enabled, then
// even if booting under ARM, with an ARM binary in a later partition, the RISC-V binary
// will be booted by default; setting PICOBIN_PARTITION_FLAGS_IGNORED_DURING_ARM_BOOT_BITS
// on the partition, will have the RISC-V binary containing partition ignored under ARM
// boot
#define PICOBIN_PARTITION_FLAGS_IGNORED_DURING_ARM_BOOT_BITS                _u(0x00000200)
#define PICOBIN_PARTITION_FLAGS_IGNORED_DURING_RISCV_BOOT_BITS              _u(0x00000400)
#define PICOBIN_PARTITION_FLAGS_UF2_DOWNLOAD_AB_NON_BOOTABLE_OWNER_AFFINITY _u(0x00000800)
#define PICOBIN_PARTITION_FLAGS_HAS_NAME_BITS                               _u(0x00001000)
#define PICOBIN_PARTITION_FLAGS_UF2_DOWNLOAD_NO_REBOOT_BITS                 _u(0x00002000)
// we have a bit for each well known family-id .. note we expect there to be more in the future with new chips,
// but we have plenty of space for now.
#define PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILIES_LSB                _u(14)
#define PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2040_BITS          _u(0x00004000)
#define PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_ABSOLUTE_BITS        _u(0x00008000)
#define PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_DATA_BITS            _u(0x00010000)
#define PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2350_ARM_S_BITS    _u(0x00020000)
#define PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2350_RISCV_BITS    _u(0x00040000)
#define PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2350_ARM_NS_BITS   _u(0x00080000)

#define PICOBIN_PARTITION_FLAGS_LINK_TYPE_NONE                        _u(0)
#define PICOBIN_PARTITION_FLAGS_LINK_TYPE_A_PARTITION                 _u(1)
#define PICOBIN_PARTITION_FLAGS_LINK_TYPE_OWNER_PARTITION             _u(2)
#define PICOBIN_PARTITION_FLAGS_LINK_TYPE_AS_BITS(x) PICOBIN_INDEX_TO_BITS(PICOBIN_PARTITION_FLAGS_LINK_TYPE, x)


#define PICOBIN_HASH_SHA256                    _u(0x01)

#define PICOBIN_SIGNATURE_SECP256K1            _u(0x01)

#ifndef __ASSEMBLER__

#include <stdbool.h>

typedef struct {
    // these must all be word aligned
    uint32_t storage_address_rel;
    uint32_t runtime_address;
    uint32_t size;
} picobin_load_map_entry;

typedef struct {
    uint32_t header;
    picobin_load_map_entry entries[];
} picobin_load_map;

static inline unsigned int picobin_load_map_entry_count(const picobin_load_map *lm) {
    return (lm->header << 1) >> 25;
}

static inline bool picobin_load_map_is_relative(const picobin_load_map *lm) {
    return (int32_t)lm->header >= 0;
}

#endif

#endif
