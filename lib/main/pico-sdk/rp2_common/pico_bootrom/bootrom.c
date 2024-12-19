/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/bootrom.h"
#include "boot/picoboot.h"
#include "boot/picobin.h"

/// \tag::table_lookup[]

void *rom_func_lookup(uint32_t code) {
    return rom_func_lookup_inline(code);
}

#pragma GCC diagnostic push
// diagnostic: GCC thinks near-zero value is a null pointer member access, but it's not
#pragma GCC diagnostic ignored "-Warray-bounds"
void *rom_data_lookup(uint32_t code) {
#if PICO_RP2040
    rom_table_lookup_fn rom_table_lookup = (rom_table_lookup_fn) rom_hword_as_ptr(BOOTROM_TABLE_LOOKUP_OFFSET);
    uint16_t *data_table = (uint16_t *) rom_hword_as_ptr(BOOTROM_DATA_TABLE_OFFSET);
    return rom_table_lookup(data_table, code);
#else
    rom_table_lookup_fn rom_table_lookup = (rom_table_lookup_fn) (uintptr_t)*(uint16_t*)(BOOTROM_TABLE_LOOKUP_OFFSET);
    return rom_table_lookup(code, RT_FLAG_DATA);
#endif
}
#pragma GCC diagnostic pop
/// \end::table_lookup[]

bool rom_funcs_lookup(uint32_t *table, unsigned int count) {
    bool ok = true;
    for (unsigned int i = 0; i < count; i++) {
        table[i] = (uintptr_t) rom_func_lookup(table[i]);
        if (!table[i]) ok = false;
    }
    return ok;
}


void __attribute__((noreturn)) rom_reset_usb_boot(uint32_t usb_activity_gpio_pin_mask, uint32_t disable_interface_mask) {
#ifdef ROM_FUNC_RESET_USB_BOOT
    rom_reset_usb_boot_fn func = (rom_reset_usb_boot_fn) rom_func_lookup(ROM_FUNC_RESET_USB_BOOT);
    func(usb_activity_gpio_pin_mask, disable_interface_mask);
#elif defined(ROM_FUNC_REBOOT)
    uint32_t flags = disable_interface_mask;
    if (usb_activity_gpio_pin_mask) {
        flags |= BOOTSEL_FLAG_GPIO_PIN_SPECIFIED;
        // the parameter is actually the gpio number, but we only care if BOOTSEL_FLAG_GPIO_PIN_SPECIFIED
        usb_activity_gpio_pin_mask = (uint32_t)__builtin_ctz(usb_activity_gpio_pin_mask);
    }
    rom_reboot(REBOOT2_FLAG_REBOOT_TYPE_BOOTSEL | REBOOT2_FLAG_NO_RETURN_ON_SUCCESS, 10, flags, usb_activity_gpio_pin_mask);
    __builtin_unreachable();
#else
    panic_unsupported();
#endif
}

#if !PICO_RP2040
bool rom_get_boot_random(uint32_t out[4]) {
    uint32_t result[5];
    rom_get_sys_info_fn func = (rom_get_sys_info_fn) rom_func_lookup_inline(ROM_FUNC_GET_SYS_INFO);
    if (5 == func(result, count_of(result), SYS_INFO_BOOT_RANDOM)) {
        for(uint i=0;i<4;i++) {
            out[i] = result[i+1];
        }
        return true;
    }
    return false;
}

int rom_add_flash_runtime_partition(uint32_t start_offset, uint32_t size, uint32_t permissions) {
    if ((start_offset) & 4095 || (size & 4095)) return PICO_ERROR_BAD_ALIGNMENT;
    if (!size || start_offset + size > 32 * 1024 * 1024) return PICO_ERROR_INVALID_ARG;
    if (permissions & ~PICOBIN_PARTITION_PERMISSIONS_BITS) return PICO_ERROR_INVALID_ARG;

    void **ptr = (void **)rom_data_lookup(ROM_DATA_PARTITION_TABLE_PTR);
    assert(ptr);
    assert(*ptr);
    struct pt {
        struct {
            uint8_t partition_count;
            uint8_t permission_partition_count; // >= partition_count and includes any regions added at runtime
            bool loaded;
        };
        uint32_t unpartitioned_space_permissions_and_flags;
        resident_partition_t partitions[PARTITION_TABLE_MAX_PARTITIONS];
    } *pt = (struct pt *)*ptr;
    assert(pt->loaded); // even if empty it should have been populated by the bootrom
    if (pt->permission_partition_count < pt->partition_count) pt->permission_partition_count = pt->partition_count;
    if (pt->permission_partition_count < PARTITION_TABLE_MAX_PARTITIONS) {
        pt->partitions[pt->permission_partition_count].permissions_and_location = permissions |
                ((start_offset / 4096) << PICOBIN_PARTITION_LOCATION_FIRST_SECTOR_LSB) |
                ((start_offset + size - 4096) / 4096) << PICOBIN_PARTITION_LOCATION_LAST_SECTOR_LSB;
        pt->partitions[pt->permission_partition_count].permissions_and_flags = permissions;
        return pt->permission_partition_count++;
    }
    return PICO_ERROR_INSUFFICIENT_RESOURCES;
}
#endif