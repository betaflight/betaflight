/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_BOOTROM_H
#define _PICO_BOOTROM_H

#include "pico.h"
#include "pico/bootrom_constants.h"

/** \file bootrom.h
 * \defgroup pico_bootrom pico_bootrom
 * \brief Access to functions and data in the bootrom
 *
 * This header may be included by assembly code
 */

#ifndef __ASSEMBLER__
#include <string.h>
#include "pico/bootrom/lock.h"
#include "pico/flash.h"
// ROM FUNCTION SIGNATURES

#if PICO_RP2040
typedef uint32_t (*rom_popcount32_fn)(uint32_t);
typedef uint32_t (*rom_reverse32_fn)(uint32_t);
typedef uint32_t (*rom_clz32_fn)(uint32_t);
typedef uint32_t (*rom_ctz32_fn)(uint32_t);
typedef uint8_t *(*rom_memset_fn)(uint8_t *, uint8_t, uint32_t);
typedef uint32_t *(*rom_memset4_fn)(uint32_t *, uint8_t, uint32_t);
typedef uint32_t *(*rom_memcpy_fn)(uint8_t *, const uint8_t *, uint32_t);
typedef uint32_t *(*rom_memcpy44_fn)(uint32_t *, const uint32_t *, uint32_t);
#endif
typedef void __attribute__((noreturn)) (*rom_reset_usb_boot_fn)(uint32_t, uint32_t);
typedef int (*rom_reboot_fn)(uint32_t flags, uint32_t delay_ms, uint32_t p0, uint32_t p1);
typedef rom_reset_usb_boot_fn reset_usb_boot_fn; // kept for backwards compatibility
typedef void (*rom_connect_internal_flash_fn)(void);
typedef void (*rom_flash_exit_xip_fn)(void);
typedef void (*rom_flash_range_erase_fn)(uint32_t, size_t, uint32_t, uint8_t);
typedef void (*rom_flash_range_program_fn)(uint32_t, const uint8_t*, size_t);
typedef void (*rom_flash_flush_cache_fn)(void);
typedef void (*rom_flash_enter_cmd_xip_fn)(void);
#if !PICO_RP2040
typedef void (*rom_bootrom_state_reset_fn)(uint32_t flags);
typedef void (*rom_flash_reset_address_trans_fn)(void);
typedef void (*rom_flash_select_xip_read_mode_fn)(bootrom_xip_mode_t mode, uint8_t clkdiv);
typedef int (*rom_get_sys_info_fn)(uint32_t *out_buffer, uint32_t out_buffer_word_size, uint32_t flags);
typedef int (*rom_get_partition_table_info_fn)(uint32_t *out_buffer, uint32_t out_buffer_word_size, uint32_t partition_and_flags);
typedef int (*rom_explicit_buy_fn)(uint8_t *buffer, uint32_t buffer_size);
typedef void* (*rom_validate_ns_buffer_fn)(const void *addr, uint32_t size, uint32_t write, uint32_t *ok);
/**
 * @return BOOTROM_OK if successful
 *         BOOTROM_ERROR_INVALID_ARG if ns_api_num is out of range
 */
typedef intptr_t (*rom_set_rom_callback_fn)(uint callback_num, bootrom_api_callback_generic_t funcptr);
typedef int (*rom_chain_image_fn)(uint8_t *workarea_base, uint32_t workarea_size, uint32_t window_base, uint32_t window_size);
typedef int (*rom_load_partition_table_fn)(uint8_t *workarea_base, uint32_t workarea_size, bool force_reload);
typedef int (*rom_pick_ab_partition_fn)(uint8_t *workarea_base, uint32_t workarea_size, uint partition_a_num, uint32_t flash_update_boot_window_base);
typedef int (*rom_get_b_partition_fn)(uint pi_a);
typedef int (*rom_get_uf2_target_partition_fn)(uint8_t *workarea_base, uint32_t workarea_size, uint32_t family_id, resident_partition_t *partition_out);
typedef int (*rom_func_otp_access_fn)(uint8_t *buf, uint32_t buf_len, otp_cmd_t cmd);
// Apply the address translation currently specified in QMI_ATRANSx ("rolling window" hardware
// translation). Need to take care using this on the boot path, as the QMI may not yet have been
// set up, but this should be suitable for translating system bus addresses into flash storage
// addresses in user callbacks. Returns all-ones for an invalid address, which is also an invalid
// flash storage address, so invalidity is propagated.
typedef intptr_t (*rom_flash_runtime_to_storage_addr_fn)(uintptr_t flash_runtime_addr);

// Perform the specified erase/program/read operation, translating addresses according to
// QMI_ATRANSx if necessary, and checking flash permissions based on the resident partition table
// and the specified effective security level. `addr` may be either a flash runtime address or a
// flash storage address, depending on the ASPACE given in `flags`.
//
// NOTE: This function does not validate the buffer for NS access. This must be validated before
// calling if the caller is reachable from a Secure Gateway.
typedef int (*rom_flash_op_fn)(cflash_flags_t flags, uintptr_t addr, uint32_t size_bytes, uint8_t *buf);

#ifndef __riscv
typedef int (*rom_set_ns_api_permission_fn)(uint ns_api_num, bool allowed);
/**
 * Note this is not strictly a C function; you must pass the function you are calling in r4
 * @param in_r4
 * `0b0xxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx` - a "well known" function selector; do not use for your own methods
 * `0b10xx xxxx xxxx xxxx xxxx xxxx xxxx xxxx` - a "unique" function selector intended to be unlikely to clash with others'.
 *                                               The lower 30 bits should be chosen at random
 * `0b11xx xxxx xxxx xxxx xxxx xxxx xxxx xxxx` - a "private" function selector intended for use by tightly coupled NS and S code
 *
 * @return whatever the secure call returns
 *         BOOTROM_ERROR_INVALID_STATE if no secure handler has been set from the secure side
 *            via rom_set_rom_callback_fn(BOOTROM_API_CALLBACK_secure_call, ...)
 */
typedef int (*rom_func_secure_call)(uintptr_t a0, ...);
#endif

#ifdef __riscv
typedef struct {
    uint32_t *base;
    uint32_t size;
} bootrom_stack_t;
// passed in, and out.
typedef int (*rom_set_bootrom_stack_fn)(bootrom_stack_t *stack);
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Return a bootrom lookup code based on two ASCII characters
 * \ingroup pico_bootrom
 *
 * These codes are uses to lookup data or function addresses in the bootrom
 *
 * \param c1 the first character
 * \param c2 the second character
 * \return the 'code' to use in rom_func_lookup() or rom_data_lookup()
 */
static inline uint32_t rom_table_code(uint8_t c1, uint8_t c2) {
    return ROM_TABLE_CODE((uint32_t) c1, (uint32_t) c2);
}

/*!
 * \brief Lookup a bootrom function by its code
 * \ingroup pico_bootrom
 * \param code the code
 * \return a pointer to the function, or NULL if the code does not match any bootrom function
 */
void *rom_func_lookup(uint32_t code);

/*!
 * \brief Lookup a bootrom data address by its code
 * \ingroup pico_bootrom
 * \param code the code
 * \return a pointer to the data, or NULL if the code does not match any bootrom function
 */
void *rom_data_lookup(uint32_t code);

/*!
 * \brief Helper function to lookup the addresses of multiple bootrom functions
 * \ingroup pico_bootrom
 *
 * This method looks up the 'codes' in the table, and convert each table entry to the looked up
 * function pointer, if there is a function for that code in the bootrom.
 *
 * \param table an IN/OUT array, elements are codes on input, function pointers on success.
 * \param count the number of elements in the table
 * \return true if all the codes were found, and converted to function pointers, false otherwise
 */
bool rom_funcs_lookup(uint32_t *table, unsigned int count);

// Bootrom function: rom_table_lookup
// Returns the 32 bit pointer into the ROM if found or NULL otherwise.
#if PICO_RP2040
typedef void *(*rom_table_lookup_fn)(uint16_t *table, uint32_t code);
#else
typedef void *(*rom_table_lookup_fn)(uint32_t code, uint32_t mask);
#endif

#if PICO_C_COMPILER_IS_GNU && (__GNUC__ >= 12)
// Convert a 16 bit pointer stored at the given rom address into a 32 bit pointer
__force_inline static void *rom_hword_as_ptr(uint16_t rom_address) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
    return (void *)(uintptr_t)*(uint16_t *)(uintptr_t)rom_address;
#pragma GCC diagnostic pop
}
#else
// Convert a 16 bit pointer stored at the given rom address into a 32 bit pointer
#define rom_hword_as_ptr(rom_address) (void *)(uintptr_t)(*(uint16_t *)(uintptr_t)(rom_address))
#endif

#ifdef __riscv
static __force_inline bool rom_size_is_64k(void) {
#ifdef RASPBERRYPI_AMETHYST_FPGA
    // Detect ROM size by testing for bus fault at +32k
    uint result;
    pico_default_asm_volatile (
        "li %0, 0\n"
        // Save and disable IRQs before touching trap vector
        "csrr t2, mstatus\n"
        "csrci mstatus, 0x8\n"
        // Set up trap vector to skip the instruction which sets the %0 flag
        "la t0, 1f\n"
        "csrrw t0, mtvec, t0\n"
        // This load will fault if the bootrom is no larger than 32k:
        "li t1, 32 * 1024\n"
        "lw t1, (t1)\n"
        // No fault, so set return to true
        "li %0, 1\n"
        ".p2align 2\n"
        // Always end up back here, restore the trap table
        "1:\n"
        "csrw mtvec, t0\n"
        // Now safe to restore interrupts
        "csrw mstatus, t2\n"
        : "=r" (result)
        :
        : "t0", "t1", "t2"
    );
    return result;
#else
    return false;
#endif
}
#endif

/*!
 * \brief Lookup a bootrom function by code. This method is forcibly inlined into the caller for FLASH/RAM sensitive code usage
 * \ingroup pico_bootrom
 * \param code the code
 * \return a pointer to the function, or NULL if the code does not match any bootrom function
 */
#pragma GCC diagnostic push
// diagnostic: GCC thinks near-zero value is a null pointer member access, but it's not
#pragma GCC diagnostic ignored "-Warray-bounds"
static __force_inline void *rom_func_lookup_inline(uint32_t code) {
#if PICO_RP2040
    rom_table_lookup_fn rom_table_lookup = (rom_table_lookup_fn) rom_hword_as_ptr(BOOTROM_TABLE_LOOKUP_OFFSET);
    uint16_t *func_table = (uint16_t *) rom_hword_as_ptr(BOOTROM_FUNC_TABLE_OFFSET);
    return rom_table_lookup(func_table, code);
#else
#ifdef __riscv
    uint32_t rom_offset_adjust = rom_size_is_64k() ? 32 * 1024 : 0;
    // on RISC-V the code (a jmp) is actually embedded in the table
    rom_table_lookup_fn rom_table_lookup = (rom_table_lookup_fn) (uintptr_t)*(uint16_t*)(BOOTROM_TABLE_LOOKUP_ENTRY_OFFSET + rom_offset_adjust);
    return rom_table_lookup(code, RT_FLAG_FUNC_RISCV);
#else
    // on ARM the function pointer is stored in the table, so we dereference it
    // via lookup() rather than lookup_entry()
    rom_table_lookup_fn rom_table_lookup = (rom_table_lookup_fn) (uintptr_t)*(uint16_t*)(BOOTROM_TABLE_LOOKUP_OFFSET);
    if (pico_processor_state_is_nonsecure()) {
        return rom_table_lookup(code, RT_FLAG_FUNC_ARM_NONSEC);
    } else {
        return rom_table_lookup(code, RT_FLAG_FUNC_ARM_SEC);
    }
#endif
#endif
}
#pragma GCC diagnostic pop

/*!
 * \brief Reboot the device into BOOTSEL mode
 * \ingroup pico_bootrom
 *
 * This function reboots the device into the BOOTSEL mode ('usb boot").
 *
 * Facilities are provided to enable an "activity light" via GPIO attached LED for the USB Mass Storage Device,
 * and to limit the USB interfaces exposed.
 *
 * \param usb_activity_gpio_pin_mask 0 No pins are used as per a cold boot. Otherwise a single bit set indicating which
 *                               GPIO pin should be set to output and raised whenever there is mass storage activity
 *                               from the host.
 * \param disable_interface_mask value to control exposed interfaces
 *  - 0 To enable both interfaces (as per a cold boot)
 *  - 1 To disable the USB Mass Storage Interface
 *  - 2 To disable the USB PICOBOOT Interface
 */
void __attribute__((noreturn)) rom_reset_usb_boot(uint32_t usb_activity_gpio_pin_mask, uint32_t disable_interface_mask);
static inline void __attribute__((noreturn)) reset_usb_boot(uint32_t usb_activity_gpio_pin_mask, uint32_t disable_interface_mask) {
    rom_reset_usb_boot(usb_activity_gpio_pin_mask, disable_interface_mask);
}

/*!
 * \brief Connect the SSI/QMI to the QSPI pads
 * \ingroup pico_bootrom
 * 
 * Restore all QSPI pad controls to their default state, and connect the SSI/QMI peripheral to the QSPI pads.
 * 
 * \if rp2350_specific
 * On RP2350 if a secondary flash chip select GPIO has been configured via OTP OTP_DATA_FLASH_DEVINFO, or by writing to the runtime
 * copy of FLASH_DEVINFO in bootram, then this bank 0 GPIO is also initialised and the QMI peripheral is connected. Otherwise,
 * bank 0 IOs are untouched.
 * \endif
 */
static inline void rom_connect_internal_flash(void) {
    rom_connect_internal_flash_fn func = (rom_connect_internal_flash_fn) rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    func();
}

/*!
 * \brief Return the QSPI device from its XIP state to a serial command state
 * \ingroup pico_bootrom
 * 
 * \if rp2040_specific
 * On RP2040, first set up the SSI for serial-mode operations, then issue the fixed XIP exit sequence described in Section 2.8.1.2
 * of the datasheet. Note that the bootrom code uses the IO forcing logic to drive the CS pin, which must be cleared before returning
 * the SSI to XIP mode (e.g. by a call to _flash_flush_cache). This function configures the SSI with a fixed SCK clock divisor of /6.
 * \endif
 * 
 * \if rp2350_specific
 * On RP2350, Initialise the QMI for serial operations (direct mode), and also initialise a basic XIP mode, where the QMI will perform
 * 03h serial read commands at low speed (CLKDIV=12) in response to XIP reads.
 * 
 * Then, issue a sequence to the QSPI device on chip select 0, designed to return it from continuous read mode ("XIP mode") and/or
 * QPI mode to a state where it will accept serial commands. This is necessary after system reset to restore the QSPI device to a known
 * state, because resetting RP2350 does not reset attached QSPI devices. It is also necessary when user code, having already performed
 * some continuous-read-mode or QPI-mode accesses, wishes to return the QSPI device to a state where it will accept the serial erase and
 * programming commands issued by the bootrom's flash access functions.
 * 
 * If a GPIO for the secondary chip select is configured via FLASH_DEVINFO, then the XIP exit sequence is also issued to chip select 1.
 * 
 * The QSPI device should be accessible for XIP reads after calling this function; the name flash_exit_xip refers to returning the QSPI
 * device from its XIP state to a serial command state.
 * \endif
 */
static inline void rom_flash_exit_xip(void) {
    rom_flash_exit_xip_fn func = (rom_flash_exit_xip_fn) rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    func();
}

/*!
 * \brief Erase bytes in flash
 * \ingroup pico_bootrom
 * 
 * Erase count bytes, starting at addr (offset from start of flash). Optionally, pass a block erase command e.g. D8h block erase,
 * and the size of the block erased by this command - this function will use the larger block erase where possible, for much higher
 * erase speed. addr must be aligned to a 4096-byte sector, and count must be a multiple of 4096 bytes.
 * 
 * This is a low-level flash API, and no validation of the arguments is performed.
 * 
 * \if rp2350_specific
 * See rom_flash_op on RP2350 for a higher-level API which checks alignment, flash bounds and partition permissions, and can transparently
 * apply a runtime-to-storage address translation.
 * 
 * The QSPI device must be in a serial command state before calling this API, which can be achieved by calling rom_connect_internal_flash()
 * followed by rom_flash_exit_xip(). After the erase, the flash cache should be flushed via rom_flash_flush_cache() to ensure the modified
 * flash data is visible to cached XIP accesses.
 * 
 * Finally, the original XIP mode should be restored by copying the saved XIP setup function from bootram into SRAM, and executing it:
 * the bootrom provides a default function which restores the flash mode/clkdiv discovered during flash scanning, and user programs can
 * override this with their own XIP setup function.
 * 
 * For the duration of the erase operation, QMI is in direct mode and attempting to access XIP from DMA, the debugger or the other core will
 * return a bus fault. XIP becomes accessible again once the function returns.
 * \endif
 * 
 * \param addr the offset from start of flash to be erased
 * \param count number of bytes to erase
 * \param block_size optional size of block erased by block_cmd
 * \param block_cmd optional block erase command e.g. D8h block erase
 */
static inline void rom_flash_range_erase(uint32_t addr, size_t count, uint32_t block_size, uint8_t block_cmd) {
    rom_flash_range_erase_fn func = (rom_flash_range_erase_fn) rom_func_lookup_inline(ROM_FUNC_FLASH_RANGE_ERASE);
    func(addr, count, block_size, block_cmd);
}

/*!
 * \brief Program bytes in flash
 * \ingroup pico_bootrom
 * 
 * Program data to a range of flash addresses starting at addr (offset from the start of flash) and count bytes in size. addr must be
 * aligned to a 256-byte boundary, and count must be a multiple of 256.
 * 
 * This is a low-level flash API, and no validation of the arguments is performed.
 * 
 * \if rp2350_specific
 * See rom_flash_op on RP2350 for a higher-level API which checks alignment, flash bounds and partition permissions,
 * and can transparently apply a runtime-to-storage address translation.
 * 
 * The QSPI device must be in a serial command state before calling this API - see notes on rom_flash_range_erase
 * \endif
 * 
 * \param addr the offset from start of flash to be erased
 * \param data buffer containing the data to be written
 * \param count number of bytes to erase
 */
static inline void rom_flash_range_program(uint32_t addr, const uint8_t *data, size_t count) {
    rom_flash_range_program_fn func = (rom_flash_range_program_fn) rom_func_lookup_inline(ROM_FUNC_FLASH_RANGE_PROGRAM);
    func(addr, data, count);
}

/*!
 * \brief Flush the XIP cache
 * \ingroup pico_bootrom
 * 
 * \if rp2040_specific
 * Flush and enable the XIP cache. Also clears the IO forcing on QSPI CSn, so that the SSI can drive the flash chip select as normal.
 * \endif
 * 
 * \if rp2350_specific
 * Flush the entire XIP cache, by issuing an invalidate by set/way maintenance operation to every cache line. This ensures that flash
 * program/erase operations are visible to subsequent cached XIP reads.
 * 
 * Note that this unpins pinned cache lines, which may interfere with cache-as-SRAM use of the XIP cache.
 * 
 * No other operations are performed.
 * \endif
 */
static inline void rom_flash_flush_cache(void) {
    rom_flash_flush_cache_fn func = (rom_flash_flush_cache_fn) rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);
    func();
}

/*!
 * \brief Configure the SSI/QMI with a standard command
 * \ingroup pico_bootrom
 * 
 * Configure the SSI/QMI to generate a standard 03h serial read command, with 24 address bits, upon each XIP access. This is a slow XIP
 * configuration, but is widely supported. CLKDIV is set to 12 on RP2350. The debugger may call this function to ensure that flash is
 * readable following a program/erase operation.
 * 
 * Note that the same setup is performed by flash_exit_xip(), and the RP2350 flash program/erase functions do not leave XIP in an
 * inaccessible state, so calls to this function are largely redundant on RP2350. It is provided on RP2350 for compatibility with RP2040.
 */
static inline void rom_flash_enter_cmd_xip(void) {
    rom_flash_enter_cmd_xip_fn func = (rom_flash_enter_cmd_xip_fn) rom_func_lookup_inline(ROM_FUNC_FLASH_ENTER_CMD_XIP);
    func();
}

#if !PICO_RP2040
#ifdef __riscv
/*!
 * \brief Give the bootrom a new stack
 * \ingroup pico_bootrom
 * 
 * Most bootrom functions are written just once, in Arm code, to save space. As a result these functions are emulated when
 * running under the RISC-V architecture. This is largely transparent to the user, however the stack used by the Arm emulation
 * is separate from the calling user's stack, and is stored in boot RAM but is of quite limited size. When using certain of the more
 * complex APIs or if nesting bootrom calls from within IRQs, you may need to provide a large stack.
 * 
 * This method allows the caller to specify a region of RAM to use as the stack for the current core by passing a pointer to two values: the word aligned base address,
 * and the size in bytes (multiple of 4).
 * 
 * The method fills in the previous base/size values into the passed array before returning.
 * 
 * \param stack bootrom_stack_t struct containing base and size
 */
static inline int rom_set_bootrom_stack(bootrom_stack_t *stack) {
    rom_set_bootrom_stack_fn func = (rom_set_bootrom_stack_fn) rom_func_lookup_inline(ROM_FUNC_SET_BOOTROM_STACK);
    return func(stack);
}
#endif

/*!
 * \brief Reboot using the watchdog
 * \ingroup pico_bootrom
 * 
 * Resets the chip and uses the watchdog facility to restart.
 * 
 * The delay_ms is the millisecond delay before the reboot occurs. Note: by default this method is asynchronous
 * (unless NO_RETURN_ON_SUCCESS is set - see below), so the method will return and the reboot will happen this many milliseconds later.
 * 
 * The flags field contains one of the following values:
 * 
 * REBOOT_TYPE_NORMAL - reboot into the normal boot path.
 * 
 * REBOOT_TYPE_BOOTSEL - reboot into BOOTSEL mode.
 *  p0 - the GPIO number to use as an activity indicator (enabled by flag in p1).
 *  p1 - a set of flags:
 *   0x01 : DISABLE_MSD_INTERFACE - Disable the BOOTSEL USB drive (see <<section_bootrom_mass_storage>>)
 *   0x02 : DISABLE_PICOBOOT_INTERFACE - Disable the {picoboot} interface (see <<section_bootrom_picoboot>>).
 *   0x10 : GPIO_PIN_ACTIVE_LOW - The GPIO in p0 is active low.
 *   0x20 : GPIO_PIN_ENABLED - Enable the activity indicator on the specified GPIO.
 * 
 * REBOOT_TYPE_RAM_IMAGE - reboot into an image in RAM. The region of RAM or XIP RAM is searched for an image to run. This is the type
 * of reboot used when a RAM UF2 is dragged onto the BOOTSEL USB drive.
 *  p0 - the region start address (word-aligned).
 *  p1 - the region size (word-aligned).
 * 
 * REBOOT_TYPE_FLASH_UPDATE - variant of REBOOT_TYPE_NORMAL to use when flash has been updated. This is the type
 * of reboot used after dragging a flash UF2 onto the BOOTSEL USB drive.
 *  p0 - the address of the start of the region of flash that was updated. If this address matches the start address of a partition or slot, then that
 *       partition or slot is treated preferentially during boot (when there is a choice). This type of boot facilitates TBYB and version downgrades.
 * 
 * REBOOT_TYPE_PC_SP - reboot to a specific PC and SP. Note: this is not allowed in the ARM-NS variant.
 *  p0 - the initial program counter (PC) to start executing at. This must have the lowest bit set for Arm and clear for RISC-V
 *  p1 - the initial stack pointer (SP).
 * 
 * All of the above, can have optional flags ORed in:
 * 
 * REBOOT_TO_ARM - switch both cores to the Arm architecture (rather than leaving them as is). The call will fail with BOOTROM_ERROR_INVALID_STATE if the Arm architecture is not supported.
 * REBOOT_TO_RISCV - switch both cores to the RISC-V architecture (rather than leaving them as is). The call will fail with BOOTROM_ERROR_INVALID_STATE if the RISC-V architecture is not supported.
 * NO_RETURN_ON_SUCCESS - the watchdog h/w is asynchronous. Setting this bit forces this method not to return if the reboot is successfully initiated.
 * 
 * \param flags the reboot flags, as detailed above
 * \param delay_ms millisecond delay before the reboot occurs
 * \param p0 parameter 0, depends on flags
 * \param p1 parameter 1, depends on flags
 */
static inline int rom_reboot(uint32_t flags, uint32_t delay_ms, uint32_t p0, uint32_t p1) {
    rom_reboot_fn func = (rom_reboot_fn) rom_func_lookup_inline(ROM_FUNC_REBOOT);
    return func(flags, delay_ms, p0, p1);
}

bool rom_get_boot_random(uint32_t out[4]);

/*!
 * \brief Reset bootrom state
 * \ingroup pico_bootrom
 * 
 * Resets internal bootrom state, based on the following flags:
 * 
 * STATE_RESET_CURRENT_CORE - Resets any internal bootrom state for the current core into a clean state.
 * This method should be called prior to calling any other bootrom APIs on the current core,
 * and is called automatically by the bootrom during normal boot of core 0 and launch of code on core 1.
 * 
 * STATE_RESET_OTHER_CORE - Resets any internal bootrom state for the other core into a clean state. This is generally called by
 * a debugger when resetting the state of one core via code running on the other.
 * 
 * STATE_RESET_GLOBAL_STATE - Resets all non core-specific state, including:
 *  Disables access to bootrom APIs from ARM-NS
 *  Unlocks all BOOT spinlocks
 *  Clears any secure code callbacks
 * 
 * Note: the sdk calls this method on runtime initialisation to put the bootrom into a known state. This
 * allows the program to function correctly if it is entered (e.g. from a debugger) without taking the usual boot path (which
 * resets the state appropriately itself).
 * 
 * \param flags flags, as detailed above
 */
static inline void rom_bootrom_state_reset(uint32_t flags) {
    rom_bootrom_state_reset_fn func = (rom_bootrom_state_reset_fn) rom_func_lookup_inline(ROM_FUNC_BOOTROM_STATE_RESET);
    return func(flags);
}

/*!
 * \brief Reset address translation
 * \ingroup pico_bootrom
 * 
 * Restore the QMI address translation registers, QMI_ATRANS0 through QMI_ATRANS7, to their reset state. This makes the
 * runtime-to-storage address map an identity map, i.e. the mapped and unmapped address are equal, and the entire space is
 * fully mapped.
 */
static inline void rom_flash_reset_address_trans(void) {
    rom_flash_reset_address_trans_fn func = (rom_flash_reset_address_trans_fn) rom_func_lookup_inline(ROM_FUNC_FLASH_RESET_ADDRESS_TRANS);
    func();
}

/*!
 * \brief Configure QMI in a XIP read mode
 * \ingroup pico_bootrom
 * 
 * Configure QMI for one of a small menu of XIP read modes supported by the bootrom. This mode is configured for both memory
 * windows (both chip selects), and the clock divisor is also applied to direct mode.
 * 
 * \param mode bootrom_xip_mode_t mode to use
 * \param clkdiv clock divider
 */
static inline void rom_flash_select_xip_read_mode(bootrom_xip_mode_t mode, uint8_t clkdiv) {
    rom_flash_select_xip_read_mode_fn func = (rom_flash_select_xip_read_mode_fn) rom_func_lookup_inline(ROM_FUNC_FLASH_SELECT_XIP_READ_MODE);
    func(mode, clkdiv);
}

typedef struct {
    cflash_flags_t flags;
    uintptr_t addr;
    uint32_t size_bytes;
    uint8_t *buf;
    int *res;
} rom_helper_flash_op_params_t;

static inline void rom_helper_flash_op(void *param) {
    const rom_helper_flash_op_params_t *op = (const rom_helper_flash_op_params_t *)param;
    rom_flash_op_fn func = (rom_flash_op_fn) rom_func_lookup_inline(ROM_FUNC_FLASH_OP);
    *(op->res) = func(op->flags, op->addr, op->size_bytes, op->buf);
}

/*!
 * \brief Perform a flash read, erase, or program operation
 * \ingroup pico_bootrom
 * 
 * The flash operation is bounds-checked against the known flash devices specified by the runtime value of FLASH_DEVINFO,
 * stored in bootram. This is initialised by the bootrom to the OTP value OTP_DATA_FLASH_DEVINFO, if
 * OTP_DATA_BOOT_FLAGS0_FLASH_DEVINFO_ENABLE is set; otherwise it is initialised to 16 MiB for chip select 0 and 0 bytes
 * for chip select 1. FLASH_DEVINFO can be updated at runtime by writing to its location in bootram, the pointer to which
 * can be looked up in the ROM table.
 * 
 * If a resident partition table is in effect, then the flash operation is also checked against the partition permissions.
 * The Secure version of this function can specify the caller's effective security level (Secure, Non-secure, bootloader)
 * using the CFLASH_SECLEVEL_BITS bitfield of the flags argument, whereas the Non-secure function is always checked against
 * the Non-secure permissions for the partition. Flash operations which span two partitions are not allowed, and will fail
 * address validation.
 * 
 * If OTP_DATA_FLASH_DEVINFO_D8H_ERASE_SUPPORTED is set, erase operations will use a D8h 64 kiB block erase command where
 * possible (without erasing outside the specified region), for faster erase time. Otherwise, only 20h 4 kiB sector erase
 * commands are used.
 * 
 * Optionally, this API can translate addr from flash runtime addresses to flash storage addresses, according to the
 * translation currently configured by QMI address translation registers, QMI_ATRANS0 through QMI_ATRANS7. For example, an
 * image stored at a +2 MiB offset in flash (but mapped at XIP address 0 at runtime), writing to an offset of +1 MiB into
 * the image, will write to a physical flash storage address of 3 MiB. Translation is enabled by setting the
 * CFLASH_ASPACE_BITS bitfield in the flags argument.
 * 
 * When translation is enabled, flash operations which cross address holes in the XIP runtime address space (created by
 * non-maximum ATRANSx_SIZE) will return an error response. This check may tear: the transfer may be partially performed
 * before encountering an address hole and ultimately returning failure.
 * 
 * When translation is enabled, flash operations are permitted to cross chip select boundaries, provided this does not
 * span an ATRANS address hole. When translation is disabled, the entire operation must target a single flash chip select
 * (as determined by bits 24 and upward of the address), else address validation will fail.
 *
 * \param flags controls the security level, address space, and flash operation
 * \param addr the address of the first flash byte to be accessed, ranging from XIP_BASE to XIP_BASE + 0x1ffffff
 * \param size_bytes size of buf, in bytes
 * \param buf contains data to be written to flash, for program operations, and data read back from flash, for read operations
 */
static inline int rom_flash_op(cflash_flags_t flags, uintptr_t addr, uint32_t size_bytes, uint8_t *buf) {
    if (!bootrom_try_acquire_lock(BOOTROM_LOCK_FLASH_OP))
        return BOOTROM_ERROR_LOCK_REQUIRED;
    int rc = 0;
    rom_helper_flash_op_params_t params = {
        .flags = flags,
        .addr = addr,
        .size_bytes = size_bytes,
        .buf = buf,
        .res = &rc
    };
    int flash_rc = flash_safe_execute(rom_helper_flash_op, &params, UINT32_MAX);
    bootrom_release_lock(BOOTROM_LOCK_FLASH_OP);
    if (flash_rc != PICO_OK) {
        return flash_rc;
    } else {
        return rc;
    }
}

/*!
 * \brief Writes data from a buffer into OTP, or reads data from OTP into a buffer
 * \ingroup pico_bootrom
 *
 * The buffer must be aligned to 2 bytes or 4 bytes according to the IS_ECC flag.
 * 
 * This method will read and write rows until the first row it encounters that fails a key or permission check at which
 * it will return BOOTROM_ERROR_NOT_PERMITTED.
 * 
 * Writing will also stop at the first row where an attempt is made to set an OTP bit from a 1 to a 0, and
 * BOOTROM_ERROR_UNSUPPORTED_MODIFICATION will be returned.
 * 
 * If all rows are read/written successfully, then BOOTROM_OK will be returned.
 * 
 * \param buf buffer to read to/write from
 * \param buf_len size of buf
 * \param cmd OTP command to execute
 *  - 0x0000ffff - ROW_NUMBER: 16 low bits are row number (0-4095)
 *  - 0x00010000 - IS_WRITE: if set, do a write (not a read)
 *  - 0x00020000 - IS_ECC: if this bit is set, each value in the buffer is 2 bytes and ECC is used when read/writing from 24
 *      bit value in OTP. If this bit is not set, each value in the buffer is 4 bytes, the low 24-bits of which are written
 *      to or read from OTP.

 */
static inline int rom_func_otp_access(uint8_t *buf, uint32_t buf_len, otp_cmd_t cmd) {
    rom_func_otp_access_fn func = (rom_func_otp_access_fn) rom_func_lookup_inline(ROM_FUNC_OTP_ACCESS);
    if (!bootrom_try_acquire_lock(BOOTROM_LOCK_OTP))
        return BOOTROM_ERROR_LOCK_REQUIRED;
    int rc = func(buf, buf_len, cmd);
    bootrom_release_lock(BOOTROM_LOCK_OTP);
    return rc;
}

/*!
 * \brief Fills a buffer with information from the partition table
 * \ingroup pico_bootrom
 *
 * Fills a buffer with information from the partition table. Note that this API is also used to return information over the
 * picoboot interface.
 * 
 * On success, the buffer is filled, and the number of words filled in the buffer is returned. If the partition table
 * has not been loaded (e.g. from a watchdog or RAM boot), then this method will return BOOTROM_ERROR_NO_DATA, and you
 * should load the partition table via load_partition_table() first.
 * 
 * Note that not all data from the partition table is kept resident in memory by the bootrom due to size constraints.
 * To protect against changes being made in flash after the bootrom has loaded the resident portion, the bootrom keeps
 * a hash of the partition table as of the time it loaded it. If the hash has changed by the time this method is called,
 * then it will return BOOTROM_ERROR_INVALID_STATE.
 * 
 * The information returned is chosen by the flags_and_partition parameter; the first word in the returned buffer,
 * is the (sub)set of those flags that the API supports. You should always check this value before interpreting
 * the buffer.
 * 
 * Following the first word, returns words of data for each present flag in order. With the exception of PT_INFO,
 * all the flags select "per partition" information, so each field is returned in flag order for one partition after
 * the next. The special SINGLE_PARTITION flag indicates that data for only a single partition is required.
 * 
 * \param out_buffer buffer to write data to
 * \param out_buffer_word_size size of out_buffer, in words
 * \param partition_and_flags partition number and flags
 */
static inline int rom_get_partition_table_info(uint32_t *out_buffer, uint32_t out_buffer_word_size, uint32_t partition_and_flags) {
    rom_get_partition_table_info_fn func = (rom_get_partition_table_info_fn) rom_func_lookup_inline(ROM_FUNC_GET_PARTITION_TABLE_INFO);
    if (!bootrom_try_acquire_lock(BOOTROM_LOCK_SHA_256))
        return BOOTROM_ERROR_LOCK_REQUIRED;
    int rc = func(out_buffer, out_buffer_word_size, partition_and_flags);
    bootrom_release_lock(BOOTROM_LOCK_SHA_256);
    return rc;
}

// todo SECURE only
/*!
 * \brief Loads the current partition table from flash, if present
 * \ingroup pico_bootrom
 *
 * This method potentially requires similar complexity to the boot path in terms of picking amongst versions, checking signatures etc.
 * As a result it requires a user provided memory buffer as a work area. The work area should byte word-aligned and of sufficient size
 * or BOOTROM_ERROR_INSUFFICIENT_RESOURCES will be returned. The work area size currently required is 3064, so 3K is a good choice.
 * 
 * If force_reload is false, then this method will return BOOTROM_OK immediately if the bootrom is loaded, otherwise it will
 * reload the partition table if it has been loaded already, allowing for the partition table to be updated in a running program.
 * 
 * \param workarea_base base address of work area
 * \param workarea_size size of work area
 * \param force_reload force reloading of the partition table
 */
static inline int rom_load_partition_table(uint8_t *workarea_base, uint32_t workarea_size, bool force_reload) {
    rom_load_partition_table_fn func = (rom_load_partition_table_fn) rom_func_lookup_inline(ROM_FUNC_LOAD_PARTITION_TABLE);
    if (!bootrom_try_acquire_lock(BOOTROM_LOCK_SHA_256))
        return BOOTROM_ERROR_LOCK_REQUIRED;
    int rc = func(workarea_base, workarea_size, force_reload);
    bootrom_release_lock(BOOTROM_LOCK_SHA_256);
    return rc;
}

// todo SECURE only
/*!
 * \brief Pick a partition from an A/B pair
 * \ingroup pico_bootrom
 *
 * Determines which of the partitions has the "better" IMAGE_DEF. In the case of executable images, this is the one that would be booted
 * 
 * This method potentially requires similar complexity to the boot path in terms of picking amongst versions, checking signatures etc.
 * As a result it requires a user provided memory buffer as a work area. The work area should bye word aligned, and of sufficient size
 * or BOOTROM_ERROR_INSUFFICIENT_RESOURCES will be returned. The work area size currently required is 3064, so 3K is a good choice.
 * 
 * The passed partition number can be any valid partition number other than the "B" partition of an A/B pair.
 * 
 * This method returns a negative error code, or the partition number of the picked partition if (i.e. partition_a_num or the
 * number of its "B" partition if any).
 * 
 * NOTE: This method does not look at owner partitions, only the A partition passed and it's corresponding B partition.
 * 
 * \param workarea_base base address of work area
 * \param workarea_size size of work area
 * \param partition_a_num the A partition of the pair
 * \param flash_update_boot_window_base the flash update base, to pick that partition instead of the normally "better" partition
 */
static inline int rom_pick_ab_partition(uint8_t *workarea_base, uint32_t workarea_size, uint partition_a_num, uint32_t flash_update_boot_window_base) {
    rom_pick_ab_partition_fn func = (rom_pick_ab_partition_fn) rom_func_lookup_inline(ROM_FUNC_PICK_AB_PARTITION);
    if (!bootrom_try_acquire_lock(BOOTROM_LOCK_SHA_256))
        return BOOTROM_ERROR_LOCK_REQUIRED;
    int rc = func(workarea_base, workarea_size, partition_a_num, flash_update_boot_window_base);
    bootrom_release_lock(BOOTROM_LOCK_SHA_256);
    return rc;
}

/*!
 * \brief Get B partition
 * \ingroup pico_bootrom
 *
 * Returns the index of the B partition of partition A if a partition table is present and loaded, and there is a partition A with a B partition;
 * otherwise returns BOOTROM_ERROR_NOT_FOUND.
 * 
 * \param pi_a the A partition number
 */
static inline int rom_get_b_partition(uint pi_a) {
    rom_get_b_partition_fn func = (rom_get_b_partition_fn) rom_func_lookup_inline(ROM_FUNC_GET_B_PARTITION);
    return func(pi_a);
}

// todo SECURE only
/*!
 * \brief Get UF2 Target Partition
 * \ingroup pico_bootrom
 * 
 * This method performs the same operation to decide on a target partition for a UF2 family ID as when a UF2 is dragged onto the USB
 * drive in BOOTSEL mode.
 * 
 * This method potentially requires similar complexity to the boot path in terms of picking amongst versions, checking signatures etc.
 * As a result it requires a user provided memory buffer as a work area. The work area should byte word-aligned and of sufficient size
 * or `BOOTROM_ERROR_INSUFFICIENT_RESOURCES` will be returned. The work area size currently required is 3064, so 3K is a good choice.
 * 
 * If the partition table
 * has not been loaded (e.g. from a watchdog or RAM boot), then this method will return `BOOTROM_ERROR_PRECONDITION_NOT_MET`, and you
 * should load the partition table via <<api-load_partition_table, load_partition_table()>> first.
 * 
 * \param workarea_base base address of work area
 * \param workarea_size size of work area
 * \param family_id the family ID to place
 * \param partition_out pointer to the resident_partition_t to fill with the partition data
 */
static inline int rom_get_uf2_target_partition(uint8_t *workarea_base, uint32_t workarea_size, uint32_t family_id, resident_partition_t *partition_out) {
    rom_get_uf2_target_partition_fn func = (rom_get_uf2_target_partition_fn) rom_func_lookup_inline(ROM_FUNC_GET_UF2_TARGET_PARTITION);
    if (!bootrom_try_acquire_lock(BOOTROM_LOCK_SHA_256))
        return BOOTROM_ERROR_LOCK_REQUIRED;
    int rc = func(workarea_base, workarea_size, family_id, partition_out);
    bootrom_release_lock(BOOTROM_LOCK_SHA_256);
    return rc;
}

/*!
 * \brief Translate runtime to storage address
 * \ingroup pico_bootrom
 *
 * Applies the address translation currently configured by QMI address translation registers.
 * 
 * Translating an address outside of the XIP runtime address window, or beyond the bounds of an ATRANSx_SIZE field, returns BOOTROM_ERROR_INVALID_ADDRESS,
 * which is not a valid flash storage address. Otherwise, return the storage address which QMI would access when presented with the runtime address addr.
 * This is effectively a virtual-to-physical address translation for QMI.
 * 
 * \param flash_runtime_addr the address to translate
 */
static inline intptr_t rom_flash_runtime_to_storage_addr(uintptr_t flash_runtime_addr) {
    rom_flash_runtime_to_storage_addr_fn func = (rom_flash_runtime_to_storage_addr_fn) rom_func_lookup_inline(ROM_FUNC_FLASH_RUNTIME_TO_STORAGE_ADDR);
    return func(flash_runtime_addr);
}

// todo SECURE only
/*!
 * \brief Chain into a launchable image
 * \ingroup pico_bootrom
 *
 * Searches a memory region for a launchable image, and executes it if possible.
 * 
 * The region_base and region_size specify a word-aligned, word-multiple-sized area of RAM, XIP RAM or flash to search.
 * The first 4 kiB of the region must contain the start of a Block Loop with an IMAGE_DEF. If the new image is launched,
 * the call does not return otherwise an error is returned.
 * 
 * The region_base is signed, as a negative value can be passed, which indicates that the (negated back to positive value)
 * is both the region_base and the base of the "flash update" region.
 * 
 * This method potentially requires similar complexity to the boot path in terms of picking amongst versions, checking signatures etc.
 * As a result it requires a user provided memory buffer as a work area. The work area should be word aligned, and of sufficient size
 * or BOOTROM_ERROR_INSUFFICIENT_RESOURCES will be returned. The work area size currently required is 3064, so 3K is a good choice.
 * 
 * NOTE: This method is primarily expected to be used when implementing bootloaders.
 * 
 * NOTE: When chaining into an image, the OTP_DATA_BOOT_FLAGS0_ROLLBACK_REQUIRED flag will not be set, to prevent invalidating a bootloader
 * without a rollback version by booting a binary which has one.
 * 
 * \param workarea_base base address of work area
 * \param workarea_size size of work area
 * \param region_base base address of image
 * \param region_size size of window containing image
 */
static inline int rom_chain_image(uint8_t *workarea_base, uint32_t workarea_size, uint32_t region_base, uint32_t region_size) {
    rom_chain_image_fn func = (rom_chain_image_fn) rom_func_lookup_inline(ROM_FUNC_CHAIN_IMAGE);
    bootrom_release_lock(BOOTROM_LOCK_ENABLE);
    uint32_t interrupt_flags = save_and_disable_interrupts();
    int rc = func(workarea_base, workarea_size, region_base, region_size);
    restore_interrupts_from_disabled(interrupt_flags);
    bootrom_acquire_lock_blocking(BOOTROM_LOCK_ENABLE);
    return rc;
}

typedef struct {
    uint8_t *buffer;
    uint32_t buffer_size;
    int *res;
} rom_helper_explicit_buy_params_t;

static inline void rom_helper_explicit_buy(void *param) {
    const rom_helper_explicit_buy_params_t *op = (const rom_helper_explicit_buy_params_t *)param;
    rom_explicit_buy_fn func = (rom_explicit_buy_fn) rom_func_lookup_inline(ROM_FUNC_EXPLICIT_BUY);
    *(op->res) = func(op->buffer, op->buffer_size);
}

// todo SECURE only
/*!
 * \brief Buy an image
 * \ingroup pico_bootrom
 *
 * Perform an "explicit" buy of an executable launched via an IMAGE_DEF which was "explicit buy" flagged. A "flash update"
 * boot of such an image is a way to have the image execute once, but only become the "current" image if it calls
 * back into the bootrom via this call.
 * 
 * This call may perform the following:
 * 
 * - Erase and rewrite the part of flash containing the "explicit buy" flag in order to clear said flag.
 * - Erase the first sector of the other partition in an A/B partition scenario, if this new IMAGE_DEF is a version downgrade
 *   (so this image will boot again when not doing a "flash update" boot)
 * - Update the rollback version in OTP if the chip is secure, and a rollback version is present in the image.
 * 
 * NOTE: The device may reboot while updating the rollback version, if multiple rollback rows need to be written - this occurs
 * when the version crosses a multiple of 24 (for example upgrading from version 23 to 25 requires a reboot, but 23 to 24 or 24 to 25 doesn't).
 * The application should therefore be prepared to reboot when calling this function, if rollback versions are in use.
 * 
 * Note that the first of the above requires 4 kiB of scratch space, so you should pass a word aligned buffer of at least 4 kiB to this method,
 * or it will return BOOTROM_ERROR_INSUFFICIENT_RESOURCES if the "explicit buy" flag needs to be cleared.
 * 
 * \param buffer base address of scratch space
 * \param buffer_size size of scratch space
 */
static inline int rom_explicit_buy(uint8_t *buffer, uint32_t buffer_size) {
    int rc = 0;
    rom_helper_explicit_buy_params_t params = {
        .buffer = buffer,
        .buffer_size = buffer_size,
        .res = &rc
    };
    int flash_rc = flash_safe_execute(rom_helper_explicit_buy, &params, UINT32_MAX);
    if (flash_rc != PICO_OK) {
        return flash_rc;
    } else {
        return rc;
    }
}

#ifndef __riscv
/*!
 * \brief Set NS API Permission
 * \ingroup pico_bootrom
 * 
 * Allow or disallow the specific NS API (note all NS APIs default to disabled).
 * 
 * ns_api_num configures ARM-NS access to the given API. When an NS API is disabled,
 * calling it will return BOOTROM_ERROR_NOT_PERMITTED.
 * 
 * NOTE: All permissions default to disallowed after a reset.
 * 
 * \param ns_api_num ns api number
 * \param allowed permission
 */
static inline int rom_set_ns_api_permission(uint ns_api_num, bool allowed) {
    rom_set_ns_api_permission_fn func = (rom_set_ns_api_permission_fn) rom_func_lookup_inline(ROM_FUNC_SET_NS_API_PERMISSION);
    return func(ns_api_num, allowed);
}
#endif

// todo SECURE only
/*!
 * \brief Validate NS Buffer
 * \ingroup pico_bootrom
 *
 * Utility method that can be used by secure ARM code to validate a buffer passed to it from Non-secure code.
 * 
 * Both the write parameter and the (out) result parameter ok are RCP booleans, so 0xa500a500 for true, and 0x00c300c3
 * for false. This enables hardening of this function, and indeed the write parameter must be one of these values or the RCP
 * will hang the system.
 * 
 * For success, the entire buffer must fit in range XIP_BASE -> SRAM_END, and must be accessible by the Non-secure
 * caller according to SAU + NS MPU (privileged or not based on current processor IPSR and NS CONTROL flag). Buffers
 * in USB RAM are also allowed if access is granted to NS via ACCESSCTRL.
 * 
 * \param addr buffer address
 * \param size buffer size
 * \param write rcp boolean, true if writeable
 * \param ok rcp boolean result
 */
static inline void* rom_validate_ns_buffer(const void *addr, uint32_t size, uint32_t write, uint32_t *ok) {
    rom_validate_ns_buffer_fn func = (rom_validate_ns_buffer_fn) rom_func_lookup_inline(ROM_FUNC_VALIDATE_NS_BUFFER);
    return func(addr, size, write, ok);
}

/*!
 * \brief Set ROM callback function
 * \ingroup pico_bootrom
 * 
 * The only currently supported callback_number is 0 which sets the callback used for the secure_call API.
 * 
 * A callback pointer of 0 deletes the callback function, a positive callback pointer (all valid function pointers are on RP2350)
 * sets the callback function, but a negative callback pointer can be passed to get the old value without setting a new value.
 * 
 * If successful, returns >=0 (the existing value of the function pointer on entry to the function).
 * 
 * \param callback_num the callback number to set - only 0 is supported on RP2350
 * \param funcptr pointer to the callback function
 */
static inline intptr_t rom_set_rom_callback(uint callback_num, bootrom_api_callback_generic_t funcptr) {
    rom_set_rom_callback_fn func = (rom_set_rom_callback_fn) rom_func_lookup_inline(ROM_FUNC_SET_ROM_CALLBACK);
    return func(callback_num, funcptr);
}

#define BOOT_TYPE_NORMAL     0
#define BOOT_TYPE_BOOTSEL    2
#define BOOT_TYPE_RAM_IMAGE  3
#define BOOT_TYPE_FLASH_UPDATE 4

// values 8-15 are secure only
#define BOOT_TYPE_PC_SP      0xd

// ORed in if a bootloader chained into the image
#define BOOT_TYPE_CHAINED_FLAG 0x80

/*!
 * \brief Get system information
 * \ingroup pico_bootrom
 *
 * Fills a buffer with various system information. Note that this API is also used to return information over the picoboot interface.
 * 
 * On success, the buffer is filled, and the number of words filled in the buffer is returned.
 * 
 * The information returned is chosen by the flags parameter; the first word in the returned buffer,
 * is the (sub)set of those flags that the API supports. You should always check this value before interpreting
 * the buffer.
 * 
 * "Boot Diagnostic" information is intended to help identify the cause of a failed boot, or booting into an unexpected binary.
 * This information can be retrieved via picoboot after a watchdog reboot, however it will not survive
 * a reset via the RUN pin or POWMAN reset.
 * 
 * There is only one word of diagnostic information. What it records is based on the pp selection above, which
 * is itself set as a parameter when rebooting programmatically into a normal boot.
 * 
 * To get diagnostic info, pp must refer to a slot or an "A" partition; image diagnostics are automatically selected on boot
 * from OTP or RAM image, or when chain_image() is called.)
 * 
 * The diagnostic word thus contains data for either slot 0 and slot 1, or the "A" partition (and its "B" partition if it has one). The low half word
 * of the diagnostic word contains information from slot 0 or partition A; the high half word contains information from slot 1 or partition B.
 * 
 * To get a full picture of a failed boot involving slots and multiple partitions, the device can be rebooted
 * multiple times to gather the information.
 * 
 * \param out_buffer buffer to write data to
 * \param out_buffer_word_size size of out_buffer, in words
 * \param flags flags
 */
static inline int rom_get_sys_info(uint32_t *out_buffer, uint32_t out_buffer_word_size, uint32_t flags) {
    rom_get_sys_info_fn func = (rom_get_sys_info_fn)rom_func_lookup_inline(ROM_FUNC_GET_SYS_INFO);
    return func(out_buffer, out_buffer_word_size, flags);
}

typedef struct {
    union {
        struct __packed {
            int8_t diagnostic_partition_index; // used BOOT_PARTITION constants
            uint8_t boot_type;
            int8_t partition;
            uint8_t tbyb_and_update_info;
        };
        uint32_t boot_word;
    };
    uint32_t boot_diagnostic;
    uint32_t reboot_params[2];
} boot_info_t;

static inline int rom_get_boot_info(boot_info_t *info) {
    uint32_t result[5];
    int words_returned = rom_get_sys_info(result, 5, SYS_INFO_BOOT_INFO);
    if (words_returned == (sizeof(result)/sizeof(result[0])) && result[0] == SYS_INFO_BOOT_INFO) {
        memcpy(info, &result[1], sizeof(boot_info_t));
        return true;
    } else {
        return false;
    }
}

static inline int rom_get_last_boot_type_with_chained_flag(void) {
    uint32_t result[5];
    int words_returned = rom_get_sys_info(result, 5, SYS_INFO_BOOT_INFO);
    if (words_returned == count_of(result) && result[0] == SYS_INFO_BOOT_INFO) {
        // todo use struct
        return (int)((result[1] & 0xff00u) >> 8);
    } else {
        return PICO_ERROR_INVALID_DATA;
    }
}

// BOOT_TYPE_NORMAL       0x0
// BOOT_TYPE_BOOTSEL      0x2
// BOOT_TYPE_RAM_IMAGE    0x3
// BOOT_TYPE_FLASH_UPDATE 0x4
// BOOT_TYPE_PC_SP        0xd
static inline int rom_get_last_boot_type(void) {
    int rc = rom_get_last_boot_type_with_chained_flag();
    if (rc >= 0) rc &= ~BOOT_TYPE_CHAINED_FLAG;
    return rc;
}

/*! \brief  Add a runtime partition to the partition table to specify flash permissions
 * \ingroup pico_bootrom
 *
 * Note that a partition is added to the runtime view of the partition table maintained by the bootrom if there is space to do so
 *
 * Note that these permissions cannot override the permissions for any pre-existing partitions, as permission matches are made on a first partition found basis.
 *
 * @param start_offset the start_offset into flash in bytes (must be a multiple of 4K)
 * @param size the size in byte (must be a multiple of 4K)
 * @param permissions the bitwise OR of permissions from PICOBIN_PARTITION_PERMISSION_ constants, e.g. \ref PICOBIN_PARTITION_PERMISSION_S_R_BITS from boot/picobin.h
 * @return >= 0 the partition number added if
 *         PICO_ERROR_BAD_ALIGNMENT if the start_offset or size aren't multiples of 4K.
 *         PICO_ERROR_INVALID_ARG if the start_offset or size are out of range, or invalid permission bits are set.
 */
int rom_add_flash_runtime_partition(uint32_t start_offset, uint32_t size, uint32_t permissions);

#endif

#ifdef __cplusplus
}
#endif

#endif // !__ASSEMBLER__
#endif
