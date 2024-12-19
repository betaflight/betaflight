/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_PLATFORM_SECTION_MACROS_H
#define _PICO_PLATFORM_SECTION_MACROS_H

#ifndef __ASSEMBLER__

/*! \brief Section attribute macro for placement in RAM after the `.data` section
 *  \ingroup pico_platform
 *
 * For example a 400 element `uint32_t` array placed after the .data section
 *
 *     uint32_t __after_data("my_group_name") a_big_array[400];
 *
 * The section attribute is `.after_data.<group>`
 *
 * \param group a string suffix to use in the section name to distinguish groups that can be linker
 *              garbage-collected independently
 */
#ifndef __after_data
#define __after_data(group) __attribute__((section(".after_data." group)))
#endif

/*! \brief Section attribute macro for placement not in flash (i.e in RAM)
 *  \ingroup pico_platform
 *
 * For example a 3 element `uint32_t` array placed in RAM (even though it is `static const`)
 *
 *     static const uint32_t __not_in_flash("my_group_name") an_array[3];
 *
 * The section attribute is `.time_critical.<group>`
 *
 * \param group a string suffix to use in the section name to distinguish groups that can be linker
 *              garbage-collected independently
 */
#ifndef __not_in_flash
#define __not_in_flash(group) __attribute__((section(".time_critical." group)))
#endif

/*! \brief Section attribute macro for placement in the SRAM bank 4 (known as "scratch X")
 *  \ingroup pico_platform
 *
 * Scratch X is commonly used for critical data and functions accessed only by one core (when only
 * one core is accessing the RAM bank, there is no opportunity for stalls)
 *
 * For example a `uint32_t` variable placed in "scratch X"
 *
 *     uint32_t __scratch_x("my_group_name") foo = 23;
 *
 * The section attribute is `.scratch_x.<group>`
 *
 * \param group a string suffix to use in the section name to distinguish groups that can be linker
 *              garbage-collected independently
 */
#ifndef __scratch_x
#define __scratch_x(group) __attribute__((section(".scratch_x." group)))
#endif

/*! \brief Section attribute macro for placement in the SRAM bank 5 (known as "scratch Y")
 *  \ingroup pico_platform
 *
 * Scratch Y is commonly used for critical data and functions accessed only by one core (when only
 * one core is accessing the RAM bank, there is no opportunity for stalls)
 *
 * For example a `uint32_t` variable placed in "scratch Y"
 *
 *     uint32_t __scratch_y("my_group_name") foo = 23;
 *
 * The section attribute is `.scratch_y.<group>`
 *
 * \param group a string suffix to use in the section name to distinguish groups that can be linker
 *              garbage-collected independently
 */
#ifndef __scratch_y
#define __scratch_y(group) __attribute__((section(".scratch_y." group)))
#endif

/*! \brief Section attribute macro for data that is to be left uninitialized
 *  \ingroup pico_platform
 *
 * Data marked this way will retain its value across a reset (normally uninitialized data - in the .bss
 * section) is initialized to zero during runtime initialization
 *
 * For example a `uint32_t` foo that will retain its value if the program is restarted by reset.
 *
 *     uint32_t __uninitialized_ram(foo);
 *
 * The section attribute is `.uninitialized_data.<group>`
 *
 * \param group a string suffix to use in the section name to distinguish groups that can be linker
 *              garbage-collected independently
 */
#ifndef __uninitialized_ram
#define __uninitialized_ram(group) __attribute__((section(".uninitialized_data." #group))) group
#endif

/*! \brief Section attribute macro for placement in flash even in a COPY_TO_RAM binary
 *  \ingroup pico_platform
 *
 * For example a `uint32_t` variable explicitly placed in flash (it will hard fault if you attempt to write it!)
 *
 *     uint32_t __in_flash("my_group_name") foo = 23;
 *
 * The section attribute is `.flashdata.<group>`
 *
 * \param group a string suffix to use in the section name to distinguish groups that can be linker
 *              garbage-collected independently
 */
#ifndef __in_flash
#define __in_flash(group) __attribute__((section(".flashdata." group)))
#endif

/*! \brief Indicates a function should not be stored in flash
 *  \ingroup pico_platform
 *
 * Decorates a function name, such that the function will execute from RAM (assuming it is not inlined
 * into a flash function by the compiler)
 *
 * For example a function called my_func taking an int parameter:
 *
 *     void __not_in_flash_func(my_func)(int some_arg) {
 *
 * The function is placed in the `.time_critical.<func_name>` linker section
 *
 * \see __no_inline_not_in_flash_func
 */
#ifndef __not_in_flash_func
#define __not_in_flash_func(func_name) __not_in_flash(__STRING(func_name)) func_name
#endif

/*! \brief Indicates a function is time/latency critical and should not run from flash
 *  \ingroup pico_platform
 *
 * Decorates a function name, such that the function will execute from RAM (assuming it is not inlined
 * into a flash function by the compiler) to avoid possible flash latency. Currently this macro is identical
 * in implementation to `__not_in_flash_func`, however the semantics are distinct and a `__time_critical_func`
 * may in the future be treated more specially to reduce the overhead when calling such function from a flash
 * function.
 *
 * For example a function called my_func taking an int parameter:
 *
 *     void __time_critical_func(my_func)(int some_arg) {
 *
 * The function is placed in the `.time_critical.<func_name>` linker section
 *
 * \see __not_in_flash_func
 */
#ifndef __time_critical_func
#define __time_critical_func(func_name) __not_in_flash_func(func_name)
#endif

/*! \brief Indicate a function should not be stored in flash and should not be inlined
 *  \ingroup pico_platform
 *
 * Decorates a function name, such that the function will execute from RAM, explicitly marking it as
 * noinline to prevent it being inlined into a flash function by the compiler
 *
 * For example a function called my_func taking an int parameter:
 *
 *     void __no_inline_not_in_flash_func(my_func)(int some_arg) {
 *
 * The function is placed in the `.time_critical.<func_name>` linker section
 */
#ifndef __no_inline_not_in_flash_func
#define __no_inline_not_in_flash_func(func_name) __noinline __not_in_flash_func(func_name)
#endif

#else

#ifndef RAM_SECTION_NAME
#define RAM_SECTION_NAME(x) .time_critical.##x
#endif

#ifndef SECTION_NAME
#define SECTION_NAME(x) .text.##x
#endif

#endif // !__ASSEMBLER__

#endif

