/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_PLATFORM_COMPILER_H
#define _PICO_PLATFORM_COMPILER_H

/** \file platform_compiler.h
 *  \defgroup pico_platform pico_platform
 *
 * \brief Macros and definitions (and functions when included by non assembly code) to adapt for different compilers
 *
 * This header may be included by assembly code
 */

#include "hardware/platform_defs.h"

#ifndef __ASSEMBLER__

#if defined __GNUC__
#include <sys/cdefs.h>
// note LLVM defines __GNUC__
#ifdef __clang__
#define PICO_C_COMPILER_IS_CLANG 1
#else
#define PICO_C_COMPILER_IS_GNU 1
#endif
#elif defined __ICCARM__
#ifndef __aligned
#define __aligned(x)	__attribute__((__aligned__(x)))
#endif
#ifndef __always_inline
#define __always_inline __attribute__((__always_inline__))
#endif
#ifndef __noinline
#define __noinline      __attribute__((__noinline__))
#endif
#ifndef __packed
#define __packed        __attribute__((__packed__))
#endif
#ifndef __printflike
#define __printflike(a, b)
#endif
#ifndef __unused
#define __unused        __attribute__((__unused__))
#endif
#ifndef __used
#define __used          __attribute__((__used__))
#endif
#ifndef __CONCAT1
#define __CONCAT1(a, b) a ## b
#endif
#ifndef __CONCAT
#define __CONCAT(a, b)  __CONCAT1(a, b)
#endif
#ifndef __STRING
#define __STRING(a)     #a
#endif
/* Compatible definitions of GCC builtins */

static inline uint __builtin_ctz(uint x) {
  extern uint32_t __ctzsi2(uint32_t);
  return __ctzsi2(x);
}
#define __builtin_expect(x, y) (x)
#define __builtin_isnan(x) __iar_isnan(x)
#else
#error Unsupported toolchain
#endif

#define __weak __attribute__((weak))

#include "pico/types.h"

// GCC_Like_Pragma(x) is a pragma on GNUC compatible compilers
#ifdef __GNUC__
#define GCC_Like_Pragma _Pragma
#else
#define GCC_Like_Pragma(x)
#endif

// Clang_Pragma(x) is a pragma on Clang only
#ifdef __clang__
#define Clang_Pragma _Pragma
#else
#define Clang_Pragma(x)
#endif

// GCC_Pragma(x) is a pragma on GCC only
#if PICO_C_COMPILER_IS_GNU
#define GCC_Pragma _Pragma
#else
#define GCC_Pragma(x)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Marker for an interrupt handler
 *  \ingroup pico_platform
 *
 * For example an IRQ handler function called my_interrupt_handler:
 *
 *     void __isr my_interrupt_handler(void) {
 */
#define __isr

#define __packed_aligned __packed __aligned(4)

/*! \brief Attribute to force inlining of a function regardless of optimization level
 *  \ingroup pico_platform
 *
 *  For example my_function here will always be inlined:
 *
 *      int __force_inline my_function(int x) {
 *
 */

#if PICO_C_COMPILER_IS_GNU && (__GNUC__ <= 6 || (__GNUC__ == 7 && (__GNUC_MINOR__ < 3 || !defined(__cplusplus))))
#define __force_inline inline __always_inline
#else
#define __force_inline __always_inline
#endif

/*! \brief Macro to determine the number of elements in an array
 *  \ingroup pico_platform
 */
#ifndef count_of
#define count_of(a) (sizeof(a)/sizeof((a)[0]))
#endif

/*! \brief Macro to return the maximum of two comparable values
 *  \ingroup pico_platform
 */
#ifndef MAX
#define MAX(a, b) ((a)>(b)?(a):(b))
#endif

/*! \brief Macro to return the minimum of two comparable values
 *  \ingroup pico_platform
 */
#ifndef MIN
#define MIN(a, b) ((b)>(a)?(a):(b))
#endif

#ifdef __ARM_ARCH_ISA_THUMB
#define pico_default_asm(...) __asm (".syntax unified\n" __VA_ARGS__)
#define pico_default_asm_volatile(...) __asm volatile (".syntax unified\n" __VA_ARGS__)
#define pico_default_asm_goto(...) __asm goto (".syntax unified\n" __VA_ARGS__)
#else
#define pico_default_asm(...) __asm (__VA_ARGS__)
#define pico_default_asm_volatile(...) __asm volatile (__VA_ARGS__)
#define pico_default_asm_goto(...) __asm goto (__VA_ARGS__)
#endif

/*! \brief Ensure that the compiler does not move memory access across this method call
 *  \ingroup pico_platform
 *
 *  For example in the following code:
 *
 *      *some_memory_location = var_a;
 *      __compiler_memory_barrier();
 *      uint32_t var_b = *some_other_memory_location
 *
 * The compiler will not move the load from `some_other_memory_location` above the memory barrier (which it otherwise
 * might - even above the memory store!)
 */
__force_inline static void __compiler_memory_barrier(void) {
    pico_default_asm_volatile ("" : : : "memory");
}

/*! \brief Utility macro to assert two types are equivalent.
 *  \ingroup pico_platform
 *
 *  This macro can be useful in other macros along with `typeof` to assert that two parameters are of equivalent type
 *  (or that a single parameter is of an expected type)
 */
#define __check_type_compatible(type_a, type_b) static_assert(__builtin_types_compatible_p(type_a, type_b), __STRING(type_a) " is not compatible with " __STRING(type_b));

#define WRAPPER_FUNC(x) __wrap_ ## x
#define REAL_FUNC(x) __real_ ## x

#ifdef __cplusplus
}
#endif

#else // __ASSEMBLER__

#if defined __GNUC__
// note LLVM defines __GNUC__
#ifdef __clang__
#define PICO_ASSEMBLER_IS_CLANG 1
#else
#define PICO_ASSEMBLER_IS_GNU 1
#endif
#elif defined __ICCARM__
#else
#error Unsupported toolchain
#endif

#define WRAPPER_FUNC_NAME(x) __wrap_##x

#endif // !__ASSEMBLER__

#endif
