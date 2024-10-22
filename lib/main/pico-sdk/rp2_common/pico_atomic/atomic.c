/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdatomic.h>
#include "pico/sync.h"

// We use __builtin_mem* to avoid libc dependency.
#define memcpy __builtin_memcpy
#define memcmp __builtin_memcmp

static inline uint32_t atomic_lock(__unused const volatile void *ptr) {
    return spin_lock_blocking(spin_lock_instance(PICO_SPINLOCK_ID_ATOMIC));
}

static inline void atomic_unlock(__unused const volatile void *ptr, uint32_t save) {
    spin_unlock(spin_lock_instance(PICO_SPINLOCK_ID_ATOMIC), save);
}

#if PICO_C_COMPILER_IS_GNU

_Bool __atomic_test_and_set_c(volatile void *mem, __unused int model) {
    uint32_t save = atomic_lock(mem);
    bool result = *(volatile bool *) mem;
    *(volatile bool *) mem = true;
    atomic_unlock(mem, save);
    return result;
}

#define __atomic_load_c __atomic_load
#define __atomic_store_c __atomic_store
#define __atomic_exchange_c __atomic_exchange
#define __atomic_compare_exchange_c __atomic_compare_exchange
#define __atomic_is_lock_free_c __atomic_is_lock_free
#else
// Clang objects if you redefine a builtin.
#pragma redefine_extname __atomic_load_c __atomic_load
#pragma redefine_extname __atomic_store_c __atomic_store
#pragma redefine_extname __atomic_exchange_c __atomic_exchange
#pragma redefine_extname __atomic_compare_exchange_c __atomic_compare_exchange
#pragma redefine_extname __atomic_is_lock_free_c __atomic_is_lock_free
#endif

// Whether atomic operations for the given size (and alignment) are lock-free.
bool __atomic_is_lock_free_c(__unused size_t size, __unused const volatile void *ptr) {
#if !__ARM_ARCH_6M__
    if (size == 1 || size == 2 || size == 4) {
        size_t align = size - 1;
        return (((uintptr_t)ptr) & align) == 0;
    }
#endif
    return false;
}



// An atomic load operation.  This is atomic with respect to the source pointer only.
void __atomic_load_c(uint size, const volatile void *src, void *dest, __unused int model) {
    uint32_t save = atomic_lock(src);
    memcpy(dest, remove_volatile_cast_no_barrier(const void *, src), size);
    atomic_unlock(src, save);
}

// An atomic store operation.  This is atomic with respect to the destination
// pointer only.
void __atomic_store_c(uint size, volatile void *dest, void *src, __unused int model) {
    uint32_t save = atomic_lock(src);
    memcpy(remove_volatile_cast_no_barrier(void *, dest), src, size);
    atomic_unlock(src, save);
}

// Atomic compare and exchange operation.  If the value at *ptr is identical
// to the value at *expected, then this copies value at *desired to *ptr.  If
// they  are not, then this stores the current value from *ptr in *expected.
//
// This function returns 1 if the exchange takes place or 0 if it fails.
_Bool __atomic_compare_exchange_c(uint size, volatile void *ptr, void *expected,
                                  void *desired, __unused int success, __unused int failure) {
    uint32_t save = atomic_lock(ptr);
    if (memcmp(remove_volatile_cast_no_barrier(void *, ptr), expected, size) == 0) {
        memcpy(remove_volatile_cast_no_barrier(void *, ptr), desired, size);
        atomic_unlock(ptr, save);
        return 1;
    }
    memcpy(expected, remove_volatile_cast_no_barrier(void *, ptr), size);
    atomic_unlock(ptr, save);
    return 0;
}

// Performs an atomic exchange operation between two pointers.  This is atomic
// with respect to the target address.
void __atomic_exchange_c(uint size, volatile void *ptr, void *val, void *old, __unused int model) {

    uint32_t save = atomic_lock(ptr);
    memcpy(old, remove_volatile_cast_no_barrier(void *, ptr), size);
    memcpy(remove_volatile_cast_no_barrier(void *, ptr), val, size);
    atomic_unlock(ptr, save);
}

#if __ARM_ARCH_6M__
#define ATOMIC_OPTIMIZED_CASES       \
  ATOMIC_OPTIMIZED_CASE(1, uint8_t)  \
  ATOMIC_OPTIMIZED_CASE(2, uint16_t) \
  ATOMIC_OPTIMIZED_CASE(4, uint) \
  ATOMIC_OPTIMIZED_CASE(8, uint64_t)
#else
#define ATOMIC_OPTIMIZED_CASES \
  ATOMIC_OPTIMIZED_CASE(8, uint64_t)
#endif

#define ATOMIC_OPTIMIZED_CASE(n, type)                                               \
  type __atomic_load_##n(const volatile void *src, __unused int memorder) {   \
    uint32_t save = atomic_lock(src);                                         \
    type val = *(const volatile type *)src;                                   \
    atomic_unlock(src, save);                                                 \
    return val;                                                               \
  }

ATOMIC_OPTIMIZED_CASES

#undef ATOMIC_OPTIMIZED_CASE

#define ATOMIC_OPTIMIZED_CASE(n, type)                                               \
  void __atomic_store_##n(volatile void *dest, type val, __unused  int model) { \
    uint32_t save = atomic_lock(dest);                                        \
    *(volatile type *)dest = val;                                             \
    atomic_unlock(dest, save);                                                \
  }

ATOMIC_OPTIMIZED_CASES

#undef ATOMIC_OPTIMIZED_CASE

#define ATOMIC_OPTIMIZED_CASE(n, type)                                               \
  bool __atomic_compare_exchange_##n(volatile void *ptr, void  *expected, type desired, \
                                     __unused bool weak, __unused int success, __unused int failure) { \
    uint32_t save = atomic_lock(ptr);                                         \
    if (*(volatile type *)ptr == *(type *)expected) {                         \
      *(volatile type *)ptr = desired;                                        \
      atomic_unlock(ptr, save);                                               \
      return true;                                                            \
    }                                                                         \
    *(type *)expected = *(volatile type *)ptr;                                \
    atomic_unlock(ptr, save);                                                 \
    return false;                                                             \
  }

ATOMIC_OPTIMIZED_CASES

#undef ATOMIC_OPTIMIZED_CASE

#define ATOMIC_OPTIMIZED_CASE(n, type)                                      \
  type __atomic_exchange_##n(volatile void *dest, type val, __unused int model) { \
    uint32_t save = atomic_lock(dest);                               \
    type tmp = *(volatile type *)dest;                               \
    *(volatile type *)dest = val;                                    \
    atomic_unlock(dest, save);                                       \
    return tmp;                                                      \
  }

ATOMIC_OPTIMIZED_CASES

#undef ATOMIC_OPTIMIZED_CASE

// Atomic read-modify-write operations for integers of various sizes.

#define ATOMIC_RMW(n, type, opname, op)                                \
  type __atomic_fetch_##opname##_##n(volatile void *ptr, type val, __unused int model) { \
    uint32_t save = atomic_lock(ptr);                                  \
    type tmp = *(volatile type *)ptr;                                  \
    *(volatile type *)ptr = tmp op val;                                \
    atomic_unlock(ptr, save);                                          \
    return tmp;                                                        \
  }

#define ATOMIC_RMW_NAND(n, type)                                     \
  type __atomic_fetch_nand_##n(type *ptr, type val, __unused int model) { \
    uint32_t save = atomic_lock(ptr);                                \
    type tmp = *ptr;                                                 \
    *ptr = ~(tmp & val);                                             \
    atomic_unlock(ptr, save);                                        \
    return tmp;                                                      \
  }

#define ATOMIC_OPTIMIZED_CASE(n, type) ATOMIC_RMW(n, type, add, +)

ATOMIC_OPTIMIZED_CASES

#undef ATOMIC_OPTIMIZED_CASE
#define ATOMIC_OPTIMIZED_CASE(n, type) ATOMIC_RMW(n, type, sub, -)

ATOMIC_OPTIMIZED_CASES

#undef ATOMIC_OPTIMIZED_CASE
#define ATOMIC_OPTIMIZED_CASE(n, type) ATOMIC_RMW(n, type, and, &)

ATOMIC_OPTIMIZED_CASES

#undef ATOMIC_OPTIMIZED_CASE
#define ATOMIC_OPTIMIZED_CASE(n, type) ATOMIC_RMW(n, type, or, |)

ATOMIC_OPTIMIZED_CASES

#undef ATOMIC_OPTIMIZED_CASE
#define ATOMIC_OPTIMIZED_CASE(n, type) ATOMIC_RMW(n, type, xor, ^)

ATOMIC_OPTIMIZED_CASES

#undef ATOMIC_OPTIMIZED_CASE

#if __has_builtin(__c11_atomic_fetch_nand)
#define ATOMIC_OPTIMIZED_CASE(n, type) ATOMIC_RMW_NAND(n, type)
ATOMIC_OPTIMIZED_CASES
#undef ATOMIC_OPTIMIZED_CASE
#endif
