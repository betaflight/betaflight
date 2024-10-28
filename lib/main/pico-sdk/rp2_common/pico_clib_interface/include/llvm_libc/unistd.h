/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_LLVM_LIBC_UNISTD_H
#define _PICO_LLVM_LIBC_UNISTD_H

#include <__llvm-libc-common.h>

typedef int pid_t;

__BEGIN_C_DECLS

_Noreturn void _exit(int) __NOEXCEPT;

__END_C_DECLS

#endif // _PICO_LLVM_LIBC_UNISTD_H
