/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_LLVM_LIBC_SYS_TIMES_H
#define _PICO_LLVM_LIBC_SYS_TIMES_H

#include <llvm-libc-types/time_t.h>

#define CLOCKS_PER_SEC 100

typedef int clock_t;

struct tms {
  clock_t tms_utime;
  clock_t tms_stime;
  clock_t tms_cutime;
  clock_t tms_cstime;
};

#endif
