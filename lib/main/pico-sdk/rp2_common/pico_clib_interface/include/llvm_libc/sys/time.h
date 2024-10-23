/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_LLVM_LIBC_SYS_TIME_H
#define _PICO_LLVM_LIBC_SYS_TIME_H

#include <__llvm-libc-common.h>

#include <llvm-libc-types/time_t.h>
#include <llvm-libc-types/struct_timespec.h>

typedef long suseconds_t;

struct timeval {
  time_t tv_sec;
  suseconds_t tv_usec;
};

struct timezone {
  int tz_minuteswest;
  int tz_dsttime;
};

__BEGIN_C_DECLS

int gettimeofday(struct timeval *tv, struct timezone *tz);
int settimeofday(const struct timeval *tv, const struct timezone *tz);

__END_C_DECLS

#endif // _PICO_LLVM_LIBC_SYS_TIME_H
