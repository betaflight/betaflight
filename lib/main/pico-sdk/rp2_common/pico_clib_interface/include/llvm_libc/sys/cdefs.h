/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __PICO_LLVM_LIBC_SYS_CDEFS_H
#define __PICO_LLVM_LIBC_SYS_CDEFS_H

#if defined(__STDC__) || defined(__cplusplus)

#define __CONCAT1(x,y)  x ## y
#define __CONCAT(x,y)   __CONCAT1(x,y)
#define __STRING(x)     #x
#define __XSTRING(x)    __STRING(x)

#endif

#define __unused        __attribute__((__unused__))
#define __used          __attribute__((__used__))
#define __packed        __attribute__((__packed__))
#define __aligned(x)    __attribute__((__aligned__(x)))

#define __always_inline __inline__ __attribute__((__always_inline__))
#define __noinline      __attribute__((__noinline__))

#define __printflike(fmtarg, firstvararg) \
  __attribute__((__format__ (__printf__, fmtarg, firstvararg)))

#endif
