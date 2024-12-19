/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_BINARY_INFO_H
#define _PICO_BINARY_INFO_H

/** \file binary_info.h
 *  \defgroup pico_binary_info pico_binary_info
 *
 * \brief Binary info is intended for embedding machine readable information with the binary in FLASH
 *
 * Example uses include:
 *
 * - Program identification / information
 * - Pin layouts
 * - Included features
 * - Identifying flash regions used as block devices/storage
 */

#include "pico/binary_info/defs.h"
#include "pico/binary_info/structure.h"

// PICO_CONFIG: PICO_NO_BINARY_INFO, Don't include "binary info" in the output binary, type=bool, default=0 except for `PICO_PLATFORM` `host`, group=pico_runtime_init
#if !PICO_ON_DEVICE && !defined(PICO_NO_BINARY_INFO)
#define PICO_NO_BINARY_INFO 1
#endif
#include "pico/binary_info/code.h"
#endif
