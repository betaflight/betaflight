/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/** \file platform.h
 *  \defgroup pico_platform pico_platform
 *
 * \brief Macros and definitions for accessing the CPU registers
 *
 * This header may be included by assembly code
 */

#ifndef _PICO_PLATFORM_CPU_REGS_H
#define _PICO_PLATFORM_CPU_REGS_H

#include "hardware/regs/m0plus.h"
#define ARM_CPU_PREFIXED(x) M0PLUS_ ## x

#ifndef __ASSEMBLER__
#include "hardware/structs/m0plus.h"
#define arm_cpu_hw m0plus_hw
#include "hardware/structs/nvic.h"
#include "hardware/structs/scb.h"
#endif

#endif