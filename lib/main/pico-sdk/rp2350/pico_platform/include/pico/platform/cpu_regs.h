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

#if defined(__riscv)
#include "hardware/hazard3.h"
#else
#include "hardware/regs/m33.h"
#define ARM_CPU_PREFIXED(x) M33_ ## x
#ifndef __ASSEMBLER__
#include "hardware/structs/m33.h"
#define arm_cpu_hw m33_hw
#include "hardware/structs/nvic.h"
#include "hardware/structs/scb.h"
#endif
#endif
#endif