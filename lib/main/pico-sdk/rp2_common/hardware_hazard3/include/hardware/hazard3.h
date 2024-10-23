/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_HAZARD3_H
#define _HARDWARE_HAZARD3_H

#include "pico.h"
#include "hardware/riscv.h"

// This includes both standard and Hazard3 custom CSRs:
#include "hardware/regs/rvcsr.h"

#include "hardware/hazard3/features.h"
#include "hardware/hazard3/instructions.h"

/** \file hardware/hazard3.h
 *  \defgroup hardware_hazard3 hardware_hazard3
 *
 * \brief Accessors for Hazard3-specific RISC-V CSRs, and intrinsics for Hazard3 custom instructions
 *
 */

#ifndef __ASSEMBLER__

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __hazard3_extension_xh3irq
#define hazard3_irqarray_read(csr, index) (riscv_read_set_csr(csr, (index)) >> 16)
#else
#define hazard3_irqarray_read(csr, index) static_assert(false, "Not supported: Xh3irq extension")
#endif

#ifdef __hazard3_extension_xh3irq
#define hazard3_irqarray_write(csr, index, data) (riscv_write_csr(csr, (index) | ((uint32_t)(data) << 16)))
#else
#define hazard3_irqarray_write(csr, index, data) static_assert(false, "Not supported: Xh3irq extension")
#endif

#ifdef __hazard3_extension_xh3irq
#define hazard3_irqarray_set(csr, index, data) (riscv_set_csr(csr, (index) | ((uint32_t)(data) << 16)))
#else
#define hazard3_irqarray_set(csr, index, data) static_assert(false, "Not supported: Xh3irq extension")
#endif

#ifdef __hazard3_extension_xh3irq
#define hazard3_irqarray_clear(csr, index, data) (riscv_clear_csr(csr, (index) | ((uint32_t)(data) << 16)))
#else
#define hazard3_irqarray_clear(csr, index, data) static_assert(false, "Not supported: Xh3irq extension")
#endif

#ifdef __cplusplus
}
#endif

#endif

#endif
