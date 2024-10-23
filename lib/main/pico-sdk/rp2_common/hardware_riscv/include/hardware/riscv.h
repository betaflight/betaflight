/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_RISCV_
#define _HARDWARE_RISCV_

#include "pico.h"
#include "hardware/regs/rvcsr.h"

#ifndef __ASSEMBLER__

#ifdef __cplusplus
extern "C" {
#endif

/** \file hardware/riscv.h
 *  \defgroup hardware_riscv hardware_riscv
 *
 * \brief Accessors for standard RISC-V hardware (mainly CSRs)
 *
 */

#define _riscv_read_csr(csrname) ({ \
    uint32_t __csr_tmp_u32; \
    asm volatile ("csrr %0, " #csrname : "=r" (__csr_tmp_u32)); \
    __csr_tmp_u32; \
})

#define _riscv_write_csr(csrname, data) ({ \
    if (__builtin_constant_p(data) && !((data) & -32u)) { \
        asm volatile ("csrwi " #csrname ", %0" : : "i" (data)); \
    } else { \
        asm volatile ("csrw " #csrname ", %0" : : "r" (data)); \
    } \
})

#define _riscv_set_csr(csrname, data) ({ \
    if (__builtin_constant_p(data) && !((data) & -32u)) { \
        asm volatile ("csrsi " #csrname ", %0" : : "i" (data)); \
    } else { \
        asm volatile ("csrs " #csrname ", %0" : : "r" (data)); \
    } \
})

#define _riscv_clear_csr(csrname, data) ({ \
    if (__builtin_constant_p(data) && !((data) & -32u)) { \
        asm volatile ("csrci " #csrname ", %0" : : "i" (data)); \
    } else { \
        asm volatile ("csrc " #csrname ", %0" : : "r" (data)); \
    } \
})

#define _riscv_read_write_csr(csrname, data) ({ \
    uint32_t __csr_tmp_u32; \
    if (__builtin_constant_p(data) && !((data) & -32u)) { \
        asm volatile ("csrrwi %0, " #csrname ", %1": "=r" (__csr_tmp_u32) : "i" (data)); \
    } else { \
        asm volatile ("csrrw %0, " #csrname ", %1": "=r" (__csr_tmp_u32) : "r" (data)); \
    } \
    __csr_tmp_u32; \
})

#define _riscv_read_set_csr(csrname, data) ({ \
    uint32_t __csr_tmp_u32; \
    if (__builtin_constant_p(data) && !((data) & -32u)) { \
        asm volatile ("csrrsi %0, " #csrname ", %1": "=r" (__csr_tmp_u32) : "i" (data)); \
    } else { \
        asm volatile ("csrrs %0, " #csrname ", %1": "=r" (__csr_tmp_u32) : "r" (data)); \
    } \
    __csr_tmp_u32; \
})

#define _riscv_read_clear_csr(csrname, data) ({ \
    uint32_t __csr_tmp_u32; \
    if (__builtin_constant_p(data) && !((data) & -32u)) { \
        asm volatile ("csrrci %0, " #csrname ", %1": "=r" (__csr_tmp_u32) : "i" (data)); \
    } else { \
        asm volatile ("csrrc %0, " #csrname ", %1": "=r" (__csr_tmp_u32) : "r" (data)); \
    } \
    __csr_tmp_u32; \
})

// Argument macro expansion layer (CSR name may be a macro that expands to a
// CSR number, or it may be a bare name that the assembler knows about.)
#define riscv_read_csr(csrname) _riscv_read_csr(csrname)
#define riscv_write_csr(csrname, data) _riscv_write_csr(csrname, data)
#define riscv_set_csr(csrname, data) _riscv_set_csr(csrname, data)
#define riscv_clear_csr(csrname, data) _riscv_clear_csr(csrname, data)
#define riscv_read_write_csr(csrname, data) _riscv_read_write_csr(csrname, data)
#define riscv_read_set_csr(csrname, data) _riscv_read_set_csr(csrname, data)
#define riscv_read_clear_csr(csrname, data) _riscv_read_clear_csr(csrname, data)

// Helpers for encoding RISC-V immediates

// U format, e.g. lui
static inline uint32_t riscv_encode_imm_u(uint32_t x) {
    return (x >> 12) << 12;
}

// I format, e.g. addi
static inline uint32_t riscv_encode_imm_i(uint32_t x) {
    return (x & 0xfff) << 20;
}

// The U-format part of a U+I 32-bit immediate:
static inline uint32_t riscv_encode_imm_u_hi(uint32_t x) {
    // We will add a signed 12 bit constant to the "lui" value,
    // so we need to correct for the carry here.
    x += (x & 0x800) << 1;
    return riscv_encode_imm_u(x);
}

// B format, e.g. bgeu
static inline uint32_t riscv_encode_imm_b(uint32_t x) {
    return
        (((x >> 12) & 0x01) << 31) |
        (((x >>  5) & 0x3f) << 25) |
        (((x >>  1) & 0x0f) <<  8) |
        (((x >> 11) & 0x01) <<  7);
}

// S format, e.g. sw
static inline uint32_t riscv_encode_imm_s(uint32_t x) {
    return
        (((x >>  5) & 0x7f) << 25) |
        (((x >>  0) & 0x1f) <<  7);
}

// J format, e.g. jal
static inline uint32_t riscv_encode_imm_j(uint32_t x) {
    return
        (((x >> 20) & 0x001) << 31) |
        (((x >>  1) & 0x3ff) << 21) |
        (((x >> 11) & 0x001) << 20) |
        (((x >> 12) & 0x0ff) << 12);
}

// CJ format, e.g. c.jal
static inline uint16_t riscv_encode_imm_cj(uint32_t x) {
    return (uint16_t)(
        (((x >> 11) & 0x1) << 12) |
        (((x >>  4) & 0x1) << 11) |
        (((x >>  8) & 0x3) <<  9) |
        (((x >> 10) & 0x1) <<  8) |
        (((x >>  6) & 0x1) <<  7) |
        (((x >>  7) & 0x1) <<  6) |
        (((x >>  1) & 0x7) <<  3) |
        (((x >>  5) & 0x1) <<  2)
    );
}

// CB format, e.g. c.beqz
static inline uint16_t riscv_encode_imm_cb(uint32_t x) {
    return (uint16_t)(
        (((x >> 8) & 0x1) << 12) |
        (((x >> 3) & 0x3) << 10) |
        (((x >> 6) & 0x3) <<  5) |
        (((x >> 1) & 0x3) <<  3) |
        (((x >> 5) & 0x1) <<  2)
    );
}

// CI format, e.g. c.addi
static inline uint16_t riscv_encode_imm_ci(uint32_t x) {
    return (uint16_t)(
        (((x >> 5) & 0x01) << 12) |
        (((x >> 0) & 0x1f) <<  2)
    );
}

#ifdef __cplusplus
}
#endif

#endif
#endif
