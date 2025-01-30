/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_RESETS_H
#define _HARDWARE_RESETS_H

#include "pico.h"
#include "hardware/structs/resets.h"

/** \file hardware/resets.h
 *  \defgroup hardware_resets hardware_resets
 *
 * \brief Hardware Reset API
 *
 * The reset controller allows software control of the resets to all of the peripherals that are not
 * critical to boot the processor in the RP-series microcontroller.
 *
 * \subsubsection reset_bitmask
 * \addtogroup hardware_resets
 *
 * Multiple blocks are referred to using a bitmask as follows:
 *
 * Block to reset | Bit
 * ---------------|----
 * USB | 24
 * UART 1 | 23
 * UART 0 | 22
 * Timer | 21
 * TB Manager | 20
 * SysInfo | 19
 * System Config | 18
 * SPI 1 | 17
 * SPI 0 | 16
 * RTC | 15
 * PWM | 14
 * PLL USB | 13
 * PLL System | 12
 * PIO 1 | 11
 * PIO 0 | 10
 * Pads - QSPI | 9
 * Pads - bank 0 | 8
 * JTAG | 7
 * IO Bank 1 | 6
 * IO Bank 0 | 5
 * I2C 1 | 4
 * I2C 0 | 3
 * DMA | 2
 * Bus Control | 1
 * ADC 0 | 0
 *
 * \subsection reset_example Example
 * \addtogroup hardware_resets
 * \include hello_reset.c
 */

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_HARDWARE_RESETS, Enable/disable assertions in the hardware_resets module, type=bool, default=0, group=hardware_adc
#ifndef PARAM_ASSERTIONS_ENABLED_HARDWARE_RESETS
#ifdef PARAM_ASSERTIONS_ENABLED_RESET // backwards compatibility with SDK < 2.0.0
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_RESETS PARAM_ASSERTIONS_ENABLED_RESET
#else
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_RESETS 0
#endif
#endif
#ifdef __cplusplus
extern "C" {
#endif

static __force_inline  void reset_block_reg_mask(io_rw_32 *reset, uint32_t mask) {
    hw_set_bits(reset, mask);
}

static __force_inline  void unreset_block_reg_mask(io_rw_32 *reset, uint32_t mask) {
    hw_clear_bits(reset, mask);
}

static __force_inline void unreset_block_reg_mask_wait_blocking(io_rw_32 *reset, io_ro_32 *reset_done, uint32_t mask) {
    hw_clear_bits(reset, mask);
    while (~*reset_done & mask)
        tight_loop_contents();
}

/// \tag::reset_funcs[]

/*! \brief Reset the specified HW blocks
 *  \ingroup hardware_resets
 *
 * \param bits Bit pattern indicating blocks to reset. See \ref reset_bitmask
 */
static __force_inline void reset_block_mask(uint32_t bits) {
    reset_block_reg_mask(&resets_hw->reset, bits);
}

/*! \brief bring specified HW blocks out of reset
 *  \ingroup hardware_resets
 *
 * \param bits Bit pattern indicating blocks to unreset. See \ref reset_bitmask
 */
static __force_inline void unreset_block_mask(uint32_t bits) {
    unreset_block_reg_mask(&resets_hw->reset, bits);
}

/*! \brief Bring specified HW blocks out of reset and wait for completion
 *  \ingroup hardware_resets
 *
 * \param bits Bit pattern indicating blocks to unreset. See \ref reset_bitmask
 */
static __force_inline void unreset_block_mask_wait_blocking(uint32_t bits) {
    unreset_block_reg_mask_wait_blocking(&resets_hw->reset, &resets_hw->reset_done, bits);
}

/// \end::reset_funcs[]

#ifndef HARDWARE_RESETS_ENABLE_SDK1XX_COMPATIBILITY
#define HARDWARE_RESETS_ENABLE_SDK1XX_COMPATIBILITY 1
#endif

#if HARDWARE_RESETS_ENABLE_SDK1XX_COMPATIBILITY
static __force_inline void reset_block(uint32_t bits) {
    reset_block_mask(bits);
}

static __force_inline void unreset_block(uint32_t bits) {
    unreset_block_mask(bits);
}

static __force_inline void unreset_block_wait(uint32_t bits) {
    return unreset_block_mask_wait_blocking(bits);
}
#endif

/*! \brief Reset the specified HW block
 *  \ingroup hardware_resets
 *
 * \param block_num the block number
 */
static inline void reset_block_num(uint32_t block_num) {
    reset_block_reg_mask(&resets_hw->reset, 1u << block_num);
}

/*! \brief bring specified HW block out of reset
 *  \ingroup hardware_resets
 *
 * \param block_num the block number
 */
static inline void unreset_block_num(uint block_num) {
    invalid_params_if(HARDWARE_RESETS, block_num > NUM_RESETS);
    unreset_block_reg_mask(&resets_hw->reset, 1u << block_num);
}

/*! \brief Bring specified HW block out of reset and wait for completion
 *  \ingroup hardware_resets
 *
 * \param block_num the block number
 */
static inline void unreset_block_num_wait_blocking(uint block_num) {
    invalid_params_if(HARDWARE_RESETS, block_num > NUM_RESETS);
    unreset_block_reg_mask_wait_blocking(&resets_hw->reset, &resets_hw->reset_done, 1u << block_num);
}

/*! \brief Reset the specified HW block, and then bring at back out of reset and wait for completion
 *  \ingroup hardware_resets
 *
 * \param block_num the block number
 */
static inline void reset_unreset_block_num_wait_blocking(uint block_num) {
    invalid_params_if(HARDWARE_RESETS, block_num > NUM_RESETS);
    reset_block_reg_mask(&resets_hw->reset, 1u << block_num);
    unreset_block_reg_mask_wait_blocking(&resets_hw->reset, &resets_hw->reset_done, 1u << block_num);
}

#ifdef __cplusplus
}
#endif

#endif
