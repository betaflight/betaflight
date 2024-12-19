/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_PIO_H
#define _HARDWARE_PIO_H

#include "pico.h"
#include "hardware/address_mapped.h"
#include "hardware/structs/pio.h"
#include "hardware/gpio.h"
#include "hardware/regs/dreq.h"
#include "hardware/pio_instructions.h"

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_HARDWARE_PIO, Enable/disable assertions in the hardware_pio module, type=bool, default=0, group=hardware_pio
#ifndef PARAM_ASSERTIONS_ENABLED_HARDWARE_PIO
#ifdef PARAM_ASSERTIONS_ENABLED_PIO // backwards compatibility with SDK < 2.0.0
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_PIO PARAM_ASSERTIONS_ENABLED_PIO
#else
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_PIO 0
#endif
#endif

// PICO_CONFIG: PICO_PIO_VERSION, PIO hardware version, type=int, default=0 on RP2040 and 1 on RP2350, group=hardware_pio
#ifndef PICO_PIO_VERSION
#if PIO_GPIOBASE_BITS
#define PICO_PIO_VERSION 1
#else
#define PICO_PIO_VERSION 0
#endif
#endif
/** \file hardware/pio.h
 *  \defgroup hardware_pio hardware_pio
 *
 * \brief Programmable I/O (PIO) API
 *
 * A programmable input/output block (PIO) is a versatile hardware interface which
 * can support a number of different IO standards. 
 * 
 * \if rp2040_specific
 * There are two PIO blocks in the RP2040.
 * \endif
 *
 * \if rp2350_specific
 * There are three PIO blocks in the RP2350
 * \endif
 *
 * Each PIO is programmable in the same sense as a processor: the four state machines independently
 * execute short, sequential programs, to manipulate GPIOs and transfer data. Unlike a general
 * purpose processor, PIO state machines are highly specialised for IO, with a focus on determinism,
 * precise timing, and close integration with fixed-function hardware. Each state machine is equipped
 * with:
 *  * Two 32-bit shift registers – either direction, any shift count
 *  * Two 32-bit scratch registers
 *  * 4×32 bit bus FIFO in each direction (TX/RX), reconfigurable as 8×32 in a single direction
 *  * Fractional clock divider (16 integer, 8 fractional bits)
 *  * Flexible GPIO mapping
 *  * DMA interface, sustained throughput up to 1 word per clock from system DMA
 *  * IRQ flag set/clear/status
 *
 * Full details of the PIO can be found in the appropriate RP-series datasheet. Note that there are
 * additional features in the RP2350 PIO implementation that mean care should be taken when writing PIO
 * code that needs to run on both the RP2040 and the RP2350.
 */

#ifdef __cplusplus
extern "C" {
#endif

static_assert(PIO_SM0_SHIFTCTRL_FJOIN_RX_LSB == PIO_SM0_SHIFTCTRL_FJOIN_TX_LSB + 1, "");

/** \brief FIFO join states
 *  \ingroup hardware_pio
 */
enum pio_fifo_join {
    PIO_FIFO_JOIN_NONE = 0,    ///< TX FIFO length=4 is used for transmit, RX FIFO length=4 is used for receive
    PIO_FIFO_JOIN_TX = 1,      ///< TX FIFO length=8 is used for transmit, RX FIFO is disabled
    PIO_FIFO_JOIN_RX = 2,      ///< RX FIFO length=8 is used for receive, TX FIFO is disabled
#if PICO_PIO_VERSION > 0
    PIO_FIFO_JOIN_TXGET = 4,   ///< TX FIFO length=4 is used for transmit, RX FIFO is disabled; space is used for "get" instructions or processor writes
    PIO_FIFO_JOIN_TXPUT = 8,   ///< TX FIFO length=4 is used for transmit, RX FIFO is disabled; space is used for "put" instructions or processor reads
    PIO_FIFO_JOIN_PUTGET = 12, ///< TX FIFO length=4 is used for transmit, RX FIFO is disabled; space is used for "put"/"get" instructions with no processor access
#endif
};

/** \brief MOV status types
 *  \ingroup hardware_pio
 */
enum pio_mov_status_type {
    STATUS_TX_LESSTHAN = 0,
    STATUS_RX_LESSTHAN = 1,
#if PICO_PIO_VERSION > 0
    STATUS_IRQ_SET = 2
#endif
};

typedef pio_hw_t *PIO;

/** Identifier for the first (PIO 0) hardware PIO instance (for use in PIO functions).
 *
 * e.g. pio_gpio_init(pio0, 5)
 *
 *  \ingroup hardware_pio
 */
#define pio0 pio0_hw

/** Identifier for the second (PIO 1) hardware PIO instance (for use in PIO functions).
 *
 * e.g. pio_gpio_init(pio1, 5)
 *
 *  \ingroup hardware_pio
 */
#define pio1 pio1_hw

#if NUM_PIOS > 2
/** Identifier for the second (PIO 1) hardware PIO instance (for use in PIO functions).
 *
 * e.g. pio_gpio_init(pio1, 5)
 *
 *  \ingroup hardware_pio
 */
#define pio2 pio2_hw
#endif

#if PICO_PIO_VERSION > 0
#ifndef PICO_PIO_USE_GPIO_BASE
// PICO_CONFIG: PICO_PIO_USE_GPIO_BASE, Enable code for handling more than 32 PIO pins, type=bool, default=true when supported and when the device has more than 32 pins, group=hardware_pio
#define PICO_PIO_USE_GPIO_BASE ((NUM_BANK0_GPIOS) > 32)
#endif
#endif

/**
 * \def PIO_NUM(pio)
 * \ingroup hardware_pio
 * \hideinitializer
 * \brief Returns the PIO number for a PIO instance
 *
 * Note this macro is intended to resolve at compile time, and does no parameter checking
 */
#ifndef PIO_NUM
static_assert(PIO1_BASE - PIO0_BASE == (1u << 20), "hardware layout mismatch");
#define PIO_NUM(pio) (((uintptr_t)(pio) - PIO0_BASE) >> 20)
#endif

/**
 * \def PIO_INSTANCE(pio_num)
 * \ingroup hardware_pio
 * \hideinitializer
 * \brief Returns the PIO instance with the given PIO number
 *
 * Note this macro is intended to resolve at compile time, and does no parameter checking
 */
#ifndef PIO_INSTANCE
static_assert(PIO1_BASE - PIO0_BASE == (1u << 20), "hardware layout mismatch");
#define PIO_INSTANCE(instance) ((pio_hw_t *)(PIO0_BASE + (instance) * (1u << 20)))
#endif

/**
 * \def PIO_FUNCSEL_NUM(pio, gpio)
 * \ingroup hardware_pio
 * \hideinitializer
 * \brief Returns \ref gpio_function_t needed to select the PIO function for the given PIO instance on the given GPIO
 *
 * Note this macro is intended to resolve at compile time, and does no parameter checking
 */
#ifndef PIO_FUNCSEL_NUM
#define PIO_FUNCSEL_NUM(pio, gpio) ((gpio_function_t) (GPIO_FUNC_PIO0 + PIO_NUM(pio)))
#endif

/**
 * \def PIO_DREQ_NUM(pio, sm, is_tx)
 * \ingroup hardware_pio
 * \hideinitializer
 * \brief Returns the \ref dreq_num_t used for pacing DMA transfers to or from a given state machine's FIFOs on this PIO instance.
 * If is_tx is true, then it is for transfers to the PIO state machine TX FIFO else for transfers from the PIO state machine RX FIFO.
 *
 * Note this macro is intended to resolve at compile time, and does no parameter checking
 */
#ifndef PIO_DREQ_NUM
static_assert(DREQ_PIO0_TX1 == DREQ_PIO0_TX0 + 1, "");
static_assert(DREQ_PIO0_TX2 == DREQ_PIO0_TX0 + 2, "");
static_assert(DREQ_PIO0_TX3 == DREQ_PIO0_TX0 + 3, "");
static_assert(DREQ_PIO0_RX0 == DREQ_PIO0_TX0 + NUM_PIO_STATE_MACHINES, "");
static_assert(DREQ_PIO1_RX0 == DREQ_PIO1_TX0 + NUM_PIO_STATE_MACHINES, "");
#define PIO_DREQ_NUM(pio, sm, is_tx) ((sm) + (((is_tx) ? 0 : NUM_PIO_STATE_MACHINES) + PIO_NUM(pio) * (DREQ_PIO1_TX0 - DREQ_PIO0_TX0)))
#endif

/**
 * \def PIO_IRQ_NUM(pio)
 * \ingroup hardware_pio
 * \hideinitializer
 * \brief Returns the \ref irq_num_t for processor interrupts from the given PIO instance
 *
 * Note this macro is intended to resolve at compile time, and does no parameter checking
 */
#ifndef PIO_IRQ_NUM
#define PIO_IRQ_NUM(pio, irqn) (PIO0_IRQ_0 + NUM_PIO_IRQS * PIO_NUM(pio) + (irqn))
#endif

/** \brief PIO state machine configuration
 *  \defgroup sm_config sm_config
 *  \ingroup hardware_pio
 *
 * A PIO block needs to be configured, these functions provide helpers to set up configuration
 * structures. See \ref pio_sm_set_config
 *
 */

/** \brief PIO Configuration structure
 *  \ingroup sm_config
 *
 * This structure is an in-memory representation of the configuration that can be applied to a PIO
 * state machine later using pio_sm_set_config() or pio_sm_init().
 */
typedef struct {
    uint32_t clkdiv;
    uint32_t execctrl;
    uint32_t shiftctrl;
    uint32_t pinctrl;
#if PICO_PIO_USE_GPIO_BASE
#define PINHI_ALL_PINCTRL_LSBS ((1u << PIO_SM0_PINCTRL_IN_BASE_LSB) | (1u << PIO_SM0_PINCTRL_OUT_BASE_LSB) | \
                               (1u << PIO_SM0_PINCTRL_SET_BASE_LSB) | (1u << PIO_SM0_PINCTRL_SIDESET_BASE_LSB))
static_assert( 0 == (0xff000000u & (PINHI_ALL_PINCTRL_LSBS * 0x1f)), "");
// note we put the jmp_ctrl pin starting at bit 24
#define PINHI_ALL_PIN_LSBS ((1u << 24) | (1u << PIO_SM0_PINCTRL_IN_BASE_LSB) | (1u << PIO_SM0_PINCTRL_OUT_BASE_LSB) | \
                               (1u << PIO_SM0_PINCTRL_SET_BASE_LSB) | (1u << PIO_SM0_PINCTRL_SIDESET_BASE_LSB))
    // each 5-bit field which would usually be used for the pin_base in pin_ctrl, is used for:
    // 0b11111 - corresponding field not specified
    // 0b00000 - pin is in range 0-15
    // 0b00001 - pin is in range 16-31
    // 0b00010 - pin is in range 32-47
    uint32_t pinhi;
#endif
} pio_sm_config;

static inline void check_sm_param(__unused uint sm) {
    valid_params_if(HARDWARE_PIO, sm < NUM_PIO_STATE_MACHINES);
}

static inline void check_sm_mask(__unused uint mask) {
    valid_params_if(HARDWARE_PIO, mask < (1u << NUM_PIO_STATE_MACHINES));
}

static inline void check_pio_param(__unused PIO pio) {
#if NUM_PIOS == 2
    valid_params_if(HARDWARE_PIO, pio == pio0 || pio == pio1);
#elif NUM_PIOS == 3
    valid_params_if(HARDWARE_PIO, pio == pio0 || pio == pio1 || pio == pio2);
#endif
}

static inline void check_pio_pin_param(__unused uint pin) {
#if !PICO_PIO_USE_GPIO_BASE
    invalid_params_if(HARDWARE_PIO, pin >= 32);
#else
    // pin base allows us to move up 16 pins at a time
    invalid_params_if(HARDWARE_PIO, pin >= ((NUM_BANK0_GPIOS + 15u)&~15u));
#endif
}

/*! \brief Set the base of the 'out' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * 'out' pins can overlap with the 'in', 'set' and 'sideset' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param out_base 0-31 First pin to set as output
 */
static inline void sm_config_set_out_pin_base(pio_sm_config *c, uint out_base) {
    check_pio_pin_param(out_base);
    c->pinctrl = (c->pinctrl & ~PIO_SM0_PINCTRL_OUT_BASE_BITS) |
                 ((out_base & 31) << PIO_SM0_PINCTRL_OUT_BASE_LSB);
#if PICO_PIO_USE_GPIO_BASE
    c->pinhi = (c->pinhi & ~(31u << PIO_SM0_PINCTRL_OUT_BASE_LSB)) |
                    ((out_base >> 4) << PIO_SM0_PINCTRL_OUT_BASE_LSB);
#endif
}

/*! \brief Set the number of 'out' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * 'out' pins can overlap with the 'in', 'set' and 'sideset' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param out_count 0-32 Number of pins to set.
 */
static inline void sm_config_set_out_pin_count(pio_sm_config *c, uint out_count) {
    valid_params_if(HARDWARE_PIO, out_count <= 32);
    c->pinctrl = (c->pinctrl & ~PIO_SM0_PINCTRL_OUT_COUNT_BITS) |
                 (out_count << PIO_SM0_PINCTRL_OUT_COUNT_LSB);
}

/*! \brief Set the 'out' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * 'out' pins can overlap with the 'in', 'set' and 'sideset' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param out_base 0-31 First pin to set as output
 * \param out_count 0-32 Number of pins to set.
 */
static inline void sm_config_set_out_pins(pio_sm_config *c, uint out_base, uint out_count) {
    sm_config_set_out_pin_base(c, out_base);
    sm_config_set_out_pin_count(c, out_count);
}

/*! \brief Set the base of the 'set' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * 'set' pins can overlap with the 'in', 'out' and 'sideset' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param set_base 0-31 First pin to set as
 */
static inline void sm_config_set_set_pin_base(pio_sm_config *c, uint set_base) {
    check_pio_pin_param(set_base);
    c->pinctrl = (c->pinctrl & ~PIO_SM0_PINCTRL_SET_BASE_BITS) |
                 ((set_base & 31) << PIO_SM0_PINCTRL_SET_BASE_LSB);
#if PICO_PIO_USE_GPIO_BASE
    c->pinhi = (c->pinhi & ~(31u << PIO_SM0_PINCTRL_SET_BASE_LSB)) |
                    ((set_base >> 4) << PIO_SM0_PINCTRL_SET_BASE_LSB);
#endif
}

/*! \brief Set the count of 'set' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * 'set' pins can overlap with the 'in', 'out' and 'sideset' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param set_count 0-5 Number of pins to set.
 */
static inline void sm_config_set_set_pin_count(pio_sm_config *c, uint set_count) {
    valid_params_if(HARDWARE_PIO, set_count <= 5);
    c->pinctrl = (c->pinctrl & ~PIO_SM0_PINCTRL_SET_COUNT_BITS) |
                 (set_count << PIO_SM0_PINCTRL_SET_COUNT_LSB);
}

/*! \brief Set the 'set' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * 'set' pins can overlap with the 'in', 'out' and 'sideset' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param set_base 0-31 First pin to set as
 * \param set_count 0-5 Number of pins to set.
 */
static inline void sm_config_set_set_pins(pio_sm_config *c, uint set_base, uint set_count) {
    sm_config_set_set_pin_base(c, set_base);
    sm_config_set_set_pin_count(c, set_count);
}

/*! \brief Set the base of the 'in' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * 'in' pins can overlap with the 'out', 'set' and 'sideset' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param in_base 0-31 First pin to use as input
 */
static inline void sm_config_set_in_pin_base(pio_sm_config *c, uint in_base) {
    check_pio_pin_param(in_base);
    c->pinctrl = (c->pinctrl & ~PIO_SM0_PINCTRL_IN_BASE_BITS) |
                 ((in_base & 31) << PIO_SM0_PINCTRL_IN_BASE_LSB);
#if PICO_PIO_USE_GPIO_BASE
    c->pinhi = (c->pinhi & ~(31u << PIO_SM0_PINCTRL_IN_BASE_LSB)) |
                    ((in_base >> 4) << PIO_SM0_PINCTRL_IN_BASE_LSB);
#endif
}

/*! \brief Set the base for the 'in' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * 'in' pins can overlap with the 'out', 'set' and 'sideset' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param in_base 0-31 First pin to use as input
 */
static inline void sm_config_set_in_pins(pio_sm_config *c, uint in_base) {
    sm_config_set_in_pin_base(c, in_base);
}

/*! \brief Set the count of 'in' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * When reading pins using the IN pin mapping, this many (low) bits will be read, with the rest taking
 * the value zero.
 *
 * \if rp2040_specific
 * RP2040 does not have the ability to mask unused input pins, so the in_count must be 32
 * \endif
 *
 * \param c Pointer to the configuration structure to modify
 * \param in_count 1-32 The number of pins to include when reading via the IN pin mapping
 */
static inline void sm_config_set_in_pin_count(pio_sm_config *c, uint in_count) {
#if PICO_PIO_VERSION == 0
    // can't be changed from 32 on PIO v0
    ((void)c);
    valid_params_if(HARDWARE_PIO, in_count == 32);
#else
    valid_params_if(HARDWARE_PIO, in_count && in_count <= 32);
    c->shiftctrl = (c->shiftctrl & ~PIO_SM0_SHIFTCTRL_IN_COUNT_BITS) |
                   ((in_count & 0x1fu) << PIO_SM0_SHIFTCTRL_IN_COUNT_LSB);
#endif
}

/*! \brief Set the base of the 'sideset' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * 'sideset' pins can overlap with the 'in', 'out' and 'set' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param sideset_base 0-31 base pin for 'side set'
 */
static inline void sm_config_set_sideset_pin_base(pio_sm_config *c, uint sideset_base) {
    check_pio_pin_param(sideset_base);
    c->pinctrl = (c->pinctrl & ~PIO_SM0_PINCTRL_SIDESET_BASE_BITS) |
                 ((sideset_base & 31) << PIO_SM0_PINCTRL_SIDESET_BASE_LSB);
#if PICO_PIO_USE_GPIO_BASE
    c->pinhi = (c->pinhi & ~(31u << PIO_SM0_PINCTRL_SIDESET_BASE_LSB)) |
                    ((sideset_base >> 4) << PIO_SM0_PINCTRL_SIDESET_BASE_LSB);
#endif
}

/*! \brief Set the 'sideset' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * This method is identical to \ref sm_config_set_sideset_pin_base, and is provided
 * for backwards compatibility
 *
 * 'sideset' pins can overlap with the 'in', 'out' and 'set' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param sideset_base 0-31 base pin for 'side set'
 */
static inline void sm_config_set_sideset_pins(pio_sm_config *c, uint sideset_base) {
    sm_config_set_sideset_pin_base(c, sideset_base);
}

/*! \brief Set the 'sideset' options in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param bit_count Number of bits to steal from delay field in the instruction for use of side set (max 5)
 * \param optional True if the topmost side set bit is used as a flag for whether to apply side set on that instruction
 * \param pindirs True if the side set affects pin directions rather than values
 */
static inline void sm_config_set_sideset(pio_sm_config *c, uint bit_count, bool optional, bool pindirs) {
    valid_params_if(HARDWARE_PIO, bit_count <= 5);
    valid_params_if(HARDWARE_PIO, !optional || bit_count >= 1);
    c->pinctrl = (c->pinctrl & ~PIO_SM0_PINCTRL_SIDESET_COUNT_BITS) |
                 (bit_count << PIO_SM0_PINCTRL_SIDESET_COUNT_LSB);
    c->execctrl = (c->execctrl & ~(PIO_SM0_EXECCTRL_SIDE_EN_BITS | PIO_SM0_EXECCTRL_SIDE_PINDIR_BITS)) |
                  (bool_to_bit(optional) << PIO_SM0_EXECCTRL_SIDE_EN_LSB) |
                  (bool_to_bit(pindirs) << PIO_SM0_EXECCTRL_SIDE_PINDIR_LSB);
}

/*! \brief Set the state machine clock divider (from integer and fractional parts - 16:8) in a state machine configuration
 *  \ingroup sm_config
 *
 * The clock divider can slow the state machine's execution to some rate below
 * the system clock frequency, by enabling the state machine on some cycles
 * but not on others, in a regular pattern. This can be used to generate e.g.
 * a given UART baud rate. See the datasheet for further detail.
 *
 * \param c Pointer to the configuration structure to modify
 * \param div_int Integer part of the divisor
 * \param div_frac Fractional part in 1/256ths
 * \sa sm_config_set_clkdiv()
 */
static inline void sm_config_set_clkdiv_int_frac(pio_sm_config *c, uint16_t div_int, uint8_t div_frac) {
    invalid_params_if(HARDWARE_PIO, div_int == 0 && div_frac != 0);
    c->clkdiv =
            (((uint)div_frac) << PIO_SM0_CLKDIV_FRAC_LSB) |
            (((uint)div_int) << PIO_SM0_CLKDIV_INT_LSB);
}

static inline void pio_calculate_clkdiv_from_float(float div, uint16_t *div_int, uint8_t *div_frac) {
    valid_params_if(HARDWARE_PIO, div >= 1 && div <= 65536);
    *div_int = (uint16_t)div;
    if (*div_int == 0) {
        *div_frac = 0;
    } else {
        *div_frac = (uint8_t)((div - (float)*div_int) * (1u << 8u));
    }
}

/*! \brief Set the state machine clock divider (from a floating point value) in a state machine configuration
 *  \ingroup sm_config
 *
 * The clock divider slows the state machine's execution by masking the
 * system clock on some cycles, in a repeating pattern, so that the state
 * machine does not advance. Effectively this produces a slower clock for the
 * state machine to run from, which can be used to generate e.g. a particular
 * UART baud rate. See the datasheet for further detail.
 *
 * \param c Pointer to the configuration structure to modify
 * \param div The fractional divisor to be set. 1 for full speed. An integer clock divisor of n
 *  will cause the state machine to run 1 cycle in every n.
 *  Note that for small n, the jitter introduced by a fractional divider (e.g. 2.5) may be unacceptable
 *  although it will depend on the use case.
 */
static inline void sm_config_set_clkdiv(pio_sm_config *c, float div) {
    uint16_t div_int;
    uint8_t div_frac;
    pio_calculate_clkdiv_from_float(div, &div_int, &div_frac);
    sm_config_set_clkdiv_int_frac(c, div_int, div_frac);
}

/*! \brief Set the wrap addresses in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param wrap_target the instruction memory address to wrap to
 * \param wrap        the instruction memory address after which to set the program counter to wrap_target
 *                    if the instruction does not itself update the program_counter
 */
static inline void sm_config_set_wrap(pio_sm_config *c, uint wrap_target, uint wrap) {
    valid_params_if(HARDWARE_PIO, wrap < PIO_INSTRUCTION_COUNT);
    valid_params_if(HARDWARE_PIO, wrap_target < PIO_INSTRUCTION_COUNT);
    c->execctrl = (c->execctrl & ~(PIO_SM0_EXECCTRL_WRAP_TOP_BITS | PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS)) |
                  (wrap_target << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB) |
                  (wrap << PIO_SM0_EXECCTRL_WRAP_TOP_LSB);
}

/*! \brief Set the 'jmp' pin in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param pin The raw GPIO pin number to use as the source for a `jmp pin` instruction
 */
static inline void sm_config_set_jmp_pin(pio_sm_config *c, uint pin) {
    check_pio_pin_param(pin);
    c->execctrl = (c->execctrl & ~PIO_SM0_EXECCTRL_JMP_PIN_BITS) |
                  ((pin & 31) << PIO_SM0_EXECCTRL_JMP_PIN_LSB);
#if PICO_PIO_USE_GPIO_BASE
    c->pinhi = (c->pinhi & ~(31u << 24)) |
               ((pin >> 4) << 24);
#endif
}

/*! \brief Setup 'in' shifting parameters in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param shift_right true to shift ISR to right, false to shift ISR to left
 * \param autopush whether autopush is enabled
 * \param push_threshold threshold in bits to shift in before auto/conditional re-pushing of the ISR
 */
static inline void sm_config_set_in_shift(pio_sm_config *c, bool shift_right, bool autopush, uint push_threshold) {
    valid_params_if(HARDWARE_PIO, push_threshold <= 32);
    c->shiftctrl = (c->shiftctrl &
                    ~(PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS |
                      PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS |
                      PIO_SM0_SHIFTCTRL_PUSH_THRESH_BITS)) |
                   (bool_to_bit(shift_right) << PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_LSB) |
                   (bool_to_bit(autopush) << PIO_SM0_SHIFTCTRL_AUTOPUSH_LSB) |
                   ((push_threshold & 0x1fu) << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB);
}

/*! \brief Setup 'out' shifting parameters in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param shift_right true to shift OSR to right, false to shift OSR to left
 * \param autopull whether autopull is enabled
 * \param pull_threshold threshold in bits to shift out before auto/conditional re-pulling of the OSR
 */
static inline void sm_config_set_out_shift(pio_sm_config *c, bool shift_right, bool autopull, uint pull_threshold) {
    valid_params_if(HARDWARE_PIO, pull_threshold <= 32);
    c->shiftctrl = (c->shiftctrl &
                    ~(PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS |
                      PIO_SM0_SHIFTCTRL_AUTOPULL_BITS |
                      PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS)) |
                   (bool_to_bit(shift_right) << PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_LSB) |
                   (bool_to_bit(autopull) << PIO_SM0_SHIFTCTRL_AUTOPULL_LSB) |
                   ((pull_threshold & 0x1fu) << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB);
}

/*! \brief Setup the FIFO joining in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param join Specifies the join type. \see enum pio_fifo_join
 */
static inline void sm_config_set_fifo_join(pio_sm_config *c, enum pio_fifo_join join) {
    valid_params_if(HARDWARE_PIO, join == PIO_FIFO_JOIN_NONE || join == PIO_FIFO_JOIN_TX || join == PIO_FIFO_JOIN_RX
#if PICO_PIO_VERSION > 0
        || join == PIO_FIFO_JOIN_TXPUT || join == PIO_FIFO_JOIN_TXGET || join == PIO_FIFO_JOIN_PUTGET
#endif
    );
#if PICO_PIO_VERSION == 0
    c->shiftctrl = (c->shiftctrl & (uint)~(PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS | PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS)) |
                   (((uint)join) << PIO_SM0_SHIFTCTRL_FJOIN_TX_LSB);
#else
    c->shiftctrl = (c->shiftctrl & (uint)~(PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS | PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS |
                                           PIO_SM0_SHIFTCTRL_FJOIN_RX_PUT_BITS | PIO_SM0_SHIFTCTRL_FJOIN_RX_GET_BITS)) |
                   (((uint)(join & 3)) << PIO_SM0_SHIFTCTRL_FJOIN_TX_LSB) |
                   (((uint)(join >> 2)) << PIO_SM0_SHIFTCTRL_FJOIN_RX_GET_LSB);
#endif
}

/*! \brief Set special 'out' operations in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param sticky to enable 'sticky' output (i.e. re-asserting most recent OUT/SET pin values on subsequent cycles)
 * \param has_enable_pin true to enable auxiliary OUT enable pin
 * \param enable_pin_index pin index for auxiliary OUT enable
 */
static inline void sm_config_set_out_special(pio_sm_config *c, bool sticky, bool has_enable_pin, uint enable_pin_index) {
    c->execctrl = (c->execctrl &
                   (uint)~(PIO_SM0_EXECCTRL_OUT_STICKY_BITS | PIO_SM0_EXECCTRL_INLINE_OUT_EN_BITS |
                     PIO_SM0_EXECCTRL_OUT_EN_SEL_BITS)) |
                  (bool_to_bit(sticky) << PIO_SM0_EXECCTRL_OUT_STICKY_LSB) |
                  (bool_to_bit(has_enable_pin) << PIO_SM0_EXECCTRL_INLINE_OUT_EN_LSB) |
                  ((enable_pin_index << PIO_SM0_EXECCTRL_OUT_EN_SEL_LSB) & PIO_SM0_EXECCTRL_OUT_EN_SEL_BITS);
}

/*! \brief Set source for 'mov status' in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param status_sel the status operation selector. \see enum pio_mov_status_type
 * \param status_n parameter for the mov status operation (currently a bit count)
 */
static inline void sm_config_set_mov_status(pio_sm_config *c, enum pio_mov_status_type status_sel, uint status_n) {
    valid_params_if(HARDWARE_PIO,
                    status_sel == STATUS_TX_LESSTHAN || status_sel == STATUS_RX_LESSTHAN
#if PICO_PIO_VERSION > 0
                    || status_sel == STATUS_IRQ_SET
#endif
    );
    c->execctrl = (c->execctrl
                  & ~(PIO_SM0_EXECCTRL_STATUS_SEL_BITS | PIO_SM0_EXECCTRL_STATUS_N_BITS))
                  | ((((uint)status_sel) << PIO_SM0_EXECCTRL_STATUS_SEL_LSB) & PIO_SM0_EXECCTRL_STATUS_SEL_BITS)
                  | ((status_n << PIO_SM0_EXECCTRL_STATUS_N_LSB) & PIO_SM0_EXECCTRL_STATUS_N_BITS);
}

/*! \brief  Get the default state machine configuration
 *  \ingroup sm_config
 *
 * Setting | Default
 * --------|--------
 * Out Pins | 32 starting at 0
 * Set Pins | 0 starting at 0
 * In Pins | 32 starting at 0
 * Side Set Pins (base) | 0
 * Side Set | disabled
 * Wrap | wrap=31, wrap_to=0
 * In Shift | shift_direction=right, autopush=false, push_threshold=32
 * Out Shift | shift_direction=right, autopull=false, pull_threshold=32
 * Jmp Pin | 0
 * Out Special | sticky=false, has_enable_pin=false, enable_pin_index=0
 * Mov Status | status_sel=STATUS_TX_LESSTHAN, n=0
 *
 * \return the default state machine configuration which can then be modified.
 */
static inline pio_sm_config pio_get_default_sm_config(void) {
    pio_sm_config c = {0};
#if PICO_PIO_USE_GPIO_BASE
    c.pinhi = -1;
#endif
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    sm_config_set_wrap(&c, 0, 31);
    sm_config_set_in_shift(&c, true, false, 32);
    sm_config_set_out_shift(&c, true, false, 32);
    return c;
}

/*! \brief Return the base GPIO base for the PIO instance
 *  \ingroup hardware_pio
 *
 * \if rp2040_specific
 * This method always return 0 in RP2040
 * \endif
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \return the current GPIO base for the PIO instance
  */
static inline uint pio_get_gpio_base(PIO pio) {
#if PICO_PIO_VERSION > 0
    return pio->gpiobase;
#else
    ((void)pio);
    return 0;
#endif
}

/*! \brief Apply a state machine configuration to a state machine
 *  \ingroup hardware_pio
 *
 * \param pio Handle to PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param config the configuration to apply
 * \return PICO_OK (0) on success, negative error code otherwise
*/
static inline int pio_sm_set_config(PIO pio, uint sm, const pio_sm_config *config) {
    check_pio_param(pio);
    check_sm_param(sm);
    pio->sm[sm].clkdiv = config->clkdiv;
    pio->sm[sm].execctrl = config->execctrl;
    pio->sm[sm].shiftctrl = config->shiftctrl;
#if PICO_PIO_USE_GPIO_BASE
    uint used = (~config->pinhi >> 4) & PINHI_ALL_PIN_LSBS;
    // configs that use pins 0-15
    uint gpio_under_16 = (~config->pinhi) & (~config->pinhi >> 1) & used;
    // configs that use pins 32-47
    uint gpio_over_32 = (config->pinhi >> 1) & used;
    uint gpio_base = pio_get_gpio_base(pio);
    invalid_params_if_and_return(PIO, gpio_under_16 && gpio_base, PICO_ERROR_BAD_ALIGNMENT);
    invalid_params_if_and_return(PIO, gpio_over_32 && !gpio_base, PICO_ERROR_BAD_ALIGNMENT);
    // flip the top bit of any used (pinctrl) values to turn:
    // bit6(32) + 0-15  -> base(16) + 16-31
    // bit6(0)  + 16-31 -> base(16) + 0-15
    pio->sm[sm].pinctrl = config->pinctrl ^ (gpio_base ? ((used << 12) >> 8) : 0);
#else
    pio->sm[sm].pinctrl = config->pinctrl;
#endif
    return PICO_OK;
}

/*! \brief Return the instance number of a PIO instance
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \return the PIO instance number (0, 1, ...)
 */
static inline uint pio_get_index(PIO pio) {
    check_pio_param(pio);
    return PIO_NUM(pio);
}

/*! \brief Return the funcsel number of a PIO instance
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \return the PIO instance number (0, 1, ...)
 * \see gpio_function
 */
static inline uint pio_get_funcsel(PIO pio) {
    check_pio_param(pio);
    return PIO_FUNCSEL_NUM(pio, 0); // note GPIO currently unused, so won't bother updating API
}

/*! \brief Convert PIO instance to hardware instance
 *  \ingroup hardware_pio
 *
 * \param instance Instance of PIO, 0 or 1
 * \return the PIO hardware instance
 */
static inline PIO pio_get_instance(uint instance) {
    invalid_params_if(HARDWARE_PIO, instance >= NUM_PIOS);
    return PIO_INSTANCE(instance);
}

/*! \brief Setup the function select for a GPIO to use output from the given PIO instance
 *  \ingroup hardware_pio
 *
 * PIO appears as an alternate function in the GPIO muxing, just like an SPI
 * or UART. This function configures that multiplexing to connect a given PIO
 * instance to a GPIO. Note that this is not necessary for a state machine to
 * be able to read the *input* value from a GPIO, but only for it to set the
 * output value or output enable.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param pin the GPIO pin whose function select to set
 */
static inline void pio_gpio_init(PIO pio, uint pin) {
    check_pio_param(pio);
    valid_params_if(HARDWARE_PIO, pin < NUM_BANK0_GPIOS);
    gpio_set_function(pin, PIO_FUNCSEL_NUM(pio, pin));
}

/*! \brief Return the DREQ to use for pacing transfers to/from a particular state machine FIFO
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param is_tx true for sending data to the state machine, false for receiving data from the state machine
 */
static inline uint pio_get_dreq(PIO pio, uint sm, bool is_tx) {
    check_pio_param(pio);
    check_sm_param(sm);
    return PIO_DREQ_NUM(pio, sm, is_tx);
}

typedef struct pio_program {
    const uint16_t *instructions;
    uint8_t length;
    int8_t origin; // required instruction memory origin or -1
    uint8_t pio_version;
#if PICO_PIO_VERSION > 0
    uint8_t used_gpio_ranges; // bitmap with one bit per 16 pins
#endif
} pio_program_t;

/*! \brief Set the base GPIO base for the PIO instance
 *  \ingroup hardware_pio
 *
 * Since an individual PIO accesses only 32 pins, to be able to access more pins, the PIO
 * instance must specify a base GPIO where the instance's "pin 0" maps. For RP2350 the valid
 * values are 0 and 16, indicating the PIO instance has access to pins 0-31, or 16-47 respectively.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param gpio_base the GPIO base (either 0 or 16)
 * \return PICO_OK (0) on success, error code otherwise
 */
int pio_set_gpio_base(PIO pio, uint gpio_base);

/*! \brief Determine whether the given program can (at the time of the call) be loaded onto the PIO instance
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param program the program definition
 * \return true if the program can be loaded; false if there is not suitable space in the instruction memory
 */
bool pio_can_add_program(PIO pio, const pio_program_t *program);

/*! \brief Determine whether the given program can (at the time of the call) be loaded onto the PIO instance starting at a particular location
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param program the program definition
 * \param offset the instruction memory offset wanted for the start of the program
 * \return true if the program can be loaded at that location; false if there is not space in the instruction memory
 */
bool pio_can_add_program_at_offset(PIO pio, const pio_program_t *program, uint offset);

/*! \brief Attempt to load the program
 *  \ingroup hardware_pio
 *
 * \see pio_can_add_program() if you need to check whether the program can be loaded
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param program the program definition
 * \return the instruction memory offset the program is loaded at, or negative for error (for
 * backwards compatibility with prior SDK the error value is -1 i.e. PICO_ERROR_GENERIC)
 */
int pio_add_program(PIO pio, const pio_program_t *program);

/*! \brief Attempt to load the program at the specified instruction memory offset
 *  \ingroup hardware_pio
 *
 * \see pio_can_add_program_at_offset() if you need to check whether the program can be loaded
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param program the program definition
 * \param offset the instruction memory offset wanted for the start of the program
 * \return the instruction memory offset the program is loaded at, or negative for error (for
 * backwards compatibility with prior SDK the error value is -1 i.e. PICO_ERROR_GENERIC)
 */
int pio_add_program_at_offset(PIO pio, const pio_program_t *program, uint offset);

/*! \brief Remove a program from a PIO instance's instruction memory
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param program the program definition
 * \param loaded_offset the loaded offset returned when the program was added
 */
void pio_remove_program(PIO pio, const pio_program_t *program, uint loaded_offset);

/*! \brief Clears all of a PIO instance's instruction memory
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 */
void pio_clear_instruction_memory(PIO pio);

/*! \brief Resets the state machine to a consistent state, and configures it
 *  \ingroup hardware_pio
 *
 * This method:
 * - Disables the state machine (if running)
 * - Clears the FIFOs
 * - Applies the configuration specified by 'config'
 * - Resets any internal state e.g. shift counters
 * - Jumps to the initial program location given by 'initial_pc'
 *
 * The state machine is left disabled on return from this call.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param initial_pc the initial program memory offset to run from
 * \param config the configuration to apply (or NULL to apply defaults)
 * \return PICO_OK, or < 0 for an error (see \enum pico_error_codes)
 */
int pio_sm_init(PIO pio, uint sm, uint initial_pc, const pio_sm_config *config);

/*! \brief Enable or disable a PIO state machine
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param enabled true to enable the state machine; false to disable
 */
static inline void pio_sm_set_enabled(PIO pio, uint sm, bool enabled) {
    check_pio_param(pio);
    check_sm_param(sm);
    pio->ctrl = (pio->ctrl & ~(1u << sm)) | (bool_to_bit(enabled) << sm);
}

/*! \brief Enable or disable multiple PIO state machines
 *  \ingroup hardware_pio
 *
 * Note that this method just sets the enabled state of the state machine;
 * if now enabled they continue exactly from where they left off.
 *
 * \see pio_enable_sm_mask_in_sync() if you wish to enable multiple state machines
 * and ensure their clock dividers are in sync.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param mask bit mask of state machine indexes to modify the enabled state of
 * \param enabled true to enable the state machines; false to disable
 */
static inline void pio_set_sm_mask_enabled(PIO pio, uint32_t mask, bool enabled) {
    check_pio_param(pio);
    check_sm_mask(mask);
    pio->ctrl = (pio->ctrl & ~mask) | (enabled ? mask : 0u);
}

#if PICO_PIO_VERSION > 0
/*! \brief Enable or disable multiple PIO state machines
 *  \ingroup hardware_pio
 *
 * Note that this method just sets the enabled state of the state machine;
 * if now enabled they continue exactly from where they left off.
 *
 * \see pio_enable_sm_mask_in_sync() if you wish to enable multiple state machines
 * and ensure their clock dividers are in sync.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param mask_prev bit mask of state machine indexes to modify the enabled state of, in the next-lower numbered PIO instance
 * \param mask bit mask of state machine indexes to modify the enabled state of, in this PIO instance
 * \param mask bit mask of state machine indexes to modify the enabled state of, in the next-higher numbered PIO instance
 * \param enabled true to enable the state machines; false to disable
 */
static inline void pio_set_sm_multi_mask_enabled(PIO pio, uint32_t mask_prev, uint32_t mask, uint32_t mask_next, bool enabled) {
    check_pio_param(pio);
    check_sm_mask(mask);
    pio->ctrl = (pio->ctrl & ~(mask << PIO_CTRL_SM_ENABLE_LSB)) |
                (enabled ? ((mask << PIO_CTRL_SM_ENABLE_LSB) & PIO_CTRL_SM_ENABLE_BITS) : 0) |
                (enabled ? PIO_CTRL_NEXTPREV_SM_ENABLE_BITS : PIO_CTRL_NEXTPREV_SM_DISABLE_BITS) |
                ((mask_prev << PIO_CTRL_PREV_PIO_MASK_LSB) & PIO_CTRL_PREV_PIO_MASK_BITS) |
                ((mask_next << PIO_CTRL_NEXT_PIO_MASK_LSB) & PIO_CTRL_NEXT_PIO_MASK_BITS);

}
#endif

/*! \brief Restart a state machine with a known state
 *  \ingroup hardware_pio
 *
 * This method clears the ISR, shift counters, clock divider counter
 * pin write flags, delay counter, latched EXEC instruction, and IRQ wait condition.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 */
static inline void pio_sm_restart(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    hw_set_bits(&pio->ctrl, 1u << (PIO_CTRL_SM_RESTART_LSB + sm));
}

/*! \brief Restart multiple state machine with a known state
 *  \ingroup hardware_pio
 *
 * This method clears the ISR, shift counters, clock divider counter
 * pin write flags, delay counter, latched EXEC instruction, and IRQ wait condition.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param mask bit mask of state machine indexes to modify the enabled state of
 */
static inline void pio_restart_sm_mask(PIO pio, uint32_t mask) {
    check_pio_param(pio);
    check_sm_mask(mask);
    hw_set_bits(&pio->ctrl, (mask << PIO_CTRL_SM_RESTART_LSB) & PIO_CTRL_SM_RESTART_BITS);
}

/*! \brief Restart a state machine's clock divider from a phase of 0
 *  \ingroup hardware_pio
 *
 * Each state machine's clock divider is a free-running piece of hardware,
 * that generates a pattern of clock enable pulses for the state machine,
 * based *only* on the configured integer/fractional divisor. The pattern of
 * running/halted cycles slows the state machine's execution to some
 * controlled rate.
 *
 * This function clears the divider's integer and fractional phase
 * accumulators so that it restarts this pattern from the beginning. It is
 * called automatically by pio_sm_init() but can also be called at a later
 * time, when you enable the state machine, to ensure precisely consistent
 * timing each time you load and run a given PIO program.
 *
 * More commonly this hardware mechanism is used to synchronise the execution
 * clocks of multiple state machines -- see pio_clkdiv_restart_sm_mask().
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 */
static inline void pio_sm_clkdiv_restart(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    hw_set_bits(&pio->ctrl, 1u << (PIO_CTRL_CLKDIV_RESTART_LSB + sm));
}

/*! \brief Restart multiple state machines' clock dividers from a phase of 0.
 *  \ingroup hardware_pio
 *
 * Each state machine's clock divider is a free-running piece of hardware,
 * that generates a pattern of clock enable pulses for the state machine,
 * based *only* on the configured integer/fractional divisor. The pattern of
 * running/halted cycles slows the state machine's execution to some
 * controlled rate.
 *
 * This function simultaneously clears the integer and fractional phase
 * accumulators of multiple state machines' clock dividers. If these state
 * machines all have the same integer and fractional divisors configured,
 * their clock dividers will run in precise deterministic lockstep from this
 * point.
 *
 * With their execution clocks synchronised in this way, it is then safe to
 * e.g. have multiple state machines performing a 'wait irq' on the same flag,
 * and all clear it on the same cycle.
 *
 * Also note that this function can be called whilst state machines are
 * running (e.g. if you have just changed the clock divisors of some state
 * machines and wish to resynchronise them), and that disabling a state
 * machine does not halt its clock divider: that is, if multiple state
 * machines have their clocks synchronised, you can safely disable and
 * re-enable one of the state machines without losing synchronisation.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param mask bit mask of state machine indexes to modify the enabled state of
 */
static inline void pio_clkdiv_restart_sm_mask(PIO pio, uint32_t mask) {
    check_pio_param(pio);
    check_sm_mask(mask);
    hw_set_bits(&pio->ctrl, (mask << PIO_CTRL_CLKDIV_RESTART_LSB) & PIO_CTRL_CLKDIV_RESTART_BITS);
}

#if PICO_PIO_VERSION > 0
/*! \brief Restart multiple state machines' clock dividers on multiple PIOs from a phase of 0.
 *  \ingroup hardware_pio
 *
 * Each state machine's clock divider is a free-running piece of hardware,
 * that generates a pattern of clock enable pulses for the state machine,
 * based *only* on the configured integer/fractional divisor. The pattern of
 * running/halted cycles slows the state machine's execution to some
 * controlled rate.
 *
 * This function simultaneously clears the integer and fractional phase
 * accumulators of multiple state machines' clock dividers. If these state
 * machines all have the same integer and fractional divisors configured,
 * their clock dividers will run in precise deterministic lockstep from this
 * point.
 *
 * With their execution clocks synchronised in this way, it is then safe to
 * e.g. have multiple state machines performing a 'wait irq' on the same flag,
 * and all clear it on the same cycle.
 *
 * Also note that this function can be called whilst state machines are
 * running (e.g. if you have just changed the clock divisors of some state
 * machines and wish to resynchronise them), and that disabling a state
 * machine does not halt its clock divider: that is, if multiple state
 * machines have their clocks synchronised, you can safely disable and
 * re-enable one of the state machines without losing synchronisation.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param mask_prev bit mask of state machine indexes to modify the enabled state of, in the next-lower numbered PIO instance
 * \param mask bit mask of state machine indexes to modify the enabled state of, in this PIO instance
 * \param mask_next bit mask of state machine indexes to modify the enabled state of, in the next-higher numbered PIO instance
 */
static inline void pio_clkdiv_restart_sm_multi_mask(PIO pio, uint32_t mask_prev, uint32_t mask, uint32_t mask_next) {
    check_pio_param(pio);
    check_sm_mask(mask);
    hw_set_bits(&pio->ctrl, ((mask << PIO_CTRL_CLKDIV_RESTART_LSB) & PIO_CTRL_CLKDIV_RESTART_BITS) |
                            PIO_CTRL_NEXTPREV_CLKDIV_RESTART_BITS |
                            ((mask_prev << PIO_CTRL_PREV_PIO_MASK_LSB) & PIO_CTRL_PREV_PIO_MASK_BITS) |
                            ((mask_next << PIO_CTRL_NEXT_PIO_MASK_LSB) & PIO_CTRL_NEXT_PIO_MASK_BITS));
}
#endif

/*! \brief Enable multiple PIO state machines synchronizing their clock dividers
 *  \ingroup hardware_pio
 *
 * This is equivalent to calling both pio_set_sm_mask_enabled() and
 * pio_clkdiv_restart_sm_mask() on the *same* clock cycle. All state machines
 * specified by 'mask' are started simultaneously and, assuming they have the
 * same clock divisors, their divided clocks will stay precisely synchronised.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param mask bit mask of state machine indexes to modify the enabled state of
 */
static inline void pio_enable_sm_mask_in_sync(PIO pio, uint32_t mask) {
    check_pio_param(pio);
    check_sm_mask(mask);
    hw_set_bits(&pio->ctrl,
        ((mask << PIO_CTRL_CLKDIV_RESTART_LSB) & PIO_CTRL_CLKDIV_RESTART_BITS) |
        ((mask << PIO_CTRL_SM_ENABLE_LSB) & PIO_CTRL_SM_ENABLE_BITS));
}

#if PICO_PIO_VERSION > 0
/*! \brief Enable multiple PIO state machines on multiple PIOs synchronizing their clock dividers
 *  \ingroup hardware_pio
 *
 * This is equivalent to calling both pio_set_sm_multi_mask_enabled() and
 * pio_clkdiv_restart_sm_multi_mask() on the *same* clock cycle. All state machines
 * specified by 'mask' are started simultaneously and, assuming they have the
 * same clock divisors, their divided clocks will stay precisely synchronised.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param mask_prev bit mask of state machine indexes to modify the enabled state of, in the next-lower numbered PIO instance
 * \param mask bit mask of state machine indexes to modify the enabled state of, in this PIO instance
 * \param mask_next bit mask of state machine indexes to modify the enabled state of, in the next-higher numbered PIO instance
 */
static inline void pio_enable_sm_multi_mask_in_sync(PIO pio, uint32_t mask_prev, uint32_t mask, uint32_t mask_next) {
    check_pio_param(pio);
    check_sm_mask(mask);
    check_pio_param(pio);
    check_sm_mask(mask);
    hw_set_bits(&pio->ctrl, ((mask << PIO_CTRL_CLKDIV_RESTART_LSB) & PIO_CTRL_CLKDIV_RESTART_BITS) |
                            ((mask << PIO_CTRL_SM_ENABLE_LSB) & PIO_CTRL_SM_ENABLE_BITS) |
                            PIO_CTRL_NEXTPREV_CLKDIV_RESTART_BITS | PIO_CTRL_NEXTPREV_SM_ENABLE_BITS |
                            ((mask_prev << PIO_CTRL_PREV_PIO_MASK_LSB) & PIO_CTRL_PREV_PIO_MASK_BITS) |
                            ((mask_next << PIO_CTRL_NEXT_PIO_MASK_LSB) & PIO_CTRL_NEXT_PIO_MASK_BITS));
}
#endif

/*! \brief PIO interrupt source numbers for pio related IRQs
 * \ingroup hardware_pio
 */
typedef enum pio_interrupt_source {
    pis_interrupt0 = PIO_INTR_SM0_LSB,                        ///< PIO interrupt 0 is raised
    pis_interrupt1 = PIO_INTR_SM1_LSB,                        ///< PIO interrupt 1 is raised
    pis_interrupt2 = PIO_INTR_SM2_LSB,                        ///< PIO interrupt 2 is raised
    pis_interrupt3 = PIO_INTR_SM3_LSB,                        ///< PIO interrupt 3 is raised
#if PICO_PIO_VERSION > 0
    pis_interrupt4 = PIO_INTR_SM4_LSB,                        ///< PIO interrupt 4 is raised
    pis_interrupt5 = PIO_INTR_SM5_LSB,                        ///< PIO interrupt 5 is raised
    pis_interrupt6 = PIO_INTR_SM6_LSB,                        ///< PIO interrupt 6 is raised
    pis_interrupt7 = PIO_INTR_SM7_LSB,                        ///< PIO interrupt 7 is raised
#endif
    pis_sm0_tx_fifo_not_full = PIO_INTR_SM0_TXNFULL_LSB,      ///< State machine 0 TX FIFO is not full
    pis_sm1_tx_fifo_not_full = PIO_INTR_SM1_TXNFULL_LSB,      ///< State machine 1 TX FIFO is not full
    pis_sm2_tx_fifo_not_full = PIO_INTR_SM2_TXNFULL_LSB,      ///< State machine 2 TX FIFO is not full
    pis_sm3_tx_fifo_not_full = PIO_INTR_SM3_TXNFULL_LSB,      ///< State machine 3 TX FIFO is not full
    pis_sm0_rx_fifo_not_empty = PIO_INTR_SM0_RXNEMPTY_LSB,    ///< State machine 0 RX FIFO is not empty
    pis_sm1_rx_fifo_not_empty = PIO_INTR_SM1_RXNEMPTY_LSB,    ///< State machine 1 RX FIFO is not empty
    pis_sm2_rx_fifo_not_empty = PIO_INTR_SM2_RXNEMPTY_LSB,    ///< State machine 2 RX FIFO is not empty
    pis_sm3_rx_fifo_not_empty = PIO_INTR_SM3_RXNEMPTY_LSB,    ///< State machine 3 RX FIFO is not empty
} pio_interrupt_source_t;

/*! \brief  Enable/Disable a single source on a PIO's IRQ 0
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param source the source number (see \ref pio_interrupt_source)
 * \param enabled true to enable IRQ 0 for the source, false to disable.
 */
static inline void pio_set_irq0_source_enabled(PIO pio, pio_interrupt_source_t source, bool enabled) {
    check_pio_param(pio);
    invalid_params_if(HARDWARE_PIO, source >= 32u || (1u << source) > PIO_INTR_BITS);
    if (enabled)
        hw_set_bits(&pio->inte0, 1u << source);
    else
        hw_clear_bits(&pio->inte0, 1u << source);
}

/*! \brief  Enable/Disable a single source on a PIO's IRQ 1
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param source the source number (see \ref pio_interrupt_source)
 * \param enabled true to enable IRQ 0 for the source, false to disable.
 */
static inline void pio_set_irq1_source_enabled(PIO pio, pio_interrupt_source_t source, bool enabled) {
    check_pio_param(pio);
    invalid_params_if(HARDWARE_PIO, source >= 32 || (1u << source) > PIO_INTR_BITS);
    if (enabled)
        hw_set_bits(&pio->inte1, 1u << source);
    else
        hw_clear_bits(&pio->inte1, 1u << source);
}

/*! \brief  Enable/Disable multiple sources on a PIO's IRQ 0
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param source_mask Mask of bits, one for each source number (see \ref pio_interrupt_source) to affect
 * \param enabled true to enable all the sources specified in the mask on IRQ 0, false to disable all the sources specified in the mask on IRQ 0
 */
static inline void pio_set_irq0_source_mask_enabled(PIO pio, uint32_t source_mask, bool enabled) {
    check_pio_param(pio);
    invalid_params_if(HARDWARE_PIO, source_mask > PIO_INTR_BITS);
    if (enabled) {
        hw_set_bits(&pio->inte0, source_mask);
    } else {
        hw_clear_bits(&pio->inte0, source_mask);
    }
}

/*! \brief  Enable/Disable multiple sources on a PIO's IRQ 1
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param source_mask Mask of bits, one for each source number (see \ref pio_interrupt_source) to affect
 * \param enabled true to enable all the sources specified in the mask on IRQ 1, false to disable all the source specified in the mask on IRQ 1
 */
static inline void pio_set_irq1_source_mask_enabled(PIO pio, uint32_t source_mask, bool enabled) {
    check_pio_param(pio);
    invalid_params_if(HARDWARE_PIO, source_mask > PIO_INTR_BITS);
    if (enabled) {
        hw_set_bits(&pio->inte1, source_mask);
    } else {
        hw_clear_bits(&pio->inte1, source_mask);
    }
}

/*! \brief  Enable/Disable a single source on a PIO's specified (0/1) IRQ index
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param irq_index the IRQ index; either 0 or 1
 * \param source the source number (see \ref pio_interrupt_source)
 * \param enabled true to enable the source on the specified IRQ, false to disable.
 */
static inline void pio_set_irqn_source_enabled(PIO pio, uint irq_index, pio_interrupt_source_t source, bool enabled) {
    invalid_params_if(HARDWARE_PIO, irq_index > NUM_PIO_IRQS);
    invalid_params_if(HARDWARE_PIO, source >= 32 || (1u << source) > PIO_INTR_BITS);
    if (enabled)
        hw_set_bits(&pio->irq_ctrl[irq_index].inte, 1u << source);
    else
        hw_clear_bits(&pio->irq_ctrl[irq_index].inte, 1u << source);
}

/*! \brief  Enable/Disable multiple sources on a PIO's specified (0/1) IRQ index
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param irq_index the IRQ index; either 0 or 1
 * \param source_mask Mask of bits, one for each source number (see \ref pio_interrupt_source) to affect
 * \param enabled true to enable all the sources specified in the mask on the specified IRQ, false to disable all the sources specified in the mask on the specified IRQ
 */
static inline void pio_set_irqn_source_mask_enabled(PIO pio, uint irq_index, uint32_t source_mask, bool enabled) {
    invalid_params_if(HARDWARE_PIO, irq_index > NUM_PIO_IRQS);
    static_assert(NUM_PIO_IRQS == 2, "");
    invalid_params_if(HARDWARE_PIO, source_mask > PIO_INTR_BITS);
    if (enabled) {
        hw_set_bits(&pio->irq_ctrl[irq_index].inte, source_mask);
    } else {
        hw_clear_bits(&pio->irq_ctrl[irq_index].inte, source_mask);
    }
}

/*! \brief  Determine if a particular PIO interrupt is set
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param pio_interrupt_num the PIO interrupt number 0-7
 * \return true if corresponding PIO interrupt is currently set
 */
static inline bool pio_interrupt_get(PIO pio, uint pio_interrupt_num) {
    check_pio_param(pio);
    invalid_params_if(HARDWARE_PIO, pio_interrupt_num >= 8);
    return pio->irq & (1u << pio_interrupt_num);
}

/*! \brief  Clear a particular PIO interrupt
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param pio_interrupt_num the PIO interrupt number 0-7
 */
static inline void pio_interrupt_clear(PIO pio, uint pio_interrupt_num) {
    check_pio_param(pio);
    invalid_params_if(HARDWARE_PIO, pio_interrupt_num >= 8);
    pio->irq = (1u << pio_interrupt_num);
}

/*! \brief Return the current program counter for a state machine
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return the program counter
 */
static inline uint8_t pio_sm_get_pc(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    return (uint8_t) pio->sm[sm].addr;
}

/*! \brief Immediately execute an instruction on a state machine
 *  \ingroup hardware_pio
 *
 * This instruction is executed instead of the next instruction in the normal control flow on the state machine.
 * Subsequent calls to this method replace the previous executed
 * instruction if it is still running. \see pio_sm_is_exec_stalled() to see if an executed instruction
 * is still running (i.e. it is stalled on some condition)
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param instr the encoded PIO instruction
 */
inline static void pio_sm_exec(PIO pio, uint sm, uint instr) {
    check_pio_param(pio);
    check_sm_param(sm);
    pio->sm[sm].instr = instr;
}

/*! \brief Determine if an instruction set by pio_sm_exec() is stalled executing
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return true if the executed instruction is still running (stalled)
 */
static inline bool pio_sm_is_exec_stalled(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    return pio->sm[sm].execctrl & PIO_SM0_EXECCTRL_EXEC_STALLED_BITS;
}

/*! \brief Immediately execute an instruction on a state machine and wait for it to complete
 *  \ingroup hardware_pio
 *
 * This instruction is executed instead of the next instruction in the normal control flow on the state machine.
 * Subsequent calls to this method replace the previous executed
 * instruction if it is still running. \see pio_sm_is_exec_stalled() to see if an executed instruction
 * is still running (i.e. it is stalled on some condition)
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param instr the encoded PIO instruction
 */
static inline void pio_sm_exec_wait_blocking(PIO pio, uint sm, uint instr) {
    check_pio_param(pio);
    check_sm_param(sm);
    pio_sm_exec(pio, sm, instr);
    while (pio_sm_is_exec_stalled(pio, sm)) tight_loop_contents();
}

/*! \brief Set the current wrap configuration for a state machine
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param wrap_target the instruction memory address to wrap to
 * \param wrap        the instruction memory address after which to set the program counter to wrap_target
 *                    if the instruction does not itself update the program_counter
 */
static inline void pio_sm_set_wrap(PIO pio, uint sm, uint wrap_target, uint wrap) {
    check_pio_param(pio);
    check_sm_param(sm);
    valid_params_if(HARDWARE_PIO, wrap < PIO_INSTRUCTION_COUNT);
    valid_params_if(HARDWARE_PIO, wrap_target < PIO_INSTRUCTION_COUNT);
    pio->sm[sm].execctrl =
            (pio->sm[sm].execctrl & ~(PIO_SM0_EXECCTRL_WRAP_TOP_BITS | PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS)) |
            (wrap_target << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB) |
            (wrap << PIO_SM0_EXECCTRL_WRAP_TOP_LSB);
}

/*! \brief Set the current 'out' pins for a state machine
 *  \ingroup hardware_pio
 *
 * 'out' pins can overlap with the 'in', 'set' and 'sideset' pins
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param out_base 0-31 First pin to set as output
 * \param out_count 0-32 Number of pins to set.
 */
static inline void pio_sm_set_out_pins(PIO pio, uint sm, uint out_base, uint out_count) {
    check_pio_param(pio);
    check_sm_param(sm);
#if PICO_PIO_USE_GPIO_BASE
    out_base -= pio_get_gpio_base(pio);
#endif
    valid_params_if(HARDWARE_PIO, out_base < 32);
    valid_params_if(HARDWARE_PIO, out_count <= 32);
    pio->sm[sm].pinctrl = (pio->sm[sm].pinctrl & ~(PIO_SM0_PINCTRL_OUT_BASE_BITS | PIO_SM0_PINCTRL_OUT_COUNT_BITS)) |
                 (out_base << PIO_SM0_PINCTRL_OUT_BASE_LSB) |
                 (out_count << PIO_SM0_PINCTRL_OUT_COUNT_LSB);
}


/*! \brief Set the current 'set' pins for a state machine
 *  \ingroup hardware_pio
 *
 * 'set' pins can overlap with the 'in', 'out' and 'sideset' pins
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param set_base 0-31 First pin to set as
 * \param set_count 0-5 Number of pins to set.
 */
static inline void pio_sm_set_set_pins(PIO pio, uint sm, uint set_base, uint set_count) {
    check_pio_param(pio);
    check_sm_param(sm);
#if PICO_PIO_USE_GPIO_BASE
    set_base -= pio_get_gpio_base(pio);
#endif
    valid_params_if(HARDWARE_PIO, set_base < 32);
    valid_params_if(HARDWARE_PIO, set_count <= 5);
    pio->sm[sm].pinctrl = (pio->sm[sm].pinctrl & ~(PIO_SM0_PINCTRL_SET_BASE_BITS | PIO_SM0_PINCTRL_SET_COUNT_BITS)) |
                 (set_base << PIO_SM0_PINCTRL_SET_BASE_LSB) |
                 (set_count << PIO_SM0_PINCTRL_SET_COUNT_LSB);
}

/*! \brief Set the current 'in' pins for a state machine
 *  \ingroup hardware_pio
 *
 * 'in' pins can overlap with the 'out', 'set' and 'sideset' pins
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param in_base 0-31 First pin to use as input
 */
static inline void pio_sm_set_in_pins(PIO pio, uint sm, uint in_base) {
    check_pio_param(pio);
    check_sm_param(sm);
#if PICO_PIO_USE_GPIO_BASE
    in_base -= pio_get_gpio_base(pio);
#endif
    valid_params_if(HARDWARE_PIO, in_base < 32);
    pio->sm[sm].pinctrl = (pio->sm[sm].pinctrl & ~PIO_SM0_PINCTRL_IN_BASE_BITS) |
                 (in_base << PIO_SM0_PINCTRL_IN_BASE_LSB);
}

/*! \brief Set the current 'sideset' pins for a state machine
 *  \ingroup hardware_pio
 *
 * 'sideset' pins can overlap with the 'in', 'out' and 'set' pins
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param sideset_base 0-31 base pin for 'side set'
 */
static inline void pio_sm_set_sideset_pins(PIO pio, uint sm, uint sideset_base) {
    check_pio_param(pio);
    check_sm_param(sm);
#if PICO_PIO_USE_GPIO_BASE
    sideset_base -= pio_get_gpio_base(pio);
#endif
    valid_params_if(HARDWARE_PIO, sideset_base < 32);
    pio->sm[sm].pinctrl = (pio->sm[sm].pinctrl & ~PIO_SM0_PINCTRL_SIDESET_BASE_BITS) |
                 (sideset_base << PIO_SM0_PINCTRL_SIDESET_BASE_LSB);
}

/*! \brief Set the 'jmp' pin for a state machine
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param pin The raw GPIO pin number to use as the source for a `jmp pin` instruction
 */

static inline void pio_sm_set_jmp_pin(PIO pio, uint sm, uint pin) {
    check_pio_param(pio);
    check_sm_param(sm);
#if PICO_PIO_USE_GPIO_BASE
    pin -= pio_get_gpio_base(pio);
#endif
    valid_params_if(HARDWARE_PIO, pin < 32);
    pio->sm[sm].execctrl =
        (pio->sm[sm].execctrl & ~PIO_SM0_EXECCTRL_JMP_PIN_BITS)
        | (pin << PIO_SM0_EXECCTRL_JMP_PIN_LSB);
}

/*! \brief Write a word of data to a state machine's TX FIFO
 *  \ingroup hardware_pio
 *
 * This is a raw FIFO access that does not check for fullness. If the FIFO is
 * full, the FIFO contents and state are not affected by the write attempt.
 * Hardware sets the TXOVER sticky flag for this FIFO in FDEBUG, to indicate
 * that the system attempted to write to a full FIFO.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param data the 32 bit data value
 *
 * \sa pio_sm_put_blocking()
 */
static inline void pio_sm_put(PIO pio, uint sm, uint32_t data) {
    check_pio_param(pio);
    check_sm_param(sm);
    pio->txf[sm] = data;
}

/*! \brief Read a word of data from a state machine's RX FIFO
 *  \ingroup hardware_pio
 *
 * This is a raw FIFO access that does not check for emptiness. If the FIFO is
 * empty, the hardware ignores the attempt to read from the FIFO (the FIFO
 * remains in an empty state following the read) and the sticky RXUNDER flag
 * for this FIFO is set in FDEBUG to indicate that the system tried to read
 * from this FIFO when empty. The data returned by this function is undefined
 * when the FIFO is empty.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 *
 * \sa pio_sm_get_blocking()
 */
static inline uint32_t pio_sm_get(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    return pio->rxf[sm];
}

/*! \brief Determine if a state machine's RX FIFO is full
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return true if the RX FIFO is full
 */
static inline bool pio_sm_is_rx_fifo_full(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    return (pio->fstat & (1u << (PIO_FSTAT_RXFULL_LSB + sm))) != 0;
}

/*! \brief Determine if a state machine's RX FIFO is empty
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return true if the RX FIFO is empty
 */
static inline bool pio_sm_is_rx_fifo_empty(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    return (pio->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm))) != 0;
}

/*! \brief Return the number of elements currently in a state machine's RX FIFO
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return the number of elements in the RX FIFO
 */
static inline uint pio_sm_get_rx_fifo_level(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    uint bitoffs = PIO_FLEVEL_RX0_LSB + sm * (PIO_FLEVEL_RX1_LSB - PIO_FLEVEL_RX0_LSB);
    const uint32_t mask = PIO_FLEVEL_RX0_BITS >> PIO_FLEVEL_RX0_LSB;
    return (pio->flevel >> bitoffs) & mask;
}

/*! \brief Determine if a state machine's TX FIFO is full
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return true if the TX FIFO is full
 */
static inline bool pio_sm_is_tx_fifo_full(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    return (pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) != 0;
}

/*! \brief Determine if a state machine's TX FIFO is empty
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return true if the TX FIFO is empty
 */
static inline bool pio_sm_is_tx_fifo_empty(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    return (pio->fstat & (1u << (PIO_FSTAT_TXEMPTY_LSB + sm))) != 0;
}

/*! \brief Return the number of elements currently in a state machine's TX FIFO
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return the number of elements in the TX FIFO
 */
static inline uint pio_sm_get_tx_fifo_level(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    unsigned int bitoffs = PIO_FLEVEL_TX0_LSB + sm * (PIO_FLEVEL_TX1_LSB - PIO_FLEVEL_TX0_LSB);
    const uint32_t mask = PIO_FLEVEL_TX0_BITS >> PIO_FLEVEL_TX0_LSB;
    return (pio->flevel >> bitoffs) & mask;
}

/*! \brief Write a word of data to a state machine's TX FIFO, blocking if the FIFO is full
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param data the 32 bit data value
 */
static inline void pio_sm_put_blocking(PIO pio, uint sm, uint32_t data) {
    check_pio_param(pio);
    check_sm_param(sm);
    while (pio_sm_is_tx_fifo_full(pio, sm)) tight_loop_contents();
    pio_sm_put(pio, sm, data);
}

/*! \brief Read a word of data from a state machine's RX FIFO, blocking if the FIFO is empty
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 */
static inline uint32_t pio_sm_get_blocking(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    while (pio_sm_is_rx_fifo_empty(pio, sm)) tight_loop_contents();
    return pio_sm_get(pio, sm);
}

/*! \brief Empty out a state machine's TX FIFO
 *  \ingroup hardware_pio
 *
 * This method executes `pull` instructions on the state machine until the TX
 * FIFO is empty. This disturbs the contents of the OSR, so see also
 * pio_sm_clear_fifos() which clears both FIFOs but leaves the state machine's
 * internal state undisturbed.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 *
 * \sa pio_sm_clear_fifos()
 */
void pio_sm_drain_tx_fifo(PIO pio, uint sm);

/*! \brief set the current clock divider for a state machine using a 16:8 fraction
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param div_int the integer part of the clock divider
 * \param div_frac the fractional part of the clock divider in 1/256s
 */
static inline void pio_sm_set_clkdiv_int_frac(PIO pio, uint sm, uint16_t div_int, uint8_t div_frac) {
    check_pio_param(pio);
    check_sm_param(sm);
    invalid_params_if(HARDWARE_PIO, div_int == 0 && div_frac != 0);
    pio->sm[sm].clkdiv =
            (((uint)div_frac) << PIO_SM0_CLKDIV_FRAC_LSB) |
            (((uint)div_int) << PIO_SM0_CLKDIV_INT_LSB);
}

/*! \brief set the current clock divider for a state machine
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param div the floating point clock divider
 */
static inline void pio_sm_set_clkdiv(PIO pio, uint sm, float div) {
    check_pio_param(pio);
    check_sm_param(sm);
    uint16_t div_int;
    uint8_t div_frac;
    pio_calculate_clkdiv_from_float(div, &div_int, &div_frac);
    pio_sm_set_clkdiv_int_frac(pio, sm, div_int, div_frac);
}

/*! \brief Clear a state machine's TX and RX FIFOs
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 */
static inline void pio_sm_clear_fifos(PIO pio, uint sm) {
    // changing the FIFO join state clears the fifo
    check_pio_param(pio);
    check_sm_param(sm);
    hw_xor_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_xor_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
}

/*! \brief Use a state machine to set a value on all pins for the PIO instance
 *  \ingroup hardware_pio
 *
 * This method repeatedly reconfigures the target state machine's pin configuration and executes 'set' instructions to set values on all 32 pins,
 * before restoring the state machine's pin configuration to what it was.
 *
 * This method is provided as a convenience to set initial pin states, and should not be used against a state machine that is enabled.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3) to use
 * \param pin_values the pin values to set
 */
void pio_sm_set_pins(PIO pio, uint sm, uint32_t pin_values);

/*! \brief Use a state machine to set a value on multiple pins for the PIO instance
 *  \ingroup hardware_pio
 *
 * This method repeatedly reconfigures the target state machine's pin configuration and executes 'set' instructions to set values on up to 32 pins,
 * before restoring the state machine's pin configuration to what it was.
 *
 * This method is provided as a convenience to set initial pin states, and should not be used against a state machine that is enabled.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3) to use
 * \param pin_values the pin values to set (if the corresponding bit in pin_mask is set)
 * \param pin_mask a bit for each pin to indicate whether the corresponding pin_value for that pin should be applied.
 */
void pio_sm_set_pins_with_mask(PIO pio, uint sm, uint32_t pin_values, uint32_t pin_mask);

/*! \brief Use a state machine to set the pin directions for multiple pins for the PIO instance
 *  \ingroup hardware_pio
 *
 * This method repeatedly reconfigures the target state machine's pin configuration and executes 'set' instructions to set pin directions on up to 32 pins,
 * before restoring the state machine's pin configuration to what it was.
 *
 * This method is provided as a convenience to set initial pin directions, and should not be used against a state machine that is enabled.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3) to use
 * \param pin_dirs the pin directions to set - 1 = out, 0 = in (if the corresponding bit in pin_mask is set)
 * \param pin_mask a bit for each pin to indicate whether the corresponding pin_value for that pin should be applied.
 */
void pio_sm_set_pindirs_with_mask(PIO pio, uint sm, uint32_t pin_dirs, uint32_t pin_mask);

/*! \brief Use a state machine to set the same pin direction for multiple consecutive pins for the PIO instance
 *  \ingroup hardware_pio
 *
 * This method repeatedly reconfigures the target state machine's pin configuration and executes 'set' instructions to set the pin direction on consecutive pins,
 * before restoring the state machine's pin configuration to what it was.
 *
 * This method is provided as a convenience to set initial pin directions, and should not be used against a state machine that is enabled.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3) to use
 * \param pins_base the first pin to set a direction for
 * \param pin_count the count of consecutive pins to set the direction for
 * \param is_out the direction to set; true = out, false = in
 * \return PICO_OK (0) on success, error code otherwise
 */
int pio_sm_set_consecutive_pindirs(PIO pio, uint sm, uint pins_base, uint pin_count, bool is_out);

/*! \brief Mark a state machine as used
 *  \ingroup hardware_pio
 *
 * Method for cooperative claiming of hardware. Will cause a panic if the state machine
 * is already claimed. Use of this method by libraries detects accidental
 * configurations that would fail in unpredictable ways.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 */
void pio_sm_claim(PIO pio, uint sm);

/*! \brief Mark multiple state machines as used
 *  \ingroup hardware_pio
 *
 * Method for cooperative claiming of hardware. Will cause a panic if any of the state machines
 * are already claimed. Use of this method by libraries detects accidental
 * configurations that would fail in unpredictable ways.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm_mask Mask of state machine indexes
 */
void pio_claim_sm_mask(PIO pio, uint sm_mask);

/*! \brief Mark a state machine as no longer used
 *  \ingroup hardware_pio
 *
 * Method for cooperative claiming of hardware.
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 */
void pio_sm_unclaim(PIO pio, uint sm);

/*! \brief Claim a free state machine on a PIO instance
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param required if true the function will panic if none are available
 * \return the state machine index or negative if required was false, and none were free (for
 * backwards compatibility with prior SDK the error value is -1 i.e. PICO_ERROR_GENERIC)
 */
int pio_claim_unused_sm(PIO pio, bool required);

/*! \brief Determine if a PIO state machine is claimed
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; e.g. \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return true if claimed, false otherwise
 * \see pio_sm_claim
 * \see pio_claim_sm_mask
 */
bool pio_sm_is_claimed(PIO pio, uint sm);

/*! \brief Finds a PIO and statemachine and adds a program into PIO memory
 *  \ingroup hardware_pio
 *
 * \param program PIO program to add
 * \param pio Returns the PIO hardware instance or NULL if no PIO is available
 * \param sm Returns the index of the PIO state machine that was claimed
 * \param offset Returns the instruction memory offset of the start of the program
 * \return true on success, false otherwise
 * \see pio_remove_program_unclaim_sm
 */
bool pio_claim_free_sm_and_add_program(const pio_program_t *program, PIO *pio, uint *sm, uint *offset);

/*! \brief Finds a PIO and statemachine and adds a program into PIO memory
 *  \ingroup hardware_pio
 *
 * This variation of \ref pio_claim_free_sm_and_add_program is useful on RP2350 QFN80 where the "GPIO Base"
 * must be set per PIO instance to either address the 32 GPIOs (0->31) or the 32 GPIOS (16-47). No single
 * PIO instance can interact with both pins 0->15 or 32->47 at the same time.
 *
 * This method takes additional information about the GPIO pins needed (via gpi_base and gpio_count),
 * and optionally will set the GPIO base (\see pio_set_gpio_base) of an unused PIO instance if necessary
 *
 * \param program PIO program to add
 * \param pio Returns the PIO hardware instance or NULL if no PIO is available
 * \param sm Returns the index of the PIO state machine that was claimed
 * \param offset Returns the instruction memory offset of the start of the program
 * \param gpio_base the lowest GPIO number required
 * \param gpio_count the count of GPIOs required
 * \param set_gpio_base if there is no free SM on a PIO instance with the right GPIO base, and there IS an unused PIO
 *                      instance, then that PIO will be reconfigured so that this method can succeed
 *
 * \return true on success, false otherwise
 * \see pio_remove_program_unclaim_sm
 */
bool pio_claim_free_sm_and_add_program_for_gpio_range(const pio_program_t *program, PIO *pio, uint *sm, uint *offset, uint gpio_base, uint gpio_count, bool set_gpio_base);

/*! \brief Removes a program from PIO memory and unclaims the state machine
 *  \ingroup hardware_pio
 *
 * \param program PIO program to remove from memory
 * \param pio PIO hardware instance being used
 * \param sm PIO state machine that was claimed
 * \param offset offset of the program in PIO memory
 * \see pio_claim_free_sm_and_add_program
 */
void pio_remove_program_and_unclaim_sm(const pio_program_t *program, PIO pio, uint sm, uint offset);

/*! \brief Return an IRQ for a PIO hardware instance
 *  \ingroup hardware_pio
 *
 * \param pio PIO hardware instance
 * \param irqn 0 for PIOx_IRQ_0 or 1 for PIOx_IRQ_1 etc where x is the PIO number
 * \return The IRQ number to use for the PIO
 */
static inline int pio_get_irq_num(PIO pio, uint irqn) {
    check_pio_param(pio);
    valid_params_if(HARDWARE_PIO, irqn < NUM_PIO_IRQS);
    return PIO_IRQ_NUM(pio, irqn);
}

/*! \brief Return the interrupt source for a state machines TX FIFO not full interrupt
 *  \ingroup hardware_pio
 *
 * \param sm State machine index (0..3)
 * \return The interrupt source number for use in \ref pio_set_irqn_source_enabled or similar functions
 */
static inline pio_interrupt_source_t pio_get_tx_fifo_not_full_interrupt_source(uint sm) {
    check_sm_param(sm);
    return ((pio_interrupt_source_t)(pis_sm0_tx_fifo_not_full + sm));
}

/*! \brief Return the interrupt source for a state machines RX FIFO not empty interrupt
 *  \ingroup hardware_pio
 *
 * \param sm State machine index (0..3)
 * \return The interrupt source number for use in \ref pio_set_irqn_source_enabled or similar functions
 */
static inline pio_interrupt_source_t pio_get_rx_fifo_not_empty_interrupt_source(uint sm) {
    check_sm_param(sm);
    return ((pio_interrupt_source_t)(pis_sm0_rx_fifo_not_empty + sm));
}

#ifdef __cplusplus
}
#endif

#endif // _PIO_H_
