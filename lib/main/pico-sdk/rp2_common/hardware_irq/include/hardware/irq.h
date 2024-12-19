/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_IRQ_H
#define _HARDWARE_IRQ_H

// These two config items are also used by assembler, so keeping separate
// PICO_CONFIG: PICO_MAX_SHARED_IRQ_HANDLERS, Maximum number of shared IRQ handlers, default=4, advanced=true, group=hardware_irq
#ifndef PICO_MAX_SHARED_IRQ_HANDLERS
#define PICO_MAX_SHARED_IRQ_HANDLERS 4
#endif

// PICO_CONFIG: PICO_DISABLE_SHARED_IRQ_HANDLERS, Disable shared IRQ handlers, type=bool, default=0, group=hardware_irq
#ifndef PICO_DISABLE_SHARED_IRQ_HANDLERS
#define PICO_DISABLE_SHARED_IRQ_HANDLERS 0
#endif

// PICO_CONFIG: PICO_VTABLE_PER_CORE, User is using separate vector tables per core, type=bool, default=0, group=hardware_irq
#ifndef PICO_VTABLE_PER_CORE
#define PICO_VTABLE_PER_CORE 0
#endif

#ifndef __ASSEMBLER__

#include "pico.h"
#include "hardware/address_mapped.h"
#include "hardware/regs/intctrl.h"

#include "pico/platform/cpu_regs.h"

/** \file irq.h
 *  \defgroup hardware_irq hardware_irq
 *
 * \brief Hardware interrupt handling API
 *
 * The RP2040 uses the standard ARM nested vectored interrupt controller (NVIC).
 *
 * Interrupts are identified by a number from 0 to 31.
 *
 * On the RP2040, only the lower 26 IRQ signals are connected on the NVIC; IRQs 26 to 31 are tied to zero (never firing).
 *
 * There is one NVIC per core, and each core's NVIC has the same hardware interrupt lines routed to it, with the exception of the IO interrupts
 * where there is one IO interrupt per bank, per core. These are completely independent, so, for example, processor 0 can be
 * interrupted by GPIO 0 in bank 0, and processor 1 by GPIO 1 in the same bank.
 *
 * \note That all IRQ APIs affect the executing core only (i.e. the core calling the function).
 *
 * \note You should not enable the same (shared) IRQ number on both cores, as this will lead to race conditions
 * or starvation of one of the cores. Additionally, don't forget that disabling interrupts on one core does not disable interrupts
 * on the other core.
 *
 * There are three different ways to set handlers for an IRQ:
 *  - Calling irq_add_shared_handler() at runtime to add a handler for a multiplexed interrupt (e.g. GPIO bank) on the current core. Each handler, should check and clear the relevant hardware interrupt source
 *  - Calling irq_set_exclusive_handler() at runtime to install a single handler for the interrupt on the current core
 *  - Defining the interrupt handler explicitly in your application (e.g. by defining void `isr_dma_0` will make that function the handler for the DMA_IRQ_0 on core 0, and
 *    you will not be able to change it using the above APIs at runtime). Using this method can cause link conflicts at runtime, and offers no runtime performance benefit (i.e, it should not generally be used).
 *
 * \note If an IRQ is enabled and fires with no handler installed, a breakpoint will be hit and the IRQ number will
 * be in register r0.
 *
 * \section interrupt_nums Interrupt Numbers
 *
 * A set of defines is available (intctrl.h) with these names to avoid using the numbers directly.
 *
 * \if rp2040_specific
 * On RP2040 the interrupt numbers are as follows:
 *
 * IRQ | Interrupt Source
 * ----|-----------------
 *  0 | TIMER_IRQ_0
 *  1 | TIMER_IRQ_1
 *  2 | TIMER_IRQ_2
 *  3 | TIMER_IRQ_3
 *  4 | PWM_IRQ_WRAP
 *  5 | USBCTRL_IRQ
 *  6 | XIP_IRQ
 *  7 | PIO0_IRQ_0
 *  8 | PIO0_IRQ_1
 *  9 | PIO1_IRQ_0
 * 10 | PIO1_IRQ_1
 * 11 | DMA_IRQ_0
 * 12 | DMA_IRQ_1
 * 13 | IO_IRQ_BANK0
 * 14 | IO_IRQ_QSPI
 * 15 | SIO_IRQ_PROC0
 * 16 | SIO_IRQ_PROC1
 * 17 | CLOCKS_IRQ
 * 18 | SPI0_IRQ
 * 19 | SPI1_IRQ
 * 20 | UART0_IRQ
 * 21 | UART1_IRQ
 * 22 | ADC0_IRQ_FIFO
 * 23 | I2C0_IRQ
 * 24 | I2C1_IRQ
 * 25 | RTC_IRQ
 * \endif
 *
 * \if rp2350_specific
 * On RP2350 the interrupt numbers are as follows:
 *
 * IRQ | Interrupt Source
 * ----|-----------------
 * 0 | TIMER0_IRQ_0
 * 1 | TIMER0_IRQ_1
 * 2 | TIMER0_IRQ_2
 * 3 | TIMER0_IRQ_3
 * 4 | TIMER1_IRQ_0
 * 5 | TIMER1_IRQ_1
 * 6 | TIMER1_IRQ_2
 * 7 | TIMER1_IRQ_3
 * 8 | PWM_IRQ_WRAP_0
 * 9 | PWM_IRQ_WRAP_1
 * 10 | DMA_IRQ_0
 * 11 | DMA_IRQ_1
 * 12 | DMA_IRQ_2
 * 13 | DMA_IRQ_3
 * 14 | USBCTRL_IRQ
 * 15 | PIO0_IRQ_0
 * 16 | PIO0_IRQ_1
 * 17 | PIO1_IRQ_0
 * 18 | PIO1_IRQ_1
 * 19 | PIO2_IRQ_0
 * 20 | PIO2_IRQ_1
 * 21 | IO_IRQ_BANK0
 * 22 | IO_IRQ_BANK0_NS
 * 23 | IO_IRQ_QSPI
 * 24 | IO_IRQ_QSPI_NS
 * 25 | SIO_IRQ_FIFO
 * 26 | SIO_IRQ_BELL
 * 27 | SIO_IRQ_FIFO_NS
 * 28 | SIO_IRQ_BELL_NS
 * 29 | SIO_IRQ_MTIMECMP
 * 30 | CLOCKS_IRQ
 * 31 | SPI0_IRQ
 * 32 | SPI1_IRQ
 * 33 | UART0_IRQ
 * 34 | UART1_IRQ
 * 35 | ADC_IRQ_FIFO
 * 36 | I2C0_IRQ
 * 37 | I2C1_IRQ
 * 38 | OTP_IRQ
 * 39 | TRNG_IRQ
 * 40 | PROC0_IRQ_CTI
 * 41 | PROC1_IRQ_CTI
 * 42 | PLL_SYS_IRQ
 * 43 | PLL_USB_IRQ
 * 44 | POWMAN_IRQ_POW
 * 45 | POWMAN_IRQ_TIMER
 * 46 | SPAREIRQ_IRQ_0
 * 47 | SPAREIRQ_IRQ_1
 * 48 | SPAREIRQ_IRQ_2
 * 49 | SPAREIRQ_IRQ_3
 * 50 | SPAREIRQ_IRQ_4
 * 51 | SPAREIRQ_IRQ_5
 * \endif
 */

// PICO_CONFIG: PICO_DEFAULT_IRQ_PRIORITY, Define the default IRQ priority, default=0x80, group=hardware_irq
#ifndef PICO_DEFAULT_IRQ_PRIORITY
#define PICO_DEFAULT_IRQ_PRIORITY 0x80
#endif

#define PICO_LOWEST_IRQ_PRIORITY 0xff
#define PICO_HIGHEST_IRQ_PRIORITY 0x00

// PICO_CONFIG: PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY, Set default shared IRQ order priority, default=0x80, group=hardware_irq
#ifndef PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY
#define PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY 0x80
#endif

#define PICO_SHARED_IRQ_HANDLER_HIGHEST_ORDER_PRIORITY 0xff
#define PICO_SHARED_IRQ_HANDLER_LOWEST_ORDER_PRIORITY 0x00

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_HARDWARE_IRQ, Enable/disable assertions in the hardware_irq module, type=bool, default=0, group=hardware_irq
#ifndef PARAM_ASSERTIONS_ENABLED_HARDWARE_IRQ
#ifdef PARAM_ASSERTIONS_ENABLED_IRQ // backwards compatibility with SDK < 2.0.0
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_IRQ PARAM_ASSERTIONS_ENABLED_IRQ
#else
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_IRQ 0
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Interrupt handler function type
 *  \ingroup hardware_irq
 *
 * All interrupts handlers should be of this type, and follow normal ARM EABI register saving conventions
 */
typedef void (*irq_handler_t)(void);

static inline void check_irq_param(__unused uint num) {
    invalid_params_if(HARDWARE_IRQ, num >= NUM_IRQS);
}

/*! \brief Set specified interrupt's priority
 *  \ingroup hardware_irq
 *
 * \param num Interrupt number \ref interrupt_nums
 * \param hardware_priority Priority to set.
 * Numerically-lower values indicate a higher priority. Hardware priorities
 * range from 0 (highest priority) to 255 (lowest priority). To make it easier to specify
 * higher or lower priorities than the default, all IRQ priorities are
 * initialized to PICO_DEFAULT_IRQ_PRIORITY by the SDK runtime at startup.
 * PICO_DEFAULT_IRQ_PRIORITY defaults to 0x80
 *
 * \if rp2040_specific
 * Only the top 2 bits are significant on ARM Cortex-M0+ on RP2040.
 * \endif
 *
 * \if rp2350_specific
 * Only the top 4 bits are significant on ARM Cortex-M33 or Hazard3 (RISC-V) on RP2350.
 * Note that this API uses the same (inverted) ordering as ARM on RISC-V
 * \endif
 */
void irq_set_priority(uint num, uint8_t hardware_priority);

/*! \brief Get specified interrupt's priority
 *  \ingroup hardware_irq
 *
 * Numerically-lower values indicate a higher priority. Hardware priorities
 * range from 0 (highest priority) to 255 (lowest priority). To make it easier to specify
 * higher or lower priorities than the default, all IRQ priorities are
 * initialized to PICO_DEFAULT_IRQ_PRIORITY by the SDK runtime at startup.
 * PICO_DEFAULT_IRQ_PRIORITY defaults to 0x80
 *
 * \if rp2040_specific
 * Only the top 2 bits are significant on ARM Cortex-M0+ on RP2040.
 * \endif
 *
 * \if rp2350_specific
 * Only the top 4 bits are significant on ARM Cortex-M33 or Hazard3 (RISC-V) on RP2350.
 * Note that this API uses the same (inverted) ordering as ARM on RISC-V
 * \endif
 *
 * \param num Interrupt number \ref interrupt_nums
 * \return the IRQ priority
 */
uint irq_get_priority(uint num);

/*! \brief Enable or disable a specific interrupt on the executing core
 *  \ingroup hardware_irq
 *
 * \param num Interrupt number \ref interrupt_nums
 * \param enabled true to enable the interrupt, false to disable
 */
void irq_set_enabled(uint num, bool enabled);

/*! \brief Determine if a specific interrupt is enabled on the executing core
 *  \ingroup hardware_irq
 *
 * \param num Interrupt number \ref interrupt_nums
 * \return true if the interrupt is enabled
 */
bool irq_is_enabled(uint num);

/*! \brief Enable/disable multiple interrupts on the executing core
 *  \ingroup hardware_irq
 *
 * \param mask 32-bit mask with one bits set for the interrupts to enable/disable \ref interrupt_nums
 * \param enabled true to enable the interrupts, false to disable them.
 */
void irq_set_mask_enabled(uint32_t mask, bool enabled);

/*! \brief Enable/disable multiple interrupts on the executing core
 *  \ingroup hardware_irq
 *
 * \param n the index of the mask to update. n == 0 means 0->31, n == 1 mean 32->63 etc.
 * \param mask 32-bit mask with one bits set for the interrupts to enable/disable \ref interrupt_nums
 * \param enabled true to enable the interrupts, false to disable them.
 */
void irq_set_mask_n_enabled(uint n, uint32_t mask, bool enabled);

/*! \brief  Set an exclusive interrupt handler for an interrupt on the executing core.
 *  \ingroup hardware_irq
 *
 * Use this method to set a handler for single IRQ source interrupts, or when
 * your code, use case or performance requirements dictate that there should
 * no other handlers for the interrupt.
 *
 * This method will assert if there is already any sort of interrupt handler installed
 * for the specified irq number.
 *
 * \param num Interrupt number \ref interrupt_nums
 * \param handler The handler to set. See \ref irq_handler_t
 * \see irq_add_shared_handler()
 */
void irq_set_exclusive_handler(uint num, irq_handler_t handler);

/*! \brief  Get the exclusive interrupt handler for an interrupt on the executing core.
 *  \ingroup hardware_irq
 *
 * This method will return an exclusive IRQ handler set on this core
 * by irq_set_exclusive_handler if there is one.
 *
 * \param num Interrupt number \ref interrupt_nums
 * \see irq_set_exclusive_handler()
 * \return handler The handler if an exclusive handler is set for the IRQ,
 *                 NULL if no handler is set or shared/shareable handlers are installed
 */
irq_handler_t irq_get_exclusive_handler(uint num);

/*! \brief Add a shared interrupt handler for an interrupt on the executing core
 *  \ingroup hardware_irq
 *
 * Use this method to add a handler on an irq number shared between multiple distinct hardware sources (e.g. GPIO, DMA or PIO IRQs).
 * Handlers added by this method will all be called in sequence from highest order_priority to lowest. The
 * irq_set_exclusive_handler() method should be used instead if you know there will or should only ever be one handler for the interrupt.
 *
 * This method will assert if there is an exclusive interrupt handler set for this irq number on this core, or if
 * the (total across all IRQs on both cores) maximum (configurable via PICO_MAX_SHARED_IRQ_HANDLERS) number of shared handlers
 * would be exceeded.
 *
 * \param num Interrupt number \ref interrupt_nums
 * \param handler The handler to set. See \ref irq_handler_t
 * \param order_priority The order priority controls the order that handlers for the same IRQ number on the core are called.
 * The shared irq handlers for an interrupt are all called when an IRQ fires, however the order of the calls is based
 * on the order_priority (higher priorities are called first, identical priorities are called in undefined order). A good
 * rule of thumb is to use PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY if you don't much care, as it is in the middle of
 * the priority range by default.
 *
 * \note The order_priority uses \em higher values for higher priorities which is the \em opposite of the CPU interrupt priorities passed
 * to irq_set_priority() which use lower values for higher priorities.
 *
 * \see irq_set_exclusive_handler()
 */
void irq_add_shared_handler(uint num, irq_handler_t handler, uint8_t order_priority);

/*! \brief Remove a specific interrupt handler for the given irq number on the executing core
 *  \ingroup hardware_irq
 *
 * This method may be used to remove an irq set via either irq_set_exclusive_handler() or
 * irq_add_shared_handler(), and will assert if the handler is not currently installed for the given
 * IRQ number
 *
 * \note This method may *only* be called from user (non IRQ code) or from within the handler
 * itself (i.e. an IRQ handler may remove itself as part of handling the IRQ). Attempts to call
 * from another IRQ will cause an assertion.
 *
 * \param num Interrupt number \ref interrupt_nums
 * \param handler The handler to removed.
 * \see irq_set_exclusive_handler()
 * \see irq_add_shared_handler()
 */
void irq_remove_handler(uint num, irq_handler_t handler);

/*! \brief Determine if the current handler for the given number is shared
 *  \ingroup hardware_irq
 *
 * \param num Interrupt number \ref interrupt_nums
 * \return true if the specified IRQ has a shared handler
 */
bool irq_has_shared_handler(uint num);

/*! \brief Get the current IRQ handler for the specified IRQ from the currently installed hardware vector table (VTOR)
 * of the execution core
 *  \ingroup hardware_irq
 *
 * \param num Interrupt number \ref interrupt_nums
 * \return the address stored in the VTABLE for the given irq number
 */
irq_handler_t irq_get_vtable_handler(uint num);

/*! \brief Clear a specific interrupt on the executing core
 *  \ingroup hardware_irq
 *
 * This method is only useful for "software" IRQs that are not connected to hardware (e.g. IRQs 26-31 on RP2040)
 * as the the NVIC always reflects the current state of the IRQ state of the hardware for hardware IRQs, and clearing
 * of the IRQ state of the hardware is performed via the hardware's registers instead.
 *
 * \param int_num Interrupt number \ref interrupt_nums
 */
static inline void irq_clear(uint int_num) {
#if PICO_RP2040
    *((volatile uint32_t *) (PPB_BASE + M0PLUS_NVIC_ICPR_OFFSET)) = (1u << ((uint32_t) (int_num & 0x1F)));
#elif defined(__riscv)
    // External IRQs are not latched, but we should clear the IRQ force bit here
    hazard3_irqarray_clear(RVCSR_MEIFA_OFFSET, int_num / 16, 1u << (int_num % 16));
#else
    nvic_hw->icpr[int_num/32] = 1 << (int_num % 32);
#endif
}

/*! \brief Force an interrupt to be pending on the executing core
 *  \ingroup hardware_irq
 *
 * This should generally not be used for IRQs connected to hardware.
 *
 * \param num Interrupt number \ref interrupt_nums
 */
void irq_set_pending(uint num);


/*! \brief Perform IRQ priority initialization for the current core
 *
 * \note This is an internal method and user should generally not call it.
 */
void runtime_init_per_core_irq_priorities(void);

static __force_inline void irq_init_priorities(void) {
    runtime_init_per_core_irq_priorities();
}

/*! \brief Claim ownership of a user IRQ on the calling core
 *  \ingroup hardware_irq
 *  
 * User IRQs starting from FIRST_USER_IRQ are not connected to any hardware, but can be triggered by \ref irq_set_pending.
 *
 * \note User IRQs are a core local feature; they cannot be used to communicate between cores. Therefore all functions
 * dealing with Uer IRQs affect only the calling core
 * 
 * This method explicitly claims ownership of a user IRQ, so other code can know it is being used.
 *
 * \param irq_num the user IRQ to claim
 */
void user_irq_claim(uint irq_num);

/*! \brief Mark a user IRQ as no longer used on the calling core
 *  \ingroup hardware_irq
 *
 * User IRQs starting from FIRST_USER_IRQ are not connected to any hardware, but can be triggered by \ref irq_set_pending.
 *
 * \note User IRQs are a core local feature; they cannot be used to communicate between cores. Therefore all functions
 * dealing with Uer IRQs affect only the calling core
 * 
 * This method explicitly releases ownership of a user IRQ, so other code can know it is free to use.
 * 
 * \note it is customary to have disabled the irq and removed the handler prior to calling this method.
 *
 * \param irq_num the irq irq_num to unclaim
 */
void user_irq_unclaim(uint irq_num);

/*! \brief Claim ownership of a free user IRQ on the calling core
 *  \ingroup hardware_irq
 *  
 * User IRQs starting from FIRST_USER_IRQ are not connected to any hardware, but can be triggered by \ref irq_set_pending.
 *
 * \note User IRQs are a core local feature; they cannot be used to communicate between cores. Therefore all functions
 * dealing with Uer IRQs affect only the calling core
 * 
 * This method explicitly claims ownership of an unused user IRQ if there is one, so other code can know it is being used.
 *
 * \param required if true the function will panic if none are available
 * \return the user IRQ number or -1 if required was false, and none were free
 */
int user_irq_claim_unused(bool required);

/*
*! \brief Check if a user IRQ is in use on the calling core
 *  \ingroup hardware_irq
 *  
 * User IRQs starting from FIRST_USER_IRQ are not connected to any hardware, but can be triggered by \ref irq_set_pending.
 *
 * \note User IRQs are a core local feature; they cannot be used to communicate between cores. Therefore all functions
 * dealing with Uer IRQs affect only the calling core
 *
 * \param irq_num the irq irq_num
 * \return true if the irq_num is claimed, false otherwise
 * \sa user_irq_claim
 * \sa user_irq_unclaim
 * \sa user_irq_claim_unused
 */
bool user_irq_is_claimed(uint irq_num);

void __unhandled_user_irq(void);

#ifdef __riscv
enum riscv_vector_num {
    RISCV_VEC_MACHINE_EXCEPTION = 0,
    RISCV_VEC_MACHINE_SOFTWARE_IRQ = 3,
    RISCV_VEC_MACHINE_TIMER_IRQ = 7,
    RISCV_VEC_MACHINE_EXTERNAL_IRQ = 11,
};

irq_handler_t irq_set_riscv_vector_handler(enum riscv_vector_num index, irq_handler_t handler);
#endif

#if PICO_SECURE
static inline void irq_assign_to_ns(uint irq_num, bool ns) {
    check_irq_param(irq_num);
    if (ns) nvic_hw->itns[irq_num >> 5] |= 1u << (irq_num & 0x1fu);
    else nvic_hw->itns[irq_num >> 5] &= ~(1u << (irq_num & 0x1fu));
}
#endif
#ifdef __cplusplus
}
#endif

#endif
#endif
