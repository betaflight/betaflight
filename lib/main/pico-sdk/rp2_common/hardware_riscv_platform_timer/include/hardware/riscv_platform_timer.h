#ifndef _HARDWARE_RISCV_PLATFORM_TIMER_
#define _HARDWARE_RISCV_PLATFORM_TIMER_

#ifdef __cplusplus
extern "C" {
#endif

#include "pico.h"
#include "hardware/structs/sio.h"

/** \file hardware/riscv_platform_timer.h
 *  \defgroup hardware_riscv_platform_timer hardware_riscv_platform_timer
 *
 * \brief Accessors for standard RISC-V platform timer (mtime/mtimecmp), available on
 * Raspberry Pi microcontrollers with RISC-V processors
 *
 * Note this header can be used by Arm as well as RISC-V processors, as the
 * timer is a memory-mapped peripheral external to the processors. The name
 * refers to this timer being a standard RISC-V peripheral.
 *
 */

/*! \brief Enable or disable the RISC-V platform timer
 *  \ingroup hardware_riscv_platform_timer
 *
 * This enables and disables the counting of the RISC-V platform timer. It
 * does not enable or disable the interrupts, which are asserted
 * unconditionally when a given core's mtimecmp/mtimecmph registers are
 * greater than the current 64-bit value of the mtime/mtimeh registers.
 *
 * \param enabled Pass true to enable, false to disable
 */
static inline void riscv_timer_set_enabled(bool enabled) {
    if (enabled) {
        // Note atomic rwtype is not supported on SIO
        sio_hw->mtime_ctrl |=  SIO_MTIME_CTRL_EN_BITS;
    } else {
        sio_hw->mtime_ctrl &= ~SIO_MTIME_CTRL_EN_BITS;
    }
}

/*! \brief Configure the RISC-V platform timer to run at full system clock speed
 *  \ingroup hardware_riscv_platform_timer
 *
 * \param fullspeed Pass true to increment at system clock speed, false to
 *  increment at the frequency defined by the system tick generator
 *  (the `ticks` block)
 */
static inline void riscv_timer_set_fullspeed(bool fullspeed) {
    if (fullspeed) {
        sio_hw->mtime_ctrl |=  SIO_MTIME_CTRL_FULLSPEED_BITS;
    } else {
        sio_hw->mtime_ctrl &= ~SIO_MTIME_CTRL_FULLSPEED_BITS;
    }
}

/*! \brief Read the RISC-V platform timer
 *  \ingroup hardware_riscv_platform_timer
 *
 * \return Current 64-bit mtime value
 */
static inline uint64_t riscv_timer_get_mtime(void) {
    // Read procedure from RISC-V ISA manual to avoid being off by 2**32 on
    // low half rollover -- note this loop generally executes only once, and
    // should never execute more than twice:
    uint32_t h0, l, h1;
    do {
        h0 = sio_hw->mtimeh;
        l  = sio_hw->mtime;
        h1 = sio_hw->mtimeh;
    } while (h0 != h1);
    return l | (uint64_t)h1 << 32;
}

/*! \brief Update the RISC-V platform timer
 *  \ingroup hardware_riscv_platform_timer
 *
 * This function should only be called when the timer is disabled via
 * riscv_timer_set_enabled(). Note also that unlike the mtimecmp comparison
 * values, mtime is *not* core-local, so updates on one core will be visible
 * to the other core.
 *
 * \param mtime New value to set the RISC-V platform timer to
 */
static inline void riscv_timer_set_mtime(uint64_t mtime) {
    // This ought really only be done when the timer is stopped, but we can
    // make things a bit safer by clearing the low half of the counter, then
    // writing high half, then low half. This protects against the low half
    // rolling over, and largely avoids getting an intermediate value that is
    // higher than either the original or new value, if the timer is running.
    //
    // Note that on RP2350, mtime is shared between the two cores!(mtimcemp is
    // core-local however.)
    sio_hw->mtime  = 0;
    sio_hw->mtimeh = mtime >> 32;
    sio_hw->mtime  = mtime & 0xffffffffu;
}

/*! \brief Get the current RISC-V platform timer mtimecmp value for this core
 *  \ingroup hardware_riscv_platform_timer
 *
 * Get the current mtimecmp value for the calling core. This function is
 * interrupt-safe as long as timer interrupts only increase the value of
 * mtimecmp. Otherwise, it must be called with timer interrupts disabled.
 *
 * \return Current value of mtimecmp
 */
static inline uint64_t riscv_timer_get_mtimecmp(void) {
    // Use the same procedure as reading mtime, which should be safe assuming
    // mtimecmp increases monotonically with successive interrupts.
    uint32_t h0, l, h1;
    do {
        h0 = sio_hw->mtimecmph;
        l  = sio_hw->mtimecmp;
        h1 = sio_hw->mtimecmph;
    } while (h0 != h1);
    return l | (uint64_t)h1 << 32;
}

/*! \brief Set a new RISC-V platform timer interrupt comparison value (mtimecmp) for this core
 *  \ingroup hardware_riscv_platform_timer
 *
 * This function updates the mtimecmp value for the current core. The calling
 * core's RISC-V platform timer interrupt is asserted whenever the 64-bit
 * mtime value (stored in 32-bit mtime/mtimeh registers) is greater than or
 * equal to this core's current mtime/mtimecmph value.
 *
 * \param mtime New value to set the RISC-V platform timer to
 */
static inline void riscv_timer_set_mtimecmp(uint64_t mtimecmp) {
    // Use write procedure from RISC-V ISA manual to avoid causing a spurious
    // interrupt when updating the two halves of mtimecmp.
    // No lower than original:
    sio_hw->mtimecmp  = -1u;
    // No lower than original, no lower than new (assuming new >= original):
    sio_hw->mtimecmph = mtimecmp >> 32;
    // Equal to new:
    sio_hw->mtimecmp  = mtimecmp & 0xffffffffu;
}

#ifdef __cplusplus
}
#endif

#endif
