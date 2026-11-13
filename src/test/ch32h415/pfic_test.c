/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/* Host test of the CH32 priority map and the real ATOMIC_BLOCK cleanup.
 * regression.py supplies the actual MMIO accessors with a simulated register.
 * This checks both possible equality rules; hardware timing still needs a board.
 */
#include <assert.h>

// Host barriers replace only the RISC-V fence, not the priority operations.
static inline void __set_BASEPRI(uint32_t priority)
{
    __set_BASEPRI_nb(priority);
    __asm volatile ("" ::: "memory");
}

static inline void __set_BASEPRI_MAX(uint32_t priority)
{
    __set_BASEPRI_MAX_nb(priority);
    __asm volatile ("" ::: "memory");
}

#define PLATFORM_CUSTOM_BASEPRI_NB
#include "build/atomic.h"
#include "drivers/nvic.h"

static bool masked(unsigned irqPriority, bool inclusive)
{
    return threshold != 0 && (inclusive ? irqPriority >= threshold : irqPriority > threshold);
}

static void checkCoverage(unsigned group)
{
    assert(threshold != 0 && (threshold & 0x0F) == 0);
    for (unsigned irqGroup = 0; irqGroup < 4; irqGroup++) {
        for (unsigned sub = 0; sub < 4; sub++) {
            const unsigned priority = NVIC_BUILD_PRIORITY(irqGroup, sub);
            assert(priority != 0);
            assert(NVIC_PRIORITY_BASE(priority) == irqGroup);
            assert(masked(priority, false) == (irqGroup >= group));
            assert(masked(priority, true) == (irqGroup >= group));
        }
    }
}

static void checkIrq(unsigned criticalPriority, unsigned irqPriority)
{
    assert(threshold == 0);
    ATOMIC_BLOCK(criticalPriority) {
        assert(masked(irqPriority, false));
        assert(masked(irqPriority, true));
    }
    assert(threshold == 0);
    assert(!masked(irqPriority, false));
    assert(!masked(irqPriority, true));
}

static void returnFromCriticalSection(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        checkCoverage(0);
        return;
    }
    assert(false);
}

int main(void)
{
    // Check every group's IRQ coverage, not just expected register constants.
    for (unsigned outer = 0; outer < 4; outer++) {
        for (unsigned sub = 0; sub < 4; sub++) {
            assert(threshold == 0);
            ATOMIC_BLOCK(NVIC_BUILD_PRIORITY(outer, sub)) {
                checkCoverage(outer);
                const unsigned saved = threshold;
                for (unsigned inner = 0; inner < 4; inner++) {
                    ATOMIC_BLOCK_NB(NVIC_BUILD_PRIORITY(inner, 0)) {
                        checkCoverage(inner < outer ? inner : outer);
                    }
                    assert(threshold == saved);
                }
                returnFromCriticalSection();
                assert(threshold == saved);
                ATOMIC_BLOCK(NVIC_PRIO_MAX) {
                    break;
                }
                assert(threshold == saved);
                ATOMIC_BLOCK(NVIC_PRIO_MAX) {
                    continue;
                }
                assert(threshold == saved);
                __set_BASEPRI_MAX_nb(0);
                assert(threshold == saved);
            }
            assert(threshold == 0);
        }
    }
    __set_BASEPRI_MAX_nb(0);
    assert(threshold == 0);

    const unsigned uartDmaPriorities[] = {
        NVIC_PRIO_SERIALUART1_TXDMA, NVIC_PRIO_SERIALUART2_TXDMA,
        NVIC_PRIO_SERIALUART3_TXDMA, NVIC_PRIO_SERIALUART4_TXDMA,
        NVIC_PRIO_SERIALUART5_TXDMA, NVIC_PRIO_SERIALUART6_TXDMA,
        NVIC_PRIO_SERIALUART7_TXDMA, NVIC_PRIO_SERIALUART8_TXDMA,
    };
    for (unsigned i = 0; i < sizeof(uartDmaPriorities) / sizeof(uartDmaPriorities[0]); i++) {
        checkIrq(NVIC_PRIO_SERIALUART_TXDMA, uartDmaPriorities[i]);
    }
    checkIrq(NVIC_PRIO_USB, NVIC_PRIO_USB);
    checkIrq(NVIC_PRIO_MAX, NVIC_PRIO_MPU_INT_EXTI);
    checkIrq(NVIC_PRIO_MAX, NVIC_PRIO_SPI_DMA);
    checkIrq(NVIC_PRIO_MAX, NVIC_PRIO_I2C_EV);
    checkIrq(NVIC_PRIO_MAX, NVIC_PRIO_I2C_ER);
    checkIrq(NVIC_PRIO_MAX, NVIC_BUILD_PRIORITY(3, 0)); // SysTick

    // Direct setters restore a saved threshold verbatim, including unmasking.
    for (unsigned group = 0; group < 4; group++) {
        const unsigned saved = ch32AtomicPriority(NVIC_BUILD_PRIORITY(group, 0));
        __set_BASEPRI_nb(saved);
        assert(__get_BASEPRI() == saved);
        __set_BASEPRI_nb(0);
        assert(__get_BASEPRI() == 0);
    }
    puts("IRQ coverage, higher-priority delivery, both equality rules, nested cleanup and zero restore");
}
