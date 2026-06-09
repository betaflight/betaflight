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

// ESP32 dual-core support. Core 0 (PRO_CPU) runs the flight controller and owns
// all interrupts/peripherals; core 1 (APP_CPU) is a compute-offload helper that
// runs a command loop and executes functions handed to it via multicoreExecute.
//
// The Xtensa cores share the instruction/data cache on the S3, so once core 0
// has enabled the cache (start_esp32s3.S) core 1 can execute from the IROM
// (flash XIP) window without any extra per-core cache setup. Internal SRAM is
// not cached, so the command queue below is coherent between cores with just
// `volatile` + memw ordering; it is single-producer (core 0) / single-consumer
// (core 1), so no lock is required.

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

// Referenced by the always-linked core 1 asm entry stub (start_esp32s3.S).
uint32_t core1StackTop;

#ifdef USE_MULTICORE

#include "platform/multicore.h"

#include "soc/soc.h"
#include "soc/system_reg.h"
#include "soc/rtc_cntl_reg.h"

// ROM helper: set the address APP_CPU jumps to when released from reset.
extern void ets_set_appcpu_boot_addr(uint32_t start);
// Core 1 entry stub (start_esp32s3.S): installs core 1's stack, calls esp32Core1Main().
extern void esp32Core1Entry(void);

#define CORE1_STACK_SIZE 8192
static uint8_t core1Stack[CORE1_STACK_SIZE] __attribute__((aligned(16)));

#define MC_QUEUE_LEN 8    // must be a power of two

typedef struct {
    multicoreCommand_e command;
    core1_func_t *func;
} mcMessage_t;

static volatile mcMessage_t mcQueue[MC_QUEUE_LEN];
static volatile uint32_t mcHead;        // next slot to write (core 0)
static volatile uint32_t mcTail;        // next slot to read  (core 1)
static volatile uint32_t mcBlockingSeq; // bumped by core 1 after a blocking func
static volatile bool core1Running;

static inline void mcBarrier(void)
{
    __asm__ volatile ("memw" ::: "memory");
}

// Runs on core 1.
void esp32Core1Main(void)
{
    core1Running = true;
    mcBarrier();

    while (true) {
        if (mcTail == mcHead) {
            continue;   // no work queued
        }

        mcMessage_t msg = mcQueue[mcTail & (MC_QUEUE_LEN - 1)];
        mcBarrier();

        switch (msg.command) {
        case MULTICORE_CMD_FUNC:
            if (msg.func) {
                msg.func();
            }
            break;
        case MULTICORE_CMD_FUNC_BLOCKING:
            if (msg.func) {
                msg.func();
            }
            mcBarrier();
            mcBlockingSeq++;   // signal completion to core 0
            break;
        case MULTICORE_CMD_STOP:
            core1Running = false;
            mcBarrier();
            while (true) { }   // park; core 0 may re-launch via the HW sequence
        default:
            break;
        }

        mcBarrier();
        mcTail++;
    }
}

static void esp32StartCore1Hw(void)
{
    core1StackTop = (uint32_t)(core1Stack + CORE1_STACK_SIZE);

    // Un-stall APP_CPU (clear the software-stall fields, split across two RTC regs).
    CLEAR_PERI_REG_MASK(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_SW_STALL_APPCPU_C0_M);
    CLEAR_PERI_REG_MASK(RTC_CNTL_SW_CPU_STALL_REG, RTC_CNTL_SW_STALL_APPCPU_C1_M);

    // Enable APP_CPU clock and take it out of reset (skip if a debugger already did).
    if (!REG_GET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_CLKGATE_EN)) {
        REG_SET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_CLKGATE_EN);
        REG_CLR_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RUNSTALL);
        REG_SET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RESETING);
        REG_CLR_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RESETING);
    }

    ets_set_appcpu_boot_addr((uint32_t)esp32Core1Entry);
}

void multicoreStart(void)
{
    mcHead = mcTail = 0;
    mcBlockingSeq = 0;
    core1Running = false;
    mcBarrier();

    esp32StartCore1Hw();

    while (!core1Running) { }   // wait for core 1 to enter its command loop
}

// Producer side (core 0 only).
static void mcEnqueue(multicoreCommand_e command, core1_func_t *func)
{
    while ((mcHead - mcTail) >= MC_QUEUE_LEN) { }   // wait for a free slot
    const uint32_t slot = mcHead & (MC_QUEUE_LEN - 1);
    mcQueue[slot].command = command;
    mcQueue[slot].func = func;
    mcBarrier();
    mcHead++;
    mcBarrier();
}

void multicoreExecute(core1_func_t *func)
{
    mcEnqueue(MULTICORE_CMD_FUNC, func);
}

void multicoreExecuteBlocking(core1_func_t *func)
{
    const uint32_t seq = mcBlockingSeq;
    mcEnqueue(MULTICORE_CMD_FUNC_BLOCKING, func);
    while (mcBlockingSeq == seq) { }   // wait for core 1 to finish
    mcBarrier();
}

void multicoreStop(void)
{
    mcEnqueue(MULTICORE_CMD_STOP, NULL);
    while (core1Running) { }
}

#else // !USE_MULTICORE

// Multicore disabled: esp32StartCore1Hw() is never called, so core 1 stays in
// reset and never jumps to the entry stub. The stub still references this
// symbol, so provide a trivial definition to satisfy the linker.
void esp32Core1Main(void)
{
    while (true) { }
}

#endif // USE_MULTICORE
