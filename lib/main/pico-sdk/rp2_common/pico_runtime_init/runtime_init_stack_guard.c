#include "pico/runtime.h"

#if !PICO_RUNTIME_NO_INIT_PER_CORE_INSTALL_STACK_GUARD
#if PICO_RP2040
#include "hardware/structs/mpu.h"
#elif defined(__riscv)
#include "hardware/riscv.h"
#endif
// this is called for each thread since they have their own MPU
void runtime_init_per_core_install_stack_guard(void *stack_bottom) {
    // this is called b4 runtime_init is complete, so beware printf or assert

    uintptr_t addr = (uintptr_t) stack_bottom;
    // the minimum we can protect is 32 bytes on a 32 byte boundary, so round up which will
    // just shorten the valid stack range a tad
    addr = (addr + 31u) & ~31u;

#if PICO_RP2040
    // Armv6-M MPU
    // make sure no one is using the MPU yet
    if (mpu_hw->ctrl) {
        // Note that it would be tempting to change this to a panic, but it happens so early, printing is not a good idea
        __breakpoint();
    }
    // mask is 1 bit per 32 bytes of the 256 byte range... clear the bit for the segment we want
    uint32_t subregion_select = 0xffu ^ (1u << ((addr >> 5u) & 7u));
    mpu_hw->ctrl = 5; // enable mpu with background default map
    mpu_hw->rbar = (addr & (uint)~0xff) | M0PLUS_MPU_RBAR_VALID_BITS | 0;
    mpu_hw->rasr = 1 // enable region
                   | (0x7 << 1) // size 2^(7 + 1) = 256
                   | (subregion_select << 8)
                   | 0x10000000; // XN = disable instruction fetch; no other bits means no permissions

#elif defined(__riscv)
    #if !PICO_RP2350
#error "Check PMP configuration for new platform"
#endif
    // RISC-V PMP, RP2350 configuration of Hazard3: 8 non-hardwired regions,
    // NAPOT only, 32-byte granule, with nonstandard PMPCFGM0 register to
    // apply regions to M-mode without locking them.
    // Make sure no one is using the PMP yet
    bool dirty_pmp =
        riscv_read_csr(pmpcfg0) != 0 ||
        riscv_read_csr(pmpcfg1) != 0 ||
        riscv_read_csr(RVCSR_PMPCFGM0_OFFSET) != 0;

    if (dirty_pmp) {
        __breakpoint();
    }

    // Note pmpaddr is in units of 4 bytes, so right-shift 2.
    riscv_write_csr(pmpaddr0, (addr | 0x0fu) >> 2);
    // Make this region inaccessible in both M-mode and U-mode (but don't lock it)
    riscv_write_csr(RVCSR_PMPCFGM0_OFFSET, 0x1u);
    riscv_write_csr(pmpcfg0, RVCSR_PMPCFG0_R0_A_VALUE_NAPOT << RVCSR_PMPCFG0_R0_A_LSB);

#else
//    // Armv8-M MPU
//    // make sure no one is using the MPU yet
//    if (mpu_hw->ctrl) {
//        __breakpoint();
//    }
//    mpu_hw->rnr = 0;
//    // Read-only, privileged-only, nonexecutable. (Good enough because stack
//    // is usually written first, on a stack push)
//    mpu_hw->rbar = addr | (2u << M33_MPU_RBAR_AP_LSB) | (M33_MPU_RBAR_XN_BITS);
//    mpu_hw->rlar = addr | M33_MPU_RLAR_EN_BITS;
//    // Enable MPU (and leave default attributes applied even for privileged software)
//    mpu_hw->ctrl = M33_MPU_CTRL_PRIVDEFENA_BITS | M33_MPU_CTRL_ENABLE_BITS;
    pico_default_asm_volatile(
            "msr msplim, %0"
            :
            : "r" (stack_bottom));
#endif
}

#endif