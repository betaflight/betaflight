/*
 * Betaflight N6 OpenBootloader — entry, boot decision, and DFU
 * trampoline.
 *
 * Loaded by the boot ROM into AXISRAM2 secure at 0x34180400 from XSPI
 * nor0 0x0 on cold boot, or via USB DFU @FSBL alt during recovery.
 * The two cases are distinguished by checking nor0 0x0 for the signed
 * FSBL header magic.
 *
 * Boot decision:
 *   - nor0 0x0 has no valid OBL header  → recovery DFU (writes nor0 0x0)
 *   - RCC->RSR has IWDGRSTF/LCKRSTF/
 *     WWDGRSTF set                      → previous BF crashed → DFU
 *   - BF vector table at 0x70100000
 *     invalid                           → DFU
 *   - otherwise                         → arm IWDG, jump to BF
 *
 * The DFU loop itself is supplied by the OBL middleware (CubeN6
 * OpenBootloader, pulled in via Makefile VPATH and re-entered via
 * obl_app.c::OBL_run_dfu_*). This file owns the boot decision, IWDG
 * arming, and the BF jump trampoline.
 */

#include <string.h>

#include "main.h"
#include "flash_iface.h"
#include "platform/bf_obl_contract.h"

/* ---------- Layout constants ---------- */
#define XSPI_NOR_BASE               0x70000000U
#define BF_VECTOR_BASE              0x70100000U
#define BF_SLOT_OFFSET              0x00100000U   /* 1 MiB OBL reserve before BF */

/* "STM2" little-endian — the signed FSBL header magic at offset 0. */
#define STM2_MAGIC_LE               0x324D5453U

/* IWDG window: ~10 s @ LSI 32 kHz / (256 × 1250). BF must refresh from
 * its scheduler-driven watchdog task before this expires. */
#define OBL_IWDG_PRESCALER          IWDG_PRESCALER_256
#define OBL_IWDG_RELOAD             1250U
#define OBL_IWDG_WINDOW             1250U        /* window disabled (= reload) */

IWDG_HandleTypeDef hiwdg;

/* Forward-declared from obl_app.c. Both run the OBL middleware DFU
 * loop and never return — they end via NVIC_SystemReset. */
__attribute__((noreturn)) void OBL_run_dfu_normal(void);
__attribute__((noreturn)) void OBL_run_dfu_recovery(void);

static void SystemClock_Config(void);
extern void HAL_MspInit(void);

/* ---------- Boot decision helpers ---------- */

static bool nor0_has_valid_obl(void)
{
    if (!flash_memmap_on()) {
        return false;
    }
    const uint32_t magic = *(volatile uint32_t *)(XSPI_NOR_BASE);
    return magic == STM2_MAGIC_LE;
}

static bool bf_vector_table_valid(void)
{
    if (!flash_memmap_on()) {
        return false;
    }
    const uint32_t bf_sp = *(volatile uint32_t *)(BF_VECTOR_BASE);
    const uint32_t bf_pc = *(volatile uint32_t *)(BF_VECTOR_BASE + 4U);

    /* SP must point into AXISRAM (top nibble 0x2). */
    if ((bf_sp & 0xF8000000U) != 0x20000000U) {
        return false;
    }
    /* Reset_Handler must live in the BF XIP slot (top 12 bits == 0x701)
     * with the thumb bit set. */
    if ((bf_pc & 0xFFF00000U) != BF_VECTOR_BASE) {
        return false;
    }
    if ((bf_pc & 1U) == 0U) {
        return false;
    }
    return true;
}


/* ---------- IWDG ---------- */

static void iwdg_start(void)
{
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = OBL_IWDG_PRESCALER;
    hiwdg.Init.Reload    = OBL_IWDG_RELOAD;
    hiwdg.Init.Window    = OBL_IWDG_WINDOW;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
    /* IWDG cannot be disabled until system reset. BF must call
     * HAL_IWDG_Refresh() from its scheduler, or the timeout fires NRST
     * and OBL routes the next boot to DFU via RSR.IWDGRSTF. */
}

/* ---------- Jump trampoline ---------- */

static __attribute__((noreturn)) void jump_to_bf(void)
{
    const uint32_t bf_sp = *(volatile uint32_t *)(BF_VECTOR_BASE);
    const uint32_t bf_pc = *(volatile uint32_t *)(BF_VECTOR_BASE + 4U);

    /* @DBGRAM markers, host-readable via DFU alt 1:
     *   0x08  jump-trampoline reached
     *   0x0C  bf_sp captured at jump
     *   0x40  bf_pc captured at jump */
    *(volatile uint32_t *)0x24100008U = 0x0B100000U;
    *(volatile uint32_t *)0x2410000CU = bf_sp;
    *(volatile uint32_t *)0x24100040U = bf_pc;

    HAL_SuspendTick();
    __disable_irq();

    /* Leave XSPI memory-mapped — BF runs XIP from 0x70100000. */
    SCB_DisableICache();
    SCB_DisableDCache();

    SCB->VTOR = BF_VECTOR_BASE;
    __DSB();
    __ISB();
    __set_MSP(bf_sp);
    /* Re-enable IRQs so BF inherits PRIMASK=0 — its startup_*.s relies
     * on this for SysTick / HAL_Delay during HAL_Init. */
    __enable_irq();
    ((void (*)(void))bf_pc)();

    while (1) {
        __NOP();
    }
}

/* ---------- Entry ---------- */

int main(void)
{
    /* Bring-up marker in the @DBGRAM buffer (AXISRAM2 NS at 0x24100000):
     * 0x04 = OBL main() ran. */
    *(volatile uint32_t *)0x24100004U = 0xCAFEBABEU;

    /* RCC->RSR must be sampled before HAL_Init / SystemClock_Config touch
     * RCC — HAL_RCC_DeInit (called from System_DeInit during
     * SystemClock_Config) writes RMVF and erases the IWDGRSTF / LCKRSTF /
     * WWDGRSTF flags the boot decision below depends on. Mirror to
     * @DBGRAM 0x88 and clear here so the boot decision sees a stable
     * snapshot. */
    const uint32_t saved_rsr = RCC->RSR;
    *(volatile uint32_t *)0x24100088U = saved_rsr;
    RCC->RSR = RCC_RSR_RMVF;
    (void)RCC->RSR;

    HAL_Init();
    SystemClock_Config();

    if (!flash_init()) {
        Error_Handler();
    }
    if (!flash_memmap_on()) {
        Error_Handler();
    }

    if (!nor0_has_valid_obl()) {
        OBL_run_dfu_recovery();
        /* unreachable */
    }

    /* @DBGRAM diagnostic snapshot, host-readable via DFU alt 1:
     *   0x80        bf_sp from BF vector
     *   0x84        bf_pc from BF vector
     *   0x88        saved_rsr (mirrored at top of main, not rewritten here)
     *   0x8C        bf vector check_fail bits
     *   0x90..0x9C  DBGMCU IDCODE / CR (S) / SR (offset 0xFC) / CR (NS)
     *   0xA0..0xD0  XSPI reads at 14 offsets across nor0
     *   0xD8..0xEC  RISAF2 (AXISRAM2) CR / IASR / REG[0]
     *   0xF0        AXISRAM2 NS write/read self-probe readback
     *   0x110       AXISRAM2 NS probe write target (0xDEADBEEF) */
    {
        const uint32_t bf_sp = *(volatile uint32_t *)BF_VECTOR_BASE;
        const uint32_t bf_pc = *(volatile uint32_t *)(BF_VECTOR_BASE + 4U);
        uint32_t check_fail = 0;
        if ((bf_sp & 0xF8000000U) != 0x20000000U) check_fail |= 0x1U;
        if ((bf_pc & 0xFFF00000U) != BF_VECTOR_BASE) check_fail |= 0x2U;
        if ((bf_pc & 1U) == 0U) check_fail |= 0x4U;
        *(volatile uint32_t *)0x24100080U = bf_sp;
        *(volatile uint32_t *)0x24100084U = bf_pc;
        *(volatile uint32_t *)0x2410008CU = check_fail;

        /* DBGMCU readbacks: IDCODE (0x00 read-only chip ID — non-zero
         * iff APB3 bus to DBGMCU is clocked), CR via S and NS aliases
         * (bit 20 = DBGCLKEN), SR (offset 0xFC; bits 16/17 = AP0/AP1
         * enable). */
        *(volatile uint32_t *)0x24100090U = *(volatile uint32_t *)0x54001000UL;
        *(volatile uint32_t *)0x24100094U = *(volatile uint32_t *)0x54001004UL;
        *(volatile uint32_t *)0x24100098U = *(volatile uint32_t *)0x540010FCUL;
        *(volatile uint32_t *)0x2410009CU = *(volatile uint32_t *)0x44001004UL;

        /* XSPI read probe across nor0: A0/A4 OBL header, A8/AC mid- and
         * end-of-OBL-slot, B0..D0 14 points into and past BF flash. */
        *(volatile uint32_t *)0x241000A0U = *(volatile uint32_t *)0x70000000UL;
        *(volatile uint32_t *)0x241000A4U = *(volatile uint32_t *)0x70000004UL;
        *(volatile uint32_t *)0x241000A8U = *(volatile uint32_t *)0x70080000UL;
        *(volatile uint32_t *)0x241000ACU = *(volatile uint32_t *)0x700FFFFCUL;
        *(volatile uint32_t *)0x241000B0U = *(volatile uint32_t *)0x70100000UL;
        *(volatile uint32_t *)0x241000B4U = *(volatile uint32_t *)0x70100100UL;
        *(volatile uint32_t *)0x241000B8U = *(volatile uint32_t *)0x70100400UL;
        *(volatile uint32_t *)0x241000BCU = *(volatile uint32_t *)0x70110000UL;
        *(volatile uint32_t *)0x241000C0U = *(volatile uint32_t *)0x70200000UL;
        *(volatile uint32_t *)0x241000C4U = *(volatile uint32_t *)0x70400000UL;
        *(volatile uint32_t *)0x241000C8U = *(volatile uint32_t *)0x70800000UL;
        *(volatile uint32_t *)0x241000CCU = *(volatile uint32_t *)0x71000000UL;
        *(volatile uint32_t *)0x241000D0U = *(volatile uint32_t *)0x71800000UL;

        /* RISAF2 (AXISRAM2 region controller, secure base 0x54027000):
         * CR / IASR / REG[0] CFGR/STARTR/ENDR/CIDCFGR. */
        *(volatile uint32_t *)0x241000D8U = *(volatile uint32_t *)0x54027000UL;
        *(volatile uint32_t *)0x241000DCU = *(volatile uint32_t *)0x54027008UL;
        *(volatile uint32_t *)0x241000E0U = *(volatile uint32_t *)0x54027040UL;
        *(volatile uint32_t *)0x241000E4U = *(volatile uint32_t *)0x54027044UL;
        *(volatile uint32_t *)0x241000E8U = *(volatile uint32_t *)0x54027048UL;
        *(volatile uint32_t *)0x241000ECU = *(volatile uint32_t *)0x5402704CUL;

        /* AXISRAM2 NS write/read round-trip probe: write 0xDEADBEEF to
         * 0x24100110, read back into 0xF0. Full 32-bit readback proves
         * the NS path preserves all bytes. */
        *(volatile uint32_t *)0x24100110U = 0xDEADBEEFU;
        *(volatile uint32_t *)0x241000F0U = *(volatile uint32_t *)0x24100110U;
    }

    /* Reset-cause boot decision. RCC->RSR is the only on-chip primitive
     * that survives any reset short of POR/BOR — its flags are cleared
     * only by RMVF (handled above) or POR, and persist across NRST /
     * SRST / SYSRESETREQ. IWDGRSTF / LCKRSTF / WWDGRSTF set means the
     * previous boot ended with a crash (watchdog timeout, CPU lockup,
     * or window WD); route to DFU instead of re-launching the broken
     * BF in a loop. */
    const uint32_t crash_flags = RCC_RSR_IWDGRSTF | RCC_RSR_LCKRSTF | RCC_RSR_WWDGRSTF;
    const bool was_crash_reset = (saved_rsr & crash_flags) != 0U;

#ifdef OBL_FORCE_DFU
    /* Bring-up override: SWD-load OBL with this flag set when we want
     * DFU mode regardless of nor0 contents — used to flash a fresh BF
     * via dfu-util on top of an existing valid (but stale) BF. */
    OBL_run_dfu_normal();
    /* unreachable */
#endif

    if (was_crash_reset || !bf_vector_table_valid()) {
        OBL_run_dfu_normal();
        /* unreachable */
    }

    iwdg_start();
    jump_to_bf();
}

/* ---------- Plumbing ---------- */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

/* Tear the clock tree back to reset defaults before SystemClock_Config_Impl
 * runs OscConfig. Boot ROM's FSBL-load path leaves PLL1 active and the
 * CPU sourced from it, which makes HAL_RCC_OscConfig fail. Also resolves
 * the OpenBootloader_DeInit reference in CubeN6's app_openbootloader.c. */
void System_DeInit(void)
{
    HAL_RCC_DeInit();
}

extern void SystemClock_Config_Impl(void);
static void SystemClock_Config(void)
{
    SystemClock_Config_Impl();

    /* Re-assert DBGMCU.CR after the clock-tree reconfig — the brief
     * debug-clock outage during the CPU-clock switch can drop the
     * bits. Same value/aliases as in SystemInit. */
    *(volatile uint32_t *)0x54001004UL = (1UL << 20)
                                       | (1UL <<  0)
                                       | (1UL <<  1)
                                       | (1UL <<  2);
    (void)*(volatile uint32_t *)0x54001004UL;
}
