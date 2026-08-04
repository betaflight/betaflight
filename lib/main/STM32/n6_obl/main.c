/*
 * Betaflight N6 OpenBootloader — entry, boot decision, and S→NS
 * hand-off trampoline.
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
 *   - otherwise                         → arm IWDG, BXNS to BF in NS
 *
 * The DFU loop itself is supplied by the OBL middleware (CubeN6
 * OpenBootloader, pulled in via Makefile VPATH and re-entered via
 * obl_app.c::OBL_run_dfu_*). This file owns the boot decision, IWDG
 * arming, and the BF jump trampoline. OBL is built with -mcmse so the
 * CMSIS peripheral pointers (RCC, RIFSC, GPIOx, DBGMCU, RISAF*, …)
 * resolve to their secure-alias bases automatically.
 */

#include <arm_cmse.h>
#include <string.h>

#include "main.h"
#include "flash_iface.h"

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

/* All-8-CIDs read + write enable for RISAF region CIDCFGR. Equivalent to
 * RDENC0_Msk | RDENC1_Msk | ... | RDENC7_Msk | WRENC0_Msk | ... | WRENC7_Msk
 * but kept as a single hex literal because the OR'd form is noisy. */
#define RISAF_CIDCFGR_ALL_CIDS_RW   0x00FF00FFUL

/* Configure one base region of a RISAF instance. `region` is 0-indexed
 * (the CMSIS RISAF_TypeDef exposes the per-region structs as REG[0..14]).
 * Per RM §7.5: write STARTR/ENDR/CIDCFGR before CFGR so BREN gates the
 * region atomically. CIDCFGR opens RDENC[0..7] and WRENC[0..7]. */
static inline void risaf_region_config(RISAF_TypeDef *risaf, uint32_t region,
                                       uint32_t start, uint32_t end,
                                       bool sec)
{
    risaf->REG[region].STARTR  = start;
    risaf->REG[region].ENDR    = end;
    risaf->REG[region].CIDCFGR = RISAF_CIDCFGR_ALL_CIDS_RW;
    risaf->REG[region].CFGR    = (sec ? RISAF_REGx_CFGR_SEC_Msk : 0UL)
                               | RISAF_REGx_CFGR_BREN_Msk;
}

static __attribute__((noreturn)) void jump_to_bf(void)
{
    const uint32_t bf_sp = *(volatile uint32_t *)(BF_VECTOR_BASE);
    const uint32_t bf_pc = *(volatile uint32_t *)(BF_VECTOR_BASE + 4U);

    HAL_SuspendTick();
    __disable_irq();

    /* Leave XSPI memory-mapped — BF runs XIP from 0x70100000. */
    SCB_DisableICache();
    SCB_DisableDCache();

    /* === Hand off to BF in NS state ===========================
     *
     * BF runs as a Non-Secure application; OBL is the only S-state
     * code on the device. RISAFs must be programmed so the NS CPU
     * can reach BF's data (AXISRAM1) and code (XSPI memory-mapped)
     * before BXNS. Boot-ROM default leaves every RISAF region
     * disabled, which per RM §3.5.7 means "Locations outside any
     * enabled region belong to the secure OS" — so without explicit
     * opens, NS access is silently RAZ.
     *
     * Order matters: RISAFs are configured AFTER OBL's last XSPI
     * read (bf_sp/bf_pc above) so OBL's S-state reads still see the
     * boot-ROM permissive default. Once XSPI's RISAF12 region is
     * flipped to NS, OBL itself can no longer read XSPI; only BF
     * (NS) can. AXISRAM2 (where OBL lives) is left at the boot-ROM
     * default (secure-only) so OBL keeps executing through the BXNS.
     *
     * RISAF6 is deliberately not programmed — ST's
     * Template_Isolation_XIP reference for the equivalent topology
     * (FSBL → AppliSecure → AppliNS in XSPI) leaves it alone too. */

    /* RISAF peripheral clock — boot ROM leaves AHB3ENR.RISAFEN=0,
     * which makes every access to RISAF register space stall the bus. */
    RCC->AHB3ENSR = RCC_AHB3ENSR_RISAFENS;
    (void)RCC->AHB3ENR;

    /* AXISRAM1 + AXISRAM3..6 clocks — boot ROM only clocks AXISRAM2
     * (where OBL itself runs). The NS BF lives in AXISRAM1 (data+stack)
     * and AXISRAM3..6 (D2_RAM); without these clocks enabled, any NS
     * access to 0x24010000+ or 0x24200000+ bus-faults. Idempotent with
     * the SystemInit set. */
    RCC->MEMENSR = RCC_MEMENSR_AXISRAM1ENS | RCC_MEMENSR_AXISRAM2ENS
                 | RCC_MEMENSR_AXISRAM3ENS | RCC_MEMENSR_AXISRAM4ENS
                 | RCC_MEMENSR_AXISRAM5ENS | RCC_MEMENSR_AXISRAM6ENS;
    (void)RCC->MEMENR;

    /* Slave-side RISAFs — open the resources BF (NS) needs.
     *
     * Boot-ROM default: every RISAF region disabled. RM §3.5.7 says a
     * memory location covered by a RISAF but outside any enabled region
     * "belongs to the secure OS", i.e., NS access bus-faults. So every
     * slave-RISAF-protected bank BF touches needs an explicit NS region.
     *
     * RISAF3  (AXISRAM2, 1 MiB):   region offset 0x91000..0xFFFFF NS
     *                              (= 0x24191000..0x241FFFFF, 444 KiB)
     *                              — BF's data/bss/stack/.ram_code. The
     *                              low half stays uncovered → S-only →
     *                              OBL's RAM at 0x34180400+ is protected.
     * RISAF12 (XSPI2, 256 MiB):    region covers offsets 0x100000..end
     *                              NS, leaving the OBL slot at offsets
     *                              0x00000..0xFFFFF uncovered. Per RM
     *                              §3.5.7 an uncovered range defaults
     *                              to S-only, so accidental NS deref of
     *                              0x70000000..0x700FFFFF bus-faults
     *                              instead of returning OBL image bytes.
     *
     * Intentionally NOT programmed:
     * - RISAF2 (AXISRAM1, 1 MiB): NS access to this bank silently RAZ's
     *   despite a correctly-programmed RISAF2 region 0 — root cause
     *   unknown after exhaustive diagnostic (boot-ROM RISAF2 left at
     *   default, RIFSC SEC bits clear, IASR clean, CIDCFGR opens CID 1
     *   which the M55 presents). Empirical workaround: BF moved to
     *   AXISRAM2 high half (above OBL's 64 KiB block) via RISAF3, which
     *   matches the ST template pattern of "S app in AXISRAM1, NS app
     *   in AXISRAM2".
     * - AXISRAM3..6 (NPURAM0..3, 0x24200000..0x243BFFFF, 1792 KiB):
     *   no slave RISAF exists for this range; bank security is gated by
     *   RIFSC->RISC_SECCFGRx[5] bits 17..20, which system_stm32n6xx_obl.c
     *   clears in its RIFSC open loop. NS access works once enabled. */
    /* RISAF3 region 0 (AXISRAM2 NS slice for BF's data/bss/stack/.ram_code).
     * BF lives in AXISRAM2 above the 64 KiB OBL block — RAM origin at
     * 0x24191000, length 444 KiB. RISAF3 granularity is 4 KiB so the
     * region is offset 0x91000..0xFFFFF (relative to AXISRAM2 base
     * 0x24100000). The lower half of AXISRAM2 (offsets 0x00000..0x90FFF,
     * which includes OBL's secure RAM) stays outside any enabled region
     * and defaults to S-only per RM §3.5.7 — protecting OBL from any
     * stray BF NS reach into its address space.
     *
     * RISAF2 (AXISRAM1) and RISAF6 are intentionally not programmed —
     * BF doesn't use AXISRAM1 in this layout, and RISAF6 matches the
     * ST Template_Isolation_XIP reference. */
    risaf_region_config(RISAF3,  0U, 0x00091000UL, 0x000FFFFFUL, false);
    risaf_region_config(RISAF12, 0U, 0x00100000UL, 0x0FFFFFFFUL, false);

    /* === Per-CPU TrustZone setup for the NS application ====
     *
     * NSACR (S-only): grant NS code access to FPU coprocessor banks
     * (CP10 + CP11). Without this, BF's first FPU instruction in NS
     * state takes a NOCP UsageFault.
     *
     * CPACR_NS: NS-bank CPACR is at reset value (FPU access disabled
     * for NS). BF's __libc_init_array runs before its own systemInit
     * and may emit FPU instructions (newlib helpers, struct passing).
     * Enable CP10/CP11 in the NS bank now so the early NS code can run.
     *
     * AIRCR.BFHFNMINS: set to 1 so NS HardFault / BusFault / NMI route to
     * NS handlers. Required for BF to diagnose its own faults via NS
     * HardFault_Handler. Without this NS faults route to S where OBL has
     * only Default_Handler (infinite loop) — chip wedges silently. */
    SCB->NSACR    = SCB_NSACR_CP10_Msk | SCB_NSACR_CP11_Msk;
    SCB_NS->CPACR = (3UL << 20) | (3UL << 22);  /* CP10/CP11 full access in NS bank */
    SCB->AIRCR = (SCB->AIRCR & ~(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_BFHFNMINS_Msk))
                 | (0x05FAUL << SCB_AIRCR_VECTKEY_Pos)
                 | SCB_AIRCR_BFHFNMINS_Msk;
    __DSB();
    __ISB();

    /* SAU (Security Attribution Unit) — REQUIRED for BXNS to NS code.
     * Per RM0486 §3.5.1: "at reset, the SAU unilaterally determines
     * that the entire memory is secure." Without SAU regions defining
     * NS, BXNS to 0x7010xxxx resolves the destination as S → SecureFault.
     *
     * Per-region NS instead of ALLNS=1: with explicit regions, addresses
     * outside any region stay S-default — which is what OBL needs for
     * its S-state accesses at 0x34xxxxxx, 0x50xxxxxx, 0x54xxxxxx etc.
     *
     *   Region 0: AXISRAM1 (0x24000000..0x240FFFFF) NS — BF main RAM
     *   Region 1: AHB/APB NS-alias peripherals (0x40000000..0x4FFFFFFF)
     *   Region 2: XSPI memory-mapped (0x70100000..0x7FFFFFFF) NS — skip
     *             the OBL slot at 0x70000000..0x700FFFFF so a stray NS
     *             deref of an OBL address is rejected by the SAU before
     *             it reaches the bus (matches the RISAF12 region split).
     *   Region 3: AXISRAM3..6 (0x24200000..0x243BFFFF) NS — D2_RAM /
     *             LCD framebuffer. AXISRAM2 NS (0x24100000..0x241FFFFF)
     *             is intentionally NOT covered: nothing in BF uses it,
     *             and leaving it outside any SAU region means a stray
     *             NS access bus-faults instead of reaching the bank.
     *
     * RBAR/RLAR are 32-byte granularity (low 5 bits ignored).
     *
     * Per UM3234 the boot ROM also configures SAU. Disable the unit and
     * clear every region (RLAR.ENABLE=0) before programming ours,
     * otherwise leftover boot-ROM regions ≥ 4 keep their classification
     * and can shadow what we configure. */
    {
        SAU->CTRL = 0;
        for (uint32_t i = 0; i < 8U; i++) {
            SAU->RNR  = i;
            SAU->RBAR = 0;
            SAU->RLAR = 0;
        }
        __DSB();
        __ISB();

        const struct { uint32_t base; uint32_t limit; } regs[] = {
            /* BF RAM lives in AXISRAM2 NS above OBL's 64 KiB block.
             * AXISRAM2 lower half + OBL's secure region stays outside
             * any SAU region → SAU-S by default, blocking any stray
             * NS deref of OBL's address space. */
            { 0x24191000UL, 0x241FFFE0UL },   /* AXISRAM2 high (BF)  */
            { 0x40000000UL, 0x4FFFFFE0UL },   /* NS peripherals      */
            { 0x70100000UL, 0x7FFFFFE0UL },   /* XSPI (excl. OBL)    */
            { 0x24200000UL, 0x243BFFE0UL },   /* AXISRAM3..6 / D2RAM */
        };
        for (uint32_t i = 0; i < (uint32_t)(sizeof(regs) / sizeof(regs[0])); i++) {
            SAU->RNR  = i;
            SAU->RBAR = regs[i].base;
            SAU->RLAR = regs[i].limit | SAU_RLAR_ENABLE_Msk;
        }
        SAU->CTRL = SAU_CTRL_ENABLE_Msk;
        __DSB();
        __ISB();
    }

    /* NS VTOR — point BF at its vector table at the XIP slot base. */
    SCB_NS->VTOR = BF_VECTOR_BASE;
    __DSB();
    __ISB();

    /* Clear every NVIC IRQ enable + pending bit before the hand-off so
     * no S-side leftover interrupt fires the instant the NS app does
     * `__enable_irq()`. BF's own startup clears these again; doing it
     * here protects the transition window. */
    for (uint32_t i = 0; i < (uint32_t)(sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0])); i++) {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }
    __DSB();
    __ISB();

    /* Re-target every external IRQ to NS so BF can enable/handle them
     * through the NS NVIC bank. Reset default is ITNS = 0 (all IRQs
     * Secure-targeted), which would route every peripheral IRQ (USB,
     * GPDMA, EXTI, TIMx, …) into S handlers OBL doesn't implement —
     * typically a tight Default_Handler loop. With NS-targeted IRQs the
     * NVIC banks the configuration registers to the NS bank so BF's
     * NVIC_EnableIRQ / SetPriority / SetTargetState calls land where
     * the IRQ actually fires. */
    for (uint32_t i = 0; i < (uint32_t)(sizeof(NVIC->ITNS) / sizeof(NVIC->ITNS[0])); i++) {
        NVIC->ITNS[i] = 0xFFFFFFFFUL;
    }

    /* SysTick is a system exception, not an external IRQ — NVIC->ITNS
     * doesn't cover it. Its NS/S target lives in ICSR.STTNS (bit 24,
     * Secure-only writable). Without this, BF's NS SysTick fires the
     * Secure-side SysTick exception, which OBL doesn't implement, so the
     * exception pends in S forever (NS-side ICSR shows PENDSTSET=1 +
     * VECTPENDING=15 with VECTACTIVE=0) and BF's NS handler never runs —
     * uwTick / sysTickUptime stay at 0 and the first HAL_Delay /
     * delay() call after the BF init that doesn't poll hardware directly
     * hangs forever. */
    SCB->ICSR |= SCB_ICSR_STTNS_Msk;
    __DSB();
    __ISB();

    /* CMSIS-compliant ARMv8-M S→NS hand-off.
     *
     * `cmse_nonsecure_call` on the function-pointer type makes the
     * compiler emit the proper BXNS-style branch *and* the AAPCS-CMSE
     * register-clearing prologue (zero R0-R3 / R12 / FPSCR caller-saved
     * state, so no Secure data leaks into the NS app). The function-
     * pointer cast also implicitly clears the LSB of the target so the
     * branch always transitions S → NS (with LSB=1 BXNS stays in S and
     * would SecureFault on the NS-attributed XSPI fetch).
     * __TZ_set_MSP_NS installs BF's stack pointer in the NS bank. */
    __TZ_set_MSP_NS(bf_sp);

    /* Clear S PRIMASK / FAULTMASK before BXNS. The __disable_irq() above
     * leaves PRIMASK_S=1; empirically the Cortex-M55 keeps NS exceptions
     * (SysTick, PendSV, peripheral IRQs targeted NS via ITNS) pending in
     * the NS-bank ICSR but never delivers them while PRIMASK_S is set —
     * even though Armv8-M masks are nominally banked per Security state.
     * Re-enable S-side IRQs so NS exception delivery works post-BXNS. */
    __enable_irq();
    __set_FAULTMASK(0);
    __set_BASEPRI(0);
    __DSB();
    __ISB();

    typedef void __attribute__((cmse_nonsecure_call)) (*ns_reset_fn)(void);
    const ns_reset_fn bf_reset = (ns_reset_fn)(bf_pc & ~1UL);
    bf_reset();

    while (1) {
        __NOP();
    }
}

/* ---------- Entry ---------- */

int main(void)
{
    /* RCC->RSR must be sampled before HAL_Init / SystemClock_Config touch
     * RCC — HAL_RCC_DeInit (called from System_DeInit during
     * SystemClock_Config) writes RMVF and erases the IWDGRSTF / LCKRSTF /
     * WWDGRSTF flags the boot decision below depends on. Snapshot the
     * value here and clear so subsequent boots see a stable state. */
    const uint32_t saved_rsr = RCC->RSR;
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

#ifdef OBL_FORCE_RECOVERY
    /* Bring-up only: SWD-loaded OBL used to refresh nor0 0x0. Skip the
     * magic check so we always enter Recovery, exposing nor0 from 0x0
     * for OBL+BF reinstall. The committed signed OBL has this flag
     * undefined. */
    (void)nor0_has_valid_obl;
    OBL_run_dfu_recovery();
    /* unreachable */
#endif

    if (!nor0_has_valid_obl()) {
        OBL_run_dfu_recovery();
        /* unreachable */
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
     * bits. Same value as in SystemInit. */
    DBGMCU->CR = DBGMCU_CR_DBGCLKEN
               | DBGMCU_CR_DBG_SLEEP
               | DBGMCU_CR_DBG_STOP
               | DBGMCU_CR_DBG_STANDBY;
    (void)DBGMCU->CR;
}
