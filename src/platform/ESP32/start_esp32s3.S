/*
 * ESP32-S3 application entry and minimal Xtensa startup.
 *
 * The image is loaded by the ESP-IDF second-stage bootloader, which maps the
 * flash segments, copies the RAM segments and calls this entry point as an
 * ordinary windowed-ABI C function. The S3 ROM has no "direct boot", so a
 * bootloader is mandatory (IDF's own entry, call_start_cpu0, is IRAM_ATTR).
 *
 * Requirements established during bring-up (each was a distinct failure):
 *  - Entry point and this stub MUST be in IRAM, in the IRAM-only region below
 *    0x40378000 (the bootloader uses 0x40378000+ as scratch during cache setup;
 *    placing code there gets it corrupted). We use 0x40374000.
 *  - The first instruction MUST be the windowed prologue `entry`.
 *  - Constants live in an inline literal pool reached by l32r (backwards only);
 *    a `j` skips over it.
 *  - The bootloader sets up only the DATA cache (to read mapped rodata); the
 *    app must configure and enable the INSTRUCTION cache itself, or the first
 *    execution from the IROM (flash XIP) window faults. See the icache config
 *    call below.
 *
 * The stub switches to its own stack, silences the watchdogs left running from
 * the boot sequence, zeroes .bss (the bootloader does not), enables the
 * instruction cache, then calls main() (which lives in the IROM window).
 */

/* RTC_CNTL register block (ESP32-S3) */
#define RTC_CNTL_WDTCONFIG0_REG     0x60008098
#define RTC_CNTL_WDTWPROTECT_REG    0x600080B0
#define RTC_CNTL_WDT_WKEY_VALUE     0x50D83AA1
#define RTC_CNTL_SWD_CONF_REG       0x600080B4
#define RTC_CNTL_SWD_WPROTECT_REG   0x600080B8
#define RTC_CNTL_SWD_WKEY_VALUE     0x8F1D312A

/* ROM rom_config_instruction_cache_mode(size, ways, line_size): configures and
 * enables the instruction cache. IDF defaults for ESP32-S3. */
#define ROM_CONFIG_INSTRUCTION_CACHE_MODE 0x40001a1c
#define ICACHE_SIZE_16KB            0x4000
#define ICACHE_WAYS                 8
#define ICACHE_LINE_SIZE            32

    .section .iram1.start, "ax", @progbits
    .balign 4
    .global _start
    .type _start, @function
_start:
    /* Windowed-ABI prologue MUST be the first instruction at the entry point. */
    entry   a1, 32

    /* Jump over the inline literal pool (l32r below reads it backwards). */
    j       .Lstart_code

    .balign 4
.LP_estack:     .word _estack
.LP_sbss:       .word _sbss
.LP_ebss:       .word _ebss
.LP_wdt_wp:     .word RTC_CNTL_WDTWPROTECT_REG
.LP_wdt_key:    .word RTC_CNTL_WDT_WKEY_VALUE
.LP_wdt_cfg:    .word RTC_CNTL_WDTCONFIG0_REG
.LP_swd_wp:     .word RTC_CNTL_SWD_WPROTECT_REG
.LP_swd_key:    .word RTC_CNTL_SWD_WKEY_VALUE
.LP_swd_cfg:    .word RTC_CNTL_SWD_CONF_REG
.LP_cfg_icache: .word ROM_CONFIG_INSTRUCTION_CACHE_MODE
.LP_main:       .word main

.Lstart_code:
    /* Switch to our own stack at the top of DRAM (16-byte aligned). */
    l32r    a3, .LP_estack
    movi    a4, -16
    and     a1, a3, a4

    /* Disable the RTC watchdog (left enabled for the boot window). */
    l32r    a2, .LP_wdt_wp
    l32r    a3, .LP_wdt_key
    s32i    a3, a2, 0           /* unlock */
    memw
    l32r    a2, .LP_wdt_cfg
    movi    a3, 0
    s32i    a3, a2, 0           /* clear enable + all stages */
    memw
    l32r    a2, .LP_wdt_wp
    movi    a3, 0
    s32i    a3, a2, 0           /* re-lock */
    memw

    /* Super (RTC) watchdog: enable auto-feed so it stops resetting us. */
    l32r    a2, .LP_swd_wp
    l32r    a3, .LP_swd_key
    s32i    a3, a2, 0           /* unlock */
    memw
    l32r    a2, .LP_swd_cfg
    l32i    a3, a2, 0
    movi    a4, 1
    slli    a4, a4, 31          /* RTC_CNTL_SWD_AUTO_FEED_EN = BIT(31) */
    or      a3, a3, a4
    s32i    a3, a2, 0
    memw
    l32r    a2, .LP_swd_wp
    movi    a3, 0
    s32i    a3, a2, 0           /* re-lock */
    memw

    /* Zero .bss (the bootloader loads .data but does not clear .bss). */
    l32r    a2, .LP_sbss
    l32r    a3, .LP_ebss
    movi    a4, 0
.Lbss_zero:
    bgeu    a2, a3, .Lbss_done
    s32i    a4, a2, 0
    addi    a2, a2, 4
    j       .Lbss_zero
.Lbss_done:

    /* Configure + enable the instruction cache before executing IROM code. */
    movi    a6, 1
    slli    a6, a6, 14          /* ICACHE_SIZE_16KB */
    movi    a7, ICACHE_WAYS
    movi    a8, ICACHE_LINE_SIZE
    l32r    a11, .LP_cfg_icache
    callx4  a11
    isync

    /* main(0, 0). */
    movi    a6, 0
    movi    a7, 0
    l32r    a8, .LP_main
    callx4  a8

    /* main() should never return; trap if it does. */
.Lhang:
    j       .Lhang
    .size _start, . - _start
