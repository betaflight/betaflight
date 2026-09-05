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

// System initialisation and timing for HPMicro.
// Clocks, debug console, PMP, watchdog, micros()/millis(), failure indication
// and SDXC pin helpers.

#include "platform.h"
#include "board.h"
#include "build/debug.h"
#include "bgpr_hpmicro.h"
#include "drivers/system.h"
#include "drivers/persistent.h"
#include "drivers/nvic.h"
#include "hpm_ppor_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_romapi.h"
#include "hpm_bgpr_drv.h"
#ifdef HPM_PDGO_BASE
#include "hpm_pdgo_drv.h"
#endif
#include "drivers/time.h"
#include "light_led.h"
#include "sound_beeper.h"
#include "hpm_mchtmr_drv.h"
#include "hpm_wdg_drv.h"
#include "scheduler/scheduler.h"
#include "watchdog_hpmicro.h"
#include "hpm_uart_drv.h"
#include "hpm_pmp_drv.h"
#include "hpm_sysctl_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_soc.h"
#include "hpm_interrupt.h"
#include "hpm_debug_console.h"
#include "hpm_pcfg_drv.h"
#include "io_hpmicro.h"
#include "nvic_hpmicro.h"
#include "hpm_sdxc_soc_drv.h"
#ifdef HPM6750
#include "hpm_pllctl_drv.h"
#else
#include "hpm_pllctlv2_drv.h"
#endif
uint32_t SystemCoreClock;
uint32_t cachedResetFlags;

static bool systemWatchdogStarted;
static bool systemWatchdogHasTaskSample;
static timeUs_t systemWatchdogLastTaskExecutionTime;

/*
 * clearResetFlagAndReset() runs with interrupts disabled until the software
 * reset fires. RESET_FLAG is W1C and its bits stay latched across warm
 * resets, so clear them immediately before ppor_sw_reset() to keep the boot
 * ROM out of its debug/ISP path on the next boot.
 */

FAST_CODE __attribute__((noinline))
static void clearResetFlagAndReset(uint32_t pporCounter)
{
    disable_global_irq(CSR_MSTATUS_MIE_MASK);
    __asm volatile ("fence.i");

    HPM_PPOR->RESET_FLAG = PPOR_RESET_FLAG_FLAG_MASK;
    ppor_reset_mask_set_source_enable(HPM_PPOR, ppor_reset_software);
    ppor_sw_reset(HPM_PPOR, pporCounter);
    while (1) {
    }
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    (void) requestType;
#ifdef HPM_BGPR_BASE
    /* BGPR word 0 is the ROM DFU trigger word; see bgpr_hpmicro.h. */
    (void) bgpr_write32(HPM_BGPR, BGPR_ROM_DFU_TRIGGER_WORD, BGPR_ROM_DFU_TRIGGER_MAGIC);
#endif
#ifdef HPM_PDGO_BASE
    if (!pdgo_is_retention_mode_enabled(HPM_PDGO)) {
        pdgo_enable_retention_mode(HPM_PDGO);
    }
    pdgo_write_gpr(HPM_PDGO, BGPR_ROM_DFU_TRIGGER_WORD, BGPR_ROM_DFU_TRIGGER_MAGIC);

#endif
    clearResetFlagAndReset(10);
}

bool isMPUSoftReset(void)
{
    return (cachedResetFlags & ppor_reset_software) != 0U;
}

void delay(timeMs_t ms)
{
    timeMs_t start = millis();
    while (millis() - start < ms) {
    }
}

static void indicate(uint8_t count, uint16_t duration)
{
    if (count) {
        LED1_ON;
        LED0_OFF;

        for (; count > 0; count--) {
            LED1_TOGGLE;
            LED0_TOGGLE;
            BEEP_ON;
            delay(duration);

            LED1_TOGGLE;
            LED0_TOGGLE;
            BEEP_OFF;
            delay(duration);
        }
    }
}

void indicateFailure(failureMode_e mode, int codeRepeatsRemaining)
{
    for (; codeRepeatsRemaining > 0; codeRepeatsRemaining--) {
        indicate(WARNING_FLASH_COUNT, WARNING_FLASH_DURATION_MS);

        delay(WARNING_PAUSE_DURATION_MS);

        indicate(mode + 1, WARNING_CODE_DURATION_LONG_MS);

        delay(1000);
    }
}

void failureMode(failureMode_e mode)
{
    // The indication sequence intentionally lasts longer than the watchdog
    // timeout and must finish before requesting the ROM bootloader.
    systemWatchdogDisable();
    indicateFailure(mode, 10);
#ifdef DEBUG
    systemReset();
#else
    systemResetToBootloader(BOOTLOADER_REQUEST_ROM);
#endif
}

void systemWatchdogInit(void)
{
    // WDG0 runs from the independent 32 kHz clock. It raises an interrupt
    // after 128K periods (4 seconds), then resets after another 128 periods
    // (approximately 4 ms) unless the ISR observes scheduler progress.
    const uint32_t control =
        WDG_CTRL_RSTTIME_SET(reset_interval_clock_period_mult_128) |
        WDG_CTRL_INTTIME_SET(interrupt_interval_clock_period_multi_128k) |
        WDG_CTRL_RSTEN_MASK | WDG_CTRL_INTEN_MASK | WDG_CTRL_EN_MASK;

    systemWatchdogHasTaskSample = false;
    systemWatchdogLastTaskExecutionTime = 0;
    clock_add_to_group(clock_watchdog0, BOARD_RUNNING_CORE);
    intc_m_disable_irq(IRQn_WDG0);
    wdg_write_enable(HPM_WDG0);
    HPM_WDG0->CTRL = control;
    wdg_clear_status(HPM_WDG0, WDG_ST_INTEXPIRED_MASK);
    wdg_restart(HPM_WDG0);
    systemWatchdogStarted = true;
    intc_m_enable_irq_with_priority(IRQn_WDG0, hpmPlicPriorityFromNvic(NVIC_PRIO_MAX));
}

void systemWatchdogDisable(void)
{
    if (systemWatchdogStarted) {
        intc_m_disable_irq(IRQn_WDG0);
        wdg_disable(HPM_WDG0);
        wdg_clear_status(HPM_WDG0, WDG_ST_INTEXPIRED_MASK);
        systemWatchdogStarted = false;
    }
}

SDK_DECLARE_EXT_ISR_M(IRQn_WDG0, systemWatchdogIsr)
void systemWatchdogIsr(void)
{
    taskInfo_t systemTaskInfo;

    getTaskInfo(TASK_SYSTEM, &systemTaskInfo);

    const bool schedulerProgressed = !systemWatchdogHasTaskSample ||
                                     systemTaskInfo.totalExecutionTimeUs != systemWatchdogLastTaskExecutionTime;
    systemWatchdogHasTaskSample = true;
    systemWatchdogLastTaskExecutionTime = systemTaskInfo.totalExecutionTimeUs;

    if (schedulerProgressed) {
        wdg_clear_status(HPM_WDG0, WDG_ST_INTEXPIRED_MASK);
        wdg_restart(HPM_WDG0);
    } else {
        // Avoid repeatedly entering the ISR while the short reset interval
        // elapses. Leaving the watchdog pending lets hardware reset the SoC.
        intc_m_disable_irq(IRQn_WDG0);
    }
}


// Default configurations
#ifndef CONFIG_HPM_CONSOLE_UART_BASE
#define CONFIG_HPM_CONSOLE_UART_BASE       HPM_UART0
#endif

#ifndef CONFIG_HPM_CONSOLE_UART_IRQ
#define CONFIG_HPM_CONSOLE_UART_IRQ        IRQn_UART0
#endif

#ifndef CONFIG_HPM_CONSOLE_UART_BAUDRATE
#define CONFIG_HPM_CONSOLE_UART_BAUDRATE   (115200UL)
#endif

#ifndef CONFIG_HPM_LED_GPIO_CTRL
#define CONFIG_HPM_LED_GPIO_CTRL         HPM_GPIO0
#endif

#ifndef CONFIG_HPM_LED_GPIO_INDEX
#define CONFIG_HPM_LED_GPIO_INDEX         GPIO_DI_GPIOB
#endif

#ifndef CONFIG_HPM_LED_GPIO_PIN
#define CONFIG_HPM_LED_GPIO_PIN           12
#endif

#ifndef CONFIG_HPM_LED_OFF_LEVEL
#define CONFIG_HPM_LED_OFF_LEVEL          0
#endif

// Delay functions
void board_delay_ms(uint32_t ms)
{
    clock_cpu_delay_ms(ms);
}

void board_delay_us(uint32_t us)
{
    clock_cpu_delay_us(us);
}

// UART initialization
static void hpmInitUartPins(UART_Type *ptr)
{
    if (ptr == CONFIG_HPM_CONSOLE_UART_BASE) {
        // Configure UART0 RX pin (PY06)
        IOConfigGPIOAF(IOGetByTag(CONSOLE_UART_RX_PIN), IOCFG_AF_PP, CONSOLE_RX_AF);
        // Configure UART0 TX pin (PY07)
        IOConfigGPIOAF(IOGetByTag(CONSOLE_UART_TX_PIN), IOCFG_AF_PP, CONSOLE_TX_AF);
    }
    // Add other UART configurations as needed
}

// Console initialization
static void hpmInitConsole(void)
{
    console_config_t cfg;

    hpmInitUartPins((UART_Type *) CONFIG_HPM_CONSOLE_UART_BASE);

    clock_add_to_group(BOARD_CONSOLE_UART_CLK_NAME, 0);

    cfg.type = CONSOLE_TYPE_UART;
    cfg.base = (uint32_t) CONFIG_HPM_CONSOLE_UART_BASE;
    cfg.src_freq_in_hz = clock_get_frequency(BOARD_CONSOLE_UART_CLK_NAME);
    cfg.baudrate = CONFIG_HPM_CONSOLE_UART_BAUDRATE;

    console_init(&cfg);
}

// PMP initialization
static void hpmInitPmp(void)
{
    uint32_t startAddr;
    uint32_t endAddr;
    uint32_t length;
    pmp_entry_t pmp_entry[16];
    uint8_t index = 0;

    // Init non-cacheable memory
    extern uint32_t __noncacheable_start__[];
    extern uint32_t __noncacheable_end__[];
    startAddr = (uint32_t) __noncacheable_start__;
    endAddr = (uint32_t) __noncacheable_end__;
    length = endAddr - startAddr;
    if (length > 0) {
        assert((length & (length - 1U)) == 0U);
        assert((startAddr & (length - 1U)) == 0U);
        pmp_entry[index].pmp_addr = PMP_NAPOT_ADDR(startAddr, length);
        pmp_entry[index].pmp_cfg.val = PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);
        pmp_entry[index].pma_addr = PMA_NAPOT_ADDR(startAddr, length);
        pmp_entry[index].pma_cfg.val = PMA_CFG(ADDR_MATCH_NAPOT, MEM_TYPE_MEM_NON_CACHE_BUF, AMO_EN);
        index++;
    }

    pmp_config(&pmp_entry[0], index);
}

// Clock initialization
static void hpmInitClock(void)
{
#ifdef HPM6360

    uint32_t cpu0Freq = clock_get_frequency(clock_cpu0);

    if (cpu0Freq == PLLCTL_SOC_PLL_REFCLK_FREQ) {
        /* Configure the External OSC ramp-up time: ~9ms */
        pllctlv2_xtal_set_rampup_time(HPM_PLLCTLV2, 32UL * 1000UL * 9U);

        /* Select clock setting preset1 */
        sysctl_clock_set_preset(HPM_SYSCTL, sysctl_preset_1);
    }

    /* Add clocks to group 0 */
    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_ahbp, 0);
    clock_add_to_group(clock_axic, 0);
    clock_add_to_group(clock_axis, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_xpi1, 0);
    clock_add_to_group(clock_xdma, 0);
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_ram0, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_gpio, 0);
    clock_add_to_group(clock_mot0, 0);
    clock_add_to_group(clock_mot1, 0);
    clock_add_to_group(clock_synt, 0);
    clock_add_to_group(clock_ptpc, 0);
    /* Connect Group0 to CPU0 */
    clock_connect_group_to_cpu(0, 0);

    /* Bump up DCDC voltage to 1275mv */
    pcfg_dcdc_set_voltage(HPM_PCFG, 1275);

    /* Configure CPU to 648MHz, AXI/AHB to 162MHz */
    sysctl_config_cpu0_domain_clock(HPM_SYSCTL, clock_source_pll1_clk0, 1, 4, 4);
    /* Configure PLL1_CLK0 Post Divider to 1 */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, pllctlv2_pll1, pllctlv2_clk0, pllctlv2_div_1p0); /* PLL1CLK0: 648MHz */
    /* Configure PLL1_CLK1 Post Divider to 2 */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, pllctlv2_pll1, pllctlv2_clk1, pllctlv2_div_2p0); /* PLL1CLK1: 324MHz */
    /* Configure PLL1 clock frequency to 648MHz */
    if (status_success != pllctlv2_init_pll_with_freq(HPM_PLLCTLV2, pllctlv2_pll1, BOARD_CPU_FREQ)) {
        printf("Failed to set pllctlv2_pll1 to %luHz\n", BOARD_CPU_FREQ);
        clearResetFlagAndReset(10);
    }
    clock_update_core_clock();

    /* Configure mchtmr to 24MHz */
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
#endif
#ifdef HPM6750

    uint32_t cpu0Freq = clock_get_frequency(clock_cpu0);

    if (cpu0Freq == PLLCTL_SOC_PLL_REFCLK_FREQ) {
        /* Configure the External OSC ramp-up time: ~9ms */
        pllctl_xtal_set_rampup_time(HPM_PLLCTL, 32UL * 1000UL * 9U);

        /* Select clock setting preset1 */
        sysctl_clock_set_preset(HPM_SYSCTL, sysctl_preset_1);
    }

    /* Add clocks to group 0 */
    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_axi0, 0);
    clock_add_to_group(clock_axi1, 0);
    clock_add_to_group(clock_axi2, 0);
    clock_add_to_group(clock_ahb, 0);
    clock_add_to_group(clock_xdma, 0);
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_xpi1, 0);
    clock_add_to_group(clock_ram0, 0);
    clock_add_to_group(clock_ram1, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_lmm1, 0);
    clock_add_to_group(clock_gpio, 0);
    clock_add_to_group(clock_mot0, 0);
    clock_add_to_group(clock_mot1, 0);
    clock_add_to_group(clock_mot2, 0);
    clock_add_to_group(clock_mot3, 0);
    clock_add_to_group(clock_synt, 0);
    clock_add_to_group(clock_ptpc, 0);
    /* Connect Group0 to CPU0 */
    clock_connect_group_to_cpu(0, 0);

    /* Add clocks to Group1 */
    clock_add_to_group(clock_cpu1, 1);
    clock_add_to_group(clock_mchtmr1, 1);
    /* Connect Group1 to CPU1 */
    clock_connect_group_to_cpu(1, 1);

    if (status_success != pllctl_init_int_pll_with_freq(HPM_PLLCTL, 0, BOARD_CPU_FREQ)) {
        printf("Failed to set pll0_clk0 to %luHz\n", BOARD_CPU_FREQ);
        clearResetFlagAndReset(10);
    }

    clock_set_source_divider(clock_cpu0, clk_src_pll0_clk0, 1);
    clock_set_source_divider(clock_cpu1, clk_src_pll0_clk0, 1);
    clock_update_core_clock();

    clock_set_source_divider(clock_ahb, clk_src_pll1_clk1, 2);  /*200m hz */
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
    clock_set_source_divider(clock_mchtmr1, clk_src_osc24m, 1);
#endif
}

// USB initialization function
void hpm_usb_init(USB_Type *ptr)
{
#ifdef HPM6360
    (void) ptr;
#endif
#ifdef HPM6750
    clock_name_t usbClk = (ptr == HPM_USB0) ? clock_usb0 : clock_usb1;
#endif
#ifdef HPM6360
    clock_name_t usbClk = clock_usb0;
#endif

    // Enable USB clock
    clock_add_to_group(usbClk, 0);
}

void systemInit(void)
{
    SystemCoreClock = BOARD_CPU_FREQ;
    // RESET_FLAG is W1C, so preserve the startup snapshot before any board or
    // application code can clear it.
    cachedResetFlags = ppor_reset_get_flags(HPM_PPOR);

    // Initialize the persistent objects (BGPR words).  Must run after the
    // reset-cause snapshot above because persistentObjectInit() classifies
    // the reset via isMPUSoftReset(), and before any persistent-object access.
    persistentObjectInit();

    // Initialize system clocks
    hpmInitClock();

    // Initialize debug console
    hpmInitConsole();

    // Initialize PMP (Memory Protection)
    hpmInitPmp();

    // Print system information
    printf("==============================\n");
    printf("cpu0:\t\t %luHz\n", clock_get_frequency(clock_cpu0));
    printf("==============================\n");

    // Print Betaflight banner
    const uint8_t banner[] = { "\n\
----------------------------------------------------------------------\n\
 ____       _        _____ _ _       _     _      _   _ ____  __  __ _\n\
| __ )  ___| |_ __ _|  ___| (_) __ _| |__ | |_   | | | |  _ \\|  \\/  (_) ___ _ __ ___\n\
|  _ \\ / _ \\ __/ _` | |_  | | |/ _` | '_ \\| __|  | |_| | |_) | |\\/| | |/ __| '__/ _ \\\n\
| |_) |  __/ || (_| |  _| | | | (_| | | | | |_   |  _  |  __/| |  | | | (__| | | (_) |\n\
|____/ \\___|\\__\\__,_|_|   |_|_|\\__, |_| |_|\\__|  |_| |_|_|   |_|  |_|_|\\___|_|  \\___/\n\
                               |___/\n\
----------------------------------------------------------------------\n"
                             };
    printf("%s", banner);

    cycleCounterInit();
    systemWatchdogInit();
}

void systemReset(void)
{
    clearResetFlagAndReset(1000);
}

// MCHTMR is configured from the 24 MHz oscillator during system clock setup.
// Keep micros() safe if it is called before cycleCounterInit().
static uint32_t usPerTick = 24U;
#ifdef HPM6750
static uint32_t cyclesPerUs = 816U;
#elif defined(HPM6360)
static uint32_t cyclesPerUs = 648U;
#endif
// micros() and millis() independently accumulate the free-running MCHTMR
// count so neither function consumes timer ticks needed by the other.
static uint64_t microsLastCount;
static uint32_t microsResidualTicks;
static timeUs_t microsElapsedUs;
static uint64_t millisLastCount;
static uint32_t millisResidualTicks;
static timeMs_t millisElapsedMs;

FAST_CODE timeUs_t micros(void)
{
    uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    uint64_t count = mchtmr_get_count(HPM_MCHTMR);
    uint64_t deltaTicks = count - microsLastCount;
    microsLastCount = count;

    if ((deltaTicks >> 32) == 0 && (uint32_t) deltaTicks <= UINT32_MAX - microsResidualTicks) {
        const uint32_t pendingMicrosTicks = (uint32_t) deltaTicks + microsResidualTicks;
        microsElapsedUs += pendingMicrosTicks / usPerTick;
        microsResidualTicks = pendingMicrosTicks % usPerTick;
    } else {
        // This is only reachable after more than 2^32 timer ticks without a
        // micros() call (about 179 seconds at 24 MHz). Keep the full-range
        // fallback for startup/debug pauses without burdening the hot path.
        const uint64_t pendingMicrosTicks = deltaTicks + microsResidualTicks;
        microsElapsedUs += (timeUs_t) (pendingMicrosTicks / usPerTick);
        microsResidualTicks = (uint32_t) (pendingMicrosTicks % usPerTick);
    }

    const timeUs_t elapsedUs = microsElapsedUs;
    enable_global_irq(irq);

    return elapsedUs;
}

FAST_CODE timeUs_t microsISR(void)
{
    return micros();
}

timeMs_t millis(void)
{
    uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    uint64_t count = mchtmr_get_count(HPM_MCHTMR);
    uint64_t deltaTicks = count - millisLastCount;
    millisLastCount = count;

    // Maintain a separate 32-bit millisecond epoch so the micros() rollover
    // after about 71 minutes cannot make millis() jump backwards.
    const uint32_t ticksPerMs = usPerTick * 1000U;
    if ((deltaTicks >> 32) == 0 && (uint32_t) deltaTicks <= UINT32_MAX - millisResidualTicks) {
        const uint32_t pendingMillisTicks = (uint32_t) deltaTicks + millisResidualTicks;
        millisElapsedMs += pendingMillisTicks / ticksPerMs;
        millisResidualTicks = pendingMillisTicks % ticksPerMs;
    } else {
        // Preserve the full timer delta after more than 2^32 ticks without a
        // millis() call (about 179 seconds at 24 MHz).
        const uint64_t pendingMillisTicks = deltaTicks + millisResidualTicks;
        millisElapsedMs += (timeMs_t) (pendingMillisTicks / ticksPerMs);
        millisResidualTicks = (uint32_t) (pendingMillisTicks % ticksPerMs);
    }

    const timeMs_t elapsedMs = millisElapsedMs;
    enable_global_irq(irq);

    return elapsedMs;
}

void cycleCounterInit(void)
{
    clock_add_to_group(clock_mchtmr0, 0);
    mchtmr_init_counter(HPM_MCHTMR, 0);

    microsLastCount = 0;
    microsResidualTicks = 0;
    microsElapsedUs = 0;
    millisLastCount = 0;
    millisResidualTicks = 0;
    millisElapsedMs = 0;

    usPerTick = clock_get_frequency(clock_mchtmr0) / 1000000;
    if (!usPerTick) {
        usPerTick = 1;
    }

    cyclesPerUs = SystemCoreClock / 1000000;
    if (!cyclesPerUs) {
        cyclesPerUs = 1;
    }
}

void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us) {
    }
}

// Cycle counter functions
int32_t clockCyclesToMicros(int32_t clockCycles)
{
    return clockCycles / (int32_t) cyclesPerUs;
}

float clockCyclesToMicrosf(int32_t clockCycles)
{
    return (float) clockCycles / (float) cyclesPerUs;
}

int32_t clockCyclesTo10thMicros(int32_t clockCycles)
{
    return 10 * clockCycles / (int32_t) cyclesPerUs;
}

int32_t clockCyclesTo100thMicros(int32_t clockCycles)
{
    return 100 * clockCycles / (int32_t) cyclesPerUs;
}

uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros * cyclesPerUs;
}

uint32_t getCycleCounter(void)
{
    return read_csr(CSR_MCYCLE);
}

void debugInit(void)
{

}

uint32_t board_sd_configure_clock(SDXC_Type *ptr, uint32_t freq, bool need_inverse)
{
#ifdef HPM6360

    uint32_t actualFreq = 0;
    do {
        clock_name_t sdxc_clk = clock_sdxc0;
        clock_add_to_group(sdxc_clk, 0);
        sdxc_enable_inverse_clock(ptr, false);
        sdxc_enable_sd_clock(ptr, false);
        /* Configure the SDXC Frequency to 200MHz */
        clock_set_source_divider(sdxc_clk, clk_src_pll0_clk0, 2);
        sdxc_enable_freq_selection(ptr);

        hpm_stat_t status = clock_wait_source_stable(sdxc_clk);
        if (status != status_success) {
            break;
        }

        /* Configure the clock below 400KHz for the identification state */
        if (freq <= 400000UL) {
            sdxc_set_clock_divider(ptr, 600);
        }
        /* configure the clock to 24MHz for the SDR12/Default speed */
        else if (freq <= 26000000UL) {
            sdxc_set_clock_divider(ptr, 8);
        }
        /* Configure the clock to 50MHz for the SDR25/High speed/50MHz DDR/50MHz SDR */
        else if (freq <= 52000000UL) {
            sdxc_set_clock_divider(ptr, 4);
        }
        /* Configure the clock to 100MHz for the SDR50 */
        else if (freq <= 100000000UL) {
            sdxc_set_clock_divider(ptr, 2);
        }
        /* Configure the clock to 166MHz for SDR104/HS200/HS400  */
        else if (freq <= 208000000UL) {
            sdxc_set_clock_divider(ptr, 1);
        }
        /* For other unsupported clock ranges, configure the clock to 24MHz */
        else {
            sdxc_set_clock_divider(ptr, 8);
        }
        if (need_inverse) {
            sdxc_enable_inverse_clock(ptr, true);
        }
        sdxc_enable_sd_clock(ptr, true);
        actualFreq = clock_get_frequency(sdxc_clk) / sdxc_get_clock_divider(ptr);
    } while (false);
#else                           /* if defined(HPM6750) */
    uint32_t actualFreq = 0;
    do {
        clock_name_t sdxc_clk = (ptr == HPM_SDXC0) ? clock_sdxc0 : clock_sdxc1;
        clock_add_to_group(sdxc_clk, 0);
        sdxc_enable_inverse_clock(ptr, false);
        sdxc_enable_sd_clock(ptr, false);
        /* Configure the clock below 400KHz for the identification state */
        if (freq <= 400000UL) {
            clock_set_source_divider(sdxc_clk, clk_src_osc24m, 63);
        }
        /* configure the clock to 24MHz for the SDR12/Default speed */
        else if (freq <= 26000000UL) {
            clock_set_source_divider(sdxc_clk, clk_src_osc24m, 1);
        }
        /* Configure the clock to 50MHz for the SDR25/High speed/50MHz DDR/50MHz SDR */
        else if (freq <= 52000000UL) {
            clock_set_source_divider(sdxc_clk, clk_src_pll1_clk1, 8);
        }
        /* Configure the clock to 100MHz for the SDR50 */
        else if (freq <= 100000000UL) {
            clock_set_source_divider(sdxc_clk, clk_src_pll1_clk1, 4);
        }
        /* Configure the clock to 166MHz for SDR104/HS200/HS400  */
        else if (freq <= 208000000UL) {
            clock_set_source_divider(sdxc_clk, clk_src_pll2_clk0, 2);
        }
        /* For other unsupported clock ranges, configure the clock to 24MHz */
        else {
            clock_set_source_divider(sdxc_clk, clk_src_osc24m, 1);
        }
        if (need_inverse) {
            sdxc_enable_inverse_clock(ptr, true);
        }
        sdxc_enable_sd_clock(ptr, true);
        actualFreq = clock_get_frequency(sdxc_clk);
    } while (false);
#endif
    return actualFreq;
}

// SDXC pin initialization functions, used by hpm_sdk's hpm_sdmmc only,
void init_sdxc_cd_pin(SDXC_Type *ptr, bool as_gpio)
{
    (void)ptr; // Pin is defined in target.h or config.h not hard coded here
    const IO_t cdPin = IOGetByTag(SDXC_CD_PIN);

    if (as_gpio) {
        IOConfigGPIO(cdPin, IOCFG_IPU);
        return;
    }

#ifdef SDXC_CD_PIN_SDXC_AF
    IOConfigGPIOAF(cdPin, IOCFG_IPU, SDXC_CD_PIN_SDXC_AF);
#else
    // This target's selected CD pin has no SDXC CD mux. Keep it as a safe
    // GPIO input if a caller requests native card detection.
    IOConfigGPIO(cdPin, IOCFG_IPU);
#endif
}

void init_sdxc_cmd_pin(SDXC_Type *ptr, bool open_drain, bool is_1v8)
{
    (void)ptr; // Pin is defined in target.h or config.h not hard coded here
    (void) is_1v8;
    uint32_t cmd_func_ctl = SDXC_CMD_PIN_AF | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
    ioConfig_t cmd_config = open_drain ? IOCFG_AF_OD : IOCFG_AF_PP;
    // Configure SDXC command pin
    IOConfigGPIOAF(IOGetByTag(SDXC_CMD_PIN), cmd_config, cmd_func_ctl);

}

void init_sdxc_clk_data_pins(SDXC_Type *ptr, uint32_t width, bool is_1v8)
{
    (void)ptr; // Pin is defined in target.h or config.h not hard coded here
    (void) is_1v8;
    const uint32_t loopBack = IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);

    /* SDXC0.CLK */
    IOConfigGPIOAF(IOGetByTag(SDXC_CLK_PIN), IOCFG_AF_PP, SDXC_CLK_PIN_AF | loopBack);

    /* SDXC0.DATA0 */
    IOConfigGPIOAF(IOGetByTag(SDXC_DATA0_PIN), IOCFG_AF_PP, SDXC_DATA0_PIN_AF | loopBack);

    if (width == 4) {
        /* SDXC0.DATA1 */
        IOConfigGPIOAF(IOGetByTag(SDXC_DATA1_PIN), IOCFG_AF_PP, SDXC_DATA1_PIN_AF | loopBack);
        /* SDXC0.DATA2 */
        IOConfigGPIOAF(IOGetByTag(SDXC_DATA2_PIN), IOCFG_AF_PP, SDXC_DATA2_PIN_AF | loopBack);
        /* SDXC0.DATA3 */
        IOConfigGPIOAF(IOGetByTag(SDXC_DATA3_PIN), IOCFG_AF_PP, SDXC_DATA3_PIN_AF | loopBack);
    }
}
