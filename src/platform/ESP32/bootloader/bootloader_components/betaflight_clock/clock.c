/*
 * Betaflight ESP32-S3 bootloader clock hook.
 *
 * The ESP-IDF second-stage bootloader intentionally runs the CPU at its
 * conservative boot frequency (80 MHz on ESP32-S3).  A normal ESP-IDF
 * application changes that later from call_start_cpu0(), but Betaflight uses
 * its own bare-metal entry point and therefore never executes that code.
 */

#include "sdkconfig.h"

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "soc/rtc.h"

// Referenced with -u by the ESP-IDF bootloader build so this component and its
// strong hook symbols are retained even though the default hooks are weak.
void bootloader_hooks_include(void)
{
}

void bootloader_after_init(void)
{
    rtc_cpu_freq_config_t config;

    // Use Espressif's clock routine rather than touching PLL/LDO registers
    // directly.  It raises the S3 voltage bias, selects the 480 MHz PLL / 2,
    // keeps APB at 80 MHz and updates the ROM ticks-per-microsecond value.
    if (rtc_clk_cpu_freq_mhz_to_config(240, &config)) {
        rtc_clk_cpu_freq_set_config(&config);
    }
}
#endif
