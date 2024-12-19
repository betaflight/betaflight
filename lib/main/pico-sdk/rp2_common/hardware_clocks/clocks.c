/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico.h"
#include "hardware/regs/clocks.h"
#include "hardware/platform_defs.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"

check_hw_layout(clocks_hw_t, clk[clk_adc].selected, CLOCKS_CLK_ADC_SELECTED_OFFSET);
check_hw_layout(clocks_hw_t, fc0.result, CLOCKS_FC0_RESULT_OFFSET);
check_hw_layout(clocks_hw_t, ints, CLOCKS_INTS_OFFSET);

static uint32_t configured_freq[CLK_COUNT];

static resus_callback_t _resus_callback;

// Clock muxing consists of two components:
// - A glitchless mux, which can be switched freely, but whose inputs must be
//   free-running
// - An auxiliary (glitchy) mux, whose output glitches when switched, but has
//   no constraints on its inputs
// Not all clocks have both types of mux.
static inline bool has_glitchless_mux(clock_handle_t clock) {
    return clock == clk_sys || clock == clk_ref;
}

void clock_stop(clock_handle_t clock) {
    clock_hw_t *clock_hw = &clocks_hw->clk[clock];
    hw_clear_bits(&clock_hw->ctrl, CLOCKS_CLK_USB_CTRL_ENABLE_BITS);
    configured_freq[clock] = 0;
}

/// \tag::clock_configure[]
static void clock_configure_internal(clock_handle_t clock, uint32_t src, uint32_t auxsrc, uint32_t actual_freq, uint32_t div) {
    clock_hw_t *clock_hw = &clocks_hw->clk[clock];

    // If increasing divisor, set divisor before source. Otherwise set source
    // before divisor. This avoids a momentary overspeed when e.g. switching
    // to a faster source and increasing divisor to compensate.
    if (div > clock_hw->div)
        clock_hw->div = div;

    // If switching a glitchless slice (ref or sys) to an aux source, switch
    // away from aux *first* to avoid passing glitches when changing aux mux.
    // Assume (!!!) glitchless source 0 is no faster than the aux source.
    if (has_glitchless_mux(clock) && src == CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX) {
        hw_clear_bits(&clock_hw->ctrl, CLOCKS_CLK_REF_CTRL_SRC_BITS);
        while (!(clock_hw->selected & 1u))
            tight_loop_contents();
    }
    // If no glitchless mux, cleanly stop the clock to avoid glitches
    // propagating when changing aux mux. Note it would be a really bad idea
    // to do this on one of the glitchless clocks (clk_sys, clk_ref).
    else {
        // Disable clock. On clk_ref and clk_sys this does nothing,
        // all other clocks have the ENABLE bit in the same position.
        hw_clear_bits(&clock_hw->ctrl, CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);
        if (configured_freq[clock] > 0) {
            // Delay for 3 cycles of the target clock, for ENABLE propagation.
            // Note XOSC_COUNT is not helpful here because XOSC is not
            // necessarily running, nor is timer...
            uint delay_cyc = configured_freq[clk_sys] / configured_freq[clock] + 1;
            busy_wait_at_least_cycles(delay_cyc * 3);
        }
    }

    // Set aux mux first, and then glitchless mux if this clock has one
    hw_write_masked(&clock_hw->ctrl,
        (auxsrc << CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB),
        CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS
    );

    if (has_glitchless_mux(clock)) {
        hw_write_masked(&clock_hw->ctrl,
            src << CLOCKS_CLK_REF_CTRL_SRC_LSB,
            CLOCKS_CLK_REF_CTRL_SRC_BITS
        );
        while (!(clock_hw->selected & (1u << src)))
            tight_loop_contents();
    }

    // Enable clock. On clk_ref and clk_sys this does nothing,
    // all other clocks have the ENABLE bit in the same position.
    hw_set_bits(&clock_hw->ctrl, CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);

    // Now that the source is configured, we can trust that the user-supplied
    // divisor is a safe value.
    clock_hw->div = div;
    configured_freq[clock] = actual_freq;
}

bool clock_configure(clock_handle_t clock, uint32_t src, uint32_t auxsrc, uint32_t src_freq, uint32_t freq) {
    assert(src_freq >= freq);

    if (freq > src_freq)
        return false;

    uint32_t div = (uint32_t)((((uint64_t) src_freq) << CLOCKS_CLK_GPOUT0_DIV_INT_LSB) / freq);
    uint32_t actual_freq = (uint32_t) ((((uint64_t) src_freq) << CLOCKS_CLK_GPOUT0_DIV_INT_LSB) / div);

    clock_configure_internal(clock, src, auxsrc, actual_freq, div);
    // Store the configured frequency
    return true;
}

void clock_configure_int_divider(clock_handle_t clock, uint32_t src, uint32_t auxsrc, uint32_t src_freq, uint32_t int_divider) {
    clock_configure_internal(clock, src, auxsrc, src_freq / int_divider, int_divider << CLOCKS_CLK_GPOUT0_DIV_INT_LSB);
}

void clock_configure_undivided(clock_handle_t clock, uint32_t src, uint32_t auxsrc, uint32_t src_freq) {
    clock_configure_internal(clock, src, auxsrc, src_freq, 1u << CLOCKS_CLK_GPOUT0_DIV_INT_LSB);
}

/// \end::clock_configure[]

/// \tag::clock_get_hz[]
uint32_t clock_get_hz(clock_handle_t clock) {
    return configured_freq[clock];
}
/// \end::clock_get_hz[]

void clock_set_reported_hz(clock_handle_t clock, uint hz) {
    configured_freq[clock] = hz;
}

/// \tag::frequency_count_khz[]
uint32_t frequency_count_khz(uint src) {
    fc_hw_t *fc = &clocks_hw->fc0;

    // If frequency counter is running need to wait for it. It runs even if the source is NULL
    while(fc->status & CLOCKS_FC0_STATUS_RUNNING_BITS) {
        tight_loop_contents();
    }

    // Set reference freq
    fc->ref_khz = clock_get_hz(clk_ref) / 1000;

    // FIXME: Don't pick random interval. Use best interval
    fc->interval = 10;

    // No min or max
    fc->min_khz = 0;
    fc->max_khz = 0xffffffff;

    // Set SRC which automatically starts the measurement
    fc->src = src;

    while(!(fc->status & CLOCKS_FC0_STATUS_DONE_BITS)) {
        tight_loop_contents();
    }

    // Return the result
    return fc->result >> CLOCKS_FC0_RESULT_KHZ_LSB;
}
/// \end::frequency_count_khz[]

static void clocks_handle_resus(void) {
    // Set clk_sys back to the ref clock rather than it being forced to clk_ref
    // by resus. Call the user's resus callback if they have set one

    // CLK SYS = CLK_REF. Must be running for this code to be running
    uint clk_ref_freq = clock_get_hz(clk_ref);
    clock_configure_undivided(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF,
                    0,
                    clk_ref_freq);

    // Assert we have been resussed
    assert(clocks_hw->resus.status & CLOCKS_CLK_SYS_RESUS_STATUS_RESUSSED_BITS);

    // Now we have fixed clk_sys we can safely remove the resus
    hw_set_bits(&clocks_hw->resus.ctrl, CLOCKS_CLK_SYS_RESUS_CTRL_CLEAR_BITS);
    hw_clear_bits(&clocks_hw->resus.ctrl, CLOCKS_CLK_SYS_RESUS_CTRL_CLEAR_BITS);

    // Now we should no longer be resussed
    assert(!(clocks_hw->resus.status & CLOCKS_CLK_SYS_RESUS_STATUS_RESUSSED_BITS));

    // Call the user's callback to notify them of the resus event
    if (_resus_callback) {
        _resus_callback();
    }
}

static void clocks_irq_handler(void) {
    // Clocks interrupt handler. Only resus but handle irq
    // defensively just in case.
    uint32_t ints = clocks_hw->ints;

    if (ints & CLOCKS_INTE_CLK_SYS_RESUS_BITS) {
        ints &= ~CLOCKS_INTE_CLK_SYS_RESUS_BITS;
        clocks_handle_resus();
    }

#ifndef NDEBUG
    if (ints) {
        panic("Unexpected clocks irq\n");
    }
#endif
}

void clocks_enable_resus(resus_callback_t resus_callback) {
    // Restart clk_sys if it is stopped by forcing it
    // to the default source of clk_ref. If clk_ref stops running this will
    // not work.

    // Store user's resus callback
    _resus_callback = resus_callback;

    irq_set_exclusive_handler(CLOCKS_IRQ, clocks_irq_handler);

    // Enable the resus interrupt in clocks
    clocks_hw->inte = CLOCKS_INTE_CLK_SYS_RESUS_BITS;

    // Enable the clocks irq
    irq_set_enabled(CLOCKS_IRQ, true);

    // 2 * clk_ref freq / clk_sys_min_freq;
    // assume clk_ref is 3MHz and we want clk_sys to be no lower than 1MHz
    uint timeout = 2 * 3 * 1;

    // Enable resus with the maximum timeout
    clocks_hw->resus.ctrl = CLOCKS_CLK_SYS_RESUS_CTRL_ENABLE_BITS | timeout;
}

void clock_gpio_init_int_frac(uint gpio, uint src, uint32_t div_int, uint8_t div_frac) {
    // Bit messy but it's as much code to loop through a lookup
    // table. The sources for each gpout generators are the same
    // so just call with the sources from GP0
    uint gpclk = 0;
    if      (gpio == 21) gpclk = clk_gpout0;
    else if (gpio == 23) gpclk = clk_gpout1;
    else if (gpio == 24) gpclk = clk_gpout2;
    else if (gpio == 25) gpclk = clk_gpout3;
#if !PICO_RP2040
    else if (gpio == 13) gpclk = clk_gpout0;
    else if (gpio == 15) gpclk = clk_gpout1;
#endif
    else {
        invalid_params_if(HARDWARE_CLOCKS, true);
    }

    // Set up the gpclk generator
    clocks_hw->clk[gpclk].ctrl = (src << CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_LSB) |
                                 CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS;
    clocks_hw->clk[gpclk].div = (div_int << CLOCKS_CLK_GPOUT0_DIV_INT_LSB) | div_frac;

    // Set gpio pin to gpclock function
    gpio_set_function(gpio, GPIO_FUNC_GPCK);
}

static const uint8_t gpin0_src[CLK_COUNT] = {
    CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0, // CLK_GPOUT0
    CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0, // CLK_GPOUT1
    CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0, // CLK_GPOUT2
    CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0, // CLK_GPOUT3
    CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,    // CLK_REF
    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,    // CLK_SYS
    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,   // CLK_PERI
#if !PICO_RP2040
    CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,   // CLK_HSTX
#endif
    CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,    // CLK_USB
    CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,    // CLK_ADC
#if PICO_RP2040
    CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,    // CLK_RTC
#endif
};

// Assert GPIN1 is GPIN0 + 1
static_assert(CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1 == (CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0 + 1), "hw mismatch");
static_assert(CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1 == (CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0 + 1), "hw mismatch");
static_assert(CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1 == (CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0 + 1), "hw mismatch");
static_assert(CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1 == (CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0 + 1), "hw mismatch");
static_assert(CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1    == (CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0    + 1), "hw mismatch");
static_assert(CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1    == (CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0    + 1), "hw mismatch");
static_assert(CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1   == (CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0   + 1), "hw mismatch");
#if HAS_HSTX
static_assert(CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1   == (CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0   + 1), "hw mismatch");
#endif
static_assert(CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1    == (CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0    + 1), "hw mismatch");
static_assert(CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1    == (CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0    + 1), "hw mismatch");
#if HAS_RP2040_RTC
static_assert(CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1    == (CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0    + 1), "hw mismatch");
#endif

bool clock_configure_gpin(clock_handle_t clock, uint gpio, uint32_t src_freq, uint32_t freq) {
    // Configure a clock to run from a GPIO input
    uint gpin = 0;
    if      (gpio == 20) gpin = 0;
    else if (gpio == 22) gpin = 1;
#if PICO_RP2350
    else if (gpio == 12) gpin = 0;
    else if (gpio == 14) gpin = 1;
#endif
    else {
        invalid_params_if(HARDWARE_CLOCKS, true);
    }

    // Work out sources. GPIN is always an auxsrc
    uint src = 0;

    // GPIN1 == GPIN0 + 1
    uint auxsrc = gpin0_src[clock] + gpin;

    if (has_glitchless_mux(clock)) {
        // AUX src is always 1
        src = 1;
    }

    // Set the GPIO function
    gpio_set_function(gpio, GPIO_FUNC_GPCK);

    // Now we have the src, auxsrc, and configured the gpio input
    // call clock configure to run the clock from a gpio
    return clock_configure(clock, src, auxsrc, src_freq, freq);
}

// everything running off the USB oscillator
void set_sys_clock_48mhz(void) {
    if (!running_on_fpga()) {
        // Change clk_sys to be 48MHz. The simplest way is to take this from PLL_USB
        // which has a source frequency of 48MHz
        clock_configure_undivided(clk_sys,
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                        USB_CLK_HZ);

        // Turn off PLL sys for good measure
        pll_deinit(pll_sys);

        // CLK peri is clocked from clk_sys so need to change clk_peri's freq
        clock_configure_undivided(clk_peri,
                        0,
                        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                        USB_CLK_HZ);
    }
}

// PICO_CONFIG: PICO_CLOCK_ADJUST_PERI_CLOCK_WITH_SYS_CLOCK, When the SYS clock PLL is changed keep the peripheral clock attached to it, type=bool, default=0, advanced=true, group=hardware_clocks
#ifndef PICO_CLOCK_ADJUST_PERI_CLOCK_WITH_SYS_CLOCK
// support old incorrect spelling too
#ifdef PICO_CLOCK_AJDUST_PERI_CLOCK_WITH_SYS_CLOCK
#define PICO_CLOCK_ADJUST_PERI_CLOCK_WITH_SYS_CLOCK PICO_CLOCK_AJDUST_PERI_CLOCK_WITH_SYS_CLOCK
#else
// By default, when reconfiguring the system clock PLL settings after runtime initialization,
// the peripheral clock is switched to the 48MHz USB clock to ensure continuity of peripheral operation.
// Setting this value to 1 changes the behavior to have the peripheral clock re-configured
// to the system clock at it's new frequency.
#define PICO_CLOCK_ADJUST_PERI_CLOCK_WITH_SYS_CLOCK 0
#endif
#endif

void set_sys_clock_pll(uint32_t vco_freq, uint post_div1, uint post_div2) {
    if (!running_on_fpga()) {
        clock_configure_undivided(clk_sys,
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                        USB_CLK_HZ);

        pll_init(pll_sys, PLL_SYS_REFDIV, vco_freq, post_div1, post_div2);
        uint32_t freq = vco_freq / (post_div1 * post_div2);

        // Configure clocks
        // CLK_REF is the XOSC source
        clock_configure_undivided(clk_ref,
                        CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
                        0, // No aux mux
                        XOSC_HZ);

        // CLK SYS = PLL SYS (usually) 125MHz / 1 = 125MHz
        clock_configure_undivided(clk_sys,
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                        freq);

#if PICO_CLOCK_ADJUST_PERI_CLOCK_WITH_SYS_CLOCK
        clock_configure_undivided(clk_peri,
                        0,
                        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                        freq);
#else
        clock_configure_undivided(clk_peri,
                        0, // Only AUX mux on ADC
                        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                        USB_CLK_HZ);
#endif
    }
}

bool check_sys_clock_hz(uint32_t freq_hz, uint *vco_out, uint *postdiv1_out, uint *postdiv2_out) {
    uint reference_freq_hz = XOSC_HZ / PLL_SYS_REFDIV;
    for (uint fbdiv = 320; fbdiv >= 16; fbdiv--) {
        uint vco_hz = fbdiv * reference_freq_hz;
        if (vco_hz < PICO_PLL_VCO_MIN_FREQ_HZ || vco_hz > PICO_PLL_VCO_MAX_FREQ_HZ) continue;
        for (uint postdiv1 = 7; postdiv1 >= 1; postdiv1--) {
            for (uint postdiv2 = postdiv1; postdiv2 >= 1; postdiv2--) {
                uint out = vco_hz / (postdiv1 * postdiv2);
                if (out == freq_hz && !(vco_hz % (postdiv1 * postdiv2))) {
                    *vco_out = vco_hz;
                    *postdiv1_out = postdiv1;
                    *postdiv2_out = postdiv2;
                    return true;
                }
            }
        }
    }
    return false;
}

// Note this impl is kept to preserve previous rounding behavior, vs calling check_sys_clock_hz
bool check_sys_clock_khz(uint32_t freq_khz, uint *vco_out, uint *postdiv1_out, uint *postdiv2_out) {
    uint reference_freq_khz = (XOSC_HZ / KHZ) / PLL_SYS_REFDIV;
    for (uint fbdiv = 320; fbdiv >= 16; fbdiv--) {
        uint vco_khz = fbdiv * reference_freq_khz;
        if (vco_khz < PICO_PLL_VCO_MIN_FREQ_HZ / KHZ || vco_khz > PICO_PLL_VCO_MAX_FREQ_HZ / KHZ) continue;
        for (uint postdiv1 = 7; postdiv1 >= 1; postdiv1--) {
            for (uint postdiv2 = postdiv1; postdiv2 >= 1; postdiv2--) {
                uint out = vco_khz / (postdiv1 * postdiv2);
                if (out == freq_khz && !(vco_khz % (postdiv1 * postdiv2))) {
                    *vco_out = vco_khz * KHZ;
                    *postdiv1_out = postdiv1;
                    *postdiv2_out = postdiv2;
                    return true;
                }
            }
        }
    }
    return false;
}