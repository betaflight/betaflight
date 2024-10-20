/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_CLOCKS_H
#define _HARDWARE_CLOCKS_H

#include "pico.h"
#include "hardware/structs/clocks.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \file hardware/clocks.h
 *  \defgroup hardware_clocks hardware_clocks
 *
 * \brief Clock Management API
 *
 * This API provides a high level interface to the clock functions.
 *
 * The clocks block provides independent clocks to on-chip and external components. It takes inputs from a variety of clock
 * sources allowing the user to trade off performance against cost, board area and power consumption. From these sources
 * it uses multiple clock generators to provide the required clocks. This architecture allows the user flexibility to start and
 * stop clocks independently and to vary some clock frequencies whilst maintaining others at their optimum frequencies
 *
 * Please refer to the appropriate datasheet for more details on the RP-series clocks.
 *
 * The clock source depends on which clock you are attempting to configure. The first table below shows main clock sources. If
 * you are not setting the Reference clock or the System clock, or you are specifying that one of those two will be using an auxiliary
 * clock source, then you will need to use one of the entries from the subsequent tables.
 *
 * * \if rp2040_specific
 * On RP2040 the clock sources are:
 *
 * **Main Clock Sources**
 *
 * Source | Reference Clock | System Clock
 * -------|-----------------|---------
 * ROSC      | CLOCKS_CLK_REF_CTRL_SRC_VALUE_ROSC_CLKSRC_PH     |  |
 * Auxiliary | CLOCKS_CLK_REF_CTRL_SRC_VALUE_CLKSRC_CLK_REF_AUX | CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX
 * XOSC      | CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC        |  |
 * Reference |                                                  | CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF
 *
 * **Auxiliary Clock Sources**
 *
 * The auxiliary clock sources available for use in the configure function depend on which clock is being configured. The following table
 * describes the available values that can be used. Note that for clk_gpout[x], x can be 0-3.
 *
 *
 * Aux Source | clk_gpout[x] | clk_ref | clk_sys
 * -----------|------------|---------|--------
 * System PLL | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS |                                                | CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS
 * GPIO in 0  | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0   | CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0  | CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0
 * GPIO in 1  | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1   | CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1  | CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1
 * USB PLL    | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB | CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB| CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB
 * ROSC       | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_ROSC_CLKSRC    |                                                | CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_ROSC_CLKSRC
 * XOSC       | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_XOSC_CLKSRC    |                                                | CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC
 * System clock | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLK_SYS      | | |
 * USB Clock  | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLK_USB        | | |
 * ADC clock  | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLK_ADC        | | |
 * RTC Clock  | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLK_RTC        | | |
 * Ref clock  | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLK_REF        | | |
 *
 * Aux Source |  clk_peri | clk_usb | clk_adc
 * -----------|-----------|---------|--------
 * System PLL | CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS    | CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS | CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS
 * GPIO in 0  | CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0      | CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0   | CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0
 * GPIO in 1  | CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1      | CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1   | CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1
 * USB PLL    | CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB    | CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB | CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB
 * ROSC       | CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH    | CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH | CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH
 * XOSC       | CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC       | CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_XOSC_CLKSRC    | CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC
 * System clock | CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS         | | |
 *
 * Aux Source | clk_rtc
 * -----------|----------
 * System PLL |  CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS
 * GPIO in 0  |  CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0
 * GPIO in 1  |  CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1
 * USB PLL    |  CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB
 * ROSC       |  CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH
 * XOSC       |  CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC
 * \endif
 *
 * \if rp2350_specific
 * On RP2350 the clock sources are:
 * * **Main Clock Sources**
 *
 * Source | Reference Clock | System Clock
 * -------|-----------------|---------
 * ROSC      | CLOCKS_CLK_REF_CTRL_SRC_VALUE_ROSC_CLKSRC_PH     |  |
 * Auxiliary | CLOCKS_CLK_REF_CTRL_SRC_VALUE_CLKSRC_CLK_REF_AUX | CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX
 * XOSC      | CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC        |  |
 * LPOSC     | CLOCKS_CLK_REF_CTRL_SRC_VALUE_LPOSC_CLKSRC | |
 * Reference |                                                  | CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF
 *
 * **Auxiliary Clock Sources**
 *
 * The auxiliary clock sources available for use in the configure function depend on which clock is being configured. The following table
 * describes the available values that can be used. Note that for clk_gpout[x], x can be 0-3.
 *
 *
 * Aux Source | clk_gpout[x] | clk_ref | clk_sys
 * -----------|------------|---------|--------
 * System PLL | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS |                                                | CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS
 * GPIO in 0  | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0   | CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0  | CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0
 * GPIO in 1  | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1   | CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1  | CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1
 * USB PLL    | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB | CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB| CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB
 * ROSC       | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_ROSC_CLKSRC    |                                                | CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_ROSC_CLKSRC
 * XOSC       | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_XOSC_CLKSRC    |                                                | CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC
 * LPOSC      | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_LPOSC_CLKSRC | CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_LPOSC_CLKSRC | |
 * System clock | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLK_SYS      | | |
 * USB Clock  | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLK_USB        | | |
 * ADC clock  | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLK_ADC        | | |
 * REF clock  | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLK_REF        | | |
 * PERI clock  | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLK_PERI      | | |
 * HSTX clock  | CLOCKS_CLK_GPOUTx_CTRL_AUXSRC_VALUE_CLK_PERI      | | |

 *
 * Aux Source |  clk_peri | clk_hstx | clk_usb | clk_adc
 * -----------|-----------|----------|---------|--------
 * System PLL | CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS    | CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS | CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS | CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS
 * GPIO in 0  | CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0      |  | CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0   | CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0
 * GPIO in 1  | CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1      |  | CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1   | CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1
 * USB PLL    | CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB    | CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB | CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB | CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB
 * ROSC       | CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH    |  | CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH | CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH
 * XOSC       | CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC       |  | CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_XOSC_CLKSRC    | CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC
 * System clock | CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS         | CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLK_SYS | | |
 * \endif

 *
 * \section clock_example Example
 * \addtogroup hardware_clocks
 * \include hello_48MHz.c
 */

#define KHZ 1000
#define MHZ 1000000

// \tag::pll_settings[]
// There are two PLLs in RP-series microcontrollers:
// 1. The 'SYS PLL' generates the system clock, the frequency is defined by `SYS_CLK_KHZ`.
// 2. The 'USB PLL' generates the USB clock, the frequency is defined by `USB_CLK_KHZ`.
//
// The two PLLs use the crystal oscillator output directly as their reference frequency input; the PLLs reference
// frequency cannot be reduced by the dividers present in the clocks block. The crystal frequency is defined by `XOSC_HZ` (or
// `XOSC_KHZ` or `XOSC_MHZ`).
//
// The system's default definitions are correct for the above frequencies with a 12MHz
// crystal frequency.  If different frequencies are required, these must be defined in
// the board configuration file together with the revised PLL settings
// Use `vcocalc.py` to check and calculate new PLL settings if you change any of these frequencies.
//
// Default PLL configuration RP2040:
//                   REF     FBDIV VCO            POSTDIV
// PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHz / 6 / 2 = 125MHz
// PLL USB: 12 / 1 = 12MHz * 100 = 1200MHz / 5 / 5 =  48MHz
//
// Default PLL configuration RP2350:
//                   REF     FBDIV VCO            POSTDIV
// PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHz / 5 / 2 = 150MHz
// PLL USB: 12 / 1 = 12MHz * 100 = 1200MHz / 5 / 5 =  48MHz
// \end::pll_settings[]

#ifndef PLL_COMMON_REFDIV
// backwards compatibility, but now deprecated
#define PLL_COMMON_REFDIV 1
#endif

// PICO_CONFIG: PLL_SYS_REFDIV, PLL reference divider setting for PLL_SYS, type=int, default=1, advanced=true, group=hardware_clocks
#ifndef PLL_SYS_REFDIV
// backwards compatibility with deprecated PLL_COMMON_REFDIV
#ifdef PLL_COMMON_REFDIV
#define PLL_SYS_REFDIV                   PLL_COMMON_REFDIV
#else
#define PLL_SYS_REFDIV                   1
#endif
#endif

#ifndef PLL_SYS_VCO_FREQ_HZ
// For backwards compatibility define PLL_SYS_VCO_FREQ_HZ if PLL_SYS_VCO_FREQ_KHZ is defined
#ifdef PLL_SYS_VCO_FREQ_KHZ
#define PLL_SYS_VCO_FREQ_HZ                (PLL_SYS_VCO_FREQ_KHZ * KHZ)
#endif
#endif

#if (SYS_CLK_HZ == 125 * MHZ || SYS_CLK_HZ == 150 * MHZ) && (XOSC_HZ == 12 * MHZ) && (PLL_SYS_REFDIV == 1)
// PLL settings for standard 125/150 MHz system clock.
// PICO_CONFIG: PLL_SYS_VCO_FREQ_HZ, System clock PLL frequency, type=int, default=(1500 * MHZ), advanced=true, group=hardware_clocks
#ifndef PLL_SYS_VCO_FREQ_HZ
#define PLL_SYS_VCO_FREQ_HZ                (1500 * MHZ)
#endif
// PICO_CONFIG: PLL_SYS_POSTDIV1, System clock PLL post divider 1 setting, type=int, default=6 on RP2040 or 5 on RP2350, advanced=true, group=hardware_clocks
#ifndef PLL_SYS_POSTDIV1
#if SYS_CLK_HZ == 125 * MHZ
#define PLL_SYS_POSTDIV1                    6
#else
#define PLL_SYS_POSTDIV1                    5
#endif
#endif
// PICO_CONFIG: PLL_SYS_POSTDIV2, System clock PLL post divider 2 setting, type=int, default=2, advanced=true, group=hardware_clocks
#ifndef PLL_SYS_POSTDIV2
#define PLL_SYS_POSTDIV2                    2
#endif
#endif // SYS_CLK_KHZ == 125000 && XOSC_KHZ == 12000 && PLL_COMMON_REFDIV == 1

#if !defined(PLL_SYS_VCO_FREQ_HZ) || !defined(PLL_SYS_POSTDIV1) || !defined(PLL_SYS_POSTDIV2)
#error PLL_SYS_VCO_FREQ_HZ, PLL_SYS_POSTDIV1 and PLL_SYS_POSTDIV2 must all be specified when using custom clock setup
#endif

// PICO_CONFIG: PLL_USB_REFDIV, PLL reference divider setting for PLL_USB, type=int, default=1, advanced=true, group=hardware_clocks
#ifndef PLL_USB_REFDIV
// backwards compatibility with deprecated PLL_COMMON_REFDIV
#ifdef PLL_COMMON_REFDIV
#define PLL_USB_REFDIV                   PLL_COMMON_REFDIV
#else
#define PLL_USB_REFDIV                   1
#endif
#endif

#ifndef PLL_USB_VCO_FREQ_HZ
// For backwards compatibility define PLL_USB_VCO_FREQ_HZ if PLL_USB_VCO_FREQ_KHZ is defined
#ifdef PLL_USB_VCO_FREQ_KHZ
#define PLL_USB_VCO_FREQ_HZ                 (PLL_USB_VCO_FREQ_KHZ * KHZ)
#endif
#endif

#if (USB_CLK_HZ == 48 * MHZ) && (XOSC_HZ == 12 * MHZ) && (PLL_USB_REFDIV == 1)
// PLL settings for a USB clock of 48MHz.
// PICO_CONFIG: PLL_USB_VCO_FREQ_HZ, USB clock PLL frequency, type=int, default=(1200 * MHZ), advanced=true, group=hardware_clocks
#ifndef PLL_USB_VCO_FREQ_HZ
#define PLL_USB_VCO_FREQ_HZ                 (1200 * MHZ)
#endif
// PICO_CONFIG: PLL_USB_POSTDIV1, USB clock PLL post divider 1 setting, type=int, default=5, advanced=true, group=hardware_clocks
#ifndef PLL_USB_POSTDIV1
#define PLL_USB_POSTDIV1                    5
#endif
// PICO_CONFIG: PLL_USB_POSTDIV2, USB clock PLL post divider 2 setting, type=int, default=5, advanced=true, group=hardware_clocks
#ifndef PLL_USB_POSTDIV2
#define PLL_USB_POSTDIV2                    5
#endif
#endif // USB_CLK_HZ == 48000000 && XOSC_HZ == 12000000 && PLL_COMMON_REFDIV == 1
#if !defined(PLL_USB_VCO_FREQ_HZ) || !defined(PLL_USB_POSTDIV1) || !defined(PLL_USB_POSTDIV2)
#error PLL_USB_VCO_FREQ_HZ, PLL_USB_POSTDIV1 and PLL_USB_POSTDIV2 must all be specified when using custom clock setup.
#endif

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_HARDWARE_CLOCKS, Enable/disable assertions in the hardware_clocks module, type=bool, default=0, group=hardware_clocks
#ifndef PARAM_ASSERTIONS_ENABLED_HARDWARE_CLOCKS
#ifdef PARAM_ASSERTIONS_ENABLED_CLOCKS // backwards compatibility with SDK < 2.0.0
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_CLOCKS PARAM_ASSERTIONS_ENABLED_CLOCKS
#else
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_CLOCKS 0
#endif
#endif

typedef clock_num_t clock_handle_t;

/*! \brief Configure the specified clock
 *  \ingroup hardware_clocks
 *
 * See the tables in the description for details on the possible values for clock sources.
 *
 * \param clock The clock to configure
 * \param src The main clock source, can be 0.
 * \param auxsrc The auxiliary clock source, which depends on which clock is being set. Can be 0
 * \param src_freq Frequency of the input clock source
 * \param freq Requested frequency
 */
bool clock_configure(clock_handle_t clock, uint32_t src, uint32_t auxsrc, uint32_t src_freq, uint32_t freq);

/*! \brief Configure the specified clock to use the undividded input source
 *  \ingroup hardware_clocks
 *
 * See the tables in the description for details on the possible values for clock sources.
 *
 * \param clock The clock to configure
 * \param src The main clock source, can be 0.
 * \param auxsrc The auxiliary clock source, which depends on which clock is being set. Can be 0
 * \param src_freq Frequency of the input clock source
 */
void clock_configure_undivided(clock_handle_t clock, uint32_t src, uint32_t auxsrc, uint32_t src_freq);

/*! \brief Configure the specified clock to use the undividded input source
 *  \ingroup hardware_clocks
 *
 * See the tables in the description for details on the possible values for clock sources.
 *
 * \param clock The clock to configure
 * \param src The main clock source, can be 0.
 * \param auxsrc The auxiliary clock source, which depends on which clock is being set. Can be 0
 * \param src_freq Frequency of the input clock source
 * \param int_divider an integer divider
 */
void clock_configure_int_divider(clock_handle_t clock, uint32_t src, uint32_t auxsrc, uint32_t src_freq, uint32_t int_divider);

/*! \brief Stop the specified clock
 *  \ingroup hardware_clocks
 *
 * \param clock The clock to stop
 */
void clock_stop(clock_handle_t clock);

/*! \brief Get the current frequency of the specified clock
 *  \ingroup hardware_clocks
 *
 * \param clock Clock
 * \return Clock frequency in Hz
 */
uint32_t clock_get_hz(clock_handle_t clock);

/*! \brief Measure a clocks frequency using the Frequency counter.
 *  \ingroup hardware_clocks
 *
 * Uses the inbuilt frequency counter to measure the specified clocks frequency.
 * Currently, this function is accurate to +-1KHz. See the datasheet for more details.
 */
uint32_t frequency_count_khz(uint src);

/*! \brief Set the "current frequency" of the clock as reported by clock_get_hz without actually changing the clock
 *  \ingroup hardware_clocks
 *
 * \see clock_get_hz()
 */
void clock_set_reported_hz(clock_handle_t clock, uint hz);

/// \tag::frequency_count_mhz[]
static inline float frequency_count_mhz(uint src) {
    return ((float) (frequency_count_khz(src))) / KHZ;
}
/// \end::frequency_count_mhz[]

/*! \brief Resus callback function type.
 *  \ingroup hardware_clocks
 *
 * User provided callback for a resus event (when clk_sys is stopped by the programmer and is restarted for them).
 */
typedef void (*resus_callback_t)(void);

/*! \brief Enable the resus function. Restarts clk_sys if it is accidentally stopped.
 *  \ingroup hardware_clocks
 *
 * The resuscitate function will restart the system clock if it falls below a certain speed (or stops). This
 * could happen if the clock source the system clock is running from stops. For example if a PLL is stopped.
 *
 * \param resus_callback a function pointer provided by the user to call if a resus event happens.
 */
void clocks_enable_resus(resus_callback_t resus_callback);

/*! \brief Output an optionally divided clock to the specified gpio pin.
 *  \ingroup hardware_clocks
 *
 * \param gpio The GPIO pin to output the clock to. Valid GPIOs are: 21, 23, 24, 25. These GPIOs are connected to the GPOUT0-3 clock generators.
 * \param src  The source clock. See the register field CLOCKS_CLK_GPOUT0_CTRL_AUXSRC for a full list. The list is the same for each GPOUT clock generator.
 * \param div_int  The integer part of the value to divide the source clock by. This is useful to not overwhelm the GPIO pin with a fast clock. this is in range of 1..2^24-1.
 * \param div_frac The fractional part of the value to divide the source clock by. This is in range of 0..255 (/256).
 */
void clock_gpio_init_int_frac(uint gpio, uint src, uint32_t div_int, uint8_t div_frac);

/*! \brief Output an optionally divided clock to the specified gpio pin.
 *  \ingroup hardware_clocks
 *
 * \param gpio The GPIO pin to output the clock to. Valid GPIOs are: 21, 23, 24, 25. These GPIOs are connected to the GPOUT0-3 clock generators.
 * \param src  The source clock. See the register field CLOCKS_CLK_GPOUT0_CTRL_AUXSRC for a full list. The list is the same for each GPOUT clock generator.
 * \param div  The float amount to divide the source clock by. This is useful to not overwhelm the GPIO pin with a fast clock.
 */
static inline void clock_gpio_init(uint gpio, uint src, float div)
{
    uint div_int = (uint)div;
    uint8_t frac = (uint8_t)((div - (float)div_int) * (1u << CLOCKS_CLK_GPOUT0_DIV_INT_LSB));
    clock_gpio_init_int_frac(gpio, src, div_int, frac);
}

/*! \brief Configure a clock to come from a gpio input
 *  \ingroup hardware_clocks
 *
 * \param clock The clock to configure
 * \param gpio The GPIO pin to run the clock from. Valid GPIOs are: 20 and 22.
 * \param src_freq Frequency of the input clock source
 * \param freq Requested frequency
 */
bool clock_configure_gpin(clock_handle_t clock, uint gpio, uint32_t src_freq, uint32_t freq);

/*! \brief Initialise the system clock to 48MHz
 *  \ingroup hardware_clocks
 *
 *  Set the system clock to 48MHz, and set the peripheral clock to match.
 */
void set_sys_clock_48mhz(void);

/*! \brief Initialise the system clock
 *  \ingroup hardware_clocks
 *
 * \param vco_freq The voltage controller oscillator frequency to be used by the SYS PLL
 * \param post_div1 The first post divider for the SYS PLL
 * \param post_div2 The second post divider for the SYS PLL.
 *
 * See the PLL documentation in the datasheet for details of driving the PLLs.
 */
void set_sys_clock_pll(uint32_t vco_freq, uint post_div1, uint post_div2);

/*! \brief Check if a given system clock frequency is valid/attainable
 *  \ingroup hardware_clocks
 *
 * \param freq_hz Requested frequency
 * \param vco_freq_out On success, the voltage controlled oscillator frequency to be used by the SYS PLL
 * \param post_div1_out On success, The first post divider for the SYS PLL
 * \param post_div2_out On success, The second post divider for the SYS PLL.
 * @return true if the frequency is possible and the output parameters have been written.
 */
bool check_sys_clock_hz(uint32_t freq_hz, uint *vco_freq_out, uint *post_div1_out, uint *post_div2_out);

/*! \brief Check if a given system clock frequency is valid/attainable
 *  \ingroup hardware_clocks
 *
 * \param freq_khz Requested frequency
 * \param vco_freq_out On success, the voltage controlled oscillator frequency to be used by the SYS PLL
 * \param post_div1_out On success, The first post divider for the SYS PLL
 * \param post_div2_out On success, The second post divider for the SYS PLL.
 * @return true if the frequency is possible and the output parameters have been written.
 */
bool check_sys_clock_khz(uint32_t freq_khz, uint *vco_freq_out, uint *post_div1_out, uint *post_div2_out);

/*! \brief Attempt to set a system clock frequency in hz
 *  \ingroup hardware_clocks
 *
 * Note that not all clock frequencies are possible; it is preferred that you
 * use src/rp2_common/hardware_clocks/scripts/vcocalc.py to calculate the parameters
 * for use with set_sys_clock_pll
 *
 * \param freq_hz Requested frequency
 * \param required if true then this function will assert if the frequency is not attainable.
 * \return true if the clock was configured
 */
static inline bool set_sys_clock_hz(uint32_t freq_hz, bool required) {
    uint vco, postdiv1, postdiv2;
    if (check_sys_clock_hz(freq_hz, &vco, &postdiv1, &postdiv2)) {
        set_sys_clock_pll(vco, postdiv1, postdiv2);
        return true;
    } else if (required) {
        panic("System clock of %u Hz cannot be exactly achieved", freq_hz);
    }
    return false;
}

/*! \brief Attempt to set a system clock frequency in khz
 *  \ingroup hardware_clocks
 *
 * Note that not all clock frequencies are possible; it is preferred that you
 * use src/rp2_common/hardware_clocks/scripts/vcocalc.py to calculate the parameters
 * for use with set_sys_clock_pll
 *
 * \param freq_khz Requested frequency
 * \param required if true then this function will assert if the frequency is not attainable.
 * \return true if the clock was configured
 */
static inline bool set_sys_clock_khz(uint32_t freq_khz, bool required) {
    uint vco, postdiv1, postdiv2;
    if (check_sys_clock_khz(freq_khz, &vco, &postdiv1, &postdiv2)) {
        set_sys_clock_pll(vco, postdiv1, postdiv2);
        return true;
    } else if (required) {
        panic("System clock of %u kHz cannot be exactly achieved", freq_khz);
    }
    return false;
}

#ifdef __cplusplus
}
#endif

#endif
