/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_CYW43_DRIVER_H
#define _PICO_CYW43_DRIVER_H

/** \file pico/cyw43_driver.h
 *  \defgroup pico_cyw43_driver pico_cyw43_driver
 *
 * \brief A wrapper around the lower level cyw43_driver, that integrates it with \ref pico_async_context
 * for handling background work
 */

#include "pico.h"

#if CYW43_PIN_WL_DYNAMIC
#include "cyw43_configport.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct async_context;

/*! \brief Initializes the lower level cyw43_driver and integrates it with the provided async_context
 *  \ingroup pico_cyw43_driver
 *
 *  If the initialization succeeds, \ref lwip_nosys_deinit() can be called to shutdown lwIP support
 *
 * \param context the async_context instance that provides the abstraction for handling asynchronous work.
 * \return true if the initialization succeeded
*/
bool cyw43_driver_init(struct async_context *context);

/*! \brief De-initialize the lowever level cyw43_driver and unhooks it from the async_context
 *  \ingroup pico_cyw43_driver
 *
 * \param context the async_context the cyw43_driver support was added to via \ref cyw43_driver_init
*/
void cyw43_driver_deinit(struct async_context *context);

// PICO_CONFIG: CYW43_PIO_CLOCK_DIV_DYNAMIC, Enable runtime configuration of the clock divider for communication with the wireless chip, type=bool, default=0, group=pico_cyw43_driver
#ifndef CYW43_PIO_CLOCK_DIV_DYNAMIC
#define CYW43_PIO_CLOCK_DIV_DYNAMIC 0
#endif

// PICO_CONFIG: CYW43_PIO_CLOCK_DIV_INT, Integer part of the clock divider for communication with the wireless chip, type=bool, default=2, group=pico_cyw43_driver
#ifndef CYW43_PIO_CLOCK_DIV_INT
// backwards compatibility using old define
#ifdef CYW43_PIO_CLOCK_DIV
#define CYW43_PIO_CLOCK_DIV_INT CYW43_PIO_CLOCK_DIV
#else
#define CYW43_PIO_CLOCK_DIV_INT 2
#endif
#endif

// PICO_CONFIG: CYW43_PIO_CLOCK_DIV_FRAC, Fractional part of the clock divider for communication with the wireless chip, type=bool, default=0, group=pico_cyw43_driver
#ifndef CYW43_PIO_CLOCK_DIV_FRAC
#define CYW43_PIO_CLOCK_DIV_FRAC 0
#endif

#if CYW43_PIO_CLOCK_DIV_DYNAMIC
/*! \brief Set the clock divisor for the cyw43 pio clock
 *  \ingroup pico_cyw43_driver
 *
 * SPI is used to communicate with the CYW43 device and this is implemented using a PIO running with a divisor of 2.
 * If the system clock is changed it may be necessary to adjust for this to allow communications to succeed.
 * This function is only available if \ref CYW43_PIO_CLOCK_DIV_DYNAMIC is true
 *
 * \param clock_div_int Integer part of the divisor
 * \param clock_div_frac Fractional part in 1/256ths
*/
void cyw43_set_pio_clock_divisor(uint16_t clock_div_int, uint8_t clock_div_frac);
#endif

#if CYW43_PIN_WL_DYNAMIC
/*! \brief Set the gpio pins for the communication with the cyw43 device
 *  \ingroup pico_cyw43_driver
 *
 * Set or change the pins used to communicate with the cyw43 device
 * This function is only available if \ref CYW43_PIN_WL_DYNAMIC is true
 *
 * \note The cyw43 driver should not be de-initialised before this function is called or else the behaviour is undefined.
 *
 * \param pins An array containing the gpio pins to use
 * \return PICO_OK if the pin configuration could be changed and is valid
*/
int cyw43_set_pins_wl(uint pins[CYW43_PIN_INDEX_WL_COUNT]);
#endif

#ifdef __cplusplus
}
#endif
#endif