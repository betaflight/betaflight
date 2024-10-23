/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_GPIO_H
#define _HARDWARE_GPIO_H

#include "pico.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/pads_bank0.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/irq.h"

// PICO_CONFIG: PICO_USE_GPIO_COPROCESSOR, Enable/disable use of the GPIO coprocessor for GPIO access, type=bool, default=1, group=hardware_gpio
#if !defined(PICO_USE_GPIO_COPROCESSOR) && HAS_GPIO_COPROCESSOR
#define PICO_USE_GPIO_COPROCESSOR 1
#endif

#if PICO_USE_GPIO_COPROCESSOR
#include "hardware/gpio_coproc.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_HARDWARE_GPIO, Enable/disable assertions in the hardware_gpio module, type=bool, default=0, group=hardware_gpio
#ifndef PARAM_ASSERTIONS_ENABLED_HARDWARE_GPIO
#ifdef PARAM_ASSERTIONS_ENABLED_GPIO // backwards compatibility with SDK < 2.0.0
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_GPIO PARAM_ASSERTIONS_ENABLED_GPIO
#else
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_GPIO 0
#endif
#endif

/** \file gpio.h
 *  \defgroup hardware_gpio hardware_gpio
 *
 * \brief General Purpose Input/Output (GPIO) API
 *
 * RP-series microcontrollers have two banks of General Purpose Input / Output (GPIO) pins, which are assigned as follows:
 * 
 * \if rp2040-specific
 * RP2040 has 30 user GPIO pins in bank 0, and 6 QSPI pins in the QSPI bank 1 (QSPI_SS, QSPI_SCLK and QSPI_SD0 to QSPI_SD3). The QSPI 
 * pins are used to execute code from an external flash device, leaving the User bank (GPIO0 to GPIO29) for the programmer to use. 
 * \endif
 * 
 * \if rp2350-specific
 * The number of GPIO pins available depends on the package. There are 30 user GPIOs in bank 0 in the QFN-60 package (RP2350A), or 48 user GPIOs 
 * in the QFN-80 package. Bank 1 contains the 6 QSPI pins and the USB DP/DM pins.
 * \endif
 *  
 * All GPIOs support digital input and output, but a subset can also be used as inputs to the chip’s Analogue to Digital
 * Converter (ADC). The allocation of GPIO pins to the ADC depends on the packaging.
 * 
 * RP2040 and RP2350 QFN-60 GPIO, ADC pins are 26-29.
 * RP2350 QFN-80, ADC pins are 40-47.
 *  
 * Each GPIO can be controlled directly by software running on the processors, or by a number of other functional blocks.
 *
 * The function allocated to each GPIO is selected by calling the \ref gpio_set_function function. \note Not all functions
 * are available on all pins.
 *
 * Each GPIO can have one function selected at a time. Likewise, each peripheral input (e.g. UART0 RX) should only be selected on
 * one _GPIO_ at a time. If the same peripheral input is connected to multiple GPIOs, the peripheral sees the logical OR of these
 * GPIO inputs. Please refer to the datasheet for more information on GPIO function select.
 *
 * ### Function Select Table
 *
 * \if rp2040_specific
 * On RP2040 the function selects are:
 *
 * | GPIO   | F1       | F2        | F3       | F4     | F5  | F6   | F7   | F8            | F9            |
 * |--------|----------|-----------|----------|--------|-----|------|------|---------------|---------------|
 * | 0      | SPI0 RX  | UART0 TX  | I2C0 SDA | PWM0 A | SIO | PIO0 | PIO1 |               | USB OVCUR DET |
 * | 1      | SPI0 CSn | UART0 RX  | I2C0 SCL | PWM0 B | SIO | PIO0 | PIO1 |               | USB VBUS DET  |
 * | 2      | SPI0 SCK | UART0 CTS | I2C1 SDA | PWM1 A | SIO | PIO0 | PIO1 |               | USB VBUS EN   |
 * | 3      | SPI0 TX  | UART0 RTS | I2C1 SCL | PWM1 B | SIO | PIO0 | PIO1 |               | USB OVCUR DET |
 * | 4      | SPI0 RX  | UART1 TX  | I2C0 SDA | PWM2 A | SIO | PIO0 | PIO1 |               | USB VBUS DET  |
 * | 5      | SPI0 CSn | UART1 RX  | I2C0 SCL | PWM2 B | SIO | PIO0 | PIO1 |               | USB VBUS EN   |
 * | 6      | SPI0 SCK | UART1 CTS | I2C1 SDA | PWM3 A | SIO | PIO0 | PIO1 |               | USB OVCUR DET |
 * | 7      | SPI0 TX  | UART1 RTS | I2C1 SCL | PWM3 B | SIO | PIO0 | PIO1 |               | USB VBUS DET  |
 * | 8      | SPI1 RX  | UART1 TX  | I2C0 SDA | PWM4 A | SIO | PIO0 | PIO1 |               | USB VBUS EN   |
 * | 9      | SPI1 CSn | UART1 RX  | I2C0 SCL | PWM4 B | SIO | PIO0 | PIO1 |               | USB OVCUR DET |
 * | 10     | SPI1 SCK | UART1 CTS | I2C1 SDA | PWM5 A | SIO | PIO0 | PIO1 |               | USB VBUS DET  |
 * | 11     | SPI1 TX  | UART1 RTS | I2C1 SCL | PWM5 B | SIO | PIO0 | PIO1 |               | USB VBUS EN   |
 * | 12     | SPI1 RX  | UART0 TX  | I2C0 SDA | PWM6 A | SIO | PIO0 | PIO1 |               | USB OVCUR DET |
 * | 13     | SPI1 CSn | UART0 RX  | I2C0 SCL | PWM6 B | SIO | PIO0 | PIO1 |               | USB VBUS DET  |
 * | 14     | SPI1 SCK | UART0 CTS | I2C1 SDA | PWM7 A | SIO | PIO0 | PIO1 |               | USB VBUS EN   |
 * | 15     | SPI1 TX  | UART0 RTS | I2C1 SCL | PWM7 B | SIO | PIO0 | PIO1 |               | USB OVCUR DET |
 * | 16     | SPI0 RX  | UART0 TX  | I2C0 SDA | PWM0 A | SIO | PIO0 | PIO1 |               | USB VBUS DET  |
 * | 17     | SPI0 CSn | UART0 RX  | I2C0 SCL | PWM0 B | SIO | PIO0 | PIO1 |               | USB VBUS EN   |
 * | 18     | SPI0 SCK | UART0 CTS | I2C1 SDA | PWM1 A | SIO | PIO0 | PIO1 |               | USB OVCUR DET |
 * | 19     | SPI0 TX  | UART0 RTS | I2C1 SCL | PWM1 B | SIO | PIO0 | PIO1 |               | USB VBUS DET  |
 * | 20     | SPI0 RX  | UART1 TX  | I2C0 SDA | PWM2 A | SIO | PIO0 | PIO1 | CLOCK GPIN0   | USB VBUS EN   |
 * | 21     | SPI0 CSn | UART1 RX  | I2C0 SCL | PWM2 B | SIO | PIO0 | PIO1 | CLOCK GPOUT0  | USB OVCUR DET |
 * | 22     | SPI0 SCK | UART1 CTS | I2C1 SDA | PWM3 A | SIO | PIO0 | PIO1 | CLOCK GPIN1   | USB VBUS DET  |
 * | 23     | SPI0 TX  | UART1 RTS | I2C1 SCL | PWM3 B | SIO | PIO0 | PIO1 | CLOCK GPOUT1  | USB VBUS EN   |
 * | 24     | SPI1 RX  | UART1 TX  | I2C0 SDA | PWM4 A | SIO | PIO0 | PIO1 | CLOCK GPOUT2  | USB OVCUR DET |
 * | 25     | SPI1 CSn | UART1 RX  | I2C0 SCL | PWM4 B | SIO | PIO0 | PIO1 | CLOCK GPOUT3  | USB VBUS DET  |
 * | 26     | SPI1 SCK | UART1 CTS | I2C1 SDA | PWM5 A | SIO | PIO0 | PIO1 |               | USB VBUS EN   |
 * | 27     | SPI1 TX  | UART1 RTS | I2C1 SCL | PWM5 B | SIO | PIO0 | PIO1 |               | USB OVCUR DET |
 * | 28     | SPI1 RX  | UART0 TX  | I2C0 SDA | PWM6 A | SIO | PIO0 | PIO1 |               | USB VBUS DET  |
 * | 29     | SPI1 CSn | UART0 RX  | I2C0 SCL | PWM6 B | SIO | PIO0 | PIO1 |               | USB VBUS EN   |
 * \endif
 * \if rp2350_specific
 * On RP2350 the function selects are:
 *
 * | GPIO  | F0   | F1       | F2        | F3       | F4     | F5  | F6   | F7   | F8   | F9           | F10           | F11      |
 * |-------|------|----------|-----------|----------|--------|-----|------|------|------|--------------|---------------|----------|
 * | 0     |      | SPI0 RX  | UART0 TX  | I2C0 SDA | PWM0 A | SIO | PIO0 | PIO1 | PIO2 | XIP_CS1n     | USB OVCUR DET |          |
 * | 1     |      | SPI0 CSn | UART0 RX  | I2C0 SCL | PWM0 B | SIO | PIO0 | PIO1 | PIO2 | TRACECLK     | USB VBUS DET  |          |
 * | 2     |      | SPI0 SCK | UART0 CTS | I2C1 SDA | PWM1 A | SIO | PIO0 | PIO1 | PIO2 | TRACEDATA0   | USB VBUS EN   | UART0 TX |
 * | 3     |      | SPI0 TX  | UART0 RTS | I2C1 SCL | PWM1 B | SIO | PIO0 | PIO1 | PIO2 | TRACEDATA1   | USB OVCUR DET | UART0 RX |
 * | 4     |      | SPI0 RX  | UART1 TX  | I2C0 SDA | PWM2 A | SIO | PIO0 | PIO1 | PIO2 | TRACEDATA2   | USB VBUS DET  |          |
 * | 5     |      | SPI0 CSn | UART1 RX  | I2C0 SCL | PWM2 B | SIO | PIO0 | PIO1 | PIO2 | TRACEDATA3   | USB VBUS EN   |          |
 * | 6     |      | SPI0 SCK | UART1 CTS | I2C1 SDA | PWM3 A | SIO | PIO0 | PIO1 | PIO2 |              | USB OVCUR DET | UART1 TX |
 * | 7     |      | SPI0 TX  | UART1 RTS | I2C1 SCL | PWM3 B | SIO | PIO0 | PIO1 | PIO2 |              | USB VBUS DET  | UART1 RX |
 * | 8     |      | SPI1 RX  | UART1 TX  | I2C0 SDA | PWM4 A | SIO | PIO0 | PIO1 | PIO2 | XIP_CS1n     | USB VBUS EN   |          |
 * | 9     |      | SPI1 CSn | UART1 RX  | I2C0 SCL | PWM4 B | SIO | PIO0 | PIO1 | PIO2 |              | USB OVCUR DET |          |
 * | 10    |      | SPI1 SCK | UART1 CTS | I2C1 SDA | PWM5 A | SIO | PIO0 | PIO1 | PIO2 |              | USB VBUS DET  | UART1 TX |
 * | 11    |      | SPI1 TX  | UART1 RTS | I2C1 SCL | PWM5 B | SIO | PIO0 | PIO1 | PIO2 |              | USB VBUS EN   | UART1 RX |
 * | 12    | HSTX | SPI1 RX  | UART0 TX  | I2C0 SDA | PWM6 A | SIO | PIO0 | PIO1 | PIO2 | CLOCK GPIN0  | USB OVCUR DET |          |
 * | 13    | HSTX | SPI1 CSn | UART0 RX  | I2C0 SCL | PWM6 B | SIO | PIO0 | PIO1 | PIO2 | CLOCK GPOUT0 | USB VBUS DET  |          |
 * | 14    | HSTX | SPI1 SCK | UART0 CTS | I2C1 SDA | PWM7 A | SIO | PIO0 | PIO1 | PIO2 | CLOCK GPIN1  | USB VBUS EN   | UART0 TX |
 * | 15    | HSTX | SPI1 TX  | UART0 RTS | I2C1 SCL | PWM7 B | SIO | PIO0 | PIO1 | PIO2 | CLOCK GPOUT1 | USB OVCUR DET | UART0 RX |
 * | 16    | HSTX | SPI0 RX  | UART0 TX  | I2C0 SDA | PWM0 A | SIO | PIO0 | PIO1 | PIO2 |              | USB VBUS DET  |          |
 * | 17    | HSTX | SPI0 CSn | UART0 RX  | I2C0 SCL | PWM0 B | SIO | PIO0 | PIO1 | PIO2 |              | USB VBUS EN   |          |
 * | 18    | HSTX | SPI0 SCK | UART0 CTS | I2C1 SDA | PWM1 A | SIO | PIO0 | PIO1 | PIO2 |              | USB OVCUR DET | UART0 TX |
 * | 19    | HSTX | SPI0 TX  | UART0 RTS | I2C1 SCL | PWM1 B | SIO | PIO0 | PIO1 | PIO2 | XIP_CS1n     | USB VBUS DET  | UART0 RX |
 * | 20    |      | SPI0 RX  | UART1 TX  | I2C0 SDA | PWM2 A | SIO | PIO0 | PIO1 | PIO2 | CLOCK GPIN0  | USB VBUS EN   |          |
 * | 21    |      | SPI0 CSn | UART1 RX  | I2C0 SCL | PWM2 B | SIO | PIO0 | PIO1 | PIO2 | CLOCK GPOUT0 | USB OVCUR DET |          |
 * | 22    |      | SPI0 SCK | UART1 CTS | I2C1 SDA | PWM3 A | SIO | PIO0 | PIO1 | PIO2 | CLOCK GPIN1  | USB VBUS DET  | UART1 TX |
 * | 23    |      | SPI0 TX  | UART1 RTS | I2C1 SCL | PWM3 B | SIO | PIO0 | PIO1 | PIO2 | CLOCK GPOUT1 | USB VBUS EN   | UART1 RX |
 * | 24    |      | SPI1 RX  | UART1 TX  | I2C0 SDA | PWM4 A | SIO | PIO0 | PIO1 | PIO2 | CLOCK GPOUT2 | USB OVCUR DET |          |
 * | 25    |      | SPI1 CSn | UART1 RX  | I2C0 SCL | PWM4 B | SIO | PIO0 | PIO1 | PIO2 | CLOCK GPOUT3 | USB VBUS DET  |          |
 * | 26    |      | SPI1 SCK | UART1 CTS | I2C1 SDA | PWM5 A | SIO | PIO0 | PIO1 | PIO2 |              | USB VBUS EN   | UART1 TX |
 * | 27    |      | SPI1 TX  | UART1 RTS | I2C1 SCL | PWM5 B | SIO | PIO0 | PIO1 | PIO2 |              | USB OVCUR DET | UART1 RX |
 * | 28    |      | SPI1 RX  | UART0 TX  | I2C0 SDA | PWM6 A | SIO | PIO0 | PIO1 | PIO2 |              | USB VBUS DET  |          |
 * | 29    |      | SPI1 CSn | UART0 RX  | I2C0 SCL | PWM6 B | SIO | PIO0 | PIO1 | PIO2 |              | USB VBUS EN   |          |
 *
 * GPIOs 30 through 47 are QFN-80 only:
 *
 * | GPIO | F0 | F1       | F2       | F3        | F4      | F5  | F6   | F7   | F8   | F9       | F10           | F11      |
 * |------|----|----------|----------|-----------|---------|-----|------|------|------|----------|---------------|----------|
 * | 30   |    | SPI1 SCK | UART0 CTS | I2C1 SDA | PWM7 A  | SIO | PIO0 | PIO1 | PIO2 |          | USB OVCUR DET | UART0 TX |
 * | 31   |    | SPI1 TX  | UART0 RTS | I2C1 SCL | PWM7 B  | SIO | PIO0 | PIO1 | PIO2 |          | USB VBUS DET  | UART0 RX |
 * | 32   |    | SPI0 RX  | UART0 TX  | I2C0 SDA | PWM8 A  | SIO | PIO0 | PIO1 | PIO2 |          | USB VBUS EN   |          |
 * | 33   |    | SPI0 CSn | UART0 RX  | I2C0 SCL | PWM8 B  | SIO | PIO0 | PIO1 | PIO2 |          | USB OVCUR DET |          |
 * | 34   |    | SPI0 SCK | UART0 CTS | I2C1 SDA | PWM9 A  | SIO | PIO0 | PIO1 | PIO2 |          | USB VBUS DET  | UART0 TX |
 * | 35   |    | SPI0 TX  | UART0 RTS | I2C1 SCL | PWM9 B  | SIO | PIO0 | PIO1 | PIO2 |          | USB VBUS EN   | UART0 RX |
 * | 36   |    | SPI0 RX  | UART1 TX  | I2C0 SDA | PWM10 A | SIO | PIO0 | PIO1 | PIO2 |          | USB OVCUR DET |          |
 * | 37   |    | SPI0 CSn | UART1 RX  | I2C0 SCL | PWM10 B | SIO | PIO0 | PIO1 | PIO2 |          | USB VBUS DET  |          |
 * | 38   |    | SPI0 SCK | UART1 CTS | I2C1 SDA | PWM11 A | SIO | PIO0 | PIO1 | PIO2 |          | USB VBUS EN   | UART1 TX |
 * | 39   |    | SPI0 TX  | UART1 RTS | I2C1 SCL | PWM11 B | SIO | PIO0 | PIO1 | PIO2 |          | USB OVCUR DET | UART1 RX |
 * | 40   |    | SPI1 RX  | UART1 TX  | I2C0 SDA | PWM8 A  | SIO | PIO0 | PIO1 | PIO2 |          | USB VBUS DET  |          |
 * | 41   |    | SPI1 CSn | UART1 RX  | I2C0 SCL | PWM8 B  | SIO | PIO0 | PIO1 | PIO2 |          | USB VBUS EN   |          |
 * | 42   |    | SPI1 SCK | UART1 CTS | I2C1 SDA | PWM9 A  | SIO | PIO0 | PIO1 | PIO2 |          | USB OVCUR DET | UART1 TX |
 * | 43   |    | SPI1 TX  | UART1 RTS | I2C1 SCL | PWM9 B  | SIO | PIO0 | PIO1 | PIO2 |          | USB VBUS DET  | UART1 RX |
 * | 44   |    | SPI1 RX  | UART0 TX  | I2C0 SDA | PWM10 A | SIO | PIO0 | PIO1 | PIO2 |          | USB VBUS EN   |          |
 * | 45   |    | SPI1 CSn | UART0 RX  | I2C0 SCL | PWM10 B | SIO | PIO0 | PIO1 | PIO2 |          | USB OVCUR DET |          |
 * | 46   |    | SPI1 SCK | UART0 CTS | I2C1 SDA | PWM11 A | SIO | PIO0 | PIO1 | PIO2 |          | USB VBUS DET  | UART0 TX |
 * | 47   |    | SPI1 TX  | UART0 RTS | I2C1 SCL | PWM11 B | SIO | PIO0 | PIO1 | PIO2 | XIP_CS1n | USB VBUS EN   | UART0 RX |
 *
 * \endif
 */

enum gpio_dir {
    GPIO_OUT = 1u, ///< set GPIO to output
    GPIO_IN = 0u,  ///< set GPIO to input
};

/*! \brief  GPIO Interrupt level definitions (GPIO events)
 *  \ingroup hardware_gpio
 *  \brief GPIO Interrupt levels
 *
 * An interrupt can be generated for every GPIO pin in 4 scenarios:
 *
 * * Level High: the GPIO pin is a logical 1
 * * Level Low: the GPIO pin is a logical 0
 * * Edge High: the GPIO has transitioned from a logical 0 to a logical 1
 * * Edge Low: the GPIO has transitioned from a logical 1 to a logical 0
 *
 * The level interrupts are not latched. This means that if the pin is a logical 1 and the level high interrupt is active, it will
 * become inactive as soon as the pin changes to a logical 0. The edge interrupts are stored in the INTR register and can be
 * cleared by writing to the INTR register.
 */
enum gpio_irq_level {
    GPIO_IRQ_LEVEL_LOW = 0x1u,  ///< IRQ when the GPIO pin is a logical 0
    GPIO_IRQ_LEVEL_HIGH = 0x2u, ///< IRQ when the GPIO pin is a logical 1
    GPIO_IRQ_EDGE_FALL = 0x4u,  ///< IRQ when the GPIO has transitioned from a logical 1 to a logical 0
    GPIO_IRQ_EDGE_RISE = 0x8u,  ///< IRQ when the GPIO has transitioned from a logical 0 to a logical 1
};

/*! Callback function type for GPIO events
 *  \ingroup hardware_gpio
 *
 * \param gpio Which GPIO caused this interrupt
 * \param event_mask Which events caused this interrupt. See \ref gpio_irq_level for details.
 * \sa gpio_set_irq_enabled_with_callback()
 * \sa gpio_set_irq_callback()
 */
typedef void (*gpio_irq_callback_t)(uint gpio, uint32_t event_mask);

enum gpio_override {
    GPIO_OVERRIDE_NORMAL = 0,      ///< peripheral signal selected via \ref gpio_set_function
    GPIO_OVERRIDE_INVERT = 1,      ///< invert peripheral signal selected via \ref gpio_set_function
    GPIO_OVERRIDE_LOW = 2,         ///< drive low/disable output
    GPIO_OVERRIDE_HIGH = 3,        ///< drive high/enable output
};

/*! \brief Slew rate limiting levels for GPIO outputs
 *  \ingroup hardware_gpio
 *
 * Slew rate limiting increases the minimum rise/fall time when a GPIO output
 * is lightly loaded, which can help to reduce electromagnetic emissions.
 * \sa gpio_set_slew_rate
 */
enum gpio_slew_rate {
    GPIO_SLEW_RATE_SLOW = 0,  ///< Slew rate limiting enabled
    GPIO_SLEW_RATE_FAST = 1   ///< Slew rate limiting disabled
};

/*! \brief Drive strength levels for GPIO outputs
 *  \ingroup hardware_gpio
 *
 * Drive strength levels for GPIO outputs.
 * \sa gpio_set_drive_strength
 */
enum gpio_drive_strength {
    GPIO_DRIVE_STRENGTH_2MA = 0, ///< 2 mA nominal drive strength
    GPIO_DRIVE_STRENGTH_4MA = 1, ///< 4 mA nominal drive strength
    GPIO_DRIVE_STRENGTH_8MA = 2, ///< 8 mA nominal drive strength
    GPIO_DRIVE_STRENGTH_12MA = 3 ///< 12 mA nominal drive strength
};

static inline void check_gpio_param(__unused uint gpio) {
    invalid_params_if(HARDWARE_GPIO, gpio >= NUM_BANK0_GPIOS);
}

// ----------------------------------------------------------------------------
// Pad Controls + IO Muxing
// ----------------------------------------------------------------------------
// Declarations for gpio.c

/*! \brief Select GPIO function
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \param fn Which GPIO function select to use from list \ref gpio_function_t
 */
void gpio_set_function(uint gpio, gpio_function_t fn);

/*! \brief Select the function for multiple GPIOs
 *  \ingroup hardware_gpio
 *
 * \sa gpio_set_function
 * \param gpio_mask Mask with 1 bit per GPIO number to set the function for
 * \param fn Which GPIO function select to use from list \ref gpio_function_t
*/
void gpio_set_function_masked(uint32_t gpio_mask, gpio_function_t fn);

/*! \brief Select the function for multiple GPIOs
 *  \ingroup hardware_gpio
 *
 * \sa gpio_set_function
 * \param gpio_mask Mask with 1 bit per GPIO number to set the function for
 * \param fn Which GPIO function select to use from list \ref gpio_function_t
*/
void gpio_set_function_masked64(uint64_t gpio_mask, gpio_function_t fn);

/*! \brief Determine current GPIO function
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return Which GPIO function is currently selected from list \ref gpio_function_t
 */
gpio_function_t gpio_get_function(uint gpio);

/*! \brief Select up and down pulls on specific GPIO
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \param up If true set a pull up on the GPIO
 * \param down If true set a pull down on the GPIO
 *
 * \note On the RP2040, setting both pulls enables a "bus keep" function,
 * i.e. a weak pull to whatever is current high/low state of GPIO.
 */
void gpio_set_pulls(uint gpio, bool up, bool down);

/*! \brief Set specified GPIO to be pulled up.
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 */
static inline void gpio_pull_up(uint gpio) {
    gpio_set_pulls(gpio, true, false);
}

/*! \brief Determine if the specified GPIO is pulled up.
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return true if the GPIO is pulled up
 */
static inline bool gpio_is_pulled_up(uint gpio) {
    return (pads_bank0_hw->io[gpio] & PADS_BANK0_GPIO0_PUE_BITS) != 0;
}

/*! \brief Set specified GPIO to be pulled down.
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 */
static inline void gpio_pull_down(uint gpio) {
    gpio_set_pulls(gpio, false, true);
}

/*! \brief Determine if the specified GPIO is pulled down.
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return true if the GPIO is pulled down
 */
static inline bool gpio_is_pulled_down(uint gpio) {
    return (pads_bank0_hw->io[gpio] & PADS_BANK0_GPIO0_PDE_BITS) != 0;
}

/*! \brief Disable pulls on specified GPIO
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 */
static inline void gpio_disable_pulls(uint gpio) {
    gpio_set_pulls(gpio, false, false);
}

/*! \brief Set GPIO IRQ override
 *  \ingroup hardware_gpio
 *
 * Optionally invert a GPIO IRQ signal, or drive it high or low
 *
 * \param gpio GPIO number
 * \param value See \ref gpio_override
 */
void gpio_set_irqover(uint gpio, uint value);

/*! \brief Set GPIO output override
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \param value See \ref gpio_override
 */
void gpio_set_outover(uint gpio, uint value);

/*! \brief Select GPIO input override
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \param value See \ref gpio_override
 */
void gpio_set_inover(uint gpio, uint value);

/*! \brief Select GPIO output enable override
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \param value See \ref gpio_override
 */
void gpio_set_oeover(uint gpio, uint value);

/*! \brief Enable GPIO input
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \param enabled true to enable input on specified GPIO
 */
void gpio_set_input_enabled(uint gpio, bool enabled);

/*! \brief Enable/disable GPIO input hysteresis (Schmitt trigger)
 *  \ingroup hardware_gpio
 *
 * Enable or disable the Schmitt trigger hysteresis on a given GPIO. This is
 * enabled on all GPIOs by default. Disabling input hysteresis can lead to
 * inconsistent readings when the input signal has very long rise or fall
 * times, but slightly reduces the GPIO's input delay.
 *
 * \sa gpio_is_input_hysteresis_enabled
 * \param gpio GPIO number
 * \param enabled true to enable input hysteresis on specified GPIO
 */
void gpio_set_input_hysteresis_enabled(uint gpio, bool enabled);

/*! \brief Determine whether input hysteresis is enabled on a specified GPIO
 *  \ingroup hardware_gpio
 *
 * \sa gpio_set_input_hysteresis_enabled
 * \param gpio GPIO number
 */
bool gpio_is_input_hysteresis_enabled(uint gpio);

/*! \brief Set slew rate for a specified GPIO
 *  \ingroup hardware_gpio
 *
 * \sa gpio_get_slew_rate
 * \param gpio GPIO number
 * \param slew GPIO output slew rate
 */
void gpio_set_slew_rate(uint gpio, enum gpio_slew_rate slew);

/*! \brief Determine current slew rate for a specified GPIO
 *  \ingroup hardware_gpio
 *
 * \sa gpio_set_slew_rate
 * \param gpio GPIO number
 * \return Current slew rate of that GPIO
 */
enum gpio_slew_rate gpio_get_slew_rate(uint gpio);

/*! \brief Set drive strength for a specified GPIO
 *  \ingroup hardware_gpio
 *
 * \sa gpio_get_drive_strength
 * \param gpio GPIO number
 * \param drive GPIO output drive strength
 */
void gpio_set_drive_strength(uint gpio, enum gpio_drive_strength drive);

/*! \brief Determine current drive strength for a specified GPIO
 *  \ingroup hardware_gpio
 *
 * \sa gpio_set_drive_strength
 * \param gpio GPIO number
 * \return Current drive strength of that GPIO
 */
enum gpio_drive_strength gpio_get_drive_strength(uint gpio);

/*! \brief Enable or disable specific interrupt events for specified GPIO
 *  \ingroup hardware_gpio
 *
 * This function sets which GPIO events cause a GPIO interrupt on the calling core. See
 * \ref gpio_set_irq_callback, \ref gpio_set_irq_enabled_with_callback and
 * \ref gpio_add_raw_irq_handler to set up a GPIO interrupt handler to handle the events.
 *
 * \note The IO IRQs are independent per-processor. This configures the interrupt events for
 * the processor that calls the function.
 *
 * \param gpio GPIO number
 * \param event_mask Which events will cause an interrupt
 * \param enabled Enable or disable flag
 *
 * Events is a bitmask of the following \ref gpio_irq_level values:
 *
 * bit | constant            | interrupt
 * ----|---------------------|------------------------------------
 *   0 | GPIO_IRQ_LEVEL_LOW  | Continuously while level is low
 *   1 | GPIO_IRQ_LEVEL_HIGH | Continuously while level is high
 *   2 | GPIO_IRQ_EDGE_FALL  | On each transition from high to low
 *   3 | GPIO_IRQ_EDGE_RISE  | On each transition from low to high
 *
 * which are specified in \ref gpio_irq_level
 */
void gpio_set_irq_enabled(uint gpio, uint32_t event_mask, bool enabled);

// PICO_CONFIG: GPIO_IRQ_CALLBACK_ORDER_PRIORITY, IRQ priority order of the default IRQ callback, min=0, max=255, default=PICO_SHARED_IRQ_HANDLER_LOWEST_ORDER_PRIORITY, group=hardware_gpio
#ifndef GPIO_IRQ_CALLBACK_ORDER_PRIORITY
#define GPIO_IRQ_CALLBACK_ORDER_PRIORITY PICO_SHARED_IRQ_HANDLER_LOWEST_ORDER_PRIORITY
#endif

// PICO_CONFIG: GPIO_RAW_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY, IRQ priority order of raw IRQ handlers if the priority is not specified, min=0, max=255, default=PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY, group=hardware_gpio
#ifndef GPIO_RAW_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY
#define GPIO_RAW_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY
#endif

/*! \brief Set the generic callback used for GPIO IRQ events for the current core
 *  \ingroup hardware_gpio
 *
 * This function sets the callback used for all GPIO IRQs on the current core that are not explicitly
 * hooked via \ref gpio_add_raw_irq_handler or other gpio_add_raw_irq_handler_ functions.
 *
 * This function is called with the GPIO number and event mask for each of the (not explicitly hooked)
 * GPIOs that have events enabled and that are pending (see \ref gpio_get_irq_event_mask).
 *
 * \note The IO IRQs are independent per-processor. This function affects
 * the processor that calls the function.
 *
 * \param callback default user function to call on GPIO irq. Note only one of these can be set per processor.
 */
void gpio_set_irq_callback(gpio_irq_callback_t callback);

/*! \brief Convenience function which performs multiple GPIO IRQ related initializations
 *  \ingroup hardware_gpio
 *
 * This method is a slightly eclectic mix of initialization, that:
 *
 * \li Updates whether the specified events for the specified GPIO causes an interrupt on the calling core based
 * on the enable flag.
 *
 * \li Sets the callback handler for the calling core to callback (or clears the handler if the callback is NULL).
 *
 * \li Enables GPIO IRQs on the current core if enabled is true.
 *
 * This method is commonly used to perform a one time setup, and following that any additional IRQs/events are enabled
 * via \ref gpio_set_irq_enabled. All GPIOs/events added in this way on the same core share the same callback; for multiple
 * independent handlers for different GPIOs you should use \ref gpio_add_raw_irq_handler and related functions.
 *
 * This method is equivalent to:
 *
 * \code{.c}
 * gpio_set_irq_enabled(gpio, event_mask, enabled);
 * gpio_set_irq_callback(callback);
 * if (enabled) irq_set_enabled(IO_IRQ_BANK0, true);
 * \endcode
 *
 * \note The IO IRQs are independent per-processor. This method affects only the processor that calls the function.
 *
 * \param gpio GPIO number
 * \param event_mask Which events will cause an interrupt. See \ref gpio_irq_level for details.
 * \param enabled Enable or disable flag
 * \param callback user function to call on GPIO irq. if NULL, the callback is removed
 */
void gpio_set_irq_enabled_with_callback(uint gpio, uint32_t event_mask, bool enabled, gpio_irq_callback_t callback);

/*! \brief Enable dormant wake up interrupt for specified GPIO and events
 *  \ingroup hardware_gpio
 *
 * This configures IRQs to restart the XOSC or ROSC when they are
 * disabled in dormant mode
 *
 * \param gpio GPIO number
 * \param event_mask Which events will cause an interrupt. See \ref gpio_irq_level for details.
 * \param enabled Enable/disable flag
 */
void gpio_set_dormant_irq_enabled(uint gpio, uint32_t event_mask, bool enabled);

/*! \brief Return the current interrupt status (pending events) for the given GPIO
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return Bitmask of events that are currently pending for the GPIO. See \ref gpio_irq_level for details.
 * \sa gpio_acknowledge_irq
 */
static inline uint32_t gpio_get_irq_event_mask(uint gpio) {
    check_gpio_param(gpio);
    io_bank0_irq_ctrl_hw_t *irq_ctrl_base = get_core_num() ?
                                            &io_bank0_hw->proc1_irq_ctrl : &io_bank0_hw->proc0_irq_ctrl;
    io_ro_32 *status_reg = &irq_ctrl_base->ints[gpio >> 3u];
    return (*status_reg >> (4 * (gpio & 7u))) & 0xfu;
}

/*! \brief Acknowledge a GPIO interrupt for the specified events on the calling core
 *  \ingroup hardware_gpio
 *
 * \note This may be called with a mask of any of valid bits specified in \ref gpio_irq_level, however
 * it has no effect on \a level sensitive interrupts which remain pending while the GPIO is at the specified
 * level. When handling \a level sensitive interrupts, you should generally disable the interrupt (see
 * \ref gpio_set_irq_enabled) and then set it up again later once the GPIO level has changed (or to catch
 * the opposite level).
 *
 * \param gpio GPIO number
 *
 * \note For callbacks set with \ref gpio_set_irq_enabled_with_callback, or \ref gpio_set_irq_callback, this function is called automatically.
 * \param event_mask Bitmask of events to clear. See \ref gpio_irq_level for details.
 */
void gpio_acknowledge_irq(uint gpio, uint32_t event_mask);

/*! \brief Adds a raw GPIO IRQ handler for the specified GPIOs on the current core
 *  \ingroup hardware_gpio
 *
 * In addition to the default mechanism of a single GPIO IRQ event callback per core (see \ref gpio_set_irq_callback),
 * it is possible to add explicit GPIO IRQ handlers which are called independent of the default callback. The order
 * relative to the default callback can be controlled via the order_priority parameter (the default callback has the priority
 * \ref GPIO_IRQ_CALLBACK_ORDER_PRIORITY which defaults to the lowest priority with the intention of it running last).
 *
 * This method adds such an explicit GPIO IRQ handler, and disables the "default" callback for the specified GPIOs.
 *
 * \note Multiple raw handlers should not be added for the same GPIOs, and this method will assert if you attempt to.
 * Internally, this function calls \ref irq_add_shared_handler, which will assert if the maximum number of shared handlers
 * (configurable via PICO_MAX_IRQ_SHARED_HANDLERS) would be exceeded.
 *
 * A raw handler should check for whichever GPIOs and events it handles, and acknowledge them itself; it might look something like:
 *
 * \code{.c}
 * void my_irq_handler(void) {
 *     if (gpio_get_irq_event_mask(my_gpio_num) & my_gpio_event_mask) {
 *        gpio_acknowledge_irq(my_gpio_num, my_gpio_event_mask);
 *       // handle the IRQ
 *     }
 *     if (gpio_get_irq_event_mask(my_gpio_num2) & my_gpio_event_mask2) {
 *        gpio_acknowledge_irq(my_gpio_num2, my_gpio_event_mask2);
 *       // handle the IRQ
 *     }
 * }
 * \endcode
 *
 * @param gpio_mask a bit mask of the GPIO numbers that will no longer be passed to the default callback for this core
 * @param handler the handler to add to the list of GPIO IRQ handlers for this core
 * @param order_priority the priority order to determine the relative position of the handler in the list of GPIO IRQ handlers for this core.
 */
void gpio_add_raw_irq_handler_with_order_priority_masked(uint32_t gpio_mask, irq_handler_t handler, uint8_t order_priority);

/*! \brief Adds a raw GPIO IRQ handler for the specified GPIOs on the current core
 *  \ingroup hardware_gpio
 *
 * In addition to the default mechanism of a single GPIO IRQ event callback per core (see \ref gpio_set_irq_callback),
 * it is possible to add explicit GPIO IRQ handlers which are called independent of the default callback. The order
 * relative to the default callback can be controlled via the order_priority parameter (the default callback has the priority
 * \ref GPIO_IRQ_CALLBACK_ORDER_PRIORITY which defaults to the lowest priority with the intention of it running last).
 *
 * This method adds such an explicit GPIO IRQ handler, and disables the "default" callback for the specified GPIOs.
 *
 * \note Multiple raw handlers should not be added for the same GPIOs, and this method will assert if you attempt to.
 * Internally, this function calls \ref irq_add_shared_handler, which will assert if the maximum number of shared handlers
 * (configurable via PICO_MAX_IRQ_SHARED_HANDLERS) would be exceeded.
 *
 * A raw handler should check for whichever GPIOs and events it handles, and acknowledge them itself; it might look something like:
 *
 * \code{.c}
 * void my_irq_handler(void) {
 *     if (gpio_get_irq_event_mask(my_gpio_num) & my_gpio_event_mask) {
 *        gpio_acknowledge_irq(my_gpio_num, my_gpio_event_mask);
 *       // handle the IRQ
 *     }
 *     if (gpio_get_irq_event_mask(my_gpio_num2) & my_gpio_event_mask2) {
 *        gpio_acknowledge_irq(my_gpio_num2, my_gpio_event_mask2);
 *       // handle the IRQ
 *     }
 * }
 * \endcode
 *
 * @param gpio_mask a bit mask of the GPIO numbers that will no longer be passed to the default callback for this core
 * @param handler the handler to add to the list of GPIO IRQ handlers for this core
 * @param order_priority the priority order to determine the relative position of the handler in the list of GPIO IRQ handlers for this core.
 */
void gpio_add_raw_irq_handler_with_order_priority_masked64(uint64_t gpio_mask, irq_handler_t handler, uint8_t order_priority);

/*! \brief Adds a raw GPIO IRQ handler for a specific GPIO on the current core
 *  \ingroup hardware_gpio
 *
 * In addition to the default mechanism of a single GPIO IRQ event callback per core (see \ref gpio_set_irq_callback),
 * it is possible to add explicit GPIO IRQ handlers which are called independent of the default callback. The order
 * relative to the default callback can be controlled via the order_priority parameter(the default callback has the priority
 * \ref GPIO_IRQ_CALLBACK_ORDER_PRIORITY which defaults to the lowest priority with the intention of it running last).
 *
 * This method adds such a callback, and disables the "default" callback for the specified GPIO.
 *
 * \note Multiple raw handlers should not be added for the same GPIO, and this method will assert if you attempt to.
 * Internally, this function calls \ref irq_add_shared_handler, which will assert if the maximum number of shared handlers
 * (configurable via PICO_MAX_IRQ_SHARED_HANDLERS) would be exceeded.
 *
 * A raw handler should check for whichever GPIOs and events it handles, and acknowledge them itself; it might look something like:
 *
 * \code{.c}
 * void my_irq_handler(void) {
 *     if (gpio_get_irq_event_mask(my_gpio_num) & my_gpio_event_mask) {
 *        gpio_acknowledge_irq(my_gpio_num, my_gpio_event_mask);
 *       // handle the IRQ
 *     }
 * }
 * \endcode
 *
 * @param gpio the GPIO number that will no longer be passed to the default callback for this core
 * @param handler the handler to add to the list of GPIO IRQ handlers for this core
 * @param order_priority the priority order to determine the relative position of the handler in the list of GPIO IRQ handlers for this core.
 */
static inline void gpio_add_raw_irq_handler_with_order_priority(uint gpio, irq_handler_t handler, uint8_t order_priority) {
    check_gpio_param(gpio);
#if NUM_BANK0_GPIOS > 32
    gpio_add_raw_irq_handler_with_order_priority_masked64(1ull << gpio, handler, order_priority);
#else
    gpio_add_raw_irq_handler_with_order_priority_masked(1u << gpio, handler, order_priority);
#endif
}

/*! \brief Adds a raw GPIO IRQ handler for the specified GPIOs on the current core
 *  \ingroup hardware_gpio
 *
 * In addition to the default mechanism of a single GPIO IRQ event callback per core (see \ref gpio_set_irq_callback),
 * it is possible to add explicit GPIO IRQ handlers which are called independent of the default event callback.
 *
 * This method adds such a callback, and disables the "default" callback for the specified GPIOs.
 *
 * \note Multiple raw handlers should not be added for the same GPIOs, and this method will assert if you attempt to.
 * Internally, this function calls \ref irq_add_shared_handler, which will assert if the maximum number of shared handlers
 * (configurable via PICO_MAX_IRQ_SHARED_HANDLERS) would be exceeded.
 *
 * A raw handler should check for whichever GPIOs and events it handles, and acknowledge them itself; it might look something like:
 *
 * \code{.c}
 * void my_irq_handler(void) {
 *     if (gpio_get_irq_event_mask(my_gpio_num) & my_gpio_event_mask) {
 *        gpio_acknowledge_irq(my_gpio_num, my_gpio_event_mask);
 *       // handle the IRQ
 *     }
 *     if (gpio_get_irq_event_mask(my_gpio_num2) & my_gpio_event_mask2) {
 *        gpio_acknowledge_irq(my_gpio_num2, my_gpio_event_mask2);
 *       // handle the IRQ
 *     }
 * }
 * \endcode
 *
 * @param gpio_mask a bit mask of the GPIO numbers that will no longer be passed to the default callback for this core
 * @param handler the handler to add to the list of GPIO IRQ handlers for this core
 */
void gpio_add_raw_irq_handler_masked(uint32_t gpio_mask, irq_handler_t handler);

/*! \brief Adds a raw GPIO IRQ handler for the specified GPIOs on the current core
 *  \ingroup hardware_gpio
 *
 * In addition to the default mechanism of a single GPIO IRQ event callback per core (see \ref gpio_set_irq_callback),
 * it is possible to add explicit GPIO IRQ handlers which are called independent of the default event callback.
 *
 * This method adds such a callback, and disables the "default" callback for the specified GPIOs.
 *
 * \note Multiple raw handlers should not be added for the same GPIOs, and this method will assert if you attempt to.
 * Internally, this function calls \ref irq_add_shared_handler, which will assert if the maximum number of shared handlers
 * (configurable via PICO_MAX_IRQ_SHARED_HANDLERS) would be exceeded.
 *
 * A raw handler should check for whichever GPIOs and events it handles, and acknowledge them itself; it might look something like:
 *
 * \code{.c}
 * void my_irq_handler(void) {
 *     if (gpio_get_irq_event_mask(my_gpio_num) & my_gpio_event_mask) {
 *        gpio_acknowledge_irq(my_gpio_num, my_gpio_event_mask);
 *       // handle the IRQ
 *     }
 *     if (gpio_get_irq_event_mask(my_gpio_num2) & my_gpio_event_mask2) {
 *        gpio_acknowledge_irq(my_gpio_num2, my_gpio_event_mask2);
 *       // handle the IRQ
 *     }
 * }
 * \endcode
 *
 * @param gpio_mask a 64 bit mask of the GPIO numbers that will no longer be passed to the default callback for this core
 * @param handler the handler to add to the list of GPIO IRQ handlers for this core
 */
void gpio_add_raw_irq_handler_masked64(uint64_t gpio_mask, irq_handler_t handler);

/*! \brief Adds a raw GPIO IRQ handler for a specific GPIO on the current core
 *  \ingroup hardware_gpio
 *
 * In addition to the default mechanism of a single GPIO IRQ event callback per core (see \ref gpio_set_irq_callback),
 * it is possible to add explicit GPIO IRQ handlers which are called independent of the default event callback.
 *
 * This method adds such a callback, and disables the "default" callback for the specified GPIO.
 *
 * \note Multiple raw handlers should not be added for the same GPIO, and this method will assert if you attempt to.
 * Internally, this function calls \ref irq_add_shared_handler, which will assert if the maximum number of shared handlers
 * (configurable via PICO_MAX_IRQ_SHARED_HANDLERS) would be exceeded.
 *
 * A raw handler should check for whichever GPIOs and events it handles, and acknowledge them itself; it might look something like:
 *
 * \code{.c}
 * void my_irq_handler(void) {
 *     if (gpio_get_irq_event_mask(my_gpio_num) & my_gpio_event_mask) {
 *        gpio_acknowledge_irq(my_gpio_num, my_gpio_event_mask);
 *       // handle the IRQ
 *     }
 * }
 * \endcode
 *
 * @param gpio the GPIO number that will no longer be passed to the default callback for this core
 * @param handler the handler to add to the list of GPIO IRQ handlers for this core
 */
static inline void gpio_add_raw_irq_handler(uint gpio, irq_handler_t handler) {
    check_gpio_param(gpio);
#if NUM_BANK0_GPIOS > 32
    gpio_add_raw_irq_handler_masked64(1ull << gpio, handler);
#else
    gpio_add_raw_irq_handler_masked(1u << gpio, handler);
#endif
}

/*! \brief Removes a raw GPIO IRQ handler for the specified GPIOs on the current core
 *  \ingroup hardware_gpio
 *
 * In addition to the default mechanism of a single GPIO IRQ event callback per core (see \ref gpio_set_irq_callback),
 * it is possible to add explicit GPIO IRQ handlers which are called independent of the default event callback.
 *
 * This method removes such a callback, and enables the "default" callback for the specified GPIOs.
 *
 * @param gpio_mask a bit mask of the GPIO numbers that will now be passed to the default callback for this core
 * @param handler the handler to remove from the list of GPIO IRQ handlers for this core
 */
void gpio_remove_raw_irq_handler_masked(uint32_t gpio_mask, irq_handler_t handler);

/*! \brief Removes a raw GPIO IRQ handler for the specified GPIOs on the current core
 *  \ingroup hardware_gpio
 *
 * In addition to the default mechanism of a single GPIO IRQ event callback per core (see \ref gpio_set_irq_callback),
 * it is possible to add explicit GPIO IRQ handlers which are called independent of the default event callback.
 *
 * This method removes such a callback, and enables the "default" callback for the specified GPIOs.
 *
 * @param gpio_mask a bit mask of the GPIO numbers that will now be passed to the default callback for this core
 * @param handler the handler to remove from the list of GPIO IRQ handlers for this core
 */
void gpio_remove_raw_irq_handler_masked64(uint64_t gpio_mask, irq_handler_t handler);

/*! \brief Removes a raw GPIO IRQ handler for the specified GPIO on the current core
 *  \ingroup hardware_gpio
 *
 * In addition to the default mechanism of a single GPIO IRQ event callback per core (see \ref gpio_set_irq_callback),
 * it is possible to add explicit GPIO IRQ handlers which are called independent of the default event callback.
 *
 * This method removes such a callback, and enables the "default" callback for the specified GPIO.
 *
 * @param gpio the GPIO number that will now be passed to the default callback for this core
 * @param handler the handler to remove from the list of GPIO IRQ handlers for this core
 */
static inline void gpio_remove_raw_irq_handler(uint gpio, irq_handler_t handler) {
    check_gpio_param(gpio);
#if NUM_BANK0_GPIOS > 32
    gpio_remove_raw_irq_handler_masked64(1ull << gpio, handler);
#else
    gpio_remove_raw_irq_handler_masked(1u << gpio, handler);
#endif
}

/*! \brief Initialise a GPIO for (enabled I/O and set func to GPIO_FUNC_SIO)
 *  \ingroup hardware_gpio
 *
 * Clear the output enable (i.e. set to input).
 * Clear any output value.
 *
 * \param gpio GPIO number
 */
void gpio_init(uint gpio);

/*! \brief Resets a GPIO back to the NULL function, i.e. disables it.
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 */
void gpio_deinit(uint gpio);

/*! \brief Initialise multiple GPIOs (enabled I/O and set func to GPIO_FUNC_SIO)
 *  \ingroup hardware_gpio
 *
 * Clear the output enable (i.e. set to input).
 * Clear any output value.
 *
 * \param gpio_mask Mask with 1 bit per GPIO number to initialize
 */
void gpio_init_mask(uint gpio_mask);
// ----------------------------------------------------------------------------
// Input
// ----------------------------------------------------------------------------

/*! \brief Get state of a single specified GPIO
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return Current state of the GPIO. 0 for low, non-zero for high
 */
static inline bool gpio_get(uint gpio) {
#if NUM_BANK0_GPIOS <= 32
    return sio_hw->gpio_in & (1u << gpio);
#else
    if (gpio < 32) {
        return sio_hw->gpio_in & (1u << gpio);
    } else {
        return sio_hw->gpio_hi_in & (1u << (gpio - 32));
    }
#endif
}

/*! \brief Get raw value of all GPIOs
 *  \ingroup hardware_gpio
 *
 * \return Bitmask of raw GPIO values
 */
static inline uint32_t gpio_get_all(void) {
#if PICO_USE_GPIO_COPROCESSOR
    return gpioc_lo_in_get();
#else
    return sio_hw->gpio_in;
#endif
}

/*! \brief Get raw value of all GPIOs
 *  \ingroup hardware_gpio
 *
 * \return Bitmask of raw GPIO values
 */
static inline uint64_t gpio_get_all64(void) {
#if PICO_USE_GPIO_COPROCESSOR
    return gpioc_hilo_in_get();
#elif NUM_BANK0_GPIOS <= 32
    return sio_hw->gpio_in;
#else
    return sio_hw->gpio_in | (((uint64_t)sio_hw->gpio_hi_in) << 32u);
#endif
}

// ----------------------------------------------------------------------------
// Output
// ----------------------------------------------------------------------------

/*! \brief Drive high every GPIO appearing in mask
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO values to set
 */
static inline void gpio_set_mask(uint32_t mask) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_lo_out_set(mask);
#else
    sio_hw->gpio_set = mask;
#endif
}

/*! \brief Drive high every GPIO appearing in mask
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO values to set
 */
static inline void gpio_set_mask64(uint64_t mask) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_hilo_out_set(mask);
#elif NUM_BANK0_GPIOS <= 32
    sio_hw->gpio_set = (uint32_t)mask;
#else
    sio_hw->gpio_set = (uint32_t)mask;
    sio_hw->gpio_hi_set = (uint32_t)(mask >> 32u);
#endif
}

/*! \brief Drive high every GPIO appearing in mask
 *  \ingroup hardware_gpio
 *
 * \param n the base GPIO index of the mask to update. n == 0 means 0->31, n == 1 mean 32->63 etc.
 * \param mask Bitmask of 32 GPIO values to set
 */
static inline void gpio_set_mask_n(uint n, uint32_t mask) {
    if (!n) {
        gpio_set_mask(mask);
    } else if (n == 1) {
#if PICO_USE_GPIO_COPROCESSOR
        gpioc_hi_out_set(mask);
#elif NUM_BANK0_GPIOS >= 32
        sio_hw->gpio_hi_set = mask;
#endif
    }
}

/*! \brief Drive low every GPIO appearing in mask
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO values to clear
 */
static inline void gpio_clr_mask(uint32_t mask) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_lo_out_clr(mask);
#else
    sio_hw->gpio_clr = mask;
#endif
}

/*! \brief Drive low every GPIO appearing in mask
*  \ingroup hardware_gpio
*
* \param mask Bitmask of GPIO values to clear
*/
static inline void gpio_clr_mask64(uint64_t mask) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_hilo_out_clr(mask);
#elif NUM_BANK0_GPIOS <= 32
    sio_hw->gpio_clr = (uint32_t)mask;
#else
    sio_hw->gpio_clr = (uint32_t)mask;
    sio_hw->gpio_hi_clr = (uint32_t)(mask >> 32u);
#endif
}


/*! \brief Drive low every GPIO appearing in mask
 *  \ingroup hardware_gpio
 *
 * \param n the base GPIO index of the mask to update. n == 0 means 0->31, n == 1 mean 32->63 etc.
 * \param mask Bitmask of 32 GPIO values to clear
 */
static inline void gpio_clr_mask_n(uint n, uint32_t mask) {
    if (!n) {
        gpio_clr_mask(mask);
    } else if (n == 1) {
#if PICO_USE_GPIO_COPROCESSOR
        gpioc_hi_out_clr(mask);
#elif NUM_BANK0_GPIOS >= 32
        sio_hw->gpio_hi_clr = mask;
#endif
    }
}

/*! \brief Toggle every GPIO appearing in mask
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO values to toggle
 */
static inline void gpio_xor_mask(uint32_t mask) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_lo_out_xor(mask);
#else
    sio_hw->gpio_togl = mask;
#endif
}

/*! \brief Toggle every GPIO appearing in mask
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO values to toggle
 */
static inline void gpio_xor_mask64(uint64_t mask) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_hilo_out_xor(mask);
#elif NUM_BANK0_GPIOS <= 32
    sio_hw->gpio_togl = (uint32_t)mask;
#else
    sio_hw->gpio_togl = (uint32_t)mask;
    sio_hw->gpio_hi_togl = (uint32_t)(mask >> 32u);
#endif
}

/*! \brief Toggle every GPIO appearing in mask
 *  \ingroup hardware_gpio
 *
 * \param n the base GPIO index of the mask to update. n == 0 means 0->31, n == 1 mean 32->63 etc.
 * \param mask Bitmask of 32 GPIO values to toggle
 */
static inline void gpio_xor_mask_n(uint n, uint32_t mask) {
    if (!n) {
        gpio_xor_mask(mask);
    } else if (n == 1) {
#if PICO_USE_GPIO_COPROCESSOR
        gpioc_hi_out_xor(mask);
#elif NUM_BANK0_GPIOS >= 32
        sio_hw->gpio_hi_togl = mask;
#endif
    }
}

/*! \brief Drive GPIOs high/low depending on parameters
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO values to change
 * \param value Value to set
 *
 * For each 1 bit in \p mask, drive that pin to the value given by
 * corresponding bit in \p value, leaving other pins unchanged.
 * Since this uses the TOGL alias, it is concurrency-safe with e.g. an IRQ
 * bashing different pins from the same core.
 */
static inline void gpio_put_masked(uint32_t mask, uint32_t value) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_lo_out_xor((gpioc_lo_out_get() ^ value) & mask);
#else
    sio_hw->gpio_togl = (sio_hw->gpio_out ^ value) & mask;
#endif
}

/*! \brief Drive GPIOs high/low depending on parameters
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO values to change
 * \param value Value to set
 *
 * For each 1 bit in \p mask, drive that pin to the value given by
 * corresponding bit in \p value, leaving other pins unchanged.
 * Since this uses the TOGL alias, it is concurrency-safe with e.g. an IRQ
 * bashing different pins from the same core.
 */
static inline void gpio_put_masked64(uint64_t mask, uint64_t value) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_hilo_out_xor((gpioc_hilo_out_get() ^ value) & mask);
#elif NUM_BANK0_GPIOS <= 32
    sio_hw->gpio_togl = (sio_hw->gpio_out ^ (uint32_t)value) & (uint32_t)mask;
#else
    sio_hw->gpio_togl = (sio_hw->gpio_out ^ (uint32_t)value) & (uint32_t)mask;
    sio_hw->gpio_hi_togl = (sio_hw->gpio_hi_out ^ (uint32_t)(value>>32u)) & (uint32_t)(mask>>32u);
#endif
}

/*! \brief Drive GPIOs high/low depending on parameters
 *  \ingroup hardware_gpio
 *
 * \param n the base GPIO index of the mask to update. n == 0 means 0->31, n == 1 mean 32->63 etc.
 * \param mask Bitmask of GPIO values to change
 * \param value Value to set
 *
 * For each 1 bit in \p mask, drive that pin to the value given by
 * corresponding bit in \p value, leaving other pins unchanged.
 * Since this uses the TOGL alias, it is concurrency-safe with e.g. an IRQ
 * bashing different pins from the same core.
 */
static inline void gpio_put_masked_n(uint n, uint32_t mask, uint32_t value) {
    if (!n) {
        gpio_put_masked(mask, value);
    } else if (n == 1) {
#if PICO_USE_GPIO_COPROCESSOR
        gpioc_hi_out_xor((gpioc_hi_out_get() ^ value) & mask);
#else
        sio_hw->gpio_hi_togl = (sio_hw->gpio_hi_out ^ value) & mask;
#endif
    }
}

/*! \brief Drive all pins simultaneously
 *  \ingroup hardware_gpio
 *
 * \param value Bitmask of GPIO values to change
 */
static inline void gpio_put_all(uint32_t value) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_lo_out_put(value);
#else
    sio_hw->gpio_out = value;
#endif
}

/*! \brief Drive all pins simultaneously
 *  \ingroup hardware_gpio
 *
 * \param value Bitmask of GPIO values to change
 */
static inline void gpio_put_all64(uint64_t value) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_hilo_out_put(value);
#elif NUM_BANK0_GPIOS <= 32
    sio_hw->gpio_out = (uint32_t)value;
#else
    sio_hw->gpio_out = (uint32_t)value;
    sio_hw->gpio_hi_out = (uint32_t)(value >> 32u);
#endif
}

/*! \brief Drive a single GPIO high/low
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \param value If false clear the GPIO, otherwise set it.
 */
static inline void gpio_put(uint gpio, bool value) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_bit_out_put(gpio, value);
#elif NUM_BANK0_GPIOS <= 32
    uint32_t mask = 1ul << gpio;
    if (value)
        gpio_set_mask(mask);
    else
        gpio_clr_mask(mask);
#else
    uint32_t mask = 1ul << (gpio & 0x1fu);
    if (gpio < 32) {
        if (value) {
            sio_hw->gpio_set = mask;
        } else {
            sio_hw->gpio_clr = mask;
        }
    } else {
        if (value) {
            sio_hw->gpio_hi_set = mask;
        } else {
            sio_hw->gpio_hi_clr = mask;
        }
    }
#endif
}

/*! \brief Determine whether a GPIO is currently driven high or low
 *  \ingroup hardware_gpio
 *
 * This function returns the high/low output level most recently assigned to a
 * GPIO via gpio_put() or similar. This is the value that is presented outward
 * to the IO muxing, *not* the input level back from the pad (which can be
 * read using gpio_get()).
 *
 * To avoid races, this function must not be used for read-modify-write
 * sequences when driving GPIOs -- instead functions like gpio_put() should be
 * used to atomically update GPIOs. This accessor is intended for debug use
 * only.
 *
 * \param gpio GPIO number
 * \return true if the GPIO output level is high, false if low.
 */
static inline bool gpio_get_out_level(uint gpio) {
#if NUM_BANK0_GPIOS <= 32
    return sio_hw->gpio_out & (1u << gpio);
#else
    uint32_t bits = gpio < 32 ? sio_hw->gpio_out : sio_hw->gpio_hi_out;
    return bits & (1u << (gpio & 0x1fu));
#endif
}

// ----------------------------------------------------------------------------
// Direction
// ----------------------------------------------------------------------------

/*! \brief Set a number of GPIOs to output
 *  \ingroup hardware_gpio
 *
 * Switch all GPIOs in "mask" to output
 *
 * \param mask Bitmask of GPIO to set to output
 */
static inline void gpio_set_dir_out_masked(uint32_t mask) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_lo_oe_set(mask);
#else
    sio_hw->gpio_oe_set = mask;
#endif
}

/*! \brief Set a number of GPIOs to output
 *  \ingroup hardware_gpio
 *
 * Switch all GPIOs in "mask" to output
 *
 * \param mask Bitmask of GPIO to set to output
 */
static inline void gpio_set_dir_out_masked64(uint64_t mask) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_hilo_oe_set(mask);
#elif NUM_BANK0_GPIOS <= 32
    sio_hw->gpio_oe_set = mask;
#else
    sio_hw->gpio_oe_set = (uint32_t)mask;
    sio_hw->gpio_hi_oe_set = (uint32_t)(mask >> 32u);
#endif
}

/*! \brief Set a number of GPIOs to input
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO to set to input
 */
static inline void gpio_set_dir_in_masked(uint32_t mask) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_lo_oe_clr(mask);
#else
    sio_hw->gpio_oe_clr = mask;
#endif
}

/*! \brief Set a number of GPIOs to input
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO to set to input
 */
static inline void gpio_set_dir_in_masked64(uint64_t mask) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_hilo_oe_clr(mask);
#elif NUM_BANK0_GPIOS <= 32
    sio_hw->gpio_oe_clr = mask;
#else
    sio_hw->gpio_oe_clr = (uint32_t)mask;
    sio_hw->gpio_hi_oe_clr = (uint32_t)(mask >> 32u);
#endif
}

/*! \brief Set multiple GPIO directions
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO to set to input, as bits 0-29
 * \param value Values to set
 *
 * For each 1 bit in "mask", switch that pin to the direction given by
 * corresponding bit in "value", leaving other pins unchanged.
 * E.g. gpio_set_dir_masked(0x3, 0x2); -> set pin 0 to input, pin 1 to output,
 * simultaneously.
 */
static inline void gpio_set_dir_masked(uint32_t mask, uint32_t value) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_lo_oe_xor((gpioc_lo_oe_get() ^ value) & mask);
#else
    sio_hw->gpio_oe_togl = (sio_hw->gpio_oe ^ value) & mask;
#endif
}

/*! \brief Set multiple GPIO directions
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO to set to input, as bits 0-29
 * \param value Values to set
 *
 * For each 1 bit in "mask", switch that pin to the direction given by
 * corresponding bit in "value", leaving other pins unchanged.
 * E.g. gpio_set_dir_masked(0x3, 0x2); -> set pin 0 to input, pin 1 to output,
 * simultaneously.
 */
static inline void gpio_set_dir_masked64(uint64_t mask, uint64_t value) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_hilo_oe_xor((gpioc_hilo_oe_get() ^ value) & mask);
#elif NUM_BANK0_GPIOS <= 32
    sio_hw->gpio_oe_togl = (sio_hw->gpio_oe ^ (uint32_t)value) & (uint32_t)mask;
#else
    sio_hw->gpio_oe_togl = (sio_hw->gpio_oe ^ (uint32_t)value) & (uint32_t)mask;
    sio_hw->gpio_hi_oe_togl = (sio_hw->gpio_hi_oe ^ (uint32_t)(value >> 32u)) & (uint32_t)(mask >> 32u);
#endif
}


/*! \brief Set direction of all pins simultaneously.
 *  \ingroup hardware_gpio
 *
 * \param values individual settings for each gpio; for GPIO N, bit N is 1 for out, 0 for in
 */
static inline void gpio_set_dir_all_bits(uint32_t values) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_lo_oe_put(values);
#else
    sio_hw->gpio_oe = values;
#endif
}

/*! \brief Set direction of all pins simultaneously.
 *  \ingroup hardware_gpio
 *
 * \param values individual settings for each gpio; for GPIO N, bit N is 1 for out, 0 for in
 */
static inline void gpio_set_dir_all_bits64(uint64_t values) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_hilo_oe_put(values);
#elif NUM_BANK0_GPIOS <= 32
    sio_hw->gpio_oe = (uint32_t)values;
#else
    sio_hw->gpio_oe = (uint32_t)values;
    sio_hw->gpio_hi_oe = (uint32_t)(values >> 32u);
#endif
}

/*! \brief Set a single GPIO direction
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \param out true for out, false for in
 */
static inline void gpio_set_dir(uint gpio, bool out) {
#if PICO_USE_GPIO_COPROCESSOR
    gpioc_bit_oe_put(gpio, out);
#elif PICO_RP2040 || NUM_BANK0_GPIOS <= 32
    uint32_t mask = 1ul << gpio;
    if (out)
        gpio_set_dir_out_masked(mask);
    else
        gpio_set_dir_in_masked(mask);
#else
    uint32_t mask = 1u << (gpio & 0x1fu);
    if (gpio < 32) {
        if (out) {
            sio_hw->gpio_oe_set = mask;
        } else {
            sio_hw->gpio_oe_clr = mask;
        }
    } else {
        if (out) {
            sio_hw->gpio_hi_oe_set = mask;
        } else {
            sio_hw->gpio_hi_oe_clr = mask;
        }
    }
#endif
}

/*! \brief Check if a specific GPIO direction is OUT
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return true if the direction for the pin is OUT
 */
static inline bool gpio_is_dir_out(uint gpio) {
#if NUM_BANK0_GPIOS <= 32
    return sio_hw->gpio_oe & (1u << (gpio));
#else
    uint32_t bits = gpio < 32 ? sio_hw->gpio_oe : sio_hw->gpio_hi_oe;
    return bits & (1u << (gpio & 0x1fu));
#endif
}

/*! \brief Get a specific GPIO direction
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return 1 for out, 0 for in
 */
static inline uint gpio_get_dir(uint gpio) {
    return gpio_is_dir_out(gpio); // note GPIO_OUT is 1/true and GPIO_IN is 0/false anyway
}

#if PICO_SECURE
static inline void gpio_assign_to_ns(uint gpio, bool ns) {
    check_gpio_param(gpio);
    if (ns) hw_set_bits(&accessctrl_hw->gpio_nsmask[gpio/32], 1u << (gpio & 0x1fu));
    else hw_clear_bits(&accessctrl_hw->gpio_nsmask[gpio/32], 1u << (gpio & 0x1fu));
}
#endif
extern void gpio_debug_pins_init(void);

#ifdef __cplusplus
}
#endif


// PICO_CONFIG: PICO_DEBUG_PIN_BASE, First pin to use for debug output (if enabled), min=0, max=28, default=19, group=hardware_gpio
#ifndef PICO_DEBUG_PIN_BASE
#define PICO_DEBUG_PIN_BASE 19u
#endif

// PICO_CONFIG: PICO_DEBUG_PIN_COUNT, Number of pins to use for debug output (if enabled), min=1, max=28, default=3, group=hardware_gpio
#ifndef PICO_DEBUG_PIN_COUNT
#define PICO_DEBUG_PIN_COUNT 3u
#endif

#ifndef __cplusplus
// note these two macros may only be used once per and only apply per compilation unit (hence the CU_)
#define CU_REGISTER_DEBUG_PINS(...) enum __unused DEBUG_PIN_TYPE { _none = 0, __VA_ARGS__ }; static enum DEBUG_PIN_TYPE __selected_debug_pins;
#define CU_SELECT_DEBUG_PINS(x) static enum DEBUG_PIN_TYPE __selected_debug_pins = (x);
#define DEBUG_PINS_ENABLED(p) (__selected_debug_pins == (p))
#else
#define CU_REGISTER_DEBUG_PINS(p...) \
    enum DEBUG_PIN_TYPE { _none = 0, p }; \
    template <enum DEBUG_PIN_TYPE> class __debug_pin_settings { \
        public: \
            static inline bool enabled() { return false; } \
    };
#define CU_SELECT_DEBUG_PINS(x) template<> inline bool __debug_pin_settings<x>::enabled() { return true; };
#define DEBUG_PINS_ENABLED(p) (__debug_pin_settings<p>::enabled())
#endif
#define DEBUG_PINS_SET(p, v) if (DEBUG_PINS_ENABLED(p)) gpio_set_mask((unsigned)(v)<<PICO_DEBUG_PIN_BASE)
#define DEBUG_PINS_CLR(p, v) if (DEBUG_PINS_ENABLED(p)) gpio_clr_mask((unsigned)(v)<<PICO_DEBUG_PIN_BASE)
#define DEBUG_PINS_XOR(p, v) if (DEBUG_PINS_ENABLED(p)) gpio_xor_mask((unsigned)(v)<<PICO_DEBUG_PIN_BASE)

#endif // _GPIO_H_
