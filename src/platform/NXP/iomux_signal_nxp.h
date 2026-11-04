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

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "common/utils.h"
#include "drivers/io_def.h"
#include "drivers/io_types.h"

#include "iomux_nxp.h"

// A board names the pin; the muxing is looked up from the pin and signal pair in
// the generated tables iomux_signal_def.h selects.

// Signal roles; the generator folds each family's vendor spelling onto these.
// Each bus has its own role numbers, so a lookup with another bus's role fails.
typedef enum {
    NXP_ROLE_NONE = 0,
    NXP_ROLE_LPUART_TX,
    NXP_ROLE_LPUART_RX,
    NXP_ROLE_LPUART_CTS,
    NXP_ROLE_LPUART_RTS,
    NXP_ROLE_LPUART_DCD,      // modem lines, unused by this platform
    NXP_ROLE_LPUART_DSR,
    NXP_ROLE_LPUART_DTR,
    NXP_ROLE_LPUART_RIN,
    NXP_ROLE_LPSPI_SCK,
    NXP_ROLE_LPSPI_SDI,
    NXP_ROLE_LPSPI_SDO,
    NXP_ROLE_LPSPI_PCS0,
    NXP_ROLE_LPSPI_PCS1,
    NXP_ROLE_LPSPI_PCS2,
    NXP_ROLE_LPSPI_PCS3,
    NXP_ROLE_LPI2C_SCL,
    NXP_ROLE_LPI2C_SDA,
    NXP_ROLE_LPI2C_SCLS,      // the slave side of a bus, unused by this platform
    NXP_ROLE_LPI2C_SDAS,
    NXP_ROLE_LPI2C_HREQ,      // host request, which gates a master transfer
    NXP_ROLE_USDHC_CLK,
    NXP_ROLE_USDHC_CMD,
    NXP_ROLE_USDHC_DATA0,     // DATA0 to DATA7 are consecutive, so a bus
    NXP_ROLE_USDHC_DATA1,     // width loop can count from DATA0
    NXP_ROLE_USDHC_DATA2,
    NXP_ROLE_USDHC_DATA3,
    NXP_ROLE_USDHC_DATA4,
    NXP_ROLE_USDHC_DATA5,
    NXP_ROLE_USDHC_DATA6,
    NXP_ROLE_USDHC_DATA7,
    NXP_ROLE_USDHC_WP,        // write protect
    NXP_ROLE_USDHC_CD,        // card detect
    NXP_ROLE_USDHC_VSELECT,
    NXP_ROLE_USDHC_RESET,
    NXP_ROLE_USDHC_STROBE,
    // FlexCAN receive is a chip input, so its rows are the ones that carry a
    // daisy chain register; transmit leaves those two fields zero.
    NXP_ROLE_FLEXCAN_TX,
    NXP_ROLE_FLEXCAN_RX,
    NXP_ROLE_COUNT,
} nxpSignalRole_e;

typedef struct nxpSignalPin_s {
    uint8_t port;       // 0 is GPIO1, which Betaflight calls port A
    uint8_t pin;
    uint8_t instance;   // 1 is LPUART1, LPSPI1, LPI2C1 or USDHC1
    uint8_t role;       // nxpSignalRole_e
    nxpPinMux_t mux;
} nxpSignalPin_t;

// Return the muxing that puts one instance's signal on a pin, or NULL when the
// chip cannot route it to that pad; the caller must check.
static inline const nxpPinMux_t *nxpPinMuxFind(const nxpSignalPin_t *table,
                                                   size_t count,
                                                   uint8_t instance,
                                                   uint8_t role,
                                                   ioTag_t pin)
{
    if (!pin) {
        return NULL;
    }

    const int port = DEFIO_TAG_GPIOID(pin);
    const int index = DEFIO_TAG_PIN(pin);

    for (size_t i = 0; i < count; i++) {
        if (table[i].instance == instance && table[i].role == role
            && table[i].port == port && table[i].pin == index) {
            return &table[i].mux;
        }
    }

    return NULL;
}

// Search a whole table, without having to repeat its length.
#define NXP_PIN_MUX(table, instance, role, pin) \
    nxpPinMuxFind((table), ARRAYLEN(table), (instance), (role), (pin))

// FlexPWM and FlexIO tables are searched by pin and give what the pad is wired
// to inside the chip.

// FlexPWM submodule outputs; X shares the reload point, so it has no pulse width of its own.
typedef enum {
    IMXRT_FLEXPWM_CHANNEL_A = 0,
    IMXRT_FLEXPWM_CHANNEL_B,
    IMXRT_FLEXPWM_CHANNEL_X,
} imxrtFlexpwmChannel_e;

typedef struct imxrtFlexpwmPin_s {
    uint8_t port;           // 0 is GPIO1, which Betaflight calls port A
    uint8_t pin;
    uint8_t instance;       // 1 is FlexPWM1
    uint8_t submodule;
    uint8_t channel;        // imxrtFlexpwmChannel_e
    nxpPinMux_t mux;
} imxrtFlexpwmPin_t;

typedef struct nxpFlexioPin_s {
    uint8_t port;
    uint8_t pin;
    uint8_t instance;       // 1 is FlexIO1
    uint8_t flexioPin;      // FlexIO pin number, which is not a shifter number
    nxpPinMux_t mux;
} nxpFlexioPin_t;

// A pad has at most one FlexPWM output, so the pin is the whole key.
static inline const imxrtFlexpwmPin_t *imxrtFlexpwmPinFind(const imxrtFlexpwmPin_t *table,
                                                           size_t count,
                                                           ioTag_t pin)
{
    if (!pin) {
        return NULL;
    }

    for (size_t i = 0; i < count; i++) {
        if (table[i].port == DEFIO_TAG_GPIOID(pin) && table[i].pin == DEFIO_TAG_PIN(pin)) {
            return &table[i];
        }
    }

    return NULL;
}

// A pad can be a pin of two different FlexIO instances, on two different mux
// modes, so the instance is part of the key here.
static inline const nxpFlexioPin_t *nxpFlexioPinFind(const nxpFlexioPin_t *table,
                                                         size_t count,
                                                         uint8_t instance,
                                                         ioTag_t pin)
{
    if (!pin) {
        return NULL;
    }

    for (size_t i = 0; i < count; i++) {
        if (table[i].instance == instance
            && table[i].port == DEFIO_TAG_GPIOID(pin) && table[i].pin == DEFIO_TAG_PIN(pin)) {
            return &table[i];
        }
    }

    return NULL;
}

#define IMXRT_FLEXPWM_PIN(table, pin) \
    imxrtFlexpwmPinFind((table), ARRAYLEN(table), (pin))

#define NXP_FLEXIO_PIN(table, instance, pin) \
    nxpFlexioPinFind((table), ARRAYLEN(table), (instance), (pin))
