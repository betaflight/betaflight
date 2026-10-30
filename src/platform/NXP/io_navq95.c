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

/*
 * GPIO primitives for the i.MX 95 RGPIO1..RGPIO5 blocks.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"

// Indexed by the 0-based port of the IO tag (RGPIO1 = 0).
static const uint32_t navq95_rgpioBase[5] = {
    0x47400000u,
    0x43810000u,
    0x43820000u,
    0x43840000u,
    0x43850000u,
};

#define RGPIO_PDOR_OFFSET  0x40u
#define RGPIO_PSOR_OFFSET  0x44u
#define RGPIO_PCOR_OFFSET  0x48u
#define RGPIO_PTOR_OFFSET  0x4Cu
#define RGPIO_PDIR_OFFSET  0x50u
#define RGPIO_PDDR_OFFSET  0x54u

// Per-pin non-secure and non-privileged attributes. Data register writes are
// dropped unless the pin's attributes match the accessing master.
#define RGPIO_PCNS_OFFSET  0x10u
#define RGPIO_PCNP_OFFSET  0x18u

// IOMUXC SW_MUX_CTL
#define IOMUXC_PAD_MUX_MODE_MASK  (0x7U)
#define IOMUXC_PAD_MUX_MODE_SHIFT (0U)
#define IOMUXC_PAD_MUX_MODE(x)    (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_MUX_MODE_SHIFT)) & IOMUXC_PAD_MUX_MODE_MASK)
#define IOMUXC_PAD_SION_MASK      (0x10U)
#define IOMUXC_PAD_SION_SHIFT     (4U)
#define IOMUXC_PAD_SION(x)        (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_SION_SHIFT)) & IOMUXC_PAD_SION_MASK)

// IOMUXC SW_PAD_CTL. HYS is bit 12; the SDK header gives a shift of 11, which does not match its mask.
#define IOMUXC_PAD_DSE_MASK    (0x7EU)
#define IOMUXC_PAD_DSE_SHIFT   (1U)
#define IOMUXC_PAD_DSE(x)      (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_DSE_SHIFT)) & IOMUXC_PAD_DSE_MASK)
#define IOMUXC_PAD_FSEL1_MASK  (0x180U)
#define IOMUXC_PAD_FSEL1_SHIFT (7U)
#define IOMUXC_PAD_FSEL1(x)    (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_FSEL1_SHIFT)) & IOMUXC_PAD_FSEL1_MASK)
#define IOMUXC_PAD_PU_MASK     (0x200U)
#define IOMUXC_PAD_PU_SHIFT    (9U)
#define IOMUXC_PAD_PU(x)       (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_PU_SHIFT)) & IOMUXC_PAD_PU_MASK)
#define IOMUXC_PAD_PD_MASK     (0x400U)
#define IOMUXC_PAD_PD_SHIFT    (10U)
#define IOMUXC_PAD_PD(x)       (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_PD_SHIFT)) & IOMUXC_PAD_PD_MASK)
#define IOMUXC_PAD_OD_MASK     (0x800U)
#define IOMUXC_PAD_OD_SHIFT    (11U)
#define IOMUXC_PAD_OD(x)       (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_OD_SHIFT)) & IOMUXC_PAD_OD_MASK)
#define IOMUXC_PAD_HYS_MASK    (0x1000U)
#define IOMUXC_PAD_HYS_SHIFT   (12U)
#define IOMUXC_PAD_HYS(x)      (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_HYS_SHIFT)) & IOMUXC_PAD_HYS_MASK)
#define IOMUXC_PAD_APC_MASK     (0xF000000U)
#define IOMUXC_PAD_APC_SHIFT    (24U)
#define IOMUXC_PAD_APC(x)       (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_APC_SHIFT)) & IOMUXC_PAD_APC_MASK)
#define IOMUXC_PAD_APC_LCK_MASK  (0xF0000000U)
#define IOMUXC_PAD_APC_LCK_SHIFT (28U)
#define IOMUXC_PAD_APC_LCK(x)    (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_APC_LCK_SHIFT)) & IOMUXC_PAD_APC_LCK_MASK)

// ioConfig_t fields, as packed by IO_CONFIG() in platform.h.
#define NAVQ95_CFG_MODE(cfg)   ((cfg) & 0x03u)
#define NAVQ95_CFG_SPEED(cfg)  (((cfg) >> 2) & 0x07u)
#define NAVQ95_CFG_PUPD(cfg)   (((cfg) >> 5) & 0x03u)

// Outputs need a non-zero drive strength; inputs need hysteresis to be read.
static uint32_t navq95_padCtlFromCfg(ioConfig_t cfg)
{
    uint32_t pad = 0u;

    if (NAVQ95_CFG_MODE(cfg) == GPIO_MODE_OUTPUT) {
        pad |= IOMUXC_PAD_DSE(NAVQ95_CFG_SPEED(cfg) == GPIO_SPEED_HIGH ? 0x3Fu : 0x0Fu);
    } else {
        pad |= IOMUXC_PAD_HYS(1u);
    }

    switch (NAVQ95_CFG_PUPD(cfg)) {
    case GPIO_PULLUP:   pad |= IOMUXC_PAD_PU(1u); break;
    case GPIO_PULLDOWN: pad |= IOMUXC_PAD_PD(1u); break;
    default:            break;
    }

    return pad;
}

typedef struct {
    int      port;
    uint32_t pin;
    uint32_t muxReg;
    uint32_t muxMode;
    uint32_t cfgReg;
} navq95_padMux_t;

// Pads not listed keep the mux the boot firmware left them in.
static const navq95_padMux_t navq95_padMux[] = {
    { 1, 23u, 0x443C006Cu, 0x0u, 0x443C0270u },     // GPIO_IO23, beeper
};

static void navq95_ioMuxConfig(int port, uint32_t pin, ioConfig_t cfg)
{
    for (size_t i = 0; i < sizeof(navq95_padMux) / sizeof(navq95_padMux[0]); i++) {
        const navq95_padMux_t *p = &navq95_padMux[i];
        if (p->port == port && (int)p->pin == (int)pin) {
            *(volatile uint32_t *)p->muxReg = IOMUXC_PAD_MUX_MODE(p->muxMode);
            if (p->cfgReg) {
                *(volatile uint32_t *)p->cfgReg = navq95_padCtlFromCfg(cfg);
            }
            return;
        }
    }
}

static bool navq95_ioResolve(IO_t io, volatile uint8_t **base, uint32_t *mask)
{
    int port = IO_GPIOPortIdx(io);
    int pin = IO_GPIOPinIdx(io);
    if (port < 0 || port >= (int)(sizeof(navq95_rgpioBase) / sizeof(navq95_rgpioBase[0]))) {
        return false;
    }
    *base = (volatile uint8_t *)navq95_rgpioBase[port];
    *mask = (1u << (uint32_t)pin);
    return true;
}

static inline volatile uint32_t *navq95_ioReg(volatile uint8_t *base, uint32_t offset)
{
    return (volatile uint32_t *)(base + offset);
}

bool IORead(IO_t io)
{
    volatile uint8_t *base;
    uint32_t mask;
    if (!navq95_ioResolve(io, &base, &mask)) {
        return false;
    }
    return (*navq95_ioReg(base, RGPIO_PDIR_OFFSET) & mask) != 0u;
}

void IOWrite(IO_t io, bool value)
{
    volatile uint8_t *base;
    uint32_t mask;
    if (!navq95_ioResolve(io, &base, &mask)) {
        return;
    }
    *navq95_ioReg(base, value ? RGPIO_PSOR_OFFSET : RGPIO_PCOR_OFFSET) = mask;
}

void IOHi(IO_t io)
{
    volatile uint8_t *base;
    uint32_t mask;
    if (!navq95_ioResolve(io, &base, &mask)) {
        return;
    }
    *navq95_ioReg(base, RGPIO_PSOR_OFFSET) = mask;
}

void IOLo(IO_t io)
{
    volatile uint8_t *base;
    uint32_t mask;
    if (!navq95_ioResolve(io, &base, &mask)) {
        return;
    }
    *navq95_ioReg(base, RGPIO_PCOR_OFFSET) = mask;
}

void IOToggle(IO_t io)
{
    volatile uint8_t *base;
    uint32_t mask;
    if (!navq95_ioResolve(io, &base, &mask)) {
        return;
    }
    *navq95_ioReg(base, RGPIO_PTOR_OFFSET) = mask;
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    volatile uint8_t *base;
    uint32_t mask;
    if (!navq95_ioResolve(io, &base, &mask)) {
        return;
    }

    navq95_ioMuxConfig(IO_GPIOPortIdx(io), (uint32_t)IO_GPIOPinIdx(io), cfg);

    // The M7 runs secure and privileged, so the pin must be too.
    *navq95_ioReg(base, RGPIO_PCNS_OFFSET) &= ~mask;
    *navq95_ioReg(base, RGPIO_PCNP_OFFSET) &= ~mask;

    volatile uint32_t *pddr = navq95_ioReg(base, RGPIO_PDDR_OFFSET);
    if (NAVQ95_CFG_MODE(cfg) == GPIO_MODE_OUTPUT) {
        *pddr |= mask;
    } else {
        *pddr &= ~mask;
    }
}

#ifdef USE_TIMER_AF
void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af) { (void)io; (void)cfg; (void)af; }
#endif

// The RGPIO ports are already clocked by the boot firmware.
void IOInitGlobal(void)
{
}

IO_t IOGetByTag(ioTag_t tag)
{
    extern ioRec_t ioRecs[];
    if (!tag) {
        return NULL;
    }
    return &ioRecs[DEFIO_TAG_GPIOID(tag) * DEFIO_PORT_PINS + DEFIO_TAG_PIN(tag)];
}
