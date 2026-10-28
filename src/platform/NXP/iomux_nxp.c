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
 * Pin muxing for the NXP parts. A pad has a mux register (mode and SION) and a pad
 * control register, whose layout on the 11xx depends on the pad group. An input also
 * has a select register, zero in the five tuple for an output.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"

#include "iomux_nxp.h"

#include "iomux_signal_def.h"

static void iomuxWrite(uint32_t address, uint32_t value)
{
    *(volatile uint32_t *)(uintptr_t)address = value;
}

#if defined(IMXRT11XX)

// Bit 1, bits 3:2 and open drain (bit 4, 5 or 6) differ per pad group; the table below picks per group.
#define IMXRT_PAD_SRE_SLOW      IOMUXC_SW_PAD_CTL_PAD_SRE_MASK   // 1 is the slow edge
#define IMXRT_PAD_DSE_HIGH      IOMUXC_SW_PAD_CTL_PAD_DSE_MASK   // 1 is the strong driver
#define IMXRT_PAD_PDRV_NORMAL   IOMUXC_SW_PAD_CTL_PAD_PDRV_MASK  // 0 is the strong driver
#define IMXRT_PAD_PUE           IOMUXC_SW_PAD_CTL_PAD_PUE_MASK   // PUS then picks the direction
#define IMXRT_PAD_PUS_UP        IOMUXC_SW_PAD_CTL_PAD_PUS_MASK
#define IMXRT_PAD_PULL_UP       IOMUXC_SW_PAD_CTL_PAD_PULL(1)    // 0 is forbidden
#define IMXRT_PAD_PULL_DOWN     IOMUXC_SW_PAD_CTL_PAD_PULL(2)
#define IMXRT_PAD_PULL_NONE     IOMUXC_SW_PAD_CTL_PAD_PULL(3)
#define IMXRT_PAD_ODE           IOMUXC_SW_PAD_CTL_PAD_ODE_MASK
#define IMXRT_PAD_ODE_LPSR      IOMUXC_LPSR_SW_PAD_CTL_PAD_ODE_LPSR_MASK
#define IMXRT_PAD_ODE_SNVS      0x40    // the always on block names this per pad only

// Pad control encoding per group; pull is indexed by IOCFG_PULL_*, keeper maps to no pull.
typedef struct imxrtPadLayout_s {
    uint8_t pull[4];
    uint8_t slowSlew;
    uint8_t normalDrive;
    uint8_t highDrive;
    uint8_t openDrain;
} imxrtPadLayout_t;

#define IMXRT_PAD_LAYOUT_DSE(ode) \
    { { 0, IMXRT_PAD_PUE | IMXRT_PAD_PUS_UP, IMXRT_PAD_PUE, 0 }, \
      IMXRT_PAD_SRE_SLOW, 0, IMXRT_PAD_DSE_HIGH, (ode) }

#define IMXRT_PAD_LAYOUT_PDRV \
    { { IMXRT_PAD_PULL_NONE, IMXRT_PAD_PULL_UP, IMXRT_PAD_PULL_DOWN, IMXRT_PAD_PULL_NONE }, \
      0, IMXRT_PAD_PDRV_NORMAL, 0, IMXRT_PAD_ODE }

static const imxrtPadLayout_t imxrtPadLayouts[] = {
    [IMXRT_PAD_GROUP_GPIO_AD]      = IMXRT_PAD_LAYOUT_DSE(IMXRT_PAD_ODE),
    [IMXRT_PAD_GROUP_GPIO_DISP_B1] = IMXRT_PAD_LAYOUT_PDRV,
    [IMXRT_PAD_GROUP_GPIO_DISP_B2] = IMXRT_PAD_LAYOUT_DSE(IMXRT_PAD_ODE),
    [IMXRT_PAD_GROUP_GPIO_EMC_B1]  = IMXRT_PAD_LAYOUT_PDRV,
    [IMXRT_PAD_GROUP_GPIO_EMC_B2]  = IMXRT_PAD_LAYOUT_PDRV,
    [IMXRT_PAD_GROUP_GPIO_LPSR]    = IMXRT_PAD_LAYOUT_DSE(IMXRT_PAD_ODE_LPSR),
    [IMXRT_PAD_GROUP_GPIO_SD_B1]   = IMXRT_PAD_LAYOUT_PDRV,
    [IMXRT_PAD_GROUP_GPIO_SD_B2]   = IMXRT_PAD_LAYOUT_PDRV,
    // The always on pads drive at one fixed strength and have no slew rate.
    [IMXRT_PAD_GROUP_GPIO_SNVS]    = { { 0, IMXRT_PAD_PUE | IMXRT_PAD_PUS_UP, IMXRT_PAD_PUE, 0 },
                                       0, 0, 0, IMXRT_PAD_ODE_SNVS },
};

static uint32_t nxpPadControlValue(ioConfig_t cfg, uint8_t padGroup)
{
    // Group zero writes no pull and no open drain.
    if (padGroup >= ARRAYLEN(imxrtPadLayouts)) {
        padGroup = IMXRT_PAD_GROUP_NONE;
    }

    const imxrtPadLayout_t *layout = &imxrtPadLayouts[padGroup];

    if (IOCFG_MODE(cfg) == IOCFG_MODE_ANALOG) {
        // Leave the pad passive: nothing may pull on an analog input, and the no
        // pull encoding is not the same bits in every group.
        return layout->pull[IOCFG_PULL_NONE];
    }

    // Hysteresis has no field on this family, so IOCFG_HYS is dropped here.
    return layout->pull[IOCFG_PULL(cfg)]
         | (IOCFG_DRIVE(cfg) >= IOCFG_DRIVE_FAST
                ? layout->highDrive
                : (uint32_t)(layout->slowSlew | layout->normalDrive))
         | (IOCFG_OD(cfg) ? layout->openDrain : 0);
}

// Linear scan of the pad grid, once per pin at setup.
static uint8_t nxpPadGroupOf(uint32_t configRegister)
{
    for (int port = 0; port < NXP_IOMUX_PORT_COUNT; port++) {
        for (int pin = 0; pin < NXP_IOMUX_PORT_PINS; pin++) {
            const nxpGpioPad_t *pad = &nxpGpioPads[port][pin];
            if (pad->padCtlRegOffset
                && nxpIomuxPortBase[port] + pad->padCtlRegOffset == configRegister) {
                return pad->padGroup;
            }
        }
    }

    // A pad control register outside the grid, which only the non GPIO pads of
    // the always on block have; group zero suits them.
    return IMXRT_PAD_GROUP_NONE;
}

#elif defined(IMXRT10XX)

// Pull resistor choices in the PUS field. The pad has one pull up value per
// setting; 100K is the middle one and is what a plain pull up asks for.
#define IMXRT_PUS_100K_DOWN     0
#define IMXRT_PUS_100K_UP       2

// Every 10xx pad control register carries the same fields, so no group lookup.
#define nxpPadGroupOf(configRegister) 0

static uint32_t nxpPadControlValue(ioConfig_t cfg, uint8_t padGroup)
{
    UNUSED(padGroup);

    if (IOCFG_MODE(cfg) == IOCFG_MODE_ANALOG) {
        // Leave the pad passive: DSE zero turns the output driver off, and with
        // no pull, keeper or hysteresis nothing loads the analog input.
        return 0;
    }

    // DSE picks R0, R0/3, R0/6 or R0/7 (R0 about 260 ohm at 3.3V); only fast levels raise SPEED and SRE.
    static const uint8_t dseByDrive[4] = { 1, 3, 6, 7 };

    const unsigned drive = IOCFG_DRIVE(cfg);

    uint32_t value = IOMUXC_SW_PAD_CTL_PAD_DSE(dseByDrive[drive])
                   | IOMUXC_SW_PAD_CTL_PAD_SPEED(drive)
                   | IOMUXC_SW_PAD_CTL_PAD_SRE(drive >= IOCFG_DRIVE_FAST ? 1 : 0)
                   | IOMUXC_SW_PAD_CTL_PAD_ODE(IOCFG_OD(cfg))
                   | IOMUXC_SW_PAD_CTL_PAD_HYS(IOCFG_HYS(cfg));

    // PKE turns the pull and keeper circuit on at all, PUE then picks between
    // the keeper (0) and a real resistor (1), and PUS picks which resistor.
    switch (IOCFG_PULL(cfg)) {
    case IOCFG_PULL_UP:
        value |= IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(1)
               | IOMUXC_SW_PAD_CTL_PAD_PUS(IMXRT_PUS_100K_UP);
        break;
    case IOCFG_PULL_DOWN:
        value |= IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(1)
               | IOMUXC_SW_PAD_CTL_PAD_PUS(IMXRT_PUS_100K_DOWN);
        break;
    case IOCFG_PULL_KEEPER:
        value |= IOMUXC_SW_PAD_CTL_PAD_PKE(1);
        break;
    default:
        break;
    }

    return value;
}

#else
#error "Unsupported NXP family: no pad control encoder"
#endif

// SION on open drain pads, so the line can be read back.
static uint32_t nxpMuxValue(uint32_t muxMode, ioConfig_t cfg)
{
    return muxMode | (IOCFG_OD(cfg) ? IOMUXC_SW_MUX_CTL_PAD_SION_MASK : 0);
}

// NXP_RESERVED_PINS, such as the XIP flash bus, are muxed by the boot ROM and never touched here.
#ifdef NXP_RESERVED_PINS
static const ioTag_t nxpReservedPins[] = { NXP_RESERVED_PINS };
#endif

static bool nxpMuxReserved(uint32_t muxRegister)
{
#ifdef NXP_RESERVED_PINS
    for (unsigned i = 0; i < ARRAYLEN(nxpReservedPins); i++) {
        const int port = DEFIO_TAG_GPIOID(nxpReservedPins[i]);
        const int pin = DEFIO_TAG_PIN(nxpReservedPins[i]);
        if (nxpIomuxPortBase[port] + nxpGpioPads[port][pin].muxRegOffset == muxRegister) {
            return true;
        }
    }
#else
    UNUSED(muxRegister);
#endif
    return false;
}

void nxpIomuxClaimReserved(void)
{
#ifdef NXP_RESERVED_PINS
    for (unsigned i = 0; i < ARRAYLEN(nxpReservedPins); i++) {
        IOInit(IOGetByTag(nxpReservedPins[i]), OWNER_SYSTEM, 0);
    }
#endif
}

void nxpIomuxConfigPin(const nxpPinMux_t *mux, ioConfig_t cfg)
{
    if (!mux || !mux->muxRegister || nxpMuxReserved(mux->muxRegister)) {
        return;
    }

    // The mux goes first, so a transmitter already on drives the pad before its
    // GPIO pull is taken away.
    iomuxWrite(mux->muxRegister, nxpMuxValue(mux->muxMode, cfg));

    if (mux->configRegister) {
        iomuxWrite(mux->configRegister, nxpPadControlValue(cfg, nxpPadGroupOf(mux->configRegister)));
    }

    // Only peripheral inputs have a select register. Outputs leave it zero.
    if (mux->inputRegister) {
        iomuxWrite(mux->inputRegister, mux->inputDaisy);
    }
}

void nxpIomuxConfigGpio(IO_t io, ioConfig_t cfg)
{
    const int port = IO_GPIOPortIdx(io);
    const int pin = IO_GPIOPinIdx(io);

    if (port < 0 || port >= NXP_IOMUX_PORT_COUNT
        || pin < 0 || pin >= NXP_IOMUX_PORT_PINS) {
        return;
    }

    const nxpGpioPad_t *pad = &nxpGpioPads[port][pin];
    if (!pad->padCtlRegOffset) {
        // The port has this bit but the package bonds no pad to it.
        return;
    }

    const uint32_t base = nxpIomuxPortBase[port];
    if (nxpMuxReserved(base + pad->muxRegOffset)) {
        return;
    }

    // The pad control goes first, so a pad leaving a peripheral already has its
    // pull when it becomes a GPIO input.
    iomuxWrite(base + pad->padCtlRegOffset, nxpPadControlValue(cfg, pad->padGroup));

    // A pin in alternate function mode was already pointed at its peripheral by
    // nxpIomuxConfigPin(); taking the mux back to GPIO would cut it off.
    if (IOCFG_MODE(cfg) != IOCFG_MODE_AF) {
        iomuxWrite(base + pad->muxRegOffset, nxpMuxValue(pad->gpioMuxMode, cfg));
    }
}
