/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// SPI pin configuration: hardware pin map for the HPM SPI peripherals
// and runtime pin/device selection from the Betaflight config.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPI

#include "build/debug.h"

#include "drivers/bus_spi_types.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "dma_hpmicro.h"
#include "drivers/exti.h"
#include "drivers/io.h"

#include "pg/bus_spi.h"

const spiHardware_t spiHardware[] = {

#ifdef HPM6750
    {
        // SPI1 = HPM_SPI0 (see target.h). PZ pads additionally need BIOC SOC
        // routing, applied by IOConfigBPIOC(), and SCK loopback, applied by
        // spiConfigIOC(); spiPinDef_t only carries { pin, af }.
        .device = SPIDEV_1,
        .reg = (spiResource_t *) SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PZ3), IOC_PZ03_FUNC_CTL_SPI0_SCLK},
        },
        .misoPins = {
            { DEFIO_TAG_E(PZ5), IOC_PZ05_FUNC_CTL_SPI0_MISO},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PZ4), IOC_PZ04_FUNC_CTL_SPI0_MOSI},
        },
        .rcc = clock_spi0,
    },
    {
        .device = SPIDEV_2,
        .reg = (spiResource_t *) SPI2,
        .sckPins = {
            { DEFIO_TAG_E(PD31), IOC_PD31_FUNC_CTL_SPI1_SCLK},
        },
        .misoPins = {
            { DEFIO_TAG_E(PD30), IOC_PD30_FUNC_CTL_SPI1_MISO},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PE4), IOC_PE04_FUNC_CTL_SPI1_MOSI},
        },
        .rcc = clock_spi1,
    },
    {
        .device = SPIDEV_3,
        .reg = (spiResource_t *) SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB0), IOC_PB00_FUNC_CTL_SPI2_SCLK},
            { DEFIO_TAG_E(PB21), IOC_PB21_FUNC_CTL_SPI2_SCLK},
            { DEFIO_TAG_E(PE27), IOC_PE27_FUNC_CTL_SPI2_SCLK},
        },
        .misoPins = {
            { DEFIO_TAG_E(PA31), IOC_PA31_FUNC_CTL_SPI2_MISO},
            { DEFIO_TAG_E(PB25), IOC_PB25_FUNC_CTL_SPI2_MISO},
            { DEFIO_TAG_E(PE28), IOC_PE28_FUNC_CTL_SPI2_MISO},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA27), IOC_PA27_FUNC_CTL_SPI2_MOSI},
            { DEFIO_TAG_E(PB22), IOC_PB22_FUNC_CTL_SPI2_MOSI},
            { DEFIO_TAG_E(PE30), IOC_PE30_FUNC_CTL_SPI2_MOSI},
        },
        .rcc = clock_spi2,
    },
#endif
#ifdef HPM6360
    {
        // SPI1 = HPM_SPI0 (see target.h). SCK loopback applied by
        // spiConfigIOC(); spiPinDef_t only carries { pin, af }.
        .device = SPIDEV_1,
        .reg = (spiResource_t *) SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PA8), IOC_PA08_FUNC_CTL_SPI0_SCLK},
            { DEFIO_TAG_E(PA12), IOC_PA12_FUNC_CTL_SPI0_SCLK},
            { DEFIO_TAG_E(PA30), IOC_PA30_FUNC_CTL_SPI0_SCLK},
        },
        .misoPins = {
            { DEFIO_TAG_E(PA7), IOC_PA07_FUNC_CTL_SPI0_MISO},
            { DEFIO_TAG_E(PA11), IOC_PA11_FUNC_CTL_SPI0_MISO},
            { DEFIO_TAG_E(PA29), IOC_PA29_FUNC_CTL_SPI0_MISO},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA9), IOC_PA09_FUNC_CTL_SPI0_MOSI},
            { DEFIO_TAG_E(PA13), IOC_PA13_FUNC_CTL_SPI0_MOSI},
            { DEFIO_TAG_E(PA31), IOC_PA31_FUNC_CTL_SPI0_MOSI},
        },
        .rcc = clock_spi0,
    },
    {
        .device = SPIDEV_2,
        .reg = (spiResource_t *) SPI2,
        .sckPins = {
            { DEFIO_TAG_E(PA18), IOC_PA18_FUNC_CTL_SPI1_SCLK},
            { DEFIO_TAG_E(PB4), IOC_PB04_FUNC_CTL_SPI1_SCLK},
            { DEFIO_TAG_E(PB9), IOC_PB09_FUNC_CTL_SPI1_SCLK},
            { DEFIO_TAG_E(PB29), IOC_PB29_FUNC_CTL_SPI1_SCLK},
        },
        .misoPins = {
            { DEFIO_TAG_E(PA17), IOC_PA17_FUNC_CTL_SPI1_MISO},
            { DEFIO_TAG_E(PB3), IOC_PB03_FUNC_CTL_SPI1_MISO},
            { DEFIO_TAG_E(PB10), IOC_PB10_FUNC_CTL_SPI1_MISO},
            { DEFIO_TAG_E(PB28), IOC_PB28_FUNC_CTL_SPI1_MISO},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA19), IOC_PA19_FUNC_CTL_SPI1_MOSI},
            { DEFIO_TAG_E(PB5), IOC_PB05_FUNC_CTL_SPI1_MOSI},
            { DEFIO_TAG_E(PB11), IOC_PB11_FUNC_CTL_SPI1_MOSI},
            { DEFIO_TAG_E(PB30), IOC_PB30_FUNC_CTL_SPI1_MOSI},
        },
        .rcc = clock_spi1,
    },
    {
        .device = SPIDEV_3,
        .reg = (spiResource_t *) SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB15), IOC_PB15_FUNC_CTL_SPI2_SCLK},
        },
        .misoPins = {
            { DEFIO_TAG_E(PB14), IOC_PB14_FUNC_CTL_SPI2_MISO},
            { DEFIO_TAG_E(PC0), IOC_PC00_FUNC_CTL_SPI2_MISO},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB16), IOC_PB16_FUNC_CTL_SPI2_MOSI},
        },
        .rcc = clock_spi2,
    },
    {
        .device = SPIDEV_4,
        .reg = (spiResource_t *) SPI4,
        .sckPins = {
            { DEFIO_TAG_E(PA2), IOC_PA02_FUNC_CTL_SPI3_SCLK},
            { DEFIO_TAG_E(PC20), IOC_PC20_FUNC_CTL_SPI3_SCLK},
        },
        .misoPins = {
            { DEFIO_TAG_E(PA1), IOC_PA01_FUNC_CTL_SPI3_MISO},
            { DEFIO_TAG_E(PC19), IOC_PC19_FUNC_CTL_SPI3_MISO},
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA3), IOC_PA03_FUNC_CTL_SPI3_MOSI},
            { DEFIO_TAG_E(PC21), IOC_PC21_FUNC_CTL_SPI3_MOSI},
        },
        .rcc = clock_spi3,
    },
#endif

};

void spiPinConfigure(const spiPinConfig_t *pConfig)
{
    for (size_t hwindex = 0; hwindex < ARRAYLEN(spiHardware); hwindex++) {
        const spiHardware_t *hw = &spiHardware[hwindex];

        if (!hw->reg) {
            continue;
        }

        const spiDevice_e device = hw->device;
        spiDevice_t *pDev = &spiDevice[device];

        for (int pindex = 0; pindex < MAX_SPI_PIN_SEL; pindex++) {
            if (pConfig[device].ioTagSck == hw->sckPins[pindex].pin) {
                pDev->sck = hw->sckPins[pindex].pin;
#if SPI_TRAIT_AF_PIN
                pDev->sckAF = hw->sckPins[pindex].af;
#endif
            }
            if (pConfig[device].ioTagMiso == hw->misoPins[pindex].pin) {
                pDev->miso = hw->misoPins[pindex].pin;
#if SPI_TRAIT_AF_PIN
                pDev->misoAF = hw->misoPins[pindex].af;
#endif
            }
            if (pConfig[device].ioTagMosi == hw->mosiPins[pindex].pin) {
                pDev->mosi = hw->mosiPins[pindex].pin;
#if SPI_TRAIT_AF_PIN
                pDev->mosiAF = hw->mosiPins[pindex].af;
#endif
            }
        }

        if (pDev->sck && pDev->miso && pDev->mosi) {
            pDev->dev = hw->reg;
#if SPI_TRAIT_AF_PORT
            pDev->af = hw->af;
#endif
            pDev->rcc = hw->rcc;
            pDev->leadingEdge = false;  // XXX Should be part of transfer context
        }
    }
}
#endif
