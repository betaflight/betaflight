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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPI

#include "build/debug.h"

#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"

#include "pg/bus_spi.h"

const spiHardware_t spiHardware[] = {
#ifdef STM32F4
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PA5) },
            { DEFIO_TAG_E(PB3) },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA6) },
            { DEFIO_TAG_E(PB4) },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA7) },
            { DEFIO_TAG_E(PB5) },
        },
        .af = GPIO_AF_SPI1,
        .rcc = RCC_APB2(SPI1),
    },
    {
        .device = SPIDEV_2,
        .reg = SPI2,
        .sckPins = {
            { DEFIO_TAG_E(PB10) },
            { DEFIO_TAG_E(PB13) },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB14) },
            { DEFIO_TAG_E(PC2) },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB15) },
            { DEFIO_TAG_E(PC3) },
        },
        .af = GPIO_AF_SPI2,
        .rcc = RCC_APB1(SPI2),
    },
    {
        .device = SPIDEV_3,
        .reg = SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB3) },
            { DEFIO_TAG_E(PC10) },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB4) },
            { DEFIO_TAG_E(PC11) },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB5) },
            { DEFIO_TAG_E(PC12) },
        },
        .af = GPIO_AF_SPI3,
        .rcc = RCC_APB1(SPI3),
    },
#endif
#ifdef STM32F7
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PA5), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB3), GPIO_AF5_SPI1 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA6), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB4), GPIO_AF5_SPI1 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA7), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB5), GPIO_AF5_SPI1 },
        },
        .rcc = RCC_APB2(SPI1),
        .dmaIrqHandler = DMA2_ST3_HANDLER,
    },
    {
        .device = SPIDEV_2,
        .reg = SPI2,
        .sckPins = {
            { DEFIO_TAG_E(PA9), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PB10), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PB13), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PD3), GPIO_AF5_SPI2 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB14), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PC2), GPIO_AF5_SPI2 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB15), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PC1), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PC3), GPIO_AF5_SPI2 },
        },
        .rcc = RCC_APB1(SPI2),
        .dmaIrqHandler = DMA1_ST4_HANDLER,
    },
    {
        .device = SPIDEV_3,
        .reg = SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB3), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC10), GPIO_AF6_SPI3 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB4), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC11), GPIO_AF6_SPI3 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB2), GPIO_AF7_SPI3 },
            { DEFIO_TAG_E(PB5), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC12), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PD6), GPIO_AF5_SPI3 },
        },
        .rcc = RCC_APB1(SPI3),
        .dmaIrqHandler = DMA1_ST7_HANDLER,
    },
    {
        .device = SPIDEV_4,
        .reg = SPI4,
        .sckPins = {
            { DEFIO_TAG_E(PE2), GPIO_AF5_SPI4 },
            { DEFIO_TAG_E(PE12), GPIO_AF5_SPI4 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PE5), GPIO_AF5_SPI4 },
            { DEFIO_TAG_E(PE13), GPIO_AF5_SPI4 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PE6), GPIO_AF5_SPI4 },
            { DEFIO_TAG_E(PE14), GPIO_AF5_SPI4 },
        },
        .rcc = RCC_APB2(SPI4),
        .dmaIrqHandler = DMA2_ST1_HANDLER,
    },
#endif
#ifdef STM32H7
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PA5), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB3), GPIO_AF5_SPI1 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA6), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB4), GPIO_AF5_SPI1 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA7), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB5), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PD7), GPIO_AF5_SPI1 },
        },
        .rcc = RCC_APB2(SPI1),
        //.dmaIrqHandler = DMA2_ST3_HANDLER,
    },
    {
        .device = SPIDEV_2,
        .reg = SPI2,
        .sckPins = {
            { DEFIO_TAG_E(PA9), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PA12), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PB10), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PB13), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PD3), GPIO_AF5_SPI2 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB14), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PC2), GPIO_AF5_SPI2 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB15), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PC1), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PC3), GPIO_AF5_SPI2 },
        },
        .rcc = RCC_APB1L(SPI2),
        //.dmaIrqHandler = DMA1_ST4_HANDLER,
    },
    {
        .device = SPIDEV_3,
        .reg = SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB3), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC10), GPIO_AF6_SPI3 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB4), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC11), GPIO_AF6_SPI3 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB2), GPIO_AF7_SPI3 },
            { DEFIO_TAG_E(PB5), GPIO_AF7_SPI3 },
            { DEFIO_TAG_E(PC12), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PD6), GPIO_AF5_SPI3 },
        },
        .rcc = RCC_APB1L(SPI3),
        //.dmaIrqHandler = DMA1_ST7_HANDLER,
    },
    {
        .device = SPIDEV_4,
        .reg = SPI4,
        .sckPins = {
            { DEFIO_TAG_E(PE2), GPIO_AF5_SPI4 },
            { DEFIO_TAG_E(PE12), GPIO_AF5_SPI4 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PE5), GPIO_AF5_SPI4 },
            { DEFIO_TAG_E(PE13), GPIO_AF5_SPI4 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PE6), GPIO_AF5_SPI4 },
            { DEFIO_TAG_E(PE14), GPIO_AF5_SPI4 },
        },
        .rcc = RCC_APB2(SPI4),
        //.dmaIrqHandler = DMA2_ST1_HANDLER,
    },
    {
        .device = SPIDEV_5,
        .reg = SPI5,
        .sckPins = {
            { DEFIO_TAG_E(PF7), GPIO_AF5_SPI5 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PF8), GPIO_AF5_SPI5 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PF11), GPIO_AF5_SPI5 },
        },
        .rcc = RCC_APB2(SPI5),
        //.dmaIrqHandler = DMA2_ST1_HANDLER,
    },
    {
        .device = SPIDEV_6,
        .reg = SPI6,
        .sckPins = {
            { DEFIO_TAG_E(PA5), GPIO_AF8_SPI6 },
            { DEFIO_TAG_E(PB3), GPIO_AF8_SPI6 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA6), GPIO_AF8_SPI6 },
            { DEFIO_TAG_E(PB4), GPIO_AF8_SPI6 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA7), GPIO_AF8_SPI6 },
            { DEFIO_TAG_E(PB5), GPIO_AF8_SPI6 },
        },
        .rcc = RCC_APB4(SPI6),
        //.dmaIrqHandler = DMA2_ST1_HANDLER,
    },
#endif
#ifdef STM32G4
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PA5), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB3), GPIO_AF5_SPI1 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA6), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB4), GPIO_AF5_SPI1 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA7), GPIO_AF5_SPI1 },
            { DEFIO_TAG_E(PB5), GPIO_AF5_SPI1 },
        },
        .rcc = RCC_APB2(SPI1),
        //.dmaIrqHandler = DMA2_ST3_HANDLER,
    },
    {
        .device = SPIDEV_2,
        .reg = SPI2,
        .sckPins = {
            { DEFIO_TAG_E(PB13), GPIO_AF5_SPI2 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA10), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PB14), GPIO_AF5_SPI2 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA11), GPIO_AF5_SPI2 },
            { DEFIO_TAG_E(PB15), GPIO_AF5_SPI2 },
        },
        .rcc = RCC_APB11(SPI2),
        //.dmaIrqHandler = DMA1_ST4_HANDLER,
    },
    {
        .device = SPIDEV_3,
        .reg = SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB3), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC10), GPIO_AF6_SPI3 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB4), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC11), GPIO_AF6_SPI3 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB5), GPIO_AF6_SPI3 },
            { DEFIO_TAG_E(PC12), GPIO_AF6_SPI3 },
        },
        .rcc = RCC_APB11(SPI3),
        //.dmaIrqHandler = DMA1_ST7_HANDLER,
    },
#endif
};

void spiPinConfigure(const spiPinConfig_t *pConfig)
{
    for (size_t hwindex = 0 ; hwindex < ARRAYLEN(spiHardware) ; hwindex++) {
        const spiHardware_t *hw = &spiHardware[hwindex];

        if (!hw->reg) {
            continue;
        }

        SPIDevice device = hw->device;
        spiDevice_t *pDev = &spiDevice[device];

        for (int pindex = 0 ; pindex < MAX_SPI_PIN_SEL ; pindex++) {
            if (pConfig[device].ioTagSck == hw->sckPins[pindex].pin) {
                pDev->sck = hw->sckPins[pindex].pin;
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
                pDev->sckAF = hw->sckPins[pindex].af;
#endif
            }
            if (pConfig[device].ioTagMiso == hw->misoPins[pindex].pin) {
                pDev->miso = hw->misoPins[pindex].pin;
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
                pDev->misoAF = hw->misoPins[pindex].af;
#endif
            }
            if (pConfig[device].ioTagMosi == hw->mosiPins[pindex].pin) {
                pDev->mosi = hw->mosiPins[pindex].pin;
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
                pDev->mosiAF = hw->mosiPins[pindex].af;
#endif
            }
        }

        if (pDev->sck && pDev->miso && pDev->mosi) {
            pDev->dev = hw->reg;
#if !(defined(STM32F7) || defined(STM32H7) || defined(STM32G4))
            pDev->af = hw->af;
#endif
            pDev->rcc = hw->rcc;
            pDev->leadingEdge = false; // XXX Should be part of transfer context
#if defined(USE_DMA) && defined(USE_HAL_DRIVER)
            pDev->dmaIrqHandler = hw->dmaIrqHandler;
#endif
        }
    }
}
#endif
