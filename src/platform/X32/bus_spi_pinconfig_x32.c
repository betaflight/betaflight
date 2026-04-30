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

#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "platform/rcc.h"
#include "pg/bus_spi.h"

const spiHardware_t spiHardware[] = {
#ifdef USE_SPI_DEVICE_1
    {
        .device = SPIDEV_1,
        .reg = (spiResource_t *)SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PA5), GPIO_AF4 },
            { DEFIO_TAG_E(PB3), GPIO_AF1 },
            { DEFIO_TAG_E(PG11), GPIO_AF6 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA6), GPIO_AF4 },
            { DEFIO_TAG_E(PB4), GPIO_AF2 },
            { DEFIO_TAG_E(PG9), GPIO_AF5 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA7), GPIO_AF5 },
            { DEFIO_TAG_E(PB5), GPIO_AF5 },
            { DEFIO_TAG_E(PD7), GPIO_AF3 },
        },
        .rcc = RCC_APB2_2(SPI1),
    },
#endif
#ifdef USE_SPI_DEVICE_2
    {
        .device = SPIDEV_2,
        .reg = (spiResource_t *)SPI2,
        .sckPins = {
            { DEFIO_TAG_E(PA9), GPIO_AF5 },
            { DEFIO_TAG_E(PB10), GPIO_AF5 },
            { DEFIO_TAG_E(PB13), GPIO_AF5 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB14), GPIO_AF5 },
            { DEFIO_TAG_E(PC2), GPIO_AF5 },
            { DEFIO_TAG_E(PI2), GPIO_AF5 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB15), GPIO_AF5 },
            { DEFIO_TAG_E(PC1), GPIO_AF5 },
            { DEFIO_TAG_E(PC3), GPIO_AF5 },
        },
        .rcc = RCC_APB2_2(SPI2),
    },
#endif
#ifdef USE_SPI_DEVICE_3
    {
        .device = SPIDEV_3,
        .reg = (spiResource_t *)SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB3), GPIO_AF6 },
            { DEFIO_TAG_E(PC10), GPIO_AF6 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB4), GPIO_AF6 },
            { DEFIO_TAG_E(PC11), GPIO_AF6 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB2), GPIO_AF7 },
            { DEFIO_TAG_E(PB5), GPIO_AF7 },
            { DEFIO_TAG_E(PC12), GPIO_AF6 },
        },
        .rcc = RCC_APB1_2(SPI3),
    },
#endif
#ifdef USE_SPI_DEVICE_4
    {
        .device = SPIDEV_4,
        .reg = (spiResource_t *)SPI4,
        .sckPins = {
            { DEFIO_TAG_E(PE2), GPIO_AF5 },
            { DEFIO_TAG_E(PE12), GPIO_AF5 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PE5), GPIO_AF5 },
            { DEFIO_TAG_E(PE13), GPIO_AF5 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PE6), GPIO_AF5 },
            { DEFIO_TAG_E(PE14), GPIO_AF5 },
        },
        .rcc = RCC_APB5_1(SPI4),
    },
#endif
#ifdef USE_SPI_DEVICE_5
    {
        .device = SPIDEV_5,
        .reg = (spiResource_t *)SPI5,
        .sckPins = {
            { DEFIO_TAG_E(PF7), GPIO_AF5 },
            { DEFIO_TAG_E(PH6), GPIO_AF5 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PF8), GPIO_AF5 },
            { DEFIO_TAG_E(PH7), GPIO_AF5 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PF9), GPIO_AF5 },
            { DEFIO_TAG_E(PF11), GPIO_AF5 },
        },
        .rcc = RCC_APB5_1(SPI5),
    },
#endif
#ifdef USE_SPI_DEVICE_6
    {
        .device = SPIDEV_6,
        .reg = (spiResource_t *)SPI6,
        .sckPins = {
            { DEFIO_TAG_E(PA5), GPIO_AF8 },
            { DEFIO_TAG_E(PB3), GPIO_AF8 },
            { DEFIO_TAG_E(PG13), GPIO_AF5 },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA6), GPIO_AF8 },
            { DEFIO_TAG_E(PB4), GPIO_AF8 },
            { DEFIO_TAG_E(PG12), GPIO_AF5 },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA7), GPIO_AF8 },
            { DEFIO_TAG_E(PB5), GPIO_AF8 },
            { DEFIO_TAG_E(PG14), GPIO_AF5 },
        },
        .rcc = RCC_APB5_1(SPI6),
    },
#endif
};

void spiPinConfigure(const spiPinConfig_t *pConfig)
{
    memset(spiDevice, 0, sizeof(spiDevice));

    for (size_t hwindex = 0; hwindex < ARRAYLEN(spiHardware); hwindex++) {
        const spiHardware_t *hw = &spiHardware[hwindex];
        const spiDevice_e device = hw->device;
        spiDevice_t *pDev = &spiDevice[device];

        for (int pindex = 0; pindex < MAX_SPI_PIN_SEL; pindex++) {
            if (pConfig[device].ioTagSck == hw->sckPins[pindex].pin) {
                pDev->sck = hw->sckPins[pindex].pin;
                pDev->sckAF = hw->sckPins[pindex].af;
            }
            if (pConfig[device].ioTagMiso == hw->misoPins[pindex].pin) {
                pDev->miso = hw->misoPins[pindex].pin;
                pDev->misoAF = hw->misoPins[pindex].af;
            }
            if (pConfig[device].ioTagMosi == hw->mosiPins[pindex].pin) {
                pDev->mosi = hw->mosiPins[pindex].pin;
                pDev->mosiAF = hw->mosiPins[pindex].af;
            }
        }

        if (pDev->sck && pDev->miso && pDev->mosi) {
            pDev->dev = hw->reg;
            pDev->rcc = hw->rcc;
            pDev->leadingEdge = false;
        }
    }
}

#endif
