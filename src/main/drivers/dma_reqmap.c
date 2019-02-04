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

#include <stdint.h>

#include "platform.h"

#ifdef USE_DMA_SPEC

#include "drivers/adc.h"
#include "drivers/bus_spi.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "drivers/dma_reqmap.h"

typedef struct dmaRequestMapping_s {
    dmaPeripheral_e device;
    uint8_t index;
    dmaChannelSpec_t channelSpec[2];
} dmaRequestMapping_t;

#if defined(STM32F4) || defined(STM32F7)

#if defined(STM32F4)
#define D(d, s, c) { DMA_CODE(d, s, c), DMA ## d ## _Stream ## s, DMA_Channel_ ## c }
#elif defined(STM32F7)
#define D(d, s, c) { DMA_CODE(d, s, c), DMA ## d ## _Stream ## s, DMA_CHANNEL_ ## c }
#endif

static const dmaRequestMapping_t dmaRequestMapping[] = {
#ifdef USE_SPI
    // Everything including F405 and F446
    { DMA_PERIPH_SPI_TX,  SPIDEV_1,  { D(2, 3, 3), D(2, 5, 3) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_1,  { D(2, 0, 3), D(2, 2, 3) } },
    { DMA_PERIPH_SPI_TX,  SPIDEV_2,  { D(1, 4, 0) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_2,  { D(1, 3, 0) } },
    { DMA_PERIPH_SPI_TX,  SPIDEV_3,  { D(1, 5, 0), D(1, 7, 0) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_3,  { D(1, 0, 0), D(1, 2, 0) } },

#if defined(STM32F411xE) || defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F765xx) || defined(STM32F722xx)
    { DMA_PERIPH_SPI_TX,  SPIDEV_4,  { D(2, 1, 4) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_4,  { D(2, 0, 4) } },

#ifdef USE_EXTENDED_SPI_DEVICE
    { DMA_PERIPH_SPI_TX,  SPIDEV_5,  { D(2, 6, 7) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_5,  { D(2, 5, 7) } },

#if !defined(STM32F722xx)
    { DMA_PERIPH_SPI_TX,  SPIDEV_6,  { D(2, 5, 1) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_6,  { D(2, 6, 1) } },
#endif
#endif // USE_EXTENDED_SPI_DEVICE
#endif
#endif // USE_SPI

#ifdef USE_ADC
    { DMA_PERIPH_ADC,     ADCDEV_1,  { D(2, 0, 0), D(2, 4, 0) } },
    { DMA_PERIPH_ADC,     ADCDEV_2,  { D(2, 2, 1), D(2, 3, 1) } },
    { DMA_PERIPH_ADC,     ADCDEV_3,  { D(2, 0, 2), D(2, 1, 2) } },
#endif

#ifdef USE_SDCARD_SDIO
    { DMA_PERIPH_SDIO,    0,         { D(2, 3, 4), D(2, 6, 4) } },
#endif

#ifdef USE_UART
    { DMA_PERIPH_UART_TX, UARTDEV_1, { D(2, 7, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_1, { D(2, 5, 4), D(2, 2, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_2, { D(1, 6, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_2, { D(1, 5, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_3, { D(1, 3, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_3, { D(1, 1, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_4, { D(1, 4, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_4, { D(1, 2, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_5, { D(1, 7, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_5, { D(1, 0, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_6, { D(2, 6, 5), D(2, 7, 5) } },
    { DMA_PERIPH_UART_RX, UARTDEV_6, { D(2, 1, 5), D(2, 2, 5) } },
#endif
};
#undef D
#else // STM32F3
    // The embedded ADC24_DMA_REMAP conditional should be removed
    // when (and if) F3 is going generic.
#define D(d, c) { DMA_CODE(d, 0, c), DMA ## d ## _Channel ## c }
static const dmaRequestMapping_t dmaRequestMapping[17] = {
#ifdef USE_SPI
    { DMA_PERIPH_SPI_TX,  1, { D(1, 3) } },
    { DMA_PERIPH_SPI_RX,  1, { D(1, 2) } },
    { DMA_PERIPH_SPI_TX,  2, { D(1, 5) } },
    { DMA_PERIPH_SPI_RX,  2, { D(1, 4) } },
    { DMA_PERIPH_SPI_TX,  3, { D(2, 2) } },
    { DMA_PERIPH_SPI_RX,  3, { D(2, 1) } },
#endif

#ifdef USE_ADC
    { DMA_PERIPH_ADC,     1, { D(1, 1) } },
#ifdef ADC24_DMA_REMAP
    { DMA_PERIPH_ADC,     2, { D(2, 3) } },
#else
    { DMA_PERIPH_ADC,     2, { D(2, 1) } },
#endif
    { DMA_PERIPH_ADC,     3, { D(2, 5) } },
#endif

#ifdef USE_UART
    { DMA_PERIPH_UART_TX, 1, { D(1, 4) } },
    { DMA_PERIPH_UART_RX, 1, { D(1, 5) } },

    { DMA_PERIPH_UART_TX, 2, { D(1, 7) } },
    { DMA_PERIPH_UART_RX, 2, { D(1, 6) } },
    { DMA_PERIPH_UART_TX, 3, { D(1, 2) } },
    { DMA_PERIPH_UART_RX, 3, { D(1, 3) } },
    { DMA_PERIPH_UART_TX, 4, { D(2, 5) } },
    { DMA_PERIPH_UART_RX, 4, { D(2, 3) } },
};
#endif
#undef D
#endif

const dmaChannelSpec_t *dmaGetChannelSpec(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    if (opt < 0 || opt >= 2) {
        return NULL;
    }

    for (unsigned i = 0 ; i < ARRAYLEN(dmaRequestMapping) ; i++) {
        const dmaRequestMapping_t *periph = &dmaRequestMapping[i];
        if (periph->device == device && periph->index == index && periph->channelSpec[opt].ref) {
            return &periph->channelSpec[opt];
        }
    }

    return NULL;
}
#endif // USE_DMA_SPEC
