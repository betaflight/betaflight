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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/atomic.h"

#ifdef USE_SPI

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma_reqmap.h"
#include "drivers/dshot.h"
#include "drivers/nvic.h"
#include "pg/bus_spi.h"

extern spiDevice_t spiDevice[SPIDEV_COUNT];
extern busDevice_t spiBusDevice[SPIDEV_COUNT];

// Interrupt handler for SPI receive DMA completion
FAST_IRQ_HANDLER static void spiIrqHandler(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    busSegment_t *nextSegment;

    if (bus->curSegment->callback) {
        switch(bus->curSegment->callback(dev->callbackArg)) {
        case BUS_BUSY:
            // Repeat the last DMA segment
            bus->curSegment--;
            // Reinitialise the cached init values as segment is not progressing
            spiInternalInitStream(dev, true);
            break;

        case BUS_ABORT:
            // Skip to the end of the segment list
            nextSegment = (busSegment_t *)bus->curSegment + 1;
            while (nextSegment->len != 0) {
                bus->curSegment = nextSegment;
                nextSegment = (busSegment_t *)bus->curSegment + 1;
            }
            break;

        case BUS_READY:
        default:
            // Advance to the next DMA segment
            break;
        }
    }

    // Advance through the segment list
    // OK to discard the volatile qualifier here
    nextSegment = (busSegment_t *)bus->curSegment + 1;

    if (nextSegment->len == 0) {
        // If a following transaction has been linked, start it
        if (nextSegment->u.link.dev) {
            const extDevice_t *nextDev = nextSegment->u.link.dev;
            busSegment_t *nextSegments = (busSegment_t *)nextSegment->u.link.segments;
            // The end of the segment list has been reached
            bus->curSegment = nextSegments;
            nextSegment->u.link.dev = NULL;
            nextSegment->u.link.segments = NULL;
            spiSequenceStart(nextDev);
        } else {
            // The end of the segment list has been reached, so mark transactions as complete
            bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
        }
    } else {
        // Do as much processing as possible before asserting CS to avoid violating minimum high time
        bool negateCS = bus->curSegment->negateCS;

        bus->curSegment = nextSegment;

        // After the completion of the first segment setup the init structure for the subsequent segment
        if (bus->initSegment) {
            spiInternalInitStream(dev, false);
            bus->initSegment = false;
        }

        if (negateCS) {
            // Assert Chip Select - it's costly so only do so if necessary
            IOLo(dev->busType_u.spi.csnPin);
        }

        // Launch the next transfer
        spiInternalStartDMA(dev);

        // Prepare the init structures ready for the next segment to reduce inter-segment time
        spiInternalInitStream(dev, true);
    }
}

// Interrupt handler for SPI receive DMA completion
FAST_IRQ_HANDLER static void spiRxIrqHandler(dmaChannelDescriptor_t* descriptor)
{
    const extDevice_t *dev = (const extDevice_t *)descriptor->userParam;

    if (!dev) {
        return;
    }

    busDevice_t *bus = dev->bus;

    if (bus->curSegment->negateCS) {
        // Negate Chip Select
        IOHi(dev->busType_u.spi.csnPin);
    }

    spiInternalStopDMA(dev);

#ifdef __DCACHE_PRESENT
#ifdef STM32H7
    if (bus->curSegment->u.buffers.rxData &&
        ((bus->curSegment->u.buffers.rxData < &_dmaram_start__) || (bus->curSegment->u.buffers.rxData >= &_dmaram_end__))) {
#else
    if (bus->curSegment->u.buffers.rxData) {
#endif
         // Invalidate the D cache covering the area into which data has been read
        SCB_InvalidateDCache_by_Addr(
            (uint32_t *)((uint32_t)bus->curSegment->u.buffers.rxData & ~CACHE_LINE_MASK),
            (((uint32_t)bus->curSegment->u.buffers.rxData & CACHE_LINE_MASK) +
              bus->curSegment->len - 1 + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
    }
#endif // __DCACHE_PRESENT

    spiIrqHandler(dev);
}

#ifdef USE_TX_IRQ_HANDLER
// Interrupt handler for SPI transmit DMA completion
FAST_IRQ_HANDLER static void spiTxIrqHandler(dmaChannelDescriptor_t* descriptor)
{
    const extDevice_t *dev = (const extDevice_t *)descriptor->userParam;

    if (!dev) {
        return;
    }

    busDevice_t *bus = dev->bus;

    spiInternalStopDMA(dev);

    if (bus->curSegment->negateCS) {
        // Negate Chip Select
        IOHi(dev->busType_u.spi.csnPin);
    }

    spiIrqHandler(dev);
}
#endif

uint16_t spiCalculateDivider(uint32_t freq)
{
#if defined(STM32F4) || defined(STM32F7) || defined(APM32F4)
    uint32_t spiClk = SystemCoreClock / 2;
#elif defined(STM32H7)
    uint32_t spiClk = 100000000;
#elif defined(STM32G4)
    uint32_t spiClk = SystemCoreClock;
#elif defined(AT32F4)
    if(freq > 36000000){
        freq = 36000000;
    }

    uint32_t spiClk = system_core_clock / 2;
#else
#error "Base SPI clock not defined for this architecture"
#endif

    uint16_t divisor = 2;

    spiClk >>= 1;

    for (; (spiClk > freq) && (divisor < 256); divisor <<= 1, spiClk >>= 1);

    return divisor;
}

uint32_t spiCalculateClock(uint16_t spiClkDivisor)
{
#if defined(STM32F4) || defined(STM32G4) || defined(STM32F7) || defined(APM32F4)
    uint32_t spiClk = SystemCoreClock / 2;
#elif defined(STM32H7)
    uint32_t spiClk = 100000000;
#elif defined(AT32F4)
    uint32_t spiClk = system_core_clock / 2;

    if ((spiClk / spiClkDivisor) > 36000000){
        return 36000000;
    }
#else
#error "Base SPI clock not defined for this architecture"
#endif

    return spiClk / spiClkDivisor;
}

void spiInitBusDMA(void)
{
    uint32_t device;
#if (defined(STM32F4) || defined(APM32F4)) && defined(USE_DSHOT_BITBANG)
    /* Check https://www.st.com/resource/en/errata_sheet/dm00037591-stm32f405407xx-and-stm32f415417xx-device-limitations-stmicroelectronics.pdf
     * section 2.1.10 which reports an errata that corruption may occurs on DMA2 if AHB peripherals (eg GPIO ports) are
     * access concurrently with APB peripherals (eg SPI busses). Bitbang DSHOT uses DMA2 to write to GPIO ports. If this
     * is enabled, then don't enable DMA on an SPI bus using DMA2
     */
    const bool dshotBitbangActive = isDshotBitbangActive(&motorConfig()->dev);
#endif

    for (device = 0; device < SPIDEV_COUNT; device++) {
        busDevice_t *bus = &spiBusDevice[device];

        if (bus->busType != BUS_TYPE_SPI) {
            // This bus is not in use
            continue;
        }

        dmaIdentifier_e dmaTxIdentifier = DMA_NONE;
        dmaIdentifier_e dmaRxIdentifier = DMA_NONE;

        int8_t txDmaopt = spiPinConfig(device)->txDmaopt;
        uint8_t txDmaoptMin = 0;
        uint8_t txDmaoptMax = MAX_PERIPHERAL_DMA_OPTIONS - 1;

        if (txDmaopt != -1) {
            txDmaoptMin = txDmaopt;
            txDmaoptMax = txDmaopt;
        }

        for (uint8_t opt = txDmaoptMin; opt <= txDmaoptMax; opt++) {
            const dmaChannelSpec_t *dmaTxChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_SDO, device, opt);

            if (dmaTxChannelSpec) {
                dmaTxIdentifier = dmaGetIdentifier(dmaTxChannelSpec->ref);
#if (defined(STM32F4) || defined(APM32F4)) && defined(USE_DSHOT_BITBANG)
                if (dshotBitbangActive && (DMA_DEVICE_NO(dmaTxIdentifier) == 2)) {
                    dmaTxIdentifier = DMA_NONE;
                    break;
                }
#endif
                if (!dmaAllocate(dmaTxIdentifier, OWNER_SPI_SDO, device + 1)) {
                    dmaTxIdentifier = DMA_NONE;
                    continue;
                }
                bus->dmaTx = dmaGetDescriptorByIdentifier(dmaTxIdentifier);
#if defined(DMA_TRAIT_SPI_STREAM)
                bus->dmaTx->stream = DMA_DEVICE_INDEX(dmaTxIdentifier);
                bus->dmaTx->channel = dmaTxChannelSpec->channel;
#endif

                dmaEnable(dmaTxIdentifier);
#if defined(USE_ATBSP_DRIVER)
                dmaMuxEnable(dmaTxIdentifier,dmaTxChannelSpec->dmaMuxId);
#endif
                break;
            }
        }

        int8_t rxDmaopt = spiPinConfig(device)->rxDmaopt;
        uint8_t rxDmaoptMin = 0;
        uint8_t rxDmaoptMax = MAX_PERIPHERAL_DMA_OPTIONS - 1;

        if (rxDmaopt != -1) {
            rxDmaoptMin = rxDmaopt;
            rxDmaoptMax = rxDmaopt;
        }

        for (uint8_t opt = rxDmaoptMin; opt <= rxDmaoptMax; opt++) {
            const dmaChannelSpec_t *dmaRxChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_SDI, device, opt);

            if (dmaRxChannelSpec) {
                dmaRxIdentifier = dmaGetIdentifier(dmaRxChannelSpec->ref);
#if (defined(STM32F4) || defined(APM32F4)) && defined(USE_DSHOT_BITBANG)
                if (dshotBitbangActive && (DMA_DEVICE_NO(dmaRxIdentifier) == 2)) {
                    dmaRxIdentifier = DMA_NONE;
                    break;
                }
#endif
                if (!dmaAllocate(dmaRxIdentifier, OWNER_SPI_SDI, device + 1)) {
                    dmaRxIdentifier = DMA_NONE;
                    continue;
                }
                bus->dmaRx = dmaGetDescriptorByIdentifier(dmaRxIdentifier);
#if defined(DMA_TRAIT_SPI_STREAM)
                bus->dmaRx->stream = DMA_DEVICE_INDEX(dmaRxIdentifier);
                bus->dmaRx->channel = dmaRxChannelSpec->channel;
#endif

                dmaEnable(dmaRxIdentifier);
#if defined(USE_ATBSP_DRIVER)
                dmaMuxEnable(dmaRxIdentifier,dmaRxChannelSpec->dmaMuxId);
#endif
                break;
            }
        }

        if (dmaTxIdentifier && dmaRxIdentifier) {
            // Ensure streams are disabled
            spiInternalResetStream(bus->dmaRx);
            spiInternalResetStream(bus->dmaTx);

            spiInternalResetDescriptors(bus);

            /* Note that this driver may be called both from the normal thread of execution, or from USB interrupt
             * handlers, so the DMA completion interrupt must be at a higher priority
             */
            dmaSetHandler(dmaRxIdentifier, spiRxIrqHandler, NVIC_PRIO_SPI_DMA, 0);

            bus->useDMA = true;
#ifdef USE_TX_IRQ_HANDLER
        } else if (dmaTxIdentifier) {
            // Transmit on DMA is adequate for OSD so worth having
            bus->dmaTx = dmaGetDescriptorByIdentifier(dmaTxIdentifier);
            bus->dmaRx = (dmaChannelDescriptor_t *)NULL;

            // Ensure streams are disabled
            spiInternalResetStream(bus->dmaTx);

            spiInternalResetDescriptors(bus);

            dmaSetHandler(dmaTxIdentifier, spiTxIrqHandler, NVIC_PRIO_SPI_DMA, 0);

            bus->useDMA = true;
#endif
        } else {
            // Disassociate channels from bus
            bus->dmaRx = (dmaChannelDescriptor_t *)NULL;
            bus->dmaTx = (dmaChannelDescriptor_t *)NULL;
        }
    }
}

#endif
