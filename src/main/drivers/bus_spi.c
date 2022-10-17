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

#include "build/atomic.h"

#ifdef USE_SPI

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma_reqmap.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/rcc.h"
#include "nvic.h"
#include "pg/bus_spi.h"

#define NUM_QUEUE_SEGS 5

static uint8_t spiRegisteredDeviceCount = 0;

spiDevice_t spiDevice[SPIDEV_COUNT];
busDevice_t spiBusDevice[SPIDEV_COUNT];

SPIDevice spiDeviceByInstance(SPI_TypeDef *instance)
{
#ifdef USE_SPI_DEVICE_1
    if (instance == SPI1) {
        return SPIDEV_1;
    }
#endif

#ifdef USE_SPI_DEVICE_2
    if (instance == SPI2) {
        return SPIDEV_2;
    }
#endif

#ifdef USE_SPI_DEVICE_3
    if (instance == SPI3) {
        return SPIDEV_3;
    }
#endif

#ifdef USE_SPI_DEVICE_4
    if (instance == SPI4) {
        return SPIDEV_4;
    }
#endif

#ifdef USE_SPI_DEVICE_5
    if (instance == SPI5) {
        return SPIDEV_5;
    }
#endif

#ifdef USE_SPI_DEVICE_6
    if (instance == SPI6) {
        return SPIDEV_6;
    }
#endif

    return SPIINVALID;
}

SPI_TypeDef *spiInstanceByDevice(SPIDevice device)
{
    if (device == SPIINVALID || device >= SPIDEV_COUNT) {
        return NULL;
    }

    return spiDevice[device].dev;
}

bool spiInit(SPIDevice device)
{
    switch (device) {
    case SPIINVALID:
        return false;

    case SPIDEV_1:
#ifdef USE_SPI_DEVICE_1
        spiInitDevice(device);
        return true;
#else
        break;
#endif

    case SPIDEV_2:
#ifdef USE_SPI_DEVICE_2
        spiInitDevice(device);
        return true;
#else
        break;
#endif

    case SPIDEV_3:
#if defined(USE_SPI_DEVICE_3)
        spiInitDevice(device);
        return true;
#else
        break;
#endif

    case SPIDEV_4:
#if defined(USE_SPI_DEVICE_4)
        spiInitDevice(device);
        return true;
#else
        break;
#endif

    case SPIDEV_5:
#if defined(USE_SPI_DEVICE_5)
        spiInitDevice(device);
        return true;
#else
        break;
#endif

    case SPIDEV_6:
#if defined(USE_SPI_DEVICE_6)
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    }
    return false;
}

// Return true if DMA engine is busy
bool spiIsBusy(const extDevice_t *dev)
{
    return (dev->bus->curSegment != (busSegment_t *)BUS_SPI_FREE);
}

// Wait for DMA completion
void spiWait(const extDevice_t *dev)
{
    // Wait for completion
    while (dev->bus->curSegment != (busSegment_t *)BUS_SPI_FREE);
}

// Wait for bus to become free, then read/write block of data
void spiReadWriteBuf(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int len)
{
    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {.u.buffers = {txData, rxData}, len, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    spiSequence(dev, &segments[0]);

    spiWait(dev);
}

// Read/Write a block of data, returning false if the bus is busy
bool spiReadWriteBufRB(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int length)
{
    // Ensure any prior DMA has completed before continuing
    if (spiIsBusy(dev)) {
        return false;
    }

    spiReadWriteBuf(dev, txData, rxData, length);

    return true;
}

// Wait for bus to become free, then read/write a single byte
uint8_t spiReadWrite(const extDevice_t *dev, uint8_t data)
{
    uint8_t retval;

    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {.u.buffers = {&data, &retval}, sizeof(data), true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    spiSequence(dev, &segments[0]);

    spiWait(dev);

    return retval;
}

// Wait for bus to become free, then read/write a single byte from a register
uint8_t spiReadWriteReg(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    uint8_t retval;

    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {.u.buffers = {&reg, NULL}, sizeof(reg), false, NULL},
            {.u.buffers = {&data, &retval}, sizeof(data), true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    spiSequence(dev, &segments[0]);

    spiWait(dev);

    return retval;
}

// Wait for bus to become free, then write a single byte
void spiWrite(const extDevice_t *dev, uint8_t data)
{
    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {.u.buffers = {&data, NULL}, sizeof(data), true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    spiSequence(dev, &segments[0]);

    spiWait(dev);
}

// Write data to a register
void spiWriteReg(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {.u.buffers = {&reg, NULL}, sizeof(reg), false, NULL},
            {.u.buffers = {&data, NULL}, sizeof(data), true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    spiSequence(dev, &segments[0]);

    spiWait(dev);
}

// Write data to a register, returning false if the bus is busy
bool spiWriteRegRB(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    // Ensure any prior DMA has completed before continuing
    if (spiIsBusy(dev)) {
        return false;
    }

    spiWriteReg(dev, reg, data);

    return true;
}

// Read a block of data from a register
void spiReadRegBuf(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {.u.buffers = {&reg, NULL}, sizeof(reg), false, NULL},
            {.u.buffers = {NULL, data}, length, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    spiSequence(dev, &segments[0]);

    spiWait(dev);
}

// Read a block of data from a register, returning false if the bus is busy
bool spiReadRegBufRB(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    // Ensure any prior DMA has completed before continuing
    if (spiIsBusy(dev)) {
        return false;
    }

    spiReadRegBuf(dev, reg, data, length);

    return true;
}

// Read a block of data where the register is ORed with 0x80, returning false if the bus is busy
bool spiReadRegMskBufRB(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    return spiReadRegBufRB(dev, reg | 0x80, data, length);
}

// Wait for bus to become free, then write a block of data to a register
void spiWriteRegBuf(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint32_t length)
{
    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {.u.buffers = {&reg, NULL}, sizeof(reg), false, NULL},
            {.u.buffers = {data, NULL}, length, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    spiSequence(dev, &segments[0]);

    spiWait(dev);
}

// Wait for bus to become free, then read a byte from a register
uint8_t spiReadReg(const extDevice_t *dev, uint8_t reg)
{
    uint8_t data;
    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {.u.buffers = {&reg, NULL}, sizeof(reg), false, NULL},
            {.u.buffers = {NULL, &data}, sizeof(data), true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    spiSequence(dev, &segments[0]);

    spiWait(dev);

    return data;
}

// Wait for bus to become free, then read a byte of data where the register is ORed with 0x80
uint8_t spiReadRegMsk(const extDevice_t *dev, uint8_t reg)
{
    return spiReadReg(dev, reg | 0x80);
}

uint16_t spiCalculateDivider(uint32_t freq)
{
#if defined(STM32F4) || defined(STM32G4) || defined(STM32F7)
    uint32_t spiClk = SystemCoreClock / 2;
#elif defined(STM32H7)
    uint32_t spiClk = 100000000;
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
#if defined(STM32F4) || defined(STM32G4) || defined(STM32F7)
    uint32_t spiClk = SystemCoreClock / 2;
#elif defined(STM32H7)
    uint32_t spiClk = 100000000;
#else
#error "Base SPI clock not defined for this architecture"
#endif

    return spiClk / spiClkDivisor;
}

// Interrupt handler for SPI receive DMA completion
static void spiIrqHandler(const extDevice_t *dev)
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
            bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
            return;

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
static void spiRxIrqHandler(dmaChannelDescriptor_t* descriptor)
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

#if !defined(STM32G4) && !defined(STM32H7)
// Interrupt handler for SPI transmit DMA completion
static void spiTxIrqHandler(dmaChannelDescriptor_t* descriptor)
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

// Mark this bus as being SPI and record the first owner to use it
bool spiSetBusInstance(extDevice_t *dev, uint32_t device)
{
    if ((device == 0) || (device > SPIDEV_COUNT)) {
        return false;
    }

    dev->bus = &spiBusDevice[SPI_CFG_TO_DEV(device)];

    // By default each device should use SPI DMA if the bus supports it
    dev->useDMA = true;

    if (dev->bus->busType == BUS_TYPE_SPI) {
        // This bus has already been initialised
        dev->bus->deviceCount++;
        return true;
    }

    busDevice_t *bus = dev->bus;

    bus->busType_u.spi.instance = spiInstanceByDevice(SPI_CFG_TO_DEV(device));

    if (bus->busType_u.spi.instance == NULL) {
        return false;
    }

    bus->busType = BUS_TYPE_SPI;
    bus->useDMA = false;
    bus->deviceCount = 1;
    bus->initTx = &dev->initTx;
    bus->initRx = &dev->initRx;

    return true;
}

void spiInitBusDMA(void)
{
    uint32_t device;
#if defined(STM32F4) && defined(USE_DSHOT_BITBANG)
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
            const dmaChannelSpec_t *dmaTxChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_MOSI, device, opt);

            if (dmaTxChannelSpec) {
                dmaTxIdentifier = dmaGetIdentifier(dmaTxChannelSpec->ref);
#if defined(STM32F4) && defined(USE_DSHOT_BITBANG)
                if (dshotBitbangActive && (DMA_DEVICE_NO(dmaTxIdentifier) == 2)) {
                    dmaTxIdentifier = DMA_NONE;
                    break;
                }
#endif
                if (!dmaAllocate(dmaTxIdentifier, OWNER_SPI_MOSI, device + 1)) {
                    dmaTxIdentifier = DMA_NONE;
                    continue;
                }
                bus->dmaTx = dmaGetDescriptorByIdentifier(dmaTxIdentifier);
                bus->dmaTx->stream = DMA_DEVICE_INDEX(dmaTxIdentifier);
                bus->dmaTx->channel = dmaTxChannelSpec->channel;

                dmaEnable(dmaTxIdentifier);

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
            const dmaChannelSpec_t *dmaRxChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_MISO, device, opt);

            if (dmaRxChannelSpec) {
                dmaRxIdentifier = dmaGetIdentifier(dmaRxChannelSpec->ref);
#if defined(STM32F4) && defined(USE_DSHOT_BITBANG)
                if (dshotBitbangActive && (DMA_DEVICE_NO(dmaRxIdentifier) == 2)) {
                    dmaRxIdentifier = DMA_NONE;
                    break;
                }
#endif
                if (!dmaAllocate(dmaRxIdentifier, OWNER_SPI_MISO, device + 1)) {
                    dmaRxIdentifier = DMA_NONE;
                    continue;
                }
                bus->dmaRx = dmaGetDescriptorByIdentifier(dmaRxIdentifier);
                bus->dmaRx->stream = DMA_DEVICE_INDEX(dmaRxIdentifier);
                bus->dmaRx->channel = dmaRxChannelSpec->channel;

                dmaEnable(dmaRxIdentifier);

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
#if !defined(STM32G4) && !defined(STM32H7)
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

void spiSetClkDivisor(const extDevice_t *dev, uint16_t divisor)
{
    ((extDevice_t *)dev)->busType_u.spi.speed = divisor;
}

// Set the clock phase/polarity to be used for accesses by the given device
void spiSetClkPhasePolarity(const extDevice_t *dev, bool leadingEdge)
{
    ((extDevice_t *)dev)->busType_u.spi.leadingEdge = leadingEdge;
}

// Enable/disable DMA on a specific device. Enabled by default.
void spiDmaEnable(const extDevice_t *dev, bool enable)
{
    ((extDevice_t *)dev)->useDMA = enable;
}

bool spiUseDMA(const extDevice_t *dev)
{
    // Full DMA only requires both transmit and receive}
    return dev->bus->useDMA && dev->bus->dmaRx && dev->useDMA;
}

bool spiUseMOSI_DMA(const extDevice_t *dev)
{
    return dev->bus->useDMA && dev->useDMA;
}

void spiBusDeviceRegister(const extDevice_t *dev)
{
    UNUSED(dev);

    spiRegisteredDeviceCount++;
}

uint8_t spiGetRegisteredDeviceCount(void)
{
    return spiRegisteredDeviceCount;
}

uint8_t spiGetExtDeviceCount(const extDevice_t *dev)
{
    return dev->bus->deviceCount;
}

// DMA transfer setup and start
void spiSequence(const extDevice_t *dev, busSegment_t *segments)
{
    busDevice_t *bus = dev->bus;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (spiIsBusy(dev)) {
            busSegment_t *endSegment;

            // Defer this transfer to be triggered upon completion of the current transfer

            // Find the last segment of the new transfer
            for (endSegment = segments; endSegment->len; endSegment++);

            // Safe to discard the volatile qualifier as we're in an atomic block
            busSegment_t *endCmpSegment = (busSegment_t *)bus->curSegment;

            if (endCmpSegment) {
                while (true) {
                    // Find the last segment of the current transfer
                    for (; endCmpSegment->len; endCmpSegment++);

                    if (endCmpSegment == endSegment) {
                        /* Attempt to use the new segment list twice in the same queue. Abort.
                         * Note that this can only happen with non-blocking transfers so drivers must take
                         * care to avoid this.
                         * */
                        return;
                    }

                    if (endCmpSegment->u.link.dev == NULL) {
                        // End of the segment list queue reached
                        break;
                    } else {
                        // Follow the link to the next queued segment list
                        endCmpSegment = (busSegment_t *)endCmpSegment->u.link.segments;
                    }
                }
            }

            // Record the dev and segments parameters in the terminating segment entry
            endCmpSegment->u.link.dev = dev;
            endCmpSegment->u.link.segments = segments;

            return;
        } else {
            // Claim the bus with this list of segments
            bus->curSegment = segments;
        }
    }

    spiSequenceStart(dev);
}
#endif
