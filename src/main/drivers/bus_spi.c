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
#include "drivers/nvic.h"
#include "pg/bus_spi.h"

#define NUM_QUEUE_SEGS 5

static uint8_t spiRegisteredDeviceCount = 0;

spiDevice_t spiDevice[SPIDEV_COUNT];
busDevice_t spiBusDevice[SPIDEV_COUNT];

SPIDevice spiDeviceByInstance(const SPI_TypeDef *instance)
{
#ifdef USE_SPI_DEVICE_0
    if (instance == SPI0) {
        return SPIDEV_0;
    }
#endif

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

#if !defined(USE_SPI_DEVICE_1)
    case SPIDEV_1:
#endif

#if !defined(USE_SPI_DEVICE_2)
    case SPIDEV_2:
#endif

#if !defined(USE_SPI_DEVICE_3)
    case SPIDEV_3:
#endif

#if !defined(USE_SPI_DEVICE_4)
    case SPIDEV_4:
#endif

#if !defined(USE_SPI_DEVICE_5)
    case SPIDEV_5:
#endif

#if !defined(USE_SPI_DEVICE_6)
    case SPIDEV_6:
#endif
        return false;
    default:
        spiInitDevice(device);
        return true;
    }
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
    while (spiIsBusy(dev));
}

// Negate CS if held asserted after a transfer
void spiRelease(const extDevice_t *dev)
{
    // Negate Chip Select
    IOHi(dev->busType_u.spi.csnPin);
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
#ifdef USE_DMA
    bus->dmaInitTx = &dev->dmaInitTx;
    bus->dmaInitRx = &dev->dmaInitRx;
#endif

    return true;
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

#ifdef USE_DMA
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

bool spiUseSDO_DMA(const extDevice_t *dev)
{
    return dev->bus->useDMA && dev->useDMA;
}
#endif

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

// Link two segment lists
// Note that there is no need to unlink segment lists as this is done automatically as they are processed
void spiLinkSegments(const extDevice_t *dev, busSegment_t *firstSegment, busSegment_t *secondSegment)
{
    busSegment_t *endSegment;

    // Find the last segment of the new transfer
    for (endSegment = firstSegment; endSegment->len; endSegment++);

    endSegment->u.link.dev = dev;
    endSegment->u.link.segments = secondSegment;
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

                // Record the dev and segments parameters in the terminating segment entry
                endCmpSegment->u.link.dev = dev;
                endCmpSegment->u.link.segments = segments;
            }

            return;
        } else {
            // Claim the bus with this list of segments
            bus->curSegment = segments;
        }
    }

    spiSequenceStart(dev);
}

// Process segments using DMA - expects DMA irq handler to have been set up to feed into spiIrqHandler.
FAST_CODE void spiProcessSegmentsDMA(const extDevice_t *dev)
{
    // Intialise the init structures for the first transfer
    spiInternalInitStream(dev, dev->bus->curSegment);

    // Assert Chip Select
    IOLo(dev->busType_u.spi.csnPin);

    // Start the transfers
    spiInternalStartDMA(dev);
}

static void spiPreInitStream(const extDevice_t *dev)
{
    // Prepare the init structure for the next segment to reduce inter-segment interval
    // (if it's a "buffers" segment, not a "link" segment).
    busSegment_t *segment = (busSegment_t *)dev->bus->curSegment + 1;
    if (segment->len > 0) {
        spiInternalInitStream(dev, segment);
    }
}

// Interrupt handler common code for SPI receive DMA completion.
// Proceed to next segment as required.
FAST_IRQ_HANDLER void spiIrqHandler(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    busSegment_t *nextSegment;

    if (bus->curSegment->callback) {
        switch(bus->curSegment->callback(dev->callbackArg)) {
        case BUS_BUSY:
            // Repeat the last DMA segment
            bus->curSegment--;
            // Reinitialise the cached init values as segment is not progressing
            spiPreInitStream(dev);
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
            spiInternalInitStream(dev, bus->curSegment);
            bus->initSegment = false;
        }

        if (negateCS) {
            // Assert Chip Select - it's costly so only do so if necessary
            IOLo(dev->busType_u.spi.csnPin);
        }

        // Launch the next transfer
        spiInternalStartDMA(dev);

        // Prepare the init structures ready for the next segment to reduce inter-segment time
        spiPreInitStream(dev);
    }
}

FAST_CODE void spiProcessSegmentsPolled(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    busSegment_t *lastSegment = NULL;
    bool segmentComplete;

    // Manually work through the segment list performing a transfer for each
    while (bus->curSegment->len) {
        if (!lastSegment || lastSegment->negateCS) {
            // Assert Chip Select if necessary - it's costly so only do so if necessary
            IOLo(dev->busType_u.spi.csnPin);
        }

        spiInternalReadWriteBufPolled(
            bus->busType_u.spi.instance,
            bus->curSegment->u.buffers.txData,
            bus->curSegment->u.buffers.rxData,
            bus->curSegment->len);

        if (bus->curSegment->negateCS) {
            // Negate Chip Select
            IOHi(dev->busType_u.spi.csnPin);
        }

        segmentComplete = true;
        if (bus->curSegment->callback) {
            switch(bus->curSegment->callback(dev->callbackArg)) {
            case BUS_BUSY:
                // Repeat the last DMA segment
                segmentComplete = false;
                break;

            case BUS_ABORT:
                bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
                segmentComplete = false;
                return;

            case BUS_READY:
            default:
                // Advance to the next DMA segment
                break;
            }
        }
        if (segmentComplete) {
            lastSegment = (busSegment_t *)bus->curSegment;
            bus->curSegment++;
        }
    }

    // If a following transaction has been linked, start it
    if (bus->curSegment->u.link.dev) {
        busSegment_t *endSegment = (busSegment_t *)bus->curSegment;
        const extDevice_t *nextDev = endSegment->u.link.dev;
        busSegment_t *nextSegments = (busSegment_t *)endSegment->u.link.segments;
        bus->curSegment = nextSegments;
        endSegment->u.link.dev = NULL;
        endSegment->u.link.segments = NULL;
        spiSequenceStart(nextDev);
    } else {
        // The end of the segment list has been reached, so mark transactions as complete
        bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
    }
}

#endif
