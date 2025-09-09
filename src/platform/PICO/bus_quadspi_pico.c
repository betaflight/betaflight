/*
 * PICO RP2350 QUADSPI adapter implementing Betaflight quadSpi API via Pico SDK QMI controller.
 */

#include "platform.h"

#ifdef USE_QUADSPI

#include <string.h>

#include "drivers/bus_quadspi.h"
#include "drivers/bus_quadspi_impl.h"
#include "pg/bus_quadspi.h"
#include "drivers/io.h"

#include "hardware/flash.h"
#include "pico/bootrom.h"
#include "hardware/regs/qmi.h"
#include "hardware/structs/qmi.h"
#include "hardware/regs/xip_aux.h"
#include "hardware/structs/xip_aux.h"
#include "hardware/xip_cache.h"
#include "drivers/time.h"
#include "drivers/bus.h"
#include "drivers/dma.h"
#include "drivers/dma_impl.h"
#include "drivers/nvic.h"

#include "hardware/dma.h"
#include "hardware/regs/dreq.h"

// Provide platform QUADSPI hardware table for RP2350 (single logical device)
const quadSpiHardware_t quadSpiHardware[QUADSPIDEV_COUNT] = {
    {
        .device = QUADSPIDEV_1,
        .reg = (QUADSPI_TypeDef *)0x1, // sentinel; common layer uses it only for identity
        .clkPins = { { 0 } },
        .bk1IO0Pins = { { 0 } },
        .bk1IO1Pins = { { 0 } },
        .bk1IO2Pins = { { 0 } },
        .bk1IO3Pins = { { 0 } },
        .bk1CSPins  = { { 0 } },
        .bk2IO0Pins = { { 0 } },
        .bk2IO1Pins = { { 0 } },
        .bk2IO2Pins = { { 0 } },
        .bk2IO3Pins = { { 0 } },
        .bk2CSPins  = { { 0 } },
    },
};

static uint8_t dummyTx = 0x00; // During polled operation 0's are written so do the same
static uint8_t dummyRx = 0xFF;

// As per section 2.15 of the datasheet access registers at an offset of 0x4000
// to prevent duplication of 8 bit values across 32 bit word
#define xip_aux_hw_8bit ((xip_aux_hw_t *)(XIP_AUX_BASE + 0x4000))

// Write enable key for access control registers, see section 10.6 of datasheet
#define ACCESSCTRL_XIP_CTRL_WE 0xacce0000

// Default timeout for QSPI direct IO operations
#define QMI_TIMEOUT_US 5

// Number of dummy clock cycles per consumed byte in direct-mode loops.
// Justification: QSPI datasheets specify dummy cycles in bits. Our direct-mode
// helper consumes dummy time by shifting out whole bytes (8 bits at a time).
// We therefore convert from bits to bytes using 8 bits/byte. If a controller
// revision were to support sub-byte dummy handling, this could be revisited.
#define QSPI_DUMMY_BITS_PER_BYTE 8

static void quadSpiSequenceStart(const extDevice_t *dev);

// Pin configure is a no-op on RP2350, QMI pins are fixed and controlled by ROM/QMI
void quadSpiPinConfigure(const quadSpiConfig_t *pConfig)
{
    UNUSED(pConfig);
    quadSpiDevice[QUADSPIDEV_1].dev = quadSpiHardware[0].reg;
}

// Wait for QMI direct mode to become idle (BUSY=0, RXEMPTY=1, TXEMPTY=1), with a small timeout
static bool qmi_direct_wait_idle(timeDelta_t timeout_us)
{
    const timeUs_t start = micros();
    do {
        uint32_t csr = qmi_hw->direct_csr;
        bool busy = csr & QMI_DIRECT_CSR_BUSY_BITS;
        bool rxempty = csr & QMI_DIRECT_CSR_RXEMPTY_BITS;
        bool txempty = csr & QMI_DIRECT_CSR_TXEMPTY_BITS;

        if (!busy && rxempty && txempty) {
            return true;
        }
    } while (cmpTimeUs(micros(), start) < timeout_us);

    return false;
}

// QMI direct helpers for CS1 transactions
static void qmi_direct_enable(void) { hw_set_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_EN_BITS); }
static void qmi_direct_disable(void) { hw_clear_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_EN_BITS); }
static void qmi_cs1_assert(const extDevice_t *dev, bool asserted) {
    if (asserted) {
        IOLo(dev->busType_u.spi.csnPin);
    } else {
        // Make sure the access is complete before negating the CS
        (void)qmi_direct_wait_idle(QMI_TIMEOUT_US);

        // Negate Chip Select
        IOHi(dev->busType_u.spi.csnPin);
    }
}

//
// Non-blocking QuadSPI sequence framework for PICO (RP2350 QMI)
// Mirrors SPI bus segment queueing model. Uses DMA where appropriate and
// returns immediately; completion is signaled via queued segments and
// quadSpiIsBusy()/quadSpiWait().
//

bool quadSpiIsBusy(const extDevice_t *dev)
{
    if (!dev || !dev->bus) {
        return false;
    }
    return (dev->bus->curSegment != (volatile busSegment_t *)BUS_QSPI_FREE);
}

void quadSpiRelease(const extDevice_t *dev)
{
    UNUSED(dev);
}

static dma_channel_config qspi_tx_cfg;
static dma_channel_config qspi_rx_cfg;

static void quadSpiIrqHandler(dmaChannelDescriptor_t* descriptor);

void quadSpiInitBusDMA(busDevice_t *bus)
{
    bus->useDMA = true;

    bus->dmaRx = &dmaDescriptors[dma_claim_unused_channel(true)];
    bus->dmaTx = &dmaDescriptors[dma_claim_unused_channel(true)];

    // Claim specific DMA channels and bind by identifier
    // Allocate descriptors so dmaSetHandler knows the channel numbers
    (void)dmaAllocate(DMA_CHANNEL_TO_IDENTIFIER(bus->dmaTx->channel), OWNER_QUADSPI_BK1IO0, 1);
    (void)dmaAllocate(DMA_CHANNEL_TO_IDENTIFIER(bus->dmaRx->channel), OWNER_QUADSPI_BK1IO1, 1);
    // Register handlers
    dmaSetHandler(DMA_CHANNEL_TO_IDENTIFIER(bus->dmaTx->channel), quadSpiIrqHandler, NVIC_PRIO_SPI_DMA, 0);

    dma_channel_set_irq0_enabled(bus->dmaTx->channel, true);
}

static void quadSpiInternalInitStream(busDevice_t *bus, uint8_t *txData, uint8_t *rxData)
{
    bool isTx = txData != NULL;
    bool isRx = rxData != NULL;

    if (!bus) {
        return;
    }

    dmaChannelDescriptor_t *dmaRx = bus->dmaRx;
    dmaChannelDescriptor_t *dmaTx = bus->dmaTx;

    qspi_tx_cfg = dma_channel_get_default_config(dmaTx->channel);
    channel_config_set_transfer_data_size(&qspi_tx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&qspi_tx_cfg, isTx);
    channel_config_set_write_increment(&qspi_tx_cfg, false);
    channel_config_set_dreq(&qspi_tx_cfg, DREQ_XIP_QMITX);

    qspi_rx_cfg = dma_channel_get_default_config(dmaRx->channel);
    channel_config_set_transfer_data_size(&qspi_rx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&qspi_rx_cfg, false);
    channel_config_set_write_increment(&qspi_rx_cfg, isRx);
    channel_config_set_dreq(&qspi_rx_cfg, DREQ_XIP_QMIRX);
}

static void quadSpiDmaSegment(const extDevice_t *dev, busSegment_t *segment)
{
    if (!dev) {
        return;
    }

    busDevice_t *bus = dev->bus;
    if (!bus || !segment) {
        return;
    }

    // Launch next segment via DMA
    dmaChannelDescriptor_t *dmaRx = bus->dmaRx;
    dmaChannelDescriptor_t *dmaTx = bus->dmaTx;

    // Use the correct callback argument
    dmaTx->userParam = (uint32_t)dev;

    quadSpiInternalInitStream(bus, segment->u.buffers.txData, segment->u.buffers.rxData);

    uint8_t *txData = segment->u.buffers.txData ? segment->u.buffers.txData : &dummyTx;
    uint8_t *rxData = segment->u.buffers.rxData ? segment->u.buffers.rxData : &dummyRx;

    dma_channel_configure(dmaTx->channel, &qspi_tx_cfg,
                          &xip_aux_hw_8bit->qmi_direct_tx,
                          txData,
                          segment->len, false);
    dma_channel_configure(dmaRx->channel, &qspi_rx_cfg,
                          rxData,
                          &xip_aux_hw_8bit->qmi_direct_rx,
                          segment->len, false);

    // Drain the RX buffer
    timeUs_t start = micros();
    while (!(qmi_hw->direct_csr & QMI_DIRECT_CSR_RXEMPTY_BITS)) {
        // This timeout should never fire, and the consequences are undefined, but avoid hanging
        if (cmpTimeUs(micros(), start) > QMI_TIMEOUT_US) {
            break;
        }
        (void)qmi_hw->direct_rx;
    }

    // Trigger the DMAs
    dma_start_channel_mask((1u << dmaTx->channel) | (1u << dmaRx->channel));
}

static void quadSpiIrqHandler(dmaChannelDescriptor_t* descriptor)
{
    const extDevice_t *dev = (const extDevice_t *)descriptor->userParam;
    busDevice_t *bus = dev->bus;
    busSegment_t *nextSegment;

    if (bus->curSegment->negateCS) {
        // Negate Chip Select
        qmi_cs1_assert(dev, false);
    }

    if (bus->curSegment->callback) {
        switch(bus->curSegment->callback(dev->callbackArg)) {
        case BUS_BUSY:
            // Repeat the last DMA segment
            bus->curSegment--;
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
        if (!bus->curSegment->negateCS) {
            // Always negate CS at the end of a transaction
            qmi_cs1_assert(dev, false);
        }

        // If a following transaction has been linked, start it
        if (nextSegment->u.link.dev) {
            const extDevice_t *nextDev = nextSegment->u.link.dev;
            busSegment_t *nextSegments = (busSegment_t *)nextSegment->u.link.segments;
            // The end of the segment list has been reached
            bus->curSegment = nextSegments;
            nextSegment->u.link.dev = NULL;
            nextSegment->u.link.segments = NULL;
            quadSpiSequenceStart(nextDev);
        } else {
            // The end of the segment list has been reached, so mark transactions as complete
            bus->curSegment = (busSegment_t *)BUS_QSPI_FREE;
            qmi_direct_disable();
       }
    } else {
        bool negateCS = bus->curSegment->negateCS;

        bus->curSegment = nextSegment;

        if (negateCS) {
            // Assert Chip Select if necessary
            qmi_cs1_assert(dev, true);
        }

        // Launch next
        quadSpiDmaSegment(dev, (busSegment_t *)bus->curSegment);
    }
}

static void quadSpiSequenceStart(const extDevice_t *dev)
{
    if (!dev || !dev->bus) {
        return;
    }

    busDevice_t *bus = dev->bus;
    busSegment_t *segment = (busSegment_t *)bus->curSegment;

    if (!segment || !segment->len) return;

    qmi_direct_enable();

    // Assert Chip Select to start the transfer
    qmi_cs1_assert(dev, true);

   // Prepare and kick off first segment via DMA
    quadSpiDmaSegment(dev, segment);
}

void quadSpiSequence(const extDevice_t *dev, busSegment_t *segments)
{
    if (!dev || !dev->bus) {
        return;
    }

    busDevice_t *bus = dev->bus;

    // Queue if busy; else claim and start
    if (bus->curSegment) {
        // Find end of new list
        busSegment_t *endNew;
        for (endNew = segments; endNew->len; endNew++);
        // Find end of existing queued chain
        busSegment_t *endCur = (busSegment_t *)bus->curSegment;
        while (true) {
            for (; endCur->len; endCur++);
            if (endCur->u.link.segments == NULL) {
                break;
            } else {
                endCur = (busSegment_t *)endCur->u.link.segments;
            }
        }
        endCur->u.link.dev = dev;
        endCur->u.link.segments = segments;
        return;
    }

    bus->curSegment = segments;
    quadSpiSequenceStart(dev);
}

void quadSpiWait(const extDevice_t *dev)
{
    while (quadSpiIsBusy(dev)) {
    }
}

static void encodeAddr(uint32_t address, uint8_t addressSize, uint8_t *addrBytes)
{
    uint8_t addrByteCount = addressSize/8;

    if (addrByteCount > sizeof (address)) {
        return;
    }

    for (int i = 0; i < addrByteCount; i++) {
        addrBytes[addrByteCount - 1 - i] = (address >> (i << 3)) & 0xff;
    }
}

bool quadSpiTransmit1LINE(const extDevice_t *dev, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length)
{
    busSegment_t segments[] = {
        {.u.buffers = {&instruction, NULL}, .len = sizeof(instruction), .negateCS = false, NULL},
        {.u.buffers = {(uint8_t *)out, NULL}, .len = length, .negateCS = true, NULL},
        {.u.link = {NULL, NULL}, .len = 0, .negateCS = true, NULL},
    };

    busSegment_t segments_dummy[] = {
        {.u.buffers = {&instruction, NULL}, .len = sizeof(instruction), .negateCS = false, NULL},
        {.u.buffers = {NULL, NULL}, .len = dummyCycles/QSPI_DUMMY_BITS_PER_BYTE, .negateCS = false, NULL},
        {.u.buffers = {(uint8_t *)out, NULL}, .len = length, .negateCS = true, NULL},
        {.u.link = {NULL, NULL}, .len = 0, .negateCS = true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    quadSpiWait(dev);

    quadSpiSequence(dev, dummyCycles ? &segments_dummy[0] : &segments[0]);

    quadSpiWait(dev);

    return true;
}

bool quadSpiReceive1LINE(const extDevice_t *dev, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{
   busSegment_t segments[] = {
        {.u.buffers = {&instruction, NULL}, .len = sizeof(instruction), .negateCS = false, NULL},
        {.u.buffers = {NULL, in}, .len = length, .negateCS = true, NULL},
        {.u.link = {NULL, NULL}, .len = 0, .negateCS = true, NULL},
    };

    busSegment_t segments_dummy[] = {
        {.u.buffers = {&instruction, NULL}, .len = sizeof(instruction), .negateCS = false, NULL},
        {.u.buffers = {NULL, NULL}, .len = dummyCycles/QSPI_DUMMY_BITS_PER_BYTE, .negateCS = false, NULL},
        {.u.buffers = {NULL, in}, .len = length, .negateCS = true, NULL},
        {.u.link = {NULL, NULL}, .len = 0, .negateCS = true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    quadSpiWait(dev);

    quadSpiSequence(dev, dummyCycles ? &segments_dummy[0] : &segments[0]);

    quadSpiWait(dev);

    return true;
}

bool quadSpiInstructionWithData1LINE(const extDevice_t *dev, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length)
{
    return quadSpiTransmit1LINE(dev, instruction, dummyCycles, out, length);
}

bool quadSpiInstructionWithAddress1LINE(const extDevice_t *dev, uint8_t instruction, uint8_t dummyCycles,
                                        uint32_t address, uint8_t addressSize)
{
    // Ensure any prior DMA has completed before continuing
    quadSpiWait(dev);

    uint8_t addrBytes[addressSize/8];

    encodeAddr(address, addressSize, addrBytes);

    busSegment_t segments[] = {
        {.u.buffers = {&instruction, NULL}, .len = sizeof(instruction), .negateCS = false, NULL},
        {.u.buffers = {addrBytes, NULL}, .len = addressSize/8, .negateCS = true, NULL},
        {.u.link = {NULL, NULL}, .len = 0, .negateCS = true, NULL},
    };

    busSegment_t segments_dummy[] = {
        {.u.buffers = {&instruction, NULL}, .len = sizeof(instruction), .negateCS = false, NULL},
        {.u.buffers = {addrBytes, NULL}, .len = addressSize/8, .negateCS = false, NULL},
        {.u.buffers = {NULL, NULL}, .len = dummyCycles/QSPI_DUMMY_BITS_PER_BYTE, .negateCS = false, NULL},
        {.u.link = {NULL, NULL}, .len = 0, .negateCS = true, NULL},
    };

    quadSpiSequence(dev, dummyCycles ? &segments_dummy[0] : &segments[0]);

    quadSpiWait(dev);

    return true;
}

bool quadSpiReceiveWithAddress1LINE(const extDevice_t *dev, uint8_t instruction, uint8_t dummyCycles,
                                    uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{
    // Ensure any prior DMA has completed before continuing
    quadSpiWait(dev);

    uint8_t addrBytes[addressSize/8];

    encodeAddr(address, addressSize, addrBytes);

    busSegment_t segments[] = {
        {.u.buffers = {&instruction, NULL}, .len = sizeof(instruction), .negateCS = false, NULL},
        {.u.buffers = {addrBytes, NULL}, .len = addressSize/8, .negateCS = false, NULL},
        {.u.buffers = {NULL, in}, .len = length, .negateCS = true, NULL},
        {.u.link = {NULL, NULL}, .len = 0, .negateCS = true, NULL},
    };

    busSegment_t segments_dummy[] = {
        {.u.buffers = {&instruction, NULL}, .len = sizeof(instruction), .negateCS = false, NULL},
        {.u.buffers = {addrBytes, NULL}, .len = addressSize/8, .negateCS = false, NULL},
        {.u.buffers = {NULL, NULL}, .len = dummyCycles/QSPI_DUMMY_BITS_PER_BYTE, .negateCS = false, NULL},
        {.u.buffers = {NULL, in}, .len = length, .negateCS = true, NULL},
        {.u.link = {NULL, NULL}, .len = 0, .negateCS = true, NULL},
    };

    quadSpiSequence(dev, dummyCycles ? &segments_dummy[0] : &segments[0]);

    quadSpiWait(dev);

    return true;
}

bool quadSpiTransmitWithAddress1LINE(const extDevice_t *dev, uint8_t instruction, uint8_t dummyCycles,
                                     uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{
    // Ensure any prior DMA has completed before continuing
    quadSpiWait(dev);

    uint8_t addrBytes[addressSize/8];

    encodeAddr(address, addressSize, addrBytes);

    busSegment_t segments[] = {
        {.u.buffers = {&instruction, NULL}, .len = sizeof(instruction), .negateCS = false, NULL},
        {.u.buffers = {addrBytes, NULL}, .len = addressSize/8, .negateCS = false, NULL},
        {.u.buffers = {(uint8_t *)out, NULL}, .len = length, .negateCS = true, NULL},
        {.u.link = {NULL, NULL}, .len = 0, .negateCS = true, NULL},
    };

    busSegment_t segments_dummy[] = {
        {.u.buffers = {&instruction, NULL}, .len = sizeof(instruction), .negateCS = false, NULL},
        {.u.buffers = {addrBytes, NULL}, .len = addressSize/8, .negateCS = false, NULL},
        {.u.buffers = {NULL, NULL}, .len = dummyCycles/QSPI_DUMMY_BITS_PER_BYTE, .negateCS = false, NULL},
        {.u.buffers = {(uint8_t *)out, NULL}, .len = length, .negateCS = true, NULL},
        {.u.link = {NULL, NULL}, .len = 0, .negateCS = true, NULL},
    };

    quadSpiSequence(dev, dummyCycles ? &segments_dummy[0] : &segments[0]);

    quadSpiWait(dev);

    return true;
}

// 4LINE operations are not truly available via ROM command mode on QMI; emulate using 1LINE for now.
bool quadSpiReceive4LINES(const extDevice_t *dev, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{
    UNUSED(dummyCycles);

    uint8_t instr = (instruction == 0x6B) ? 0x03 : instruction; // fast quad->fast serial
    return quadSpiReceive1LINE(dev, instr, 0, in, length);
}

bool quadSpiReceiveWithAddress4LINES(const extDevice_t *dev, uint8_t instruction, uint8_t dummyCycles,
                                     uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{
    UNUSED(dummyCycles);

    uint8_t instr = (instruction == 0x6B) ? 0x03 : instruction; // map to serial fast read
    return quadSpiReceiveWithAddress1LINE(dev, instr, 0, address, addressSize, in, length);
}

bool quadSpiTransmitWithAddress4LINES(const extDevice_t *dev, uint8_t instruction, uint8_t dummyCycles,
                                      uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{
    UNUSED(dummyCycles);

    uint8_t instr = (instruction == 0x32) ? 0x02 : instruction; // quad page prog -> serial page prog
    return quadSpiTransmitWithAddress1LINE(dev, instr, 0, address, addressSize, out, length);
}

void quadSpiSetDivisor(const extDevice_t *dev, uint16_t divisor)
{
    if (dev) {
        ((extDevice_t *)dev)->busType_u.spi.speed = divisor;
    }
    // Pico ROM manages clocking for direct command mode; leave as-is.
}

bool quadSpiInit(quadSpiDevice_e device);
void quadSpiPreInit(void) {}

void quadSpiInitDevice(quadSpiDevice_e device)
{
    if (device >= QUADSPIDEV_COUNT) {
        return;
    }

    // Enable DMA access to QMI - Boot ROM defaults to 0xb8 (DMA disabled)
    // Current default: 0xb8 = DBG|CORE1|CORE0|SP, but DMA bit is 0
    // We need to set bit 6 (DMA) to enable DMA access to XIP_QMI
    uint32_t current_qmi = accessctrl_hw->xip_qmi;
    if (!(current_qmi & ACCESSCTRL_XIP_QMI_DMA_BITS)) {
        // DMA access is disabled (bit 6 = 0) - enable it
        accessctrl_hw->xip_qmi = (accessctrl_hw->xip_qmi & ACCESSCTRL_XIP_CTRL_BITS) |
                                 ACCESSCTRL_XIP_CTRL_WE | ACCESSCTRL_XIP_QMI_DMA_BITS;
    }

    // Winbond W25Q series support 64k (D8h) erase; enable to speed up erases if present
    flash_devinfo_set_d8h_erase_supported(true);
}

#endif // USE_QUADSPI
