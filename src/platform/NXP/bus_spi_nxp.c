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
 * LPSPI master driver for the NXP parts. Chip select is a GPIO driven by the bus
 * layer, frames are one byte each with TCR CONT clear.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPI

#include "common/utils.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/dma_impl.h"
#include "drivers/io.h"
#include "drivers/nvic.h"

#include "platform/dma.h"
#include "platform/nxp_clock.h"

#include "pg/bus_spi.h"

#include "iomux_nxp.h"
#include "iomux_signal_nxp.h"
#include "iomux_signal_def.h"

#ifdef USE_DMA
#define NXP_SPI_DMA
#endif

const spiHardware_t spiHardware[SPIDEV_COUNT] = {
#ifdef USE_SPI_DEVICE_1
    {
        .device = SPIDEV_1,
        .reg = (spiResource_t *)LPSPI1,
    },
#endif
#ifdef USE_SPI_DEVICE_2
    {
        .device = SPIDEV_2,
        .reg = (spiResource_t *)LPSPI2,
    },
#endif
#ifdef USE_SPI_DEVICE_3
    {
        .device = SPIDEV_3,
        .reg = (spiResource_t *)LPSPI3,
    },
#endif
#ifdef USE_SPI_DEVICE_4
    {
        .device = SPIDEV_4,
        .reg = (spiResource_t *)LPSPI4,
    },
#endif
#ifdef USE_SPI_DEVICE_5
    {
        .device = SPIDEV_5,
        .reg = (spiResource_t *)LPSPI5,
    },
#endif
#ifdef USE_SPI_DEVICE_6
    {
        .device = SPIDEV_6,
        .reg = (spiResource_t *)LPSPI6,
    },
#endif
};

extern busDevice_t spiBusDevice[SPIDEV_COUNT];

// Any pad in the generated nxpLpspiPins table can carry the signal.
// LPSPI1 is instance 1, so the bus number is one past the device index.
#define SPI_INSTANCE(device)    ((uint8_t)((device) - SPIDEV_1 + 1))

static const nxpPinMux_t *spiSckMux(spiDevice_e device, ioTag_t pin)
{
    return NXP_PIN_MUX(nxpLpspiPins, SPI_INSTANCE(device), NXP_ROLE_LPSPI_SCK, pin);
}

static const nxpPinMux_t *spiSdiMux(spiDevice_e device, ioTag_t pin)
{
    return NXP_PIN_MUX(nxpLpspiPins, SPI_INSTANCE(device), NXP_ROLE_LPSPI_SDI, pin);
}

static const nxpPinMux_t *spiSdoMux(spiDevice_e device, ioTag_t pin)
{
    return NXP_PIN_MUX(nxpLpspiPins, SPI_INSTANCE(device), NXP_ROLE_LPSPI_SDO, pin);
}

void spiPinConfigure(const struct spiPinConfig_s *pConfig)
{
    for (size_t hwindex = 0; hwindex < ARRAYLEN(spiHardware); hwindex++) {
        const spiHardware_t *hw = &spiHardware[hwindex];

        if (!hw->reg) {
            continue;
        }

        const spiDevice_e device = hw->device;
        spiDevice_t *pDev = &spiDevice[device];

        const ioTag_t sck = pConfig[device].ioTagSck;
        const ioTag_t miso = pConfig[device].ioTagMiso;
        const ioTag_t mosi = pConfig[device].ioTagMosi;

        // One unreachable pad leaves the bus unclaimed.
        if (!spiSckMux(device, sck) || !spiSdiMux(device, miso) || !spiSdoMux(device, mosi)) {
            continue;
        }

        pDev->sck = sck;
        pDev->miso = miso;
        pDev->mosi = mosi;
        pDev->dev = hw->reg;
        pDev->leadingEdge = false;
    }
}

// The divider entry points take no bus, so they use the LPSPI1 clock.
// The i.MX95 has a clock root per LPSPI, so one that differs from LPSPI1 needs its rate carried per bus.
STATIC_ASSERT(NXP_CLK_LPI2C1 - NXP_CLK_LPSPI1 >= SPIDEV_COUNT,
              clock_enum_needs_one_entry_per_lpspi_instance);

static uint32_t spiClockHz(spiDevice_e device)
{
    return nxpClockHz((nxpClock_e)(NXP_CLK_LPSPI1 + (device - SPIDEV_1)));
}

// SCK = clock / 2^TCR[PRESCALE] / (CCR[SCKDIV] + 2), fastest pair not above the requested rate.
// Carried as one 16 bit number, PRESCALE in the high byte and SCKDIV in the low byte.
#define LPSPI_PRESCALE_MAX      7u
#define LPSPI_SCKDIV_MAX        LPSPI_CCR_SCKDIV_MASK
#if defined(NXP_LPSPI_EVEN_SCKDIV)
#define LPSPI_SCKDIV_STEP       2u
#else
#define LPSPI_SCKDIV_STEP       1u
#endif

#define SPI_DIVIDER(prescale, sckdiv)   ((uint16_t)(((prescale) << 8) | (sckdiv)))
#define SPI_DIVIDER_PRESCALE(d)         (((d) >> 8) & 0x7)
#define SPI_DIVIDER_SCKDIV(d)           ((d) & 0xff)

static uint32_t spiSckHz(uint32_t clockHz, uint32_t prescale, uint32_t sckdiv)
{
    return (clockHz >> prescale) / (sckdiv + 2);
}

uint16_t spiCalculateDivider(uint32_t freq)
{
    const uint32_t clockHz = spiClockHz(SPIDEV_1);

    // Slowest setting, used when even that is faster than requested.
    uint32_t bestPrescale = LPSPI_PRESCALE_MAX;
    uint32_t bestSckdiv = LPSPI_SCKDIV_MAX + 1 - LPSPI_SCKDIV_STEP;
    uint32_t bestRate = 0;

    for (uint32_t prescale = 0; prescale <= LPSPI_PRESCALE_MAX; prescale++) {
        for (uint32_t sckdiv = 0; sckdiv <= LPSPI_SCKDIV_MAX; sckdiv += LPSPI_SCKDIV_STEP) {
            const uint32_t rate = spiSckHz(clockHz, prescale, sckdiv);

            if (rate <= freq && rate > bestRate) {
                bestRate = rate;
                bestPrescale = prescale;
                bestSckdiv = sckdiv;
            }

            // Rates only fall as sckdiv rises, so the rest of the row is slower.
            if (rate <= freq) {
                break;
            }
        }
    }

    return SPI_DIVIDER(bestPrescale, bestSckdiv);
}

uint32_t spiCalculateClock(uint16_t spiClkDivisor)
{
    return spiSckHz(spiClockHz(SPIDEV_1), SPI_DIVIDER_PRESCALE(spiClkDivisor), SPI_DIVIDER_SCKDIV(spiClkDivisor));
}

// One byte per frame. FRAMESZ holds bits minus one.
#define LPSPI_FRAME_BITS        8
#define LPSPI_FRAMESZ           (LPSPI_FRAME_BITS - 1)

// Written whole rather than read back, as a register read stalls on the peripheral bus.
// CFGR1[NOSTALL] clear stretches SCK when the transmit FIFO runs dry.
#define SPI_CFGR1_VALUE         LPSPI_CFGR1_MASTER_MASK
#define SPI_CR_ENABLED          LPSPI_CR_MEN_MASK

// The write one to clear flags of SR; REF and TEF do not clear themselves.
#define SPI_SR_W1C_FLAGS        (LPSPI_SR_WCF_MASK | LPSPI_SR_FCF_MASK | LPSPI_SR_TCF_MASK \
                                 | LPSPI_SR_TEF_MASK | LPSPI_SR_REF_MASK | LPSPI_SR_DMF_MASK)

// Sent while receiving. All ones leaves the line idle high.
#define SPI_DUMMY_TX_BYTE       0xff

#if defined(NXP_SPI_DMA)
// Segment lists one bus keeps descriptors for, so two devices taking turns keep a slot each.
#define SPI_CHAIN_SLOTS         2

// The exclusive or below only means "the other one" while there are two.
STATIC_ASSERT(SPI_CHAIN_SLOTS == 2, spi_chain_slots_are_a_pair);

// What one slot's descriptors were built for.
typedef struct spiChainSlot_s {
    unsigned count;
} spiChainSlot_t;
#endif

typedef struct spiState_s {
    uint8_t txFifoDepth;
    uint8_t rxFifoDepth;
    uint32_t tcr;               // the command word, minus the clock fields
#ifdef NXP_SPI_DMA
    // DMA_NONE until spiInitBusDMA() claims a pair for the bus.
    dmaIdentifier_e dmaTx;
    dmaIdentifier_e dmaRx;
    // For the completion interrupt, as userParam is only 32 bits.
    const extDevice_t *dev;
    volatile busSegment_t *chainLast;
    spiChainSlot_t chain[SPI_CHAIN_SLOTS];
    // The slot a list matching neither is built into: the least recently used.
    unsigned chainSpare;
    // Receive list of the ring both channels hold, NULL after an error.
    const dmaTcd_t *chainHeld;
#endif
} spiState_t;

static spiState_t spiState[SPIDEV_COUNT];

// Rate the module starts at; every device sets its own before transferring.
#define SPI_INIT_HZ             1000000u

// CCR SCKDIV and DBT count prescaled cycles minus two, PCSSCK and SCKPCS minus one.
// PCS is not on a pad, so the delays are set to the minimum. CCR is only writable with CR[MEN] clear.
static void spiSetClockDivider(LPSPI_Type *lpspi, uint16_t divisor)
{
    lpspi->CR = 0;
    lpspi->CCR = LPSPI_CCR_SCKDIV(SPI_DIVIDER_SCKDIV(divisor))
               | LPSPI_CCR_DBT(0)
               | LPSPI_CCR_PCSSCK(0)
               | LPSPI_CCR_SCKPCS(0);
    lpspi->CR = SPI_CR_ENABLED;
}

// PRESCALE, CPOL and CPHA are in TCR, which is writable while enabled. Leading edge is mode 0, else mode 3.
static uint32_t spiBuildTcr(const spiState_t *state, uint16_t divisor, bool leadingEdge)
{
    uint32_t tcr = state->tcr | LPSPI_TCR_PRESCALE(SPI_DIVIDER_PRESCALE(divisor));

    if (!leadingEdge) {
        tcr |= LPSPI_TCR_CPOL_MASK | LPSPI_TCR_CPHA_MASK;
    }

    return tcr;
}

void spiInitDevice(spiDevice_e device)
{
    spiDevice_t *spi = &spiDevice[device];
    spiState_t *state = &spiState[device];

#ifdef NXP_SPI_DMA
    // Set before the pin checks below can bail out; spiInitBusDMA() reads these.
    state->dmaTx = DMA_NONE;
    state->dmaRx = DMA_NONE;
#endif

    if (!spi->dev) {
        return;
    }

    LPSPI_Type *lpspi = SPI_INST(spi->dev);

    const nxpPinMux_t *sckMux = spiSckMux(device, spi->sck);
    const nxpPinMux_t *sdiMux = spiSdiMux(device, spi->miso);
    const nxpPinMux_t *sdoMux = spiSdoMux(device, spi->mosi);

    if (!sckMux || !sdiMux || !sdoMux) {
        return;
    }

    if (!nxpClockGateEnable((nxpClock_e)(NXP_CLK_LPSPI1 + (device - SPIDEV_1)))) {
        return;
    }

    // CR[RST] does not empty the FIFOs, so flush them while reset is asserted.
    lpspi->CR = LPSPI_CR_RST_MASK;
    lpspi->CR = LPSPI_CR_RST_MASK | LPSPI_CR_RTF_MASK | LPSPI_CR_RRF_MASK;
    lpspi->CR = 0;

    IOInit(IOGetByTag(spi->sck), OWNER_SPI_SCK, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_SDI, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_SDO, RESOURCE_INDEX(device));

    nxpIomuxConfigPin(sckMux, SPI_IO_AF_CFG);
    // Pull up, so a missing device reads back as all ones rather than as noise.
    nxpIomuxConfigPin(sdiMux, SPI_IO_AF_SDI_CFG);
    nxpIomuxConfigPin(sdoMux, SPI_IO_AF_CFG);

    // PARAM reports the FIFO depths as powers of two; they differ per instance.
    const uint32_t param = lpspi->PARAM;
    state->txFifoDepth = 1u << ((param & LPSPI_PARAM_TXFIFO_MASK) >> LPSPI_PARAM_TXFIFO_SHIFT);
    state->rxFifoDepth = 1u << ((param & LPSPI_PARAM_RXFIFO_MASK) >> LPSPI_PARAM_RXFIFO_SHIFT);

    state->tcr = LPSPI_TCR_FRAMESZ(LPSPI_FRAMESZ);

    lpspi->CFGR0 = 0;
    lpspi->CFGR1 = SPI_CFGR1_VALUE;

    // Transmit request at half full, receive request on the first word.
    lpspi->FCR = LPSPI_FCR_TXWATER(state->txFifoDepth / 2) | LPSPI_FCR_RXWATER(0);

    lpspi->IER = 0;
    lpspi->DER = 0;
    lpspi->SR = SPI_SR_W1C_FLAGS;

    const uint16_t divisor = spiCalculateDivider(SPI_INIT_HZ);

    // Divider first: it leaves the module enabled, and TCR only holds when written while enabled.
    spiSetClockDivider(lpspi, divisor);
    lpspi->TCR = spiBuildTcr(state, divisor, spi->leadingEdge);
}

// Erratum ERR050456: the FIFO reset also clears CFGR1, so it is written back straight after.
FAST_CODE static void spiFlushFifos(LPSPI_Type *lpspi)
{
    lpspi->CR = SPI_CR_ENABLED | LPSPI_CR_RTF_MASK | LPSPI_CR_RRF_MASK;
    lpspi->CFGR1 = SPI_CFGR1_VALUE;
    lpspi->SR = SPI_SR_W1C_FLAGS;
}

// Backstop for a dead clock gate or pad; far above the slowest frame of about 4 ms.
#define SPI_POLL_SPIN_BUDGET    1000000u

FAST_CODE bool spiInternalReadWriteBufPolled(spiResource_t *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
    LPSPI_Type *lpspi = SPI_INST(instance);
    const spiDevice_e device = spiDeviceByInstance(instance);

    if (device == SPIINVALID) {
        return false;
    }

    const spiState_t *state = &spiState[device];
    int sent = 0;
    int received = 0;
    uint32_t spins = 0;

    while (received < len) {
        // Never have more bytes in flight than the receive FIFO can hold.
        while ((sent < len)
               && ((sent - received) < state->rxFifoDepth)
               && ((lpspi->FSR & LPSPI_FSR_TXCOUNT_MASK) < state->txFifoDepth)) {
            lpspi->TDR = txData ? txData[sent] : SPI_DUMMY_TX_BYTE;
            sent++;
        }

        if (lpspi->RSR & LPSPI_RSR_RXEMPTY_MASK) {
            if (++spins > SPI_POLL_SPIN_BUDGET) {
                spiDevice[device].errorCount++;
                // Drop what is still in flight so the next transfer does not read it.
                spiFlushFifos(lpspi);
                return false;
            }
            continue;
        }

        spins = 0;

        const uint8_t data = lpspi->RDR;

        if (rxData) {
            rxData[received] = data;
        }
        received++;
    }

    return true;
}

#ifdef NXP_SPI_DMA

// Segments per scatter gather chain; a register read takes two.
#define SPI_TCD_MAX             4

// The engine reads these, so they go in DTCM; 512 bytes per bus.
#define SPI_TCD_LIST_INDEX(device, slot)    (((device) * SPI_CHAIN_SLOTS + (slot)) * SPI_TCD_MAX)

static DMA_TCD_LIST(spiTxTcds, SPIDEV_COUNT * SPI_CHAIN_SLOTS * SPI_TCD_MAX);
static DMA_TCD_LIST(spiRxTcds, SPIDEV_COUNT * SPI_CHAIN_SLOTS * SPI_TCD_MAX);

// Source or sink for a segment without a buffer; the address is not incremented.
static DMA_DATA uint8_t spiDummyTxByte = SPI_DUMMY_TX_BYTE;
static DMA_DATA uint8_t spiDummyRxByte;

// DMAMUX request sources, one per direction per bus.
#if NXP_HAS_EDMA5
static const uint32_t spiDmaMuxTx[SPIDEV_COUNT] = {
    [SPIDEV_1] = NXP_DMA_REQ_LPSPI1_TX,
    [SPIDEV_2] = NXP_DMA_REQ_LPSPI2_TX,
    [SPIDEV_3] = NXP_DMA_REQ_LPSPI3_TX,
    [SPIDEV_4] = NXP_DMA_REQ_LPSPI4_TX,
    [SPIDEV_5] = NXP_DMA_REQ_LPSPI5_TX,
    [SPIDEV_6] = NXP_DMA_REQ_LPSPI6_TX,
};

static const uint32_t spiDmaMuxRx[SPIDEV_COUNT] = {
    [SPIDEV_1] = NXP_DMA_REQ_LPSPI1_RX,
    [SPIDEV_2] = NXP_DMA_REQ_LPSPI2_RX,
    [SPIDEV_3] = NXP_DMA_REQ_LPSPI3_RX,
    [SPIDEV_4] = NXP_DMA_REQ_LPSPI4_RX,
    [SPIDEV_5] = NXP_DMA_REQ_LPSPI5_RX,
    [SPIDEV_6] = NXP_DMA_REQ_LPSPI6_RX,
};
#else
static const uint32_t spiDmaMuxTx[SPIDEV_COUNT] = {
#ifdef USE_SPI_DEVICE_1
    [SPIDEV_1] = kDmaRequestMuxLPSPI1Tx,
#endif
#ifdef USE_SPI_DEVICE_2
    [SPIDEV_2] = kDmaRequestMuxLPSPI2Tx,
#endif
#ifdef USE_SPI_DEVICE_3
    [SPIDEV_3] = kDmaRequestMuxLPSPI3Tx,
#endif
#ifdef USE_SPI_DEVICE_4
    [SPIDEV_4] = kDmaRequestMuxLPSPI4Tx,
#endif
};

static const uint32_t spiDmaMuxRx[SPIDEV_COUNT] = {
#ifdef USE_SPI_DEVICE_1
    [SPIDEV_1] = kDmaRequestMuxLPSPI1Rx,
#endif
#ifdef USE_SPI_DEVICE_2
    [SPIDEV_2] = kDmaRequestMuxLPSPI2Rx,
#endif
#ifdef USE_SPI_DEVICE_3
    [SPIDEV_3] = kDmaRequestMuxLPSPI3Rx,
#endif
#ifdef USE_SPI_DEVICE_4
    [SPIDEV_4] = kDmaRequestMuxLPSPI4Rx,
#endif
};
#endif

// One byte per request, so the descriptors do not depend on the FIFO watermarks.
FAST_CODE static bool spiBuildSegmentTcds(LPSPI_Type *lpspi, const busSegment_t *segment,
                                          dmaTcd_t *txTcd, dmaTcd_t *rxTcd)
{
    const uint8_t *txData = segment->u.buffers.txData;
    uint8_t *rxData = segment->u.buffers.rxData;

    const dmaTransfer_t txTransfer = {
        .src = txData ? (const void *)txData : (const void *)&spiDummyTxByte,
        .dst = (void *)&lpspi->TDR,
        .srcOffset = txData ? 1 : 0,
        .dstOffset = 0,
        .srcSize = DMA_XFER_SIZE_1B,
        .dstSize = DMA_XFER_SIZE_1B,
        .minorBytes = 1,
        .majorCount = segment->len,
    };

    const dmaTransfer_t rxTransfer = {
        .src = (const void *)&lpspi->RDR,
        .dst = rxData ? (void *)rxData : (void *)&spiDummyRxByte,
        .srcOffset = 0,
        .dstOffset = rxData ? 1 : 0,
        .srcSize = DMA_XFER_SIZE_1B,
        .dstSize = DMA_XFER_SIZE_1B,
        .minorBytes = 1,
        .majorCount = segment->len,
    };

    return dmaBuildTcd(txTcd, &txTransfer) && dmaBuildTcd(rxTcd, &rxTransfer);
}

// Compares the descriptors in memory, not the segment list address, which may be on the stack.
FAST_CODE static bool spiChainIsBuilt(const spiChainSlot_t *slot, const dmaTcd_t *txList, const dmaTcd_t *rxList,
                                      const busSegment_t *segment, unsigned count)
{
    if (!count || (slot->count != count)) {
        return false;
    }

    for (unsigned i = 0; i < count; i++) {
        const uint8_t *txData = segment[i].u.buffers.txData;
        const uint8_t *rxData = segment[i].u.buffers.rxData;

        if ((txList[i].SADDR != (uint32_t)(uintptr_t)(txData ? txData : &spiDummyTxByte))
            || (rxList[i].DADDR != (uint32_t)(uintptr_t)(rxData ? rxData : &spiDummyRxByte))
            || (txList[i].BITER != segment[i].len)) {
            return false;
        }
    }

    return true;
}

// Segments that can share one chain: no callback and chip select held, except on the last.
FAST_CODE static unsigned spiChainableSegments(volatile busSegment_t *segment)
{
    unsigned count = 0;

    while ((count < SPI_TCD_MAX)
           && segment[count].len
           && (segment[count].len <= DMA_MAJOR_COUNT_MAX)) {
        const bool stop = segment[count].negateCS || segment[count].callback;

        // The last segment's callback and chip select run in the completion interrupt.
        count++;

        if (stop) {
            break;
        }
    }

    return count;
}

// A channel error ends the transfer without walking the rest of the list. The bus is
// released because spiWait() has no timeout.
FAST_IRQ_HANDLER static void spiDmaErrorTerminate(const extDevice_t *dev, spiDevice_e device)
{
    busDevice_t *bus = dev->bus;
    spiState_t *state = &spiState[device];

    state->chainLast = NULL;
    state->chainHeld = NULL;

    spiInternalStopDMA(dev);
    spiFlushFifos(SPI_INST(bus->busType_u.spi.instance));
    IOHi(dev->busType_u.spi.csnPin);

    spiDevice[device].errorCount++;

    // Drop a fault the other channel raised, or it would be blamed on the next transfer.
    dmaClearError(state->dmaTx);
    dmaClearError(state->dmaRx);

    // An error raised on an idle channel finds the bus already free.
    if (bus->curSegment == (busSegment_t *)BUS_SPI_FREE) {
        return;
    }

    // Skip to the end of the list; the receive buffers are only partly filled.
    busSegment_t *end = (busSegment_t *)bus->curSegment;

    while (end->len) {
        end++;
    }

    // Start a queued transaction as a completion would.
    const extDevice_t *nextDev = end->u.link.dev;
    busSegment_t *nextSegments = (busSegment_t *)end->u.link.segments;

    end->u.link.dev = NULL;
    end->u.link.segments = NULL;

    if (nextDev) {
        bus->curSegment = nextSegments;
        spiSequenceStart(nextDev);
        return;
    }

    bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
}

// The transmit ring has no completion interrupt, so this runs only on a fault.
FAST_IRQ_HANDLER static void spiTxDmaHandler(dmaChannelDescriptor_t *descriptor)
{
    const spiDevice_e device = (spiDevice_e)descriptor->userParam;

    if (device >= SPIDEV_COUNT) {
        return;
    }

    const extDevice_t *dev = spiState[device].dev;

    if (dev && DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF)) {
        spiDmaErrorTerminate(dev, device);
    }
}

// Receive completes after the last byte is shifted out.
FAST_IRQ_HANDLER static void spiRxDmaHandler(dmaChannelDescriptor_t *descriptor)
{
    const spiDevice_e device = (spiDevice_e)descriptor->userParam;

    if (device >= SPIDEV_COUNT) {
        return;
    }

    spiState_t *state = &spiState[device];
    const extDevice_t *dev = state->dev;

    if (!dev) {
        return;
    }

    busDevice_t *bus = dev->bus;

    // The same callback reports a channel error.
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF)) {
        spiDmaErrorTerminate(dev, device);
        return;
    }

    // Point the bus layer at the last segment the chain covered.
    if (state->chainLast) {
        bus->curSegment = state->chainLast;
        state->chainLast = NULL;
    }

    if (bus->curSegment->negateCS) {
        IOHi(dev->busType_u.spi.csnPin);
    }

    spiInternalStopDMA(dev);

    spiIrqHandler(dev);
}

void spiInitBusDMA(void)
{
    for (spiDevice_e device = 0; device < SPIDEV_COUNT; device++) {
        busDevice_t *bus = &spiBusDevice[device];

        if (bus->busType != BUS_TYPE_SPI) {
            continue;
        }

        spiState_t *state = &spiState[device];

        if (state->dmaRx != DMA_NONE) {
            continue;
        }

        const dmaIdentifier_e tx = dmaGetFreeIdentifierForRequest(spiDmaMuxTx[device]);

        if (tx == DMA_NONE || !dmaAllocate(tx, OWNER_SPI_SDO, RESOURCE_INDEX(device))) {
            continue;
        }

        const dmaIdentifier_e rx = dmaGetFreeIdentifierForRequest(spiDmaMuxRx[device]);

        if (rx == DMA_NONE || !dmaAllocate(rx, OWNER_SPI_SDI, RESOURCE_INDEX(device))) {
            // This bus stays polled, so release the transmit channel by hand.
            dmaChannelDescriptor_t *txDescriptor = &dmaDescriptors[DMA_IDENTIFIER_TO_CHANNEL(tx)];

            txDescriptor->resourceOwner.owner = OWNER_FREE;
            txDescriptor->resourceOwner.index = 0;
            continue;
        }

        dmaEnable(tx);
        dmaMuxEnable(tx, spiDmaMuxTx[device]);
        dmaEnable(rx);
        dmaMuxEnable(rx, spiDmaMuxRx[device]);

        // Receive completion ends the transfer; the transmit handler is for faults only.
        dmaSetHandler(tx, spiTxDmaHandler, NVIC_PRIO_SPI_DMA, device);
        dmaSetHandler(rx, spiRxDmaHandler, NVIC_PRIO_SPI_DMA, device);

        // Requests stay on; the channel request enable arms each transfer.
        SPI_INST(bus->busType_u.spi.instance)->DER = LPSPI_DER_RDDE_MASK | LPSPI_DER_TDDE_MASK;

        state->dmaTx = tx;
        state->dmaRx = rx;

        bus->dmaTx = &dmaDescriptors[DMA_IDENTIFIER_TO_CHANNEL(tx)];
        bus->dmaRx = &dmaDescriptors[DMA_IDENTIFIER_TO_CHANNEL(rx)];

        bus->useDMA = true;
    }
}

// Descriptors are built in spiInternalStartDMA(). In ITCM as it is called per gyro transfer.
FAST_CODE void spiInternalInitStream(const extDevice_t *dev, volatile busSegment_t *segment)
{
    UNUSED(dev);
    UNUSED(segment);
}

FAST_CODE void spiInternalStartDMA(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    const spiDevice_e device = spiDeviceByInstance(bus->busType_u.spi.instance);

    if (device == SPIINVALID) {
        return;
    }

    LPSPI_Type *lpspi = SPI_INST(bus->busType_u.spi.instance);
    spiState_t *state = &spiState[device];

    unsigned count = spiChainableSegments(bus->curSegment);

    // Which slot already holds this list, if either; otherwise the spare one.
    unsigned slot = state->chainSpare;
    bool built = false;

    for (unsigned i = 0; i < SPI_CHAIN_SLOTS; i++) {
        if (spiChainIsBuilt(&state->chain[i], &spiTxTcds[SPI_TCD_LIST_INDEX(device, i)],
                            &spiRxTcds[SPI_TCD_LIST_INDEX(device, i)],
                            (const busSegment_t *)bus->curSegment, count)) {
            slot = i;
            built = true;
            break;
        }
    }

    dmaTcd_t *txList = &spiTxTcds[SPI_TCD_LIST_INDEX(device, slot)];
    dmaTcd_t *rxList = &spiRxTcds[SPI_TCD_LIST_INDEX(device, slot)];

    if (built) {
        // A ring that ran to the end reloaded its own head, so only arming is left.
        if (state->chainHeld != rxList) {
            dmaLoadTcd(state->dmaRx, &rxList[0]);
            dmaLoadTcd(state->dmaTx, &txList[0]);
            state->chainHeld = rxList;
        }
    } else {
        state->chain[slot].count = 0;

        for (unsigned i = 0; i < count; i++) {
            if (!spiBuildSegmentTcds(lpspi, (const busSegment_t *)&bus->curSegment[i], &txList[i], &rxList[i])) {
                count = i;
                break;
            }
        }

        if (!count) {
            // Run the list by hand, as spiWait() has no timeout.
            spiProcessSegmentsPolled(dev);
            return;
        }

        // Rings reload their own head, so the same transfer can be armed again unchanged.
        if (!dmaSetTransferRing(state->dmaRx, rxList, count, true)
            || !dmaSetTransferRing(state->dmaTx, txList, count, false)) {
            // Run the list by hand instead.
            state->chainHeld = NULL;
            spiProcessSegmentsPolled(dev);
            return;
        }

        state->chain[slot].count = count;
        state->chainHeld = rxList;
    }

    // A third list takes the other slot.
    state->chainSpare = slot ^ 1;

    state->chainLast = &bus->curSegment[count - 1];
    state->dev = dev;

    // Receive is armed first: no byte can arrive before its channel is listening.
    dmaEnableRequest(state->dmaRx);
    dmaEnableRequest(state->dmaTx);
}

FAST_CODE void spiInternalStopDMA(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    const spiDevice_e device = spiDeviceByInstance(bus->busType_u.spi.instance);

    if (device == SPIINVALID) {
        return;
    }

    const spiState_t *state = &spiState[device];

    if (state->dmaRx == DMA_NONE) {
        return;
    }

    dmaDisableRequest(state->dmaTx);
    dmaDisableRequest(state->dmaRx);
}

#else // NXP_SPI_DMA

void spiInitBusDMA(void)
{
}

// In ITCM like the eDMA version.
FAST_CODE void spiInternalInitStream(const extDevice_t *dev, volatile busSegment_t *segment)
{
    UNUSED(dev);
    UNUSED(segment);
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    UNUSED(dev);
}

void spiInternalStopDMA(const extDevice_t *dev)
{
    UNUSED(dev);
}

#endif // NXP_SPI_DMA

// Part of the platform interface; nothing in Betaflight calls either of these.
void spiInternalResetDescriptors(busDevice_t *bus)
{
    UNUSED(bus);
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    UNUSED(descriptor);
}

// Transfers shorter than this, in total, are polled; arming two channels costs more.
#ifndef SPI_DMA_MIN_LEN
#define SPI_DMA_MIN_LEN         8
#endif

#define SPI_DMA_WORTH_IT(segmentCount, xferLen) ((xferLen) >= SPI_DMA_MIN_LEN)

FAST_CODE void spiSequenceStart(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    const spiDevice_e device = spiDeviceByInstance(bus->busType_u.spi.instance);

    if (device == SPIINVALID) {
        return;
    }

    LPSPI_Type *lpspi = SPI_INST(bus->busType_u.spi.instance);
    spiState_t *state = &spiState[device];

    bus->initSegment = true;

    // A device that clocked out extra bytes can leave one behind.
    spiFlushFifos(lpspi);

    const bool speedChanged = dev->busType_u.spi.speed != bus->busType_u.spi.speed;
    const bool edgeChanged = dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge;

    if (speedChanged || edgeChanged) {
        // Devices on one bus run at different rates.
        bus->busType_u.spi.speed = dev->busType_u.spi.speed;
        bus->busType_u.spi.leadingEdge = dev->busType_u.spi.leadingEdge;

        lpspi->TCR = spiBuildTcr(state, dev->busType_u.spi.speed, dev->busType_u.spi.leadingEdge);

        if (speedChanged) {
            spiSetClockDivider(lpspi, dev->busType_u.spi.speed);
        }
    }

    unsigned segmentCount = 0;
    int xferLen = 0;
    // A segment longer than a major loop puts the whole list on the polled path.
    bool dmaCapable = true;

    for (busSegment_t *checkSegment = (busSegment_t *)bus->curSegment; checkSegment->len; checkSegment++) {
        segmentCount++;
        xferLen += checkSegment->len;

        if (checkSegment->len > DMA_MAJOR_COUNT_MAX) {
            dmaCapable = false;
        }
    }

#ifdef NXP_SPI_DMA
    // A list with a callback, such as the gyro read, is reused, so the threshold does not apply.
    const bool repeatedList = bus->curSegment->callback != NULL;

    if (spiUseDMA(dev) && dmaCapable
        && (repeatedList || SPI_DMA_WORTH_IT(segmentCount, xferLen))) {
        spiProcessSegmentsDMA(dev);
        return;
    }
#else
    UNUSED(xferLen);
    UNUSED(segmentCount);
    UNUSED(dmaCapable);
#endif

    spiProcessSegmentsPolled(dev);
}

#endif // USE_SPI
