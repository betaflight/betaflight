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
 * LPUART driver for the NXP parts. Transmit and receive share one interrupt per
 * instance and use the FIFO watermarks; eDMA is used where a port gets a channel.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "build/atomic.h"
#include "build/build_config.h"

#include "common/maths.h"
#include "common/utils.h"
#include "drivers/dma.h"
#include "drivers/dma_impl.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/serial_impl.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "drivers/time.h"

#include "platform/dma.h"
#include "platform/nxp_clock.h"

#include "dshot_flexio.h"
#include "iomux_nxp.h"
#include "iomux_signal_def.h"

#ifdef USE_DMA
#define NXP_LPUART_DMA
#endif

// Request source zero means no request on both families, so a row without this has no eDMA.
#ifdef USE_DMA
#define UART_DMA_REQUESTS(n)                            \
    .txDMAMuxId = kDmaRequestMuxLPUART ## n ## Tx,      \
    .rxDMAMuxId = kDmaRequestMuxLPUART ## n ## Rx,
#else
#define UART_DMA_REQUESTS(n)
#endif

// Pin arrays are left empty; pads are resolved against the generated nxpLpuartPins table.
FAST_RODATA const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .identifier = SERIAL_PORT_USART1,
        .reg = (usartResource_t *)LPUART1,
        .irqn = LPUART1_IRQn,
        UART_DMA_REQUESTS(1)
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART1,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
#endif
#ifdef USE_UART2
    {
        .identifier = SERIAL_PORT_USART2,
        .reg = (usartResource_t *)LPUART2,
        .irqn = LPUART2_IRQn,
        UART_DMA_REQUESTS(2)
        .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART2,
        .txBuffer = uart2TxBuffer,
        .rxBuffer = uart2RxBuffer,
        .txBufferSize = sizeof(uart2TxBuffer),
        .rxBufferSize = sizeof(uart2RxBuffer),
    },
#endif
#ifdef USE_UART3
    {
        .identifier = SERIAL_PORT_USART3,
        .reg = (usartResource_t *)LPUART3,
        .irqn = LPUART3_IRQn,
        UART_DMA_REQUESTS(3)
        .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART3,
        .txBuffer = uart3TxBuffer,
        .rxBuffer = uart3RxBuffer,
        .txBufferSize = sizeof(uart3TxBuffer),
        .rxBufferSize = sizeof(uart3RxBuffer),
    },
#endif
#ifdef USE_UART4
    {
        .identifier = SERIAL_PORT_UART4,
        .reg = (usartResource_t *)LPUART4,
        .irqn = LPUART4_IRQn,
        UART_DMA_REQUESTS(4)
        .txPriority = NVIC_PRIO_SERIALUART4_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART4,
        .txBuffer = uart4TxBuffer,
        .rxBuffer = uart4RxBuffer,
        .txBufferSize = sizeof(uart4TxBuffer),
        .rxBufferSize = sizeof(uart4RxBuffer),
    },
#endif
#ifdef USE_UART5
    {
        .identifier = SERIAL_PORT_UART5,
        .reg = (usartResource_t *)LPUART5,
        .irqn = LPUART5_IRQn,
        UART_DMA_REQUESTS(5)
        .txPriority = NVIC_PRIO_SERIALUART5_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART5,
        .txBuffer = uart5TxBuffer,
        .rxBuffer = uart5RxBuffer,
        .txBufferSize = sizeof(uart5TxBuffer),
        .rxBufferSize = sizeof(uart5RxBuffer),
    },
#endif
#ifdef USE_UART6
    {
        .identifier = SERIAL_PORT_USART6,
        .reg = (usartResource_t *)LPUART6,
        .irqn = LPUART6_IRQn,
        UART_DMA_REQUESTS(6)
        .txPriority = NVIC_PRIO_SERIALUART6_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART6,
        .txBuffer = uart6TxBuffer,
        .rxBuffer = uart6RxBuffer,
        .txBufferSize = sizeof(uart6TxBuffer),
        .rxBufferSize = sizeof(uart6RxBuffer),
    },
#endif
#ifdef USE_UART7
    {
        .identifier = SERIAL_PORT_USART7,
        .reg = (usartResource_t *)LPUART7,
        .irqn = LPUART7_IRQn,
        UART_DMA_REQUESTS(7)
        .txPriority = NVIC_PRIO_SERIALUART7_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART7,
        .txBuffer = uart7TxBuffer,
        .rxBuffer = uart7RxBuffer,
        .txBufferSize = sizeof(uart7TxBuffer),
        .rxBufferSize = sizeof(uart7RxBuffer),
    },
#endif
#ifdef USE_UART8
    {
        .identifier = SERIAL_PORT_USART8,
        .reg = (usartResource_t *)LPUART8,
        .irqn = LPUART8_IRQn,
        UART_DMA_REQUESTS(8)
        .txPriority = NVIC_PRIO_SERIALUART8_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART8,
        .txBuffer = uart8TxBuffer,
        .rxBuffer = uart8RxBuffer,
        .txBufferSize = sizeof(uart8TxBuffer),
        .rxBufferSize = sizeof(uart8RxBuffer),
    },
#endif
#ifdef USE_UART9
    {
        .identifier = SERIAL_PORT_UART9,
        .reg = (usartResource_t *)LPUART9,
        .irqn = LPUART9_IRQn,
        UART_DMA_REQUESTS(9)
        .txPriority = NVIC_PRIO_SERIALUART9_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART9,
        .txBuffer = uart9TxBuffer,
        .rxBuffer = uart9RxBuffer,
        .txBufferSize = sizeof(uart9TxBuffer),
        .rxBufferSize = sizeof(uart9RxBuffer),
    },
#endif
#ifdef USE_UART10
    {
        .identifier = SERIAL_PORT_USART10,
        .reg = (usartResource_t *)LPUART10,
        .irqn = LPUART10_IRQn,
        UART_DMA_REQUESTS(10)
        .txPriority = NVIC_PRIO_SERIALUART10_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART10,
        .txBuffer = uart10TxBuffer,
        .rxBuffer = uart10RxBuffer,
        .txBufferSize = sizeof(uart10TxBuffer),
        .rxBufferSize = sizeof(uart10RxBuffer),
    },
#endif
// LPUART11 and LPUART12 on the 11xx have no eDMA request source.
#ifdef USE_UART11
    {
        .identifier = SERIAL_PORT_UART11,
        .reg = (usartResource_t *)LPUART11,
        .irqn = LPUART11_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART11_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART11,
        .txBuffer = uart11TxBuffer,
        .rxBuffer = uart11RxBuffer,
        .txBufferSize = sizeof(uart11TxBuffer),
        .rxBufferSize = sizeof(uart11RxBuffer),
    },
#endif
#ifdef USE_UART12
    {
        .identifier = SERIAL_PORT_UART12,
        .reg = (usartResource_t *)LPUART12,
        .irqn = LPUART12_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART12_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART12,
        .txBuffer = uart12TxBuffer,
        .rxBuffer = uart12RxBuffer,
        .txBufferSize = sizeof(uart12TxBuffer),
        .rxBufferSize = sizeof(uart12RxBuffer),
    },
#endif
};

// The clock enum lists the LPUART instances in port order, one entry each.
STATIC_ASSERT(NXP_CLK_LPSPI1 - NXP_CLK_LPUART1 >= NXP_LPUART_COUNT,
              "the clock enum needs one entry per LPUART instance");

static nxpClock_e uartClock(serialPortIdentifier_e identifier)
{
    return (nxpClock_e)(NXP_CLK_LPUART1 + (identifier - SERIAL_PORT_USART1));
}

// baud = clock / (ratio * SBR), ratio 4 to 32 held in OSR as (ratio - 1), SBR 13 bits and non zero.
// A ratio below 8 needs BOTHEDGE. The largest ratio inside the error budget wins.
#define LPUART_OSR_MIN          4
#define LPUART_OSR_MAX          32
#define LPUART_SBR_MAX          LPUART_BAUD_SBR_MASK
// Rate error budget, in parts per thousand of the wanted rate.
#define LPUART_BAUD_ERROR_PPT   5

static void uartBaudDivider(uint32_t clockHz, uint32_t baudRate, uint32_t *ratioOut, uint32_t *sbrOut)
{
    if (baudRate == 0) {
        // Avoid a division by zero; give the slowest reachable rate.
        *ratioOut = LPUART_OSR_MAX;
        *sbrOut = LPUART_SBR_MAX;
        return;
    }

    const uint32_t errorBudget = baudRate * LPUART_BAUD_ERROR_PPT / 1000;

    uint32_t bestRatio = LPUART_OSR_MAX;
    uint32_t bestSbr = 1;
    uint32_t bestError = UINT32_MAX;
    uint32_t goodRatio = 0;
    uint32_t goodSbr = 0;

    for (uint32_t ratio = LPUART_OSR_MIN; ratio <= LPUART_OSR_MAX; ratio++) {
        const uint32_t step = baudRate * ratio;
        // Round to nearest, so the divider one above is considered too.
        const uint32_t sbr = constrain((clockHz + step / 2) / step, 1, LPUART_SBR_MAX);

        const uint32_t actual = clockHz / (ratio * sbr);
        const uint32_t error = actual > baudRate ? actual - baudRate : baudRate - actual;

        // Ties go to the later, larger ratio.
        if (error <= bestError) {
            bestError = error;
            bestRatio = ratio;
            bestSbr = sbr;
        }

        // The loop counts up, so the last one to pass is the largest.
        if (error <= errorBudget) {
            goodRatio = ratio;
            goodSbr = sbr;
        }
    }

    *ratioOut = goodRatio ? goodRatio : bestRatio;
    *sbrOut = goodRatio ? goodSbr : bestSbr;
}

// The write one to clear flags of STAT; the rest of the word holds settings that must be kept.
#define LPUART_STAT_W1C_FLAGS   (LPUART_STAT_PF_MASK | LPUART_STAT_FE_MASK | LPUART_STAT_NF_MASK \
                                 | LPUART_STAT_OR_MASK | LPUART_STAT_IDLE_MASK \
                                 | LPUART_STAT_MA1F_MASK | LPUART_STAT_MA2F_MASK \
                                 | LPUART_STAT_RXEDGIF_MASK | LPUART_STAT_LBKDIF_MASK)

// FIFO[RXFIFOSIZE/TXFIFOSIZE] code n means 2^(n+1) datawords, 0 means no FIFO.
// RDRF asserts above RXWATER, so a watermark two below the depth leaves one dataword free.
// RXIDEN raises RDRF after one idle character to drain the tail of a frame.
// TDRE asserts at or below TXWATER; STAT[TC] stays low until the FIFO and shifter are empty.
#define LPUART_RXWATER_MAX          (LPUART_WATER_RXWATER_MASK >> LPUART_WATER_RXWATER_SHIFT)
#define LPUART_TXWATER_MAX          (LPUART_WATER_TXWATER_MASK >> LPUART_WATER_TXWATER_SHIFT)
#define LPUART_RXIDEN_ONE_CHARACTER 1
#define LPUART_TXWATER_WANTED       1

STATIC_ASSERT(LPUART_TXWATER_WANTED <= LPUART_TXWATER_MAX,
              "the transmit watermark has to fit the field");

static uint32_t uartFifoDepth(uint32_t sizeCode)
{
    return sizeCode ? (1u << (sizeCode + 1)) : 1u;
}

static uint32_t uartRxWatermark(const LPUART_Type *lpuart)
{
    const uint32_t depth =
        uartFifoDepth((lpuart->FIFO & LPUART_FIFO_RXFIFOSIZE_MASK) >> LPUART_FIFO_RXFIFOSIZE_SHIFT);

    return depth > 2 ? MIN(depth - 2, (uint32_t)LPUART_RXWATER_MAX) : 0;
}

static uint32_t uartTxFifoDepth(const LPUART_Type *lpuart)
{
    return uartFifoDepth((lpuart->FIFO & LPUART_FIFO_TXFIFOSIZE_MASK) >> LPUART_FIFO_TXFIFOSIZE_SHIFT);
}

// Cached transmit FIFO depth per port, saving a peripheral bus read per interrupt.
static uint8_t uartTxDepth[UARTDEV_COUNT];

// Which entry of uartDevice[] a port is, and so its place in the array above.
static unsigned uartDeviceIndex(uartPort_t *s)
{
    return container_of(s, uartDevice_t, port) - uartDevice;
}

// The muxing that puts a port's signal on a pin, NULL if the chip cannot.
static const nxpPinMux_t *uartPadMux(serialPortIdentifier_e identifier, ioTag_t pin, bool transmit)
{
    const uint8_t instance = identifier - SERIAL_PORT_USART1 + 1;

    return NXP_PIN_MUX(nxpLpuartPins, instance,
                         transmit ? NXP_ROLE_LPUART_TX : NXP_ROLE_LPUART_RX, pin);
}

// A pad that cannot carry the signal is left alone.
static void uartConfigurePad(serialPortIdentifier_e identifier, ioTag_t pin, bool transmit, ioConfig_t cfg)
{
    nxpIomuxConfigPin(uartPadMux(identifier, pin, transmit), cfg);
}

static bool uartIsSingleWire(const uartDevice_t *uartdev, portOptions_e options)
{
    return (options & SERIAL_BIDIR)
        || (uartdev->tx.pin && uartdev->tx.pin == uartdev->rx.pin);
}

#if SERIAL_PIOUART_COUNT > 0
// PIOUART ports run on FlexIO shifters, in serial_flexio_nxp.c.
static inline bool uartIsFlexio(serialPortIdentifier_e identifier)
{
    return (unsigned)(identifier - SERIAL_PORT_PIOUART_FIRST) < SERIAL_PIOUART_COUNT;
}
#endif

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
    for (const uartHardware_t *hardware = uartHardware; hardware < ARRAYEND(uartHardware); hardware++) {
        const serialPortIdentifier_e identifier = hardware->identifier;
        uartDevice_t *uartdev = uartDeviceFromIdentifier(identifier);
        const int resourceIndex = serialResourceIndex(identifier);
        if (uartdev == NULL || resourceIndex < 0) {
            continue;
        }

        const ioTag_t cfgRx = pSerialPinConfig->ioTagRx[resourceIndex];
        const ioTag_t cfgTx = pSerialPinConfig->ioTagTx[resourceIndex];

        // A pad the chip cannot route to leaves the port off.
        if (uartPadMux(identifier, cfgRx, false)) {
            uartdev->rx.pin = cfgRx;
        }

        if (uartPadMux(identifier, cfgTx, true)) {
            uartdev->tx.pin = cfgTx;
        }

        if (uartdev->rx.pin || uartdev->tx.pin) {
            uartdev->hardware = hardware;
        }
    }

#if SERIAL_PIOUART_COUNT > 0
    flexioUartPinConfigure(pSerialPinConfig);
#endif
}

// The LPUART inverts on the pin itself, set up in uartReconfigure().
void uartConfigureExternalPinInversion(uartPort_t *uartPort)
{
    UNUSED(uartPort);
}

// Receive is one endless descriptor over the ring: DLAST wraps the destination and the
// CITER reload from BITER is the wrap, so CITER is the read cursor. Transmit is one
// descriptor per contiguous chunk, chained by the completion interrupt.
// The buffers are in .bss, which is DTCM and so reachable by the engine.

#ifdef NXP_LPUART_DMA

// A finished channel reads back its chunk size after the CITER reload, so it is parked here.
static FAST_RODATA const dmaTcd_t uartDmaIdleTcd;

static dmaIdentifier_e uartClaimDmaChannel(uartDevice_t *uartdev, resourceOwner_e owner,
                                           uint32_t muxId, uint32_t priority)
{
    if (muxId == 0) {
        // Source zero is no request.
        return DMA_NONE;
    }

    const dmaIdentifier_e channel = dmaGetFreeIdentifierForRequest(muxId);
    if (channel == DMA_NONE
        || !dmaAllocate(channel, owner, serialOwnerIndex(uartdev->port.port.identifier))) {
        return DMA_NONE;
    }

    dmaEnable(channel);
    dmaMuxEnable(channel, muxId);
    // The receive ring reaches the handler only through the error vector.
    // userParam is 32 bits, so it carries the port index rather than its address.
    dmaSetHandler(channel, uartDmaIrqHandler, priority,
                  (uint32_t)uartDeviceIdxFromIdentifier(uartdev->port.port.identifier));

    return channel;
}

// Called from uartReconfigure(), after the generic layer has set the receive callback.
void uartConfigureDma(uartDevice_t *uartdev)
{
    uartPort_t *s = &uartdev->port;
    const uartHardware_t *hardware = uartdev->hardware;
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(s->port.identifier); // rx is always +1

    // Channels stay claimed, so a reopened port takes its own back.
    if (s->rxDMAChannel != DMA_NONE) {
        dmaDisableRequest((dmaIdentifier_e)s->rxDMAChannel);
    }
    if (s->txDMAChannel != DMA_NONE) {
        dmaDisableRequest((dmaIdentifier_e)s->txDMAChannel);
    }

    s->rxDMAResource = NULL;
    s->txDMAResource = NULL;

    // The half duplex turnaround needs the interrupt paths.
    if (uartIsSingleWire(uartdev, s->port.options)) {
        return;
    }

    // A receive callback is fed from the interrupt, so such a port keeps it.
    if ((s->port.mode & MODE_RX) && !s->port.rxCallback) {
        if (s->rxDMAChannel == DMA_NONE) {
            s->rxDMAChannel = uartClaimDmaChannel(uartdev, ownerTxRx + 1,
                                                  hardware->rxDMAMuxId, hardware->rxPriority);
        }
        if (s->rxDMAChannel != DMA_NONE) {
            s->rxDMAResource = dmaDescriptors[DMA_IDENTIFIER_TO_CHANNEL(s->rxDMAChannel)].ref;
        }
    }

    if (s->port.mode & MODE_TX) {
        if (s->txDMAChannel == DMA_NONE) {
            s->txDMAChannel = uartClaimDmaChannel(uartdev, ownerTxRx,
                                                  hardware->txDMAMuxId, hardware->txPriority);
        }
        if (s->txDMAChannel != DMA_NONE) {
            s->txDMAResource = dmaDescriptors[DMA_IDENTIFIER_TO_CHANNEL(s->txDMAChannel)].ref;
        }
    }
}

static void uartStartRxDma(uartPort_t *s)
{
    const dmaTransfer_t transfer = {
        // A byte wide read of DATA takes its low byte, the received character.
        .src = (const void *)&UART_INST(s->USARTx)->DATA,
        .dst = (void *)s->port.rxBuffer,
        .srcOffset = 0,
        .dstOffset = 1,
        .srcSize = DMA_XFER_SIZE_1B,
        .dstSize = DMA_XFER_SIZE_1B,
        .minorBytes = 1,
        .majorCount = s->port.rxBufferSize,
        .srcLastAdjust = 0,
        // Back to the start of the ring when the major loop ends.
        .dstLastAdjust = -(int32_t)s->port.rxBufferSize,
    };

    dmaTcd_t tcd;
    if (!dmaBuildTcd(&tcd, &transfer)) {
        // The ring is longer than a major loop can count; fall back to the interrupt.
        s->rxDMAResource = NULL;
        return;
    }

    dmaLoadTcd((dmaIdentifier_e)s->rxDMAChannel, &tcd);
    dmaEnableRequest((dmaIdentifier_e)s->rxDMAChannel);

    s->rxDMAPos = dmaGetDataLength(s->rxDMAResource);
}

// Returns the BAUD bits that hand the port's requests over to the channels.
static uint32_t uartStartDma(uartPort_t *s)
{
    uint32_t baud = 0;

    if (s->rxDMAResource) {
        uartStartRxDma(s);              // may give the channel back
    }

    if (s->rxDMAResource) {
        baud |= LPUART_BAUD_RDMAE_MASK;
    }

    if (s->txDMAResource) {
        // uartTryStartTxDMA() at the end of the reconfigure parks the channel.
        s->txDMAEmpty = true;
        baud |= LPUART_BAUD_TDMAE_MASK;
    }

    return baud;
}

FAST_CODE void uartTryStartTxDMA(uartPort_t *s)
{
    uartDevice_t *uartdev = container_of(s, uartDevice_t, port);
    LPUART_Type *lpuart = UART_INST(s->USARTx);
    const dmaIdentifier_e channel = (dmaIdentifier_e)s->txDMAChannel;

    // Called from task level and from the completion interrupt.
    ATOMIC_BLOCK(uartdev->hardware->txPriority) {
        if (dmaChannelBusy(channel)) {
            // Its completion comes back here for whatever is queued by then.
            return;
        }

        // Read once, as task level moves the head while this runs from the interrupt.
        const uint32_t head = s->port.txBufferHead;
        const uint32_t tail = s->port.txBufferTail;

        if (head == tail) {
            dmaLoadTcd(channel, &uartDmaIdleTcd);

            if (s->txDMAEmpty) {
                // Nothing was in flight, so the pad is already where it belongs.
                return;
            }

            s->txDMAEmpty = true;

            if (uartdev->txPinState != TX_PIN_IGNORE) {
                // TC is high whenever the transmitter is idle, so arm it only once all is queued.
                ATOMIC_BLOCK(uartdev->hardware->rxPriority) {
                    lpuart->CTRL |= LPUART_CTRL_TCIE_MASK;
                }
            }

            return;
        }

        // Up to the end of the buffer; the completion sends the piece after the wrap.
        const uint32_t chunk = (head > tail) ? head - tail : s->port.txBufferSize - tail;

        const dmaTransfer_t transfer = {
            .src = (const void *)&s->port.txBuffer[tail],
            .dst = (void *)&lpuart->DATA,
            .srcOffset = 1,
            .dstOffset = 0,
            .srcSize = DMA_XFER_SIZE_1B,
            .dstSize = DMA_XFER_SIZE_1B,
            .minorBytes = 1,
            .majorCount = (uint16_t)chunk,
            .srcLastAdjust = 0,
            .dstLastAdjust = 0,
        };

        // DREQ stops the channel at the end of the chunk.
        if (!dmaSetTransfer(channel, &transfer, true)) {
            return;
        }

        // Load before moving the tail, so the in flight count covers these bytes.
        s->port.txBufferTail = (tail + chunk) % s->port.txBufferSize;
        s->txDMAEmpty = false;

        // A transmit complete armed for the last chunk must not end this one.
        if (uartdev->txPinState != TX_PIN_IGNORE) {
            ATOMIC_BLOCK(uartdev->hardware->rxPriority) {
                lpuart->CTRL &= ~LPUART_CTRL_TCIE_MASK;
            }
        }

        dmaEnableRequest(channel);
    }
}

FAST_IRQ_HANDLER void uartDmaIrqHandler(dmaChannelDescriptor_t *descriptor)
{
    uartPort_t *s = &uartDevice[descriptor->userParam].port;

    if (descriptor->ref == s->rxDMAResource) {
        // The receive ring has no completion interrupt, so this is an error; reload the ring.
        uartStartRxDma(s);
        return;
    }

    // Done or stopped on an error: the channel is idle either way.
    uartTryStartTxDMA(s);
}

#else // NXP_LPUART_DMA

void uartConfigureDma(uartDevice_t *uartdev)
{
    UNUSED(uartdev);
}

void uartTryStartTxDMA(uartPort_t *s)
{
    UNUSED(s);
}

#endif // NXP_LPUART_DMA

// Both read false in a build with no serial DMA.
static bool uartRxIsDma(const uartPort_t *s)
{
#ifdef NXP_LPUART_DMA
    return s->rxDMAResource != NULL;
#else
    UNUSED(s);
    return false;
#endif
}

static bool uartTxIsDma(const uartPort_t *s)
{
#ifdef NXP_LPUART_DMA
    return s->txDMAResource != NULL;
#else
    UNUSED(s);
    return false;
#endif
}

// CTRL is shared with the port's interrupt, so every read-modify-write runs with it masked.
FAST_CODE void uartTxMonitor(uartPort_t *s)
{
    uartDevice_t *uartdev = container_of(s, uartDevice_t, port);

    if (uartdev->txPinState == TX_PIN_ACTIVE) {
        LPUART_Type *lpuart = UART_INST(s->USARTx);

        // Pull up, so an unpowered peripheral holding the line low can be seen.
        IOConfigGPIO(IOGetByTag(uartdev->tx.pin), IOCFG_IPU);

        ATOMIC_BLOCK(uartdev->hardware->rxPriority) {
            lpuart->CTRL &= ~LPUART_CTRL_TE_MASK;
            uartdev->txPinState = TX_PIN_MONITOR;
        }
    }
}

static timeUs_t txHeldLowAt[UARTDEV_COUNT];

FAST_CODE bool checkUsartTxOutput(uartPort_t *s)
{
    uartDevice_t *uartdev = container_of(s, uartDevice_t, port);
    IO_t txIO = IOGetByTag(uartdev->tx.pin);

    if (uartdev->txPinState == TX_PIN_MONITOR && txIO) {
        // Low for under a bit time, or while RAF is set, is a coupled spike rather than a held line.
        const LPUART_Type *lpuart = UART_INST(s->USARTx);
        const timeUs_t start = micros();
        const timeDelta_t window = MAX(10, s->port.baudRate ? 1000000 / (int)s->port.baudRate : 0);
        // A pad held low within the last character time is still held.
        timeUs_t *heldLowAt = &txHeldLowAt[uartdev - uartDevice];
        const timeDelta_t limit = (*heldLowAt && start - *heldLowAt < 10u * window) ? 0 : window;
        while (!IORead(txIO) && !(lpuart->STAT & LPUART_STAT_RAF_MASK)) {
            if (cmpTimeUs(micros(), start) >= limit) {
                *heldLowAt = micros();
                return false;
            }
        }
        *heldLowAt = 0;

        // Enable the transmitter before the pad goes back to it.
        ATOMIC_BLOCK(uartdev->hardware->rxPriority) {
            uartdev->txPinState = TX_PIN_ACTIVE;
            UART_INST(s->USARTx)->CTRL |= LPUART_CTRL_TE_MASK;
        }

        uartConfigurePad(s->port.identifier, uartdev->tx.pin, true, IOCFG_AF_PP);
    }

    return true;
}

void uartReconfigure(uartPort_t *s)
{
#if SERIAL_PIOUART_COUNT > 0
    if (uartIsFlexio(s->port.identifier)) {
        flexioUartReconfigure(s);
        return;
    }
#endif

    LPUART_Type *lpuart = UART_INST(s->USARTx);
    uartDevice_t *uartdev = container_of(s, uartDevice_t, port);
    const portOptions_e options = s->port.options;
    const portMode_e mode = s->port.mode;

    // GLOBAL[RST] clears every other register, including what the boot ROM serial
    // downloader left behind, and CTRL, which must be zero before BAUD changes.
    lpuart->GLOBAL = LPUART_GLOBAL_RST_MASK;
    lpuart->GLOBAL = 0;

    uartConfigureDma(uartdev);

    uint32_t ratio;
    uint32_t sbr;
    uartBaudDivider(nxpClockHz(uartClock(s->port.identifier)), s->port.baudRate, &ratio, &sbr);

    uint32_t baud = LPUART_BAUD_OSR(ratio - 1) | LPUART_BAUD_SBR(sbr);
    if (ratio < 8) {
        baud |= LPUART_BAUD_BOTHEDGE_MASK;
    }
    if (options & SERIAL_STOPBITS_2) {
        baud |= LPUART_BAUD_SBNS_MASK;
    }
#ifdef NXP_LPUART_DMA
    baud |= uartStartDma(s);
#endif
    lpuart->BAUD = baud;

    // RXINV may only be written while the receiver is off.
    lpuart->STAT = ((options & SERIAL_INVERTED) ? LPUART_STAT_RXINV_MASK : 0) | LPUART_STAT_W1C_FLAGS;

    // RXWATER may only be changed while the receiver is off. Without a transmit FIFO
    // TXWATER must stay zero, or TDRE never asserts.
    const uint32_t txDepth = uartTxFifoDepth(lpuart);
    const uint32_t txWater = txDepth > LPUART_TXWATER_WANTED ? LPUART_TXWATER_WANTED : 0;

    uartTxDepth[uartDeviceIndex(s)] = txDepth;

    // A receive channel takes every byte as it lands.
    lpuart->WATER = LPUART_WATER_RXWATER(uartRxIsDma(s) ? 0 : uartRxWatermark(lpuart))
                  | LPUART_WATER_TXWATER(txWater);

    // Both FIFOs on, flushed, with the idle drain for the tail of a frame.
    lpuart->FIFO = LPUART_FIFO_TXFLUSH_MASK | LPUART_FIFO_RXFLUSH_MASK
                 | LPUART_FIFO_RXUF_MASK | LPUART_FIFO_TXOF_MASK
                 | LPUART_FIFO_RXFE_MASK | LPUART_FIFO_RXIDEN(LPUART_RXIDEN_ONE_CHARACTER)
                 | (txDepth > 1 ? LPUART_FIFO_TXFE_MASK : 0);

    // No hardware flow control and no IrDA.
    lpuart->MODIR = 0;

    uint32_t ctrl = 0;

    if (options & SERIAL_PARITY_EVEN) {
        // Parity takes the top bit, so nine bit words keep eight data bits. PT 0 is even.
        ctrl |= LPUART_CTRL_PE_MASK | LPUART_CTRL_M_MASK;
    }

    if (options & SERIAL_INVERTED) {
        ctrl |= LPUART_CTRL_TXINV_MASK;
    }

    if (uartIsSingleWire(uartdev, options)) {
        // The receiver listens on the transmit pad and TXDIR sets its direction.
        ctrl |= LPUART_CTRL_LOOPS_MASK | LPUART_CTRL_RSRC_MASK;
    }

    if (mode & MODE_RX) {
        // An overrun stalls the receiver until the interrupt clears it.
        ctrl |= LPUART_CTRL_RE_MASK
              | LPUART_CTRL_ORIE_MASK | LPUART_CTRL_FEIE_MASK | LPUART_CTRL_NEIE_MASK;

        // With DMA, BAUD[RDMAE] routes RDRF to the engine instead.
        if (!uartRxIsDma(s)) {
            ctrl |= LPUART_CTRL_RIE_MASK | LPUART_CTRL_ILIE_MASK;
        }
    }

    // A watched line keeps TE off until checkUsartTxOutput() sees it high. TE is set
    // once only, as every off to on change sends an idle character.
    if ((mode & MODE_TX) && uartdev->txPinState != TX_PIN_MONITOR) {
        ctrl |= LPUART_CTRL_TE_MASK;
    }

    lpuart->CTRL = ctrl;

    // With the transmitter on, park the channel and send what is left in the ring.
    if (uartTxIsDma(s)) {
        uartTryStartTxDMA(s);
    }
}

FAST_CODE void uartEnableTxInterrupt(uartPort_t *uartPort)
{
#if SERIAL_PIOUART_COUNT > 0
    if (uartIsFlexio(uartPort->port.identifier)) {
        flexioUartEnableTxInterrupt(uartPort);
        return;
    }
#endif

    const uartDevice_t *uartdev = container_of(uartPort, uartDevice_t, port);
    LPUART_Type *lpuart = UART_INST(uartPort->USARTx);

    // TC is high whenever the transmitter is idle, so TCIE is armed here and cleared in the interrupt.
    uint32_t bits = LPUART_CTRL_TIE_MASK;

    if (uartdev->txPinState != TX_PIN_IGNORE) {
        bits |= LPUART_CTRL_TCIE_MASK;
    }

    if (uartIsSingleWire(uartdev, uartPort->port.options)) {
        // Turn the pad around before the first start bit; TC turns it back.
        bits |= LPUART_CTRL_TCIE_MASK | LPUART_CTRL_TXDIR_MASK;
    }

    ATOMIC_BLOCK(uartdev->hardware->rxPriority) {
        lpuart->CTRL |= bits;
    }
}

FAST_IRQ_HANDLER void uartIrqHandler(uartPort_t *s)
{
    LPUART_Type *lpuart = UART_INST(s->USARTx);
    const uint32_t ctrl = lpuart->CTRL;
    uint32_t stat = lpuart->STAT;

    if (ctrl & LPUART_CTRL_RIE_MASK) {
        // One read of RXCOUNT, as every register read stalls on the peripheral bus.
        const unsigned waiting = (lpuart->WATER & LPUART_WATER_RXCOUNT_MASK) >> LPUART_WATER_RXCOUNT_SHIFT;

        for (unsigned i = 0; i < waiting; i++) {
            const uint8_t ch = lpuart->DATA;
            if (s->port.rxCallback) {
                s->port.rxCallback(ch, s->port.rxCallbackData);
            } else {
                // A full ring drops the new byte.
                const uint32_t next = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
                if (next != s->port.rxBufferTail) {
                    s->port.rxBuffer[s->port.rxBufferHead] = ch;
                    s->port.rxBufferHead = next;
                }
            }
        }
    }

    const uint32_t settings = stat & ~LPUART_STAT_W1C_FLAGS;

    // Cleared after the drain above, so the receiver has room again.
    const uint32_t errors = stat & (LPUART_STAT_OR_MASK | LPUART_STAT_FE_MASK
                                  | LPUART_STAT_NF_MASK | LPUART_STAT_PF_MASK);
    if (errors) {
        lpuart->STAT = settings | errors;
    }

    if ((ctrl & LPUART_CTRL_ILIE_MASK) && (stat & LPUART_STAT_IDLE_MASK)) {
        lpuart->STAT = settings | LPUART_STAT_IDLE_MASK;
        if (s->port.idleCallback) {
            s->port.idleCallback();
        }
    }

    if ((ctrl & LPUART_CTRL_TIE_MASK) && (stat & LPUART_STAT_TDRE_MASK)) {
        // TDRE means down to the watermark, so the room is the depth less WATER[TXCOUNT].
        const uint32_t depth = uartTxDepth[uartDeviceIndex(s)];
        const uint32_t held = (lpuart->WATER & LPUART_WATER_TXCOUNT_MASK) >> LPUART_WATER_TXCOUNT_SHIFT;
        const uint32_t room = depth > held ? depth - held : 0;
        bool wrote = false;

        for (uint32_t i = 0; i < room && s->port.txBufferTail != s->port.txBufferHead; i++) {
            lpuart->DATA = s->port.txBuffer[s->port.txBufferTail];
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
            wrote = true;
        }

        if (s->port.txBufferTail == s->port.txBufferHead) {
            lpuart->CTRL &= ~LPUART_CTRL_TIE_MASK;
        }

        if (wrote) {
            // Writing DATA clears TC.
            stat = lpuart->STAT;
        }
    }

    // With DMA the ring reads empty while a chunk is still in flight.
    if ((ctrl & LPUART_CTRL_TCIE_MASK) && (stat & LPUART_STAT_TC_MASK)
        && s->port.txBufferTail == s->port.txBufferHead
        && (!uartTxIsDma(s) || s->txDMAEmpty)) {
        // The last stop bit has gone; release the line.
        lpuart->CTRL &= ~(LPUART_CTRL_TCIE_MASK | LPUART_CTRL_TXDIR_MASK);
        uartTxMonitor(s);
    }
}

uartPort_t *serialUART(uartDevice_t *uartdev, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
#if SERIAL_PIOUART_COUNT > 0
    if (uartIsFlexio(uartdev->port.port.identifier)) {
        return flexioUartOpen(uartdev, baudRate, mode, options);
    }
#endif

    uartPort_t *s = &uartdev->port;
    const uartHardware_t *hardware = uartdev->hardware;

    s->port.vTable = uartVTable;
    s->port.baudRate = baudRate;
    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;
    s->USARTx = hardware->reg;
    s->checkUsartTxOutput = checkUsartTxOutput;

    if (!nxpClockGateEnable(uartClock(hardware->identifier))) {
        return NULL;
    }

    // Reset before the pads go live; GLOBAL[RST] is not self clearing.
    LPUART_Type *lpuart = UART_INST(s->USARTx);
    lpuart->GLOBAL = LPUART_GLOBAL_RST_MASK;
    lpuart->GLOBAL = 0;

    const serialPortIdentifier_e identifier = s->port.identifier;
    const int ownerIndex = serialOwnerIndex(identifier);
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(identifier); // rx is always +1

    IO_t txIO = IOGetByTag(uartdev->tx.pin);
    IO_t rxIO = IOGetByTag(uartdev->rx.pin);

    uartdev->txPinState = TX_PIN_IGNORE;

    if (uartIsSingleWire(uartdev, options) && txIO) {
        // The pull holds the idle line while neither end is driving it.
        const serialPullMode_t pull = serialOptions_pull(options);
        const ioConfig_t cfg = IO_CONFIG(IOCFG_MODE_AF, IOCFG_DRIVE_FAST,
                                         ((const unsigned[]){ IOCFG_PULL_NONE, IOCFG_PULL_DOWN, IOCFG_PULL_UP })[pull],
                                         serialOptions_pushPull(options) ? 0 : 1, 1);
        IOInit(txIO, ownerTxRx, ownerIndex);
        uartConfigurePad(identifier, uartdev->tx.pin, true, cfg);
    } else {
        if ((mode & MODE_TX) && txIO) {
            IOInit(txIO, ownerTxRx, ownerIndex);

            if (options & SERIAL_CHECK_TX) {
                uartdev->txPinState = TX_PIN_ACTIVE;
                uartTxMonitor(s);
            } else {
                uartConfigurePad(identifier, uartdev->tx.pin, true, IOCFG_AF_PP);
            }
        }

        if ((mode & MODE_RX) && rxIO) {
            IOInit(rxIO, ownerTxRx + 1, ownerIndex);
            uartConfigurePad(identifier, uartdev->rx.pin, false, IOCFG_AF_PP_UP);
        }
    }

    NVIC_SetPriority((IRQn_Type)hardware->irqn, NVIC_PRIORITY_LEVEL(hardware->rxPriority));
    NVIC_EnableIRQ((IRQn_Type)hardware->irqn);

    return s;
}

// One vector per instance for receive, transmit and errors.
#define LPUART_IRQHandler(number, dev)                          \
    FAST_IRQ_HANDLER void LPUART ## number ## _IRQHandler(void) \
    {                                                           \
        uartIrqHandler(&uartDevice[(dev)].port);                \
    }                                                           \
/**/

#ifdef USE_UART1
LPUART_IRQHandler(1, UARTDEV_1)
#endif
#ifdef USE_UART2
LPUART_IRQHandler(2, UARTDEV_2)
#endif
#ifdef USE_UART3
LPUART_IRQHandler(3, UARTDEV_3)
#endif
#ifdef USE_UART4
LPUART_IRQHandler(4, UARTDEV_4)
#endif
#ifdef USE_UART5
LPUART_IRQHandler(5, UARTDEV_5)
#endif
#ifdef USE_UART6
LPUART_IRQHandler(6, UARTDEV_6)
#endif
#ifdef USE_UART7
LPUART_IRQHandler(7, UARTDEV_7)
#endif
#ifdef USE_UART8
LPUART_IRQHandler(8, UARTDEV_8)
#endif
#ifdef USE_UART9
LPUART_IRQHandler(9, UARTDEV_9)
#endif
#ifdef USE_UART10
LPUART_IRQHandler(10, UARTDEV_10)
#endif
#ifdef USE_UART11
LPUART_IRQHandler(11, UARTDEV_11)
#endif
#ifdef USE_UART12
LPUART_IRQHandler(12, UARTDEV_12)
#endif

#endif // USE_UART
