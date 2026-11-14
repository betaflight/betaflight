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
 * i.MX 95 LPUART back-end for the common serial UART driver
 * (drivers/serial_uart.c). UART5 is LPUART5 and UART7 is LPUART7; pads are
 * muxed through the System Manager over SCMI.
 */

#include "platform.h"

#ifdef USE_UART

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_impl.h"
#include "drivers/serial_uart_impl.h"

extern int navq95_scmi_clock_enable(uint32_t clockRootId);

#define LPUART7_BASE          0x42690000u

#define LPUART_GLOBAL_OFFSET  0x08u
#define LPUART_BAUD_OFFSET    0x10u
#define LPUART_STAT_OFFSET    0x14u
#define LPUART_CTRL_OFFSET    0x18u
#define LPUART_DATA_OFFSET    0x1Cu
#define LPUART_MODIR_OFFSET   0x24u
#define LPUART_FIFO_OFFSET    0x28u

#define LPUART_GLOBAL_RST     (1u << 1)
#define LPUART_FIFO_RXFE      (1u << 3)
#define LPUART_FIFO_TXFE      (1u << 7)
#define LPUART_FIFO_RXFLUSH   (1u << 14)
#define LPUART_FIFO_TXFLUSH   (1u << 15)
#define LPUART_STAT_TDRE      (1u << 23)
#define LPUART_STAT_RDRF      (1u << 21)
#define LPUART_STAT_IDLE      (1u << 20)
#define LPUART_STAT_OR        (1u << 19)
#define LPUART_STAT_NF        (1u << 18)
#define LPUART_STAT_FE        (1u << 17)
#define LPUART_STAT_PF        (1u << 16)
#define LPUART_STAT_W1C       (LPUART_STAT_IDLE | LPUART_STAT_OR | LPUART_STAT_NF | \
                               LPUART_STAT_FE | LPUART_STAT_PF)
#define LPUART_CTRL_TIE       (1u << 23)
#define LPUART_CTRL_RIE       (1u << 21)
#define LPUART_CTRL_RE        (1u << 18)
#define LPUART_CTRL_TE        (1u << 19)
#define LPUART_BAUD_SBR(n)    ((uint32_t)(n) & 0x1FFFu)
#define LPUART_BAUD_BOTHEDGE  (1u << 17)
#define LPUART_BAUD_OSR(n)    (((uint32_t)((n) - 1u) & 0x1Fu) << 24)
#define LPUART_DATA_MASK      0xFFu

// Framing options used for SBUS.
#define LPUART_CTRL_PE        (1u << 1)
#define LPUART_CTRL_PT        (1u << 0)
#define LPUART_CTRL_M         (1u << 4)
#define LPUART_BAUD_SBNS      (1u << 13)
#define LPUART_STAT_RXINV     (1u << 28)

// The LPUART clock roots run from the 24 MHz oscillator.
#define NAVQ95_LPUART_CLK_HZ  24000000u

static inline volatile uint32_t *lpReg(const usartResource_t *reg, uint32_t off)
{
    return (volatile uint32_t *)((uint32_t)(uintptr_t)reg + off);
}

#define IOMUXC_BASE           0x443C0000u
#define IOMUXC_MUXREG_OFF     0x0000u
#define IOMUXC_DAISYREG_OFF   0x0408u

#define IOMUXC_MUX_UART7_TX   0x443C0030u
#define IOMUXC_MUX_UART7_RX   0x443C0034u
#define IOMUXC_PAD_UART7_TX   0x443C0234u
#define IOMUXC_PAD_UART7_RX   0x443C0238u
#define IOMUXC_DSY_UART7_RX   0x443C0584u
#define LPUART7_ALT_MODE      0x5u
#define LPUART7_RX_DAISY      0x1u

// TX: slow slew, drive strength 4. RX: pull-up and hysteresis.
#define IOMUXC_PAD_UART7_TXVAL (0x1Eu >> 1)
#define IOMUXC_PAD_UART7_RXVAL (0x1200u >> 1)

// LPUART5 uses shared pads, with an input select (daisy) on both TX and RX.
#define LPUART5_BASE          0x42590000u
#define LPUART5_CLK_ROOT      103u
#define IOMUXC_MUX_UART5_TX   0x443C0010u
#define IOMUXC_MUX_UART5_RX   0x443C0014u
#define IOMUXC_PAD_UART5_TX   0x443C0214u
#define IOMUXC_PAD_UART5_RX   0x443C0218u
#define IOMUXC_DSY_UART5_TX   0x443C0574u
#define IOMUXC_DSY_UART5_RX   0x443C0570u
#define LPUART5_ALT_MODE      0x5u
#define LPUART5_DAISY         0x1u

#define IOMUXC_PAD_UART5_TXVAL (0x1Eu >> 1)
#define IOMUXC_PAD_UART5_RXVAL (0x1200u >> 1)

#define MU5_BASE              0x44610000u
#define MU5_GCR               (MU5_BASE + 0x0114u)
#define MU5_GCR_GIR0          (1u << 0)
#define SMT_A2P_BUF           0x44611000u
#define SMT_CHAN_STATUS       (SMT_A2P_BUF + 0x04u)
#define SMT_IMP_STATUS        (SMT_A2P_BUF + 0x08u)
#define SMT_IMP_CRC           (SMT_A2P_BUF + 0x0Cu)
#define SMT_CHAN_FLAGS        (SMT_A2P_BUF + 0x10u)
#define SMT_LENGTH            (SMT_A2P_BUF + 0x14u)
#define SMT_MSG_HEADER        (SMT_A2P_BUF + 0x18u)
#define SMT_PAYLOAD           (SMT_A2P_BUF + 0x1Cu)
#define SMT_FREE              (1u << 0)
#define SMT_CRC_NONE          0u
#define SMT_CRC_XOR           1u
#define SMT_CRC_CRC32         3u

#define SCMI_PROTO_PINCTRL    0x19u
#define SCMI_MSG_PINCTRL_SET  0x06u
#define SCMI_PINCTRL_TYPE_MUX       192u
#define SCMI_PINCTRL_TYPE_CONFIG    193u
#define SCMI_PINCTRL_TYPE_DAISY_ID  194u
#define SCMI_PINCTRL_TYPE_DAISY_CFG 195u

static inline void w32(uint32_t a, uint32_t v) { *(volatile uint32_t *)a = v; }
static inline uint32_t r32(uint32_t a)         { return *(volatile uint32_t *)a; }

static uint32_t navq95_crcxor(const uint32_t *addr, uint32_t words)
{
    uint32_t crc = 0u;
    while (words--) {
        crc ^= *addr++;
    }
    return crc;
}

static uint32_t navq95_crc32(const uint8_t *addr, uint32_t nbytes)
{
    static const uint32_t crc_table[] = {
        0x4dbdf21cu, 0x500ae278u, 0x76d3d2d4u, 0x6b64c2b0u,
        0x3b61b38cu, 0x26d6a3e8u, 0x000f9344u, 0x1db88320u,
        0xa005713cu, 0xbdb26158u, 0x9b6b51f4u, 0x86dc4190u,
        0xd6d930acu, 0xcb6e20c8u, 0xedb71064u, 0xf0000000u
    };
    uint32_t crc = 0u;
    while (nbytes--) {
        crc = (crc >> 4u) ^ crc_table[(crc ^ ((uint32_t)(*addr) >> 0u)) & 0x0Fu];
        crc = (crc >> 4u) ^ crc_table[(crc ^ ((uint32_t)(*addr) >> 4u)) & 0x0Fu];
        addr++;
    }
    return crc;
}

static void navq95_smt_crc(uint32_t len)
{
    uint32_t impStatus = r32(SMT_IMP_STATUS);
    if (impStatus == SMT_CRC_XOR) {
        w32(SMT_IMP_CRC, navq95_crcxor((const uint32_t *)SMT_MSG_HEADER, len / 4u));
    } else if (impStatus == SMT_CRC_CRC32) {
        w32(SMT_IMP_CRC, navq95_crc32((const uint8_t *)SMT_MSG_HEADER, len));
    }
}

// SCMI PINCTRL_CONFIG_SET for one pad; dsyReg != 0 adds the input select.
static int navq95_scmi_pinmux(uint32_t muxReg, uint32_t muxMode, uint32_t sion,
                              uint32_t padVal, uint32_t dsyReg, uint32_t dsyVal)
{
    static uint32_t token = 0u;

    uint32_t spin = 2000000u;
    while (!(r32(SMT_CHAN_STATUS) & SMT_FREE) && spin) {
        spin--;
    }
    if (!(r32(SMT_CHAN_STATUS) & SMT_FREE)) {
        return 1;
    }

    uint32_t hdr = (SCMI_MSG_PINCTRL_SET & 0xFFu)
                 | ((SCMI_PROTO_PINCTRL & 0xFFu) << 10)
                 | ((token & 0x3FFu) << 18);
    uint32_t identifier = (muxReg - (IOMUXC_BASE + IOMUXC_MUXREG_OFF)) / 4u;

    uint32_t numConfigs = dsyReg ? 4u : 2u;
    uint32_t attributes = (numConfigs << 2);
    uint32_t muxVal = (muxMode & 0x7u) | ((sion & 0x1u) << 4);

    w32(SMT_A2P_BUF + 0x00u, 0u);
    w32(SMT_CHAN_FLAGS, 0u);
    w32(SMT_MSG_HEADER, hdr);
    w32(SMT_PAYLOAD + 0x00u, identifier);
    w32(SMT_PAYLOAD + 0x04u, 0xFFFFFFFFu);
    w32(SMT_PAYLOAD + 0x08u, attributes);

    uint32_t off = 0x0Cu;
    w32(SMT_PAYLOAD + off + 0x00u, SCMI_PINCTRL_TYPE_MUX);
    w32(SMT_PAYLOAD + off + 0x04u, muxVal);
    off += 0x08u;
    w32(SMT_PAYLOAD + off + 0x00u, SCMI_PINCTRL_TYPE_CONFIG);
    w32(SMT_PAYLOAD + off + 0x04u, padVal);
    off += 0x08u;
    if (dsyReg) {
        uint32_t daisyId = (dsyReg - (IOMUXC_BASE + IOMUXC_DAISYREG_OFF)) / 4u;
        w32(SMT_PAYLOAD + off + 0x00u, SCMI_PINCTRL_TYPE_DAISY_ID);
        w32(SMT_PAYLOAD + off + 0x04u, daisyId);
        off += 0x08u;
        w32(SMT_PAYLOAD + off + 0x00u, SCMI_PINCTRL_TYPE_DAISY_CFG);
        w32(SMT_PAYLOAD + off + 0x04u, dsyVal);
        off += 0x08u;
    }

    uint32_t msgLen = 4u + 12u + numConfigs * 8u;
    w32(SMT_LENGTH, msgLen);
    navq95_smt_crc(msgLen);

    w32(SMT_CHAN_STATUS, r32(SMT_CHAN_STATUS) & ~SMT_FREE);
    w32(MU5_GCR, r32(MU5_GCR) | MU5_GCR_GIR0);

    spin = 2000000u;
    while (!(r32(SMT_CHAN_STATUS) & SMT_FREE) && spin) {
        spin--;
    }
    token++;
    if (!(r32(SMT_CHAN_STATUS) & SMT_FREE)) {
        return 1;
    }
    if (r32(SMT_LENGTH) >= 8u) {
        return (int)r32(SMT_MSG_HEADER + 0x04u);
    }
    return 0;
}

static int navq95_lpuart7_iomux(void)
{
    int rxs = navq95_scmi_pinmux(IOMUXC_MUX_UART7_RX, LPUART7_ALT_MODE, 1u,
                                 IOMUXC_PAD_UART7_RXVAL,
                                 IOMUXC_DSY_UART7_RX, LPUART7_RX_DAISY);
    int txs = navq95_scmi_pinmux(IOMUXC_MUX_UART7_TX, LPUART7_ALT_MODE, 0u,
                                 IOMUXC_PAD_UART7_TXVAL, 0u, 0u);
    return rxs ? rxs : txs;
}

static int navq95_lpuart5_iomux(void)
{
    int rxs = navq95_scmi_pinmux(IOMUXC_MUX_UART5_RX, LPUART5_ALT_MODE, 1u,
                                 IOMUXC_PAD_UART5_RXVAL, IOMUXC_DSY_UART5_RX, LPUART5_DAISY);
    int txs = navq95_scmi_pinmux(IOMUXC_MUX_UART5_TX, LPUART5_ALT_MODE, 0u,
                                 IOMUXC_PAD_UART5_TXVAL, IOMUXC_DSY_UART5_TX, LPUART5_DAISY);
    return rxs ? rxs : txs;
}

static uint32_t navq95_lpuart_baudval(uint32_t clkHz, uint32_t baud)
{
    uint32_t bestOsr = 16u, bestSbr = 1u, bestDiff = baud;

    for (uint32_t osr = 4u; osr <= 32u; osr++) {
        uint32_t sbr = clkHz / (baud * osr);
        if (sbr == 0u) {
            sbr = 1u;
        }
        uint32_t calc = clkHz / (osr * sbr);
        uint32_t diff = (calc > baud) ? (calc - baud) : (baud - calc);
        uint32_t calc2 = clkHz / (osr * (sbr + 1u));
        uint32_t diff2 = (calc2 > baud) ? (calc2 - baud) : (baud - calc2);
        if (diff2 < diff) {
            diff = diff2;
            sbr++;
        }
        if (diff <= bestDiff) {
            bestDiff = diff;
            bestOsr = osr;
            bestSbr = sbr;
        }
    }

    uint32_t val = LPUART_BAUD_OSR(bestOsr) | LPUART_BAUD_SBR(bestSbr);
    if (bestOsr >= 4u && bestOsr <= 7u) {
        val |= LPUART_BAUD_BOTHEDGE;
    }
    return val;
}

static void navq95_lpuart_init(const usartResource_t *reg, uint32_t clkHz, uint32_t baud,
                               bool inverted, bool evenParity2Stop)
{
    volatile uint32_t *global = lpReg(reg, LPUART_GLOBAL_OFFSET);
    volatile uint32_t *fifo   = lpReg(reg, LPUART_FIFO_OFFSET);
    volatile uint32_t *modir  = lpReg(reg, LPUART_MODIR_OFFSET);
    volatile uint32_t *baudr  = lpReg(reg, LPUART_BAUD_OFFSET);
    volatile uint32_t *statr  = lpReg(reg, LPUART_STAT_OFFSET);
    volatile uint32_t *ctrl   = lpReg(reg, LPUART_CTRL_OFFSET);

    *ctrl = 0u;
    *global |= LPUART_GLOBAL_RST;
    for (volatile int d = 0; d < 10000; d++) {
    }
    *global &= ~LPUART_GLOBAL_RST;
    for (volatile int d = 0; d < 10000; d++) {
    }

    *fifo = LPUART_FIFO_RXFE | LPUART_FIFO_TXFE |
            LPUART_FIFO_RXFLUSH | LPUART_FIFO_TXFLUSH;
    *modir = 0u;

    uint32_t bv = navq95_lpuart_baudval(clkHz, baud);
    if (evenParity2Stop) {
        bv |= LPUART_BAUD_SBNS;
    }
    *baudr = bv;

    uint32_t st = *statr | LPUART_STAT_W1C;
    if (inverted) {
        st |= LPUART_STAT_RXINV;
    } else {
        st &= ~LPUART_STAT_RXINV;
    }
    *statr = st;

    uint32_t c = LPUART_CTRL_RE | LPUART_CTRL_TE;
    if (evenParity2Stop) {
        c |= LPUART_CTRL_PE | LPUART_CTRL_M;
    }
    *ctrl = c;
    (void)*ctrl;
}

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART5
    {
        .identifier = SERIAL_PORT_UART5,
        .reg        = (usartResource_t *)LPUART5_BASE,
        // Placeholder tags: the common driver drops a UART without pins. They are never driven.
        .rxPins     = { { DEFIO_TAG_E(PA6) } },
        .txPins     = { { DEFIO_TAG_E(PA7) } },
        .irqn       = LPUART5_IRQn,
        .txBuffer   = uart5TxBuffer,
        .rxBuffer   = uart5RxBuffer,
        .txBufferSize = sizeof(uart5TxBuffer),
        .rxBufferSize = sizeof(uart5RxBuffer),
    },
#endif
#ifdef USE_UART7
    {
        .identifier = SERIAL_PORT_UART7,
        .reg        = (usartResource_t *)LPUART7_BASE,
        .rxPins     = { { DEFIO_TAG_E(PA9) } },
        .txPins     = { { DEFIO_TAG_E(PA8) } },
        .irqn       = LPUART7_IRQn,
        .txBuffer   = uart7TxBuffer,
        .rxBuffer   = uart7RxBuffer,
        .txBufferSize = sizeof(uart7TxBuffer),
        .rxBufferSize = sizeof(uart7RxBuffer),
    },
#endif
};

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
    for (const uartHardware_t *hardware = uartHardware;
         hardware < ARRAYEND(uartHardware); hardware++) {
        const serialPortIdentifier_e identifier = hardware->identifier;
        uartDevice_t *uartdev = uartDeviceFromIdentifier(identifier);
        const int resourceIndex = serialResourceIndex(identifier);
        if (uartdev == NULL || resourceIndex < 0) {
            continue;
        }

        // UART5 pads are muxed over SCMI, so there is nothing to match.
        if (identifier == SERIAL_PORT_UART5) {
            uartdev->hardware = hardware;
            uartdev->rx = hardware->rxPins[0];
            uartdev->tx = hardware->txPins[0];
            continue;
        }

        const ioTag_t cfgRx = pSerialPinConfig->ioTagRx[resourceIndex];
        const ioTag_t cfgTx = pSerialPinConfig->ioTagTx[resourceIndex];
        if (!cfgRx && !cfgTx) {
            continue;
        }

        for (unsigned pindex = 0; pindex < UARTHARDWARE_MAX_PINS; pindex++) {
            if (cfgRx && cfgRx == hardware->rxPins[pindex].pin) {
                uartdev->rx = hardware->rxPins[pindex];
            }
            if (cfgTx && cfgTx == hardware->txPins[pindex].pin) {
                uartdev->tx = hardware->txPins[pindex];
            }
        }

        if (uartdev->rx.pin || uartdev->tx.pin) {
            uartdev->hardware = hardware;
        }
    }
}

uartPort_t *serialUART(uartDevice_t *uartdev, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    UNUSED(baudRate);
    UNUSED(mode);
    UNUSED(options);

    uartPort_t *s = &uartdev->port;
    const uartHardware_t *hardware = uartdev->hardware;
    if (!hardware) {
        return NULL;
    }

    const serialPortIdentifier_e identifier = hardware->identifier;
    const int ownerIndex = serialOwnerIndex(identifier);
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(identifier);

    // The UART5 pin tags are placeholders and must not be claimed as GPIO.
    if (identifier != SERIAL_PORT_UART5) {
        IO_t txIO = IOGetByTag(uartdev->tx.pin);
        IO_t rxIO = IOGetByTag(uartdev->rx.pin);
        if (txIO) {
            IOInit(txIO, ownerTxRx, ownerIndex);
        }
        if (rxIO) {
            IOInit(rxIO, ownerTxRx + 1, ownerIndex);
        }
    }

    if (identifier == SERIAL_PORT_UART5) {
        (void)navq95_scmi_clock_enable(LPUART5_CLK_ROOT);
        (void)navq95_lpuart5_iomux();
    } else {
        (void)navq95_lpuart7_iomux();
    }

    s->port.rxBuffer     = hardware->rxBuffer;
    s->port.txBuffer     = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;
    s->USARTx            = hardware->reg;

    uartdev->txPinState = TX_PIN_IGNORE;

    s->port.vTable = uartVTable;
    return s;
}

void uartReconfigure(uartPort_t *s)
{
    const uartHardware_t *hardware = container_of(s, uartDevice_t, port)->hardware;
    if (!hardware) {
        return;
    }

    const portOptions_e options = s->port.options;
    const bool inverted = (options & SERIAL_INVERTED) != 0;
    // SBUS is 8E2.
    const bool evenParity2Stop = (options & SERIAL_PARITY_EVEN) &&
                                 (options & SERIAL_STOPBITS_2);

    navq95_lpuart_init(s->USARTx, NAVQ95_LPUART_CLK_HZ, s->port.baudRate,
                       inverted, evenParity2Stop);

    volatile uint32_t *ctrl = lpReg(s->USARTx, LPUART_CTRL_OFFSET);
    if (s->port.mode & MODE_RX) {
        *ctrl |= LPUART_CTRL_RIE;
    }
    NVIC_EnableIRQ((IRQn_Type)hardware->irqn);
}

void uartEnableTxInterrupt(uartPort_t *s)
{
    if (s->port.txBufferTail == s->port.txBufferHead) {
        return;
    }
    volatile uint32_t *ctrl = lpReg(s->USARTx, LPUART_CTRL_OFFSET);
    *ctrl |= LPUART_CTRL_TIE;
}

void uartIrqHandler(uartPort_t *s)
{
    volatile uint32_t *stat = lpReg(s->USARTx, LPUART_STAT_OFFSET);
    volatile uint32_t *data = lpReg(s->USARTx, LPUART_DATA_OFFSET);
    volatile uint32_t *ctrl = lpReg(s->USARTx, LPUART_CTRL_OFFSET);

    if (*stat & LPUART_STAT_OR) {
        *stat = LPUART_STAT_OR;
    }

    while (*stat & LPUART_STAT_RDRF) {
        const uint8_t ch = (uint8_t)(*data & LPUART_DATA_MASK);
        if (s->port.rxCallback) {
            s->port.rxCallback(ch, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = ch;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }

    while ((*ctrl & LPUART_CTRL_TIE) && (*stat & LPUART_STAT_TDRE)) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            *data = s->port.txBuffer[s->port.txBufferTail];
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            *ctrl &= ~LPUART_CTRL_TIE;
            break;
        }
    }
}

bool checkUsartTxOutput(uartPort_t *s)
{
    UNUSED(s);
    return true;
}

void uartTxMonitor(uartPort_t *s)
{
    UNUSED(s);
}

void uartConfigureExternalPinInversion(uartPort_t *uartPort)
{
    UNUSED(uartPort);
}

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
    UNUSED(s);
}
#endif

#endif // USE_UART
