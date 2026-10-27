/*
 * serial_uart_navq95.c — i.MX 95 (NavQ95, Cortex-M7) LPUART platform layer for
 * Betaflight's generic serial UART core (drivers/serial_uart.c).
 *
 * The generic core (uartOpen/uartRead/uartWrite/the uartVTable, ring-buffer
 * TX/RX, and the UARTx_IRQHandler wrappers) stays unchanged. This file supplies
 * the i.MX 95 platform half of the serial_uart_impl.h contract:
 *   - uartHardware[] : the per-UART description (register base, IRQ, pins,
 *     buffers). Only LPUART7 is wired on NavQ95.
 *   - uartPinConfigure() : latch the configured RX/TX pins into uartDevice[].
 *   - serialUART() : bind a device to its port, mux the pads over SCMI and run
 *     the local LPUART init.
 *   - uartReconfigure() : (re)program baud / 8N1 and enable RX IRQ.
 *   - uartEnableTxInterrupt() : prime + enable the TDRE (TX) interrupt.
 *   - uartIrqHandler() : drain RX into the port ring, fill TX FIFO from it.
 *   - checkUsartTxOutput()/uartTxMonitor()/uartConfigureExternalPinInversion()/
 *     uartTryStartTxDMA() : required stubs (no SERIAL_CHECK_TX / inverter / DMA).
 *
 * This mirrors the PICO port (serial_uart_pico.c + uart_hw.c).
 *
 * Pads (NavQ95 board):
 *   TX  GPIO_IO08 = PA8 -> LPUART7_TX : mux 0x443C0030 ALT5, pad 0x443C0234
 *   RX  GPIO_IO09 = PA9 -> LPUART7_RX : mux 0x443C0034 ALT5, pad 0x443C0238,
 *                                       SELECT_INPUT (daisy) reg 0x443C0584 = 0x1
 *
 * CRITICAL i.MX 95 fact: the IOMUXC block is owned by the System Manager; direct
 * M7 writes to the pad MUX/CONFIG/SELECT_INPUT registers are silently dropped.
 * We ask the SM to apply the pinmux via an SCMI PINCTRL_CONFIG_SET message
 * (protocol 0x19, msg 0x6) over the MU5 A2P transport. The RX SELECT_INPUT
 * (daisy) is sent as two extra config entries (TYPE_DAISY_ID + TYPE_DAISY_CFG),
 * per NuttX imx9_sm_iomucx_configure() (robotics-0.7.0-navq95).
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

/* ── LPUART7 register block ─────────────────────────────────────────────────
 * i.MX 95 LPUART bases: 1=0x44380000 2=0x44390000 3=0x42570000 4=0x42580000
 *                       5=0x42590000 6=0x425A0000 7=0x42690000 8=0x426A0000
 */
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
#define LPUART_CTRL_TIE       (1u << 23)        /* TDRE (TX data reg empty) IRQ */
#define LPUART_CTRL_RIE       (1u << 21)        /* RDRF (RX data reg full)  IRQ */
#define LPUART_CTRL_RE        (1u << 18)        /* receiver enable              */
#define LPUART_CTRL_TE        (1u << 19)        /* transmitter enable           */
#define LPUART_BAUD_SBR(n)    ((uint32_t)(n) & 0x1FFFu)
#define LPUART_BAUD_BOTHEDGE  (1u << 17)
#define LPUART_BAUD_OSR(n)    (((uint32_t)((n) - 1u) & 0x1Fu) << 24)
#define LPUART_DATA_MASK      0xFFu
/* SBUS framing: 8E2 inverted. PE/PT/M select 8E (parity in the 9th slot),
 * BAUD.SBNS = 2 stop bits, STAT.RXINV inverts the RX line (the SBUS inversion). */
#define LPUART_CTRL_PE        (1u << 1)         /* parity enable                */
#define LPUART_CTRL_PT        (1u << 0)         /* 0 = even parity              */
#define LPUART_CTRL_M         (1u << 4)         /* 9-bit (8 data + parity)      */
#define LPUART_BAUD_SBNS      (1u << 13)        /* 2 stop bits                  */
#define LPUART_STAT_RXINV     (1u << 28)        /* invert RX data               */

/* The LPUART functional clock root is the 24 MHz crystal oscillator; U-Boot/the
 * SM leave LPUART7 on it (no SCMI clock-enable needed). */
#define NAVQ95_LPUART_CLK_HZ  24000000u

static inline volatile uint32_t *lpReg(const usartResource_t *reg, uint32_t off)
{
    return (volatile uint32_t *)((uint32_t)(uintptr_t)reg + off);
}

/* ── IOMUXC pads / SCMI identifiers for LPUART7 ─────────────────────────────*/
#define IOMUXC_BASE           0x443C0000u
#define IOMUXC_MUXREG_OFF     0x0000u   /* SM_PLATFORM_PINCTRL_MUXREG_OFF  */
#define IOMUXC_DAISYREG_OFF   0x0408u   /* SM_PLATFORM_PINCTRL_DAISYREG_OFF*/

/* {muxReg, muxMode, inputReg, inputDaisy, cfgReg} from the i.MX 95 IOMUXC table. */
#define IOMUXC_MUX_UART7_TX   0x443C0030u
#define IOMUXC_MUX_UART7_RX   0x443C0034u
#define IOMUXC_PAD_UART7_TX   0x443C0234u
#define IOMUXC_PAD_UART7_RX   0x443C0238u
#define IOMUXC_DSY_UART7_RX   0x443C0584u   /* LPUART7_RX SELECT_INPUT         */
#define LPUART7_ALT_MODE      0x5u
#define LPUART7_RX_DAISY      0x1u

/* Pad CONFIG values are sent PRE-SHIFTED (>>1); the SM re-applies <<1. Same
 * electrical config the console proved: TX raw 0x1E (FSEL slow | DSE X4),
 * RX raw 0x1200 (PU bit9 | HYS bit12). */
#define IOMUXC_PAD_UART7_TXVAL (0x1Eu >> 1)
#define IOMUXC_PAD_UART7_RXVAL (0x1200u >> 1)

/* ── LPUART5 (RC / SBUS) — DIAGNOSTIC ────────────────────────────────────────
 * UART5 == LPUART5 (base 0x42590000). SHARED GPIO_IO00/IO01 pads at ALT5 that
 * REQUIRE the SELECT_INPUT/daisy register on BOTH TX and RX (these are the
 * HW-proven console pads). The SM does NOT clock LPUART5 at boot, so an SCMI
 * clock-enable (root 103) is issued in serialUART(). SBUS framing (8E2 +
 * inverted RX) is applied in uartReconfigure() from the port options. */
#define LPUART5_BASE          0x42590000u
#define LPUART5_CLK_ROOT      103u
#define IOMUXC_MUX_UART5_TX   0x443C0010u   /* GPIO_IO00, ALT5           */
#define IOMUXC_MUX_UART5_RX   0x443C0014u   /* GPIO_IO01, ALT5 (+SION)   */
#define IOMUXC_PAD_UART5_TX   0x443C0214u
#define IOMUXC_PAD_UART5_RX   0x443C0218u
#define IOMUXC_DSY_UART5_TX   0x443C0574u   /* LPUART5_TX SELECT_INPUT   */
#define IOMUXC_DSY_UART5_RX   0x443C0570u   /* LPUART5_RX SELECT_INPUT   */
#define LPUART5_ALT_MODE      0x5u
#define LPUART5_DAISY         0x1u
/* TX raw 0x1E -> 0x0F; RX raw 0x1200 (PU|HYS) -> 0x0900 (idle-high line reads). */
#define IOMUXC_PAD_UART5_TXVAL (0x1Eu >> 1)
#define IOMUXC_PAD_UART5_RXVAL (0x1200u >> 1)

/* ── SCMI-over-MU5 transport (minimal, polled) — same channel the console uses.
 * The SM owns side-B and leaves the channel FREE at boot. */
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

/* SMT CRC helpers, ported from NuttX imx9_scmi.c. */
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

/*
 * Send one SCMI PINCTRL_CONFIG_SET muxing a single pad. When dsyReg != 0, two
 * extra config entries (TYPE_DAISY_ID + TYPE_DAISY_CFG) carry the SELECT_INPUT
 * (needed for the LPUART7 RX pad). Returns the SCMI status (0 = OK, 1 = timeout).
 */
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
    w32(SMT_PAYLOAD + 0x04u, 0xFFFFFFFFu);    /* function_id = no function */
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

/* Mux both LPUART7 pads via the SM. RX carries the SELECT_INPUT daisy. */
static int navq95_lpuart7_iomux(void)
{
    int rxs = navq95_scmi_pinmux(IOMUXC_MUX_UART7_RX, LPUART7_ALT_MODE, 1u,
                                 IOMUXC_PAD_UART7_RXVAL,
                                 IOMUXC_DSY_UART7_RX, LPUART7_RX_DAISY);
    int txs = navq95_scmi_pinmux(IOMUXC_MUX_UART7_TX, LPUART7_ALT_MODE, 0u,
                                 IOMUXC_PAD_UART7_TXVAL, 0u, 0u);
    return rxs ? rxs : txs;
}

/* Mux both LPUART5 pads via the SM. SHARED GPIO_IO00/IO01 at ALT5 — BOTH carry a
 * SELECT_INPUT/daisy (dsyReg != 0). RX pad gets SION + PU|HYS so the idle-high
 * line reads. */
static int navq95_lpuart5_iomux(void)
{
    int rxs = navq95_scmi_pinmux(IOMUXC_MUX_UART5_RX, LPUART5_ALT_MODE, 1u,
                                 IOMUXC_PAD_UART5_RXVAL, IOMUXC_DSY_UART5_RX, LPUART5_DAISY);
    int txs = navq95_scmi_pinmux(IOMUXC_MUX_UART5_TX, LPUART5_ALT_MODE, 0u,
                                 IOMUXC_PAD_UART5_TXVAL, IOMUXC_DSY_UART5_TX, LPUART5_DAISY);
    return rxs ? rxs : txs;
}

/* BAUD value search (NuttX imx9_lpuart_configure): OSR 4..32, smallest error. */
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

/* Full LPUART init at a known functional clock (NuttX imx9_lpuart_configure):
 * software reset, flush + enable RX/TX FIFOs, clear MODIR, program BAUD, clear
 * stale W1C flags, enable RE|TE. `inverted` sets STAT.RXINV; `evenParity2Stop`
 * selects 8E2 framing (SBUS). Default (false/false) is 8N1. TX/RX interrupts
 * are left off here; the generic core enables them via uartReconfigure/
 * uartEnableTxInterrupt. */
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
        bv |= LPUART_BAUD_SBNS;                 /* 2 stop bits */
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
        c |= LPUART_CTRL_PE | LPUART_CTRL_M;    /* 8E: parity enable + 9-bit; PT=0 even */
    }
    *ctrl = c;
    (void)*ctrl;
}

/* ════════════════════════════════════════════════════════════════════════════
 * uartHardware[] — the per-UART description table the generic core indexes by
 * uartDeviceIdx_e. NavQ95 wires LPUART2 (UARTDEV_2, RC/SBUS) and LPUART7
 * (UARTDEV_7, MSP). Rows are matched by .identifier (order is not significant).
 * ════════════════════════════════════════════════════════════════════════════ */
const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART5
    {
        .identifier = SERIAL_PORT_UART5,
        .reg        = (usartResource_t *)LPUART5_BASE,
        /* Shared GPIO_IO00/IO01 pads (ALT5 + daisy). The REAL pads are muxed over
         * SCMI in serialUART() (navq95_lpuart5_iomux); they have NO RGPIO IO-HAL
         * tag. BUT the generic core's uartSanitizeMode() strips MODE_RX/MODE_TX
         * when rx.pin/tx.pin are 0, disabling the SBUS receiver. So we give this
         * row NON-NONE PLACEHOLDER RGPIO tags (PA6/PA7, otherwise unused) so
         * rx.pin/tx.pin are non-zero and the mode survives. serialUART() SKIPS
         * IOInit for UART5, so these placeholder RGPIO pins are never driven —
         * only the LPUART5 pads over SCMI. */
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
        .rxPins     = { { DEFIO_TAG_E(PA9) } },   /* GPIO_IO09 -> LPUART7_RX */
        .txPins     = { { DEFIO_TAG_E(PA8) } },   /* GPIO_IO08 -> LPUART7_TX */
        .irqn       = LPUART7_IRQn,
        .txBuffer   = uart7TxBuffer,
        .rxBuffer   = uart7RxBuffer,
        .txBufferSize = sizeof(uart7TxBuffer),
        .rxBufferSize = sizeof(uart7RxBuffer),
    },
#endif
};

/* ════════════════════════════════════════════════════════════════════════════
 * Platform hooks required by drivers/serial_uart.c and serial_uart_impl.h.
 * ════════════════════════════════════════════════════════════════════════════ */

/*
 * uartPinConfigure() — fill uartDevice[].rx/.tx/.hardware from the configured
 * serialPinConfig (which drivers/serial_uart_pinconfig.c populated from the
 * UART7_RX/TX_PIN tags). Same structure as the PICO uartPinConfigure_hw().
 */
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

        /* SCMI-muxed UART (UART5 == LPUART5 on the shared GPIO_IO00/IO01 pads):
         * the real pads are muxed over SCMI in serialUART(), so there is nothing
         * to match against the serialPinConfig. Bind ->hardware AND copy the
         * (placeholder PA6/PA7) pin records into rx/tx so uartSanitizeMode() in
         * the generic core keeps MODE_RX/MODE_TX (it strips them when rx.pin ==
         * 0). serialUART() skips IOInit for UART5 so these RGPIO pins are never
         * driven. Identify the SCMI-muxed port by its identifier. */
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

/*
 * serialUART() — bind the device to its port, claim the IO pins, mux the pads
 * over SCMI and bring the LPUART up. Buffers / register handle are latched into
 * the port; the generic uartOpen() then calls uartReconfigure() to program the
 * baud and enable RX. Mirrors PICO serialUART_hw() + serialUART().
 */
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
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(identifier);   /* rx = +1 */

    /* UART7 pins are RGPIO IO tags (claim them via the IO HAL); UART5 uses the
     * shared GPIO_IO00/IO01 pads with NO IO-HAL record. UART5's rx/tx carry
     * PLACEHOLDER tags (PA6/PA7) only so the generic core keeps MODE_RX — they
     * must NOT be driven as GPIO, so skip IOInit for UART5 (its real pads are
     * muxed over SCMI below). */
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

    /* Pad mux + clock over SCMI, per UART:
     *   UART7 : shared GPIO_IO08/09 pads, ALT5 + RX daisy (SM clocks it at boot).
     *   UART5 : shared GPIO_IO00/IO01 pads, ALT5 + daisy on both; the SM does
     *           NOT clock LPUART5 at boot, so enable its root (103) over SCMI. */
    if (identifier == SERIAL_PORT_UART5) {
        extern int navq95_scmi_clock_enable_ex(uint32_t clockRootId);
        (void)navq95_scmi_clock_enable_ex(LPUART5_CLK_ROOT);
        (void)navq95_lpuart5_iomux();
    } else {
        (void)navq95_lpuart7_iomux();
    }

    s->port.rxBuffer     = hardware->rxBuffer;
    s->port.txBuffer     = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;
    s->USARTx            = hardware->reg;

    uartdev->txPinState = TX_PIN_IGNORE;   /* no SERIAL_CHECK_TX on NavQ95 */

    s->port.vTable = uartVTable;
    return s;
}

/*
 * uartReconfigure() — (re)program the LPUART from the current port baud/mode/
 * options. Called by the generic uartOpen() and by uartSetBaudRate()/
 * uartSetMode(). Framing is derived from the port options so this one path
 * serves both the 8N1 MSP port (UART7) and the 8E2-inverted SBUS port (UART2):
 *   SERIAL_INVERTED    -> STAT.RXINV (invert the RX line)
 *   SERIAL_PARITY_EVEN -> 8E (CTRL.PE | CTRL.M, PT=0 even)
 *   SERIAL_STOPBITS_2  -> BAUD.SBNS (2 stop bits)
 * Enable the RX interrupt when the port is in RX mode; the TX interrupt is
 * primed separately by uartEnableTxInterrupt().
 */
void uartReconfigure(uartPort_t *s)
{
    const uartHardware_t *hardware = container_of(s, uartDevice_t, port)->hardware;
    if (!hardware) {
        return;
    }

    const portOptions_e options = s->port.options;
    const bool inverted = (options & SERIAL_INVERTED) != 0;
    /* SBUS = 8E2: Betaflight passes SERIAL_PARITY_EVEN | SERIAL_STOPBITS_2. */
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

/*
 * uartEnableTxInterrupt() — if there is data queued, enable the TDRE interrupt
 * so the IRQ drains the port TX ring into the LPUART. The generic uartWrite()
 * calls this after queuing a byte.
 */
void uartEnableTxInterrupt(uartPort_t *s)
{
    if (s->port.txBufferTail == s->port.txBufferHead) {
        return;
    }
    volatile uint32_t *ctrl = lpReg(s->USARTx, LPUART_CTRL_OFFSET);
    *ctrl |= LPUART_CTRL_TIE;
}

/*
 * uartIrqHandler() — called from the generic UART7_IRQHandler wrapper (in
 * drivers/serial_uart.c). Drain RX into the port ring (or the rxCallback), and
 * fill the TX FIFO from the port TX ring, masking TDRE when the ring drains.
 */
void uartIrqHandler(uartPort_t *s)
{
    volatile uint32_t *stat = lpReg(s->USARTx, LPUART_STAT_OFFSET);
    volatile uint32_t *data = lpReg(s->USARTx, LPUART_DATA_OFFSET);
    volatile uint32_t *ctrl = lpReg(s->USARTx, LPUART_CTRL_OFFSET);

    if (*stat & LPUART_STAT_OR) {
        *stat = LPUART_STAT_OR;             /* clear overrun */
    }

    /* RX: drain while data is available. */
    while (*stat & LPUART_STAT_RDRF) {
        const uint8_t ch = (uint8_t)(*data & LPUART_DATA_MASK);   /* read clears RDRF */
        if (s->port.rxCallback) {
            s->port.rxCallback(ch, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = ch;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }

    /* TX: while the TX interrupt is enabled and the data register is empty,
     * push the next queued byte; mask the interrupt once the ring is empty. */
    while ((*ctrl & LPUART_CTRL_TIE) && (*stat & LPUART_STAT_TDRE)) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            *data = s->port.txBuffer[s->port.txBufferTail];
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            *ctrl &= ~LPUART_CTRL_TIE;      /* nothing left to send */
            break;
        }
    }
}

/* ── Required stubs (no SERIAL_CHECK_TX / external inverter / DMA on NavQ95) ──*/

bool checkUsartTxOutput(uartPort_t *s)
{
    UNUSED(s);
    return true;                            /* TX always allowed */
}

void uartTxMonitor(uartPort_t *s)
{
    UNUSED(s);
}

void uartConfigureExternalPinInversion(uartPort_t *uartPort)
{
    UNUSED(uartPort);                       /* no external inverter wired */
}

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
    UNUSED(s);                              /* interrupt-driven, no DMA */
}
#endif

#endif /* USE_UART */
