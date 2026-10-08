/*
 * platform_stubs.c — platform layer for the NavQ95 (NXP i.MX 95, Cortex-M7).
 *
 * Early bring-up port: provides the FPU/timebase/console the common Betaflight
 * code requires, plus stub bodies for peripheral drivers not yet implemented
 * (SPI/I2C/DMA/timer/flash/sdcard/...). Real drivers replace the stubs
 * incrementally. The console is a minimal LPUART-backed serialPort_t exposed as
 * the VCP port so the CLI runs over it; a dedicated UART driver will replace it.
 */

#include "platform.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/exti.h"
#include "drivers/timer.h"
#include "drivers/motor.h"
#include "drivers/pwm_output.h"
#include "drivers/servo_impl.h"
#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/dma.h"
#include "drivers/transponder_ir.h"
#include "drivers/system.h"
#include "drivers/adc.h"
#include "common/color.h"
#include "drivers/light_ws2811strip.h"
#include "drivers/dshot_bitbang.h"
#include "drivers/dshot.h"
#include "drivers/rx/expresslrs_driver.h"
#include "drivers/nvic.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/persistent.h"
#include "drivers/flash/flash.h"
#include "drivers/sdcard.h"
#include "io/flashfs.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/asyncfatfs/fat_standard.h"
#include "build/debug.h"

/*
 * Console LPUART (LPUART2). systemInit() muxes the pads (over SCMI) and does a
 * local reset + BAUD setup. Register offsets per the i.MX 95 LPUART block;
 * bases for LPUART1..8:
 *   1=0x44380000 2=0x44390000 3=0x42570000 4=0x42580000
 *   5=0x42590000 6=0x425A0000 7=0x42690000 8=0x426A0000
 */
#ifndef NAVQ95_CONSOLE_LPUART_BASE
#define NAVQ95_CONSOLE_LPUART_BASE 0x44390000u   /* LPUART2 */
#endif

#define LPUART_GLOBAL_OFFSET 0x08u
#define LPUART_BAUD_OFFSET   0x10u
#define LPUART_FIFO_OFFSET   0x28u
#define LPUART_STAT_OFFSET 0x14u
#define LPUART_CTRL_OFFSET 0x18u
#define LPUART_DATA_OFFSET 0x1Cu
#define LPUART_MODIR_OFFSET  0x24u
#define LPUART_GLOBAL_RST  (1u << 1)             /* software reset              */
#define LPUART_FIFO_RXFE    (1u << 3)            /* RX FIFO enable              */
#define LPUART_FIFO_TXFE    (1u << 7)            /* TX FIFO enable              */
#define LPUART_FIFO_RXFLUSH (1u << 14)           /* flush RX FIFO               */
#define LPUART_FIFO_TXFLUSH (1u << 15)           /* flush TX FIFO               */
#define LPUART_STAT_TDRE   (1u << 23)            /* TX data register empty      */
#define LPUART_STAT_RDRF   (1u << 21)            /* RX data register full       */
#define LPUART_STAT_IDLE   (1u << 20)            /* idle line (write-1-clear)   */
#define LPUART_STAT_OR     (1u << 19)            /* RX overrun (write-1-clear)  */
#define LPUART_STAT_NF     (1u << 18)            /* noise flag (write-1-clear)  */
#define LPUART_STAT_FE     (1u << 17)            /* framing err (write-1-clear) */
#define LPUART_STAT_PF     (1u << 16)            /* parity err (write-1-clear)  */
/* All write-1-to-clear RX error/status flags, cleared before enabling RX so a
 * stale OR/FE/NF/PF cannot block RDRF. */
#define LPUART_STAT_W1C    (LPUART_STAT_IDLE | LPUART_STAT_OR | LPUART_STAT_NF | \
                            LPUART_STAT_FE | LPUART_STAT_PF)
#define LPUART_CTRL_RE     (1u << 18)            /* receiver enable             */
#define LPUART_CTRL_TE     (1u << 19)            /* transmitter enable          */
#define LPUART_CTRL_RIE    (1u << 21)            /* RX interrupt enable         */
#define LPUART_BAUD_SBR(n)      ((uint32_t)(n) & 0x1FFFu)       /* SBR  [12:0]  */
#define LPUART_BAUD_BOTHEDGE    (1u << 17)       /* both-edge sampling (low OSR)*/
#define LPUART_BAUD_OSR(n)      (((uint32_t)((n) - 1u) & 0x1Fu) << 24)  /* OSR  */
#define LPUART_DATA_MASK   0xFFu                 /* received byte in DATA[7:0]  */

/* Known-good console functional clock. The i.MX 95 LPUART root is the 24 MHz
 * crystal oscillator (lpuart_clk_root); the SM/U-Boot leave LPUART2 on it.
 * Proven: the SM's BAUD 0x19000008 (OSR 26 / SBR 8) prints 115200 cleanly only
 * if the clock is 115200*26*8 = 23.96 MHz. We compute our own divisor from this
 * rather than sweeping candidate clocks. */
#define NAVQ95_LPUART_CLK_HZ   24000000u
#define NAVQ95_CONSOLE_BAUD    115200u

static inline void navq95_lowputc(uint8_t ch)
{
    volatile uint32_t *stat =
        (volatile uint32_t *)(NAVQ95_CONSOLE_LPUART_BASE + LPUART_STAT_OFFSET);
    volatile uint32_t *data =
        (volatile uint32_t *)(NAVQ95_CONSOLE_LPUART_BASE + LPUART_DATA_OFFSET);

    while (!(*stat & LPUART_STAT_TDRE)) {
    }
    *data = ch;
}

static void navq95_lowputs(const char *s)
{
    while (*s) {
        if (*s == '\n') {
            navq95_lowputc('\r');
        }
        navq95_lowputc((uint8_t)*s++);
    }
}

/*
 * Interrupt-driven console RX: the LPUART IRQ fills a ring buffer that the
 * serialPort_t read path drains. LPUART2_IRQHandler sits at external IRQ 20
 * (vector index 36) in startup_navq95.s.
 */
#define NAVQ95_RX_RING_SZ 256u                 /* power of two */
static volatile uint8_t  s_rxRing[NAVQ95_RX_RING_SZ];
static volatile uint32_t s_rxHead;             /* written by IRQ  */
static volatile uint32_t s_rxTail;             /* read by thread  */

void LPUART2_IRQHandler(void)
{
    volatile uint32_t *stat =
        (volatile uint32_t *)(NAVQ95_CONSOLE_LPUART_BASE + LPUART_STAT_OFFSET);
    volatile uint32_t *data =
        (volatile uint32_t *)(NAVQ95_CONSOLE_LPUART_BASE + LPUART_DATA_OFFSET);

    uint32_t s = *stat;
    if (s & LPUART_STAT_OR) {
        *stat = LPUART_STAT_OR;                /* clear overrun */
    }
    while (*stat & LPUART_STAT_RDRF) {
        uint8_t b = (uint8_t)(*data & LPUART_DATA_MASK);   /* read clears RDRF */
        uint32_t next = (s_rxHead + 1u) & (NAVQ95_RX_RING_SZ - 1u);
        if (next != s_rxTail) {                /* drop on overflow */
            s_rxRing[s_rxHead] = b;
            s_rxHead = next;
        }
    }
}

static inline int navq95_rx_pop(void)
{
    if (s_rxTail == s_rxHead) {
        return -1;
    }
    uint8_t b = s_rxRing[s_rxTail];
    s_rxTail = (s_rxTail + 1u) & (NAVQ95_RX_RING_SZ - 1u);
    return (int)b;
}

/*
 * Polled RX drain: move any bytes currently in the LPUART RX FIFO into the ring.
 * The console read path calls this so RX does not depend on interrupt latency;
 * the IRQ handler feeds the same ring. The LPUART IRQ is masked around the drain
 * so the two producers never race on s_rxHead.
 */
static void navq95_rx_poll_drain(void)
{
    volatile uint32_t *stat =
        (volatile uint32_t *)(NAVQ95_CONSOLE_LPUART_BASE + LPUART_STAT_OFFSET);
    volatile uint32_t *data =
        (volatile uint32_t *)(NAVQ95_CONSOLE_LPUART_BASE + LPUART_DATA_OFFSET);

    NVIC_DisableIRQ(LPUART2_IRQn);
    if (*stat & LPUART_STAT_OR) {
        *stat = LPUART_STAT_OR;                /* clear overrun */
    }
    while (*stat & LPUART_STAT_RDRF) {
        uint8_t b = (uint8_t)(*data & LPUART_DATA_MASK);   /* read clears RDRF */
        uint32_t next = (s_rxHead + 1u) & (NAVQ95_RX_RING_SZ - 1u);
        if (next != s_rxTail) {
            s_rxRing[s_rxHead] = b;
            s_rxHead = next;
        }
    }
    NVIC_EnableIRQ(LPUART2_IRQn);
}

/* Enable the LPUART RX interrupt: set CTRL.RIE and enable the NVIC line. The
 * IRQ handler and the polled drain (navq95_rx_poll_drain) feed the same ring;
 * the console read path polls, so RX works regardless of interrupt latency. */
static void navq95_uart_rx_irq_enable(void)
{
    volatile uint32_t *ctrl =
        (volatile uint32_t *)(NAVQ95_CONSOLE_LPUART_BASE + LPUART_CTRL_OFFSET);
    *ctrl |= LPUART_CTRL_RIE;
    NVIC_EnableIRQ(LPUART2_IRQn);
}

/*
 * LPUART2 console pad IOMUX — via SCMI to the System Manager.
 *
 * CRITICAL i.MX 95 fact: the IOMUXC block (0x443C0000) is owned by the System
 * Manager (the M33 security domain). DIRECT M7 writes to the pad MUX/CONFIG
 * registers are SILENTLY DROPPED (same symptom as the RGPIO2 pad-latch issue).
 * The M7 must ask the SM to apply the pinmux by sending an SCMI
 * PINCTRL_CONFIG_SET message (protocol 0x19, msg 0x6) over the MU5 shared-memory
 * transport. This mirrors NuttX's CONFIG_IMX9_IOMUX_OVER_SCMI path
 * (arch/arm/src/imx9/imx9_iomuxc.c + imx9_scmi.c @ robotics-0.7.0-navq95); the
 * byte layout below is reproduced from that source.
 *
 * Pads (authoritative, from the NavQ95 board support):
 *   TX pad UART2_TXD : MUX 0x443C01DC (ALT0), PAD 0x443C03E0 val 0x1E
 *   RX pad UART2_RXD : MUX 0x443C01D8 (ALT0 + SION), PAD 0x443C03DC val 0x00
 * RX uses the dedicated UART2_RXD pad at ALT0 -> no SELECT_INPUT/daisy needed;
 * SION (mux bit4) forces the RX input path on.
 */
#define IOMUXC_BASE            0x443C0000u
#define IOMUXC_MUX_UART2_TX    (IOMUXC_BASE + 0x01DCu)
#define IOMUXC_MUX_UART2_RX    (IOMUXC_BASE + 0x01D8u)
#define IOMUXC_PAD_UART2_TX    (IOMUXC_BASE + 0x03E0u)
#define IOMUXC_PAD_UART2_RX    (IOMUXC_BASE + 0x03DCu)
#define IOMUXC_MUX_SION        (1u << 4)        /* software input-on (force RX)  */
/*
 * Pad CONFIG (SW_PAD_CTL) values for the SCMI TYPE_CONFIG entry. CRITICAL: the
 * value sent over SCMI is PRE-SHIFTED — the SM applies a <<1 (IOMUXC_PAD_
 * CONFIG_SHIFT=1) before writing the register (NuttX IOMUX_CFG stores pad>>1;
 * the SCMI path forwards that stored value and the SM re-shifts). So we send
 * raw_register_value >> 1.
 *   TX: raw 0x1E (FSEL slow | DSE X4) -> send 0x0F.
 *   RX: raw 0x1200 (PU bit9 | HYS bit12, DSE off) -> send 0x0900. An input pad
 *       needs hysteresis + a pull-up so the idle-high UART line reads cleanly
 *       and edges are seen; DSE=0 leaves the output driver off. (Sending 0x00
 *       left the RX input floating -> no RX despite a successful mux.)
 */
#define IOMUXC_PAD_LPUART_TXVAL 0x0Fu           /* -> SM writes raw 0x1E         */
#define IOMUXC_PAD_LPUART_RXVAL 0x0900u         /* -> SM writes raw 0x1200 PU|HYS*/

/* ── SCMI-over-MU5 transport (minimal, polled, no NuttX framework) ──────────
 * MU5 side-A base + the A2P shared-memory transport (SMT) buffer. The SM owns
 * side-B and leaves the channel FREE at boot, so no hardware init is needed —
 * we only drive the SMT handshake + the GCR doorbell. */
#define MU5_BASE               0x44610000u
#define MU5_GCR                (MU5_BASE + 0x0114u)    /* general ctrl: GIR0 doorbell */
#define MU5_GCR_GIR0           (1u << 0)
#define SMT_A2P_BUF            0x44611000u             /* MU5 SRAM + 0x1000, channel 0 */
/* SMT header (24 bytes) then the SCMI message header + payload. */
#define SMT_CHAN_STATUS        (SMT_A2P_BUF + 0x04u)   /* bit0 FREE, bit1 ERROR */
#define SMT_CHAN_FLAGS         (SMT_A2P_BUF + 0x10u)   /* bit0 = request completion IRQ */
#define SMT_LENGTH             (SMT_A2P_BUF + 0x14u)   /* msg-header word + payload bytes */
#define SMT_MSG_HEADER         (SMT_A2P_BUF + 0x18u)   /* SCMI header word */
#define SMT_PAYLOAD            (SMT_A2P_BUF + 0x1Cu)   /* first payload word */
#define SMT_FREE               (1u << 0)
#define SMT_IMP_STATUS         (SMT_A2P_BUF + 0x08u)   /* SM-owned: CRC type selector */
#define SMT_IMP_CRC            (SMT_A2P_BUF + 0x0Cu)   /* CRC value (we compute+write) */
/* SMT CRC modes (impStatus). The SM leaves this per-channel; we READ it and
 * compute the matching CRC over the message (header + payload). A missing CRC
 * when the SM expects one is reported as SM_ERR_CRC_ERROR (-129 / 0xFFFFFF7F). */
#define SMT_CRC_NONE           0u
#define SMT_CRC_XOR            1u
#define SMT_CRC_CRC32          3u
/* SCMI header: msg[7:0] | type[9:8] | protocol[17:10] | token[27:18]. */
#define SCMI_PROTO_PINCTRL     0x19u
#define SCMI_MSG_PINCTRL_SET   0x06u
/* PINCTRL config entry types + attributes. */
#define SCMI_PINCTRL_TYPE_MUX    192u
#define SCMI_PINCTRL_TYPE_CONFIG 193u

static inline void navq95_w32(uint32_t addr, uint32_t v)
{
    *(volatile uint32_t *)addr = v;
}
static inline uint32_t navq95_r32(uint32_t addr)
{
    return *(volatile uint32_t *)addr;
}

/*
 * SMT CRC helpers, ported verbatim from NuttX imx9_scmi.c (imx9_crcxor /
 * imx9_crc32). The SM may enable a CRC on the channel; we must compute the same
 * value over the message (from the SCMI header word for `length` bytes) or the
 * SM rejects with SM_ERR_CRC_ERROR. imx9_crc32 is a NON-standard reflected
 * CRC-32: init 0, nibble table (reversed poly 0xEDB88320), NO final xor — do
 * NOT substitute a stock zlib crc32.
 */
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

/* Compute the SMT CRC for a just-built message of `len` bytes, per the SM's
 * channel CRC mode (impStatus), and write it to imp_crc. CRC covers the message
 * starting at the SCMI header word (SMT +0x18). No-op for SMT_CRC_NONE. */
static void navq95_smt_crc(uint32_t len)
{
    uint32_t impStatus = navq95_r32(SMT_IMP_STATUS);
    if (impStatus == SMT_CRC_XOR) {
        navq95_w32(SMT_IMP_CRC,
                   navq95_crcxor((const uint32_t *)SMT_MSG_HEADER, len / 4u));
    } else if (impStatus == SMT_CRC_CRC32) {
        navq95_w32(SMT_IMP_CRC,
                   navq95_crc32((const uint8_t *)SMT_MSG_HEADER, len));
    }
}

/*
 * Send one SCMI PINCTRL_CONFIG_SET to the SM, muxing a single pad: set its MUX
 * (mode + optional SION) and CONFIG (pad electrical) value. Blocks (bounded)
 * until the SM frees the channel, then returns the SCMI status (0 = success,
 * negative = SCMI error, 1 = timeout waiting for the channel).
 */
static int navq95_scmi_pinmux(uint32_t muxReg, uint32_t muxMode, uint32_t sion,
                              uint32_t padReg, uint32_t padVal)
{
    static uint32_t token = 0u;

    (void)padReg;   /* identifier is derived from muxReg; padReg kept for clarity */

    /* 1. Wait for the SM to mark the channel FREE. */
    uint32_t spin = 2000000u;
    while (!(navq95_r32(SMT_CHAN_STATUS) & SMT_FREE) && spin) {
        spin--;
    }
    if (!(navq95_r32(SMT_CHAN_STATUS) & SMT_FREE)) {
        return 1;   /* channel never became free */
    }

    /* 2. Build the message. header = msg | (proto<<10) | (type0) | (token<<18) */
    uint32_t hdr = (SCMI_MSG_PINCTRL_SET & 0xFFu)
                 | ((SCMI_PROTO_PINCTRL & 0xFFu) << 10)
                 | ((token & 0x3FFu) << 18);
    uint32_t identifier = (muxReg - IOMUXC_BASE) / 4u;   /* pin selector */
    uint32_t attributes = (2u << 2);                      /* SEL_PIN, num_configs=2 */
    uint32_t muxVal = (muxMode & 0x7u) | ((sion & 0x1u) << 4);

    navq95_w32(SMT_A2P_BUF + 0x00u, 0u);          /* resv */
    navq95_w32(SMT_CHAN_FLAGS, 0u);               /* poll, no completion IRQ */
    navq95_w32(SMT_MSG_HEADER, hdr);
    navq95_w32(SMT_PAYLOAD + 0x00u, identifier);
    navq95_w32(SMT_PAYLOAD + 0x04u, 0xFFFFFFFFu); /* function_id = no function */
    navq95_w32(SMT_PAYLOAD + 0x08u, attributes);
    navq95_w32(SMT_PAYLOAD + 0x0Cu, SCMI_PINCTRL_TYPE_MUX);
    navq95_w32(SMT_PAYLOAD + 0x10u, muxVal);
    navq95_w32(SMT_PAYLOAD + 0x14u, SCMI_PINCTRL_TYPE_CONFIG);
    navq95_w32(SMT_PAYLOAD + 0x18u, padVal);
    /* length = SCMI header word (4) + identifier+function_id+attributes (12)
     *          + 2 config entries * 8 = 32 bytes. */
    uint32_t msgLen = 4u + 12u + 2u * 8u;
    navq95_w32(SMT_LENGTH, msgLen);

    /* Compute the channel CRC (if the SM enabled one) over the message. Must be
     * done AFTER length is final and BEFORE handing the channel to the SM. */
    navq95_smt_crc(msgLen);

    /* 3. Hand the channel to the SM (clear FREE) and ring the doorbell. */
    navq95_w32(SMT_CHAN_STATUS, navq95_r32(SMT_CHAN_STATUS) & ~SMT_FREE);
    navq95_w32(MU5_GCR, navq95_r32(MU5_GCR) | MU5_GCR_GIR0);

    /* 4. Wait for the response (SM sets FREE again), then read status. */
    spin = 2000000u;
    while (!(navq95_r32(SMT_CHAN_STATUS) & SMT_FREE) && spin) {
        spin--;
    }
    token++;
    if (!(navq95_r32(SMT_CHAN_STATUS) & SMT_FREE)) {
        return 1;   /* no response */
    }
    if (navq95_r32(SMT_LENGTH) >= 8u) {
        return (int)navq95_r32(SMT_MSG_HEADER + 0x04u);   /* status word */
    }
    return 0;
}

/* Mux both LPUART2 console pads (TX + RX) via the SM over SCMI. Returns 0 if the
 * SM accepted both, else the first non-zero SCMI status. */
static int navq95_lpuart2_iomux(void)
{
    int rxs = navq95_scmi_pinmux(IOMUXC_MUX_UART2_RX, 0u, 1u,
                                 IOMUXC_PAD_UART2_RX, IOMUXC_PAD_LPUART_RXVAL);
    int txs = navq95_scmi_pinmux(IOMUXC_MUX_UART2_TX, 0u, 0u,
                                 IOMUXC_PAD_UART2_TX, IOMUXC_PAD_LPUART_TXVAL);
    return rxs ? rxs : txs;
}

/*
 * Compute a BAUD register value for a target baud from a known functional
 * clock, mirroring NuttX imx9_lpuart_configure(): search OSR 4..32 for the
 * divisor pair (OSR, SBR) with the smallest baud error, and set BOTHEDGE when
 * OSR is 4..7 (required for low-OSR sampling). Returns the full BAUD value
 * (OSR-1 in [29:24], SBR in [12:0], 1 stop bit). clkHz/baud are both nonzero.
 */
static uint32_t navq95_lpuart_baudval(uint32_t clkHz, uint32_t baud)
{
    uint32_t bestOsr = 16u;
    uint32_t bestSbr = 1u;
    uint32_t bestDiff = baud;                   /* worst case = full baud */

    for (uint32_t osr = 4u; osr <= 32u; osr++) {
        uint32_t sbr = clkHz / (baud * osr);
        if (sbr == 0u) {
            sbr = 1u;
        }
        uint32_t calc = clkHz / (osr * sbr);
        uint32_t diff = (calc > baud) ? (calc - baud) : (baud - calc);

        /* Also try sbr+1 — the rounded-up divisor is often closer. */
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
    return val;                                 /* SBNS left 0 => 1 stop bit */
}

/*
 * Full LPUART init at a known functional clock, ported from NuttX
 * imx9_lpuart_configure() (Apache-2.0): software reset (GLOBAL.RST), flush +
 * enable the RX/TX FIFOs, clear MODIR (no flow control), program BAUD from the
 * functional clock, and CTRL = RE|TE for 8N1 (M=0). Leaves TX/RX enabled.
 */
static void navq95_lpuart_init(uint32_t base, uint32_t clkHz, uint32_t baud)
{
    volatile uint32_t *global = (volatile uint32_t *)(base + LPUART_GLOBAL_OFFSET);
    volatile uint32_t *fifo   = (volatile uint32_t *)(base + LPUART_FIFO_OFFSET);
    volatile uint32_t *modir  = (volatile uint32_t *)(base + LPUART_MODIR_OFFSET);
    volatile uint32_t *baudr  = (volatile uint32_t *)(base + LPUART_BAUD_OFFSET);
    volatile uint32_t *statr  = (volatile uint32_t *)(base + LPUART_STAT_OFFSET);
    volatile uint32_t *ctrl   = (volatile uint32_t *)(base + LPUART_CTRL_OFFSET);

    /* Software reset: clears all internal logic/registers except GLOBAL. */
    *ctrl = 0u;                                 /* TX/RX off while resetting */
    *global |= LPUART_GLOBAL_RST;
    for (volatile int d = 0; d < 10000; d++) {
    }
    *global &= ~LPUART_GLOBAL_RST;
    for (volatile int d = 0; d < 10000; d++) {
    }

    /* Flush then enable the RX/TX FIFOs. */
    *fifo = LPUART_FIFO_RXFE | LPUART_FIFO_TXFE |
            LPUART_FIFO_RXFLUSH | LPUART_FIFO_TXFLUSH;

    *modir = 0u;                                /* no RTS/CTS flow control */
    *baudr = navq95_lpuart_baudval(clkHz, baud);

    /* Clear any stale W1C error/status flags so a leftover OR/FE/NF/PF cannot
     * block RDRF once the receiver is enabled. */
    *statr = LPUART_STAT_W1C;

    *ctrl = LPUART_CTRL_RE | LPUART_CTRL_TE;    /* 8N1, M=0 */
    (void)*ctrl;
}

/* CMSIS SystemInit — runs from Reset_Handler before main(). */
void SystemInit(void)
{
    /*
     * Enable the FPU. Betaflight is built hard-float (fpv5-d16); the M7 powers
     * up with CP10/CP11 disabled in CPACR, so the first VFP instruction would
     * raise a UsageFault (UNDEFINSTR). Grant full access, then barrier.
     */
    SCB->CPACR |= (3u << (10 * 2)) | (3u << (11 * 2));   /* CP10 + CP11 */
    __DSB();
    __ISB();

    /* Clock/MPU/cache init will be added with the real device drivers. */
}

void SystemCoreClockUpdate(void)
{
}

/*
 * Core clock. Placeholder value for the micros()/millis() scaling below; the
 * M7 clock is programmed by the System Manager/U-Boot. The DWT-based time is
 * monotonic regardless, and this value matches the measured 1 Hz cadence.
 */
uint32_t SystemCoreClock = 800000000U;

void systemInit(void)
{
    cycleCounterInit();                        /* DWT microsecond time base */
    navq95_timebaseInit();                     /* 1 kHz SysTick folds CYCCNT wraps */

    /*
     * Console (LPUART2) init. On i.MX 95 the M7 cannot write IOMUXC/CCM
     * directly — those blocks are owned by the System Manager (SM). We therefore
     * mux the TX/RX pads over SCMI (navq95_lpuart2_iomux), then do a local
     * software reset + FIFO + BAUD setup (navq95_lpuart_init). The LPUART
     * functional clock is the 24 MHz oscillator root the SM leaves enabled.
     */
    (void)navq95_lpuart2_iomux();
    navq95_lpuart_init(NAVQ95_CONSOLE_LPUART_BASE, NAVQ95_LPUART_CLK_HZ,
                       NAVQ95_CONSOLE_BAUD);
    navq95_uart_rx_irq_enable();

    navq95_lowputs("\nBetaflight NAVQ95 (NXP i.MX 95, Cortex-M7)\n");
    navq95_lowputs("Type '#' then Enter to open the CLI.\n");
}

/*
 * ────────────────────────────────────────────────────────────────────────────
 * NOTE: the DWT-based time base (micros/microsISR/millis/delay/
 * delayMicroseconds), the cycle counter (cycleCounterInit/getCycleCounter) and
 * the clock-cycle conversions now live in NAVQ95/system.c, matching the
 * per-platform system.c layout of the other targets. The reset and failure
 * primitives below are still stubs and stay here until implemented.
 * ────────────────────────────────────────────────────────────────────────────
 */

/* ── Reset (stubs) ──────────────────────────────────────────────────────── */

void systemReset(void)
{
    for (;;) { }
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    (void)requestType;
    for (;;) { }
}

/*
 * Console serial port. The default serial configuration assigns FUNCTION_MSP to
 * portConfigs[0] (SERIAL_PORT_USB_VCP), and MSP enters the CLI on that port. We
 * therefore expose the LPUART console as the "VCP" port: usbVcpOpen() returns a
 * serialPort_t whose vTable routes to the LPUART primitives above (polled TX,
 * interrupt-driven RX ring), and usbVcpIsConnected() returns true so MSP opens
 * it. A dedicated UART driver can replace this later.
 */
#include "drivers/serial_usb_vcp.h"

static void navq95_serWrite(serialPort_t *instance, uint8_t ch)
{
    (void)instance;
    navq95_lowputc(ch);
}

static uint32_t navq95_serRxWaiting(const serialPort_t *instance)
{
    (void)instance;
    navq95_rx_poll_drain();
    return (s_rxHead - s_rxTail) & (NAVQ95_RX_RING_SZ - 1u);
}

static uint32_t navq95_serTxFree(const serialPort_t *instance)
{
    (void)instance;
    return 256u;                          /* TX is synchronous/polled: always room */
}

static uint8_t navq95_serRead(serialPort_t *instance)
{
    (void)instance;
    int c = navq95_rx_pop();
    return (c < 0) ? 0u : (uint8_t)c;      /* MSP only calls when RxWaiting>0 */
}

static void navq95_serSetBaud(serialPort_t *instance, uint32_t baudRate)
{
    (void)instance; (void)baudRate;        /* U-Boot/SM fixed the console baud */
}

static bool navq95_serTxEmpty(const serialPort_t *instance)
{
    (void)instance;
    return true;                           /* synchronous TX: always drained */
}

static void navq95_serSetMode(serialPort_t *instance, portMode_e mode)
{
    (void)instance; (void)mode;
}

static void navq95_serSetCtrlLineStateCb(serialPort_t *instance,
        void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    (void)instance; (void)cb; (void)context;
}

static void navq95_serSetBaudRateCb(serialPort_t *instance,
        void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context)
{
    (void)instance; (void)cb; (void)context;
}

static void navq95_serWriteBuf(serialPort_t *instance, const void *data, int count)
{
    (void)instance;
    const uint8_t *p = (const uint8_t *)data;
    for (int i = 0; i < count; i++) {
        navq95_lowputc(p[i]);
    }
}

static void navq95_serBeginWrite(serialPort_t *instance) { (void)instance; }
static void navq95_serEndWrite(serialPort_t *instance)   { (void)instance; }

static const struct serialPortVTable navq95_vTable = {
    .serialWrite               = navq95_serWrite,
    .serialTotalRxWaiting      = navq95_serRxWaiting,
    .serialTotalTxFree         = navq95_serTxFree,
    .serialRead                = navq95_serRead,
    .serialSetBaudRate         = navq95_serSetBaud,
    .isSerialTransmitBufferEmpty = navq95_serTxEmpty,
    .setMode                   = navq95_serSetMode,
    .setCtrlLineStateCb        = navq95_serSetCtrlLineStateCb,
    .setBaudRateCb             = navq95_serSetBaudRateCb,
    .writeBuf                  = navq95_serWriteBuf,
    .beginWrite                = navq95_serBeginWrite,
    .endWrite                  = navq95_serEndWrite,
};

static serialPort_t navq95_consolePort;

void usbVcpInit(void)
{
    /* console LPUART already up (U-Boot/SM) + RX IRQ enabled in systemInit() */
}

serialPort_t *usbVcpOpen(void)
{
    serialPort_t *p = &navq95_consolePort;
    p->vTable       = &navq95_vTable;
    p->mode         = MODE_RXTX;
    p->baudRate     = 115200;
    p->rxBufferSize = 0;
    p->txBufferSize = 0;
    p->rxCallback   = NULL;
    p->identifier   = 20;                  /* SERIAL_PORT_USB_VCP */
    return p;
}

uint32_t usbVcpGetBaudRate(struct serialPort_s *instance)
{
    (void)instance;
    return 115200u;
}

uint8_t usbVcpIsConnected(void)
{
    return 1;                              /* always "connected" so MSP opens it */
}

uint8_t usbVcpIsActive(void)
{
    return 1;
}

/* ════════════════════════════════════════════════════════════════════════════
 * Platform helpers and IO primitives the common code requires (cycle-count
 * conversions, IO record access, EXTI, failure handling, newlib syscalls, ...).
 * Uses CMSIS-Core intrinsics only; no vendor SDK dependency.
 * ════════════════════════════════════════════════════════════════════════════ */

/* ── Cycle-count helpers: moved to NAVQ95/system.c ──────────────────────── */

/* ── IO: all IO HAL (hardware primitives + record keeping) moved to
 *      NAVQ95/io_navq95.c, matching the STM32 layout where io_stm32.c owns the
 *      whole IO layer (IORead/IOWrite/IOHi/IOLo/IOToggle/IOConfigGPIO plus
 *      IOInitGlobal/IOGetByTag). Nothing IO-related lives here any more. ──── */

/* ── EXTI ───────────────────────────────────────────────────────────────── */
void EXTIInit(void)            { }
void EXTIRelease(IO_t io)      { (void)io; }

/* ── System / failure (stubs) ───────────────────────────────────────────── */
/* failureMode() / indicateFailure() now live in NAVQ95/system.c (they drive
 * the beeper and, once ported, the LED), matching the per-platform system.c
 * layout. Only the remaining reset-reason / pin stubs stay here. */

bool isMPUSoftReset(void)      { return false; }
void unusedPinsInit(void)      { }

/* ── Debug pins ─────────────────────────────────────────────────────────── */
void dbgPinHi(int index)       { (void)index; }
void dbgPinLo(int index)       { (void)index; }

/* ── Beeper ─────────────────────────────────────────────────────────────── */
void beeperPwmInit(const ioTag_t tag, uint16_t frequency) { (void)tag; (void)frequency; }
void pwmWriteBeeper(bool onoffBeep)                        { (void)onoffBeep; }

/* ── Timer (generic layer; no timer driver yet) ─────────────────────────── */
void timerSetPeriod(const timerHardware_t *timer, uint32_t period)   { (void)timer; (void)period; }
void timerSetCounter(const timerHardware_t *timer, uint32_t counter) { (void)timer; (void)counter; }
void timerEnable(const timerHardware_t *timer)                       { (void)timer; }
void timerDisable(const timerHardware_t *timer)                      { (void)timer; }
void timerEnableInterrupt(const timerHardware_t *timer)              { (void)timer; }
void timerReconfigureTimeBase(const timerHardware_t *timer, uint16_t period, uint32_t hz)
{
    (void)timer; (void)period; (void)hz;
}
void timerConfigUpdateCallback(const timerHardware_t *timer, timerOvrHandlerRec_t *cb)
{
    (void)timer; (void)cb;
}
void timerChannelOverflowHandlerInit(timerOvrHandlerRec_t *cb, timerOvrHandlerCallback *fn)
{
    (void)cb; (void)fn;
}
void pwmICConfig(timerResource_t *tim, uint8_t channel, uint16_t polarity, uint8_t filter)
{
    (void)tim; (void)channel; (void)polarity; (void)filter;
}

/* ── DShot bit-bang (not implemented) ───────────────────────────────────── */
bool isDshotBitbangActive(const motorDevConfig_t *motorConfig) { (void)motorConfig; return false; }
dshotBitbangStatus_e dshotBitbangGetStatus(void)              { return DSHOT_BITBANG_STATUS_OK; }

/* ── WS2811 LED strip (not implemented) ─────────────────────────────────── */
void ws2811LedStripInit(ioTag_t ioTag, ledStripFormatRGB_e ledFormat) { (void)ioTag; (void)ledFormat; }
void ws2811LedStripStartTransfer(void)                                { }
void ws2811LedStripUpdateTransferBuffer(const rgbColor24bpp_t *color, unsigned ledIndex)
{
    (void)color; (void)ledIndex;
}

/* ── Transponder IR (not implemented) ───────────────────────────────────── */
bool transponderIrInit(const ioTag_t ioTag, const transponderProvider_e provider)
{
    (void)ioTag; (void)provider; return false;
}
void transponderIrDisable(void)                   { }
bool transponderIrIsReady(void)                   { return false; }
void transponderIrTransmit(void)                  { }
void transponderIrUpdateData(const uint8_t *data) { (void)data; }

/* ── ExpressLRS timer (not implemented) ─────────────────────────────────── */
void expressLrsTimerEnableIRQs(const struct timerHardware_s *timer __attribute__((unused))) { }

/* ── DMA introspection (no DMA driver yet) ──────────────────────────────── */
uint32_t dmaGetDataLength(dmaResource_t *ref)  { (void)ref; return 0; }
const char *dmaGetDisplayString(void)          { return ""; }
int dmaGetHandlerCount(void)                   { return 0; }

/* ── ADC (no ADC driver yet) ────────────────────────────────────────────── */
uint16_t adcGetValue(adcSource_e source __attribute__((unused))) { return 0; }

/* ── Motor / servo / ESC (no PWM/DShot driver yet) ──────────────────────── */
bool motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorDevConfig, uint16_t idlePulse)
{
    (void)device; (void)motorDevConfig; (void)idlePulse; return false;
}
bool dshotPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig)
{
    (void)device; (void)motorConfig; return false;
}
void servoDevInit(const servoDevConfig_t *servoConfig) { (void)servoConfig; }
void servoWrite(uint8_t index, float value)       { (void)index; (void)value; }
/* escEnablePassthrough() is provided by drivers/serial_escserial.c (USE_ESCSERIAL)
 * and is intentionally not stubbed here to avoid a duplicate definition. */

/* ── Newlib syscall stubs (bare-metal; no OS) ───────────────────────────── */
int _close(int fd) { (void)fd; return -1; }
int _fstat(int fd, void *st) { (void)fd; (void)st; return -1; }
int _getpid(void) { return 1; }
int _isatty(int fd) { (void)fd; return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; return -1; }
long _lseek(int fd, long offset, int whence) { (void)fd; (void)offset; (void)whence; return -1; }
int _read(int fd, void *buf, unsigned count) { (void)fd; (void)buf; (void)count; return -1; }
void *_sbrk(int incr) { (void)incr; return (void *)-1; }
int _write(int fd, const void *buf, unsigned count) { (void)fd; (void)buf; (void)count; return -1; }
void _exit(int status) { (void)status; while (1) { } }

/* ========================================================================= *
 *  Driver stubs
 *
 *  No-op/placeholder bodies for peripheral driver functions the common code
 *  references but that are not yet implemented for this platform (SPI, EXTI,
 *  flash, sdcard, FAT, DShot/motor, WS2811, transponder, ADC, DMA, ...). Each
 *  signature matches its header exactly. Functions are replaced with real
 *  implementations as the corresponding drivers are brought up.
 * ========================================================================= */

/* ── SPI (drivers/bus_spi.h) ────────────────────────────────────────────── */
uint8_t spiReadReg(const extDevice_t *dev, uint8_t reg) { (void)dev; (void)reg; return 0; }
uint8_t spiReadRegMsk(const extDevice_t *dev, uint8_t reg) { (void)dev; (void)reg; return 0; }
void spiReadRegBuf(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length) { (void)dev; (void)reg; (void)data; (void)length; }
bool spiReadRegBufRB(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length) { (void)dev; (void)reg; (void)data; (void)length; return false; }
bool spiReadRegMskBufRB(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length) { (void)dev; (void)reg; (void)data; (void)length; return false; }
void spiWrite(const extDevice_t *dev, uint8_t data) { (void)dev; (void)data; }
void spiWriteReg(const extDevice_t *dev, uint8_t reg, uint8_t data) { (void)dev; (void)reg; (void)data; }
bool spiWriteRegRB(const extDevice_t *dev, uint8_t reg, uint8_t data) { (void)dev; (void)reg; (void)data; return false; }
uint8_t spiReadWrite(const extDevice_t *dev, uint8_t data) { (void)dev; (void)data; return 0; }
void spiWriteRegBuf(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint32_t length) { (void)dev; (void)reg; (void)data; (void)length; }
uint8_t spiReadWriteReg(const extDevice_t *dev, uint8_t reg, uint8_t data) { (void)dev; (void)reg; (void)data; return 0; }
void spiReadWriteBuf(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int len) { (void)dev; (void)txData; (void)rxData; (void)len; }
bool spiReadWriteBufRB(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int length) { (void)dev; (void)txData; (void)rxData; (void)length; return false; }
void spiSequence(const extDevice_t *dev, busSegment_t *segments) { (void)dev; (void)segments; }
void spiWait(const extDevice_t *dev) { (void)dev; }
bool spiIsBusy(const extDevice_t *dev) { (void)dev; return false; }
void spiSetClkDivisor(const extDevice_t *dev, uint16_t divider) { (void)dev; (void)divider; }
void spiSetClkPhasePolarity(const extDevice_t *dev, bool leadingEdge) { (void)dev; (void)leadingEdge; }
uint16_t spiCalculateDivider(uint32_t freq) { (void)freq; return 1; }
bool spiSetBusInstance(extDevice_t *dev, uint32_t device) { (void)dev; (void)device; return false; }
void spiDmaEnable(const extDevice_t *dev, bool enable) { (void)dev; (void)enable; }
bool spiUseDMA(const extDevice_t *dev) { (void)dev; return false; }
bool spiUseSDO_DMA(const extDevice_t *dev) { (void)dev; return false; }
spiDevice_e spiDeviceByInstance(const spiResource_t *instance) { (void)instance; return SPIINVALID; }
void spiLinkSegments(const extDevice_t *dev, busSegment_t *firstSegment, busSegment_t *secondSegment) { (void)dev; (void)firstSegment; (void)secondSegment; }
/* spiSequenceStart: platform hook in bus_spi_impl.h, called by bus_spi.c. */
void spiSequenceStart(const extDevice_t *dev) { (void)dev; }

/* ── EXTI (drivers/exti.h) — EXTIInit/EXTIRelease are stubbed earlier ────── */
void EXTIHandlerInit(extiCallbackRec_t *cb, extiHandlerCallback *fn) { (void)cb; (void)fn; }
void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, ioConfig_t config, extiTrigger_t trigger) { (void)io; (void)cb; (void)irqPriority; (void)config; (void)trigger; }
void EXTIEnable(IO_t io) { (void)io; }
void EXTIDisable(IO_t io) { (void)io; }

/* ── persistent (drivers/persistent.h) ──────────────────────────────────── */
uint32_t persistentObjectRead(persistentObjectId_e id) { (void)id; return 0; }
void persistentObjectWrite(persistentObjectId_e id, uint32_t value) { (void)id; (void)value; }

/* ── dshot (drivers/dshot.h) — variable, not defined in dshot.c this tree ─ */
bool useDshotTelemetry = false;
#ifdef USE_DSHOT_TELEMETRY
dshotTelemetryCycleCounters_t dshotDMAHandlerCycleCounters;
#endif

/* ── flash (drivers/flash/flash.h) ──────────────────────────────────────── */
const flashGeometry_t *flashGetGeometry(void) { return NULL; }
const flashPartition_t *flashPartitionFindByIndex(uint8_t index) { (void)index; return NULL; }
const char *flashPartitionGetTypeName(flashPartitionType_e type) { (void)type; return NULL; }
flashPartition_t *flashPartitionFindByType(flashPartitionType_e type) { (void)type; return NULL; }

/* ── Peripheral init entry points (no-op until the drivers are implemented) ─ */
void timerInit(void) { }
bool flashInit(const flashConfig_t *cfg) { (void)cfg; return false; }
void flashfsInit(void) { }
void sdcard_init(const sdcardConfig_t *config) { (void)config; }
void afatfs_init(void) { }
void debugInit(void) { }
spiResource_t *spiInstanceByDevice(spiDevice_e device) { (void)device; return NULL; }

/* ── flashfs (io/flashfs.h) ─────────────────────────────────────────────── */
bool flashfsIsSupported(void) { return false; }
void flashfsEraseCompletely(void) { }
bool flashfsIsReady(void) { return false; }
void flashfsEraseAsync(void) { }
uint32_t flashfsGetOffset(void) { return 0; }
void flashfsSeekAbs(uint32_t offset) { (void)offset; }
void flashfsWrite(const uint8_t *data, unsigned int len, bool sync) { (void)data; (void)len; (void)sync; }
void flashfsFlushSync(void) { }
int flashfsReadAbs(uint32_t offset, uint8_t *data, unsigned int len) { (void)offset; (void)data; (void)len; return 0; }
bool flashfsVerifyEntireFlash(void) { return false; }
uint32_t flashfsGetSize(void) { return 0; }
void flashfsClose(void) { }
bool flashfsIsEOF(void) { return true; }
uint32_t flashfsGetWriteBufferSize(void) { return 0; }
uint32_t flashfsGetWriteBufferFreeSpace(void) { return 0; }
bool flashfsFlushAsync(bool force) { (void)force; return true; }
void flashfsWriteByte(uint8_t byte) { (void)byte; }

/* ── sdcard (drivers/sdcard.h) ──────────────────────────────────────────── */
bool sdcard_isInserted(void) { return false; }
bool sdcard_isFunctional(void) { return false; }
bool sdcard_isInitialized(void) { return false; }
const sdcardMetadata_t *sdcard_getMetadata(void) { return NULL; }

/* ── asyncfatfs (io/asyncfatfs/asyncfatfs.h) ────────────────────────────── */
afatfsFilesystemState_e afatfs_getFilesystemState(void) { return AFATFS_FILESYSTEM_STATE_UNKNOWN; }
afatfsError_e afatfs_getLastError(void) { return AFATFS_ERROR_NONE; }
void afatfs_poll(void) { }
uint32_t afatfs_getContiguousFreeSpace(void) { return 0; }
bool afatfs_chdir(afatfsFilePtr_t dirHandle) { (void)dirHandle; return false; }
bool afatfs_fclose(afatfsFilePtr_t file, afatfsCallback_t callback) { (void)file; (void)callback; return false; }
bool afatfs_fopen(const char *filename, const char *mode, afatfsFileCallback_t complete) { (void)filename; (void)mode; (void)complete; return false; }
afatfsOperationStatus_e afatfs_findNext(afatfsFilePtr_t directory, afatfsFinder_t *finder, fatDirectoryEntry_t **dirEntry) { (void)directory; (void)finder; (void)dirEntry; return AFATFS_OPERATION_FAILURE; }
bool afatfs_mkdir(const char *filename, afatfsFileCallback_t complete) { (void)filename; (void)complete; return false; }
bool afatfs_funlink(afatfsFilePtr_t file, afatfsCallback_t callback) { (void)file; (void)callback; return false; }
bool afatfs_sectorCacheInSync(void) { return true; }
void afatfs_findLast(afatfsFilePtr_t directory) { (void)directory; }
bool afatfs_isFull(void) { return true; }
uint32_t afatfs_fwrite(afatfsFilePtr_t file, const uint8_t *buffer, uint32_t len) { (void)file; (void)buffer; (void)len; return 0; }
void afatfs_fputc(afatfsFilePtr_t file, uint8_t c) { (void)file; (void)c; }
bool afatfs_flush(void) { return true; }
uint32_t afatfs_getFreeBufferSpace(void) { return 0; }
void afatfs_findFirst(afatfsFilePtr_t directory, afatfsFinder_t *finder) { (void)directory; (void)finder; }

/* ── fat_standard (io/asyncfatfs/fat_standard.h) ────────────────────────── */
bool fat_isDirectoryEntryTerminator(fatDirectoryEntry_t *entry) { (void)entry; return true; }

/* ── ws2811 (drivers/light_ws2811strip.h) ───────────────────────────────── */
bool ws2811LedStripHardwareInit(void) { return false; }

/* ── IO port/pin index helpers (drivers/io_impl.h; platform-provided) ─────
 * cli.c cliResource prints these. Recover the 0-based gpioid/pin from the
 * ioRecs[] index, same scheme as the IMXRT port (IO_t is &ioRecs[gpioid*32+pin]). */
int IO_GPIOPortIdx(IO_t io)
{
    extern ioRec_t ioRecs[];
    if (!io) {
        return -1;
    }
    return (int)(((ioRec_t *)io - ioRecs) / DEFIO_PORT_PINS);
}

int IO_GPIOPinIdx(IO_t io)
{
    extern ioRec_t ioRecs[];
    if (!io) {
        return -1;
    }
    return (int)(((ioRec_t *)io - ioRecs) % DEFIO_PORT_PINS);
}

/* ── USB cable detect (drivers/usb_io.h; usb_io.c not compiled for NAVQ95) ─
 * Report inserted when the VCP reports connected, else false. */
bool usbCableIsInserted(void)
{
#if defined(USE_VCP)
    return usbVcpIsConnected() != 0;
#else
    return false;
#endif
}

/* Live cable detect, used by io/beeper.c (beeperUsbSuppressed) once USE_BEEPER
 * is enabled. No USB_DETECT pin on NavQ95, so mirror the real usb_io.c path and
 * fall back to the VCP active state. */
bool usbCableIsActive(void)
{
#if defined(USE_VCP)
    return usbVcpIsActive() != 0;
#else
    return false;
#endif
}
