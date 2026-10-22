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
#include "drivers/serial_uart.h"
#include "io/serial.h"
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

/* Shared i.MX 95 System-Manager SCMI helpers (pinmux/clock), defined below and
 * used by the peripheral bring-up files (LPI2C in bus_i2c_navq95.c). Including
 * the header here checks the definitions match the shared declarations. */
#include "navq95_scmi.h"

/*
 * Console LPUART (LPUART2). systemInit() muxes the pads (over SCMI) and does a
 * local reset + BAUD setup. Register offsets per the i.MX 95 LPUART block;
 * bases for LPUART1..8:
 *   1=0x44380000 2=0x44390000 3=0x42570000 4=0x42580000
 *   5=0x42590000 6=0x425A0000 7=0x42690000 8=0x426A0000
 */
#ifndef NAVQ95_CONSOLE_LPUART_BASE
#define NAVQ95_CONSOLE_LPUART_BASE 0x42590000u   /* LPUART5 (console moved here;
                                                  * RC/SBUS now owns LPUART2) */
#endif
/* The console LPUART5 root is NOT clock-enabled by the SM at boot (unlike the
 * old LPUART2 console, which the boot ROM/U-Boot left running). Enable it. */
#define NAVQ95_CONSOLE_CLK_ROOT    103u          /* LPUART5 CCM root */

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

/* Exposed console debug helpers (declared in navq95_scmi.h) used by the I2C
 * bring-up diagnostics. They write to the same LPUART5 console as the banner. */
void navq95_dbg_puts(const char *s)
{
    navq95_lowputs(s);
}

void navq95_dbg_hex32(uint32_t v)
{
    static const char hexd[] = "0123456789ABCDEF";
    navq95_lowputc('0');
    navq95_lowputc('x');
    for (int i = 28; i >= 0; i -= 4) {
        navq95_lowputc((uint8_t)hexd[(v >> i) & 0xFu]);
    }
}

/*
 * Interrupt-driven console RX: the LPUART IRQ fills a ring buffer that the
 * serialPort_t read path drains. The console now lives on LPUART5 (external IRQ
 * 66, vector index 82) in startup_navq95.s; RC/SBUS owns LPUART2.
 */
#define NAVQ95_RX_RING_SZ 256u                 /* power of two */
static volatile uint8_t  s_rxRing[NAVQ95_RX_RING_SZ];
static volatile uint32_t s_rxHead;             /* written by IRQ  */
static volatile uint32_t s_rxTail;             /* read by thread  */

void LPUART5_IRQHandler(void)
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

    NVIC_DisableIRQ(LPUART5_IRQn);
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
    NVIC_EnableIRQ(LPUART5_IRQn);
}

/* Enable the LPUART RX interrupt: set CTRL.RIE and enable the NVIC line. The
 * IRQ handler and the polled drain (navq95_rx_poll_drain) feed the same ring;
 * the console read path polls, so RX works regardless of interrupt latency. */
static void navq95_uart_rx_irq_enable(void)
{
    volatile uint32_t *ctrl =
        (volatile uint32_t *)(NAVQ95_CONSOLE_LPUART_BASE + LPUART_CTRL_OFFSET);
    *ctrl |= LPUART_CTRL_RIE;
    NVIC_EnableIRQ(LPUART5_IRQn);
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
/* CLOCK protocol (0x14): CLOCK_CONFIG_SET ungates a clock root. The SM clock id
 * = CCM root index + ROOT_CLOCK_OFFSET; attributes bit0 = ENABLE. CLOCK_RATE_SET
 * (0x05) programs the root's physical rate in Hz (64-bit, lower+upper). Some
 * roots (LPSPI1/LPUART5) are left at a sane rate by the boot ROM/U-Boot, but
 * others (e.g. the LPI2C roots) may come up with no rate set — so the M7 must
 * program the rate itself or the peripheral gets no functional clock. */
#define SCMI_PROTO_CLOCK         0x14u
#define SCMI_MSG_CLOCK_CONFIG_SET 0x07u
#define SCMI_MSG_CLOCK_RATE_SET   0x05u
#define SCMI_CLOCK_ROOT_OFFSET   41u
/* SYSTEM power-state protocol (0x12): SYSTEM_POWER_STATE_SET performs a warm
 * reset of this core's logical machine (other cores keep running). This is a
 * void-reply SET like the PINCTRL/CLOCK messages, so navq95_scmi_txn handles it
 * safely (no payload-returning GET on the boot path). Payload = { flags, state }.
 * Ported from PR #15853 scmi_imx95.c scmiSystemReset() (GPLv3), which sends
 * tx[] = { SCMI_SYSTEM_FORCEFUL=0, SCMI_SYSTEM_WARM_RESET=2 }. NOTE: FORCEFUL is
 * the VALUE 0 here (the SCMI flags word), not a bit — graceful vs forceful is
 * encoded as 0 in this message as #15853 uses it. */
#define SCMI_PROTO_SYSTEM             0x12u
#define SCMI_MSG_SYSTEM_POWER_STATE_SET 0x03u
#define SCMI_SYSTEM_FORCEFUL          0u          /* flags word = 0 (as #15853)  */
#define SCMI_SYSTEM_WARM_RESET        2u          /* system state: warm reset    */

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
 * Send one SCMI message (header already built by the caller into SMT_MSG_HEADER
 * and payload words written) of `len` bytes, and return the SM status word.
 * Handles the SMT handshake: wait FREE, write resv/flags/length, CRC, clear
 * FREE, ring the doorbell, wait for the response. Returns the SCMI status
 * (0 = success, negative = SCMI error, 1 = channel timeout). The caller must
 * have written SMT_MSG_HEADER + the payload before calling.
 */
static int navq95_scmi_txn(uint32_t len)
{
    uint32_t spin = 2000000u;
    while (!(navq95_r32(SMT_CHAN_STATUS) & SMT_FREE) && spin) {
        spin--;
    }
    if (!(navq95_r32(SMT_CHAN_STATUS) & SMT_FREE)) {
        return 1;   /* channel never became free */
    }

    navq95_w32(SMT_A2P_BUF + 0x00u, 0u);          /* resv */
    navq95_w32(SMT_CHAN_FLAGS, 0u);               /* poll, no completion IRQ */
    navq95_w32(SMT_LENGTH, len);
    navq95_smt_crc(len);                          /* CRC AFTER length, BEFORE FREE */

    navq95_w32(SMT_CHAN_STATUS, navq95_r32(SMT_CHAN_STATUS) & ~SMT_FREE);
    navq95_w32(MU5_GCR, navq95_r32(MU5_GCR) | MU5_GCR_GIR0);

    spin = 2000000u;
    while (!(navq95_r32(SMT_CHAN_STATUS) & SMT_FREE) && spin) {
        spin--;
    }
    if (!(navq95_r32(SMT_CHAN_STATUS) & SMT_FREE)) {
        return 1;   /* no response */
    }
    if (navq95_r32(SMT_LENGTH) >= 8u) {
        return (int)navq95_r32(SMT_MSG_HEADER + 0x04u);   /* status word */
    }
    return 0;
}

static uint32_t s_scmiToken;

static inline uint32_t navq95_scmi_header(uint32_t proto, uint32_t msg)
{
    uint32_t h = (msg & 0xFFu) | ((proto & 0xFFu) << 10) | ((s_scmiToken & 0x3FFu) << 18);
    s_scmiToken++;
    return h;
}

/*
 * SCMI PINCTRL_CONFIG_SET for a single pad: set its MUX (mode + optional SION)
 * and CONFIG (pad electrical) value. Returns the SCMI status (0 = success).
 */
int navq95_scmi_pinmux(uint32_t muxReg, uint32_t muxMode, uint32_t sion,
                       uint32_t padReg, uint32_t padVal)
{
    (void)padReg;   /* identifier is derived from muxReg; padReg kept for clarity */

    uint32_t identifier = (muxReg - IOMUXC_BASE) / 4u;   /* pin selector */
    uint32_t attributes = (2u << 2);                      /* SEL_PIN, num_configs=2 */
    uint32_t muxVal = (muxMode & 0x7u) | ((sion & 0x1u) << 4);

    navq95_w32(SMT_MSG_HEADER, navq95_scmi_header(SCMI_PROTO_PINCTRL, SCMI_MSG_PINCTRL_SET));
    navq95_w32(SMT_PAYLOAD + 0x00u, identifier);
    navq95_w32(SMT_PAYLOAD + 0x04u, 0xFFFFFFFFu); /* function_id = no function */
    navq95_w32(SMT_PAYLOAD + 0x08u, attributes);
    navq95_w32(SMT_PAYLOAD + 0x0Cu, SCMI_PINCTRL_TYPE_MUX);
    navq95_w32(SMT_PAYLOAD + 0x10u, muxVal);
    navq95_w32(SMT_PAYLOAD + 0x14u, SCMI_PINCTRL_TYPE_CONFIG);
    navq95_w32(SMT_PAYLOAD + 0x18u, padVal);
    /* length = header(4) + identifier+function_id+attributes(12) + 2*config(16) = 32 */
    return navq95_scmi_txn(4u + 12u + 2u * 8u);
}

/*
 * SCMI CLOCK_CONFIG_SET: ungate (enable) a clock root. clockRootId is the i.MX95
 * CCM root index (e.g. LPSPI1 = 8); the SM clock id = root + ROOT_CLOCK_OFFSET
 * (41). Enabling alone is enough to ungate — we keep the SM's default parent/
 * rate. Returns the SCMI status (0 = success).
 */
int navq95_scmi_clock_enable(uint32_t clockRootId)
{
    uint32_t clkId = clockRootId + SCMI_CLOCK_ROOT_OFFSET;

    navq95_w32(SMT_MSG_HEADER, navq95_scmi_header(SCMI_PROTO_CLOCK, SCMI_MSG_CLOCK_CONFIG_SET));
    navq95_w32(SMT_PAYLOAD + 0x00u, clkId);       /* clock id            */
    navq95_w32(SMT_PAYLOAD + 0x04u, 1u);          /* attributes: ENABLE  */
    navq95_w32(SMT_PAYLOAD + 0x08u, 0u);          /* oem_config_val = 0  */
    /* length = header(4) + clockid+attributes+oem(12) = 16 */
    return navq95_scmi_txn(4u + 12u);
}

/*
 * SCMI CLOCK_RATE_SET: program a clock root's physical rate (Hz). Ported from
 * NuttX imx9_scmi_clockratesett() (msg_tclockd5_t = { header, flags, clockid,
 * rate.lower, rate.upper }). flags = 0 (synchronous, round down). clockRootId
 * is the CCM root index; the SM clock id = root + ROOT_CLOCK_OFFSET (41).
 * Returns the SCMI status (0 = success). Needed for roots the boot firmware
 * leaves un-rated (e.g. the LPI2C roots) so the peripheral gets a real clock.
 */
int navq95_scmi_clock_rate_set(uint32_t clockRootId, uint32_t rateHz)
{
    uint32_t clkId = clockRootId + SCMI_CLOCK_ROOT_OFFSET;

    navq95_w32(SMT_MSG_HEADER, navq95_scmi_header(SCMI_PROTO_CLOCK, SCMI_MSG_CLOCK_RATE_SET));
    navq95_w32(SMT_PAYLOAD + 0x00u, 0u);          /* flags: sync, round down */
    navq95_w32(SMT_PAYLOAD + 0x04u, clkId);       /* clock id                */
    navq95_w32(SMT_PAYLOAD + 0x08u, rateHz);      /* rate lower 32 bits (Hz) */
    navq95_w32(SMT_PAYLOAD + 0x0Cu, 0u);          /* rate upper 32 bits      */
    /* length = header(4) + flags+clockid+rate.lo+rate.hi(16) = 20 */
    return navq95_scmi_txn(4u + 16u);
}

/* NOTE: the LPUART2 (RC/SBUS) pad mux is NOT done here — RC/SBUS runs through
 * the generic serial core, and NAVQ95/serial_uart_navq95.c muxes the dedicated
 * UART2 pads over SCMI in serialUART(). The console (LPUART5) pinmux lives in
 * navq95_lpuart5_iomux() further down. */

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

/* Forward decl: the console (moved to LPUART5) muxes its pads with the daisy
 * pinmux helper, which is defined further down with the RC/serial block. */
static int navq95_lpuart5_iomux(void);

void systemInit(void)
{
    cycleCounterInit();                        /* DWT microsecond time base */
    navq95_timebaseInit();                     /* 1 kHz SysTick folds CYCCNT wraps */

    /*
     * Console (LPUART5) init. RC/SBUS owns LPUART2's dedicated pads, so the
     * console lives on LPUART5. On i.MX 95 the M7 cannot write IOMUXC/CCM
     * directly — those blocks are owned by the System Manager (SM). So we:
     * (1) enable the LPUART5 clock root over SCMI (unlike LPUART2, the SM does
     * NOT leave LPUART5 clocked at boot); (2) mux the shared ALT5 TX/RX pads
     * over SCMI including the SELECT_INPUT daisy (navq95_lpuart5_iomux); then
     * (3) do the local reset + FIFO + BAUD setup.
     */
    navq95_scmi_clock_enable(NAVQ95_CONSOLE_CLK_ROOT);
    (void)navq95_lpuart5_iomux();
    navq95_lpuart_init(NAVQ95_CONSOLE_LPUART_BASE, NAVQ95_LPUART_CLK_HZ,
                       NAVQ95_CONSOLE_BAUD);
    navq95_uart_rx_irq_enable();

    navq95_lowputs("\nBetaflight NAVQ95 (NXP i.MX 95, Cortex-M7)\n");
    navq95_lowputs("Console on LPUART5; RC/SBUS on LPUART2 (Betaflight UART2).\n");
    navq95_lowputs("Type '#' then Enter to open the CLI.\n");
}

/*
 * NOTE: the DWT-based time base (micros/microsISR/millis/delay/
 * delayMicroseconds), the cycle counter (cycleCounterInit/getCycleCounter) and
 * the clock-cycle conversions live in NAVQ95/system.c, matching the per-platform
 * system.c layout of the other targets.
 */

/* ── Reset (SCMI warm reset) ────────────────────────────────────────────── */

/*
 * SCMI SYSTEM_POWER_STATE_SET warm reset. The System Manager resets this core's
 * logical machine; on success the SM resets us before the SCMI transaction even
 * returns. Returns the SCMI status (0 = success, negative = SCMI error,
 * 1 = channel timeout).
 */
static int navq95_scmi_system_reset(void)
{
    navq95_w32(SMT_MSG_HEADER,
               navq95_scmi_header(SCMI_PROTO_SYSTEM, SCMI_MSG_SYSTEM_POWER_STATE_SET));
    navq95_w32(SMT_PAYLOAD + 0x00u, SCMI_SYSTEM_FORCEFUL);   /* flags */
    navq95_w32(SMT_PAYLOAD + 0x04u, SCMI_SYSTEM_WARM_RESET); /* state */
    return navq95_scmi_txn(4u + 2u * 4u);   /* header + 2 payload words */
}

/* First two words of our vector table: [0]=initial SP, [1]=reset handler. */
extern uint32_t __isr_vector[];

void systemReset(void)
{
    __disable_irq();

    /* On success the System Manager resets this core; the fall-through below is
     * only reached if the SM refuses the reset. */
    if (navq95_scmi_system_reset() == 0) {
        for (;;) { }
    }

    /* Fallback (SM refused): re-enter our own reset vector from thread mode.
     * Only valid outside an exception (IPSR == 0). */
    if (__get_IPSR() == 0u) {
        __set_CONTROL(0);
        __ISB();
        ((void (*)(void))__isr_vector[1])();
    }

    for (;;) { }
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    (void)requestType;
    /* This core has no separate bootloader to enter; a warm reset is all we do. */
    systemReset();
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
 * Console (LPUART5) shared-pad pinmux.
 *
 * The CLI console runs on LPUART5 (exposed as the USB-VCP port above). LPUART5's
 * TX/RX use the SHARED GPIO_IO00/GPIO_IO01 pads at ALT5, which REQUIRE the
 * SELECT_INPUT/daisy register (unlike a dedicated pad). navq95_scmi_pinmux_daisy()
 * emits the extra DAISY_ID(194)/DAISY_CFG(195) SCMI config pairs, per NuttX
 * imx9_sm_iomucx_configure. RC/SBUS (UART2) uses the DEDICATED UART2 pads and is
 * muxed by serial_uart_navq95.c's own (non-daisy) SCMI pinmux.
 *
 * LPUART5 hardware (from PX4 board.h + NuttX imx9 hardware headers):
 *   base 0x42590000 (NAVQ95_CONSOLE_LPUART_BASE); CLK_ROOT index 103
 *   (NAVQ95_CONSOLE_CLK_ROOT, enabled over SCMI in systemInit — the SM does NOT
 *   leave LPUART5 clocked at boot). RX pad GPIO_IO01 (MUX 0x0014 ALT5 +SION, PAD
 *   0x0218, daisy 0x0570 val 0x1); TX pad GPIO_IO00 (MUX 0x0010 ALT5, PAD 0x0214,
 *   daisy 0x0574 val 0x1).
 * ════════════════════════════════════════════════════════════════════════════ */

/* LPUART5 (console) pad registers (absolute = IOMUXC_BASE + offset). */
#define IOMUXC_MUX_LPUART5_RX      (IOMUXC_BASE + 0x0014u)   /* GPIO_IO01 */
#define IOMUXC_MUX_LPUART5_TX      (IOMUXC_BASE + 0x0010u)   /* GPIO_IO00 */
#define IOMUXC_PAD_LPUART5_RX      (IOMUXC_BASE + 0x0218u)
#define IOMUXC_PAD_LPUART5_TX      (IOMUXC_BASE + 0x0214u)
#define IOMUXC_DSY_LPUART5_RX      (IOMUXC_BASE + 0x0570u)   /* RXD SELECT_INPUT */
#define IOMUXC_DSY_LPUART5_TX      (IOMUXC_BASE + 0x0574u)   /* TXD SELECT_INPUT */
#define IOMUXC_LPUART5_ALT         5u                        /* ALT mux mode     */
#define IOMUXC_LPUART5_DSY_VAL     1u                        /* daisy select     */

/* SCMI pinctrl daisy type ids (NuttX imx9_scmi.h). MUX=192/CONFIG=193 already
 * defined above for the console. DAISYREG base offset = IOMUXC_BASE + 0x408. */
#define SCMI_PINCTRL_TYPE_DAISY_ID  194u
#define SCMI_PINCTRL_TYPE_DAISY_CFG 195u
#define SCMI_PINCTRL_DAISYREG_OFF   0x408u

/*
 * SCMI PINCTRL_CONFIG_SET for a SHARED pad that needs a SELECT_INPUT/daisy
 * register (ALT != 0 alternate-function routing). Emits 4 config pairs:
 * MUX, DAISY_ID, DAISY_CFG, CONFIG (num_configs = 4), mirroring NuttX
 * imx9_sm_iomucx_configure(). daisyReg is the absolute SELECT_INPUT register;
 * daisyVal is the input mux selector (0x1 for both LPUART5 pads).
 */
int navq95_scmi_pinmux_daisy(uint32_t muxReg, uint32_t muxMode, uint32_t sion,
                             uint32_t daisyReg, uint32_t daisyVal,
                             uint32_t padVal)
{
    uint32_t identifier = (muxReg - IOMUXC_BASE) / 4u;
    uint32_t daisyId    = (daisyReg - (IOMUXC_BASE + SCMI_PINCTRL_DAISYREG_OFF)) / 4u;
    uint32_t attributes = (4u << 2);                      /* SEL_PIN, num_configs=4 */
    uint32_t muxVal = (muxMode & 0x7u) | ((sion & 0x1u) << 4);

    navq95_w32(SMT_MSG_HEADER, navq95_scmi_header(SCMI_PROTO_PINCTRL, SCMI_MSG_PINCTRL_SET));
    navq95_w32(SMT_PAYLOAD + 0x00u, identifier);
    navq95_w32(SMT_PAYLOAD + 0x04u, 0xFFFFFFFFu); /* function_id = no function */
    navq95_w32(SMT_PAYLOAD + 0x08u, attributes);
    navq95_w32(SMT_PAYLOAD + 0x0Cu, SCMI_PINCTRL_TYPE_MUX);
    navq95_w32(SMT_PAYLOAD + 0x10u, muxVal);
    navq95_w32(SMT_PAYLOAD + 0x14u, SCMI_PINCTRL_TYPE_DAISY_ID);
    navq95_w32(SMT_PAYLOAD + 0x18u, daisyId);
    navq95_w32(SMT_PAYLOAD + 0x1Cu, SCMI_PINCTRL_TYPE_DAISY_CFG);
    navq95_w32(SMT_PAYLOAD + 0x20u, daisyVal);
    navq95_w32(SMT_PAYLOAD + 0x24u, SCMI_PINCTRL_TYPE_CONFIG);
    navq95_w32(SMT_PAYLOAD + 0x28u, padVal);
    /* length = header(4) + id+func+attr(12) + 4*config(32) = 48 */
    return navq95_scmi_txn(4u + 12u + 4u * 8u);
}

/* Mux both LPUART5 pads (RX + TX) via the SM over SCMI (with daisy). RX pad gets
 * SION + PU|HYS (0x0900) so the idle-high line reads; TX pad gets 0x0F. */
static int navq95_lpuart5_iomux(void)
{
    int rxs = navq95_scmi_pinmux_daisy(IOMUXC_MUX_LPUART5_RX, IOMUXC_LPUART5_ALT, 1u,
                                       IOMUXC_DSY_LPUART5_RX, IOMUXC_LPUART5_DSY_VAL,
                                       IOMUXC_PAD_LPUART_RXVAL);
    int txs = navq95_scmi_pinmux_daisy(IOMUXC_MUX_LPUART5_TX, IOMUXC_LPUART5_ALT, 0u,
                                       IOMUXC_DSY_LPUART5_TX, IOMUXC_LPUART5_DSY_VAL,
                                       IOMUXC_PAD_LPUART_TXVAL);
    return rxs ? rxs : txs;
}

/* RC/SBUS (UART2 == LPUART2) is handled by the GENERIC serial core
 * (drivers/serial_uart.c) plus NAVQ95/serial_uart_navq95.c (uartHardware[] row
 * for UART2 + SBUS 8E2/RXINV framing in uartReconfigure). openSerialPort()
 * routes UART identifiers to the generic uartOpen(); this file no longer
 * provides its own uartOpen()/uartPinConfigure(). The LPUART5 pinmux helpers
 * above (navq95_scmi_pinmux_daisy / navq95_lpuart5_iomux) are kept — the
 * console (LPUART5) uses them. */

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

/* ── System / failure ───────────────────────────────────────────────────────
 * failureMode()/indicateFailure() live in NAVQ95/system.c (they use the beeper
 * + delay()). isMPUSoftReset/unusedPinsInit stay here. */
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

/* ── SPI: register-level LPSPI driver (polled) ──────────────────────────────
 * Betaflight's common bus_spi.c provides the high-level API (spiReadReg/
 * spiWriteReg/spiSequence/...) once USE_SPI is defined; the platform supplies
 * the hardware hooks below. This is a polled driver (no DMA): spiSequenceStart
 * calls the common spiProcessSegmentsPolled(), which drives the CS GPIO and
 * calls spiInternalReadWriteBufPolled() per segment.
 *
 * i.MX 95 LPSPI register layout (NuttX arch/arm/src/imx9/hardware/imx9_lpspi.h).
 * CS is a software GPIO (not hardware PCS); the common polled path toggles it.
 * The LPSPI functional clock is ungated over SCMI; the pads are muxed over SCMI
 * pinctrl (same path as the console). SPI1 == LPSPI1 (0x44360000, platform.h).
 */
#define LPSPI_CR_OFFSET     0x10u
#define LPSPI_SR_OFFSET     0x14u
#define LPSPI_CFGR1_OFFSET  0x24u
#define LPSPI_CCR_OFFSET    0x40u
#define LPSPI_FCR_OFFSET    0x58u
#define LPSPI_TCR_OFFSET    0x60u
#define LPSPI_TDR_OFFSET    0x64u
#define LPSPI_RDR_OFFSET    0x74u
#define LPSPI_CR_MEN        (1u << 0)
#define LPSPI_CR_RST        (1u << 1)
#define LPSPI_CR_RTF        (1u << 8)
#define LPSPI_CR_RRF        (1u << 9)
#define LPSPI_SR_TDF        (1u << 0)
#define LPSPI_SR_RDF        (1u << 1)
#define LPSPI_CFGR1_MASTER  (1u << 0)
#define LPSPI_TCR_FRAMESZ(n)  ((uint32_t)(n) & 0xFFFu)
#define LPSPI_TCR_PRESCALE(n) (((uint32_t)(n) & 0x7u) << 27)
#define LPSPI_TCR_PCS(n)      (((uint32_t)(n) & 0x7u) << 24)
#define LPSPI_TCR_CPHA      (1u << 30)
#define LPSPI_TCR_CPOL      (1u << 31)
#define LPSPI_CCR_SCKDIV(n) ((uint32_t)(n) & 0xFFu)

/* NavQ95 LPSPI1 is on the OSC_24M root (SCMI CLOCK_ROOT id 8). */
#define NAVQ95_LPSPI1_BASE     0x44360000u
#define NAVQ95_LPSPI1_CLK_ROOT 8u
#define NAVQ95_LPSPI1_SRC_HZ   24000000u

/* LPSPI1 pads (IOMUXC, ALT 2 for the LPSPI function; CS is a GPIO, ALT 5).
 * Source: PX4/NuttX NavQ95 board.h (SAI1_* balls routed to LPSPI1). */
#define IOMUXC_MUX_LPSPI1_SCK  (IOMUXC_BASE + 0x01F4u)   /* SAI1_TXD0 */
#define IOMUXC_PAD_LPSPI1_SCK  (IOMUXC_BASE + 0x03F8u)
#define IOMUXC_MUX_LPSPI1_SOUT (IOMUXC_BASE + 0x01F8u)   /* SAI1_RXD0 (MOSI) */
#define IOMUXC_PAD_LPSPI1_SOUT (IOMUXC_BASE + 0x03FCu)
#define IOMUXC_MUX_LPSPI1_SIN  (IOMUXC_BASE + 0x01F0u)   /* SAI1_TXC  (MISO) */
#define IOMUXC_PAD_LPSPI1_SIN  (IOMUXC_BASE + 0x03F4u)
#define IOMUXC_MUX_LPSPI1_CS   (IOMUXC_BASE + 0x01ECu)   /* SAI1_TXFS -> GPIO1_IO11 */
#define IOMUXC_PAD_LPSPI1_CS   (IOMUXC_BASE + 0x03F0u)
#define IOMUXC_LPSPI1_ALT      0x2u
#define IOMUXC_LPSPI1_CS_ALT   0x5u                       /* GPIO alt for the CS pad */
/* Pad value (pre-shift, SM applies <<1): SPI signals get drive strength;
 * MISO (input) + CS readback get hysteresis. */
#define IOMUXC_PAD_SPI_OUT     0x0Fu                      /* -> raw 0x1E (DSE X4)      */
#define IOMUXC_PAD_SPI_IN      0x0900u                    /* -> raw 0x1200 (PU | HYS)  */

static int navq95_lpspi1_padmux(void)
{
    int s = 0, r;
    r = navq95_scmi_pinmux(IOMUXC_MUX_LPSPI1_SCK,  IOMUXC_LPSPI1_ALT, 0u,
                           IOMUXC_PAD_LPSPI1_SCK,  IOMUXC_PAD_SPI_OUT); if (!s) s = r;
    r = navq95_scmi_pinmux(IOMUXC_MUX_LPSPI1_SOUT, IOMUXC_LPSPI1_ALT, 0u,
                           IOMUXC_PAD_LPSPI1_SOUT, IOMUXC_PAD_SPI_OUT); if (!s) s = r;
    r = navq95_scmi_pinmux(IOMUXC_MUX_LPSPI1_SIN,  IOMUXC_LPSPI1_ALT, 1u,
                           IOMUXC_PAD_LPSPI1_SIN,  IOMUXC_PAD_SPI_IN);  if (!s) s = r;
    /* CS as GPIO (software-driven by the common polled path). */
    r = navq95_scmi_pinmux(IOMUXC_MUX_LPSPI1_CS,   IOMUXC_LPSPI1_CS_ALT, 0u,
                           IOMUXC_PAD_LPSPI1_CS,   IOMUXC_PAD_SPI_OUT); if (!s) s = r;
    return s;
}

/* Program LPSPI1 as an 8-bit SPI mode-3 master. SCK ~= src/(2^prescale*(sckdiv+2)).
 * Mode 3 (CPOL=1,CPHA=1) is the ICM-45686 default. divider selects SCK. */
static void navq95_lpspi1_configure(uint32_t sckdiv, uint32_t prescale)
{
    volatile uint32_t *cr    = (volatile uint32_t *)(NAVQ95_LPSPI1_BASE + LPSPI_CR_OFFSET);
    volatile uint32_t *cfgr1 = (volatile uint32_t *)(NAVQ95_LPSPI1_BASE + LPSPI_CFGR1_OFFSET);
    volatile uint32_t *ccr   = (volatile uint32_t *)(NAVQ95_LPSPI1_BASE + LPSPI_CCR_OFFSET);
    volatile uint32_t *fcr   = (volatile uint32_t *)(NAVQ95_LPSPI1_BASE + LPSPI_FCR_OFFSET);
    volatile uint32_t *tcr   = (volatile uint32_t *)(NAVQ95_LPSPI1_BASE + LPSPI_TCR_OFFSET);

    *cr = 0u;                                   /* disable (MEN=0) */
    *cr = LPSPI_CR_RST;                         /* software reset  */
    *cr = LPSPI_CR_RTF | LPSPI_CR_RRF;          /* flush FIFOs     */
    *cr = 0u;

    *cfgr1 = LPSPI_CFGR1_MASTER;                /* master, SIN/SOUT, output retain */
    *ccr   = LPSPI_CCR_SCKDIV(sckdiv);          /* SCK divider (MEN must be 0) */
    *fcr   = 0u;                                /* watermarks 0 */

    *cr = LPSPI_CR_MEN;                         /* enable so TCR is writable */
    *tcr = LPSPI_TCR_FRAMESZ(7u)                /* 8-bit frames (bits-1) */
         | LPSPI_TCR_PRESCALE(prescale)
         | LPSPI_TCR_PCS(0u)
         | LPSPI_TCR_CPOL | LPSPI_TCR_CPHA;     /* SPI mode 3 */
    (void)*tcr;
}

/* spiHardware[] — SPI1 maps to LPSPI1. Pins are placeholders (real CS/SCK/etc.
 * come from the PG config / target.h); the common layer matches ioTags against
 * these, but our single-bus bring-up resolves the instance directly. */
const spiHardware_t spiHardware[SPIDEV_COUNT] = {
    {
        .device = SPIDEV_1,
        .reg    = (spiResource_t *)SPI1,
    },
};

void spiPinConfigure(const struct spiPinConfig_s *pConfig)
{
    (void)pConfig;
    /* Single bus: bind SPI1 -> LPSPI1 so spiInstanceByDevice() resolves it. */
    extern spiDevice_t spiDevice[SPIDEV_COUNT];
    spiDevice[SPIDEV_1].dev = (spiResource_t *)SPI1;
}

void spiInitDevice(spiDevice_e device)
{
    if (device != SPIDEV_1) {
        return;
    }
    /* Ungate the LPSPI1 clock + mux its pads via the SM, then configure the
     * peripheral. ~6 MHz SCK (24 MHz src, prescale 0, sckdiv 2) for bring-up. */
    navq95_scmi_clock_enable(NAVQ95_LPSPI1_CLK_ROOT);
    navq95_lpspi1_padmux();
    navq95_lpspi1_configure(2u, 0u);
}

/* Betaflight encodes SPI speed as a power-of-two "divider" of a nominal clock.
 * We keep it simple: carry the divider through and derive the LPSPI SCKDIV in
 * spiInternal* if needed. For bring-up the fixed configure() rate is used. */
uint16_t spiCalculateDivider(uint32_t freq)
{
    uint32_t div = NAVQ95_LPSPI1_SRC_HZ / (freq ? freq : 1u);
    if (div < 2u) {
        div = 2u;
    }
    return (uint16_t)div;
}

uint32_t spiCalculateClock(uint16_t spiClkDivisor)
{
    return NAVQ95_LPSPI1_SRC_HZ / (spiClkDivisor ? spiClkDivisor : 1u);
}

void spiInitBusDMA(void)
{
    /* Polled-only: no DMA. Called unconditionally from fc/init.c. */
}

/* Platform polled transfer: exchange `len` bytes on `instance` (== LPSPI1).
 * NULL tx => send 0xFF; NULL rx => discard. One RDR read per TDR write. */
bool spiInternalReadWriteBufPolled(spiResource_t *instance, const uint8_t *txData,
                                   uint8_t *rxData, int len)
{
    uint32_t base = (uint32_t)instance;
    volatile uint32_t *sr  = (volatile uint32_t *)(base + LPSPI_SR_OFFSET);
    volatile uint32_t *tdr = (volatile uint32_t *)(base + LPSPI_TDR_OFFSET);
    volatile uint32_t *rdr = (volatile uint32_t *)(base + LPSPI_RDR_OFFSET);

    for (int i = 0; i < len; i++) {
        uint8_t tx = txData ? txData[i] : 0xFFu;

        uint32_t spin = 1000000u;
        while (!(*sr & LPSPI_SR_TDF) && spin) {
            spin--;
        }
        *tdr = tx;

        spin = 1000000u;
        while (!(*sr & LPSPI_SR_RDF) && spin) {
            spin--;
        }
        uint8_t rx = (uint8_t)(*rdr & 0xFFu);
        if (rxData) {
            rxData[i] = rx;
        }
        (void)*sr;                              /* read SR clears error flags */
    }
    return true;
}

/* Platform SPI sequence entry: polled-only, so always use the common polled
 * segment processor (which drives the CS GPIO and calls the primitive above). */
void spiSequenceStart(const extDevice_t *dev)
{
    spiProcessSegmentsPolled(dev);
}

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

/* flash + flashfs: now provided by the REAL drivers/flash/flash.c and
 * io/flashfs.c (built in NAVQ95.mk; USE_SPI auto-enables USE_FLASH_CHIP via
 * common_post.h). Their stubs were REMOVED from here to avoid duplicate
 * symbols (flashInit, flashGetGeometry, flashPartitionXxx, flashfsXxx). */

/* ── Peripheral init entry points (no-op until the drivers are implemented) ─ */
void timerInit(void) { }
void sdcard_init(const sdcardConfig_t *config) { (void)config; }
void afatfs_init(void) { }
void debugInit(void) { }


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
 * Report inserted/active when the VCP reports connected/active, else false.
 * usbCableIsActive() is the "live" check beeper.c uses (io/beeper.c) and must
 * be provided alongside usbCableIsInserted(); without it the link fails with
 * "undefined reference to usbCableIsActive" once USE_BEEPER is enabled. */
bool usbCableIsInserted(void)
{
#if defined(USE_VCP)
    return usbVcpIsConnected() != 0;
#else
    return false;
#endif
}

bool usbCableIsActive(void)
{
#if defined(USE_VCP)
    return usbVcpIsActive() != 0;
#else
    return false;
#endif
}
