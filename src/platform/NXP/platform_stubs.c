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

/* Shared i.MX 95 System-Manager SCMI helpers (pinmux/clock) + console debug
 * helpers, defined below and used by the peripheral bring-up files (LPI2C in
 * bus_i2c_navq95.c). Including the header here checks the definitions match the
 * shared declarations. */
#include "navq95_scmi.h"

/*
 * Console LPUART (LPUART2). systemInit() muxes the pads (over SCMI) and does a
 * local reset + BAUD setup. Register offsets per the i.MX 95 LPUART block;
 * bases for LPUART1..8:
 *   1=0x44380000 2=0x44390000 3=0x42570000 4=0x42580000
 *   5=0x42590000 6=0x425A0000 7=0x42690000 8=0x426A0000
 */
#ifndef NAVQ95_CONSOLE_LPUART_BASE
/* Console VCP shim on LPUART2 (dedicated pads, clocked by the SM at boot);
 * LPUART5 carries SBUS via the generic serial core (Betaflight UART5). */
#define NAVQ95_CONSOLE_LPUART_BASE 0x44390000u   /* LPUART2 (console) */
#endif
/* LPUART2 is clocked by the SM/U-Boot at boot (24 MHz XTAL), so NO SCMI clock-
 * enable is needed for the console (unlike LPUART5). */
#define NAVQ95_CONSOLE_CLK_ROOT    12u           /* LPUART2 CCM root (unused: SM on) */

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
#define LPUART_STAT_TC     (1u << 22)            /* transmission complete       */
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
 * bring-up diagnostics. They write to the same console as the boot banner. */
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

/* Block until the shift register has fully drained (TC set), so the last bytes
 * of a burst are never truncated if the core moves on immediately after. */
static void navq95_lowdrain(void)
{
    volatile uint32_t *stat =
        (volatile uint32_t *)(NAVQ95_CONSOLE_LPUART_BASE + LPUART_STAT_OFFSET);
    uint32_t guard = 2000000u;                 /* ~bounded; never hang the boot */
    while (!(*stat & LPUART_STAT_TC) && guard--) {
    }
}

/* Non-static console helpers for other platform TUs (e.g. the SD self-test).
 * puthex prints "0x" + 8 hex digits, no newline. */
void navq95_console_puts(const char *s)
{
    navq95_lowputs(s);
}

void navq95_console_puthex(uint32_t v)
{
    static const char hexd[] = "0123456789abcdef";
    char buf[11];
    buf[0] = '0';
    buf[1] = 'x';
    for (int i = 0; i < 8; i++) {
        buf[2 + i] = hexd[(v >> ((7 - i) * 4)) & 0xF];
    }
    buf[10] = '\0';
    navq95_lowputs(buf);
}

/* Public TX drain (wait for TC) for other platform TUs, so a burst of console
 * output (e.g. the SD self-test) fully leaves the shift register and the last
 * bytes are never truncated. */
void navq95_console_drain(void)
{
    navq95_lowdrain();
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
 * On i.MX 95 the IOMUXC block (0x443C0000) is owned by the SM (M33 domain);
 * direct M7 writes are silently dropped, so the M7 asks the SM to apply the
 * pinmux via SCMI PINCTRL_CONFIG_SET (protocol 0x19, msg 0x6) over the MU5
 * transport (mirrors NuttX CONFIG_IMX9_IOMUX_OVER_SCMI).
 *
 * Pads (authoritative, from the NavQ95 board support):
 *   TX pad UART2_TXD : MUX 0x443C01DC (ALT0), PAD 0x443C03E0 val 0x1E
 *   RX pad UART2_RXD : MUX 0x443C01D8 (ALT0 + SION), PAD 0x443C03DC val 0x00
 * RX uses the dedicated UART2_RXD pad at ALT0 (no daisy); SION forces the RX
 * input path on.
 */
#define IOMUXC_BASE            0x443C0000u
#define IOMUXC_MUX_UART2_TX    (IOMUXC_BASE + 0x01DCu)
#define IOMUXC_MUX_UART2_RX    (IOMUXC_BASE + 0x01D8u)
#define IOMUXC_PAD_UART2_TX    (IOMUXC_BASE + 0x03E0u)
#define IOMUXC_PAD_UART2_RX    (IOMUXC_BASE + 0x03DCu)
#define IOMUXC_MUX_SION        (1u << 4)        /* software input-on (force RX)  */
/*
 * Pad CONFIG (SW_PAD_CTL) values for the SCMI TYPE_CONFIG entry. The value sent
 * over SCMI is PRE-SHIFTED: the SM applies a <<1, so we send raw_value >> 1.
 *   TX: raw 0x1E (FSEL slow | DSE X4) -> send 0x0F.
 *   RX: raw 0x1200 (PU bit9 | HYS bit12) -> send 0x0900. An input pad needs
 *       hysteresis + a pull so the idle-high line reads and edges are seen.
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
 * = CCM root index + ROOT_CLOCK_OFFSET; attributes bit0 = ENABLE. */
#define SCMI_PROTO_CLOCK         0x14u
#define SCMI_MSG_CLOCK_CONFIG_SET 0x07u
#define SCMI_MSG_CLOCK_RATE_SET   0x05u
#define SCMI_CLOCK_ROUND_AUTO    (2u << 2)      /* flags: let the SM pick divider */
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
 * SCMI CLOCK_RATE_SET (msg 0x05): set a clock root's rate in Hz. A bare
 * CLOCK_CONFIG_SET (enable) leaves the root at the SM's DEFAULT rate, which is
 * NOT necessarily what we want (e.g. TPM4's default root rate != the 1 MHz the
 * motor PWM assumes, so a timer enabled-but-not-rate-set runs at the wrong
 * frequency — PX4 explicitly programs TPM4/5/6 = OSC_24M/24 = 1 MHz). This is a
 * VOID-reply SET (like CONFIG_SET / the SYSTEM reset), so navq95_scmi_txn
 * handles it on the boot path (unlike a payload-returning RATE_GET, which is
 * not safe with the minimal txn). Payload (NuttX /
 * PR#15853 scmiClockRateSet): { flags=ROUND_AUTO, clockId, rate_lo, rate_hi }.
 * Returns the SCMI status (0 = success).
 */
int navq95_scmi_clock_rate_set(uint32_t clockRootId, uint32_t rateHz)
{
    uint32_t clkId = clockRootId + SCMI_CLOCK_ROOT_OFFSET;

    navq95_w32(SMT_MSG_HEADER, navq95_scmi_header(SCMI_PROTO_CLOCK, SCMI_MSG_CLOCK_RATE_SET));
    navq95_w32(SMT_PAYLOAD + 0x00u, SCMI_CLOCK_ROUND_AUTO);   /* flags          */
    navq95_w32(SMT_PAYLOAD + 0x04u, clkId);                   /* clock id       */
    navq95_w32(SMT_PAYLOAD + 0x08u, rateHz);                  /* rate low 32    */
    navq95_w32(SMT_PAYLOAD + 0x0Cu, 0u);                      /* rate high 32   */
    /* length = header(4) + flags+id+rate_lo+rate_hi(16) = 20 */
    return navq95_scmi_txn(4u + 16u);
}

/* Non-static exports so other platform TUs (e.g. NXP/sdio_usdhc_navq95.c)
 * can drive the SM over the same SCMI channel. The SMT channel is polled/
 * synchronous, so there is no concurrency with the console/SPI users. */
int navq95_scmi_pinmux_ex(uint32_t muxReg, uint32_t muxMode, uint32_t sion,
                          uint32_t padReg, uint32_t padVal)
{
    return navq95_scmi_pinmux(muxReg, muxMode, sion, padReg, padVal);
}

int navq95_scmi_clock_enable_ex(uint32_t clockRootId)
{
    return navq95_scmi_clock_enable(clockRootId);
}

int navq95_scmi_clock_rate_set_ex(uint32_t clockRootId, uint32_t rateHz)
{
    return navq95_scmi_clock_rate_set(clockRootId, rateHz);
}

/* SCMI pinctrl daisy type ids (NuttX imx9_scmi.h). MUX=192/CONFIG=193 are
 * already defined above for the console. DAISYREG base offset = IOMUXC_BASE +
 * 0x408. Used by the shared LPI2C6 pads (GPIO_IO02/03, ALT6 + SELECT_INPUT). */
#define SCMI_PINCTRL_TYPE_DAISY_ID  194u
#define SCMI_PINCTRL_TYPE_DAISY_CFG 195u
#define SCMI_PINCTRL_DAISYREG_OFF   0x408u

/*
 * SCMI PINCTRL_CONFIG_SET for a SHARED pad that needs a SELECT_INPUT/daisy
 * register (ALT != 0 alternate-function routing). Emits 4 config pairs:
 * MUX, DAISY_ID, DAISY_CFG, CONFIG (num_configs = 4), mirroring NuttX
 * imx9_sm_iomucx_configure(). daisyReg is the absolute SELECT_INPUT register;
 * daisyVal is the input mux selector. Non-static (declared in navq95_scmi.h) so
 * NXP/bus_i2c_navq95.c can mux the shared LPI2C6 pads over the same channel.
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

/* NOTE: the LPUART2 (RC/SBUS) pad mux is NOT done here — RC/SBUS runs through
 * the generic serial core, and NXP/serial_uart_navq95.c muxes the dedicated
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
static int navq95_lpuart2_iomux(void);

void systemInit(void)
{
    cycleCounterInit();                        /* DWT microsecond time base */
    navq95_timebaseInit();                     /* 1 kHz SysTick folds CYCCNT wraps */

    /*
     * Console (LPUART2) init. For the SBUS-on-LPUART5 diagnostic the console VCP
     * shim is back on LPUART2's dedicated pads. LPUART2 is left clocked by the
     * SM/U-Boot at boot, so NO SCMI clock-enable is needed (unlike LPUART5); we
     * just mux the dedicated UART2 pads over SCMI then do the local reset/BAUD.
     */
    (void)navq95_lpuart2_iomux();
    navq95_lpuart_init(NAVQ95_CONSOLE_LPUART_BASE, NAVQ95_LPUART_CLK_HZ,
                       NAVQ95_CONSOLE_BAUD);
    navq95_uart_rx_irq_enable();

    /*
     * LINE WARM-UP. Unlike LPUART2 (which the SM/U-Boot leave running from
     * power-on, so the line is already idle-high and stable when we print),
     * LPUART5 is clock-ungated + pad-muxed + software-reset RIGHT HERE.
     * Immediately after that the TX line is still settling to the idle-high mark
     * level through the freshly-muxed pad, and the first characters can be
     * mangled/lost. Wait for the clock/pad to settle, then clock out a few NUL
     * bytes to drive the line to a clean idle before the real banner, and drain
     * to TC. This makes the opening bytes of the banner survive. */
    delayMicroseconds(2000);                 /* let clock root + pad mux settle */
    for (int i = 0; i < 8; i++) {
        navq95_lowputc('\0');                /* warm the line to idle-high       */
    }
    navq95_lowdrain();
    delayMicroseconds(1000);

    navq95_lowputs("\nBetaflight NAVQ95 (NXP i.MX 95, Cortex-M7)\n");
    navq95_lowputs("Type '#' then Enter to open the CLI.\n");
    navq95_lowdrain();   /* flush the shift register (TC) so nothing truncates */

#ifdef NAVQ95_NOR_PROBE
    /* Boot-time FlexSPI NOR probe (non-fatal): dumps the live controller state,
     * borrows the mcuboot-configured controller (flexspiNorInit), reads the NOR
     * status, and — under NAVQ95_NOR_WRITE_TEST — erases/programs/reads-back one
     * scratch sector of the config partition to prove persistence is viable.
     * Defined in NXP/flexspi_nor_selftest.c. Never hangs boot. */
    {
        extern void navq95_nor_probe(void);
        navq95_nor_probe();
    }
#endif

#ifdef NAVQ95_PWM_SELFTEST
    /* Boot-time TPM3 PWM self-test (BENCH ONLY — props OFF): prints the TPM3
     * SCMI clock-enable status + MOD/SC + per-pad SCMI pinmux return codes, then
     * drives a static 1500 us pulse on CH0..3 so a scope/logic-analyzer can
     * measure the period (PWM freq) and pulse width on pads IO04/IO20/IO12/IO24.
     * Runs BEFORE Betaflight's motorInit (which re-inits TPM3), so it isolates
     * the HW path from the core. Defined in NXP/pwm_navq95.c. Turn OFF for
     * normal use (it drives ESC lines). */
    {
        extern void navq95_pwm_selftest(void);
        navq95_pwm_selftest();
    }
#endif
}

/*
 * NOTE: the DWT-based time base (micros/microsISR/millis/delay/
 * delayMicroseconds), the cycle counter (cycleCounterInit/getCycleCounter) and
 * the clock-cycle conversions live in NXP/system.c, matching the per-platform
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
 * Console (LPUART2) dedicated-pad pinmux.
 *
 * For the SBUS-on-LPUART5 diagnostic the CLI console (USB-VCP shim) is on
 * LPUART2's DEDICATED UART2_RXD/TXD pads (ALT0, no SELECT_INPUT/daisy), which
 * the SM leaves clocked at boot. The shared-pad daisy pinmux that LPUART5 needed
 * now lives entirely in serial_uart_navq95.c (the generic-core UART5/SBUS row).
 * ════════════════════════════════════════════════════════════════════════════ */

/* Mux both LPUART2 console pads (RX + TX) via the SM over SCMI. LPUART2 uses the
 * DEDICATED UART2_RXD/TXD pads at ALT0 — no SELECT_INPUT/daisy (unlike LPUART5's
 * shared pads). RX gets SION + PU|HYS (0x0900) so the idle-high line reads; TX
 * gets 0x0F. (Console moved back to LPUART2 so LPUART5 is free for SBUS.) */
static int navq95_lpuart2_iomux(void)
{
    int rxs = navq95_scmi_pinmux(IOMUXC_MUX_UART2_RX, 0u, 1u,
                                 IOMUXC_PAD_UART2_RX, IOMUXC_PAD_LPUART_RXVAL);
    int txs = navq95_scmi_pinmux(IOMUXC_MUX_UART2_TX, 0u, 0u,
                                 IOMUXC_PAD_UART2_TX, IOMUXC_PAD_LPUART_TXVAL);
    return rxs ? rxs : txs;
}

/* RC/SBUS now lives on LPUART5 (Betaflight UART5) via the GENERIC serial core
 * (drivers/serial_uart.c + NXP/serial_uart_navq95.c uartHardware[] UART5 row,
 * shared ALT5 pads + daisy, 8E2/RXINV framing). The console (this file) is on
 * LPUART2's dedicated pads via navq95_lpuart2_iomux above. */

/* ════════════════════════════════════════════════════════════════════════════
 * Platform helpers and IO primitives the common code requires (cycle-count
 * conversions, IO record access, EXTI, failure handling, newlib syscalls, ...).
 * Uses CMSIS-Core intrinsics only; no vendor SDK dependency.
 * ════════════════════════════════════════════════════════════════════════════ */

/* ── Cycle-count helpers: moved to NXP/system.c ──────────────────────── */

/* ── IO: all IO HAL (hardware primitives + record keeping) moved to
 *      NXP/io_navq95.c, matching the STM32 layout where io_stm32.c owns the
 *      whole IO layer (IORead/IOWrite/IOHi/IOLo/IOToggle/IOConfigGPIO plus
 *      IOInitGlobal/IOGetByTag). Nothing IO-related lives here any more. ──── */

/* ── EXTI ───────────────────────────────────────────────────────────────── */
void EXTIInit(void)            { }
void EXTIRelease(IO_t io)      { (void)io; }

/* ── System / failure ───────────────────────────────────────────────────────
 * failureMode()/indicateFailure() live in NXP/system.c (they use the beeper
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

/* ── Motor / servo / ESC ────────────────────────────────────────────────
 * motorPwmDevInit() (standard PWM on TPM3) is now the REAL driver in
 * NXP/pwm_navq95.c — its stub was removed here (two-sided edit). DShot is
 * not implemented yet (needs eDMA / FlexIO), so dshotPwmDevInit still stubs to
 * false; servo is likewise not wired. */
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

/* ── SPI: moved to NXP/bus_spi_navq95.c ─────────────────────────────────────
 * The i.MX 95 LPSPI1 platform driver (spiHardware[]/spiPinConfigure/
 * spiInitDevice/spiCalculateDivider/spiCalculateClock/spiInitBusDMA/
 * spiInternalReadWriteBufPolled/spiSequenceStart) now lives in its own file,
 * NXP/bus_spi_navq95.c, alongside serial_uart_navq95.c and bus_i2c_navq95.c.
 * It reaches the System Manager through the same shared SCMI helpers declared
 * in navq95_scmi.h (navq95_scmi_pinmux / navq95_scmi_clock_enable), which are
 * still defined here.
 */

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
void debugInit(void) { }

/* sdcard_init/afatfs_init + the sdcard_* / afatfs_* / fat_* API are now provided
 * by the REAL drivers (drivers/sdcard.c + sdcard_sdio_baremetal.c + the
 * io/asyncfatfs stack, built under USE_SDCARD/USE_SDCARD_SDIO). Their stubs were
 * REMOVED from here to avoid duplicate symbols. The platform SD_* HAL that the
 * SDIO baremetal glue calls lives in NXP/sdio_usdhc_navq95.c. */

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
