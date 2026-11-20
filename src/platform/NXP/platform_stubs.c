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
 * NavQ95 (i.MX 95 Cortex-M7) platform support: startup, the LPUART2 console
 * exposed as the VCP port, the System Manager SCMI client, reset, and stubs for
 * drivers this platform does not provide yet.
 *
 * On the i.MX 95 the System Manager owns IOMUXC and the clock controller, so
 * pad muxing and clock enables are SCMI requests over the MU5 shared-memory
 * transport.
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

#include "navq95_scmi.h"

#ifndef NAVQ95_CONSOLE_LPUART_BASE
// LPUART2 is left clocked by the boot firmware at 24 MHz.
#define NAVQ95_CONSOLE_LPUART_BASE 0x44390000u
#endif

#define LPUART_GLOBAL_OFFSET 0x08u
#define LPUART_BAUD_OFFSET   0x10u
#define LPUART_FIFO_OFFSET   0x28u
#define LPUART_STAT_OFFSET 0x14u
#define LPUART_CTRL_OFFSET 0x18u
#define LPUART_DATA_OFFSET 0x1Cu
#define LPUART_MODIR_OFFSET  0x24u
#define LPUART_GLOBAL_RST  (1u << 1)
#define LPUART_FIFO_RXFE    (1u << 3)
#define LPUART_FIFO_TXFE    (1u << 7)
#define LPUART_FIFO_RXFLUSH (1u << 14)
#define LPUART_FIFO_TXFLUSH (1u << 15)
#define LPUART_STAT_TDRE   (1u << 23)
#define LPUART_STAT_TC     (1u << 22)
#define LPUART_STAT_RDRF   (1u << 21)
#define LPUART_STAT_IDLE   (1u << 20)
#define LPUART_STAT_OR     (1u << 19)
#define LPUART_STAT_NF     (1u << 18)
#define LPUART_STAT_FE     (1u << 17)
#define LPUART_STAT_PF     (1u << 16)

// Cleared before enabling RX so a stale error flag cannot block RDRF.
#define LPUART_STAT_W1C    (LPUART_STAT_IDLE | LPUART_STAT_OR | LPUART_STAT_NF | \
                            LPUART_STAT_FE | LPUART_STAT_PF)
#define LPUART_CTRL_RE     (1u << 18)
#define LPUART_CTRL_TE     (1u << 19)
#define LPUART_CTRL_RIE    (1u << 21)
#define LPUART_BAUD_SBR(n)      ((uint32_t)(n) & 0x1FFFu)
#define LPUART_BAUD_BOTHEDGE    (1u << 17)
#define LPUART_BAUD_OSR(n)      (((uint32_t)((n) - 1u) & 0x1Fu) << 24)
#define LPUART_DATA_MASK   0xFFu

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

// Wait for the shift register to empty so the end of a burst is not cut off.
static void navq95_lowdrain(void)
{
    volatile uint32_t *stat =
        (volatile uint32_t *)(NAVQ95_CONSOLE_LPUART_BASE + LPUART_STAT_OFFSET);
    uint32_t guard = 2000000u;
    while (!(*stat & LPUART_STAT_TC) && guard--) {
    }
}

// Console RX: the IRQ and the polled read path both fill this ring.
#define NAVQ95_RX_RING_SZ 256u
static volatile uint8_t  s_rxRing[NAVQ95_RX_RING_SZ];
static volatile uint32_t s_rxHead;
static volatile uint32_t s_rxTail;

void LPUART2_IRQHandler(void)
{
    volatile uint32_t *stat =
        (volatile uint32_t *)(NAVQ95_CONSOLE_LPUART_BASE + LPUART_STAT_OFFSET);
    volatile uint32_t *data =
        (volatile uint32_t *)(NAVQ95_CONSOLE_LPUART_BASE + LPUART_DATA_OFFSET);

    uint32_t s = *stat;
    if (s & LPUART_STAT_OR) {
        *stat = LPUART_STAT_OR;
    }
    while (*stat & LPUART_STAT_RDRF) {
        uint8_t b = (uint8_t)(*data & LPUART_DATA_MASK);
        uint32_t next = (s_rxHead + 1u) & (NAVQ95_RX_RING_SZ - 1u);
        if (next != s_rxTail) {
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

// The IRQ is masked while polling so the two producers never race on s_rxHead.
static void navq95_rx_poll_drain(void)
{
    volatile uint32_t *stat =
        (volatile uint32_t *)(NAVQ95_CONSOLE_LPUART_BASE + LPUART_STAT_OFFSET);
    volatile uint32_t *data =
        (volatile uint32_t *)(NAVQ95_CONSOLE_LPUART_BASE + LPUART_DATA_OFFSET);

    NVIC_DisableIRQ(LPUART2_IRQn);
    if (*stat & LPUART_STAT_OR) {
        *stat = LPUART_STAT_OR;
    }
    while (*stat & LPUART_STAT_RDRF) {
        uint8_t b = (uint8_t)(*data & LPUART_DATA_MASK);
        uint32_t next = (s_rxHead + 1u) & (NAVQ95_RX_RING_SZ - 1u);
        if (next != s_rxTail) {
            s_rxRing[s_rxHead] = b;
            s_rxHead = next;
        }
    }
    NVIC_EnableIRQ(LPUART2_IRQn);
}

static void navq95_uart_rx_irq_enable(void)
{
    volatile uint32_t *ctrl =
        (volatile uint32_t *)(NAVQ95_CONSOLE_LPUART_BASE + LPUART_CTRL_OFFSET);
    *ctrl |= LPUART_CTRL_RIE;
    NVIC_EnableIRQ(LPUART2_IRQn);
}

#define IOMUXC_BASE            0x443C0000u
#define IOMUXC_MUX_UART2_TX    (IOMUXC_BASE + 0x01DCu)
#define IOMUXC_MUX_UART2_RX    (IOMUXC_BASE + 0x01D8u)
#define IOMUXC_PAD_UART2_TX    (IOMUXC_BASE + 0x03E0u)
#define IOMUXC_PAD_UART2_RX    (IOMUXC_BASE + 0x03DCu)

// Pad control values for the console pads; RX needs a pull-up and hysteresis.
#define IOMUXC_PAD_LPUART_TXVAL 0x0Fu
#define IOMUXC_PAD_LPUART_RXVAL 0x0900u

// SCMI transport: MU5 doorbell plus the A2P shared-memory buffer.
#define MU5_BASE               0x44610000u
#define MU5_GCR                (MU5_BASE + 0x0114u)
#define MU5_GCR_GIR0           (1u << 0)
#define SMT_A2P_BUF            0x44611000u

#define SMT_CHAN_STATUS        (SMT_A2P_BUF + 0x04u)
#define SMT_CHAN_FLAGS         (SMT_A2P_BUF + 0x10u)
#define SMT_LENGTH             (SMT_A2P_BUF + 0x14u)
#define SMT_MSG_HEADER         (SMT_A2P_BUF + 0x18u)
#define SMT_PAYLOAD            (SMT_A2P_BUF + 0x1Cu)
#define SMT_FREE               (1u << 0)
#define SMT_IMP_STATUS         (SMT_A2P_BUF + 0x08u)
#define SMT_IMP_CRC            (SMT_A2P_BUF + 0x0Cu)

// Message CRC mode, chosen per channel by the System Manager.
#define SMT_CRC_NONE           0u
#define SMT_CRC_XOR            1u
#define SMT_CRC_CRC32          3u

#define SCMI_PROTO_PINCTRL     0x19u
#define SCMI_MSG_PINCTRL_SET   0x06u

#define SCMI_PINCTRL_TYPE_MUX    192u
#define SCMI_PINCTRL_TYPE_CONFIG 193u

#define SCMI_PROTO_CLOCK         0x14u
#define SCMI_MSG_CLOCK_CONFIG_SET 0x07u
#define SCMI_MSG_CLOCK_RATE_SET   0x05u
#define SCMI_CLOCK_ROUND_AUTO    (2u << 2)
#define SCMI_CLOCK_ROOT_OFFSET   41u

#define SCMI_PROTO_SYSTEM             0x12u
#define SCMI_MSG_SYSTEM_POWER_STATE_SET 0x03u
#define SCMI_SYSTEM_FORCEFUL          0u
#define SCMI_SYSTEM_WARM_RESET        2u

static inline void navq95_w32(uint32_t addr, uint32_t v)
{
    *(volatile uint32_t *)addr = v;
}
static inline uint32_t navq95_r32(uint32_t addr)
{
    return *(volatile uint32_t *)addr;
}

static uint32_t navq95_crcxor(const uint32_t *addr, uint32_t words)
{
    uint32_t crc = 0u;
    while (words--) {
        crc ^= *addr++;
    }
    return crc;
}

// Not the zlib CRC-32: initial value 0 and no final XOR, as the System Manager expects.
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
 * Send the message already written to the SMT buffer and wait for the reply.
 * Returns the SCMI status, or 1 on a channel timeout.
 */
static int navq95_scmi_txn(uint32_t len)
{
    uint32_t spin = 2000000u;
    while (!(navq95_r32(SMT_CHAN_STATUS) & SMT_FREE) && spin) {
        spin--;
    }
    if (!(navq95_r32(SMT_CHAN_STATUS) & SMT_FREE)) {
        return 1;
    }

    navq95_w32(SMT_A2P_BUF + 0x00u, 0u);
    navq95_w32(SMT_CHAN_FLAGS, 0u);
    navq95_w32(SMT_LENGTH, len);
    navq95_smt_crc(len);

    navq95_w32(SMT_CHAN_STATUS, navq95_r32(SMT_CHAN_STATUS) & ~SMT_FREE);
    navq95_w32(MU5_GCR, navq95_r32(MU5_GCR) | MU5_GCR_GIR0);

    spin = 2000000u;
    while (!(navq95_r32(SMT_CHAN_STATUS) & SMT_FREE) && spin) {
        spin--;
    }
    if (!(navq95_r32(SMT_CHAN_STATUS) & SMT_FREE)) {
        return 1;
    }
    if (navq95_r32(SMT_LENGTH) >= 8u) {
        return (int)navq95_r32(SMT_MSG_HEADER + 0x04u);
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

int navq95_scmi_pinmux(uint32_t muxReg, uint32_t muxMode, uint32_t sion,
                       uint32_t padReg, uint32_t padVal)
{
    (void)padReg;

    uint32_t identifier = (muxReg - IOMUXC_BASE) / 4u;
    uint32_t attributes = (2u << 2);
    uint32_t muxVal = (muxMode & 0x7u) | ((sion & 0x1u) << 4);

    navq95_w32(SMT_MSG_HEADER, navq95_scmi_header(SCMI_PROTO_PINCTRL, SCMI_MSG_PINCTRL_SET));
    navq95_w32(SMT_PAYLOAD + 0x00u, identifier);
    navq95_w32(SMT_PAYLOAD + 0x04u, 0xFFFFFFFFu);
    navq95_w32(SMT_PAYLOAD + 0x08u, attributes);
    navq95_w32(SMT_PAYLOAD + 0x0Cu, SCMI_PINCTRL_TYPE_MUX);
    navq95_w32(SMT_PAYLOAD + 0x10u, muxVal);
    navq95_w32(SMT_PAYLOAD + 0x14u, SCMI_PINCTRL_TYPE_CONFIG);
    navq95_w32(SMT_PAYLOAD + 0x18u, padVal);

    return navq95_scmi_txn(4u + 12u + 2u * 8u);
}

int navq95_scmi_clock_enable(uint32_t clockRootId)
{
    uint32_t clkId = clockRootId + SCMI_CLOCK_ROOT_OFFSET;

    navq95_w32(SMT_MSG_HEADER, navq95_scmi_header(SCMI_PROTO_CLOCK, SCMI_MSG_CLOCK_CONFIG_SET));
    navq95_w32(SMT_PAYLOAD + 0x00u, clkId);
    navq95_w32(SMT_PAYLOAD + 0x04u, 1u);
    navq95_w32(SMT_PAYLOAD + 0x08u, 0u);

    return navq95_scmi_txn(4u + 12u);
}

int navq95_scmi_clock_rate_set(uint32_t clockRootId, uint32_t rateHz)
{
    uint32_t clkId = clockRootId + SCMI_CLOCK_ROOT_OFFSET;

    navq95_w32(SMT_MSG_HEADER, navq95_scmi_header(SCMI_PROTO_CLOCK, SCMI_MSG_CLOCK_RATE_SET));
    navq95_w32(SMT_PAYLOAD + 0x00u, SCMI_CLOCK_ROUND_AUTO);
    navq95_w32(SMT_PAYLOAD + 0x04u, clkId);
    navq95_w32(SMT_PAYLOAD + 0x08u, rateHz);
    navq95_w32(SMT_PAYLOAD + 0x0Cu, 0u);

    return navq95_scmi_txn(4u + 16u);
}

#define SCMI_PINCTRL_TYPE_DAISY_ID  194u
#define SCMI_PINCTRL_TYPE_DAISY_CFG 195u
#define SCMI_PINCTRL_DAISYREG_OFF   0x408u

int navq95_scmi_pinmux_daisy(uint32_t muxReg, uint32_t muxMode, uint32_t sion,
                             uint32_t daisyReg, uint32_t daisyVal,
                             uint32_t padVal)
{
    uint32_t identifier = (muxReg - IOMUXC_BASE) / 4u;
    uint32_t daisyId    = (daisyReg - (IOMUXC_BASE + SCMI_PINCTRL_DAISYREG_OFF)) / 4u;
    uint32_t attributes = (4u << 2);
    uint32_t muxVal = (muxMode & 0x7u) | ((sion & 0x1u) << 4);

    navq95_w32(SMT_MSG_HEADER, navq95_scmi_header(SCMI_PROTO_PINCTRL, SCMI_MSG_PINCTRL_SET));
    navq95_w32(SMT_PAYLOAD + 0x00u, identifier);
    navq95_w32(SMT_PAYLOAD + 0x04u, 0xFFFFFFFFu);
    navq95_w32(SMT_PAYLOAD + 0x08u, attributes);
    navq95_w32(SMT_PAYLOAD + 0x0Cu, SCMI_PINCTRL_TYPE_MUX);
    navq95_w32(SMT_PAYLOAD + 0x10u, muxVal);
    navq95_w32(SMT_PAYLOAD + 0x14u, SCMI_PINCTRL_TYPE_DAISY_ID);
    navq95_w32(SMT_PAYLOAD + 0x18u, daisyId);
    navq95_w32(SMT_PAYLOAD + 0x1Cu, SCMI_PINCTRL_TYPE_DAISY_CFG);
    navq95_w32(SMT_PAYLOAD + 0x20u, daisyVal);
    navq95_w32(SMT_PAYLOAD + 0x24u, SCMI_PINCTRL_TYPE_CONFIG);
    navq95_w32(SMT_PAYLOAD + 0x28u, padVal);

    return navq95_scmi_txn(4u + 12u + 4u * 8u);
}

// Pick the OSR/SBR pair with the smallest baud error; OSR 4..7 needs both-edge sampling.
static uint32_t navq95_lpuart_baudval(uint32_t clkHz, uint32_t baud)
{
    uint32_t bestOsr = 16u;
    uint32_t bestSbr = 1u;
    uint32_t bestDiff = baud;

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

static void navq95_lpuart_init(uint32_t base, uint32_t clkHz, uint32_t baud)
{
    volatile uint32_t *global = (volatile uint32_t *)(base + LPUART_GLOBAL_OFFSET);
    volatile uint32_t *fifo   = (volatile uint32_t *)(base + LPUART_FIFO_OFFSET);
    volatile uint32_t *modir  = (volatile uint32_t *)(base + LPUART_MODIR_OFFSET);
    volatile uint32_t *baudr  = (volatile uint32_t *)(base + LPUART_BAUD_OFFSET);
    volatile uint32_t *statr  = (volatile uint32_t *)(base + LPUART_STAT_OFFSET);
    volatile uint32_t *ctrl   = (volatile uint32_t *)(base + LPUART_CTRL_OFFSET);

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
    *baudr = navq95_lpuart_baudval(clkHz, baud);

    *statr = LPUART_STAT_W1C;

    *ctrl = LPUART_CTRL_RE | LPUART_CTRL_TE;
    (void)*ctrl;
}

void SystemInit(void)
{

    // The build is hard-float; the FPU is disabled out of reset.
    SCB->CPACR |= (3u << (10 * 2)) | (3u << (11 * 2));
    __DSB();
    __ISB();

}

void SystemCoreClockUpdate(void)
{
}

// The core clock is set by the System Manager.
uint32_t SystemCoreClock = 800000000U;

static int navq95_lpuart2_iomux(void);

void systemInit(void)
{
    cycleCounterInit();
    navq95_timebaseInit();

    (void)navq95_lpuart2_iomux();
    navq95_lpuart_init(NAVQ95_CONSOLE_LPUART_BASE, NAVQ95_LPUART_CLK_HZ,
                       NAVQ95_CONSOLE_BAUD);
    navq95_uart_rx_irq_enable();

    // Let the freshly muxed TX line settle to idle before the banner.
    delayMicroseconds(2000);
    for (int i = 0; i < 8; i++) {
        navq95_lowputc('\0');
    }
    navq95_lowdrain();
    delayMicroseconds(1000);

    navq95_lowputs("\nBetaflight NAVQ95 (NXP i.MX 95, Cortex-M7)\n");
    navq95_lowputs("Type '#' then Enter to open the CLI.\n");
    navq95_lowdrain();
}

// On success the System Manager resets this core before the call returns.
static int navq95_scmi_system_reset(void)
{
    navq95_w32(SMT_MSG_HEADER,
               navq95_scmi_header(SCMI_PROTO_SYSTEM, SCMI_MSG_SYSTEM_POWER_STATE_SET));
    navq95_w32(SMT_PAYLOAD + 0x00u, SCMI_SYSTEM_FORCEFUL);
    navq95_w32(SMT_PAYLOAD + 0x04u, SCMI_SYSTEM_WARM_RESET);
    return navq95_scmi_txn(4u + 2u * 4u);
}

extern uint32_t __isr_vector[];

void systemReset(void)
{
    __disable_irq();

    if (navq95_scmi_system_reset() == 0) {
        for (;;) { }
    }

    // Fallback if the System Manager refuses: jump to our own reset handler.
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

    systemReset();
}

/*
 * The console is exposed as the VCP serial port so MSP and the CLI run on it.
 * TX is polled; RX comes from the ring above.
 */
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
    return 256u;
}

static uint8_t navq95_serRead(serialPort_t *instance)
{
    (void)instance;
    int c = navq95_rx_pop();
    return (c < 0) ? 0u : (uint8_t)c;
}

static void navq95_serSetBaud(serialPort_t *instance, uint32_t baudRate)
{
    (void)instance; (void)baudRate;
}

static bool navq95_serTxEmpty(const serialPort_t *instance)
{
    (void)instance;
    return true;
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
    p->identifier   = SERIAL_PORT_USB_VCP;

    return p;
}

uint32_t usbVcpGetBaudRate(struct serialPort_s *instance)
{
    (void)instance;
    return 115200u;
}

uint8_t usbVcpIsConnected(void)
{
    return 1;
}

uint8_t usbVcpIsActive(void)
{
    return 1;
}

static int navq95_lpuart2_iomux(void)
{
    int rxs = navq95_scmi_pinmux(IOMUXC_MUX_UART2_RX, 0u, 1u,
                                 IOMUXC_PAD_UART2_RX, IOMUXC_PAD_LPUART_RXVAL);
    int txs = navq95_scmi_pinmux(IOMUXC_MUX_UART2_TX, 0u, 0u,
                                 IOMUXC_PAD_UART2_TX, IOMUXC_PAD_LPUART_TXVAL);
    return rxs ? rxs : txs;
}

void EXTIInit(void)            { }
void EXTIRelease(IO_t io)      { (void)io; }

bool isMPUSoftReset(void)      { return false; }
void unusedPinsInit(void)      { }

void dbgPinHi(int index)       { (void)index; }
void dbgPinLo(int index)       { (void)index; }

void beeperPwmInit(const ioTag_t tag, uint16_t frequency) { (void)tag; (void)frequency; }
void pwmWriteBeeper(bool onoffBeep)                        { (void)onoffBeep; }

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

bool isDshotBitbangActive(const motorDevConfig_t *motorConfig) { (void)motorConfig; return false; }
dshotBitbangStatus_e dshotBitbangGetStatus(void)              { return DSHOT_BITBANG_STATUS_OK; }

void ws2811LedStripInit(ioTag_t ioTag, ledStripFormatRGB_e ledFormat) { (void)ioTag; (void)ledFormat; }
void ws2811LedStripStartTransfer(void)                                { }
void ws2811LedStripUpdateTransferBuffer(const rgbColor24bpp_t *color, unsigned ledIndex)
{
    (void)color; (void)ledIndex;
}

bool transponderIrInit(const ioTag_t ioTag, const transponderProvider_e provider)
{
    (void)ioTag; (void)provider; return false;
}
void transponderIrDisable(void)                   { }
bool transponderIrIsReady(void)                   { return false; }
void transponderIrTransmit(void)                  { }
void transponderIrUpdateData(const uint8_t *data) { (void)data; }

void expressLrsTimerEnableIRQs(const struct timerHardware_s *timer __attribute__((unused))) { }

uint32_t dmaGetDataLength(dmaResource_t *ref)  { (void)ref; return 0; }
const char *dmaGetDisplayString(void)          { return ""; }
int dmaGetHandlerCount(void)                   { return 0; }

uint16_t adcGetValue(adcSource_e source __attribute__((unused))) { return 0; }

bool dshotPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig)
{
    (void)device; (void)motorConfig; return false;
}
void servoDevInit(const servoDevConfig_t *servoConfig) { (void)servoConfig; }
void servoWrite(uint8_t index, float value)       { (void)index; (void)value; }

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

void EXTIHandlerInit(extiCallbackRec_t *cb, extiHandlerCallback *fn) { (void)cb; (void)fn; }
void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, ioConfig_t config, extiTrigger_t trigger) { (void)io; (void)cb; (void)irqPriority; (void)config; (void)trigger; }
void EXTIEnable(IO_t io) { (void)io; }
void EXTIDisable(IO_t io) { (void)io; }

uint32_t persistentObjectRead(persistentObjectId_e id) { (void)id; return 0; }
void persistentObjectWrite(persistentObjectId_e id, uint32_t value) { (void)id; (void)value; }

// Defined by the DShot driver on other platforms.
bool useDshotTelemetry = false;
#ifdef USE_DSHOT_TELEMETRY
dshotTelemetryCycleCounters_t dshotDMAHandlerCycleCounters;
#endif

void timerInit(void) { }
void debugInit(void) { }

bool ws2811LedStripHardwareInit(void) { return false; }

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

// drivers/usb_io.c is not built; the console stands in for the USB cable.
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
