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
 * bus_spi_navq95.c — i.MX 95 LPSPI platform layer for the Betaflight SPI core.
 *
 * Betaflight's common drivers/bus_spi.c provides the high-level API
 * (spiReadReg / spiWriteReg / spiSequence / ...) once USE_SPI is defined; this
 * file supplies the i.MX 95 hardware hooks (register-level LPSPI, polled, no
 * DMA). It is the SPI sibling of serial_uart_navq95.c (UART) and
 * bus_i2c_navq95.c (I2C): a self-contained platform driver that reaches the
 * System Manager through the shared SCMI helpers in platform_stubs.c (declared
 * in navq95_scmi.h) to ungate the LPSPI functional clock and mux its pads (the
 * M7 owns neither the CCM nor the IOMUXC).
 *
 * Bus: LPSPI1 (SPI1 == 0x44360000, platform.h) carries the ICM-45686 6-axis
 * gyro/acc. CS is a software GPIO (GPIO1_IO11 = PA11), driven by the common
 * polled path; the i.MX 95 hardware PCS is not used. The ICM-45686 wants SPI
 * mode 3 (CPOL=1, CPHA=1). SCK bring-up rate ~6 MHz (24 MHz src / (2^0 * (2+2))).
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_SPI

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"

#include "navq95_scmi.h"

/* IOMUXC mux/pad registers are absolute addresses; the SM identifies a pin from
 * (muxReg - IOMUXC_BASE) / 4. See navq95_scmi.h. */
#ifndef IOMUXC_BASE
#define IOMUXC_BASE            0x443C0000u
#endif

/* ── i.MX 95 LPSPI register layout (NuttX arch/arm/src/imx9/hardware/
 * imx9_lpspi.h). CS is a software GPIO (not hardware PCS); the common polled
 * path toggles it. The LPSPI functional clock is ungated over SCMI; the pads
 * are muxed over SCMI pinctrl (same path as the console). ───────────────── */
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

#endif /* USE_SPI */
