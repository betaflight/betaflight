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
 * i.MX 95 LPSPI back-end for the common SPI driver: polled register access, no
 * DMA. LPSPI1 carries the gyro; chip select is a GPIO driven by the common code.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_SPI

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"

#include "navq95_scmi.h"

#ifndef IOMUXC_BASE
#define IOMUXC_BASE            0x443C0000u
#endif

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

#define NAVQ95_LPSPI1_BASE     0x44360000u
#define NAVQ95_LPSPI1_CLK_ROOT 8u
#define NAVQ95_LPSPI1_SRC_HZ   24000000u

#define IOMUXC_MUX_LPSPI1_SCK  (IOMUXC_BASE + 0x01F4u)
#define IOMUXC_PAD_LPSPI1_SCK  (IOMUXC_BASE + 0x03F8u)
#define IOMUXC_MUX_LPSPI1_SOUT (IOMUXC_BASE + 0x01F8u)
#define IOMUXC_PAD_LPSPI1_SOUT (IOMUXC_BASE + 0x03FCu)
#define IOMUXC_MUX_LPSPI1_SIN  (IOMUXC_BASE + 0x01F0u)
#define IOMUXC_PAD_LPSPI1_SIN  (IOMUXC_BASE + 0x03F4u)
// Chip select is the pad in GPIO mode.
#define IOMUXC_MUX_LPSPI1_CS   (IOMUXC_BASE + 0x01ECu)
#define IOMUXC_PAD_LPSPI1_CS   (IOMUXC_BASE + 0x03F0u)
#define IOMUXC_LPSPI1_ALT      0x2u
#define IOMUXC_LPSPI1_CS_ALT   0x5u

// Outputs: slow slew, drive strength 4. MISO: pull-up and hysteresis.
#define IOMUXC_PAD_SPI_OUT     0x0Fu
#define IOMUXC_PAD_SPI_IN      0x0900u

static int navq95_lpspi1_padmux(void)
{
    int s = 0, r;
    r = navq95_scmi_pinmux(IOMUXC_MUX_LPSPI1_SCK,  IOMUXC_LPSPI1_ALT, 0u,
                           IOMUXC_PAD_LPSPI1_SCK,  IOMUXC_PAD_SPI_OUT); if (!s) s = r;
    r = navq95_scmi_pinmux(IOMUXC_MUX_LPSPI1_SOUT, IOMUXC_LPSPI1_ALT, 0u,
                           IOMUXC_PAD_LPSPI1_SOUT, IOMUXC_PAD_SPI_OUT); if (!s) s = r;
    r = navq95_scmi_pinmux(IOMUXC_MUX_LPSPI1_SIN,  IOMUXC_LPSPI1_ALT, 1u,
                           IOMUXC_PAD_LPSPI1_SIN,  IOMUXC_PAD_SPI_IN);  if (!s) s = r;
    r = navq95_scmi_pinmux(IOMUXC_MUX_LPSPI1_CS,   IOMUXC_LPSPI1_CS_ALT, 0u,
                           IOMUXC_PAD_LPSPI1_CS,   IOMUXC_PAD_SPI_OUT); if (!s) s = r;
    return s;
}

// CCR may only be written while the module is disabled, TCR only while it is enabled.
static void navq95_lpspi1_configure(uint32_t sckdiv, uint32_t prescale)
{
    volatile uint32_t *cr    = (volatile uint32_t *)(NAVQ95_LPSPI1_BASE + LPSPI_CR_OFFSET);
    volatile uint32_t *cfgr1 = (volatile uint32_t *)(NAVQ95_LPSPI1_BASE + LPSPI_CFGR1_OFFSET);
    volatile uint32_t *ccr   = (volatile uint32_t *)(NAVQ95_LPSPI1_BASE + LPSPI_CCR_OFFSET);
    volatile uint32_t *fcr   = (volatile uint32_t *)(NAVQ95_LPSPI1_BASE + LPSPI_FCR_OFFSET);
    volatile uint32_t *tcr   = (volatile uint32_t *)(NAVQ95_LPSPI1_BASE + LPSPI_TCR_OFFSET);

    *cr = 0u;
    *cr = LPSPI_CR_RST;
    *cr = LPSPI_CR_RTF | LPSPI_CR_RRF;
    *cr = 0u;

    *cfgr1 = LPSPI_CFGR1_MASTER;
    *ccr   = LPSPI_CCR_SCKDIV(sckdiv);
    *fcr   = 0u;

    *cr = LPSPI_CR_MEN;
    *tcr = LPSPI_TCR_FRAMESZ(7u)
         | LPSPI_TCR_PRESCALE(prescale)
         | LPSPI_TCR_PCS(0u)
         | LPSPI_TCR_CPOL | LPSPI_TCR_CPHA;
    (void)*tcr;
}

const spiHardware_t spiHardware[SPIDEV_COUNT] = {
    {
        .device = SPIDEV_1,
        .reg    = (spiResource_t *)SPI1,
    },
};

void spiPinConfigure(const struct spiPinConfig_s *pConfig)
{
    (void)pConfig;

    extern spiDevice_t spiDevice[SPIDEV_COUNT];
    spiDevice[SPIDEV_1].dev = (spiResource_t *)SPI1;
}

void spiInitDevice(spiDevice_e device)
{
    if (device != SPIDEV_1) {
        return;
    }

    navq95_scmi_clock_enable(NAVQ95_LPSPI1_CLK_ROOT);
    navq95_lpspi1_padmux();
    // SPI mode 3 at 24 MHz / (2 + 2) = 6 MHz.
    navq95_lpspi1_configure(2u, 0u);
}

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
}

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
        (void)*sr;
    }
    return true;
}

void spiSequenceStart(const extDevice_t *dev)
{
    spiProcessSegmentsPolled(dev);
}

#endif // USE_SPI
