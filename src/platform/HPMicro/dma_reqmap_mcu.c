/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * DMA channel request mapping tables and lookup functions for HPMicro peripherals.
 */

#include <stdint.h>

#include "platform.h"

#ifdef USE_DMA_SPEC

#include "timer_def.h"
#include "drivers/bus_spi.h"
#include "drivers/dma_reqmap.h"
#include "dma_hpmicro.h"
#include "hpm_dma_reqmap.h"

typedef struct dmaPeripheralMapping_s {
    dmaPeripheral_e device;
    uint8_t index;
    dmaChannelSpec_t channelSpec[MAX_PERIPHERAL_DMA_OPTIONS];
} dmaPeripheralMapping_t;

typedef struct dmaPeripheralMappingHPM_s {
    hpmDmaPeripheral_e device;
    uint8_t index;
    dmaChannelSpec_t channelSpec[MAX_PERIPHERAL_DMA_OPTIONS];
} dmaPeripheralMappingHPM_t;

// DMA(d, c, mux): packs (controller, channel, dmamux_src) into a single channelSpec.
//   d   = DMA controller (1 = HDMA, 2 = XDMA)
//   c   = physical channel number (0..7)
//   mux = DMAMUX source ID (HPM_DMA_SRC_xxx)
#define DMA(d, c, mux) { DMA_CODE(d, c, 0), HPM_DMA_RESOURCE(d, c), (mux) }

// Helpers: same controller and mux, varying only the channel number
#define DMA_OPT1(d, mux, c0)              DMA(d, c0, mux)
#define DMA_OPT2(d, mux, c0, c1)          DMA(d, c0, mux), DMA(d, c1, mux)
#define DMA_OPT3(d, mux, c0, c1, c2)      DMA(d, c0, mux), DMA(d, c1, mux), DMA(d, c2, mux)
#define DMA_OPT4(d, mux, c0, c1, c2, c3)  DMA(d, c0, mux), DMA(d, c1, mux), DMA(d, c2, mux), DMA(d, c3, mux)

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
#ifdef USE_SPI
#ifdef HPM6750
    { DMA_PERIPH_SPI_SDO, SPIDEV_1, { DMA_OPT4(1, HPM_DMA_SRC_SPI0_TX, 4, 5, 6, 7)} },
    { DMA_PERIPH_SPI_SDI, SPIDEV_1, { DMA_OPT4(1, HPM_DMA_SRC_SPI0_RX, 4, 5, 6, 7)} },
    { DMA_PERIPH_SPI_SDO, SPIDEV_2, { DMA_OPT4(1, HPM_DMA_SRC_SPI1_TX, 4, 5, 6, 7)} },
    { DMA_PERIPH_SPI_SDI, SPIDEV_2, { DMA_OPT4(1, HPM_DMA_SRC_SPI1_RX, 4, 5, 6, 7)} },
    { DMA_PERIPH_SPI_SDO, SPIDEV_3, { DMA_OPT4(1, HPM_DMA_SRC_SPI2_TX, 4, 5, 6, 7)} },
    { DMA_PERIPH_SPI_SDI, SPIDEV_3, { DMA_OPT4(1, HPM_DMA_SRC_SPI2_RX, 4, 5, 6, 7)} },
    { DMA_PERIPH_SPI_SDO, SPIDEV_4, { DMA_OPT4(1, HPM_DMA_SRC_SPI3_TX, 4, 5, 6, 7)} },
    { DMA_PERIPH_SPI_SDI, SPIDEV_4, { DMA_OPT4(1, HPM_DMA_SRC_SPI3_RX, 4, 5, 6, 7)} },
#endif
#ifdef HPM6360
    { DMA_PERIPH_SPI_SDO, SPIDEV_1, { DMA_OPT3(1, HPM_DMA_SRC_SPI0_TX, 5, 6, 7)} },
    { DMA_PERIPH_SPI_SDI, SPIDEV_1, { DMA_OPT1(1, HPM_DMA_SRC_SPI0_RX, 0)} },
    { DMA_PERIPH_SPI_SDO, SPIDEV_2, { DMA_OPT3(1, HPM_DMA_SRC_SPI1_TX, 5, 6, 7)} },
    { DMA_PERIPH_SPI_SDI, SPIDEV_2, { DMA_OPT3(1, HPM_DMA_SRC_SPI1_RX, 5, 6, 7)} },
    { DMA_PERIPH_SPI_SDO, SPIDEV_3, { DMA_OPT3(1, HPM_DMA_SRC_SPI2_TX, 5, 6, 7)} },
    { DMA_PERIPH_SPI_SDI, SPIDEV_3, { DMA_OPT3(1, HPM_DMA_SRC_SPI2_RX, 5, 6, 7)} },
    { DMA_PERIPH_SPI_SDO, SPIDEV_4, { DMA_OPT3(1, HPM_DMA_SRC_SPI3_TX, 5, 6, 7)} },
    { DMA_PERIPH_SPI_SDI, SPIDEV_4, { DMA_OPT3(1, HPM_DMA_SRC_SPI3_RX, 5, 6, 7)} },
#endif
#endif                          // USE_SPI
};

static const dmaPeripheralMappingHPM_t dmaPeripheralMappingHPM[] = {
#ifdef USE_DSHOT
    /*
     * HPMicro DShot/Telemetry DMA channel specs.
     *
     * PWM output channels are indexed by motor index (0..3).
     * GPTMR capture channels are indexed by GPTMR instance (0..5).
     */
#ifdef HPM6360
    { HPM_DMA_PERIPH_PWM_OUT, 0, { DMA(2, 0, HPM_DMA_SRC_MOT1_0)} },
    { HPM_DMA_PERIPH_PWM_OUT, 1, { DMA(2, 1, HPM_DMA_SRC_MOT1_1)} },
    { HPM_DMA_PERIPH_PWM_OUT, 2, { DMA(2, 2, HPM_DMA_SRC_MOT0_2)} },
    { HPM_DMA_PERIPH_PWM_OUT, 3, { DMA(2, 3, HPM_DMA_SRC_MOT0_3)} },

    { HPM_DMA_PERIPH_GPTMR_CAP_NEG, 2, { DMA(1, 4, HPM_DMA_SRC_GPTMR2_3)} },
    { HPM_DMA_PERIPH_GPTMR_CAP_POS, 2, { DMA(2, 4, HPM_DMA_SRC_GPTMR2_2)} },
    { HPM_DMA_PERIPH_GPTMR_CAP_NEG, 3, { DMA(1, 1, HPM_DMA_SRC_GPTMR3_3)} },
    { HPM_DMA_PERIPH_GPTMR_CAP_POS, 3, { DMA(2, 5, HPM_DMA_SRC_GPTMR3_2)} },
    { HPM_DMA_PERIPH_GPTMR_CAP_NEG, 0, { DMA(1, 2, HPM_DMA_SRC_GPTMR0_3)} },
    { HPM_DMA_PERIPH_GPTMR_CAP_POS, 0, { DMA(2, 6, HPM_DMA_SRC_GPTMR0_2)} },
    { HPM_DMA_PERIPH_GPTMR_CAP_NEG, 1, { DMA(1, 3, HPM_DMA_SRC_GPTMR1_3)} },
    { HPM_DMA_PERIPH_GPTMR_CAP_POS, 1, { DMA(2, 7, HPM_DMA_SRC_GPTMR1_2)} },
#endif
#ifdef HPM6750
    { HPM_DMA_PERIPH_PWM_OUT, 0, { DMA(2, 0, HPM_DMA_SRC_MOT1_0)} },
    { HPM_DMA_PERIPH_PWM_OUT, 1, { DMA(2, 1, HPM_DMA_SRC_MOT1_1)} },
    { HPM_DMA_PERIPH_PWM_OUT, 2, { DMA(2, 2, HPM_DMA_SRC_MOT1_2)} },
    { HPM_DMA_PERIPH_PWM_OUT, 3, { DMA(2, 3, HPM_DMA_SRC_MOT1_3)} },

    { HPM_DMA_PERIPH_GPTMR_CAP_NEG, 2, { DMA(1, 0, HPM_DMA_SRC_GPTMR2_3)} },
    { HPM_DMA_PERIPH_GPTMR_CAP_POS, 2, { DMA(2, 4, HPM_DMA_SRC_GPTMR2_2)} },
    { HPM_DMA_PERIPH_GPTMR_CAP_NEG, 3, { DMA(1, 1, HPM_DMA_SRC_GPTMR3_3)} },
    { HPM_DMA_PERIPH_GPTMR_CAP_POS, 3, { DMA(2, 5, HPM_DMA_SRC_GPTMR3_2)} },
    { HPM_DMA_PERIPH_GPTMR_CAP_NEG, 4, { DMA(1, 2, HPM_DMA_SRC_GPTMR4_3)} },
    { HPM_DMA_PERIPH_GPTMR_CAP_POS, 4, { DMA(2, 6, HPM_DMA_SRC_GPTMR4_2)} },
    { HPM_DMA_PERIPH_GPTMR_CAP_NEG, 5, { DMA(1, 3, HPM_DMA_SRC_GPTMR5_3)} },
    { HPM_DMA_PERIPH_GPTMR_CAP_POS, 5, { DMA(2, 7, HPM_DMA_SRC_GPTMR5_2)} },
#endif
#endif                          // USE_DSHOT
};

#undef DMA
#undef DMA_OPT1
#undef DMA_OPT2
#undef DMA_OPT3
#undef DMA_OPT4

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    if (opt < 0 || opt >= MAX_PERIPHERAL_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0; i < ARRAYLEN(dmaPeripheralMapping); i++) {
        const dmaPeripheralMapping_t *periph = &dmaPeripheralMapping[i];

        if (periph->device == device && periph->index == index && periph->channelSpec[opt].ref) {
            return &periph->channelSpec[opt];
        }
    }

    return NULL;
}

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheralHPM(hpmDmaPeripheral_e device, uint8_t index, int8_t opt)
{
    if (opt < 0 || opt >= MAX_PERIPHERAL_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0; i < ARRAYLEN(dmaPeripheralMappingHPM); i++) {
        const dmaPeripheralMappingHPM_t *periph = &dmaPeripheralMappingHPM[i];

        if (periph->device == device && periph->index == index && periph->channelSpec[opt].ref) {
            return &periph->channelSpec[opt];
        }
    }

    return NULL;
}

FAST_CODE const dmaChannelSpec_t *hpmDmaGetPwmOutChannelSpec(uint8_t motorIndex)
{
    return dmaGetChannelSpecByPeripheralHPM(HPM_DMA_PERIPH_PWM_OUT, motorIndex, 0);
}

FAST_CODE const dmaChannelSpec_t *hpmDmaGetGptmrCapChannelSpec(uint8_t gptmrIndex, bool posEdge)
{
    hpmDmaPeripheral_e device = (posEdge) ? HPM_DMA_PERIPH_GPTMR_CAP_POS : HPM_DMA_PERIPH_GPTMR_CAP_NEG;
    return dmaGetChannelSpecByPeripheralHPM(device, gptmrIndex, 0);
}

dmaoptValue_t dmaoptByTag(ioTag_t ioTag)
{
    UNUSED(ioTag);

    return DMA_OPT_UNUSED;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(timerResource_t *tim, uint8_t channel, dmaoptValue_t dmaopt)
{
    (void) channel;
    (void) tim;
    (void) dmaopt;
    return NULL;
}

#endif                          // USE_DMA_SPEC
