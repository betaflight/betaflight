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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPI

#define IS_CCM(p) (((uint32_t)p & 0xffff0000) == 0x10000000)

#include "common/maths.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "platform/rcc.h"

// Use DMA if possible if this many bytes are to be transferred
#define SPI_DMA_THRESHOLD 8

static spi_init_type defaultInit = {
    .master_slave_mode = SPI_MODE_MASTER,
    .transmission_mode = SPI_TRANSMIT_FULL_DUPLEX,
    .frame_bit_num = SPI_FRAME_8BIT,
    .cs_mode_selection = SPI_CS_SOFTWARE_MODE,
    .first_bit_transmission = SPI_FIRST_BIT_MSB,
    .mclk_freq_division = SPI_MCLK_DIV_8,
    .clock_polarity = SPI_CLOCK_POLARITY_HIGH,
    .clock_phase = SPI_CLOCK_PHASE_2EDGE
};

static uint16_t spiDivisorToBRbits(spi_type  *instance, uint16_t divisor)
{
    UNUSED(instance);
    divisor = constrain(divisor, 2, 256);
    return (ffs(divisor) - 2) << 3; // SPI_CR1_BR_Pos

}

static void spiSetDivisorBRreg(spi_type *instance, uint16_t divisor)
{
#define BR_BITS ((BIT(5) | BIT(4) | BIT(3)))
    const uint16_t tempRegister = (instance->ctrl1 & ~BR_BITS);
    instance->ctrl1 = tempRegister | spiDivisorToBRbits(instance, divisor);
#undef BR_BITS
}

void spiInitDevice(SPIDevice device)
{
    spiDevice_t *spi = &(spiDevice[device]);

    if (!spi->dev) {
        return;
    }

    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);

    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK,  RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_SDI, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_SDO, RESOURCE_INDEX(device));

    IOConfigGPIOAF(IOGetByTag(spi->sck),  SPI_IO_AF_SCK_CFG_HIGH, spi->sckAF);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_SDI_CFG, spi->misoAF);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->mosiAF);

    // Init SPI hardware
    spi_i2s_reset(spi->dev);

    spi_i2s_dma_transmitter_enable(spi->dev, TRUE);
    spi_i2s_dma_receiver_enable(spi->dev, TRUE);

    spi_init(spi->dev, &defaultInit);
    spi_crc_polynomial_set(spi->dev, 7);

    spi_enable(spi->dev, TRUE);
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    dma_init_type *dmaInitTx = bus->dmaInitTx;

    dma_default_para_init(dmaInitTx);

    dmaInitTx->direction=DMA_DIR_MEMORY_TO_PERIPHERAL;
    dmaInitTx->loop_mode_enable=FALSE;
    dmaInitTx->peripheral_base_addr=(uint32_t)&bus->busType_u.spi.instance->dt ;
    dmaInitTx->priority =DMA_PRIORITY_LOW;
    dmaInitTx->peripheral_inc_enable =FALSE;
    dmaInitTx->peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dmaInitTx->memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;

    if (bus->dmaRx) {
        dma_init_type *dmaInitRx = bus->dmaInitRx;

        dma_default_para_init(dmaInitRx);

        dmaInitRx->direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
        dmaInitRx->loop_mode_enable = FALSE;
        dmaInitRx->peripheral_base_addr = (uint32_t)&bus->busType_u.spi.instance->dt;
        dmaInitRx->priority = DMA_PRIORITY_LOW;
        dmaInitRx->peripheral_inc_enable = FALSE;
        dmaInitRx->peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;

    }
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    DMA_ARCH_TYPE *streamRegs = (DMA_ARCH_TYPE *)descriptor->ref;
    xDMA_Cmd(streamRegs, FALSE);
    DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
}

bool spiInternalReadWriteBufPolled(spi_type *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
    uint8_t b;

    while (len--) {
        b = txData ? *(txData++) : 0xFF;

        while (spi_i2s_flag_get(instance, SPI_I2S_TDBE_FLAG) == RESET);
        spi_i2s_data_transmit(instance, b);

        while (spi_i2s_flag_get(instance, SPI_I2S_RDBF_FLAG) == RESET);
        b = (uint8_t)spi_i2s_data_receive(instance);

        if (rxData) {
            *(rxData++) = b;
        }
    }

    return true;
}

void spiInternalInitStream(const extDevice_t *dev, volatile busSegment_t *segment)
{
    STATIC_DMA_DATA_AUTO uint8_t dummyTxByte = 0xff;
    STATIC_DMA_DATA_AUTO uint8_t dummyRxByte;
    busDevice_t *bus = dev->bus;
    int len = segment->len;

    uint8_t *txData = segment->u.buffers.txData;
    dma_init_type  *dmaInitTx = bus->dmaInitTx;

    if (txData) {
        dmaInitTx->memory_base_addr = (uint32_t)txData;
        dmaInitTx->memory_inc_enable =TRUE;
    } else {
        dummyTxByte = 0xff;
        dmaInitTx->memory_base_addr = (uint32_t)&dummyTxByte;
        dmaInitTx->memory_inc_enable =FALSE;
    }
    dmaInitTx->buffer_size =len;

    if (dev->bus->dmaRx) {
        uint8_t *rxData = segment->u.buffers.rxData;
        dma_init_type *dmaInitRx = bus->dmaInitRx;

        if (rxData) {
            dmaInitRx->memory_base_addr= (uint32_t)rxData;
            dmaInitRx->memory_inc_enable = TRUE;
        } else {
            dmaInitRx->memory_base_addr = (uint32_t)&dummyRxByte;
            dmaInitRx->memory_inc_enable = FALSE;
        }

        dmaInitRx->buffer_size = len;
    }
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    dmaChannelDescriptor_t *dmaTx = dev->bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = dev->bus->dmaRx;
    DMA_ARCH_TYPE *streamRegsTx = (DMA_ARCH_TYPE *)dmaTx->ref;

    // Wait for any ongoing transmission to complete
    while (spi_i2s_flag_get(dev->bus->busType_u.spi.instance, SPI_I2S_BF_FLAG) == SET);

    if (dmaRx) {
        DMA_ARCH_TYPE *streamRegsRx = (DMA_ARCH_TYPE *)dmaRx->ref;

        // Use the correct callback argument
        dmaRx->userParam = (uint32_t)dev;

        // Clear transfer flags
        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
        DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        // Disable streams to enable update
        xDMA_Cmd(streamRegsTx, FALSE);
        xDMA_Cmd(streamRegsRx, FALSE);

        xDMA_ITConfig(streamRegsRx, DMA_IT_TCIF, TRUE);

        // Update streams
        xDMA_Init(streamRegsTx, dev->bus->dmaInitTx);
        xDMA_Init(streamRegsRx, dev->bus->dmaInitRx);

        // Enable streams
        xDMA_Cmd(streamRegsRx, TRUE);
        xDMA_Cmd(streamRegsTx, TRUE);

        /* Enable the receiver before the transmitter to ensure that no bits are missed on reception. An interrupt between
         * the transmitter and receiver being enabled can otherwise cause a hang.
         */
        spi_i2s_dma_receiver_enable(dev->bus->busType_u.spi.instance, TRUE);
        spi_i2s_dma_transmitter_enable(dev->bus->busType_u.spi.instance, TRUE);

    } else {
        // Use the correct callback argument
        dmaTx->userParam = (uint32_t)dev;

        // Clear transfer flags
        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        // Disable stream to enable update
        xDMA_Cmd(streamRegsTx, FALSE);

        // Update stream
        xDMA_Init(streamRegsTx, dev->bus->dmaInitTx);

        // Enable stream
        xDMA_Cmd(streamRegsTx, TRUE);
        xDMA_ITConfig(streamRegsTx, DMA_IT_TCIF, TRUE);

        /* Enable the SPI DMA Tx request */
        spi_i2s_dma_transmitter_enable(dev->bus->busType_u.spi.instance, TRUE);
    }
}

void spiInternalStopDMA (const extDevice_t *dev)
{
    dmaChannelDescriptor_t *dmaTx = dev->bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = dev->bus->dmaRx;
    spi_type *instance = dev->bus->busType_u.spi.instance;
    DMA_ARCH_TYPE *streamRegsTx = (DMA_ARCH_TYPE *)dmaTx->ref;

    if (dmaRx) {
        DMA_ARCH_TYPE *streamRegsRx = (DMA_ARCH_TYPE *)dmaRx->ref;

        // Disable streams
        xDMA_Cmd(streamRegsTx, FALSE);
        xDMA_Cmd(streamRegsRx, FALSE);

        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
        DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        spi_i2s_dma_transmitter_enable(instance, FALSE);
        spi_i2s_dma_receiver_enable(instance, FALSE);
    } else {
        // Ensure the current transmission is complete
        while (spi_i2s_flag_get(instance, SPI_I2S_BF_FLAG));

        // Drain the RX buffer
        while (spi_i2s_flag_get(instance, SPI_I2S_RDBF_FLAG)) {
            instance->dt;
        }

        // Disable stream
        xDMA_Cmd(streamRegsTx, FALSE);
        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        spi_i2s_dma_transmitter_enable(instance, FALSE);
    }
}

// DMA transfer setup and start
void spiSequenceStart(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    spi_type *instance = bus->busType_u.spi.instance;
    bool dmaSafe = dev->useDMA;
    uint32_t xferLen = 0;
    uint32_t segmentCount = 0;

    dev->bus->initSegment = true;

    spi_enable(instance, FALSE);

    // Switch bus speed
    if (dev->busType_u.spi.speed != bus->busType_u.spi.speed) {
        spiSetDivisorBRreg(bus->busType_u.spi.instance, dev->busType_u.spi.speed);
        bus->busType_u.spi.speed = dev->busType_u.spi.speed;
    }

    if (dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge) {
        if (dev->busType_u.spi.leadingEdge) {
            instance->ctrl1_bit.clkpol = SPI_CLOCK_POLARITY_LOW;
            instance->ctrl1_bit.clkpha = SPI_CLOCK_PHASE_1EDGE;
        } else {
            instance->ctrl1_bit.clkpol = SPI_CLOCK_POLARITY_HIGH;
            instance->ctrl1_bit.clkpha = SPI_CLOCK_PHASE_2EDGE;
        }
        bus->busType_u.spi.leadingEdge = dev->busType_u.spi.leadingEdge;
    }

    spi_enable(instance, TRUE);

    // Count segments
    for (busSegment_t *checkSegment = (busSegment_t *)bus->curSegment; checkSegment->len; checkSegment++) {
        // Note that these counts are only valid if dmaSafe is true
        segmentCount++;
        xferLen += checkSegment->len;
    }

    // Use DMA if possible
    // If there are more than one segments, or a single segment with negateCS negated in the list terminator then force DMA irrespective of length
    if (bus->useDMA && dmaSafe && ((segmentCount > 1) ||
                                   (xferLen >= SPI_DMA_THRESHOLD) ||
                                   !bus->curSegment[segmentCount].negateCS)) {
        spiProcessSegmentsDMA(dev);
    } else {
        spiProcessSegmentsPolled(dev);
    }
}
#endif
