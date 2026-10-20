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

// SPI bus driver for HPMicro.
// Implements polled and DMA transfers for the common SPI bus layer, including
// DMA buffer cache maintenance and bus DMA setup via spiInitBusDMA().

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "hpm_spi.h"
#include "platform.h"
#ifdef USE_SPI

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/io.h"
#include "drivers/system.h"
#include "hpm_clock_drv.h"
#include "hpm_spi_drv.h"
#include "hpm_l1c_drv.h"
#include "io_hpmicro.h"
#include "dma_hpmicro.h"
#include "hpm_dma_reqmap.h"
#include "common/maths.h"
#include "common/utils.h"
#include "pg/bus_spi.h"
#include "nvic.h"

extern spiDevice_t spiDevice[SPIDEV_COUNT];
extern busDevice_t spiBusDevice[SPIDEV_COUNT];

typedef struct spiDmaDummyBuffer_s {
    uint8_t tx;
    uint8_t rx;
} spiDmaDummyBuffer_t;

static volatile DMA_DATA_ZERO_INIT spiDmaDummyBuffer_t spiDmaDummyBuffer;

STATIC_ASSERT(sizeof(((dmaChannelDescriptor_t *)0)->userParam) >= sizeof(uintptr_t),
              spiDmaUserParam_must_hold_pointer);

// Interrupt handler for SPI receive DMA completion
FAST_IRQ_HANDLER static void spiRxIrqHandler(dmaChannelDescriptor_t *descriptor)
{
    const extDevice_t *dev = (const extDevice_t *)(uintptr_t) descriptor->userParam;

    if (!dev) {
        return;
    }

    busDevice_t *bus = dev->bus;

    // Guard against DMA abort having already freed the segment list
    if (!bus->curSegment) {
        return;
    }

    if (hpmDmaGetChannelStatus(descriptor) & (DMA_CHANNEL_STATUS_ERROR | DMA_CHANNEL_STATUS_ABORT)) {
        if (bus->dmaTx) {
            dma_disable_channel(bus->dmaTx->dma, DMA_DESC_CHANNEL(bus->dmaTx));
        }
        if (bus->dmaRx) {
            dma_disable_channel(bus->dmaRx->dma, DMA_DESC_CHANNEL(bus->dmaRx));
        }
        spiInternalStopDMA(dev);
        IOHi(dev->busType_u.spi.csnPin);
        bus->curSegment = (busSegment_t *) BUS_SPI_FREE;
        spiDevice[spiDeviceByInstance(bus->busType_u.spi.instance)].errorCount++;
        return;
    }

    if (bus->curSegment->negateCS) {
        // Negate Chip Select
        IOHi(dev->busType_u.spi.csnPin);
    }

    if (l1c_dc_is_enabled() && bus->curSegment->u.buffers.rxData) {
        // Invalidate the receive buffer so the CPU sees the data written by DMA
        const uint32_t alignedStart = HPM_L1C_CACHELINE_ALIGN_DOWN((uint32_t) bus->curSegment->u.buffers.rxData);
        const uint32_t alignedEnd = HPM_L1C_CACHELINE_ALIGN_UP((uint32_t) bus->curSegment->u.buffers.rxData +
                                                               bus->curSegment->len);
        l1c_dc_invalidate(alignedStart, alignedEnd - alignedStart);
    }

    spiInternalStopDMA(dev);

    spiIrqHandler(dev);
}

static void spiConfigIOC(const spiDevice_t *spi)
{
    // Configure SCK: add LOOP_BACK to FUNC_CTL, set PAD_CTL for drive strength and pull-up
    IO_t sckIO = IOGetByTag(spi->sck);

    if (sckIO) {
        uint32_t sckIocIndex = IOCIndex(sckIO);
        HPM_IOC->PAD[sckIocIndex].FUNC_CTL |= IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
    }
}

void spiInitDevice(spiDevice_e device)
{
    spi_timing_config_t timingConfig = { 0 };
    spiDevice_t *spi = &spiDevice[device];

    if (spi->dev == NULL) {
        return;
    }
    spi_format_config_t formatConfig = { 0 };

    // All SPI instances share the same clock source and divider so that
    // spiCalculateDivider() (which has no instance context) is valid on every bus
    clock_set_source_divider(spi->rcc, clk_src_pll1_clk1, 5);
    clock_add_to_group(spi->rcc, 0);
    IOInit(IOGetByTag(spi->sck), OWNER_SPI_SCK, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_SDI, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_SDO, RESOURCE_INDEX(device));

    // HPMicro: SPI AF values are set via pinmux in board.c, not stored in spiDevice_t
    IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG, spi->sckAF);
    IOConfigGPIOAF(IOGetByTag(spi->miso), IOCFG_AF_PP, spi->misoAF);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), IOCFG_AF_PP, spi->mosiAF);

    // Configure IOC pad settings: drive strength and loop-back for SCK
    spiConfigIOC(spi);

    // Read clock after source & divider are configured
    const uint32_t spiClock = clock_get_frequency(spi->rcc);

    // Compute the highest initial SCLK not above 12 MHz. spi_master_timing_init()
    // requires the source/SCLK divider to be an even integer <= 510, so the
    // selected SCLK must divide spiClock exactly.  Round the ceiling quotient UP
    // to the next even integer: rounding down (e.g. spiClock 60 MHz: ceil 5 -> 4)
    // would select an SCLK above the 12 MHz limit.
    uint32_t divisor = (spiClock + 12000000U - 1U) / 12000000U;
    divisor += divisor & 1U;
    divisor = MAX(2U, divisor);

    while ((divisor <= 510U) && ((spiClock % divisor) != 0U)) {
        divisor += 2U;
    }
    const uint32_t spiSclkFreq = spiClock / divisor;

    spi_master_get_default_timing_config(&timingConfig);
    timingConfig.master_config.clk_src_freq_in_hz = spiClock;
    timingConfig.master_config.sclk_freq_in_hz = spiSclkFreq;
    if (status_success != spi_master_timing_init((SPI_Type *) (spi->dev), &timingConfig)) {
        failureMode(FAILURE_DEVELOPER);
    }
    /* set SPI format config for master */
    spi_master_get_default_format_config(&formatConfig);
    formatConfig.common_config.data_len_in_bits = 8;
    formatConfig.common_config.mode = spi_master_mode;
    formatConfig.common_config.cpol = spi_sclk_low_idle;
    formatConfig.common_config.cpha = spi_sclk_sampling_odd_clk_edges;
    spi_format_init((SPI_Type *) (spi->dev), &formatConfig);
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    (void) bus;
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    (void) descriptor;
}

FAST_CODE bool spiInternalReadWriteBufPolled(spiResource_t *instance, const uint8_t *txData, uint8_t *rxData,
                                             int len)
{
    static uint8_t wbuff[32] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    };
    hpm_stat_t stat;
    spi_control_config_t controlConfig = { 0 };
    SPI_Type *ptr = (SPI_Type *) instance;


    /* set SPI control config for master */
    spi_master_get_default_control_config(&controlConfig);
    controlConfig.master_config.cmd_enable = false;    /* cmd phase control for master */
    controlConfig.master_config.addr_enable = false;   /* address phase control for master */

    if (txData && rxData) {
        controlConfig.common_config.trans_mode = spi_trans_write_read_together;
        stat = spi_transfer(ptr, &controlConfig, NULL, NULL, (uint8_t *) txData, len, (uint8_t *) rxData, len);
    } else if (txData) {
        controlConfig.common_config.trans_mode = spi_trans_write_only;
        stat = spi_transfer(ptr, &controlConfig, NULL, NULL, (uint8_t *) txData, len, NULL, 0);
    } else if (rxData) {
        controlConfig.common_config.trans_mode = spi_trans_read_only;
        stat = spi_transfer(ptr, &controlConfig, NULL, NULL, NULL, 0, (uint8_t *) rxData, len);
    } else {
        // No TX/RX data: emit dummy bytes in chunks bounded by sizeof(wbuff)
        controlConfig.common_config.trans_mode = spi_trans_dummy_write;
        stat = status_success;
        while (len > 0) {
            const uint32_t chunk = MIN((uint32_t) len, (uint32_t) sizeof(wbuff));
            stat = spi_transfer(ptr, &controlConfig, NULL, NULL, (uint8_t *) wbuff, chunk, NULL, chunk);
            if (stat != status_success) {
                break;
            }
            len -= chunk;
        }
    }

    return stat == status_success;
}

FAST_CODE void spiInternalInitStream(const extDevice_t *dev, volatile busSegment_t *segment)
{
    /* spi_setup_dma_transfer() writes CMD and starts an HPM SPI master transfer,
     * so it cannot be used for the common SPI code's next-segment pre-init.
     * All segment-specific setup is instead performed by spiInternalStartDMA().
     */
    UNUSED(dev);
    UNUSED(segment);
}

void spiInitBusDMA(void)
{
    uint32_t device;

    spiDmaDummyBuffer.tx = 0xFF;
    // "fence iorw, iorw" (not just "fence w, w"): the DMA channels enabled
    // below read this buffer on behalf of the device, so the store must be
    // ordered against the subsequent MMIO setup writes.
    __asm__ volatile ("fence iorw, iorw":::"memory");

    for (device = 0; device < SPIDEV_COUNT; device++) {
        busDevice_t *bus = &spiBusDevice[device];

        if (bus->busType != BUS_TYPE_SPI) {
            // This bus is not in use
            continue;
        }

        dmaIdentifier_e dmaTxIdentifier = DMA_NONE;
        dmaIdentifier_e dmaRxIdentifier = DMA_NONE;

        int8_t txDmaopt = spiPinConfig(device)->txDmaopt;
        uint8_t txDmaoptMin = 0;
        uint8_t txDmaoptMax = MAX_PERIPHERAL_DMA_OPTIONS - 1;

        if (txDmaopt != -1) {
            txDmaoptMin = txDmaopt;
            txDmaoptMax = txDmaopt;
        }

        for (uint8_t opt = txDmaoptMin; opt <= txDmaoptMax; opt++) {
            const dmaChannelSpec_t *dmaTxChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_SDO, device, opt);

            if (dmaTxChannelSpec) {
                dmaTxIdentifier = dmaGetIdentifier(dmaTxChannelSpec->ref);

                if (!dmaAllocate(dmaTxIdentifier, OWNER_SPI_SDO, device + 1)) {
                    dmaTxIdentifier = DMA_NONE;
                    continue;
                }
                bus->dmaTx = dmaGetDescriptorByIdentifier(dmaTxIdentifier);
                dmaEnable(dmaTxIdentifier);
                dmaMuxEnable(dmaTxIdentifier, dmaTxChannelSpec->dmaMuxId);
                break;
            }
        }

        int8_t rxDmaopt = spiPinConfig(device)->rxDmaopt;
        uint8_t rxDmaoptMin = 0;
        uint8_t rxDmaoptMax = MAX_PERIPHERAL_DMA_OPTIONS - 1;

        if (rxDmaopt != -1) {
            rxDmaoptMin = rxDmaopt;
            rxDmaoptMax = rxDmaopt;
        }

        for (uint8_t opt = rxDmaoptMin; opt <= rxDmaoptMax; opt++) {
            const dmaChannelSpec_t *dmaRxChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_SDI, device, opt);

            if (dmaRxChannelSpec) {
                dmaRxIdentifier = dmaGetIdentifier(dmaRxChannelSpec->ref);
                if (!dmaAllocate(dmaRxIdentifier, OWNER_SPI_SDI, device + 1)) {
                    dmaRxIdentifier = DMA_NONE;
                    continue;
                }
                bus->dmaRx = dmaGetDescriptorByIdentifier(dmaRxIdentifier);
                dmaEnable(dmaRxIdentifier);
                dmaMuxEnable(dmaRxIdentifier, dmaRxChannelSpec->dmaMuxId);
                break;
            }
        }

        if ((dmaTxIdentifier != DMA_NONE) && (dmaRxIdentifier != DMA_NONE)) {
            // Ensure streams are disabled
            spiInternalResetStream(bus->dmaRx);
            spiInternalResetStream(bus->dmaTx);

            spiInternalResetDescriptors(bus);

            /* Note that this driver may be called both from the normal thread of execution, or from USB interrupt
             * handlers, so the DMA completion interrupt must be at a higher priority
             */
            dmaSetHandler(dmaRxIdentifier, spiRxIrqHandler, NVIC_PRIO_SPI_DMA, 0);
            // TX terminal-count is masked per transfer, but its error/abort
            // events use the same cleanup path as RX failures.
            dmaSetHandler(dmaTxIdentifier, spiRxIrqHandler, NVIC_PRIO_SPI_DMA, 0);

            bus->useDMA = true;
#ifdef USE_TX_IRQ_HANDLER
        } else if (dmaTxIdentifier != DMA_NONE) {
            // Transmit on DMA is adequate for OSD so worth having
            bus->dmaTx = dmaGetDescriptorByIdentifier(dmaTxIdentifier);
            bus->dmaRx = (dmaChannelDescriptor_t *) NULL;

            // Ensure streams are disabled
            spiInternalResetStream(bus->dmaTx);

            spiInternalResetDescriptors(bus);

            dmaSetHandler(dmaTxIdentifier, spiTxIrqHandler, NVIC_PRIO_SPI_DMA, 0);

            bus->useDMA = true;
#endif
        } else {
            if (dmaTxIdentifier != DMA_NONE) {
                hpmDmaRelease(dmaTxIdentifier);
            }
            if (dmaRxIdentifier != DMA_NONE) {
                hpmDmaRelease(dmaRxIdentifier);
            }
            // Disassociate channels from bus
            bus->dmaRx = (dmaChannelDescriptor_t *) NULL;
            bus->dmaTx = (dmaChannelDescriptor_t *) NULL;
        }
    }
}


FAST_CODE static hpm_stat_t spiTxTriggerDma(DMA_Type *dmaPtr, uint8_t chNum, SPI_Type *spiPtr, uint32_t src,
                                               bool srcFixed, uint8_t dataWidth, uint32_t size)
{
    dma_handshake_config_fixed_t config = { 0 };

    config.ch_index = chNum;
    config.dst = (uint32_t) &spiPtr->DATA;
    config.dst_fixed = true;
    config.src = src;
    config.src_fixed = srcFixed;
    config.data_width = dataWidth;
    config.size_in_byte = size;
    // TX completion is handled by RX, but TX errors/aborts must still reach
    // the shared DMA IRQ so a failed transfer is not left active silently.
    config.interrupt_mask = DMA_INTERRUPT_MASK_TERMINAL_COUNT;

    return dma_setup_handshake_fixed(dmaPtr, &config, true);
}

FAST_CODE static hpm_stat_t spiRxTriggerDma(DMA_Type *dmaPtr, uint8_t chNum, SPI_Type *spiPtr, uint32_t dst,
                                               bool dstFixed, uint8_t dataWidth, uint32_t size)
{
    dma_handshake_config_fixed_t config = { 0 };
    config.ch_index = chNum;
    config.dst = dst;
    config.dst_fixed = dstFixed;
    config.src = (uint32_t) &spiPtr->DATA;
    config.src_fixed = true;
    config.data_width = dataWidth;
    config.size_in_byte = size;
    // In the HPM SDK a set interrupt-mask bit disables that interrupt.
    config.interrupt_mask = DMA_INTERRUPT_MASK_NONE;

    return dma_setup_handshake_fixed(dmaPtr, &config, true);
}

FAST_CODE void spiInternalStartDMA(const extDevice_t *dev)
{
    dmaChannelDescriptor_t *dmaTx = dev->bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = dev->bus->dmaRx;
    hpm_stat_t stat;
    busDevice_t *bus = dev->bus;
    SPI_Type *instance = (SPI_Type *) (bus->busType_u.spi.instance);
    spi_control_config_t controlConfig = { 0 };

    // If DMA was disabled after the sequence was selected, clean up and abort.
    // This must run before dereferencing the descriptors: spiInitBusDMA()
    // leaves bus->dmaTx/dmaRx NULL when it fails to allocate channels, which
    // is exactly the case useDMA guards.
    if (!bus->useDMA || dmaTx == NULL || dmaRx == NULL) {
        IOHi(dev->busType_u.spi.csnPin);
        bus->curSegment = (busSegment_t *) BUS_SPI_FREE;
        return;
    }

    dmaRx->userParam = (uint32_t)(uintptr_t) dev;
    dmaTx->userParam = (uint32_t)(uintptr_t) dev;

    /* Keep peripheral DMA requests disabled while configuring and enabling both
     * DMA channels.  spi_setup_dma_transfer() enables the requests and starts
     * the SPI transfer after the channels are ready.
     */
    spi_disable_tx_dma(instance);
    spi_disable_rx_dma(instance);

    // Segments without a TX/RX buffer transfer a fixed dummy byte instead
    const uint8_t *txData = bus->curSegment->u.buffers.txData;
    uint8_t *rxData = bus->curSegment->u.buffers.rxData;
    const uint32_t txAddr = (txData) ? (uint32_t) txData : (uint32_t) &spiDmaDummyBuffer.tx;
    const uint32_t rxAddr = (rxData) ? (uint32_t) rxData : (uint32_t) &spiDmaDummyBuffer.rx;

    if (l1c_dc_is_enabled()) {
        if (txData) {
            /* cache writeback for sent buff */
            const uint32_t alignedStart = HPM_L1C_CACHELINE_ALIGN_DOWN(txAddr);
            const uint32_t alignedEnd = HPM_L1C_CACHELINE_ALIGN_UP(txAddr + bus->curSegment->len);

            l1c_dc_writeback(alignedStart, alignedEnd - alignedStart);
        }
        if (rxData) {
            /* drop any dirty lines covering the receive buffer so they cannot
             * evict over the DMA-written data; buffer is invalidated again on completion */
            const uint32_t rxAlignedStart = HPM_L1C_CACHELINE_ALIGN_DOWN(rxAddr);
            const uint32_t rxAlignedEnd = HPM_L1C_CACHELINE_ALIGN_UP(rxAddr + bus->curSegment->len);
            l1c_dc_flush(rxAlignedStart, rxAlignedEnd - rxAlignedStart);
        }
    }
    stat = spiRxTriggerDma(dmaRx->dma,
                              dmaRx->stream,
                              instance,
                              core_local_mem_to_sys_address(HPM_CORE0, rxAddr),
                              !rxData, DMA_TRANSFER_WIDTH_BYTE, bus->curSegment->len);
    if (stat != status_success) {
        // TX not started yet, only RX channel needs cleanup
        dma_disable_channel(dmaRx->dma, DMA_DESC_CHANNEL(dmaRx));
        spi_disable_rx_dma(instance);
        IOHi(dev->busType_u.spi.csnPin);
        bus->curSegment = (busSegment_t *) BUS_SPI_FREE;
        spiDevice[spiDeviceByInstance(bus->busType_u.spi.instance)].errorCount++;
        return;
    }
    stat = spiTxTriggerDma(dmaTx->dma,
                              dmaTx->stream,
                              instance,
                              core_local_mem_to_sys_address(HPM_CORE0, txAddr),
                              !txData, DMA_TRANSFER_WIDTH_BYTE, bus->curSegment->len);
    if (stat != status_success) {
        // RX was already triggered, both channels need cleanup
        dma_disable_channel(dmaRx->dma, DMA_DESC_CHANNEL(dmaRx));
        dma_disable_channel(dmaTx->dma, DMA_DESC_CHANNEL(dmaTx));
        spiInternalStopDMA(dev);
        IOHi(dev->busType_u.spi.csnPin);
        bus->curSegment = (busSegment_t *) BUS_SPI_FREE;
        spiDevice[spiDeviceByInstance(bus->busType_u.spi.instance)].errorCount++;
        return;
    }

    spi_master_get_default_control_config(&controlConfig);
    controlConfig.master_config.cmd_enable = false;
    controlConfig.master_config.addr_enable = false;
    controlConfig.master_config.addr_phase_fmt = spi_address_phase_format_single_io_mode;
    controlConfig.common_config.tx_dma_enable = true;
    controlConfig.common_config.rx_dma_enable = true;
    controlConfig.common_config.trans_mode = spi_trans_write_read_together;
    controlConfig.common_config.data_phase_fmt = spi_single_io_mode;
    controlConfig.common_config.dummy_cnt = spi_dummy_count_1;

    stat = spi_setup_dma_transfer(instance, &controlConfig, NULL, NULL, bus->curSegment->len, bus->curSegment->len);
    if (stat != status_success) {
        dma_disable_channel(dmaRx->dma, DMA_DESC_CHANNEL(dmaRx));
        dma_disable_channel(dmaTx->dma, DMA_DESC_CHANNEL(dmaTx));
        spiInternalStopDMA(dev);
        IOHi(dev->busType_u.spi.csnPin);
        bus->curSegment = (busSegment_t *) BUS_SPI_FREE;
        spiDevice[spiDeviceByInstance(bus->busType_u.spi.instance)].errorCount++;
    }
}

FAST_CODE void spiInternalStopDMA(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    SPI_Type *instance = (SPI_Type *) (bus->busType_u.spi.instance);
    spi_disable_tx_dma(instance);
    spi_disable_rx_dma(instance);
}

uint16_t spiCalculateDivider(uint32_t freq)
{
    // This API has no instance context; since spiInitDevice() clocks every SPI
    // instance from the same source/divider, clock_spi0 is representative of all
    const uint32_t spiClk = clock_get_frequency(clock_spi0);

    if (freq == 0) {
        return UINT16_MAX - 1U;
    }

    // Round the quotient UP to the next even integer so the resulting SCLK
    // never exceeds the requested frequency, matching the STM32/PICO
    // spiCalculateDivider() contract.  Floor-to-even here would overrun the
    // caller's requested speed by up to a third (e.g. spiClk 80 MHz, 11 MHz
    // requested: floor 6 -> 13.3 MHz).
    uint32_t divisor = ((spiClk + freq - 1U) / freq + 1U) & ~1U;

    // Keep the value non-zero at both ends of the input range so
    // spiSequenceStart() can safely convert it back to Hz.
    return MIN(MAX(2U, divisor), UINT16_MAX - 1U);
}

uint32_t spiCalculateClock(uint16_t spiClkDivisor)
{
    // All SPI instances use the same source clock; match spiCalculateDivider().
    const uint32_t spiClk = clock_get_frequency(clock_spi0);
    const uint32_t divisor = MAX(2U, (uint32_t) spiClkDivisor);

    return spiClk / divisor;
}

FAST_CODE void spiSequenceStart(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    SPI_Type *instance = (SPI_Type *) (bus->busType_u.spi.instance);
    bool dmaSafe = dev->useDMA;
    uint32_t segmentCount = 0;

    dev->bus->initSegment = true;

    if (dev->busType_u.spi.speed != bus->busType_u.spi.speed) {
        // Convert the requested divisor to an SCLK frequency using this bus' own clock
        const spiDevice_e device = spiDeviceByInstance(bus->busType_u.spi.instance);
        const uint32_t spiClock = clock_get_frequency(spiDevice[device].rcc);

        if (hpm_spi_set_sclk_frequency(instance, spiClock / dev->busType_u.spi.speed) == status_success) {
            // Cache the requested divisor only after the hardware accepts it
            bus->busType_u.spi.speed = dev->busType_u.spi.speed;
        }
    }

    if (dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge) {
        // Betaflight supports SPI modes 0 (leading edge) and 3 (trailing edge)
        if (dev->busType_u.spi.leadingEdge) {
            spi_set_clock_phase(instance, spi_sclk_sampling_odd_clk_edges);
            spi_set_clock_polarity(instance, spi_sclk_low_idle);
        } else {
            spi_set_clock_phase(instance, spi_sclk_sampling_even_clk_edges);
            spi_set_clock_polarity(instance, spi_sclk_high_idle);
        }
        bus->busType_u.spi.leadingEdge = dev->busType_u.spi.leadingEdge;
    }
    // Check that any there are no attempts to DMA to/from CCD SRAM
    for (busSegment_t *checkSegment = (busSegment_t *) bus->curSegment; checkSegment->len; checkSegment++) {

        // Note that these counts are only valid if dmaSafe is true
        segmentCount++;
    }

    if (bus->useDMA && dmaSafe && (!bus->curSegment[segmentCount].negateCS)) {
        // initialise the init structures for the first transfer
        spiInternalInitStream(dev, NULL);

        // Assert Chip Select
        IOLo(dev->busType_u.spi.csnPin);

        // Start the transfers
        spiInternalStartDMA(dev);
    } else {
        busSegment_t *lastSegment = NULL;
        bool segmentComplete;

        // Manually work through the segment list performing a transfer for each
        while (bus->curSegment->len) {
            if (!lastSegment || lastSegment->negateCS) {
                // Assert Chip Select if necessary - it's costly so only do so if necessary
                IOLo(dev->busType_u.spi.csnPin);
            }

            spiInternalReadWriteBufPolled(bus->busType_u.spi.instance,
                                          bus->curSegment->u.buffers.txData,
                                          bus->curSegment->u.buffers.rxData, bus->curSegment->len);


            if (bus->curSegment->negateCS) {
                // Negate Chip Select
                IOHi(dev->busType_u.spi.csnPin);
            }

            segmentComplete = true;
            if (bus->curSegment->callback) {
                switch (bus->curSegment->callback(dev->callbackArg)) {
                case BUS_BUSY:
                    // Repeat the last DMA segment
                    segmentComplete = false;
                    break;

                case BUS_ABORT:
                    bus->curSegment = (busSegment_t *) BUS_SPI_FREE;
                    segmentComplete = false;
                    return;

                case BUS_READY:
                default:
                    // Advance to the next DMA segment
                    break;
                }
            }
            if (segmentComplete) {
                lastSegment = (busSegment_t *) bus->curSegment;
                bus->curSegment++;
            }
        }

        // If a following transaction has been linked, start it
        if (bus->curSegment->u.link.dev) {
            busSegment_t *endSegment = (busSegment_t *) bus->curSegment;
            const extDevice_t *nextDev = endSegment->u.link.dev;
            busSegment_t *nextSegments = (busSegment_t *) endSegment->u.link.segments;
            bus->curSegment = nextSegments;
            endSegment->u.link.dev = NULL;
            endSegment->u.link.segments = NULL;
            spiSequenceStart(nextDev);
        } else {
            // The end of the segment list has been reached, so mark transactions as complete
            bus->curSegment = (busSegment_t *) BUS_SPI_FREE;
        }
    }
}
#endif
