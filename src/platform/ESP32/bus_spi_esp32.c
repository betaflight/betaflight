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

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/dma_impl.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "pg/bus_spi.h"

#include "platform/dma.h"

// Undefine SPI0/SPI1 macros before ESP-IDF headers to avoid conflict
// with extern spi_dev_t SPI0/SPI1 in soc/spi_struct.h (included transitively)
#undef SPI0
#undef SPI1

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wsign-compare"
#include "hal/spi_ll.h"
#include "hal/gpio_ll.h"
#ifdef USE_DMA
#include "hal/gdma_ll.h"
#endif
#pragma GCC diagnostic pop

// Do NOT restore SPI0/SPI1 macros in this file — ESP-IDF's SPI_LL_GET_HW
// references SPI1/SPI2/SPI3 as extern spi_dev_t symbols, and our macros
// would break &SPI1 expansions. Use esp32SpiDev0/1 directly below.

#include "soc/gpio_struct.h"
#ifdef USE_DMA
#include "soc/gdma_channel.h"
#include "esp_rom_lldesc.h"
#endif
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

// SPI clock idle edge register lives under misc.ck_idle_edge on modern
// SoCs (S3/C5/P4) and under pin.ck_idle_edge on the original ESP32 (LX6).
#if defined(ESP32S3) || defined(ESP32C5) || defined(ESP32P4)
#define ESP32_SPI_CK_IDLE  misc.ck_idle_edge
#else
#define ESP32_SPI_CK_IDLE  pin.ck_idle_edge
#endif

// APB clock frequency
#define ESP32_APB_CLK_FREQ  80000000

// SPI hardware buffer size in bytes (for polled transfers)
#define SPI_MAX_TRANSFER_SIZE  64

// DMA threshold: use DMA for transfers >= this many bytes
#define SPI_DMA_THRESHOLD  8

// RX descriptor capacity is rounded to a 32-bit boundary, so use Espressif's
// largest four-byte-aligned descriptor payload rather than the raw 12-bit max.
#define SPI_DMA_MAX_DESCRIPTOR_SIZE  4092

// Per-bus storage covers a complete descriptor.  Besides supplying a missing
// TX/RX direction it is used as an RX bounce buffer when a caller supplies an
// address that GDMA cannot access on a 32-bit boundary.
#define SPI_DMA_DUMMY_BUFFER_SIZE  SPI_DMA_MAX_DESCRIPTOR_SIZE

#if defined(ESP32S3)
// For very short full-duplex sensor reads the fixed GDMA setup plus a second
// interrupt costs more CPU time than filling/polling the 64-byte SPI FIFO.
// Sixteen bytes covers the common 8/9/13/14/15-byte gyro bursts while keeping
// longer/background transfers asynchronous on DMA.  A board target may set
// this to zero (or another measured limit) without changing the common driver.
#ifndef ESP32S3_SPI_SHORT_POLLED_MAX_SIZE
#define ESP32S3_SPI_SHORT_POLLED_MAX_SIZE  16
#endif
#endif

// GPIO matrix signal indices for SPI2 (FSPI) and SPI3 (HSPI)
static const struct {
    uint8_t clkOut;
    uint8_t mosiOut;
    uint8_t misoIn;
} spiSignals[] = {
#if defined(ESP32C5)
    // ESP32-C5 exposes only FSPI as a general-purpose master; second slot
    // is left pointing at FSPI as a placeholder until the C5 SPI port lands.
    { FSPICLK_OUT_IDX, FSPID_OUT_IDX,    FSPIQ_IN_IDX    },  // SPI2 (FSPI)
    { FSPICLK_OUT_IDX, FSPID_OUT_IDX,    FSPIQ_IN_IDX    },  // SPI3 placeholder
#elif defined(ESP32P4)
    // ESP32-P4 GPIO signals are all _PAD_ suffixed.
    { SPI2_CK_PAD_OUT_IDX, SPI2_D_PAD_OUT_IDX, SPI2_Q_PAD_IN_IDX },  // SPI2
    { SPI3_CK_PAD_OUT_IDX, SPI3_D_PAD_OUT_IDX, SPI3_Q_PAD_IN_IDX },  // SPI3
#elif defined(ESP32S3)
    { FSPICLK_OUT_IDX, FSPID_OUT_IDX,    FSPIQ_IN_IDX    },  // SPI2 (FSPI)
    { SPI3_CLK_OUT_IDX, SPI3_D_OUT_IDX,  SPI3_Q_IN_IDX   },  // SPI3
#else
    { HSPICLK_OUT_IDX, HSPID_OUT_IDX,    HSPIQ_IN_IDX    },  // SPI2 (HSPI)
    { VSPICLK_OUT_IDX, VSPID_OUT_IDX,    VSPIQ_IN_IDX    },  // SPI3 (VSPI)
#endif
};

#ifdef USE_DMA
// GDMA trigger peripheral IDs for SPI2 and SPI3
static const int spiGdmaPeriphId[] = {
    SOC_GDMA_TRIG_PERIPH_SPI2,  // SPI device 0 -> SPI2
    SOC_GDMA_TRIG_PERIPH_SPI3,  // SPI device 1 -> SPI3
};
#endif

// ESP32-S3 has SPI2 (FSPI) and SPI3 (HSPI) available for general use.
// GPIO matrix allows any pin to be used, but define common defaults.
const spiHardware_t spiHardware[SPIDEV_COUNT] = {
    {
        .device = SPIDEV_0,
        .reg = (spiResource_t *)&esp32SpiDev0,  // Maps to SPI2
        .sckPins = {
#if defined(ESP32S3)
            { DEFIO_TAG_E(PA12) },
            { DEFIO_TAG_E(PA36) },
#else
            { DEFIO_TAG_E(PA14) },  // HSPI default SCK
#endif
        },
        .misoPins = {
#if defined(ESP32S3)
            { DEFIO_TAG_E(PA13) },
            { DEFIO_TAG_E(PA37) },
#else
            { DEFIO_TAG_E(PA12) },  // HSPI default MISO
#endif
        },
        .mosiPins = {
#if defined(ESP32S3)
            { DEFIO_TAG_E(PA11) },
            { DEFIO_TAG_E(PA35) },
#else
            { DEFIO_TAG_E(PA13) },  // HSPI default MOSI
#endif
        },
    },
#if SPIDEV_COUNT > 1
    {
        .device = SPIDEV_1,
        .reg = (spiResource_t *)&esp32SpiDev1,  // Maps to SPI3
        .sckPins = {
#if defined(ESP32S3)
            { DEFIO_TAG_E(PA14) },
            { DEFIO_TAG_E(PA38) },
#else
            { DEFIO_TAG_E(PA18) },  // VSPI default SCK
#endif
        },
        .misoPins = {
#if defined(ESP32S3)
            { DEFIO_TAG_E(PA15) },
            { DEFIO_TAG_E(PA39) },
#else
            { DEFIO_TAG_E(PA19) },  // VSPI default MISO
#endif
        },
        .mosiPins = {
#if defined(ESP32S3)
            { DEFIO_TAG_E(PA16) },
            { DEFIO_TAG_E(PA40) },
#else
            { DEFIO_TAG_E(PA23) },  // VSPI default MOSI
#endif
        },
    },
#endif // SPIDEV_COUNT > 1
};

extern busDevice_t spiBusDevice[SPIDEV_COUNT];

#ifdef USE_DMA
// DMA descriptors for SPI transfers (one TX + one RX per SPI bus).
static lldesc_t dmaTxDesc[SPIDEV_COUNT] __attribute__((aligned(4)));
static lldesc_t dmaRxDesc[SPIDEV_COUNT] __attribute__((aligned(4)));

typedef struct spiDmaDescriptorCache_s {
    const uint8_t *txBuffer;
    uint8_t *rxBuffer;
    uint8_t *rxCopyTarget;
    uint16_t length;
    uint16_t rxCopyLength;
} spiDmaDescriptorCache_t;

static spiDmaDescriptorCache_t dmaDescriptorCache[SPIDEV_COUNT];

// Dummy buffers for DMA when no TX/RX buffer is provided.
// Keep one pair per bus so simultaneous SPI2/SPI3 activity cannot alias them.
static uint8_t dummyTxBuf[SPIDEV_COUNT][SPI_DMA_DUMMY_BUFFER_SIZE] __attribute__((aligned(4)));
static uint8_t dummyRxBuf[SPIDEV_COUNT][SPI_DMA_DUMMY_BUFFER_SIZE] __attribute__((aligned(4)));
#endif

// Map from Betaflight SPI device number (0/1) to ESP-IDF SPI host ID.
// esp32SpiDev0 = 0 -> SPI2_HOST (1), esp32SpiDev1 = 1 -> SPI3_HOST (2)
static spi_host_device_t spiGetHostId(int deviceNum)
{
    return (spi_host_device_t)(deviceNum + 1);
}

static spi_dev_t *spiGetHw(int deviceNum)
{
    return SPI_LL_GET_HW(spiGetHostId(deviceNum));
}

void spiPinConfigure(const struct spiPinConfig_s *pConfig)
{
    for (size_t hwindex = 0; hwindex < ARRAYLEN(spiHardware); hwindex++) {
        const spiHardware_t *hw = &spiHardware[hwindex];

        if (!hw->reg) {
            continue;
        }

        const spiDevice_e device = hw->device;
        spiDevice_t *pDev = &spiDevice[device];

        // First try to match against the known pin table entries
        for (int pindex = 0; pindex < MAX_SPI_PIN_SEL; pindex++) {
            if (pConfig[device].ioTagSck == hw->sckPins[pindex].pin) {
                pDev->sck = hw->sckPins[pindex].pin;
            }
            if (pConfig[device].ioTagMiso == hw->misoPins[pindex].pin) {
                pDev->miso = hw->misoPins[pindex].pin;
            }
            if (pConfig[device].ioTagMosi == hw->mosiPins[pindex].pin) {
                pDev->mosi = hw->mosiPins[pindex].pin;
            }
        }

        // On ESP32 any GPIO can be routed via the GPIO matrix, so accept
        // configured pins even when they don't match a table entry.
        if (!pDev->sck && pConfig[device].ioTagSck) {
            pDev->sck = pConfig[device].ioTagSck;
        }
        if (!pDev->miso && pConfig[device].ioTagMiso) {
            pDev->miso = pConfig[device].ioTagMiso;
        }
        if (!pDev->mosi && pConfig[device].ioTagMosi) {
            pDev->mosi = pConfig[device].ioTagMosi;
        }

        if (pDev->sck) {
            pDev->dev = hw->reg;
            pDev->leadingEdge = false;
        }
    }
}

void spiPreinitRegister(ioTag_t iotag, uint32_t iocfg, uint8_t init)
{
    ioPreinitByTag(iotag, iocfg, init);
}

void spiPreinitByIO(IO_t io)
{
    ioPreinitByIO(io, SPI_IO_CS_HIGH_CFG, PREINIT_PIN_STATE_HIGH);
}

void spiPreinitByTag(ioTag_t tag)
{
    spiPreinitByIO(IOGetByTag(tag));
}

void spiInitDevice(spiDevice_e device)
{
    const spiDevice_t *spi = &spiDevice[device];

    if (!spi->dev) {
        return;
    }

    int deviceNum = SPI_INST((SPI_TypeDef *)spi->dev);
    spi_host_device_t hostId = spiGetHostId(deviceNum);
    spi_dev_t *hw = SPI_LL_GET_HW(hostId);

    // Enable bus clock and reset peripheral
    // The LL macros require __DECLARE_RCC_ATOMIC_ENV to be in scope
    {
        int __DECLARE_RCC_ATOMIC_ENV __attribute__((unused));
        spi_ll_enable_bus_clock(hostId, true);
        spi_ll_reset_register(hostId);
    }

    // Initialize as SPI master
    spi_ll_master_init(hw);

    // Enable full-duplex mode
    hw->user.doutdin = 1;
    hw->user.usr_miso = 1;
    hw->user.usr_mosi = 1;

    // Set default clock: 1 MHz
    spi_ll_master_set_clock(hw, ESP32_APB_CLK_FREQ, 1000000, 128);

    // Apply configuration
    spi_ll_apply_config(hw);

    // Set resource owners
    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_SDI, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_SDO, RESOURCE_INDEX(device));

    // Configure SCK pin via GPIO matrix
    IO_t sckIO = IOGetByTag(spi->sck);
    if (sckIO) {
        uint32_t pin = IO_Pin(sckIO);
        esp_rom_gpio_pad_select_gpio(pin);
        gpio_ll_output_enable(&GPIO, pin);
        esp_rom_gpio_connect_out_signal(pin, spiSignals[deviceNum].clkOut, false, false);
    }

    // Configure MOSI pin via GPIO matrix
    IO_t mosiIO = IOGetByTag(spi->mosi);
    if (mosiIO) {
        uint32_t pin = IO_Pin(mosiIO);
        esp_rom_gpio_pad_select_gpio(pin);
        gpio_ll_output_enable(&GPIO, pin);
        esp_rom_gpio_connect_out_signal(pin, spiSignals[deviceNum].mosiOut, false, false);
    }

    // Configure MISO pin via GPIO matrix
    IO_t misoIO = IOGetByTag(spi->miso);
    if (misoIO) {
        uint32_t pin = IO_Pin(misoIO);
        esp_rom_gpio_pad_select_gpio(pin);
        gpio_ll_input_enable(&GPIO, pin);
        gpio_ll_pullup_en(&GPIO, pin);
        esp_rom_gpio_connect_in_signal(pin, spiSignals[deviceNum].misoIn, false);
    }
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    UNUSED(bus);
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    UNUSED(descriptor);
}

#ifdef USE_DMA
// Interrupt handler for SPI RX DMA completion
FAST_IRQ_HANDLER static void spiRxIrqHandler(dmaChannelDescriptor_t *descriptor)
{
    const extDevice_t *dev = (const extDevice_t *)descriptor->userParam;

    if (!dev) {
        return;
    }

    busDevice_t *bus = dev->bus;

    if (bus->curSegment->negateCS) {
        IOHi(dev->busType_u.spi.csnPin);
    }

    spiInternalStopDMA(dev);

    // GDMA requires a word-aligned RX start address (all ESP GDMA SoCs, not
    // just S3).  Copy from the per-bus bounce buffer before the segment
    // callback consumes the data.
    int deviceNum = SPI_INST((SPI_TypeDef *)bus->busType_u.spi.instance);
    spiDmaDescriptorCache_t *cache = &dmaDescriptorCache[deviceNum];
    if (cache->rxCopyTarget) {
        memcpy(cache->rxCopyTarget, dummyRxBuf[deviceNum], cache->rxCopyLength);
        cache->rxCopyTarget = NULL;
        cache->rxCopyLength = 0;
    }

    spiIrqHandler(dev);
}
#endif

FAST_CODE void spiInternalStopDMA(const extDevice_t *dev)
{
#ifndef USE_DMA
    UNUSED(dev);
#else
    busDevice_t *bus = dev->bus;

    if (!bus->dmaTx || !bus->dmaRx) {
        return;
    }

    int txCh = bus->dmaTx->channel;
    int rxCh = bus->dmaRx->channel;

    // Stop both GDMA channels
    gdma_ll_tx_stop(&GDMA, txCh);
    gdma_ll_rx_stop(&GDMA, rxCh);

    // Disable SPI DMA
    int deviceNum = SPI_INST((SPI_TypeDef *)bus->busType_u.spi.instance);
    spi_dev_t *hw = spiGetHw(deviceNum);
    spi_ll_dma_tx_enable(hw, false);
    spi_ll_dma_rx_enable(hw, false);
#endif
}

FAST_CODE void spiInternalInitStream(const extDevice_t *dev, volatile busSegment_t *segment)
{
#ifndef USE_DMA
    UNUSED(dev);
    UNUSED(segment);
#else
    // ESP32 descriptors are bound immediately before launch.  Keeping the
    // STM32-style per-segment init hook as a no-op avoids a store/load pair in
    // the gyro ISR while preserving the shared bus API.
    UNUSED(dev);
    UNUSED(segment);
#endif
}

FAST_CODE void spiInternalStartDMA(const extDevice_t *dev)
{
#ifndef USE_DMA
    UNUSED(dev);
#else
    busDevice_t *bus = dev->bus;
    int deviceNum = SPI_INST((SPI_TypeDef *)bus->busType_u.spi.instance);
    spi_dev_t *hw = spiGetHw(deviceNum);

    int txCh = bus->dmaTx->channel;
    int rxCh = bus->dmaRx->channel;

    // Pass device pointer to RX ISR via userParam
    bus->dmaRx->userParam = (uint32_t)dev;

    volatile busSegment_t *segment = bus->curSegment;
    int xferLen = segment->len & BUS_SEGMENT_LEN_LENGTH_MASK;

    const uint8_t *txBuffer = segment->u.buffers.txData;
    uint8_t *rxBuffer = segment->u.buffers.rxData;

    const bool hasTx = txBuffer != NULL;
    const bool hasRx = rxBuffer != NULL;
    const bool rxNeedsBounce = hasRx && (((uintptr_t)rxBuffer & 3U) != 0);
    const uint8_t *effectiveTxBuffer = hasTx ? txBuffer : dummyTxBuf[deviceNum];
    uint8_t *effectiveRxBuffer = hasRx && !rxNeedsBounce ? rxBuffer : dummyRxBuf[deviceNum];

    // Reset GDMA channels
    gdma_ll_tx_reset_channel(&GDMA, txCh);
    gdma_ll_rx_reset_channel(&GDMA, rxCh);

    // Reset SPI DMA FIFOs
    spi_ll_dma_tx_fifo_reset(hw);
    spi_ll_dma_rx_fifo_reset(hw);

    // Configure TX DMA descriptor.  The invariant fields (offset, sosf and
    // next link) are zeroed once in spiInitBusDMA(); only fields changed by a
    // transaction or by GDMA are re-armed here.
    lldesc_t *txDesc = &dmaTxDesc[deviceNum];
    lldesc_t *rxDesc = &dmaRxDesc[deviceNum];
    spiDmaDescriptorCache_t *cache = &dmaDescriptorCache[deviceNum];
    const uint16_t rxDmaSize = (xferLen + 3U) & ~3U;

    if (cache->length != xferLen || cache->txBuffer != effectiveTxBuffer) {
        txDesc->size = xferLen;
        txDesc->buf = effectiveTxBuffer;
        cache->txBuffer = effectiveTxBuffer;
    }
    if (cache->length != xferLen || cache->rxBuffer != effectiveRxBuffer) {
        rxDesc->size = rxDmaSize;
        rxDesc->buf = effectiveRxBuffer;
        cache->rxBuffer = effectiveRxBuffer;
    }
    cache->length = xferLen;
    cache->rxCopyTarget = rxNeedsBounce ? rxBuffer : NULL;
    cache->rxCopyLength = rxNeedsBounce ? xferLen : 0;

    txDesc->length = xferLen;
    txDesc->eof = 1;

    // Configure RX DMA descriptor
    rxDesc->length = 0;  // filled by hardware
    rxDesc->eof = 0;

    // Hand ownership over only after all descriptor contents are visible.
    txDesc->owner = 1;
    rxDesc->owner = 1;  // owned by DMA hardware
#if defined(ESP32S3)
    __asm__ volatile ("memw" ::: "memory");
#else
    __asm__ volatile ("" ::: "memory");
#endif

    // Set SPI transfer length
    spi_ll_set_mosi_bitlen(hw, xferLen * 8);
    spi_ll_set_miso_bitlen(hw, xferLen * 8);

    // Enable SPI DMA
    spi_ll_dma_tx_enable(hw, true);
    spi_ll_dma_rx_enable(hw, true);

    // Set descriptor addresses
    gdma_ll_tx_set_desc_addr(&GDMA, txCh, (uint32_t)txDesc);
    gdma_ll_rx_set_desc_addr(&GDMA, rxCh, (uint32_t)rxDesc);

    // Clear any pending RX interrupt
    gdma_ll_rx_clear_interrupt_status(&GDMA, rxCh, GDMA_LL_EVENT_RX_SUC_EOF);

    // Start both DMA channels
    gdma_ll_rx_start(&GDMA, rxCh);
    gdma_ll_tx_start(&GDMA, txCh);

    // Apply SPI config and start the SPI transaction
    spi_ll_apply_config(hw);
    spi_ll_user_start(hw);
#endif
}

FAST_CODE bool spiInternalReadWriteBufPolled(spiResource_t *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
    int deviceNum = SPI_INST((SPI_TypeDef *)instance);
    spi_dev_t *hw = spiGetHw(deviceNum);

    int offset = 0;
    while (offset < len) {
        int chunkLen = MIN(len - offset, SPI_MAX_TRANSFER_SIZE);
        int bitLen = chunkLen * 8;

        // Reset FIFOs
        spi_ll_cpu_tx_fifo_reset(hw);
        spi_ll_cpu_rx_fifo_reset(hw);

        // Set transfer length
        spi_ll_set_mosi_bitlen(hw, bitLen);
        spi_ll_set_miso_bitlen(hw, bitLen);

        // Write TX data into hardware buffer
        if (txData) {
            spi_ll_write_buffer(hw, txData + offset, bitLen);
        } else {
            // Send 0xFF when no TX data (same convention as other platforms)
            uint8_t dummy[SPI_MAX_TRANSFER_SIZE];
            memset(dummy, 0xFF, chunkLen);
            spi_ll_write_buffer(hw, dummy, bitLen);
        }

        // Ensure DMA is disabled for polled transfers
        spi_ll_dma_tx_enable(hw, false);
        spi_ll_dma_rx_enable(hw, false);

        // Apply config and start transfer
        spi_ll_apply_config(hw);
        spi_ll_user_start(hw);

        // Poll for completion
        while (!spi_ll_usr_is_done(hw)) {
            // busy wait
        }

        // Clear the transfer done interrupt flag
        spi_ll_clear_int_stat(hw);

        // Read RX data from hardware buffer
        if (rxData) {
            spi_ll_read_buffer(hw, rxData + offset, bitLen);
        }

        offset += chunkLen;
    }

    return true;
}

FAST_CODE void spiSequenceStart(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    spiResource_t *instance = bus->busType_u.spi.instance;
    spiDevice_t *spi = &spiDevice[spiDeviceByInstance(instance)];
    bool dmaSafe = dev->useDMA;
    bool shortPolledTransfer = false;
    uint32_t xferLen = 0;
    uint32_t segmentCount = 0;

    bus->initSegment = true;

    // Switch bus speed if needed
    if (dev->busType_u.spi.speed != bus->busType_u.spi.speed) {
        int deviceNum = SPI_INST((SPI_TypeDef *)instance);
        spi_dev_t *hw = spiGetHw(deviceNum);
        uint32_t freq = spiCalculateClock(dev->busType_u.spi.speed);
        spi_ll_master_set_clock(hw, ESP32_APB_CLK_FREQ, freq, 128);
        spi_ll_apply_config(hw);
        bus->busType_u.spi.speed = dev->busType_u.spi.speed;
    }

    // Switch SPI clock polarity/phase if necessary
    if (dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge) {
        int deviceNum = SPI_INST((SPI_TypeDef *)instance);
        spi_dev_t *hw = spiGetHw(deviceNum);

        if (dev->busType_u.spi.leadingEdge) {
            // SPI Mode 0: CPOL=0, CPHA=0
            hw->ESP32_SPI_CK_IDLE = 0;    // Clock idle low
            hw->user.ck_out_edge = 0;     // Data clocked on leading edge
        } else {
            // SPI Mode 3: CPOL=1, CPHA=1
            hw->ESP32_SPI_CK_IDLE = 1;    // Clock idle high
            hw->user.ck_out_edge = 1;     // Data clocked on trailing edge
        }
        spi_ll_apply_config(hw);

        bus->busType_u.spi.leadingEdge = dev->busType_u.spi.leadingEdge;
    }

    UNUSED(spi);

    // Count segments and total transfer length
    for (busSegment_t *checkSegment = (busSegment_t *)bus->curSegment; checkSegment->len; checkSegment++) {
        const uint32_t segmentLen = checkSegment->len & BUS_SEGMENT_LEN_LENGTH_MASK;
        segmentCount++;
        xferLen += segmentLen;

        // A single descriptor cannot represent a larger segment.  Larger
        // transactions retain the existing chunked polling fallback.
        if (segmentLen > SPI_DMA_MAX_DESCRIPTOR_SIZE) {
            dmaSafe = false;
        }
    }

#if defined(ESP32S3)
    shortPolledTransfer = ESP32S3_SPI_SHORT_POLLED_MAX_SIZE > 0 &&
        segmentCount == 1 &&
        xferLen <= ESP32S3_SPI_SHORT_POLLED_MAX_SIZE &&
        bus->curSegment[segmentCount].negateCS &&
        bus->curSegment->u.buffers.txData &&
        bus->curSegment->u.buffers.rxData;
#endif

    // Use DMA if possible:
    // - Bus supports DMA (bus->useDMA set by spiInitBusDMA)
    // - Device allows DMA (dev->useDMA)
    // - Multiple segments, or single segment >= threshold, or CS held after last segment
    if (!shortPolledTransfer && bus->useDMA && dmaSafe && ((segmentCount > 1) ||
                                    (xferLen >= SPI_DMA_THRESHOLD) ||
                                    !bus->curSegment[segmentCount].negateCS)) {
        spiProcessSegmentsDMA(dev);
    } else {
        spiProcessSegmentsPolled(dev);
    }
}

uint16_t spiCalculateDivider(uint32_t freq)
{
    if (freq == 0) {
        return 0;
    }

    // Pack the target frequency into the uint16_t divider value.
    // APB clock is 80 MHz; compute a simple integer divider.
    uint32_t divider = (ESP32_APB_CLK_FREQ + freq - 1) / freq;
    if (divider < 1) {
        divider = 1;
    }
    if (divider > 65535) {
        divider = 65535;
    }

    return (uint16_t)divider;
}

uint32_t spiCalculateClock(uint16_t spiClkDivisor)
{
    if (spiClkDivisor == 0) {
        return ESP32_APB_CLK_FREQ;
    }

    return ESP32_APB_CLK_FREQ / spiClkDivisor;
}

void spiInitBusDMA(void)
{
#ifdef USE_DMA
    // Initialise the GDMA hardware
    esp32DmaInit();

    // Fill TX dummy buffer with 0xFF (SPI idle byte convention)
    memset(dummyTxBuf, 0xFF, sizeof(dummyTxBuf));
    memset(dmaTxDesc, 0, sizeof(dmaTxDesc));
    memset(dmaRxDesc, 0, sizeof(dmaRxDesc));
    memset(dmaDescriptorCache, 0, sizeof(dmaDescriptorCache));

    for (uint32_t device = 0; device < SPIDEV_COUNT; device++) {
        busDevice_t *bus = &spiBusDevice[device];

        if (bus->busType != BUS_TYPE_SPI) {
            continue;
        }

        int deviceNum = SPI_INST((SPI_TypeDef *)bus->busType_u.spi.instance);

        // Allocate a GDMA channel for TX
        dmaIdentifier_e txId = dmaGetFreeIdentifier();
        if (txId == DMA_NONE) {
            return;
        }

        // Allocate a GDMA channel for RX
        dmaIdentifier_e rxId = dmaGetFreeIdentifier();
        if (rxId == DMA_NONE) {
            return;
        }

        if (!dmaAllocate(txId, OWNER_SPI_SDO, device + 1) ||
            !dmaAllocate(rxId, OWNER_SPI_SDI, device + 1)) {
            return;
        }

        int txCh = DMA_IDENTIFIER_TO_CHANNEL(txId);
        int rxCh = DMA_IDENTIFIER_TO_CHANNEL(rxId);

        bus->dmaTx = &dmaDescriptors[DMA_CHANNEL_TO_INDEX(txCh)];
        bus->dmaTx->channel = txCh;

        bus->dmaRx = &dmaDescriptors[DMA_CHANNEL_TO_INDEX(rxCh)];
        bus->dmaRx->channel = rxCh;

        // Use the dmaInit fields to cache per-segment config (has TX/RX buffer flags)
        static DMA_InitTypeDef dmaInitTxStore[SPIDEV_COUNT];
        static DMA_InitTypeDef dmaInitRxStore[SPIDEV_COUNT];
        bus->dmaInitTx = &dmaInitTxStore[device];
        bus->dmaInitRx = &dmaInitRxStore[device];

        // Connect GDMA channels to SPI peripheral
        int periphId = spiGdmaPeriphId[deviceNum];
        gdma_ll_tx_connect_to_periph(&GDMA, txCh, GDMA_TRIG_PERIPH_SPI, periphId);
        gdma_ll_rx_connect_to_periph(&GDMA, rxCh, GDMA_TRIG_PERIPH_SPI, periphId);

        // Enable burst mode for better throughput
        gdma_ll_tx_enable_data_burst(&GDMA, txCh, true);
        gdma_ll_tx_enable_descriptor_burst(&GDMA, txCh, true);
        gdma_ll_rx_enable_data_burst(&GDMA, rxCh, true);
        gdma_ll_rx_enable_descriptor_burst(&GDMA, rxCh, true);

        spi_dev_t *hw = spiGetHw(deviceNum);
        spi_ll_dma_set_rx_eof_generation(hw, true);

        // Register RX DMA completion interrupt handler
        dmaSetHandler(rxId, spiRxIrqHandler, 0, 0);

        bus->useDMA = true;
    }
#endif
}

#endif // USE_SPI
