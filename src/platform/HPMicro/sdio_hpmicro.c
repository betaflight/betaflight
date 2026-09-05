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

// SD card driver over SDXC/SDIO for HPMicro.
// Provides pin/clock configuration, non-blocking DMA block transfers and card
// initialisation through the HPM SDMMC host stack.

/* Include(s) -------------------------------------------------------------------------------------------------------*/

#include <stdbool.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SDCARD_SDIO
#include "board.h"
#include "drivers/io.h"
#include "drivers/sdio.h"
#include "drivers/sdmmc_sdio.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "io_hpmicro.h"
#include "sdio_hpmicro.h"
#include "hpm_clock_drv.h"
#include "hpm_iomux.h"
#include "hpm_sdmmc_sd.h"
#include "hpm_sdmmc_host.h"
#include "hpm_l1c_drv.h"
#include "pg/sdcard.h"
#include "pg/sdio.h"
#include "hpm_sdxc_soc_drv.h"
#include "nvic_hpmicro.h"

ATTR_PLACE_AT_NONCACHEABLE_BSS sdmmc_host_t g_sdmmc_host;
sd_card_t g_sd = { .host = &g_sdmmc_host };

typedef struct SD_Handle_s {
    volatile uint32_t RXCplt;   // SD RX Complete is equal 0 when no transfer
    volatile uint32_t TXCplt;   // SD TX Complete is equal 0 when no transfer
} SD_Handle_t;

typedef struct _SD_nonblock_transfer_t {
    uint32_t *buffer;
    uint32_t block_count;
    uint32_t start_block;

} SD_nonblock_transfer_t;
static SD_nonblock_transfer_t nonblockParam;
SD_CardInfo_t SD_CardInfo;
SD_CardType_t SD_CardType;

static SD_Handle_t SD_Handle;
#if ENABLE_SDIO_INIT

typedef struct sdioPin_s {
    ioTag_t pin;
    uint32_t af;
} sdioPin_t;

#define SDIO_PIN_D0  0
#define SDIO_PIN_D1  1
#define SDIO_PIN_D2  2
#define SDIO_PIN_D3  3
#define SDIO_PIN_CK  4
#define SDIO_PIN_CMD 5
#define SDIO_PIN_COUNT 6

#define SDIO_MAX_PINDEFS 2

typedef struct sdioHardware_s {
    SDXC_Type *instance;
    IRQn_Type irqn;
    sdioPin_t sdioPinCK[SDIO_MAX_PINDEFS];
    sdioPin_t sdioPinCMD[SDIO_MAX_PINDEFS];
    sdioPin_t sdioPinD0[SDIO_MAX_PINDEFS];
    sdioPin_t sdioPinD1[SDIO_MAX_PINDEFS];
    sdioPin_t sdioPinD2[SDIO_MAX_PINDEFS];
    sdioPin_t sdioPinD3[SDIO_MAX_PINDEFS];
} sdioHardware_t;

#define PINDEF(pin, af) { DEFIO_TAG_E(pin), af }

static const sdioHardware_t sdioPinHardware[SDIODEV_COUNT] = {
#if defined(HPM6360)
    [SDIODEV_1] = {
        .instance = HPM_SDXC0,
        .irqn = IRQn_SDXC0,
        .sdioPinCK = { PINDEF(PA2, IOC_PA02_FUNC_CTL_SDC0_CLK), PINDEF(PA11, IOC_PA11_FUNC_CTL_SDC0_CLK)},
        .sdioPinCMD = { PINDEF(PA3, IOC_PA03_FUNC_CTL_SDC0_CMD), PINDEF(PA10, IOC_PA10_FUNC_CTL_SDC0_CMD)},
        .sdioPinD0 =
        { PINDEF(PA1, IOC_PA01_FUNC_CTL_SDC0_DATA_0), PINDEF(PA12, IOC_PA12_FUNC_CTL_SDC0_DATA_0)},
        .sdioPinD1 =
        { PINDEF(PA0, IOC_PA00_FUNC_CTL_SDC0_DATA_1), PINDEF(PA13, IOC_PA13_FUNC_CTL_SDC0_DATA_1)},
        .sdioPinD2 =
        { PINDEF(PA5, IOC_PA05_FUNC_CTL_SDC0_DATA_2), PINDEF(PA8, IOC_PA08_FUNC_CTL_SDC0_DATA_2)},
        .sdioPinD3 =
        { PINDEF(PA4, IOC_PA04_FUNC_CTL_SDC0_DATA_3), PINDEF(PA9, IOC_PA09_FUNC_CTL_SDC0_DATA_3)},
    },
#elif defined(HPM6750)
    [SDIODEV_1] = {
        .instance = HPM_SDXC0,
        .irqn = IRQn_SDXC0,
        .sdioPinCK = { PINDEF(PE11, IOC_PE11_FUNC_CTL_SDC0_CLK), PINDEF(PE27, IOC_PE27_FUNC_CTL_SDC0_CLK)},
        .sdioPinCMD = { PINDEF(PE10, IOC_PE10_FUNC_CTL_SDC0_CMD), PINDEF(PE22, IOC_PE22_FUNC_CTL_SDC0_CMD)},
        .sdioPinD0 =
        { PINDEF(PE9, IOC_PE09_FUNC_CTL_SDC0_DATA_0), PINDEF(PE26, IOC_PE26_FUNC_CTL_SDC0_DATA_0)},
        .sdioPinD1 =
        { PINDEF(PE8, IOC_PE08_FUNC_CTL_SDC0_DATA_1), PINDEF(PE21, IOC_PE21_FUNC_CTL_SDC0_DATA_1)},
        .sdioPinD2 =
        { PINDEF(PE13, IOC_PE13_FUNC_CTL_SDC0_DATA_2), PINDEF(PE28, IOC_PE28_FUNC_CTL_SDC0_DATA_2)},
        .sdioPinD3 =
        { PINDEF(PE12, IOC_PE12_FUNC_CTL_SDC0_DATA_3), PINDEF(PE23, IOC_PE23_FUNC_CTL_SDC0_DATA_3)},
    },
    [SDIODEV_2] = {
        .instance = HPM_SDXC1,
        .irqn = IRQn_SDXC1,
        .sdioPinCK = { PINDEF(PD22, IOC_PD22_FUNC_CTL_SDC1_CLK)},
        .sdioPinCMD = { PINDEF(PD21, IOC_PD21_FUNC_CTL_SDC1_CMD)},
        .sdioPinD0 = { PINDEF(PD18, IOC_PD18_FUNC_CTL_SDC1_DATA_0)},
        .sdioPinD1 = { PINDEF(PD17, IOC_PD17_FUNC_CTL_SDC1_DATA_1)},
        .sdioPinD2 = { PINDEF(PD27, IOC_PD27_FUNC_CTL_SDC1_DATA_2)},
        .sdioPinD3 = { PINDEF(PD26, IOC_PD26_FUNC_CTL_SDC1_DATA_3)},
    },
#endif
};

#undef PINDEF

static const sdioHardware_t *sdioHardware;
static sdioPin_t sdioPin[SDIO_PIN_COUNT];

static const sdioPin_t *sdioFindPinDef(const sdioPin_t *pindefs, ioTag_t pin)
{
    for (unsigned index = 0; index < SDIO_MAX_PINDEFS; index++) {
        if (pindefs[index].pin == pin) {
            return &pindefs[index];
        }
    }

    return NULL;
}

#define SDIOFINDPIN(pinname) do { \
    const sdioPin_t *pindef = sdioFindPinDef(hardware->sdioPin ## pinname, pinConfig->pinname ## Pin); \
    if (!pindef) { \
        return; \
    } \
    sdioPin[SDIO_PIN_ ## pinname] = *pindef; \
} while (false)

void sdioPinConfigure(void)
{
    sdioHardware = NULL;
    memset(sdioPin, 0, sizeof(sdioPin));

    const SDIODevice device = SDIO_CFG_TO_DEV(sdioConfig()->device);

    if (device < SDIODEV_1 || device >= SDIODEV_COUNT) {
        return;
    }

    const sdioHardware_t *hardware = &sdioPinHardware[device];

    if (!hardware->instance) {
        return;
    }

    const sdioPinConfig_t *pinConfig = sdioPinConfig();

    SDIOFINDPIN(CK);
    SDIOFINDPIN(CMD);
    SDIOFINDPIN(D0);

    if (sdioConfig()->use4BitWidth) {
        SDIOFINDPIN(D1);
        SDIOFINDPIN(D2);
        SDIOFINDPIN(D3);
    }

    sdioHardware = hardware;
}

#undef SDIOFINDPIN

void sdioInitialize(void)
{
    if (!sdioHardware) {
        return;
    }

    IOInit(IOGetByTag(sdioPin[SDIO_PIN_CK].pin), OWNER_SDIO_CK, 0);
    IOInit(IOGetByTag(sdioPin[SDIO_PIN_CMD].pin), OWNER_SDIO_CMD, 0);
    IOInit(IOGetByTag(sdioPin[SDIO_PIN_D0].pin), OWNER_SDIO_D0, 0);

    if (sdioConfig()->use4BitWidth) {
        IOInit(IOGetByTag(sdioPin[SDIO_PIN_D1].pin), OWNER_SDIO_D1, 0);
        IOInit(IOGetByTag(sdioPin[SDIO_PIN_D2].pin), OWNER_SDIO_D2, 0);
        IOInit(IOGetByTag(sdioPin[SDIO_PIN_D3].pin), OWNER_SDIO_D3, 0);
    }
}

static void sdioConfigurePin(const sdioPin_t *pin, bool pullUp, bool openDrain, bool loopBack, bool is1v8)
{
    IO_t io = IOGetByTag(pin->pin);

    if (!io) {
        return;
    }

    uint32_t funcCtl = pin->af;

    if (loopBack) {
        funcCtl |= IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
    }

    ioConfig_t ioConfig = (pullUp) ? IOCFG_AF_PP_UP : IOCFG_AF_PP;
    if (openDrain) {
        ioConfig = (pullUp) ? IOCFG_AF_OD_UP : IOCFG_AF_OD;
    }
    IOConfigGPIOAF(io, ioConfig, funcCtl);

#if defined(HPM6360)
    UNUSED(is1v8);
    uint32_t padCtl = IOC_PAD_PAD_CTL_DS_SET(7);
#else
    uint32_t padCtl = IOC_PAD_PAD_CTL_MS_SET(is1v8) | IOC_PAD_PAD_CTL_DS_SET(6);
#endif
    if (pullUp) {
        padCtl |= IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    }
    if (openDrain) {
        padCtl |= IOC_PAD_PAD_CTL_OD_MASK;
    }
    HPM_IOC->PAD[IOCIndex(io)].PAD_CTL = padCtl;
}

static void sdioCmdPinInit(SDXC_Type *instance, bool openDrain, bool is1v8)
{
    if (!sdioHardware || instance != sdioHardware->instance) {
        return;
    }

    sdioConfigurePin(&sdioPin[SDIO_PIN_CMD], true, openDrain, true, is1v8);
}

static void sdioClkDataPinsInit(SDXC_Type *instance, uint32_t width, bool is1v8)
{
    if (!sdioHardware || instance != sdioHardware->instance) {
        return;
    }
#if defined(HPM6360)
    const bool loopBack = true;
    const bool clockPullUp = false;
#else
    const bool loopBack = false;
    const bool clockPullUp = true;
#endif
    sdioConfigurePin(&sdioPin[SDIO_PIN_CK], clockPullUp, false, loopBack, is1v8);
    sdioConfigurePin(&sdioPin[SDIO_PIN_D0], true, false, loopBack, is1v8);

    if (width == 4) {
        sdioConfigurePin(&sdioPin[SDIO_PIN_D1], true, false, loopBack, is1v8);
        sdioConfigurePin(&sdioPin[SDIO_PIN_D2], true, false, loopBack, is1v8);
        sdioConfigurePin(&sdioPin[SDIO_PIN_D3], true, false, loopBack, is1v8);
    }
}

static void sdioCardDetectPinInit(SDXC_Type *instance, bool asGpio)
{
    UNUSED(instance);
    UNUSED(asGpio);

    const IO_t io = IOGetByTag(sdcardConfig()->cardDetectTag);

    if (io) {
        IOConfigGPIO(io, IOCFG_IPU);
    }
}

static uint32_t sdioConfigureClock(SDXC_Type *instance, uint32_t frequency, bool inverse)
{
#if defined(HPM6360)
    const clock_name_t sdxcClock = clock_sdxc0;

    if (instance != HPM_SDXC0) {
        return 0;
    }

    clock_add_to_group(sdxcClock, HPM_CORE0);
    sdxc_enable_inverse_clock(instance, false);
    sdxc_enable_sd_clock(instance, false);
    clock_set_source_divider(sdxcClock, clk_src_pll0_clk0, 2);
    sdxc_enable_freq_selection(instance);

    if (clock_wait_source_stable(sdxcClock) != status_success) {
        return 0;
    }

    uint32_t divider;

    if (frequency <= 400000UL) {
        divider = 600;
    } else if (frequency <= 26000000UL) {
        divider = 8;
    } else if (frequency <= 52000000UL) {
        divider = 4;
    } else if (frequency <= 100000000UL) {
        divider = 2;
    } else if (frequency <= 208000000UL) {
        divider = 1;
    } else {
        divider = 8;
    }
    sdxc_set_clock_divider(instance, divider);

    sdxc_enable_inverse_clock(instance, inverse);
    sdxc_enable_sd_clock(instance, true);
    return clock_get_frequency(sdxcClock) / sdxc_get_clock_divider(instance);
#elif defined(HPM6750)
    clock_name_t sdxcClock;

    if (instance == HPM_SDXC0) {
        sdxcClock = clock_sdxc0;
    } else if (instance == HPM_SDXC1) {
        sdxcClock = clock_sdxc1;
    } else {
        return 0;
    }

    clock_add_to_group(sdxcClock, HPM_CORE0);
    sdxc_enable_inverse_clock(instance, false);
    sdxc_enable_sd_clock(instance, false);

    if (frequency <= 400000UL) {
        clock_set_source_divider(sdxcClock, clk_src_osc24m, 63);
    } else if (frequency <= 26000000UL) {
        clock_set_source_divider(sdxcClock, clk_src_osc24m, 1);
    } else if (frequency <= 52000000UL) {
        clock_set_source_divider(sdxcClock, clk_src_pll1_clk1, 8);
    } else if (frequency <= 100000000UL) {
        clock_set_source_divider(sdxcClock, clk_src_pll1_clk1, 4);
    } else if (frequency <= 208000000UL) {
        clock_set_source_divider(sdxcClock, clk_src_pll2_clk0, 2);
    } else {
        clock_set_source_divider(sdxcClock, clk_src_osc24m, 1);
    }

    sdxc_enable_inverse_clock(instance, inverse);
    sdxc_enable_sd_clock(instance, true);
    return clock_get_frequency(sdxcClock);
#else
#error "Unsupported HPMicro SoC"
#endif
}

static void sdioDelayMs(uint32_t milliseconds)
{
    delay(milliseconds);
}

hpm_stat_t board_init_sd_host_params(sdmmc_host_t *host, SDMMCHOST_Type *base)
{
    if (!host || !sdioHardware || base != sdioHardware->instance) {
        return status_invalid_argument;
    }

    sdmmc_host_param_t *param = &host->host_param;

    memset(param, 0, sizeof(*param));

    param->base = base;
    param->clock_init_func = sdioConfigureClock;
    param->hart_id = HPM_CORE0;
    param->delay_ms = sdioDelayMs;
    param->host_flags = HPM_SDMMC_HOST_SUPPORT_3V3;
    if (sdioConfig()->use4BitWidth) {
        param->host_flags |= HPM_SDMMC_HOST_SUPPORT_4BIT;
    }

    param->io_init_apis.cmd_io_init = sdioCmdPinInit;
    param->io_init_apis.clk_data_io_init = sdioClkDataPinsInit;

    const IO_t cardDetect = IOGetByTag(sdcardConfig()->cardDetectTag);

    if (cardDetect) {
        param->host_flags |= HPM_SDMMC_HOST_SUPPORT_CARD_DETECTION;
        param->io_init_apis.cd_io_init = sdioCardDetectPinInit;
        param->io_data.cd_pin.use_gpio = true;
        param->io_data.cd_pin.gpio_pin = IOCIndex(cardDetect);
        param->io_data.cd_pin.polarity = sdcardConfig()->cardDetectInverted;
    }

    return status_success;
}

hpm_stat_t hpmSdioConfigureHost(sdmmc_host_t *host, uint32_t irqPriority)
{
    if (!sdioHardware) {
        return status_invalid_argument;
    }

    intc_m_enable_irq_with_priority(sdioHardware->irqn, irqPriority);
    return board_init_sd_host_params(host, sdioHardware->instance);
}
#else
hpm_stat_t hpmSdioConfigureHost(sdmmc_host_t *host, uint32_t irqPriority)
{
    intc_m_enable_irq_with_priority(BOARD_APP_SDCARD_SDXC_IRQ, irqPriority);
    return board_init_sd_host_params(host, HPM_SDXC0);
}
#endif

#if defined(HPM_SDMMC_HOST_ENABLE_IRQ) && (HPM_SDMMC_HOST_ENABLE_IRQ == 1)
#if ENABLE_SDIO_INIT
SDK_DECLARE_EXT_ISR_M(IRQn_SDXC0, sdxc0_isr)
void sdxc0_isr(void)
{
    sdmmchost_irq_handler(&g_sdmmc_host);
}

#if defined(HPM6750)
SDK_DECLARE_EXT_ISR_M(IRQn_SDXC1, sdxc1_isr)
void sdxc1_isr(void)
{
    sdmmchost_irq_handler(&g_sdmmc_host);
}
#endif
#else
SDK_DECLARE_EXT_ISR_M(BOARD_APP_SDCARD_SDXC_IRQ, sdxc_isr)
void sdxc_isr(void)
{
    sdmmchost_irq_handler(&g_sdmmc_host);
}
#endif
#endif
/* Define(s) --------------------------------------------------------------------------------------------------------*/

static void sdTransferCallback(void *param);

bool SD_InitialiseHardware(dmaResource_t *dma)
{
    (void) dma;
#if ENABLE_SDIO_INIT
    return sdioHardware != NULL;
#else
    return true;
#endif
}

SD_Error_t SD_CheckWrite(void)
{
    if (SD_Handle.TXCplt != 0) {
        return SD_BUSY;
    }
    return SD_OK;
}

SD_Error_t SD_CheckRead(void)
{
    if (SD_Handle.RXCplt != 0) {
        return SD_BUSY;
    }
    return SD_OK;
}

SD_Error_t SD_ReadBlocks_DMA(uint64_t ReadAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    if (BlockSize != 512) {
        return SD_ERROR;        // unsupported.
    }

    /* DMA reads invalidate whole L1 cache lines at completion.  Require a
     * cache-line-aligned start address so this operation cannot invalidate
     * data preceding the DMA buffer. */
    if (buffer == NULL || ((uintptr_t) buffer & (HPM_L1C_CACHELINE_SIZE - 1U)) != 0) {
        return SD_ADDR_MISALIGNED;
    }

    nonblockParam.block_count = NumberOfBlocks;
    nonblockParam.start_block = ReadAddress;
    nonblockParam.buffer = buffer;

    // Set the in-progress flag before starting so a fast completion IRQ can
    // never run ahead of it; clear it again if the start itself fails so
    // SD_CheckRead() does not report SD_BUSY forever.
    SD_Handle.RXCplt = 1;
    hpm_stat_t status;

    status = sd_start_read_blocks(&g_sd, (uint8_t *) buffer, ReadAddress, NumberOfBlocks, sdTransferCallback,
                                  &nonblockParam);
    if (status != status_success) {
        SD_Handle.RXCplt = 0;
        return SD_ERROR;
    }

    return SD_OK;
}

SD_Error_t SD_WriteBlocks_DMA(uint64_t WriteAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    if (BlockSize != 512) {
        return SD_ERROR;        // unsupported.
    }

    /* Keep writes under the same DMA-buffer alignment contract as reads. */
    if (buffer == NULL || ((uintptr_t) buffer & (HPM_L1C_CACHELINE_SIZE - 1U)) != 0) {
        return SD_ADDR_MISALIGNED;
    }

    nonblockParam.block_count = NumberOfBlocks;
    nonblockParam.start_block = WriteAddress;
    nonblockParam.buffer = buffer;

    SD_Handle.TXCplt = 1;
    hpm_stat_t status;

    status = sd_start_write_blocks(&g_sd, (uint8_t *) buffer, WriteAddress, NumberOfBlocks, sdTransferCallback,
                                   &nonblockParam);
    if (status != status_success) {
        SD_Handle.TXCplt = 0;
        return SD_ERROR;
    }

    return SD_OK;
}

// Tx Transfer completed callback
static void sdTxCpltCallback(void *param)
{
    UNUSED(param);

    SD_Handle.TXCplt = 0;
}

// Rx Transfer completed callback
static void sdRxCpltCallback(void *param)
{
    SD_nonblock_transfer_t *context = (SD_nonblock_transfer_t *) param;
    /*
       l1c_dc_invalidate requires a cacheline-aligned address;
       adjust the address and the D-Cache size to invalidate accordingly.
     */
    uint32_t alignedStart = HPM_L1C_CACHELINE_ALIGN_DOWN((uint32_t) context->buffer);
    uint32_t endAddr = (uint32_t) context->buffer + g_sd.block_size * context->block_count;
    uint32_t alignedEnd = HPM_L1C_CACHELINE_ALIGN_UP(endAddr);
    uint32_t alignedSize = alignedEnd - alignedStart;

    /* Invalidate the D-cache BEFORE clearing the completion flag: anything
       that observes RXCplt == 0 (a higher priority interrupt preempting this
       handler, or a poller on the other core) must already read the DMA'd
       buffer contents instead of stale cache lines. */
    l1c_dc_invalidate(alignedStart, alignedSize);

    SD_Handle.RXCplt = 0;
}

bool SD_GetState(void)
{
    if (sd_polling_card_status_busy(&g_sd, 0) != status_success) {
        return false;
    }

    return g_sd.r1_status.current_state == sdmmc_state_transfer;
}

static void sdTransferCallback(void *param)
{
    if (SD_Handle.RXCplt) {
        sdRxCpltCallback(param);
    }
    if (SD_Handle.TXCplt) {
        sdTxCpltCallback(param);
    }
}

// Gets the SD card status
SD_Error_t SD_GetCardInfo(void)
{
    /* The SDK keeps the CID/CSD in decoded form (sd_cid_t bitfields /
     * sd_csd_t fields), so map those directly instead of parsing raw
     * register words the way the STM32 implementation does. */
    SD_CardInfo.SD_cid.ManufacturerID = g_sd.cid.mid;
    SD_CardInfo.SD_cid.OEM_AppliID = g_sd.cid.oid;
    SD_CardInfo.SD_cid.ProdName1 = (uint32_t) (g_sd.cid.pnm >> 8);
    SD_CardInfo.SD_cid.ProdName2 = (uint8_t) (g_sd.cid.pnm & 0xFF);
    SD_CardInfo.SD_cid.ProdRev = g_sd.cid.prv;
    SD_CardInfo.SD_cid.ProdSN = g_sd.cid.psn;
    SD_CardInfo.SD_cid.Reserved1 = 0;
    SD_CardInfo.SD_cid.ManufactDate = g_sd.cid.mdt;
    SD_CardInfo.SD_cid.CID_CRC = g_sd.cid.crc7;
    SD_CardInfo.SD_cid.Reserved2 = 1;

    SD_CardInfo.SD_csd.CSDStruct = g_sd.csd.csd_structure;
    SD_CardInfo.SD_csd.TAAC = g_sd.csd.data_read_access_time1;
    SD_CardInfo.SD_csd.NSAC = g_sd.csd.data_read_access_time2;
    SD_CardInfo.SD_csd.MaxBusClkFrec = g_sd.csd.transfer_speed;
    SD_CardInfo.SD_csd.CardComdClasses = g_sd.csd.card_command_class;
    SD_CardInfo.SD_csd.RdBlockLen = (uint8_t) g_sd.csd.read_block_len;
    SD_CardInfo.SD_csd.PartBlockRead = g_sd.csd.support_read_block_partial;
    SD_CardInfo.SD_csd.WrBlockMisalign = g_sd.csd.support_write_block_misalignment;
    SD_CardInfo.SD_csd.RdBlockMisalign = g_sd.csd.support_read_block_misalignment;
    SD_CardInfo.SD_csd.DSRImpl = g_sd.csd.is_dsr_implemented;
    SD_CardInfo.SD_csd.DeviceSize = g_sd.csd.device_size;
    SD_CardInfo.SD_csd.MaxRdCurrentVDDMin = g_sd.csd.read_current_vdd_min;
    SD_CardInfo.SD_csd.MaxRdCurrentVDDMax = g_sd.csd.read_current_vdd_max;
    SD_CardInfo.SD_csd.MaxWrCurrentVDDMin = g_sd.csd.write_current_vdd_min;
    SD_CardInfo.SD_csd.MaxWrCurrentVDDMax = g_sd.csd.write_current_vdd_max;
    SD_CardInfo.SD_csd.DeviceSizeMul = g_sd.csd.device_size_multiplier;
    SD_CardInfo.SD_csd.WrSpeedFact = g_sd.csd.write_speed_factor;
    SD_CardInfo.SD_csd.MaxWrBlockLen = (uint8_t) g_sd.csd.max_write_block_len;
    SD_CardInfo.SD_csd.WriteBlockPaPartial = g_sd.csd.support_write_block_partial;
    SD_CardInfo.SD_csd.WrProtectGrEnable = g_sd.csd.is_write_protection_group_enabled;
    SD_CardInfo.SD_csd.FileFormatGrouop = g_sd.csd.support_file_format_group;
    SD_CardInfo.SD_csd.CopyFlag = g_sd.csd.support_copy;
    SD_CardInfo.SD_csd.PermWrProtect = g_sd.csd.support_permanent_write_protect;
    SD_CardInfo.SD_csd.TempWrProtect = g_sd.csd.support_temporary_write_protect;
    SD_CardInfo.SD_csd.FileFormat = g_sd.csd.file_format;
    /* No decoded counterparts: SysSpecVersion, EraseGrSize/EraseGrMul,
     * WrProtectGrSize, ManDeflECC, ECC, CSD_CRC and the Reserved fields stay
     * zero. Common code only reads SD_cid, CardCapacity and CardBlockSize. */

    SD_CardInfo.CardCapacity = g_sd.block_count;
    SD_CardInfo.CardBlockSize = g_sd.block_size;

    return SD_OK;
}

SD_Error_t SD_Init(void)
{
    static bool sdInitAttempted = false;
    static SD_Error_t result = SD_ERROR;
    hpm_stat_t status;

    if (sdInitAttempted) {
        return result;
    }

    sdInitAttempted = true;
    status = hpmSdioConfigureHost(&g_sdmmc_host, hpmPlicPriorityFromNvic(NVIC_PRIO_SDIO_DMA));
    if (status != status_success) {
        return result;
    }
    status = sd_init(&g_sd);
    if (status == status_success) {
        result = SD_OK;
        // SD Card 2.0
        switch (g_sd.status.card_type) {
        case 0:
            SD_CardType = SD_STD_CAPACITY_V1_1;
            if (g_sd.capacity_v2_0_or_high) {
                SD_CardType = SD_STD_CAPACITY_V2_0;
            }
            break;
        case 1:
            SD_CardType = SD_HIGH_CAPACITY;
            break;
        default:
            result = SD_ERROR;
            return result;
        }
        // Fill in SD_CardInfo from the SDK-decoded CID/CSD
        result = SD_GetCardInfo();
    }

    return result;
}

/* ------------------------------------------------------------------------------------------------------------------*/
#endif
