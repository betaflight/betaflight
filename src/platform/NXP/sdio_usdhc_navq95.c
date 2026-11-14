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
 * SD card on the i.MX 95 uSDHC2 controller, implementing the SD_* interface
 * used by drivers/sdcard_sdio_baremetal.c.
 *
 * Transfers use polled programmed I/O through the data port. The *_DMA entry
 * points complete the whole transfer before returning, and the check functions
 * then report it done.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SDCARD_SDIO

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/sdcard_standard.h"
#include "drivers/sdio.h"
#include "drivers/sdmmc_sdio.h"
#include "drivers/time.h"
#include "drivers/nvic.h"

#include "pg/sdio.h"

#include "platform/usdhc_regs.h"

#include "navq95_scmi.h"

#define NAVQ95_USDHC            ((USDHC_Type *)USDHC2_BASE)

#define NAVQ95_USDHC_CLK_ROOT   118u

// Nominal root rate. The real rate is measured at init and used if it is plausible.
#define NAVQ95_USDHC_ROOT_HZ    345600000u

#define NAVQ95_USDHC_ROOT_MIN   50000000u
#define NAVQ95_USDHC_ROOT_MAX   500000000u

#define IOMUXC_BASE             0x443C0000u
/*
 * Pad control values, passed to the System Manager unshifted. CLK: slow slew,
 * drive strength 4. CMD/DATA: the same plus pull-up and hysteresis. CD: input.
 */
#define NAVQ95_SD_PAD_CLK       0x001Eu
#define NAVQ95_SD_PAD_DATA      0x121Eu
#define NAVQ95_SD_PAD_CD        0x1200u

#define NAVQ95_SD_CD_MUX        (IOMUXC_BASE + 0x01A0u)
#define NAVQ95_SD_CD_PAD        (IOMUXC_BASE + 0x03A4u)

typedef struct {
    uint32_t muxReg;
    uint32_t padReg;
    uint32_t padVal;
    uint8_t  dataLine;
} navq95SdPad_t;

// CLK, CMD, DATA0..DATA3.
static const navq95SdPad_t navq95SdPads[] = {
    { IOMUXC_BASE + 0x01A4u, IOMUXC_BASE + 0x03A8u, NAVQ95_SD_PAD_CLK,  0 },
    { IOMUXC_BASE + 0x01A8u, IOMUXC_BASE + 0x03ACu, NAVQ95_SD_PAD_DATA, 1 },
    { IOMUXC_BASE + 0x01ACu, IOMUXC_BASE + 0x03B0u, NAVQ95_SD_PAD_DATA, 1 },
    { IOMUXC_BASE + 0x01B0u, IOMUXC_BASE + 0x03B4u, NAVQ95_SD_PAD_DATA, 1 },
    { IOMUXC_BASE + 0x01B4u, IOMUXC_BASE + 0x03B8u, NAVQ95_SD_PAD_DATA, 1 },
    { IOMUXC_BASE + 0x01B8u, IOMUXC_BASE + 0x03BCu, NAVQ95_SD_PAD_DATA, 1 },
};
#define NAVQ95_SD_ALT           0u

#define U   NAVQ95_USDHC

#define USDHC_ID_CLOCK_HZ               400000u
#define USDHC_TRANSFER_CLOCK_HZ         25000000u
#define USDHC_DATA_TIMEOUT_DTOCV        9u
#define USDHC_COMMAND_TIMEOUT_US        100000u
#define USDHC_POWER_UP_TIMEOUT_US       1000000u
#define USDHC_TRANSFER_TIMEOUT_US       2000000u

#define SDCARD_COMMAND_ALL_SEND_CID             2
#define SDCARD_COMMAND_SEND_RELATIVE_ADDR       3
#define SDCARD_COMMAND_SELECT_CARD              7
#define SDCARD_ACOMMAND_SET_BUS_WIDTH           6
#define SDCARD_IF_COND_ARGUMENT                 0x1AAu
#define SDCARD_OP_COND_VOLTAGE_WINDOW           0x00FF8000u
#define SDCARD_OP_COND_HIGH_CAPACITY            0x40000000u
#define SDCARD_OCR_POWER_UP_DONE                0x80000000u
#define SDCARD_OCR_CARD_CAPACITY_STATUS         0x40000000u
#define SDCARD_STATUS_READY_FOR_DATA            (1u << 8)
#define SDCARD_STATUS_CURRENT_STATE(status)     (((status) >> 9) & 0x0F)
#define SDCARD_STATE_TRANSFER                   4
#define SDCARD_ACMD6_BUS_WIDTH_1BIT             0
#define SDCARD_ACMD6_BUS_WIDTH_4BIT             2

#define USDHC_INT_COMMAND_ERRORS    (USDHC_INT_STATUS_CTOE_MASK  \
                                   | USDHC_INT_STATUS_CCE_MASK   \
                                   | USDHC_INT_STATUS_CEBE_MASK  \
                                   | USDHC_INT_STATUS_CIE_MASK)

#define USDHC_INT_DATA_ERRORS       (USDHC_INT_STATUS_DTOE_MASK  \
                                   | USDHC_INT_STATUS_DCE_MASK   \
                                   | USDHC_INT_STATUS_DEBE_MASK  \
                                   | USDHC_INT_STATUS_AC12E_MASK \
                                   | USDHC_INT_STATUS_DMAE_MASK)

#define USDHC_INT_ALL               (USDHC_INT_STATUS_CC_MASK    \
                                   | USDHC_INT_STATUS_TC_MASK    \
                                   | USDHC_INT_STATUS_BGE_MASK   \
                                   | USDHC_INT_STATUS_DINT_MASK  \
                                   | USDHC_INT_STATUS_BWR_MASK   \
                                   | USDHC_INT_STATUS_BRR_MASK   \
                                   | USDHC_INT_STATUS_CINS_MASK  \
                                   | USDHC_INT_STATUS_CRM_MASK   \
                                   | USDHC_INT_STATUS_CINT_MASK  \
                                   | USDHC_INT_STATUS_RTE_MASK   \
                                   | USDHC_INT_STATUS_TP_MASK    \
                                   | USDHC_INT_STATUS_TNE_MASK   \
                                   | USDHC_INT_COMMAND_ERRORS    \
                                   | USDHC_INT_DATA_ERRORS)

// One 512-byte block per buffer-ready event.
#define USDHC_PIO_WML_WORDS         128u

typedef enum {
    USDHC_RESPONSE_NONE = 0,
    USDHC_RESPONSE_R1,
    USDHC_RESPONSE_R1B,
    USDHC_RESPONSE_R2,
    USDHC_RESPONSE_R3,
} usdhcResponse_e;

typedef struct usdhcState_s {
    bool initialised;
    bool transferActive;
    SD_Error_t pendingError;
    uint16_t rca;
    bool highCapacity;
    bool fourBitBus;
    uint32_t csd[4];
    uint32_t cid[4];
} usdhcState_t;

static usdhcState_t usdhc;

// Last error, kept for debugging.
#define USDHC_NO_COMMAND    0xFFu

static volatile struct {
    SD_Error_t error;
    uint8_t command;
    uint32_t detail;
} usdhcLastError;

static volatile uint32_t usdhcLastCmdStatus;

static SD_Error_t usdhcRecordError(uint8_t command, SD_Error_t error, uint32_t detail)
{
    if (error != SD_OK) {
        usdhcLastError.error = error;
        usdhcLastError.command = command;
        usdhcLastError.detail = detail;
    }
    return error;
}

SD_CardInfo_t SD_CardInfo;
SD_CardType_t SD_CardType;

static uint32_t usdhcClockDivider(uint32_t rootHz, uint32_t targetHz, uint32_t *actualHz)
{
    for (uint32_t prescaler = 2; prescaler <= 256; prescaler <<= 1) {
        for (uint32_t divisor = 1; divisor <= 16; divisor++) {
            const uint32_t hz = rootHz / (prescaler * divisor);
            if (hz <= targetHz) {
                if (actualHz) {
                    *actualHz = hz;
                }
                return USDHC_SYS_CTRL_SDCLKFS(prescaler >> 1) | USDHC_SYS_CTRL_DVS(divisor - 1);
            }
        }
    }
    if (actualHz) {
        *actualHz = rootHz / (256u * 16u);
    }
    return USDHC_SYS_CTRL_SDCLKFS(128) | USDHC_SYS_CTRL_DVS(15);
}

#define USDHC_FLAG_TIMEOUT_US   10000u

static bool usdhcWaitFlag(const volatile uint32_t *reg, uint32_t mask, bool set)
{
    const uint32_t start = micros();
    while (((*reg & mask) != 0) != set) {
        if (cmpTimeUs(micros(), start) > (timeDelta_t)USDHC_FLAG_TIMEOUT_US) {
            return false;
        }
    }
    return true;
}

static uint32_t usdhcRootHz = NAVQ95_USDHC_ROOT_HZ;

static bool usdhcSetClock(uint32_t targetHz)
{
    const uint32_t rootHz = usdhcRootHz;
    if (rootHz == 0) {
        return false;
    }
    const uint32_t divider = usdhcClockDivider(rootHz, targetHz, NULL);
    U->SYS_CTRL = (U->SYS_CTRL & ~(USDHC_SYS_CTRL_SDCLKFS_MASK | USDHC_SYS_CTRL_DVS_MASK)) | divider;
    return usdhcWaitFlag(&U->PRES_STATE, USDHC_PRES_STATE_SDSTB_MASK, true);
}

static int usdhcConfigPads(void)
{
    const unsigned count = sizeof(navq95SdPads) / sizeof(navq95SdPads[0]);
    for (unsigned i = 0; i < count; i++) {
        const navq95SdPad_t *p = &navq95SdPads[i];
        int s = navq95_scmi_pinmux(p->muxReg, NAVQ95_SD_ALT, p->dataLine ? 1u : 0u,
                                   p->padReg, p->padVal);
        if (s != 0) {
            return s;
        }
    }
    return 0;
}

static int usdhcConfigCdPad(void)
{
    return navq95_scmi_pinmux(NAVQ95_SD_CD_MUX, NAVQ95_SD_ALT, 1u,
                              NAVQ95_SD_CD_PAD, NAVQ95_SD_PAD_CD);
}

static void usdhcSetBusWidth(bool fourBit)
{
    U->PROT_CTRL = (U->PROT_CTRL & ~USDHC_PROT_CTRL_DTW_MASK) | USDHC_PROT_CTRL_DTW(fourBit ? 1 : 0);
}

/*
 * INITA sends 80 SD clocks and then clears itself. Timing it at a known divider
 * gives the root clock rate without needing a card.
 */
static uint32_t usdhcMeasureRootHz(void)
{
    const uint32_t prescaler = 256u;
    const uint32_t divisor   = 16u;
    U->SYS_CTRL = (U->SYS_CTRL & ~(USDHC_SYS_CTRL_SDCLKFS_MASK | USDHC_SYS_CTRL_DVS_MASK))
                | USDHC_SYS_CTRL_SDCLKFS(prescaler >> 1)
                | USDHC_SYS_CTRL_DVS(divisor - 1u);
    if (!usdhcWaitFlag(&U->PRES_STATE, USDHC_PRES_STATE_SDSTB_MASK, true)) {
        return 0;
    }

    const uint32_t t0 = micros();
    U->SYS_CTRL |= USDHC_SYS_CTRL_INITA_MASK;
    while (U->SYS_CTRL & USDHC_SYS_CTRL_INITA_MASK) {
        if (cmpTimeUs(micros(), t0) > (timeDelta_t)1000000) {
            return 0;
        }
    }
    const uint32_t dtUs = (uint32_t)cmpTimeUs(micros(), t0);
    if (dtUs == 0u) {
        return 0;
    }

    const uint64_t sdClkHz = (80ull * 1000000ull) / dtUs;
    return (uint32_t)(sdClkHz * (uint64_t)prescaler * (uint64_t)divisor);
}

static void usdhcCalibrateRoot(void)
{
    const uint32_t hz = usdhcMeasureRootHz();
    if (hz >= NAVQ95_USDHC_ROOT_MIN && hz <= NAVQ95_USDHC_ROOT_MAX) {
        usdhcRootHz = hz;
    }
}

static bool usdhcInitController(void)
{
    U->INT_SIGNAL_EN = 0;

    U->SYS_CTRL |= USDHC_SYS_CTRL_RSTA_MASK;
    if (!usdhcWaitFlag(&U->SYS_CTRL, USDHC_SYS_CTRL_RSTA_MASK, false)) {
        return false;
    }

    U->INT_SIGNAL_EN = 0;
    U->INT_STATUS_EN = USDHC_INT_ALL;
    U->INT_STATUS = USDHC_INT_ALL;

    U->PROT_CTRL = (U->PROT_CTRL
                    & ~(USDHC_PROT_CTRL_EMODE_MASK | USDHC_PROT_CTRL_DTW_MASK
                        | USDHC_PROT_CTRL_DMASEL_MASK))
                 | USDHC_PROT_CTRL_EMODE(2)
                 | USDHC_PROT_CTRL_DTW(0);

    U->WTMK_LVL = USDHC_WTMK_LVL_RD_WML(USDHC_PIO_WML_WORDS)
                | USDHC_WTMK_LVL_WR_WML(USDHC_PIO_WML_WORDS);

    U->SYS_CTRL = (U->SYS_CTRL & ~USDHC_SYS_CTRL_DTOCV_MASK)
                | USDHC_SYS_CTRL_DTOCV(USDHC_DATA_TIMEOUT_DTOCV);

    return usdhcSetClock(USDHC_ID_CLOCK_HZ);
}

static bool usdhcSendInitClocks(void)
{
    U->SYS_CTRL |= USDHC_SYS_CTRL_INITA_MASK;
    return usdhcWaitFlag(&U->SYS_CTRL, USDHC_SYS_CTRL_INITA_MASK, false);
}

static uint32_t usdhcTransferType(uint8_t index, usdhcResponse_e response, bool dataPresent)
{
    uint32_t value = USDHC_CMD_XFR_TYP_CMDINX(index);
    switch (response) {
    case USDHC_RESPONSE_NONE:
        break;
    case USDHC_RESPONSE_R1:
        value |= USDHC_CMD_XFR_TYP_RSPTYP(2) | USDHC_CMD_XFR_TYP_CICEN_MASK | USDHC_CMD_XFR_TYP_CCCEN_MASK;
        break;
    case USDHC_RESPONSE_R1B:
        value |= USDHC_CMD_XFR_TYP_RSPTYP(3) | USDHC_CMD_XFR_TYP_CICEN_MASK | USDHC_CMD_XFR_TYP_CCCEN_MASK;
        break;
    case USDHC_RESPONSE_R2:
        value |= USDHC_CMD_XFR_TYP_RSPTYP(1) | USDHC_CMD_XFR_TYP_CCCEN_MASK;
        break;
    case USDHC_RESPONSE_R3:
        value |= USDHC_CMD_XFR_TYP_RSPTYP(2);
        break;
    }
    if (dataPresent) {
        value |= USDHC_CMD_XFR_TYP_DPSEL_MASK;
    }
    return value;
}

static SD_Error_t usdhcCommandError(uint32_t status)
{
    if (status & USDHC_INT_STATUS_CTOE_MASK) {
        return SD_CMD_RSP_TIMEOUT;
    }
    if (status & USDHC_INT_STATUS_CIE_MASK) {
        return SD_ILLEGAL_CMD;
    }
    if (status & (USDHC_INT_STATUS_CCE_MASK | USDHC_INT_STATUS_CEBE_MASK)) {
        return SD_CMD_CRC_FAIL;
    }
    return SD_OK;
}

static SD_Error_t usdhcDataError(uint32_t status)
{
    if (status & USDHC_INT_STATUS_DTOE_MASK) {
        return SD_DATA_TIMEOUT;
    }
    if (status & (USDHC_INT_STATUS_DCE_MASK | USDHC_INT_STATUS_DEBE_MASK)) {
        return SD_DATA_CRC_FAIL;
    }
    if (status & USDHC_INT_STATUS_DMAE_MASK) {
        return SD_INTERNAL_ERROR;
    }
    if (status & USDHC_INT_STATUS_AC12E_MASK) {
        return SD_ERROR;
    }
    return SD_OK;
}

static void usdhcResetLines(bool data)
{
    U->SYS_CTRL |= USDHC_SYS_CTRL_RSTC_MASK | (data ? USDHC_SYS_CTRL_RSTD_MASK : 0);
    usdhcWaitFlag(&U->SYS_CTRL, USDHC_SYS_CTRL_RSTC_MASK | USDHC_SYS_CTRL_RSTD_MASK, false);
    U->INT_STATUS = USDHC_INT_ALL;
}

static SD_Error_t usdhcWaitBusyEnd(void)
{
    const uint32_t start = micros();
    for (;;) {
        const uint32_t status = U->INT_STATUS;
        if (status & USDHC_INT_DATA_ERRORS) {
            const SD_Error_t error = usdhcDataError(status);
            usdhcResetLines(true);
            return error;
        }
        if ((status & USDHC_INT_STATUS_TC_MASK) || !(U->PRES_STATE & USDHC_PRES_STATE_CDIHB_MASK)) {
            U->INT_STATUS = USDHC_INT_STATUS_TC_MASK;
            return SD_OK;
        }
        if (cmpTimeUs(micros(), start) > (timeDelta_t)USDHC_TRANSFER_TIMEOUT_US) {
            usdhcResetLines(true);
            return SD_DATA_TIMEOUT;
        }
    }
}

static SD_Error_t usdhcSendCommandFull(uint8_t index, uint32_t argument, usdhcResponse_e response,
                                       bool dataPresent, uint32_t mixControl)
{
    uint32_t inhibit = USDHC_PRES_STATE_CIHB_MASK;
    if (dataPresent || response == USDHC_RESPONSE_R1B) {
        inhibit |= USDHC_PRES_STATE_CDIHB_MASK;
    }

    const uint32_t inhibitStart = micros();
    while (U->PRES_STATE & inhibit) {
        if (cmpTimeUs(micros(), inhibitStart) > (timeDelta_t)USDHC_COMMAND_TIMEOUT_US) {
            usdhcResetLines(dataPresent);
            return SD_BUSY;
        }
    }

    U->INT_STATUS = USDHC_INT_ALL;
    U->CMD_ARG = argument;

    const uint32_t mixMask = USDHC_MIX_CTRL_DMAEN_MASK | USDHC_MIX_CTRL_BCEN_MASK
                           | USDHC_MIX_CTRL_AC12EN_MASK | USDHC_MIX_CTRL_DTDSEL_MASK
                           | USDHC_MIX_CTRL_MSBSEL_MASK;
    U->MIX_CTRL = (U->MIX_CTRL & ~mixMask) | (mixControl & mixMask);

    U->CMD_XFR_TYP = usdhcTransferType(index, response, dataPresent);

    const uint32_t commandStart = micros();
    uint32_t status;
    for (;;) {
        status = U->INT_STATUS;
        if (status & (USDHC_INT_STATUS_CC_MASK | USDHC_INT_COMMAND_ERRORS)) {
            break;
        }
        if (cmpTimeUs(micros(), commandStart) > (timeDelta_t)USDHC_COMMAND_TIMEOUT_US) {
            usdhcResetLines(dataPresent);
            return SD_CMD_RSP_TIMEOUT;
        }
    }

    const SD_Error_t error = usdhcCommandError(status);
    usdhcLastCmdStatus = status;
    if (error != SD_OK) {
        usdhcResetLines(dataPresent);
        return error;
    }

    U->INT_STATUS = USDHC_INT_STATUS_CC_MASK;

    if (response == USDHC_RESPONSE_R1B && !dataPresent) {
        return usdhcWaitBusyEnd();
    }
    return SD_OK;
}

static SD_Error_t usdhcSendCommand(uint8_t index, uint32_t argument, usdhcResponse_e response)
{
    const SD_Error_t error = usdhcSendCommandFull(index, argument, response, false, 0);
    return usdhcRecordError(index, error, usdhcLastCmdStatus);
}

static SD_Error_t usdhcSendAppCommand(uint8_t index, uint32_t argument, usdhcResponse_e response)
{
    const SD_Error_t error = usdhcSendCommand(SDCARD_COMMAND_APP_CMD,
                                              (uint32_t)usdhc.rca << 16, USDHC_RESPONSE_R1);
    if (error != SD_OK) {
        return error;
    }
    return usdhcSendCommand(index, argument, response);
}

// The controller drops the CRC byte of R2 responses, so shift the 120 bits back into place.
static void usdhcReadLongResponse(uint32_t out[4])
{
    const uint32_t rsp0 = U->CMD_RSP0;
    const uint32_t rsp1 = U->CMD_RSP1;
    const uint32_t rsp2 = U->CMD_RSP2;
    const uint32_t rsp3 = U->CMD_RSP3;
    out[0] = (rsp3 << 8) | (rsp2 >> 24);
    out[1] = (rsp2 << 8) | (rsp1 >> 24);
    out[2] = (rsp1 << 8) | (rsp0 >> 24);
    out[3] = (rsp0 << 8);
}

static uint32_t usdhcBlockAddress(uint32_t blockIndex)
{
    return usdhc.highCapacity ? blockIndex : blockIndex * SDCARD_BLOCK_SIZE;
}

static SD_Error_t usdhcWaitDataFlag(uint32_t flag)
{
    const uint32_t start = micros();
    for (;;) {
        const uint32_t status = U->INT_STATUS;
        if (status & USDHC_INT_DATA_ERRORS) {
            return usdhcDataError(status);
        }
        if (status & flag) {
            return SD_OK;
        }
        if (cmpTimeUs(micros(), start) > (timeDelta_t)USDHC_TRANSFER_TIMEOUT_US) {
            return SD_DATA_TIMEOUT;
        }
    }
}

static SD_Error_t usdhcPioBlock(bool read, uint32_t *words)
{
    const uint32_t ready = read ? USDHC_INT_STATUS_BRR_MASK : USDHC_INT_STATUS_BWR_MASK;
    const SD_Error_t error = usdhcWaitDataFlag(ready);
    if (error != SD_OK) {
        return error;
    }
    U->INT_STATUS = ready;
    if (read) {
        for (uint32_t i = 0; i < USDHC_PIO_WML_WORDS; i++) {
            words[i] = U->DATA_BUFF;
        }
    } else {
        for (uint32_t i = 0; i < USDHC_PIO_WML_WORDS; i++) {
            U->DATA_BUFF = words[i];
        }
    }
    return SD_OK;
}

static SD_Error_t usdhcStartTransfer(bool read, uint32_t blockIndex, uint32_t *buffer,
                                     uint32_t blockSize, uint32_t blockCount)
{
    if (!usdhc.initialised) {
        return usdhcRecordError(USDHC_NO_COMMAND, SD_NOT_CONFIGURED, 0);
    }
    if (usdhc.transferActive) {
        return usdhcRecordError(USDHC_NO_COMMAND, SD_BUSY, 0);
    }
    if (blockSize != SDCARD_BLOCK_SIZE || blockCount == 0) {
        return usdhcRecordError(USDHC_NO_COMMAND, SD_INVALID_PARAMETER, 0);
    }
    if ((uintptr_t)buffer & 0x03) {
        return usdhcRecordError(USDHC_NO_COMMAND, SD_ADDR_MISALIGNED, 0);
    }

    U->BLK_ATT = USDHC_BLK_ATT_BLKSIZE(blockSize) | USDHC_BLK_ATT_BLKCNT(blockCount);

    uint32_t mix = read ? USDHC_MIX_CTRL_DTDSEL_MASK : 0;
    uint8_t command;
    if (blockCount > 1) {
        mix |= USDHC_MIX_CTRL_MSBSEL_MASK | USDHC_MIX_CTRL_BCEN_MASK | USDHC_MIX_CTRL_AC12EN_MASK;
        command = read ? SDCARD_COMMAND_READ_MULTIPLE_BLOCK : SDCARD_COMMAND_WRITE_MULTIPLE_BLOCK;
    } else {
        command = read ? SDCARD_COMMAND_READ_SINGLE_BLOCK : SDCARD_COMMAND_WRITE_BLOCK;
    }

    usdhc.pendingError = SD_OK;

    SD_Error_t error = usdhcRecordError(command,
                                        usdhcSendCommandFull(command, usdhcBlockAddress(blockIndex),
                                                             USDHC_RESPONSE_R1, true, mix), 0);
    if (error != SD_OK) {
        return error;
    }

    for (uint32_t block = 0; block < blockCount && error == SD_OK; block++) {
        error = usdhcPioBlock(read, buffer + block * USDHC_PIO_WML_WORDS);
    }
    if (error == SD_OK) {
        error = usdhcWaitDataFlag(USDHC_INT_STATUS_TC_MASK);
    }
    if (error != SD_OK) {
        usdhc.pendingError = usdhcRecordError(command, error, U->INT_STATUS);
        usdhcResetLines(true);
    } else {
        U->INT_STATUS = USDHC_INT_STATUS_TC_MASK;
    }

    usdhc.transferActive = true;
    return SD_OK;
}

static SD_Error_t usdhcCheckTransfer(void)
{
    usdhc.transferActive = false;
    return SD_OK;
}

// Switch to 4-bit and read block 0 to check DAT1..DAT3; fall back to 1-bit if that fails.
static void usdhcTryFourBitBus(void)
{
    static uint32_t probe[USDHC_PIO_WML_WORDS];

    if (usdhcSendAppCommand(SDCARD_ACOMMAND_SET_BUS_WIDTH, SDCARD_ACMD6_BUS_WIDTH_4BIT,
                            USDHC_RESPONSE_R1) != SD_OK) {
        usdhcSetBusWidth(false);
        return;
    }
    usdhcSetBusWidth(true);

    usdhc.initialised = true;
    const SD_Error_t start = usdhcStartTransfer(true, 0, probe, SDCARD_BLOCK_SIZE, 1);
    const bool ok = (start == SD_OK) && (usdhc.pendingError == SD_OK);
    usdhc.transferActive = false;
    usdhc.pendingError = SD_OK;
    usdhc.initialised = false;

    if (!ok) {
        usdhcSetBusWidth(false);
        usdhcSendAppCommand(SDCARD_ACOMMAND_SET_BUS_WIDTH, SDCARD_ACMD6_BUS_WIDTH_1BIT,
                            USDHC_RESPONSE_R1);
    }
}

static SD_Error_t usdhcIdentifyCard(void)
{
    usdhc.rca = 0;
    usdhc.highCapacity = false;

    if (!usdhcSetClock(USDHC_ID_CLOCK_HZ)) {
        return usdhcRecordError(USDHC_NO_COMMAND, SD_ERROR, 0);
    }
    usdhcSetBusWidth(false);

    if (!usdhcSendInitClocks()) {
        return usdhcRecordError(USDHC_NO_COMMAND, SD_ERROR, 0);
    }

    SD_Error_t error = usdhcSendCommand(SDCARD_COMMAND_GO_IDLE_STATE, 0, USDHC_RESPONSE_NONE);
    if (error != SD_OK) {
        return error;
    }

    bool version2 = false;
    error = usdhcSendCommandFull(SDCARD_COMMAND_SEND_IF_COND, SDCARD_IF_COND_ARGUMENT,
                                 USDHC_RESPONSE_R3, false, 0);
    if (error == SD_OK) {
        if ((U->CMD_RSP0 & 0xFFF) != SDCARD_IF_COND_ARGUMENT) {
            return usdhcRecordError(SDCARD_COMMAND_SEND_IF_COND, SD_INVALID_VOLTRANGE, U->CMD_RSP0);
        }
        version2 = true;
    } else if (error != SD_CMD_RSP_TIMEOUT) {
        return usdhcRecordError(SDCARD_COMMAND_SEND_IF_COND, error, usdhcLastCmdStatus);
    }

    const uint32_t argument = SDCARD_OP_COND_VOLTAGE_WINDOW | (version2 ? SDCARD_OP_COND_HIGH_CAPACITY : 0);
    const uint32_t start = micros();
    uint32_t ocr;
    for (;;) {
        error = usdhcSendAppCommand(SDCARD_ACOMMAND_SEND_OP_COND, argument, USDHC_RESPONSE_R3);
        if (error != SD_OK) {
            return error;
        }
        ocr = U->CMD_RSP0;
        if (ocr & SDCARD_OCR_POWER_UP_DONE) {
            break;
        }
        if (cmpTimeUs(micros(), start) > (timeDelta_t)USDHC_POWER_UP_TIMEOUT_US) {
            return SD_CMD_RSP_TIMEOUT;
        }
        delayMicroseconds(1000);
    }

    usdhc.highCapacity = (ocr & SDCARD_OCR_CARD_CAPACITY_STATUS) != 0;
    if (usdhc.highCapacity) {
        SD_CardType = SD_HIGH_CAPACITY;
    } else {
        SD_CardType = version2 ? SD_STD_CAPACITY_V2_0 : SD_STD_CAPACITY_V1_1;
    }

    error = usdhcSendCommand(SDCARD_COMMAND_ALL_SEND_CID, 0, USDHC_RESPONSE_R2);
    if (error != SD_OK) {
        return error;
    }
    usdhcReadLongResponse(usdhc.cid);

    error = usdhcSendCommand(SDCARD_COMMAND_SEND_RELATIVE_ADDR, 0, USDHC_RESPONSE_R1);
    if (error != SD_OK) {
        return error;
    }
    usdhc.rca = (uint16_t)(U->CMD_RSP0 >> 16);

    error = usdhcSendCommand(SDCARD_COMMAND_SEND_CSD, (uint32_t)usdhc.rca << 16, USDHC_RESPONSE_R2);
    if (error != SD_OK) {
        return error;
    }
    usdhcReadLongResponse(usdhc.csd);

    error = usdhcSendCommand(SDCARD_COMMAND_SELECT_CARD, (uint32_t)usdhc.rca << 16, USDHC_RESPONSE_R1B);
    if (error != SD_OK) {
        return error;
    }

    error = usdhcSendCommand(SDCARD_COMMAND_SET_BLOCKLEN, SDCARD_BLOCK_SIZE, USDHC_RESPONSE_R1);
    if (error != SD_OK) {
        return error;
    }

    if (!usdhcSetClock(USDHC_TRANSFER_CLOCK_HZ)) {
        return usdhcRecordError(USDHC_NO_COMMAND, SD_ERROR, 0);
    }

    if (usdhc.fourBitBus) {
        usdhcTryFourBitBus();
    }
    return SD_OK;
}

bool SD_InitialiseHardware(dmaResource_t *dma)
{
    UNUSED(dma);

    if (SDIO_CFG_TO_DEV(sdioConfig()->device) != SDIODEV_1) {
        return false;
    }

    memset(&usdhc, 0, sizeof(usdhc));
    usdhc.fourBitBus = sdioConfig()->use4BitWidth;

    navq95_scmi_clock_enable(NAVQ95_USDHC_CLK_ROOT);
    if (usdhcConfigPads() != 0) {
        return false;
    }
    usdhcConfigCdPad();

    if (!usdhcInitController()) {
        return false;
    }
    // Initialise once to measure the root clock, then again with the measured rate.
    usdhcCalibrateRoot();
    return usdhcInitController();
}

SD_Error_t SD_Init(void)
{
    usdhc.initialised = false;
    usdhc.transferActive = false;
    usdhc.pendingError = SD_OK;

    if (!usdhcInitController()) {
        return usdhcRecordError(USDHC_NO_COMMAND, SD_ERROR, 0);
    }

    const SD_Error_t error = usdhcIdentifyCard();
    if (error != SD_OK) {
        return error;
    }

    usdhc.initialised = true;
    return SD_OK;
}

bool SD_IsDetected(void)
{
    return usdhc.initialised;
}

bool SD_GetState(void)
{
    if (!usdhc.initialised || usdhc.transferActive) {
        return false;
    }
    if (usdhc.pendingError != SD_OK) {
        usdhc.pendingError = SD_OK;
        return false;
    }
    if (usdhcSendCommand(SDCARD_COMMAND_SEND_STATUS, (uint32_t)usdhc.rca << 16, USDHC_RESPONSE_R1) != SD_OK) {
        return false;
    }
    const uint32_t status = U->CMD_RSP0;
    if (!(status & SDCARD_STATUS_READY_FOR_DATA)
        || SDCARD_STATUS_CURRENT_STATE(status) != SDCARD_STATE_TRANSFER) {
        usdhcRecordError(SDCARD_COMMAND_SEND_STATUS, SD_ERROR, status);
        return false;
    }
    return true;
}

SD_Error_t SD_GetCardInfo(void)
{
    if (!usdhc.initialised) {
        return SD_NOT_CONFIGURED;
    }
    memset(&SD_CardInfo, 0, sizeof(SD_CardInfo));

    const uint32_t *csd = usdhc.csd;
    const uint32_t *cid = usdhc.cid;

    SD_CardInfo.SD_csd.CSDStruct = (uint8_t)(csd[0] >> 30);
    SD_CardInfo.SD_csd.RdBlockLen = (uint8_t)((csd[1] >> 16) & 0x0F);

    if (SD_CardType == SD_HIGH_CAPACITY) {
        const uint32_t deviceSize = ((csd[1] & 0x3F) << 16) | (csd[2] >> 16);
        SD_CardInfo.SD_csd.DeviceSize = deviceSize;
        SD_CardInfo.CardBlockSize = SDCARD_BLOCK_SIZE;
        SD_CardInfo.CardCapacity = ((uint64_t)deviceSize + 1) * 1024;
    } else {
        const uint32_t deviceSize = ((csd[1] & 0x3FF) << 2) | (csd[2] >> 30);
        const uint32_t deviceSizeMul = (csd[2] >> 15) & 0x07;
        const uint32_t blockSize = 1u << SD_CardInfo.SD_csd.RdBlockLen;
        SD_CardInfo.SD_csd.DeviceSize = deviceSize;
        SD_CardInfo.SD_csd.DeviceSizeMul = (uint8_t)deviceSizeMul;
        SD_CardInfo.CardBlockSize = blockSize;
        SD_CardInfo.CardCapacity = (uint64_t)(deviceSize + 1) * (1u << (deviceSizeMul + 2))
                                 * blockSize / SDCARD_BLOCK_SIZE;
    }

    SD_CardInfo.SD_cid.ManufacturerID = (uint8_t)(cid[0] >> 24);
    SD_CardInfo.SD_cid.OEM_AppliID = (uint16_t)((cid[0] >> 8) & 0xFFFF);
    SD_CardInfo.SD_cid.ProdName1 = (cid[0] << 24) | (cid[1] >> 8);
    SD_CardInfo.SD_cid.ProdName2 = (uint8_t)(cid[1] & 0xFF);
    SD_CardInfo.SD_cid.ProdRev = (uint8_t)(cid[2] >> 24);
    SD_CardInfo.SD_cid.ProdSN = (cid[2] << 8) | (cid[3] >> 24);
    SD_CardInfo.SD_cid.ManufactDate = (uint16_t)((cid[3] >> 8) & 0x0FFF);

    return SD_OK;
}

SD_Error_t SD_ReadBlocks_DMA(uint64_t ReadAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    return usdhcStartTransfer(true, (uint32_t)ReadAddress, buffer, BlockSize, NumberOfBlocks);
}

SD_Error_t SD_CheckRead(void)
{
    return usdhcCheckTransfer();
}

SD_Error_t SD_WriteBlocks_DMA(uint64_t WriteAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    return usdhcStartTransfer(false, (uint32_t)WriteAddress, buffer, BlockSize, NumberOfBlocks);
}

SD_Error_t SD_CheckWrite(void)
{
    return usdhcCheckTransfer();
}

#endif // USE_SDCARD_SDIO
