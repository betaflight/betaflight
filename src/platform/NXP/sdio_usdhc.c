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
 * SD card over USDHC for drivers/sdmmc_sdio.h. Data moves by the USDHC's own
 * ADMA2 engine, not eDMA, and completion is polled from INT_STATUS.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SDCARD_SDIO

#include "common/utils.h"

#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/sdcard_standard.h"
#include "drivers/sdio.h"
#include "drivers/sdmmc_sdio.h"
#include "drivers/time.h"

#include "pg/sdio.h"

#include "platform/nxp_clock.h"

#include "iomux_nxp.h"
#include "iomux_signal_def.h"

// Identification runs at or below 400 kHz; 25 MHz is default speed mode.
#define USDHC_ID_CLOCK_HZ               400000u
#define USDHC_TRANSFER_CLOCK_HZ         25000000u

// Data timeout in SDCLK cycles is 2 raised to (14 + DTOCV): 9 gives about 339 ms
// at 24.75 MHz, above the 250 ms an SD card may hold the bus busy.
#define USDHC_DATA_TIMEOUT_DTOCV        9u

// Backstop against a wedged controller; CTOE normally fires long before this.
#define USDHC_COMMAND_TIMEOUT_US        100000u

#define USDHC_POWER_UP_TIMEOUT_US       1000000u

// Longer than the hardware data timeout, so reaching it means a stuck controller.
#define USDHC_TRANSFER_TIMEOUT_US       2000000u

// SD bus pads: CMD and data lines need pull ups; CLK has no pull. A zero pad
// control word keeps what the generic pad encoder wrote.
#if defined(IMXRT11XX)

#define USDHC_PAD_CFG_DATA  IOCFG_AF_PP_UP
#define USDHC_PAD_CFG_CLK   IOCFG_AF_PP
#define USDHC_PAD_CTL_DATA  0
#define USDHC_PAD_CTL_CLK   0

#elif defined(IMXRT10XX)

#define USDHC_PAD_CFG_DATA  IOCFG_AF_PP
#define USDHC_PAD_CFG_CLK   IOCFG_AF_PP

#define USDHC_PAD_CTL_DATA  (IOMUXC_SW_PAD_CTL_PAD_SRE(1)    /* fast slew */    \
                           | IOMUXC_SW_PAD_CTL_PAD_DSE(2)    /* R0/2 */         \
                           | IOMUXC_SW_PAD_CTL_PAD_SPEED(0)                     \
                           | IOMUXC_SW_PAD_CTL_PAD_PKE(1)                       \
                           | IOMUXC_SW_PAD_CTL_PAD_PUE(1)                       \
                           | IOMUXC_SW_PAD_CTL_PAD_PUS(1)    /* 47k pull up */  \
                           | IOMUXC_SW_PAD_CTL_PAD_HYS(1))

#define USDHC_PAD_CTL_CLK   (IOMUXC_SW_PAD_CTL_PAD_SRE(1)                       \
                           | IOMUXC_SW_PAD_CTL_PAD_DSE(2)                       \
                           | IOMUXC_SW_PAD_CTL_PAD_SPEED(3)) /* 200 MHz pad */

#else
#error "Unsupported i.MXRT family: no uSDHC pad settings"
#endif

// Card commands and status bits beyond those drivers/sdcard_standard.h names.

#define SDCARD_COMMAND_ALL_SEND_CID             2
#define SDCARD_COMMAND_SEND_RELATIVE_ADDR       3
#define SDCARD_COMMAND_SELECT_CARD              7

#define SDCARD_ACOMMAND_SET_BUS_WIDTH           6

// CMD8 argument: 2.7-3.6 V and check pattern 0xAA, echoed back by the card.
#define SDCARD_IF_COND_ARGUMENT                 0x1AAu

// ACMD41: host capacity support in bit 30, the 2.7-3.6 V window in bits 23-15.
#define SDCARD_OP_COND_VOLTAGE_WINDOW           0x00FF8000u
#define SDCARD_OP_COND_HIGH_CAPACITY            0x40000000u

#define SDCARD_OCR_POWER_UP_DONE                0x80000000u
#define SDCARD_OCR_CARD_CAPACITY_STATUS         0x40000000u

// Card status, as returned in an R1 response.
#define SDCARD_STATUS_READY_FOR_DATA            (1u << 8)
#define SDCARD_STATUS_CURRENT_STATE(status)     (((status) >> 9) & 0x0F)
#define SDCARD_STATE_TRANSFER                   4

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

// Every write one to clear INT_STATUS bit this part implements.
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

// 32 bit ADMA2 descriptor: attributes with the byte count in the top half, then
// the address. The list must be four byte aligned.
typedef struct usdhcAdmaDescriptor_s {
    uint32_t attributes;
    uint32_t address;
} usdhcAdmaDescriptor_t;

#define USDHC_ADMA_VALID            (1u << 0)
#define USDHC_ADMA_END              (1u << 1)
#define USDHC_ADMA_ACT_TRAN         (2u << 4)
#define USDHC_ADMA_LENGTH(bytes)    ((uint32_t)(bytes) << 16)

// The 16 bit length field reads zero as 65536; stay a power of two below it.
#define USDHC_ADMA_MAX_BYTES        32768u

// Eight descriptors cover a 256 KB transfer.
#define USDHC_ADMA_DESCRIPTORS      8

static DMA_DATA usdhcAdmaDescriptor_t usdhcAdmaTable[USDHC_ADMA_DESCRIPTORS];

STATIC_ASSERT(sizeof(usdhcAdmaDescriptor_t) == 8, adma2_descriptor_must_be_two_words);
STATIC_ASSERT(__alignof__(usdhcAdmaTable) >= 4, adma2_descriptor_table_must_be_word_aligned);

typedef enum {
    USDHC_RESPONSE_NONE = 0,
    USDHC_RESPONSE_R1,      // 48 bit, index and CRC checked
    USDHC_RESPONSE_R1B,     // 48 bit, then the card holds DAT0 low while busy
    USDHC_RESPONSE_R2,      // 136 bit, CRC checked, carries no index
    USDHC_RESPONSE_R3,      // 48 bit with neither, which is how OCR comes back
} usdhcResponse_e;

typedef struct usdhcState_s {
    bool initialised;
    bool transferActive;
    uint8_t transferCommand;
    uint32_t transferStartUs;

    // Reported once through SD_GetState() and then cleared.
    SD_Error_t pendingError;

    uint16_t rca;
    bool highCapacity;
    bool fourBitBus;

    uint32_t csd[4];
    uint32_t cid[4];
} usdhcState_t;

static usdhcState_t usdhc;

// Last card bus failure for reading over a debugger: the error, the command
// (USDHC_NO_COMMAND if none was sent) and INT_STATUS or CMD13 status. Never cleared.
#define USDHC_NO_COMMAND    0xFFu

static volatile struct {
    SD_Error_t error;
    uint8_t command;
    uint32_t detail;
} usdhcLastError;

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

// SDCLK = root / (prescaler 2..256, SDCLKFS = prescaler / 2) / (divisor 1..16,
// DVS = divisor - 1). Returns the fastest setting not above targetHz.
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

static bool usdhcSetClock(uint32_t targetHz)
{
    // Zero means the root rate is unknown.
    const uint32_t rootHz = nxpClockHz(NXP_CLK_USDHC1);
    if (rootHz == 0) {
        return false;
    }

    const uint32_t divider = usdhcClockDivider(rootHz, targetHz, NULL);

    USDHC1->SYS_CTRL = (USDHC1->SYS_CTRL & ~(USDHC_SYS_CTRL_SDCLKFS_MASK | USDHC_SYS_CTRL_DVS_MASK))
                     | divider;

    return usdhcWaitFlag(&USDHC1->PRES_STATE, USDHC_PRES_STATE_SDSTB_MASK, true);
}

// A board with no card slot names no pins, and pad setup then fails.
#ifndef SDIO_CK_PIN
#define SDIO_CK_PIN     NONE
#endif
#ifndef SDIO_CMD_PIN
#define SDIO_CMD_PIN    NONE
#endif
#ifndef SDIO_D0_PIN
#define SDIO_D0_PIN     NONE
#endif
#ifndef SDIO_D1_PIN
#define SDIO_D1_PIN     NONE
#endif
#ifndef SDIO_D2_PIN
#define SDIO_D2_PIN     NONE
#endif
#ifndef SDIO_D3_PIN
#define SDIO_D3_PIN     NONE
#endif

// A non zero padControl overrides the pad control word written from cfg.
static bool usdhcConfigPad(uint8_t role, ioTag_t pin, ioConfig_t cfg, uint32_t padControl, resourceOwner_e owner)
{
    const nxpPinMux_t *mux = NXP_PIN_MUX(nxpUsdhcPins, 1, role, pin);
    if (!mux || !mux->configRegister) {
        return false;
    }

    IOInit(IOGetByTag(pin), owner, 0);
    nxpIomuxConfigPin(mux, cfg);

    if (padControl) {
        *(volatile uint32_t *)(uintptr_t)mux->configRegister = padControl;
    }

    return true;
}

static bool usdhcConfigPads(bool fourBitBus)
{
    bool ok = usdhcConfigPad(NXP_ROLE_USDHC_CLK, IO_TAG(SDIO_CK_PIN),
                             USDHC_PAD_CFG_CLK, USDHC_PAD_CTL_CLK, OWNER_SDIO_CK)
            && usdhcConfigPad(NXP_ROLE_USDHC_CMD, IO_TAG(SDIO_CMD_PIN),
                              USDHC_PAD_CFG_DATA, USDHC_PAD_CTL_DATA, OWNER_SDIO_CMD)
            && usdhcConfigPad(NXP_ROLE_USDHC_DATA0, IO_TAG(SDIO_D0_PIN),
                              USDHC_PAD_CFG_DATA, USDHC_PAD_CTL_DATA, OWNER_SDIO_D0);

    if (ok && fourBitBus) {
        ok = usdhcConfigPad(NXP_ROLE_USDHC_DATA1, IO_TAG(SDIO_D1_PIN),
                            USDHC_PAD_CFG_DATA, USDHC_PAD_CTL_DATA, OWNER_SDIO_D1)
          && usdhcConfigPad(NXP_ROLE_USDHC_DATA2, IO_TAG(SDIO_D2_PIN),
                            USDHC_PAD_CFG_DATA, USDHC_PAD_CTL_DATA, OWNER_SDIO_D2)
          && usdhcConfigPad(NXP_ROLE_USDHC_DATA3, IO_TAG(SDIO_D3_PIN),
                            USDHC_PAD_CFG_DATA, USDHC_PAD_CTL_DATA, OWNER_SDIO_D3);
    }

    return ok;
}

static void usdhcSetBusWidth(bool fourBit)
{
    USDHC1->PROT_CTRL = (USDHC1->PROT_CTRL & ~USDHC_PROT_CTRL_DTW_MASK)
                      | USDHC_PROT_CTRL_DTW(fourBit ? 1 : 0);
}

static bool usdhcInitController(void)
{
    USDHC1->INT_SIGNAL_EN = 0;

    USDHC1->SYS_CTRL |= USDHC_SYS_CTRL_RSTA_MASK;
    if (!usdhcWaitFlag(&USDHC1->SYS_CTRL, USDHC_SYS_CTRL_RSTA_MASK, false)) {
        return false;
    }

    USDHC1->INT_SIGNAL_EN = 0;
    USDHC1->INT_STATUS_EN = USDHC_INT_ALL;
    USDHC1->INT_STATUS = USDHC_INT_ALL;

    // Little endian, one bit bus, DMASEL 2 for ADMA2.
    USDHC1->PROT_CTRL = (USDHC1->PROT_CTRL
                         & ~(USDHC_PROT_CTRL_EMODE_MASK | USDHC_PROT_CTRL_DTW_MASK
                             | USDHC_PROT_CTRL_DMASEL_MASK))
                      | USDHC_PROT_CTRL_EMODE(2)
                      | USDHC_PROT_CTRL_DTW(0)
                      | USDHC_PROT_CTRL_DMASEL(2);

    USDHC1->SYS_CTRL = (USDHC1->SYS_CTRL & ~USDHC_SYS_CTRL_DTOCV_MASK)
                     | USDHC_SYS_CTRL_DTOCV(USDHC_DATA_TIMEOUT_DTOCV);

    return usdhcSetClock(USDHC_ID_CLOCK_HZ);
}

// Send the 80 clock cycles a card needs to power up before it will answer.
static bool usdhcSendInitClocks(void)
{
    USDHC1->SYS_CTRL |= USDHC_SYS_CTRL_INITA_MASK;
    return usdhcWaitFlag(&USDHC1->SYS_CTRL, USDHC_SYS_CTRL_INITA_MASK, false);
}

// R3 (and R7, sent as R3) carries no valid index or CRC, so neither is checked.
static uint32_t usdhcTransferType(uint8_t index, usdhcResponse_e response, bool dataPresent)
{
    uint32_t value = USDHC_CMD_XFR_TYP_CMDINX(index);

    switch (response) {
    case USDHC_RESPONSE_NONE:
        break;
    case USDHC_RESPONSE_R1:
        value |= USDHC_CMD_XFR_TYP_RSPTYP(2)
               | USDHC_CMD_XFR_TYP_CICEN_MASK | USDHC_CMD_XFR_TYP_CCCEN_MASK;
        break;
    case USDHC_RESPONSE_R1B:
        value |= USDHC_CMD_XFR_TYP_RSPTYP(3)
               | USDHC_CMD_XFR_TYP_CICEN_MASK | USDHC_CMD_XFR_TYP_CCCEN_MASK;
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

// Reset the command line after a failed command, the data line after a failed transfer.
static void usdhcResetLines(bool data)
{
    USDHC1->SYS_CTRL |= USDHC_SYS_CTRL_RSTC_MASK | (data ? USDHC_SYS_CTRL_RSTD_MASK : 0);
    usdhcWaitFlag(&USDHC1->SYS_CTRL,
                  USDHC_SYS_CTRL_RSTC_MASK | USDHC_SYS_CTRL_RSTD_MASK, false);

    USDHC1->INT_STATUS = USDHC_INT_ALL;
}

// Wait out an R1b busy phase. TC is only raised if the card went busy, so the
// data line inhibit is checked too.
static SD_Error_t usdhcWaitBusyEnd(void)
{
    const uint32_t start = micros();

    for (;;) {
        const uint32_t status = USDHC1->INT_STATUS;

        if (status & USDHC_INT_DATA_ERRORS) {
            const SD_Error_t error = usdhcDataError(status);
            usdhcResetLines(true);
            return error;
        }

        if ((status & USDHC_INT_STATUS_TC_MASK)
            || !(USDHC1->PRES_STATE & USDHC_PRES_STATE_CDIHB_MASK)) {
            USDHC1->INT_STATUS = USDHC_INT_STATUS_TC_MASK;
            return SD_OK;
        }

        if (cmpTimeUs(micros(), start) > (timeDelta_t)USDHC_TRANSFER_TIMEOUT_US) {
            usdhcResetLines(true);
            return SD_DATA_TIMEOUT;
        }
    }
}

static FAST_CODE SD_Error_t usdhcSendCommandFull(uint8_t index, uint32_t argument, usdhcResponse_e response,
                                                 bool dataPresent, uint32_t mixControl)
{
    uint32_t inhibit = USDHC_PRES_STATE_CIHB_MASK;
    if (dataPresent || response == USDHC_RESPONSE_R1B) {
        inhibit |= USDHC_PRES_STATE_CDIHB_MASK;
    }

    const uint32_t inhibitStart = micros();
    while (USDHC1->PRES_STATE & inhibit) {
        if (cmpTimeUs(micros(), inhibitStart) > (timeDelta_t)USDHC_COMMAND_TIMEOUT_US) {
            usdhcResetLines(dataPresent);
            return SD_BUSY;
        }
    }

    USDHC1->INT_STATUS = USDHC_INT_ALL;
    USDHC1->CMD_ARG = argument;

    // MIX_CTRL holds the transfer mode; its reserved bits must keep their value.
    const uint32_t mixMask = USDHC_MIX_CTRL_DMAEN_MASK | USDHC_MIX_CTRL_BCEN_MASK
                           | USDHC_MIX_CTRL_AC12EN_MASK | USDHC_MIX_CTRL_DTDSEL_MASK
                           | USDHC_MIX_CTRL_MSBSEL_MASK;
    USDHC1->MIX_CTRL = (USDHC1->MIX_CTRL & ~mixMask) | (mixControl & mixMask);

    USDHC1->CMD_XFR_TYP = usdhcTransferType(index, response, dataPresent);

    const uint32_t commandStart = micros();
    uint32_t status;
    for (;;) {
        status = USDHC1->INT_STATUS;
        if (status & (USDHC_INT_STATUS_CC_MASK | USDHC_INT_COMMAND_ERRORS)) {
            break;
        }
        if (cmpTimeUs(micros(), commandStart) > (timeDelta_t)USDHC_COMMAND_TIMEOUT_US) {
            usdhcResetLines(dataPresent);
            return SD_CMD_RSP_TIMEOUT;
        }
    }

    const SD_Error_t error = usdhcCommandError(status);
    if (error != SD_OK) {
        usdhcResetLines(dataPresent);
        return error;
    }

    USDHC1->INT_STATUS = USDHC_INT_STATUS_CC_MASK;

    // A data command's busy end is polled by the caller instead.
    if (response == USDHC_RESPONSE_R1B && !dataPresent) {
        return usdhcWaitBusyEnd();
    }

    return SD_OK;
}

static SD_Error_t usdhcSendCommand(uint8_t index, uint32_t argument, usdhcResponse_e response)
{
    return usdhcRecordError(index, usdhcSendCommandFull(index, argument, response, false, 0), 0);
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

// R2 arrives without its CRC byte, shifted right by eight; shift it back so
// word 0 holds bits 127..96.
static void usdhcReadLongResponse(uint32_t out[4])
{
    const uint32_t rsp0 = USDHC1->CMD_RSP0;
    const uint32_t rsp1 = USDHC1->CMD_RSP1;
    const uint32_t rsp2 = USDHC1->CMD_RSP2;
    const uint32_t rsp3 = USDHC1->CMD_RSP3;

    out[0] = (rsp3 << 8) | (rsp2 >> 24);
    out[1] = (rsp2 << 8) | (rsp1 >> 24);
    out[2] = (rsp1 << 8) | (rsp0 >> 24);
    out[3] = (rsp0 << 8);
}

// High capacity cards are addressed in blocks, standard capacity in bytes.
static uint32_t usdhcBlockAddress(uint32_t blockIndex)
{
    return usdhc.highCapacity ? blockIndex : blockIndex * SDCARD_BLOCK_SIZE;
}

static bool usdhcBuildAdmaTable(const void *buffer, uint32_t bytes)
{
    if (bytes == 0 || ((uintptr_t)buffer & 3) != 0) {
        return false;
    }

    uintptr_t address = (uintptr_t)buffer;
    unsigned index = 0;

    while (bytes > 0) {
        if (index >= USDHC_ADMA_DESCRIPTORS) {
            return false;
        }

        const uint32_t chunk = bytes > USDHC_ADMA_MAX_BYTES ? USDHC_ADMA_MAX_BYTES : bytes;

        usdhcAdmaTable[index].attributes = USDHC_ADMA_LENGTH(chunk)
                                         | USDHC_ADMA_ACT_TRAN | USDHC_ADMA_VALID;
        usdhcAdmaTable[index].address = (uint32_t)address;

        address += chunk;
        bytes -= chunk;
        index++;
    }

    usdhcAdmaTable[index - 1].attributes |= USDHC_ADMA_END;

    return true;
}

static FAST_CODE SD_Error_t usdhcStartTransfer(bool read, uint32_t blockIndex, uint32_t *buffer,
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

    const uint32_t bytes = blockCount * blockSize;
    if (!usdhcBuildAdmaTable(buffer, bytes)) {
        return usdhcRecordError(USDHC_NO_COMMAND, SD_INVALID_PARAMETER, 0);
    }

    // Order the descriptor and buffer stores before the transfer start.
    __DMB();

    USDHC1->ADMA_SYS_ADDR = (uint32_t)(uintptr_t)usdhcAdmaTable;
    USDHC1->BLK_ATT = USDHC_BLK_ATT_BLKSIZE(blockSize) | USDHC_BLK_ATT_BLKCNT(blockCount);

    uint32_t mix = USDHC_MIX_CTRL_DMAEN_MASK | (read ? USDHC_MIX_CTRL_DTDSEL_MASK : 0);
    uint8_t command;

    if (blockCount > 1) {
        // The block counter ends the transfer and the controller sends CMD12.
        mix |= USDHC_MIX_CTRL_MSBSEL_MASK | USDHC_MIX_CTRL_BCEN_MASK | USDHC_MIX_CTRL_AC12EN_MASK;
        command = read ? SDCARD_COMMAND_READ_MULTIPLE_BLOCK : SDCARD_COMMAND_WRITE_MULTIPLE_BLOCK;
    } else {
        command = read ? SDCARD_COMMAND_READ_SINGLE_BLOCK : SDCARD_COMMAND_WRITE_BLOCK;
    }

    usdhc.pendingError = SD_OK;

    const SD_Error_t error = usdhcRecordError(command,
                                              usdhcSendCommandFull(command, usdhcBlockAddress(blockIndex),
                                                                   USDHC_RESPONSE_R1, true, mix), 0);
    if (error != SD_OK) {
        return error;
    }

    usdhc.transferActive = true;
    usdhc.transferCommand = command;
    usdhc.transferStartUs = micros();

    return SD_OK;
}

// SD_BUSY while running, SD_OK once stopped; a failure is reported by SD_GetState().
static FAST_CODE SD_Error_t usdhcCheckTransfer(void)
{
    if (!usdhc.transferActive) {
        return SD_OK;
    }

    const uint32_t status = USDHC1->INT_STATUS;

    if (status & USDHC_INT_DATA_ERRORS) {
        usdhc.pendingError = usdhcRecordError(usdhc.transferCommand, usdhcDataError(status), status);
        usdhc.transferActive = false;
        usdhcResetLines(true);
        return SD_OK;
    }

    if (status & USDHC_INT_STATUS_TC_MASK) {
        USDHC1->INT_STATUS = USDHC_INT_STATUS_TC_MASK | USDHC_INT_STATUS_DINT_MASK;
        usdhc.transferActive = false;

        return SD_OK;
    }

    if (cmpTimeUs(micros(), usdhc.transferStartUs) > (timeDelta_t)USDHC_TRANSFER_TIMEOUT_US) {
        usdhc.pendingError = usdhcRecordError(usdhc.transferCommand, SD_DATA_TIMEOUT, status);
        usdhc.transferActive = false;
        usdhcResetLines(true);
        return SD_OK;
    }

    return SD_BUSY;
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

    // A version 1 card does not answer CMD8; a version 2 card echoes the pattern.
    bool version2 = false;

    error = usdhcSendCommandFull(SDCARD_COMMAND_SEND_IF_COND, SDCARD_IF_COND_ARGUMENT,
                                 USDHC_RESPONSE_R3, false, 0);
    if (error == SD_OK) {
        if ((USDHC1->CMD_RSP0 & 0xFFF) != SDCARD_IF_COND_ARGUMENT) {
            return usdhcRecordError(SDCARD_COMMAND_SEND_IF_COND, SD_INVALID_VOLTRANGE,
                                    USDHC1->CMD_RSP0);
        }
        version2 = true;
    } else if (error != SD_CMD_RSP_TIMEOUT) {
        return error;
    }

    // Repeat ACMD41 until power up is done; only a version 2 card may be offered high capacity.
    const uint32_t argument = SDCARD_OP_COND_VOLTAGE_WINDOW
                            | (version2 ? SDCARD_OP_COND_HIGH_CAPACITY : 0);
    const uint32_t start = micros();
    uint32_t ocr;

    for (;;) {
        error = usdhcSendAppCommand(SDCARD_ACOMMAND_SEND_OP_COND, argument, USDHC_RESPONSE_R3);
        if (error != SD_OK) {
            return error;
        }

        ocr = USDHC1->CMD_RSP0;
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

    // CMD2 reads the CID, CMD3 the address, CMD9 the CSD.
    error = usdhcSendCommand(SDCARD_COMMAND_ALL_SEND_CID, 0, USDHC_RESPONSE_R2);
    if (error != SD_OK) {
        return error;
    }
    usdhcReadLongResponse(usdhc.cid);

    error = usdhcSendCommand(SDCARD_COMMAND_SEND_RELATIVE_ADDR, 0, USDHC_RESPONSE_R1);
    if (error != SD_OK) {
        return error;
    }
    usdhc.rca = (uint16_t)(USDHC1->CMD_RSP0 >> 16);

    error = usdhcSendCommand(SDCARD_COMMAND_SEND_CSD, (uint32_t)usdhc.rca << 16, USDHC_RESPONSE_R2);
    if (error != SD_OK) {
        return error;
    }
    usdhcReadLongResponse(usdhc.csd);

    // CMD7 selects the card.
    error = usdhcSendCommand(SDCARD_COMMAND_SELECT_CARD, (uint32_t)usdhc.rca << 16,
                             USDHC_RESPONSE_R1B);
    if (error != SD_OK) {
        return error;
    }

    error = usdhcSendCommand(SDCARD_COMMAND_SET_BLOCKLEN, SDCARD_BLOCK_SIZE, USDHC_RESPONSE_R1);
    if (error != SD_OK) {
        return error;
    }

    // Set the bus width on the card first, then the controller.
    if (usdhc.fourBitBus) {
        error = usdhcSendAppCommand(SDCARD_ACOMMAND_SET_BUS_WIDTH, SDCARD_ACMD6_BUS_WIDTH_4BIT,
                                    USDHC_RESPONSE_R1);
        if (error != SD_OK) {
            return error;
        }
        usdhcSetBusWidth(true);
    }

    if (!usdhcSetClock(USDHC_TRANSFER_CLOCK_HZ)) {
        return usdhcRecordError(USDHC_NO_COMMAND, SD_ERROR, 0);
    }

    return SD_OK;
}

bool SD_InitialiseHardware(dmaResource_t *dma)
{
    UNUSED(dma);

    // Only USDHC1 is supported.
    if (SDIO_CFG_TO_DEV(sdioConfig()->device) != SDIODEV_1) {
        return false;
    }

    memset(&usdhc, 0, sizeof(usdhc));
    usdhc.fourBitBus = sdioConfig()->use4BitWidth;

    if (!usdhcConfigPads(usdhc.fourBitBus)) {
        return false;
    }

    nxpClockGateEnable(NXP_CLK_USDHC1);

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

// No card detect pin: a card is present if identification found one.
bool SD_IsDetected(void)
{
    return usdhc.initialised;
}

// Is the card ready for another command? A failed transfer is reported once.
FAST_CODE bool SD_GetState(void)
{
    if (!usdhc.initialised || usdhc.transferActive) {
        return false;
    }

    if (usdhc.pendingError != SD_OK) {
        usdhc.pendingError = SD_OK;
        return false;
    }

    if (usdhcSendCommand(SDCARD_COMMAND_SEND_STATUS, (uint32_t)usdhc.rca << 16,
                         USDHC_RESPONSE_R1) != SD_OK) {
        return false;
    }

    const uint32_t status = USDHC1->CMD_RSP0;

    if (!(status & SDCARD_STATUS_READY_FOR_DATA)
        || SDCARD_STATUS_CURRENT_STATE(status) != SDCARD_STATE_TRANSFER) {
        usdhcRecordError(SDCARD_COMMAND_SEND_STATUS, SD_ERROR, status);
        return false;
    }

    return true;
}

// Fills in the CID fields and the capacity in 512 byte blocks; the rest stays zero.
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
        // CSD version 2: (C_SIZE + 1) * 1024 blocks.
        const uint32_t deviceSize = ((csd[1] & 0x3F) << 16) | (csd[2] >> 16);

        SD_CardInfo.SD_csd.DeviceSize = deviceSize;
        SD_CardInfo.CardBlockSize = SDCARD_BLOCK_SIZE;
        SD_CardInfo.CardCapacity = ((uint64_t)deviceSize + 1) * 1024;
    } else {
        // CSD version 1: size, multiplier and read block length, in 512 byte blocks.
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

FAST_CODE SD_Error_t SD_ReadBlocks_DMA(uint64_t ReadAddress, uint32_t *buffer, uint32_t BlockSize,
                                       uint32_t NumberOfBlocks)
{
    return usdhcStartTransfer(true, (uint32_t)ReadAddress, buffer, BlockSize, NumberOfBlocks);
}

FAST_CODE SD_Error_t SD_CheckRead(void)
{
    return usdhcCheckTransfer();
}

FAST_CODE SD_Error_t SD_WriteBlocks_DMA(uint64_t WriteAddress, uint32_t *buffer, uint32_t BlockSize,
                                        uint32_t NumberOfBlocks)
{
    return usdhcStartTransfer(false, (uint32_t)WriteAddress, buffer, BlockSize, NumberOfBlocks);
}

FAST_CODE SD_Error_t SD_CheckWrite(void)
{
    return usdhcCheckTransfer();
}

#endif // USE_SDCARD_SDIO
