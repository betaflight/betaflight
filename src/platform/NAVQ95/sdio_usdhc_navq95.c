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
 * sdio_usdhc_navq95.c — SD card over the i.MX 95 uSDHC controller for the
 * Betaflight SDIO baremetal glue (drivers/sdcard_sdio_baremetal.c -> SD_* HAL
 * in drivers/sdmmc_sdio.h). Data moves by the uSDHC's OWN ADMA2 engine (not the
 * SoC eDMA), and completion is polled from INT_STATUS.
 *
 * The command sequence / ADMA2 / clock-divider / card-identify LOGIC is ported
 * from Betaflight PR #15853 (PetervdPerk, src/platform/NXP/sdio_usdhc.c, GPLv3).
 * The three platform dependencies of that file are replaced for NavQ95:
 *   (1) the MCUXpresso-SDK USDHC1-> register block  -> raw USDHC_Type + masks
 *       (include/platform/usdhc_regs.h, offsets from NuttX imx9_usdhc.h);
 *   (2) nxpClockHz()/nxpClockGateEnable()           -> SCMI clock-enable to the
 *       System Manager (the M7 cannot gate CCM roots directly) + a fixed root Hz;
 *   (3) nxpUsdhcPins/NXP_PIN_MUX/nxpIomuxConfigPin   -> SCMI PINCTRL_CONFIG_SET
 *       to the SM (the M7 cannot write IOMUXC directly), hardcoded SD pads.
 *
 * *** BEST-GUESS WIRING — VERIFY ON HARDWARE ***
 * No source (PX4 imx95 board, Zephyr mr_navq95b, Zephyr imx95, PR #15853) wires
 * the NavQ95 microSD, so the instance + pads below are a best guess:
 *   - uSDHC1 (0x42850000), CCM root 117, i.MX95 "SD1" pads at ALT0.
 * If the boot self-test reports no card / CMD0 timeout, correct NAVQ95_USDHC_*
 * and the nav95SdPads[] table against the NavQ95 schematic / Linux DTS.
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

/* ════════════════════════════════════════════════════════════════════════════
 * NavQ95 platform wiring (BEST GUESS — verify on HW; see file header).
 * ════════════════════════════════════════════════════════════════════════════ */

/* Controller instance. Peter's driver assumed uSDHC1; we do the same. */
#define NAVQ95_USDHC            ((USDHC_Type *)USDHC1_BASE)
#define NAVQ95_USDHC_CLK_ROOT   117u            /* CCM root for uSDHC1          */

/* uSDHC functional (base) clock in Hz. The SCMI CLOCK_CONFIG_SET we issue only
 * ungates the root at the SM's default parent/rate; we ASSUME ~200 MHz here
 * (the i.MX93-EVK value) for the SDCLK divider math. If the card enumerates but
 * runs at the wrong speed, this is the number to fix (or extend SCMI to set the
 * parent + rate). */
#define NAVQ95_USDHC_ROOT_HZ    200000000u

/* ── SCMI transport (shared with platform_stubs.c, re-declared here; the SMT
 * channel is polled/synchronous so there is no concurrency). These helpers are
 * defined in platform_stubs.c. ───────────────────────────────────────────── */
extern int  navq95_scmi_clock_enable_ex(uint32_t clockRootId);
extern int  navq95_scmi_pinmux_ex(uint32_t muxReg, uint32_t muxMode, uint32_t sion,
                                  uint32_t padReg, uint32_t padVal);
#define navq95_scmi_clock_enable  navq95_scmi_clock_enable_ex
#define navq95_scmi_pinmux        navq95_scmi_pinmux_ex

/*
 * i.MX95 "SD1" pads (best guess), from the i.MX93 pinmux table shifted to the
 * i.MX95 IOMUXC map. MUX/PAD are absolute IOMUXC addresses; ALT = 0.
 * CLK needs no pull; CMD + DATA need a pull-up + hysteresis so idle-high lines
 * read. Pad CONFIG values are PRE-SHIFTED (>>1); the SM re-applies <<1.
 *   CLK  : raw 0x1E (FSEL | DSE)                -> send 0x0F
 *   CMD/D: raw 0x1200 | DSE (PU | HYS | drive)  -> send 0x090F
 */
#define IOMUXC_BASE             0x443C0000u
#define NAVQ95_SD_PAD_CLK       (0x1Eu   >> 1)   /* 0x0F  */
#define NAVQ95_SD_PAD_DATA      (0x121Eu >> 1)   /* 0x090F : PU|HYS + drive */

typedef struct {
    uint32_t muxReg;
    uint32_t padReg;
    uint32_t padVal;
    uint8_t  dataLine;   /* 1 = CMD/DAT (needs the self-test name); 0 = CLK */
} navq95SdPad_t;

/* ALT0 for all SD1 signals. Offsets are the AUTHORITATIVE i.MX95 SD1 block from
 * imx95_iomuxc.h (MUX_CTL_SD1_* 0x0128..0x013C, PAD_CTL_SD1_* 0x032C..0x0340).
 * No SELECT_INPUT/daisy for USDHC1 (the pinmux header marks the daisy register
 * NOT_AVAILABLE for the USDHC1 function). Order: CLK, CMD, DAT0, DAT1, DAT2,
 * DAT3. (Earlier revisions used i.MX93 offsets by mistake -> the SM returned
 * -3 / SM_ERR_DENIED on the pinmux because the pin identifiers were wrong.) */
static const navq95SdPad_t navq95SdPads[] = {
    { IOMUXC_BASE + 0x0128u, IOMUXC_BASE + 0x032Cu, NAVQ95_SD_PAD_CLK,  0 }, /* CLK  */
    { IOMUXC_BASE + 0x012Cu, IOMUXC_BASE + 0x0330u, NAVQ95_SD_PAD_DATA, 1 }, /* CMD  */
    { IOMUXC_BASE + 0x0130u, IOMUXC_BASE + 0x0334u, NAVQ95_SD_PAD_DATA, 1 }, /* DAT0 */
    { IOMUXC_BASE + 0x0134u, IOMUXC_BASE + 0x0338u, NAVQ95_SD_PAD_DATA, 1 }, /* DAT1 */
    { IOMUXC_BASE + 0x0138u, IOMUXC_BASE + 0x033Cu, NAVQ95_SD_PAD_DATA, 1 }, /* DAT2 */
    { IOMUXC_BASE + 0x013Cu, IOMUXC_BASE + 0x0340u, NAVQ95_SD_PAD_DATA, 1 }, /* DAT3 */
};
#define NAVQ95_SD_ALT           0u

/* ════════════════════════════════════════════════════════════════════════════
 * The rest below is Peter's driver logic, retargeted to NAVQ95_USDHC-> and the
 * NavQ95 clock/pinmux helpers above. Behaviour is unchanged.
 * ════════════════════════════════════════════════════════════════════════════ */

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

typedef struct usdhcAdmaDescriptor_s {
    uint32_t attributes;
    uint32_t address;
} usdhcAdmaDescriptor_t;

#define USDHC_ADMA_VALID            (1u << 0)
#define USDHC_ADMA_END              (1u << 1)
#define USDHC_ADMA_ACT_TRAN         (2u << 4)
#define USDHC_ADMA_LENGTH(bytes)    ((uint32_t)(bytes) << 16)
#define USDHC_ADMA_MAX_BYTES        32768u
#define USDHC_ADMA_DESCRIPTORS      8

/* The ADMA2 descriptor table must be word-aligned and live in SRAM the uSDHC
 * bus master can read. The M7 SRAM region (0x20008000) is a flat physical
 * address with no MMU remap, so a plain static in .bss is reachable as-is. */
static usdhcAdmaDescriptor_t usdhcAdmaTable[USDHC_ADMA_DESCRIPTORS] __attribute__((aligned(4)));

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
    uint8_t transferCommand;
    uint32_t transferStartUs;
    SD_Error_t pendingError;
    uint16_t rca;
    bool highCapacity;
    bool fourBitBus;
    uint32_t csd[4];
    uint32_t cid[4];
} usdhcState_t;

static usdhcState_t usdhc;

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
    const uint32_t rootHz = NAVQ95_USDHC_ROOT_HZ;
    if (rootHz == 0) {
        return false;
    }
    const uint32_t divider = usdhcClockDivider(rootHz, targetHz, NULL);
    U->SYS_CTRL = (U->SYS_CTRL & ~(USDHC_SYS_CTRL_SDCLKFS_MASK | USDHC_SYS_CTRL_DVS_MASK)) | divider;
    return usdhcWaitFlag(&U->PRES_STATE, USDHC_PRES_STATE_SDSTB_MASK, true);
}

/* NavQ95 pad mux over SCMI (the SM owns IOMUXC). Returns 0 on success. */
static int usdhcConfigPads(bool fourBitBus)
{
    const unsigned count = fourBitBus ? 6u : 3u;   /* CLK,CMD,D0 [,D1,D2,D3] */
    for (unsigned i = 0; i < count; i++) {
        const navq95SdPad_t *p = &navq95SdPads[i];
        /* SION on CMD/DATA so the input path is forced (ERR052021-style). */
        int s = navq95_scmi_pinmux(p->muxReg, NAVQ95_SD_ALT, p->dataLine ? 1u : 0u,
                                   p->padReg, p->padVal);
        if (s != 0) {
            return s;
        }
    }
    return 0;
}

static void usdhcSetBusWidth(bool fourBit)
{
    U->PROT_CTRL = (U->PROT_CTRL & ~USDHC_PROT_CTRL_DTW_MASK) | USDHC_PROT_CTRL_DTW(fourBit ? 1 : 0);
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

    /* Little endian, one bit bus, DMASEL 2 for ADMA2. */
    U->PROT_CTRL = (U->PROT_CTRL
                    & ~(USDHC_PROT_CTRL_EMODE_MASK | USDHC_PROT_CTRL_DTW_MASK
                        | USDHC_PROT_CTRL_DMASEL_MASK))
                 | USDHC_PROT_CTRL_EMODE(2)
                 | USDHC_PROT_CTRL_DTW(0)
                 | USDHC_PROT_CTRL_DMASEL(2);

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
        usdhcAdmaTable[index].attributes = USDHC_ADMA_LENGTH(chunk) | USDHC_ADMA_ACT_TRAN | USDHC_ADMA_VALID;
        usdhcAdmaTable[index].address = (uint32_t)address;
        address += chunk;
        bytes -= chunk;
        index++;
    }
    usdhcAdmaTable[index - 1].attributes |= USDHC_ADMA_END;
    return true;
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

    const uint32_t bytes = blockCount * blockSize;
    if (!usdhcBuildAdmaTable(buffer, bytes)) {
        return usdhcRecordError(USDHC_NO_COMMAND, SD_INVALID_PARAMETER, 0);
    }

    __DMB();

    U->ADMA_SYS_ADDR = (uint32_t)(uintptr_t)usdhcAdmaTable;
    U->BLK_ATT = USDHC_BLK_ATT_BLKSIZE(blockSize) | USDHC_BLK_ATT_BLKCNT(blockCount);

    uint32_t mix = USDHC_MIX_CTRL_DMAEN_MASK | (read ? USDHC_MIX_CTRL_DTDSEL_MASK : 0);
    uint8_t command;
    if (blockCount > 1) {
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

static SD_Error_t usdhcCheckTransfer(void)
{
    if (!usdhc.transferActive) {
        return SD_OK;
    }
    const uint32_t status = U->INT_STATUS;

    if (status & USDHC_INT_DATA_ERRORS) {
        usdhc.pendingError = usdhcRecordError(usdhc.transferCommand, usdhcDataError(status), status);
        usdhc.transferActive = false;
        usdhcResetLines(true);
        return SD_OK;
    }
    if (status & USDHC_INT_STATUS_TC_MASK) {
        U->INT_STATUS = USDHC_INT_STATUS_TC_MASK | USDHC_INT_STATUS_DINT_MASK;
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

    bool version2 = false;
    error = usdhcSendCommandFull(SDCARD_COMMAND_SEND_IF_COND, SDCARD_IF_COND_ARGUMENT,
                                 USDHC_RESPONSE_R3, false, 0);
    if (error == SD_OK) {
        if ((U->CMD_RSP0 & 0xFFF) != SDCARD_IF_COND_ARGUMENT) {
            return usdhcRecordError(SDCARD_COMMAND_SEND_IF_COND, SD_INVALID_VOLTRANGE, U->CMD_RSP0);
        }
        version2 = true;
    } else if (error != SD_CMD_RSP_TIMEOUT) {
        return error;
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

    if (usdhc.fourBitBus) {
        error = usdhcSendAppCommand(SDCARD_ACOMMAND_SET_BUS_WIDTH, SDCARD_ACMD6_BUS_WIDTH_4BIT, USDHC_RESPONSE_R1);
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

/* ════════════════════════════════════════════════════════════════════════════
 * Betaflight SD_* HAL (drivers/sdmmc_sdio.h), called from sdcard_sdio_baremetal.c
 * ════════════════════════════════════════════════════════════════════════════ */

bool SD_InitialiseHardware(dmaResource_t *dma)
{
    UNUSED(dma);

    if (SDIO_CFG_TO_DEV(sdioConfig()->device) != SDIODEV_1) {
        return false;
    }

    memset(&usdhc, 0, sizeof(usdhc));
    usdhc.fourBitBus = sdioConfig()->use4BitWidth;

    /* Ungate the uSDHC clock root + mux its pads via the SM (SCMI). */
    navq95_scmi_clock_enable(NAVQ95_USDHC_CLK_ROOT);
    if (usdhcConfigPads(usdhc.fourBitBus) != 0) {
        return false;
    }

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

/* No card-detect pin wired yet: report present if identification found one. */
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

/* ════════════════════════════════════════════════════════════════════════════
 * Optional boot self-test (NAVQ95_SD_SELFTEST). Prints the uSDHC controller
 * state + the card-identify result to the console, bypassing the Betaflight
 * sdcard/afatfs stack, so the pads/instance/clock best guess can be validated
 * on hardware without CONFIG_IN_SDCARD (which is boot-fatal on failure).
 * Call navq95_sd_selftest() from systemInit() after the console is up.
 * ════════════════════════════════════════════════════════════════════════════ */
#ifdef NAVQ95_SD_SELFTEST

extern void navq95_console_puts(const char *s);   /* platform_stubs.c */
extern void navq95_console_puthex(uint32_t v);    /* platform_stubs.c */
extern void navq95_console_drain(void);           /* platform_stubs.c */

void navq95_sd_selftest(void)
{
    navq95_console_puts("\n[SD selftest]\n");

    /* Enable the uSDHC clock root over SCMI and CAPTURE the status. A negative
     * value is an SM error (e.g. 0xfffffffd = -3 = SM_ERR_DENIED), which means
     * the System Manager refuses this resource to the M7 agent. CRITICAL: if the
     * clock is NOT enabled, touching the uSDHC register block (0x4285xxxx) can
     * HANG the AHB bus (an unclocked peripheral does not ACK), so we must NOT
     * proceed to usdhcInitController() in that case. */
    int clkStatus = navq95_scmi_clock_enable(NAVQ95_USDHC_CLK_ROOT);
    navq95_console_puts("  clk=");
    navq95_console_puthex((uint32_t)clkStatus);

    int padStatus = usdhcConfigPads(false);
    navq95_console_puts(" pinmux=");
    navq95_console_puthex((uint32_t)padStatus);

    /* GATE the register access. Only poke the uSDHC block if the SM granted BOTH
     * the clock and the pads. Otherwise print the verdict and bail safely — the
     * register access would otherwise brick the board (bus stall on an
     * unclocked / unowned peripheral). */
    if (clkStatus != 0 || padStatus != 0) {
        navq95_console_puts("\n  DENIED by SM (clk/pinmux not granted) -> skipping"
                            " uSDHC register access to avoid a bus hang.\n");
        navq95_console_puts("  => wrong instance/root/pad identifiers, or the"
                            " uSDHC is owned by the SM/A55 (not the M7 agent).\n");
        navq95_console_puts("[/SD selftest]\n");
        navq95_console_drain();
        return;
    }

    bool ctrlOk = usdhcInitController();
    navq95_console_puts(" PRES_STATE=");
    navq95_console_puthex(U->PRES_STATE);
    navq95_console_puts(" HOST_CAP=");
    navq95_console_puthex(U->HOST_CTRL_CAP);
    navq95_console_puts(" cardInserted=");
    navq95_console_puthex((U->PRES_STATE & USDHC_PRES_STATE_CINS_MASK) ? 1u : 0u);
    navq95_console_puts(ctrlOk ? " ctrlInit=OK\n" : " ctrlInit=FAIL\n");

    usdhc.fourBitBus = false;
    SD_Error_t e = usdhcIdentifyCard();
    navq95_console_puts("  identify=");
    navq95_console_puthex((uint32_t)e);
    if (e == SD_OK) {
        usdhc.initialised = true;
        SD_GetCardInfo();
        navq95_console_puts(" rca=");
        navq95_console_puthex(usdhc.rca);
        navq95_console_puts(" hc=");
        navq95_console_puthex(usdhc.highCapacity);
        navq95_console_puts(" blocks=");
        navq95_console_puthex((uint32_t)SD_CardInfo.CardCapacity);
    } else {
        navq95_console_puts(" lastErr=");
        navq95_console_puthex((uint32_t)usdhcLastError.error);
        navq95_console_puts(" cmd=");
        navq95_console_puthex(usdhcLastError.command);
        navq95_console_puts(" detail=");
        navq95_console_puthex(usdhcLastError.detail);
    }
    navq95_console_puts("\n[/SD selftest]\n");
    navq95_console_drain();
}
#endif /* NAVQ95_SD_SELFTEST */

#endif /* USE_SDCARD_SDIO */
