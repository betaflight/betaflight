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
 * usdhc_regs.h — minimal i.MX 95 uSDHC register block + bitfields for the
 * bare-metal SD-card driver (sdio_usdhc_navq95.c). We do not use the MCUXpresso
 * SDK, so this provides the USDHC_Type struct + USDHC_* masks the driver needs.
 * Offsets/bitfields are from the NuttX i.MX9 header
 * (robotics-nuttx arch/arm64/src/imx9/hardware/imx9_usdhc.h), which is the
 * authoritative description of the uSDHC IP on this SoC.
 *
 * Base addresses (i.MX95): uSDHC1 0x42850000, uSDHC2 0x42860000, uSDHC3 0x428B0000.
 */

#pragma once

#include <stdint.h>

/* Register block. Only the registers the driver touches are named; the rest are
 * reserved padding so the named ones land at the correct offsets. */
typedef struct {
    volatile uint32_t DS_ADDR;        /* 0x00 SDMA system address             */
    volatile uint32_t BLK_ATT;        /* 0x04 block attributes (size + count) */
    volatile uint32_t CMD_ARG;        /* 0x08 command argument                */
    volatile uint32_t CMD_XFR_TYP;    /* 0x0C command transfer type           */
    volatile uint32_t CMD_RSP0;       /* 0x10 response 0                      */
    volatile uint32_t CMD_RSP1;       /* 0x14 response 1                      */
    volatile uint32_t CMD_RSP2;       /* 0x18 response 2                      */
    volatile uint32_t CMD_RSP3;       /* 0x1C response 3                      */
    volatile uint32_t DATA_BUFF;      /* 0x20 data buffer access port         */
    volatile uint32_t PRES_STATE;     /* 0x24 present state                   */
    volatile uint32_t PROT_CTRL;      /* 0x28 protocol control                */
    volatile uint32_t SYS_CTRL;       /* 0x2C system control (clock/reset)    */
    volatile uint32_t INT_STATUS;     /* 0x30 interrupt status                */
    volatile uint32_t INT_STATUS_EN;  /* 0x34 interrupt status enable         */
    volatile uint32_t INT_SIGNAL_EN;  /* 0x38 interrupt signal enable         */
    volatile uint32_t AC12_ERR;       /* 0x3C auto CMD12 error status         */
    volatile uint32_t HOST_CTRL_CAP;  /* 0x40 host controller capabilities    */
    volatile uint32_t WTMK_LVL;       /* 0x44 watermark level                 */
    volatile uint32_t MIX_CTRL;       /* 0x48 mixer control (transfer mode)   */
    volatile uint32_t RESERVED_4C;    /* 0x4C                                 */
    volatile uint32_t FORCE_EVENT;    /* 0x50 force event                     */
    volatile uint32_t ADMA_ERR_STATUS;/* 0x54 ADMA error status               */
    volatile uint32_t ADMA_SYS_ADDR;  /* 0x58 ADMA system address             */
    volatile uint32_t RESERVED_5C;    /* 0x5C                                 */
    volatile uint32_t DLL_CTRL;       /* 0x60 DLL control                     */
    volatile uint32_t DLL_STATUS;     /* 0x64 DLL status                      */
    volatile uint32_t CLK_TUNE_CTRL;  /* 0x68 CLK tune control + status       */
    volatile uint32_t RESERVED_6C[21];/* 0x6C..0xBC                           */
    volatile uint32_t VEND_SPEC;      /* 0xC0 vendor specific                 */
    volatile uint32_t MMC_BOOT;       /* 0xC4 MMC boot                        */
    volatile uint32_t VEND_SPEC2;     /* 0xC8 vendor specific 2               */
    volatile uint32_t TUNING_CTRL;    /* 0xCC tuning control                  */
} USDHC_Type;

#define USDHC1_BASE   0x42850000u
#define USDHC2_BASE   0x42860000u
#define USDHC3_BASE   0x428B0000u

/* ── BLK_ATT ───────────────────────────────────────────────────────────────*/
#define USDHC_BLK_ATT_BLKSIZE(n)   ((uint32_t)(n) & 0x1FFFu)
#define USDHC_BLK_ATT_BLKCNT(n)    (((uint32_t)(n) & 0xFFFFu) << 16)

/* ── CMD_XFR_TYP ───────────────────────────────────────────────────────────*/
#define USDHC_CMD_XFR_TYP_DMAEN_MASK   (1u << 0)
#define USDHC_CMD_XFR_TYP_BCEN_MASK    (1u << 1)
#define USDHC_CMD_XFR_TYP_AC12EN_MASK  (1u << 2)
#define USDHC_CMD_XFR_TYP_DTDSEL_MASK  (1u << 4)
#define USDHC_CMD_XFR_TYP_MSBSEL_MASK  (1u << 5)
#define USDHC_CMD_XFR_TYP_RSPTYP(n)    (((uint32_t)(n) & 3u) << 16)
#define USDHC_CMD_XFR_TYP_CCCEN_MASK   (1u << 19)
#define USDHC_CMD_XFR_TYP_CICEN_MASK   (1u << 20)
#define USDHC_CMD_XFR_TYP_DPSEL_MASK   (1u << 21)
#define USDHC_CMD_XFR_TYP_CMDINX(n)    (((uint32_t)(n) & 0x3Fu) << 24)

/* ── MIX_CTRL (same bit layout as the XFR_TYP transfer-mode half) ──────────*/
#define USDHC_MIX_CTRL_DMAEN_MASK      (1u << 0)
#define USDHC_MIX_CTRL_BCEN_MASK       (1u << 1)
#define USDHC_MIX_CTRL_AC12EN_MASK     (1u << 2)
#define USDHC_MIX_CTRL_DTDSEL_MASK     (1u << 4)
#define USDHC_MIX_CTRL_MSBSEL_MASK     (1u << 5)

/* ── PRES_STATE ────────────────────────────────────────────────────────────*/
#define USDHC_PRES_STATE_CIHB_MASK     (1u << 0)   /* command inhibit (CMD)   */
#define USDHC_PRES_STATE_CDIHB_MASK    (1u << 1)   /* command inhibit (DAT)   */
#define USDHC_PRES_STATE_SDSTB_MASK    (1u << 3)   /* SD clock stable         */
#define USDHC_PRES_STATE_CINS_MASK     (1u << 16)  /* card inserted           */

/* ── PROT_CTRL ─────────────────────────────────────────────────────────────*/
#define USDHC_PROT_CTRL_DTW_MASK       (3u << 1)
#define USDHC_PROT_CTRL_DTW(n)         (((uint32_t)(n) & 3u) << 1)   /* 0=1bit 1=4bit 2=8bit */
#define USDHC_PROT_CTRL_EMODE_MASK     (3u << 4)
#define USDHC_PROT_CTRL_EMODE(n)       (((uint32_t)(n) & 3u) << 4)   /* 2 = little endian */
#define USDHC_PROT_CTRL_DMASEL_MASK    (3u << 8)
#define USDHC_PROT_CTRL_DMASEL(n)      (((uint32_t)(n) & 3u) << 8)   /* 2 = ADMA2 */

/* ── SYS_CTRL ──────────────────────────────────────────────────────────────*/
#define USDHC_SYS_CTRL_DVS(n)          (((uint32_t)(n) & 0xFu) << 4)
#define USDHC_SYS_CTRL_DVS_MASK        (0xFu << 4)
#define USDHC_SYS_CTRL_SDCLKFS(n)      (((uint32_t)(n) & 0xFFu) << 8)
#define USDHC_SYS_CTRL_SDCLKFS_MASK    (0xFFu << 8)
#define USDHC_SYS_CTRL_DTOCV(n)        (((uint32_t)(n) & 0xFu) << 16)
#define USDHC_SYS_CTRL_DTOCV_MASK      (0xFu << 16)
#define USDHC_SYS_CTRL_RSTA_MASK       (1u << 24)
#define USDHC_SYS_CTRL_RSTC_MASK       (1u << 25)
#define USDHC_SYS_CTRL_RSTD_MASK       (1u << 26)
#define USDHC_SYS_CTRL_INITA_MASK      (1u << 27)

/* ── INT_STATUS / INT_STATUS_EN / INT_SIGNAL_EN (shared layout) ────────────*/
#define USDHC_INT_STATUS_CC_MASK       (1u << 0)   /* command complete        */
#define USDHC_INT_STATUS_TC_MASK       (1u << 1)   /* transfer complete       */
#define USDHC_INT_STATUS_BGE_MASK      (1u << 2)   /* block gap event         */
#define USDHC_INT_STATUS_DINT_MASK     (1u << 3)   /* DMA interrupt           */
#define USDHC_INT_STATUS_BWR_MASK      (1u << 4)   /* buffer write ready      */
#define USDHC_INT_STATUS_BRR_MASK      (1u << 5)   /* buffer read ready       */
#define USDHC_INT_STATUS_CINS_MASK     (1u << 6)   /* card insertion          */
#define USDHC_INT_STATUS_CRM_MASK      (1u << 7)   /* card removal            */
#define USDHC_INT_STATUS_CINT_MASK     (1u << 8)   /* card interrupt          */
#define USDHC_INT_STATUS_RTE_MASK      (1u << 12)  /* re-tuning event         */
#define USDHC_INT_STATUS_TP_MASK       (1u << 13)  /* tuning pass             */
#define USDHC_INT_STATUS_CTOE_MASK     (1u << 16)  /* command timeout error   */
#define USDHC_INT_STATUS_CCE_MASK      (1u << 17)  /* command CRC error       */
#define USDHC_INT_STATUS_CEBE_MASK     (1u << 18)  /* command end-bit error   */
#define USDHC_INT_STATUS_CIE_MASK      (1u << 19)  /* command index error     */
#define USDHC_INT_STATUS_DTOE_MASK     (1u << 20)  /* data timeout error      */
#define USDHC_INT_STATUS_DCE_MASK      (1u << 21)  /* data CRC error          */
#define USDHC_INT_STATUS_DEBE_MASK     (1u << 22)  /* data end-bit error      */
#define USDHC_INT_STATUS_AC12E_MASK    (1u << 24)  /* auto CMD12 error        */
#define USDHC_INT_STATUS_TNE_MASK      (1u << 26)  /* tuning error            */
#define USDHC_INT_STATUS_DMAE_MASK     (1u << 28)  /* DMA error               */
