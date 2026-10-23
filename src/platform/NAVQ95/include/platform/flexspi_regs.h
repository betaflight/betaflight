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
 * NavQ95 (i.MX 95 M7) FlexSPI1 controller registers, hand-written for the
 * no-SDK NAVQ95 port (mirrors the SDK FLEXSPI_Type / masks that
 * NAVQ95/flexspi_nor_imx95.c uses, ported from Betaflight PR #15853).
 *
 *   FlexSPI1 control registers : 0x425E0000 (IMX9_FLEXSPI_BASE)
 *   FlexSPI1 AMBA (NOR window)  : 0x28000000 (IMX9_FLEXSPI1_BASE), alias 0x02000000
 *
 * We XIP from the 0x02020000 low alias, but the FlexSPI IP command offset
 * (IPCR0 = address - FlexSPI_AMBA_BASE) is a NOR byte offset, so it uses the
 * PRIMARY window base 0x28000000. The config partition the writer touches is
 * therefore addressed in the 0x28xxxxxx space (see navq95_flexspi.ld
 * __config_start / __config_end).
 *
 * Only the fields/masks the NOR driver references are defined. Register offsets
 * follow the standard i.MX FlexSPI block layout (same as i.MXRT, which uses the
 * identical IP).
 */

#pragma once

#include <stdint.h>

/* ── FlexSPI1 bases ───────────────────────────────────────────────────────── */
#define FlexSPI_AMBA_BASE           0x28000000u     /* NOR memory-mapped window  */
#define NAVQ95_FLEXSPI_BASE         0x425E0000u     /* controller registers      */

/* ── FlexSPI register block (offsets per the i.MX / i.MXRT FlexSPI IP, matching
 * the MCUXpresso FLEXSPI_Type) ──────────────────────────────────────────────
 * Only members used by the NOR writer are named; reserved gaps use uint8_t[]
 * so every named register lands at its documented offset. */
typedef struct {
    volatile uint32_t MCR0;            /* 0x000 Module Control 0            */
    volatile uint32_t MCR1;            /* 0x004                              */
    volatile uint32_t MCR2;            /* 0x008                              */
    volatile uint32_t AHBCR;           /* 0x00C                              */
    volatile uint32_t INTEN;           /* 0x010                              */
    volatile uint32_t INTR;            /* 0x014 Interrupt (W1C)             */
    volatile uint32_t LUTKEY;          /* 0x018 LUT key                     */
    volatile uint32_t LUTCR;           /* 0x01C LUT control                 */
    volatile uint32_t AHBRXBUFCR0[8];  /* 0x020..0x03C                       */
    uint8_t           RESERVED_0[32];  /* 0x040..0x05F                       */
    volatile uint32_t FLSHCR0[4];      /* 0x060..0x06C flash A1/A2/B1/B2     */
    volatile uint32_t FLSHCR1[4];      /* 0x070..0x07C                       */
    volatile uint32_t FLSHCR2[4];      /* 0x080..0x08C                       */
    uint8_t           RESERVED_1[4];   /* 0x090                              */
    volatile uint32_t FLSHCR4;         /* 0x094                              */
    uint8_t           RESERVED_2[8];   /* 0x098..0x09C                       */
    volatile uint32_t IPCR0;           /* 0x0A0 IP Control 0 (SFAR offset)  */
    volatile uint32_t IPCR1;           /* 0x0A4 IP Control 1 (seq/size)     */
    uint8_t           RESERVED_3[8];   /* 0x0A8..0x0AC                       */
    volatile uint32_t IPCMD;           /* 0x0B0 IP Command trigger          */
    uint8_t           RESERVED_4[4];   /* 0x0B4                              */
    volatile uint32_t IPRXFCR;         /* 0x0B8 IP RX FIFO control          */
    volatile uint32_t IPTXFCR;         /* 0x0BC IP TX FIFO control          */
    volatile uint32_t DLLCR[2];        /* 0x0C0..0x0C4 DLL control A/B      */
    uint8_t           RESERVED_5[8];   /* 0x0C8..0x0CC                       */
    volatile uint32_t MISCCR4;         /* 0x0D0                              */
    volatile uint32_t MISCCR5;         /* 0x0D4                              */
    volatile uint32_t MISCCR6;         /* 0x0D8                              */
    volatile uint32_t MISCCR7;         /* 0x0DC                              */
    volatile uint32_t STS0;            /* 0x0E0 Status 0 (arb/seq idle)     */
    volatile uint32_t STS1;            /* 0x0E4                              */
    volatile uint32_t STS2;            /* 0x0E8 Status 2 (DLL lock)         */
    volatile uint32_t AHBSPNDSTS;      /* 0x0EC                              */
    volatile uint32_t IPRXFSTS;        /* 0x0F0 IP RX FIFO status           */
    volatile uint32_t IPTXFSTS;        /* 0x0F4 IP TX FIFO status           */
    uint8_t           RESERVED_6[8];   /* 0x0F8..0x0FC                       */
    volatile uint32_t RFDR[32];        /* 0x100..0x17C RX FIFO data         */
    volatile uint32_t TFDR[32];        /* 0x180..0x1FC TX FIFO data         */
    volatile uint32_t LUT[64];         /* 0x200..0x2FC Look-Up Table        */
} FLEXSPI_Type;

#define FLEXSPI                     ((FLEXSPI_Type *)NAVQ95_FLEXSPI_BASE)

/* ── MCR0 ─────────────────────────────────────────────────────────────────── */
#define FLEXSPI_MCR0_SWRESET_MASK        (1u << 0)

/* ── INTR (write-1-clear) ─────────────────────────────────────────────────── */
#define FLEXSPI_INTR_IPCMDDONE_MASK      (1u << 0)
#define FLEXSPI_INTR_IPCMDGE_MASK        (1u << 1)
#define FLEXSPI_INTR_IPCMDERR_MASK       (1u << 3)
#define FLEXSPI_INTR_IPRXWA_MASK         (1u << 5)   /* RX FIFO watermark avail */
#define FLEXSPI_INTR_IPTXWE_MASK         (1u << 6)   /* TX FIFO watermark empty */

/* ── LUTCR ────────────────────────────────────────────────────────────────── */
#define FLEXSPI_LUTCR_LOCK_MASK          (1u << 0)
#define FLEXSPI_LUTCR_UNLOCK_MASK        (1u << 1)

/* ── FLSHCR2 (per-port; [0] = A1) ─────────────────────────────────────────── */
#define FLEXSPI_FLSHCR2_ARDSEQID_SHIFT   16
#define FLEXSPI_FLSHCR2_ARDSEQID_MASK    (0x1Fu << FLEXSPI_FLSHCR2_ARDSEQID_SHIFT)
#define FLEXSPI_FLSHCR2_ARDSEQNUM_SHIFT  21
#define FLEXSPI_FLSHCR2_ARDSEQNUM_MASK   (0x7u << FLEXSPI_FLSHCR2_ARDSEQNUM_SHIFT)
#define FLEXSPI_FLSHCR2_CLRINSTRPTR_MASK (1u << 31)

/* ── IPCR1 field builders (ISEQID[31:16], ISEQNUM[26:24], IDATSZ[15:0]) ────── */
#define FLEXSPI_IPCR1_IDATSZ(x)          ((uint32_t)(x) & 0xFFFFu)
#define FLEXSPI_IPCR1_ISEQID(x)          (((uint32_t)(x) & 0x1Fu) << 16)
#define FLEXSPI_IPCR1_ISEQNUM(x)         (((uint32_t)(x) & 0x7u) << 24)

/* ── IPCMD ────────────────────────────────────────────────────────────────── */
#define FLEXSPI_IPCMD_TRG_MASK           (1u << 0)

/* ── IP FIFO control ──────────────────────────────────────────────────────── */
#define FLEXSPI_IPTXFCR_CLRIPTXF_MASK    (1u << 0)
#define FLEXSPI_IPRXFCR_CLRIPRXF_MASK    (1u << 0)

/* ── IPRXFSTS (FILL = 64-bit words available, [7:0]) ──────────────────────── */
#define FLEXSPI_IPRXFSTS_FILL_MASK       (0xFFu << 0)

/* ── STS0 (arbiter / sequence idle) ───────────────────────────────────────── */
#define FLEXSPI_STS0_SEQIDLE_MASK        (1u << 0)
#define FLEXSPI_STS0_ARBIDLE_MASK        (1u << 1)

/* ── STS2 (A/B sample + reference DLL lock) ───────────────────────────────── */
#define FLEXSPI_STS2_AREFLOCK_MASK       (1u << 1)
#define FLEXSPI_STS2_ASLVLOCK_MASK       (1u << 0)

/* ── DLLCR[0] ─────────────────────────────────────────────────────────────── */
#define FLEXSPI_DLLCR_DLLEN_MASK         (1u << 0)

/* ── Code that erases/programs the NOR must NOT execute from it: place such
 *    functions in the OCRAM section the linker copies out of flash at reset.
 *    (See navq95_flexspi.ld .ocram_text and the startup copy loop.) */
#define OCRAM_CODE                       __attribute__((section(".ocram_text"), noinline))
