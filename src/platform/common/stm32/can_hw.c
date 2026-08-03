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

#if ENABLE_CAN

// Driver supports the Bosch M_CAN IP as implemented on G4 / H7 / C5.
// Other families either lack FDCAN (e.g. F4 / F7) or use a different IP
// revision that this driver has not been ported to yet. Fail loud at
// compile time if ENABLE_CAN is forced on elsewhere, rather than silently
// producing a non-functional build where TX would accept frames but do
// nothing.
#if !defined(STM32G4) && !defined(STM32H7) && !defined(STM32C5)
#error "ENABLE_CAN is set but the target MCU family has no FDCAN support in this driver"
#endif

#include "common/utils.h"

#include "drivers/can/can.h"
#include "drivers/can/can_impl.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/resource.h"
#include "platform/rcc.h"

//-----------------------------------------------------------------------------
// Message RAM layout
//
// All currently supported families (G4, H7, C5) expose the same Bosch M_CAN
// IP and share a dedicated Message RAM (CAN SRAM) at SRAMCAN_BASE. The
// significant difference is *who* picks the layout:
//
//   - G4 and C5: layout is fixed by silicon. Each FDCAN instance owns a
//     region of SRAMCAN_SIZE bytes at SRAMCAN_BASE + N*SRAMCAN_SIZE; the
//     sub-ranges (filters, FIFOs, Tx buffers) are at hardwired offsets
//     within the region. We only have to match the SDK's assumed element
//     counts and sizes when addressing the RAM.
//
//   - H7: layout is software-defined. We partition the shared RAM and
//     program the start-address / element-count / element-size into each
//     instance's SIDFC/XIDFC/RXF0C/RXF1C/RXBC/TXEFC/TXBC and RXESC/TXESC
//     registers. The driver hands out a fixed-size slice to each instance
//     using a per-instance word offset; within each slice we only use
//     Rx FIFO 0 and the Tx FIFO, and pick the smallest element size
//     (8-byte data) since this driver currently targets classic CAN.
//
// To keep the Tx / Rx element access paths identical across families, both
// configurations use a 4-word slot layout:
//
//   word 0: identifier + XTD flag
//   word 1: DLC (+ classic CAN flags, zero for now)
//   word 2: data[0..3]
//   word 3: data[4..7]
//
// On G4/C5 the hardware reserves 18 words per element (enough for FD
// 64-byte payloads); we just leave words 4..17 unused. On H7 we select
// element size code 0b000 = 8-byte data which gives a natural 4-word
// element.
//-----------------------------------------------------------------------------

#if defined(STM32G4) || defined(STM32C5)

// Element counts mirror ST's HAL private constants so the fixed offsets we
// compute below match the silicon layout.
#define CAN_SRAM_FLS_NBR    (28U)
#define CAN_SRAM_FLE_NBR    (8U)
#define CAN_SRAM_RF0_NBR    (3U)
#define CAN_SRAM_RF1_NBR    (3U)
#define CAN_SRAM_TEF_NBR    (3U)
#define CAN_SRAM_TFQ_NBR    (3U)

#define CAN_SRAM_FLS_SIZE   (1U * 4U)           // standard filter element
#define CAN_SRAM_FLE_SIZE   (2U * 4U)           // extended filter element
#define CAN_SRAM_RF_SIZE    (18U * 4U)          // Rx FIFO element (header + 64B)
#define CAN_SRAM_TEF_SIZE   (2U * 4U)           // Tx event FIFO element
#define CAN_SRAM_TFQ_SIZE   (18U * 4U)          // Tx FIFO element

#define CAN_SRAM_FLSSA      (0U)
#define CAN_SRAM_FLESA      (CAN_SRAM_FLSSA + CAN_SRAM_FLS_NBR * CAN_SRAM_FLS_SIZE)
#define CAN_SRAM_RF0SA      (CAN_SRAM_FLESA + CAN_SRAM_FLE_NBR * CAN_SRAM_FLE_SIZE)
#define CAN_SRAM_RF1SA      (CAN_SRAM_RF0SA + CAN_SRAM_RF0_NBR * CAN_SRAM_RF_SIZE)
#define CAN_SRAM_TEFSA      (CAN_SRAM_RF1SA + CAN_SRAM_RF1_NBR * CAN_SRAM_RF_SIZE)
#define CAN_SRAM_TFQSA      (CAN_SRAM_TEFSA + CAN_SRAM_TEF_NBR * CAN_SRAM_TEF_SIZE)
#define CAN_SRAM_PER_INSTANCE \
                            (CAN_SRAM_TFQSA + CAN_SRAM_TFQ_NBR * CAN_SRAM_TFQ_SIZE)

#define CAN_RX_ELEMENT_BYTES    CAN_SRAM_RF_SIZE
#define CAN_TX_ELEMENT_BYTES    CAN_SRAM_TFQ_SIZE

#elif defined(STM32H7)

// H7 layout (software-defined). Values are in *elements* or *bytes* as noted.
// Per-instance RAM reservation in WORDS (1 word = 4 bytes):
//  - 3 Rx FIFO 0 slots × 4 words = 12 words
//  - 3 Tx FIFO slots  × 4 words = 12 words
//  Round to a fixed 64-word (256-byte) slice per instance to leave head-room
//  for future additions (extra FIFOs, filters). Total usage with 3 instances
//  is 192 words / 768 bytes out of the 10 KiB of shared Message RAM.
#define CAN_H7_RF0_NBR              (3U)
#define CAN_H7_TFQ_NBR              (3U)
#define CAN_H7_ELEMENT_WORDS        (4U)        // 2 hdr + 8-byte data
#define CAN_H7_PER_INSTANCE_WORDS   (64U)
#define CAN_H7_RF0_WORD_OFFSET      (0U)
#define CAN_H7_TFQ_WORD_OFFSET      (CAN_H7_RF0_WORD_OFFSET + \
                                     CAN_H7_RF0_NBR * CAN_H7_ELEMENT_WORDS)

// Element-size code 0b000 = 8-byte data field (classic CAN).
#define CAN_H7_ESC_CODE_8B          (0U)

#define CAN_RX_ELEMENT_BYTES        (CAN_H7_ELEMENT_WORDS * 4U)
#define CAN_TX_ELEMENT_BYTES        (CAN_H7_ELEMENT_WORDS * 4U)

#endif

// Per-instance base address of Message RAM. On all supported families the
// regions are laid out contiguously from SRAMCAN_BASE in device-index order.
static uint32_t canMessageRamBase(canDevice_e device)
{
#if defined(STM32G4) || defined(STM32C5)
    return SRAMCAN_BASE + (uint32_t)device * CAN_SRAM_PER_INSTANCE;
#elif defined(STM32H7)
    return SRAMCAN_BASE + (uint32_t)device * CAN_H7_PER_INSTANCE_WORDS * 4U;
#else
    UNUSED(device);
    return 0;
#endif
}

static uint32_t canRxElementAddress(canDevice_e device, uint32_t index)
{
#if defined(STM32G4) || defined(STM32C5)
    return canMessageRamBase(device) + CAN_SRAM_RF0SA + index * CAN_SRAM_RF_SIZE;
#elif defined(STM32H7)
    return canMessageRamBase(device)
         + CAN_H7_RF0_WORD_OFFSET * 4U
         + index * CAN_RX_ELEMENT_BYTES;
#else
    UNUSED(device); UNUSED(index);
    return 0;
#endif
}

static uint32_t canTxElementAddress(canDevice_e device, uint32_t index)
{
#if defined(STM32G4) || defined(STM32C5)
    return canMessageRamBase(device) + CAN_SRAM_TFQSA + index * CAN_SRAM_TFQ_SIZE;
#elif defined(STM32H7)
    return canMessageRamBase(device)
         + CAN_H7_TFQ_WORD_OFFSET * 4U
         + index * CAN_TX_ELEMENT_BYTES;
#else
    UNUSED(device); UNUSED(index);
    return 0;
#endif
}

//-----------------------------------------------------------------------------
// Bit timing for 1 Mbit/s classic CAN
//
// The FDCAN kernel clock tree differs per family and per board. On G4 the
// system clock config explicitly selects RCC_FDCANCLKSOURCE_PCLK1 (see
// system_stm32g4xx.c) which on overclocked FCs runs at 150-170 MHz. On H7
// the selector in RCC->D2CCIP1R.FDCANSEL defaults to HSE but may be tuned
// per board. Hard-coding a prescaler therefore gives the wrong line rate on
// any board whose FDCAN kernel clock does not match the assumption.
//
// canNominalBitTiming() discovers the actual FDCAN kernel clock at init
// time and searches for a (prescaler, tseg1, tseg2) triple that lands
// exactly on the requested bit rate with a sample point near 75 %. On
// clocks that are integer multiples of the target rate (common FC
// configurations) the search always succeeds; if no valid setting is
// found we return false so the caller can leave the device uninitialised
// rather than silently running at a different bit rate than configured.
//-----------------------------------------------------------------------------

// NBTP fields: NTSEG1=bits 8:15, NTSEG2=bits 0:6, NBRP=16:24, NSJW=25:31
#define CAN_NBTP_NSJW_POS       (25U)
#define CAN_NBTP_NBRP_POS       (16U)
#define CAN_NBTP_NTSEG1_POS     (8U)
#define CAN_NBTP_NTSEG2_POS     (0U)

#define CAN_NBRP_MAX            (512U)
#define CAN_NTSEG1_MAX          (256U)
#define CAN_NTSEG2_MAX          (128U)
#define CAN_TOTAL_TQ_MIN        (8U)
#define CAN_TOTAL_TQ_MAX        (25U)

// Sample-point target in tenths of a per-mille (i.e. 750 = 75.0%).
#define CAN_SAMPLE_POINT_PERMILLE (750U)

static uint32_t canPackNominalBitTiming(uint32_t nbrp, uint32_t ntseg1,
                                        uint32_t ntseg2, uint32_t nsjw)
{
    // Register fields store (value - 1); caller passes the real tq counts.
    return ((nsjw - 1U)   << CAN_NBTP_NSJW_POS)
         | ((nbrp - 1U)   << CAN_NBTP_NBRP_POS)
         | ((ntseg1 - 1U) << CAN_NBTP_NTSEG1_POS)
         | ((ntseg2 - 1U) << CAN_NBTP_NTSEG2_POS);
}

// Query the FDCAN kernel clock. HAL_RCCEx_GetPeriphCLKFreq tracks the
// family's actual clock-source selection where available; C5's HAL2 has
// no equivalent that works without a handle, so fall back to PCLK1 which
// matches the HAL2 reset default for FDCAN.
static uint32_t canGetKernelClockHz(void)
{
#if defined(STM32G4) || defined(STM32H7)
    return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
#else
    return HAL_RCC_GetPCLK1Freq();
#endif
}

static bool canNominalBitTiming(uint32_t bitrate, uint32_t *nbtp)
{
    if (bitrate == 0U || nbtp == NULL) {
        return false;
    }

    const uint32_t kernelHz = canGetKernelClockHz();

    // Search for a prescaler that divides the kernel clock down to an
    // integer tq count in the valid range [8..25]. Pick the first match,
    // which is the smallest prescaler (gives finest resolution for SJW).
    for (uint32_t nbrp = 1; nbrp <= CAN_NBRP_MAX; nbrp++) {
        if ((kernelHz % nbrp) != 0U) {
            continue;
        }
        const uint32_t tqClock = kernelHz / nbrp;
        if ((tqClock % bitrate) != 0U) {
            continue;
        }
        const uint32_t totalTq = tqClock / bitrate;
        if (totalTq < CAN_TOTAL_TQ_MIN || totalTq > CAN_TOTAL_TQ_MAX) {
            continue;
        }

        // Sample point is (SYNC_SEG + TSEG1) / totalTq. Round the target
        // position to the nearest tq, then clamp so TSEG1 >= 1 and
        // TSEG2 >= 1.
        uint32_t samplePointTq = (totalTq * CAN_SAMPLE_POINT_PERMILLE + 500U) / 1000U;
        if (samplePointTq < 2U) {
            samplePointTq = 2U;
        }
        if (samplePointTq >= totalTq) {
            samplePointTq = totalTq - 1U;
        }
        const uint32_t ntseg1 = samplePointTq - 1U;
        const uint32_t ntseg2 = totalTq - samplePointTq;
        if (ntseg1 > CAN_NTSEG1_MAX || ntseg2 > CAN_NTSEG2_MAX) {
            continue;
        }

        // SJW = min(4, tseg2) keeps resync margin within spec.
        const uint32_t nsjw = (ntseg2 < 4U) ? ntseg2 : 4U;
        *nbtp = canPackNominalBitTiming(nbrp, ntseg1, ntseg2, nsjw);
        return true;
    }

    return false;
}

//-----------------------------------------------------------------------------
// Tx/Rx element header encoding (identical across G4 and H7 M_CAN IP)
//
// Word 0: ID field.
//  - Standard ID occupies bits 18..28 (11-bit ID shifted left by 18).
//  - Extended ID occupies bits 0..28, with XTD (bit 30) set.
// Word 1: DLC + flags (bits 16..19 = DLC, FDF = bit 21, BRS = bit 20).
// Words 2..: payload, packed little-endian 4 bytes per word.
//-----------------------------------------------------------------------------

#define CAN_TX_WORD0_XTD        (1UL << 30)
#define CAN_TX_WORD1_DLC_POS    (16U)

#define CAN_RX_WORD0_XTD        (1UL << 30)
#define CAN_RX_WORD0_EXTID_MASK (0x1FFFFFFFUL)
#define CAN_RX_WORD0_STDID_POS  (18U)
#define CAN_RX_WORD0_STDID_MASK (0x7FFUL << CAN_RX_WORD0_STDID_POS)
#define CAN_RX_WORD1_DLC_POS    (16U)
#define CAN_RX_WORD1_DLC_MASK   (0xFUL << CAN_RX_WORD1_DLC_POS)

//-----------------------------------------------------------------------------
// Low-level helpers (register level)
//-----------------------------------------------------------------------------

static FDCAN_GlobalTypeDef *canRegs(canDevice_e device)
{
    return (FDCAN_GlobalTypeDef *)canDevice[device].reg;
}

// Bounded spin count for CCCR.INIT transitions. At 170 MHz SYSCLK this
// translates to ~600 us of worst-case polling, which is more than enough
// for the peripheral to synchronise (the datasheet spec is a few bit times)
// but short enough that a stuck peripheral does not wedge boot.
#define CAN_INIT_SPIN_MAX       (100000U)

// Enter INIT + CCE (configuration change enable). Required for any change to
// bit timing or message RAM layout. Busy-loops until the peripheral reports
// that INIT has taken effect.
static bool canEnterConfigMode(FDCAN_GlobalTypeDef *regs)
{
    regs->CCCR |= FDCAN_CCCR_INIT;

    // Peripheral synchronises to the bus before asserting INIT. Give it a
    // bounded number of polls so a disconnected transceiver does not hang us.
    uint32_t timeout = CAN_INIT_SPIN_MAX;
    while (!(regs->CCCR & FDCAN_CCCR_INIT) && timeout--) {
        // spin
    }
    if (!(regs->CCCR & FDCAN_CCCR_INIT)) {
        return false;
    }

    regs->CCCR |= FDCAN_CCCR_CCE;
    return true;
}

// Leaving INIT is also asynchronous: the peripheral waits for 11 consecutive
// recessive bits on the bus before it accepts normal operation. Poll CCCR
// until the bit clears so callers do not attempt TX/RX while the controller
// is still held in init. Returns false on timeout so the caller can decide
// whether to mark the device initialised or bail.
static bool canExitConfigMode(FDCAN_GlobalTypeDef *regs)
{
    regs->CCCR &= ~FDCAN_CCCR_INIT;

    uint32_t timeout = CAN_INIT_SPIN_MAX;
    while ((regs->CCCR & FDCAN_CCCR_INIT) && timeout--) {
        // spin
    }

    return (regs->CCCR & FDCAN_CCCR_INIT) == 0U;
}

// Clear out the per-instance message RAM region. A cold peripheral contains
// indeterminate SRAM contents; sending garbage would be reported as spurious
// messages on start-up.
static void canClearMessageRam(canDevice_e device)
{
    uint32_t base = canMessageRamBase(device);

#if defined(STM32G4) || defined(STM32C5)
    uint32_t bytes = CAN_SRAM_PER_INSTANCE;
#elif defined(STM32H7)
    uint32_t bytes = CAN_H7_PER_INSTANCE_WORDS * 4U;
#else
    uint32_t bytes = 0;
#endif

    for (uint32_t addr = base; addr < base + bytes; addr += 4) {
        *(volatile uint32_t *)addr = 0;
    }
}

static void canEnableInterrupt(canDevice_e device, uint8_t irq)
{
    UNUSED(device);
#if defined(USE_HAL_DRIVER)
    HAL_NVIC_SetPriority(irq, NVIC_PRIORITY_BASE(NVIC_PRIO_CAN),
                         NVIC_PRIORITY_SUB(NVIC_PRIO_CAN));
    HAL_NVIC_EnableIRQ(irq);
#else
    NVIC_SetPriority(irq, NVIC_PRIO_CAN);
    NVIC_EnableIRQ(irq);
#endif
}

#if defined(STM32H7)
// Program the H7 Message RAM layout for this instance. Addresses in the
// SIDFC/XIDFC/RXF0C/TXBC registers are word offsets from SRAMCAN_BASE, not
// absolute byte addresses. We only activate Rx FIFO 0 and the Tx FIFO; the
// other regions (standard/extended filters, Rx FIFO 1, Rx buffers, Tx event
// FIFO) are left zero-sized.
static void canConfigureMessageRamH7(canDevice_e device, FDCAN_GlobalTypeDef *regs)
{
    const uint32_t instanceWords = (uint32_t)device * CAN_H7_PER_INSTANCE_WORDS;

    const uint32_t rf0SA = instanceWords + CAN_H7_RF0_WORD_OFFSET;
    const uint32_t tfqSA = instanceWords + CAN_H7_TFQ_WORD_OFFSET;

    // No standard / extended filter lists installed — point at the region
    // start with size 0 so the peripheral never walks them.
    regs->SIDFC = (instanceWords << FDCAN_SIDFC_FLSSA_Pos) & FDCAN_SIDFC_FLSSA_Msk;
    regs->XIDFC = (instanceWords << FDCAN_XIDFC_FLESA_Pos) & FDCAN_XIDFC_FLESA_Msk;

    // Rx FIFO 0: 3 classic-CAN elements.
    regs->RXF0C = ((rf0SA << FDCAN_RXF0C_F0SA_Pos) & FDCAN_RXF0C_F0SA_Msk)
                | ((CAN_H7_RF0_NBR << FDCAN_RXF0C_F0S_Pos) & FDCAN_RXF0C_F0S_Msk);
    regs->RXF1C = 0;
    regs->RXBC  = 0;

    // All Rx paths use 8-byte data elements (smallest, matches classic CAN).
    regs->RXESC = (CAN_H7_ESC_CODE_8B << FDCAN_RXESC_F0DS_Pos)
                | (CAN_H7_ESC_CODE_8B << FDCAN_RXESC_F1DS_Pos)
                | (CAN_H7_ESC_CODE_8B << FDCAN_RXESC_RBDS_Pos);

    // Tx buffers: 3 FIFO-mode slots, 0 dedicated buffers. TFQM = 0 (FIFO mode).
    regs->TXBC = ((tfqSA << FDCAN_TXBC_TBSA_Pos) & FDCAN_TXBC_TBSA_Msk)
               | ((CAN_H7_TFQ_NBR << FDCAN_TXBC_TFQS_Pos) & FDCAN_TXBC_TFQS_Msk);
    regs->TXEFC = 0;
    regs->TXESC = (CAN_H7_ESC_CODE_8B << FDCAN_TXESC_TBDS_Pos);
}
#endif // STM32H7

//-----------------------------------------------------------------------------
// Init / public API
//-----------------------------------------------------------------------------

void canInitDevice(canDevice_e device, uint32_t bitrate)
{
    if (device < 0 || device >= CANDEV_COUNT) {
        return;
    }

    canDevice_t *pDev = &canDevice[device];

    if (!pDev->reg || !pDev->tx || !pDev->rx) {
        return;
    }

    FDCAN_GlobalTypeDef *regs = (FDCAN_GlobalTypeDef *)pDev->reg;

    // Kernel clock first; register writes before this have undefined effect.
    RCC_ClockCmd(pDev->rcc, ENABLE);

    // Claim the pins. IOInit's resource-owner records let the CLI `resource`
    // command surface pin conflicts against existing drivers.
    IO_t txIO = IOGetByTag(pDev->tx);
    IO_t rxIO = IOGetByTag(pDev->rx);

    IOInit(txIO, OWNER_CAN_TX, RESOURCE_INDEX(device));
    IOInit(rxIO, OWNER_CAN_RX, RESOURCE_INDEX(device));

    // TX is a push-pull alternate; RX needs a pull-up so the line idles
    // recessive when no transceiver is connected (prevents spurious dominant
    // readings during initialisation).
    IOConfigGPIOAF(txIO, IOCFG_AF_PP, pDev->txAF);
    IOConfigGPIOAF(rxIO, IOCFG_AF_PP_UP, pDev->rxAF);

    if (!canEnterConfigMode(regs)) {
        // Peripheral did not latch INIT; nothing more we can do safely.
        return;
    }

    // Classic CAN: clear the FD-enable and bit-rate-switch bits. Everything
    // else in CCCR can be left at reset (normal mode, no monitoring).
    regs->CCCR &= ~(FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE);

    // Nominal bit timing. Data bit timing (DBTP) is not needed for classic CAN.
    // If synthesis fails the caller-visible `initialized` flag stays false so
    // canTransmit() rejects traffic rather than running at an unknown rate.
    uint32_t nbtp;
    if (!canNominalBitTiming(bitrate, &nbtp)) {
        return;
    }
    regs->NBTP = nbtp;

    // Wipe the per-instance message RAM region to remove any prior contents.
    canClearMessageRam(device);

#if defined(STM32G4) || defined(STM32C5)
    // Accept-non-matching policy: route everything to Rx FIFO 0 (fields 00b
    // in ANFS / ANFE). The Message RAM layout itself is fixed by silicon.
    regs->RXGFC &= ~(FDCAN_RXGFC_ANFS | FDCAN_RXGFC_ANFE);
#elif defined(STM32H7)
    // H7 uses a separate GFC register and requires explicit Message RAM
    // configuration via SIDFC/XIDFC/RXF0C/TXBC/RXESC/TXESC.
    regs->GFC &= ~(FDCAN_GFC_ANFS | FDCAN_GFC_ANFE);
    canConfigureMessageRamH7(device, regs);
#endif

    // Enable Rx FIFO 0 new-message and message-lost interrupts, and arm
    // the peripheral side of IRQ line 0. The line-select register (ILS)
    // resets to 0 which routes all sources to line 0, so no write needed.
    // Enabling RF0LE lets the IRQ path observe and clear overrun flags so
    // the peripheral does not stay stuck in the "lost" state after a burst.
    regs->IE  |= FDCAN_IE_RF0NE | FDCAN_IE_RF0LE;
    regs->ILE |= FDCAN_ILE_EINT0;

    canEnableInterrupt(device, pDev->irq0);

    if (!canExitConfigMode(regs)) {
        // Peripheral stuck in init — leave device flagged uninitialised so
        // canTransmit() and canInit()'s caller see the failure.
        return;
    }

    pDev->initialized = true;
}

bool canInit(canDevice_e device, uint32_t bitrate)
{
    if (device < 0 || device >= CANDEV_COUNT) {
        return false;
    }

    canInitDevice(device, bitrate);
    return canDevice[device].initialized;
}

void canRegisterRxCallback(canDevice_e device, canRxCallbackPtr callback)
{
    if (device < 0 || device >= CANDEV_COUNT) {
        return;
    }

    canDevice[device].rxCallback = callback;
}

// NOTE: canTransmit() is NOT internally synchronised. The read-modify-write
// sequence on the Tx FIFO (read TXFQS put-index, write slot, set TXBAR) is
// not re-entrant; calling concurrently from thread context and an ISR can
// corrupt messages. Callers must serialise externally if they need that.
bool canTransmit(canDevice_e device, uint32_t identifier, bool isExtended,
                 const uint8_t *data, uint8_t length)
{
    if (device < 0 || device >= CANDEV_COUNT) {
        return false;
    }

    canDevice_t *pDev = &canDevice[device];
    if (!pDev->initialized || length > CAN_CLASSIC_MAX_DLC) {
        return false;
    }

    // Per API contract: data may be NULL only if length is zero. Reject
    // the invalid combination rather than dereferencing a null pointer.
    if (length > 0 && data == NULL) {
        return false;
    }

    FDCAN_GlobalTypeDef *regs = canRegs(device);

    // Bail early if the hardware FIFO is full. The caller decides whether to
    // retry or drop; we don't block so the ISR-side callers stay non-blocking.
    uint32_t txfqs = regs->TXFQS;
    if (txfqs & FDCAN_TXFQS_TFQF) {
        return false;
    }

    // Put index tells us which slot in the Tx FIFO the next message belongs
    // in. The peripheral manages the ring; software just fills the slot and
    // sets the corresponding TXBAR bit.
    uint32_t putIndex = (txfqs & FDCAN_TXFQS_TFQPI) >> 16;
    volatile uint32_t *slot = (volatile uint32_t *)canTxElementAddress(device, putIndex);

    // Word 0: identifier with XTD flag if extended.
    if (isExtended) {
        slot[0] = CAN_TX_WORD0_XTD | (identifier & 0x1FFFFFFFUL);
    } else {
        slot[0] = (identifier & 0x7FFUL) << 18;
    }

    // Word 1: length only (no BRS/FDF -> classic CAN frame).
    slot[1] = (uint32_t)length << CAN_TX_WORD1_DLC_POS;

    // Words 2..3: copy the payload in 32-bit chunks. The Tx RAM requires
    // 32-bit writes and classic CAN tops out at 8 bytes of payload, so two
    // words is always enough.
    uint32_t words[2] = { 0, 0 };
    for (uint8_t i = 0; i < length; i++) {
        words[i >> 2] |= ((uint32_t)data[i]) << ((i & 3U) * 8U);
    }
    slot[2] = words[0];
    slot[3] = words[1];

    // Trigger transmission of this slot.
    regs->TXBAR = 1UL << putIndex;

    return true;
}

//-----------------------------------------------------------------------------
// ISR dispatch
//-----------------------------------------------------------------------------

static void canDispatchRx(canDevice_e device)
{
    canDevice_t *pDev = &canDevice[device];
    FDCAN_GlobalTypeDef *regs = canRegs(device);

    // Drain the FIFO. F0FL = fill level; looping until it hits zero guarantees
    // we handle multiple arrivals that may have accumulated before the ISR ran.
    while (regs->RXF0S & FDCAN_RXF0S_F0FL) {
        uint32_t getIndex = (regs->RXF0S & FDCAN_RXF0S_F0GI) >> 8;

        volatile const uint32_t *slot =
            (volatile const uint32_t *)canRxElementAddress(device, getIndex);

        uint32_t w0 = slot[0];
        uint32_t w1 = slot[1];

        bool isExtended = (w0 & CAN_RX_WORD0_XTD) != 0;
        uint32_t identifier = isExtended
            ? (w0 & CAN_RX_WORD0_EXTID_MASK)
            : ((w0 & CAN_RX_WORD0_STDID_MASK) >> CAN_RX_WORD0_STDID_POS);

        uint8_t dlc = (w1 & CAN_RX_WORD1_DLC_MASK) >> CAN_RX_WORD1_DLC_POS;
        if (dlc > CAN_CLASSIC_MAX_DLC) {
            dlc = CAN_CLASSIC_MAX_DLC;        // classic CAN safety clamp
        }

        uint8_t payload[CAN_CLASSIC_MAX_DLC];
        uint32_t w2 = slot[2];
        uint32_t w3 = slot[3];
        for (uint8_t i = 0; i < dlc; i++) {
            uint32_t word = (i < 4) ? w2 : w3;
            payload[i] = (uint8_t)(word >> ((i & 3U) * 8U));
        }

        // Ack the slot so the peripheral can recycle it for the next message.
        // Must be done before invoking the callback in case the callback is
        // slow; pending messages can still queue in other slots.
        regs->RXF0A = getIndex;

        if (pDev->rxCallback) {
            pDev->rxCallback(identifier, isExtended, payload, dlc);
        }
    }
}

void canIrqHandler(canDevice_e device)
{
    FDCAN_GlobalTypeDef *regs = canRegs(device);

    uint32_t ir = regs->IR;

    if (ir & FDCAN_IR_RF0N) {
        regs->IR = FDCAN_IR_RF0N;             // write-1-to-clear
        canDispatchRx(device);
    }

    // RF0L = Rx FIFO 0 message lost (overrun). Drop the flag so the
    // peripheral re-arms; dropped frames are visible in the device state
    // via an incrementing counter for diagnostics.
    if (ir & FDCAN_IR_RF0L) {
        regs->IR = FDCAN_IR_RF0L;
        canDevice[device].rxOverruns++;
    }
}

// Strong definitions of the FDCAN IRQ handlers (weak stubs in startup *.s).
// These only exist when CAN is compiled in; otherwise the linker keeps the
// weak Default_Handler stubs. FDCAN3 is guarded with #ifdef so H7 variants
// without that instance (H74x/H75x) do not pick up an unresolved symbol.

void FDCAN1_IT0_IRQHandler(void)
{
    canIrqHandler(CANDEV_1);
}

void FDCAN2_IT0_IRQHandler(void)
{
    canIrqHandler(CANDEV_2);
}

#if defined(FDCAN3)
void FDCAN3_IT0_IRQHandler(void)
{
    canIrqHandler(CANDEV_3);
}
#endif

#endif // ENABLE_CAN
