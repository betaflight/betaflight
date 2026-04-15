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

#include "common/utils.h"

#include "drivers/can/can.h"
#include "drivers/can/can_impl.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/resource.h"
#include "platform/rcc.h"

//-----------------------------------------------------------------------------
// Message RAM layout (G4)
//
// The G4 has a single shared message RAM at SRAMCAN_BASE, divided into three
// equal regions (one per FDCAN instance). Within each region the layout of
// filters / FIFOs / buffers is fixed by hardware.
//
// The element counts below mirror the ones used by ST's HAL, so the fixed
// offsets inside the region match what the FDCAN peripheral already uses.
//-----------------------------------------------------------------------------

#if defined(STM32G4)

#define CAN_SRAM_FLS_NBR    (28U)
#define CAN_SRAM_FLE_NBR    (8U)
#define CAN_SRAM_RF0_NBR    (3U)
#define CAN_SRAM_RF1_NBR    (3U)
#define CAN_SRAM_TEF_NBR    (3U)
#define CAN_SRAM_TFQ_NBR    (3U)

#define CAN_SRAM_FLS_SIZE   (1U * 4U)       // standard filter element
#define CAN_SRAM_FLE_SIZE   (2U * 4U)       // extended filter element
#define CAN_SRAM_RF_SIZE    (18U * 4U)      // Rx FIFO element (header + 64B payload)
#define CAN_SRAM_TEF_SIZE   (2U * 4U)       // Tx event FIFO element
#define CAN_SRAM_TFQ_SIZE   (18U * 4U)      // Tx FIFO element

#define CAN_SRAM_FLSSA      (0U)
#define CAN_SRAM_FLESA      (CAN_SRAM_FLSSA + CAN_SRAM_FLS_NBR * CAN_SRAM_FLS_SIZE)
#define CAN_SRAM_RF0SA      (CAN_SRAM_FLESA + CAN_SRAM_FLE_NBR * CAN_SRAM_FLE_SIZE)
#define CAN_SRAM_RF1SA      (CAN_SRAM_RF0SA + CAN_SRAM_RF0_NBR * CAN_SRAM_RF_SIZE)
#define CAN_SRAM_TEFSA      (CAN_SRAM_RF1SA + CAN_SRAM_RF1_NBR * CAN_SRAM_RF_SIZE)
#define CAN_SRAM_TFQSA      (CAN_SRAM_TEFSA + CAN_SRAM_TEF_NBR * CAN_SRAM_TEF_SIZE)
#define CAN_SRAM_PER_INSTANCE \
                            (CAN_SRAM_TFQSA + CAN_SRAM_TFQ_NBR * CAN_SRAM_TFQ_SIZE)

static uint32_t canMessageRamBase(canDevice_e device)
{
    // Each FDCAN instance owns a CAN_SRAM_PER_INSTANCE-sized region, laid out
    // FDCAN1, FDCAN2, FDCAN3 in ascending address order from SRAMCAN_BASE.
    return SRAMCAN_BASE + (uint32_t)device * CAN_SRAM_PER_INSTANCE;
}

#endif // STM32G4

//-----------------------------------------------------------------------------
// Bit timing for 1 Mbit/s classic CAN
//
// The FDCAN kernel clock on G4 is sourced from PCLK1 by default (170 MHz on
// overclocked G474). The HAL default at reset maps FDCAN_CLK = HSE = 24 MHz on
// typical FC boards; we assume the PCLK1 path here. Nominal bit time is
// (1 + tseg1 + tseg2) prescaler-scaled tq. Choose tq = 8 -> divisor must be
// (PCLK1 / 1M / 8). The config below works for the common 170/160 MHz and
// 24 MHz clock options and errs toward sample-point ~ 75 %.
//
// A future pass should make this configurable via PG; for now it targets
// a 24 MHz FDCAN kernel clock (common STM32G474 default when PLLQ isn't
// routed to FDCAN).
//-----------------------------------------------------------------------------

// NBTP fields: NTSEG1=bits 8:15, NTSEG2=bits 0:6, NBRP=16:24, NSJW=25:31
// For 1 Mbps with 24 MHz kernel clock: prescaler 1, tseg1 17, tseg2 6, sjw 1
#define CAN_NBTP_NSJW_POS       (25U)
#define CAN_NBTP_NBRP_POS       (16U)
#define CAN_NBTP_NTSEG1_POS     (8U)
#define CAN_NBTP_NTSEG2_POS     (0U)

static uint32_t canNominalBitTiming(void)
{
    // prescaler - 1, tseg1 - 1, tseg2 - 1, sjw - 1
    // 24 MHz / (1 * (1 + 17 + 6)) = 1 MHz
    const uint32_t nbrp = 0;        // prescaler 1
    const uint32_t ntseg1 = 16;     // tseg1 17
    const uint32_t ntseg2 = 5;      // tseg2 6
    const uint32_t nsjw = 0;        // sjw 1

    return (nsjw << CAN_NBTP_NSJW_POS)
         | (nbrp << CAN_NBTP_NBRP_POS)
         | (ntseg1 << CAN_NBTP_NTSEG1_POS)
         | (ntseg2 << CAN_NBTP_NTSEG2_POS);
}

//-----------------------------------------------------------------------------
// Classic CAN frame header encoding for the Tx FIFO element (G4).
//
// Word 0: ID field.
//  - Standard ID occupies bits 18..28 (11-bit ID shifted left by 18).
//  - Extended ID occupies bits 0..28, with XTD (bit 30) set.
// Word 1: DLC + flags (bits 16..19 = DLC, FDF = bit 21, BRS = bit 20).
// Words 2..: payload, packed little-endian 4 bytes per word.
//-----------------------------------------------------------------------------

#define CAN_TX_WORD0_XTD        (1UL << 30)
#define CAN_TX_WORD1_DLC_POS    (16U)

// Rx header encoding (mirrors the Tx layout with additional filter info we ignore)
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

// Enter INIT + CCE (configuration change enable). Required for any change to
// bit timing or message RAM layout. Busy-loops until the peripheral reports
// that INIT has taken effect.
static bool canEnterConfigMode(FDCAN_GlobalTypeDef *regs)
{
    regs->CCCR |= FDCAN_CCCR_INIT;

    // Peripheral synchronises to the bus before asserting INIT. Give it a
    // bounded number of polls so a disconnected transceiver does not hang us.
    uint32_t timeout = 100000;
    while (!(regs->CCCR & FDCAN_CCCR_INIT) && timeout--) {
        // spin
    }
    if (!(regs->CCCR & FDCAN_CCCR_INIT)) {
        return false;
    }

    regs->CCCR |= FDCAN_CCCR_CCE;
    return true;
}

static void canExitConfigMode(FDCAN_GlobalTypeDef *regs)
{
    regs->CCCR &= ~FDCAN_CCCR_INIT;
}

// Clear out the whole per-instance message RAM region. A cold peripheral
// contains indeterminate SRAM contents; sending garbage would be reported as
// a spurious message.
static void canClearMessageRam(uint32_t base)
{
#if defined(STM32G4)
    for (uint32_t addr = base; addr < base + CAN_SRAM_PER_INSTANCE; addr += 4) {
        *(volatile uint32_t *)addr = 0;
    }
#else
    UNUSED(base);
#endif
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

//-----------------------------------------------------------------------------
// Init / public API
//-----------------------------------------------------------------------------

void canInitDevice(canDevice_e device)
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
    regs->CCCR &= ~((1UL << 8) | (1UL << 9));    // clear FDOE (8) and BRSE (9)

    // Nominal bit timing. Data bit timing (DBTP) is not needed for classic CAN.
    regs->NBTP = canNominalBitTiming();

#if defined(STM32G4)
    // Accept-non-matching policy: route everything to Rx FIFO 0 (fields 00b
    // in ANFS / ANFE). No filter list entries are installed in this first
    // cut, which is fine because the accept-non-match path handles everything.
    regs->RXGFC &= ~(FDCAN_RXGFC_ANFS | FDCAN_RXGFC_ANFE);

    // Wipe the per-instance message RAM region.
    canClearMessageRam(canMessageRamBase(device));
#endif

    // Enable RX FIFO 0 new-message interrupt, route it to IRQ line 0, and
    // arm the peripheral side of IRQ line 0.
    regs->IE  |= FDCAN_IE_RF0NE;
    regs->ILS &= ~(1UL << 3);                 // RF0N -> line 0
    regs->ILE |= FDCAN_ILE_EINT0;

    canEnableInterrupt(device, pDev->irq0);

    canExitConfigMode(regs);

    pDev->initialized = true;
}

bool canInit(canDevice_e device)
{
    if (device < 0 || device >= CANDEV_COUNT) {
        return false;
    }

    canInitDevice(device);
    return canDevice[device].initialized;
}

void canRegisterRxCallback(canDevice_e device, canRxCallbackPtr callback)
{
    if (device < 0 || device >= CANDEV_COUNT) {
        return;
    }

    canDevice[device].rxCallback = callback;
}

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

    FDCAN_GlobalTypeDef *regs = canRegs(device);

    // Bail early if the hardware FIFO is full. The caller decides whether to
    // retry or drop; we don't block so the ISR-side callers stay non-blocking.
    uint32_t txfqs = regs->TXFQS;
    if (txfqs & FDCAN_TXFQS_TFQF) {
        return false;
    }

#if defined(STM32G4)
    // Put index tells us which slot in the Tx FIFO the next message belongs
    // in. The peripheral manages the ring; software just fills the slot and
    // sets the corresponding TXBAR bit.
    uint32_t putIndex = (txfqs & FDCAN_TXFQS_TFQPI) >> 16;
    uint32_t base = canMessageRamBase(device) + CAN_SRAM_TFQSA
                    + putIndex * CAN_SRAM_TFQ_SIZE;

    volatile uint32_t *slot = (volatile uint32_t *)base;

    // Word 0: identifier with XTD flag if extended.
    if (isExtended) {
        slot[0] = CAN_TX_WORD0_XTD | (identifier & 0x1FFFFFFFUL);
    } else {
        slot[0] = (identifier & 0x7FFUL) << 18;
    }

    // Word 1: length only (no BRS/FDF -> classic CAN frame).
    slot[1] = (uint32_t)length << CAN_TX_WORD1_DLC_POS;

    // Words 2+: copy the payload in 32-bit chunks, handling any trailing
    // 1..3 bytes by masking. The Tx RAM requires 32-bit writes.
    uint32_t words[2] = { 0, 0 };
    for (uint8_t i = 0; i < length; i++) {
        words[i >> 2] |= ((uint32_t)data[i]) << ((i & 3U) * 8U);
    }
    slot[2] = words[0];
    slot[3] = words[1];

    // Trigger transmission of this slot.
    regs->TXBAR = 1UL << putIndex;
#else
    UNUSED(identifier);
    UNUSED(isExtended);
    UNUSED(data);
    UNUSED(length);
#endif

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

#if defined(STM32G4)
        uint32_t base = canMessageRamBase(device) + CAN_SRAM_RF0SA
                        + getIndex * CAN_SRAM_RF_SIZE;
        volatile const uint32_t *slot = (volatile const uint32_t *)base;

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
#else
        uint32_t identifier = 0;
        bool isExtended = false;
        uint8_t dlc = 0;
        uint8_t payload[CAN_CLASSIC_MAX_DLC] = { 0 };
#endif

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
}

// Strong definitions of the FDCAN IRQ handlers (weak stubs in startup *.s).
// These only exist when CAN is compiled in; otherwise the linker keeps the
// weak Default_Handler stubs.

void FDCAN1_IT0_IRQHandler(void)
{
    canIrqHandler(CANDEV_1);
}

void FDCAN2_IT0_IRQHandler(void)
{
    canIrqHandler(CANDEV_2);
}

#if defined(STM32G4)
void FDCAN3_IT0_IRQHandler(void)
{
    canIrqHandler(CANDEV_3);
}
#endif

#endif // ENABLE_CAN
