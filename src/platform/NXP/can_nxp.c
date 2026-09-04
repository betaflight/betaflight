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
 * FlexCAN driver for the NXP parts, classic CAN only. CTRL1[LBUF] keeps transmit
 * order and the legacy receive FIFO keeps arrival order.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if ENABLE_CAN

#include "common/time.h"
#include "common/utils.h"

#include "drivers/can/can.h"
#include "drivers/can/can_impl.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/resource.h"
#include "drivers/time.h"

#include "platform/nxp_clock.h"

#if ENABLE_DRONECAN
#include "pg/dronecan.h"
#endif

#include "iomux_nxp.h"
#include "iomux_pad_def.h"
#include "iomux_signal_def.h"

// Only CAN_RX_FIFO is implemented.
typedef enum {
    CAN_RX_FIFO = 0,
    CAN_RX_MAILBOX,
} canRxStructure_e;

// From the vendor feature headers: FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL,
// _HAS_ERRATA_5829, _INSTANCE_HAS_FLEXIBLE_DATA_RATEn and
// _HAS_MESSAGE_BUFFER_MAX_NUMBERn.
typedef struct canCapability_s {
    nxpClock_e clk;               // clock gate and rate of this controller
    uint8_t mbCount;                // message buffers the RAM holds
    bool fdCapable;                 // unused: no FD path is compiled
    bool eccInit;                   // message RAM needs seeding before it is read
    bool txErrataMb;                // ERR005829 wants a reserved mailbox in front
    canRxStructure_e rxStructure;   // always CAN_RX_FIFO
} canCapability_t;

#if defined(IMXRT11XX)

// The af column is unused; muxing comes from nxpFlexcanPins.
const canHardware_t canHardware[CANDEV_COUNT] = {
    {
        .device = CANDEV_1,
        .reg = (canResource_t *)CAN1,
        .txPins = {
            { DEFIO_TAG_E(GPIO_AD_06), 0 },
            { DEFIO_TAG_E(GPIO_DISP_B2_12), 0 },
            { DEFIO_TAG_E(GPIO_DISP_B2_14), 0 },
        },
        .rxPins = {
            { DEFIO_TAG_E(GPIO_AD_07), 0 },
            { DEFIO_TAG_E(GPIO_DISP_B2_13), 0 },
            { DEFIO_TAG_E(GPIO_DISP_B2_15), 0 },
        },
        .irq0 = CAN1_IRQn,
        .irq1 = CAN1_ERROR_IRQn,
    },
    {
        .device = CANDEV_2,
        .reg = (canResource_t *)CAN2,
        .txPins = {
            { DEFIO_TAG_E(GPIO_AD_00), 0 },
            { DEFIO_TAG_E(GPIO_AD_30), 0 },
        },
        .rxPins = {
            { DEFIO_TAG_E(GPIO_AD_01), 0 },
            { DEFIO_TAG_E(GPIO_AD_31), 0 },
        },
        .irq0 = CAN2_IRQn,
        .irq1 = CAN2_ERROR_IRQn,
    },
    {
        .device = CANDEV_3,
        .reg = (canResource_t *)CAN3,
        .txPins = {
            { DEFIO_TAG_E(GPIO_LPSR_00), 0 },
            { DEFIO_TAG_E(GPIO_LPSR_06), 0 },
            { DEFIO_TAG_E(GPIO_LPSR_08), 0 },
        },
        .rxPins = {
            { DEFIO_TAG_E(GPIO_LPSR_01), 0 },
            { DEFIO_TAG_E(GPIO_LPSR_07), 0 },
            { DEFIO_TAG_E(GPIO_LPSR_09), 0 },
        },
        .irq0 = CAN3_IRQn,
        .irq1 = CAN3_ERROR_IRQn,
    },
};

static const canCapability_t canCapability[CANDEV_COUNT] = {
    [CANDEV_1] = { NXP_CLK_CAN1, 64, true, true, false, CAN_RX_FIFO },
    [CANDEV_2] = { NXP_CLK_CAN2, 64, true, true, false, CAN_RX_FIFO },
    [CANDEV_3] = { NXP_CLK_CAN3, 64, true, true, false, CAN_RX_FIFO },
};

#elif defined(IMXRT10XX)

// The af column is unused; muxing comes from nxpFlexcanPins.
const canHardware_t canHardware[CANDEV_COUNT] = {
    {
        .device = CANDEV_1,
        .reg = (canResource_t *)CAN1,
        .txPins = {
            { DEFIO_TAG_E(GPIO_EMC_17), 0 },
            { DEFIO_TAG_E(GPIO_AD_B1_08), 0 },
            { DEFIO_TAG_E(GPIO_B0_02), 0 },
            { DEFIO_TAG_E(GPIO_SD_B1_02), 0 },
        },
        .rxPins = {
            { DEFIO_TAG_E(GPIO_EMC_18), 0 },
            { DEFIO_TAG_E(GPIO_AD_B1_09), 0 },
            { DEFIO_TAG_E(GPIO_B0_03), 0 },
            { DEFIO_TAG_E(GPIO_SD_B1_03), 0 },
        },
        .irq0 = CAN1_IRQn,
        .irq1 = CAN1_IRQn,
    },
    {
        .device = CANDEV_2,
        .reg = (canResource_t *)CAN2,
        .txPins = {
            { DEFIO_TAG_E(GPIO_EMC_09), 0 },
            { DEFIO_TAG_E(GPIO_AD_B0_02), 0 },
            { DEFIO_TAG_E(GPIO_AD_B0_14), 0 },
            { DEFIO_TAG_E(GPIO_B1_08), 0 },
        },
        .rxPins = {
            { DEFIO_TAG_E(GPIO_EMC_10), 0 },
            { DEFIO_TAG_E(GPIO_AD_B0_03), 0 },
            { DEFIO_TAG_E(GPIO_AD_B0_15), 0 },
            { DEFIO_TAG_E(GPIO_B1_09), 0 },
        },
        .irq0 = CAN2_IRQn,
        .irq1 = CAN2_IRQn,
    },
    {
        .device = CANDEV_3,
        .reg = (canResource_t *)CAN3,
        .txPins = {
            { DEFIO_TAG_E(GPIO_EMC_36), 0 },
            { DEFIO_TAG_E(GPIO_AD_B0_10), 0 },
            { DEFIO_TAG_E(GPIO_AD_B0_14), 0 },
        },
        .rxPins = {
            { DEFIO_TAG_E(GPIO_EMC_37), 0 },
            { DEFIO_TAG_E(GPIO_AD_B0_11), 0 },
            { DEFIO_TAG_E(GPIO_AD_B0_15), 0 },
        },
        .irq0 = CAN3_IRQn,
        .irq1 = CAN3_IRQn,
    },
};

// One interrupt vector per controller, so irq1 repeats irq0 above.
static const canCapability_t canCapability[CANDEV_COUNT] = {
    [CANDEV_1] = { NXP_CLK_CAN1, 64, false, false, true, CAN_RX_FIFO },
    [CANDEV_2] = { NXP_CLK_CAN2, 64, false, false, true, CAN_RX_FIFO },
    [CANDEV_3] = { NXP_CLK_CAN3, 64, true,  false, true, CAN_RX_FIFO },
};

#else
#error "Unsupported NXP family: no FlexCAN capability table (add the family block here)"
#endif

// The clock entry is NXP_CLK_CAN1 plus the device index.
STATIC_ASSERT(NXP_CLK_GPT1 - NXP_CLK_CAN1 >= CANDEV_COUNT,
              clock_enum_needs_one_entry_per_flexcan_instance);

// With MCR[RFEN] set and CTRL2[RFFN] zero the FIFO owns buffers 0 to 5 (output
// at buffer 0) and its eight filters sit in buffers 6 and 7.
#define CAN_FIFO_OUTPUT_MB      0u
#define CAN_FIFO_FILTER_COUNT   8u
#define CAN_FIRST_FREE_MB       8u

// Mailboxes in the transmit window, above the ERR005829 one where needed.
#define CAN_TX_WINDOW           4u

// Receive FIFO interrupt flag bits.
#define CAN_IFLAG_FIFO_AVAILABLE (1u << 5)
#define CAN_IFLAG_FIFO_WARNING   (1u << 6)
#define CAN_IFLAG_FIFO_OVERFLOW  (1u << 7)

// IFLAG1 and IMASK1 reach buffers 0 to 31 only.
STATIC_ASSERT(CAN_FIRST_FREE_MB + 1u + CAN_TX_WINDOW <= 32u,
              can_tx_window_runs_past_the_first_interrupt_flag_register);

// MCR[IDAM]: one full identifier, standard or extended, per filter element.
#define CAN_IDAM_FORMAT_A       0u

// Message buffer CODE values; CAN_CODE_TX_ERRATA is for ERR005829, see canBringUp().
#define CAN_CODE_TX_INACTIVE    0x8u
#define CAN_CODE_TX_ABORT       0x9u
#define CAN_CODE_TX_DATA        0xCu
#define CAN_CODE_TX_ERRATA      0x3u

// Standard identifier is bits 28:18 of the ID word, extended is bits 28:0.
#define CAN_ID_STANDARD_MASK    0x7FFu
#define CAN_ID_EXTENDED_MASK    0x1FFFFFFFu

// ECC protected message RAM ranges that must be written before any read.
#define CAN_ECC_RAM1_START      0x080u
#define CAN_ECC_RAM1_END        0xAE0u
#define CAN_ECC_RAM2_START      0xF28u
#define CAN_ECC_RAM2_END        0x1000u

// Bit timing: 16 quanta per bit sampled at 87.5%; fields hold quanta minus one,
// and only the prescaler changes with the bit rate.
#define CAN_TQ_PER_BIT          16u
#define CAN_PROPSEG             7u
#define CAN_PSEG1               4u
#define CAN_PSEG2               1u
#define CAN_RJW                 1u

STATIC_ASSERT(1u + (CAN_PROPSEG + 1u) + (CAN_PSEG1 + 1u) + (CAN_PSEG2 + 1u) == CAN_TQ_PER_BIT,
              can_bit_segments_do_not_add_up_to_the_quantum_count);
STATIC_ASSERT(CAN_RJW <= CAN_PSEG2,
              can_jump_width_is_longer_than_phase_segment_two);

#define CAN_PRESDIV_AT_80MHZ(bitrate)   (80000000u / (CAN_TQ_PER_BIT * (bitrate)) - 1u)

STATIC_ASSERT(CAN_PRESDIV_AT_80MHZ(1000000u) == 4u, can_prescaler_wrong_at_1_mbit);
STATIC_ASSERT(CAN_PRESDIV_AT_80MHZ(500000u) == 9u, can_prescaler_wrong_at_500_kbit);
STATIC_ASSERT(CAN_PRESDIV_AT_80MHZ(250000u) == 19u, can_prescaler_wrong_at_250_kbit);
STATIC_ASSERT(CAN_PRESDIV_AT_80MHZ(125000u) == 39u, can_prescaler_wrong_at_125_kbit);

// Bound on every hardware handshake wait; freeze entry waits for the frame on the wire.
#define CAN_HANDSHAKE_TIMEOUT_US 10000

// No transmit completion for this long with the ring full means a wedged bus.
#define CAN_TX_STALL_TIMEOUT_US  100000

// Slot numbers relative to canTxMailbox(); canTxKick() has the rules.
static struct {
    uint8_t busy;       // one bit per slot
    uint8_t loaded;     // slots filled since the window last rewound
} canTx[CANDEV_COUNT];

STATIC_ASSERT(CAN_TX_WINDOW <= 8u, can_tx_window_does_not_fit_the_busy_mask);

static CAN_Type *canRegs(canDevice_e device)
{
    return (CAN_Type *)canDevice[device].reg;
}

// Write one to clear; on the receive FIFO this write also pops the frame.
static inline void canClearFlags(CAN_Type *can, uint32_t flags)
{
    can->IFLAG1 = flags;
}

// The ERR005829 reserved mailbox sits below this one.
static uint32_t canTxMailbox(canDevice_e device)
{
    return canCapability[device].txErrataMb ? CAN_FIRST_FREE_MB + 1u : CAN_FIRST_FREE_MB;
}

static uint32_t canTxWindowFlags(canDevice_e device)
{
    return ((1u << CAN_TX_WINDOW) - 1u) << canTxMailbox(device);
}

static bool canWaitMcr(const CAN_Type *can, uint32_t mask, bool set)
{
    const timeUs_t start = micros();

    while (((can->MCR & mask) != 0) != set) {
        if (cmpTimeUs(micros(), start) >= (timeDelta_t)CAN_HANDSHAKE_TIMEOUT_US) {
            return false;
        }
    }

    return true;
}

static bool canWaitFlag(const CAN_Type *can, uint32_t flag)
{
    const timeUs_t start = micros();

    while (!(can->IFLAG1 & flag)) {
        if (cmpTimeUs(micros(), start) >= (timeDelta_t)CAN_HANDSHAKE_TIMEOUT_US) {
            return false;
        }
    }

    return true;
}

// FRZ and HALT must move together to enter or leave freeze mode.
static bool canSetFreeze(CAN_Type *can, bool freeze)
{
    if (freeze) {
        can->MCR |= CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK;
    } else {
        can->MCR &= ~(CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK);
    }

    return canWaitMcr(can, CAN_MCR_FRZACK_MASK, freeze);
}

static void canZeroRam(CAN_Type *can, uint32_t from, uint32_t to)
{
    volatile uint32_t *const base = (volatile uint32_t *)can;

    for (uint32_t offset = from; offset < to; offset += 4) {
        base[offset / 4] = 0;
    }
}

// Payload words hold byte 0 in the most significant byte.
static uint32_t canDataToWord(const uint8_t *data, uint8_t length, uint8_t first)
{
    uint32_t word = 0;

    for (uint8_t i = 0; i < 4u; i++) {
        const uint8_t index = first + i;

        if (index < length) {
            word |= (uint32_t)data[index] << (24u - 8u * i);
        }
    }

    return word;
}

static void canWordToData(uint32_t word, uint8_t *data, uint8_t length, uint8_t first)
{
    for (uint8_t i = 0; i < 4u; i++) {
        const uint8_t index = first + i;

        if (index < length) {
            data[index] = (uint8_t)(word >> (24u - 8u * i));
        }
    }
}

// Pin muxing for a controller signal, NULL if the pin cannot carry it.
static const nxpPinMux_t *canPadMux(canDevice_e device, bool rx, ioTag_t pin)
{
    return NXP_PIN_MUX(nxpFlexcanPins, (uint8_t)(device - CANDEV_1 + 1),
                         rx ? NXP_ROLE_FLEXCAN_RX : NXP_ROLE_FLEXCAN_TX, pin);
}

// The prescaler for one bit rate, or false when the clock cannot make it exactly.
static bool canPrescaler(canDevice_e device, uint32_t bitrate, uint32_t *presdiv)
{
    const uint32_t clockHz = nxpClockHz(canCapability[device].clk);

    if (bitrate == 0 || clockHz == 0) {
        return false;
    }

    const uint32_t quanta = CAN_TQ_PER_BIT * bitrate;

    if (clockHz % quanta) {
        return false;
    }

    const uint32_t divisor = clockHz / quanta;

    // PRESDIV is eight bits and holds the divisor minus one.
    if (divisor == 0 || divisor > 256) {
        return false;
    }

    *presdiv = divisor - 1;

    return true;
}

// Unmask only the topmost filled mailbox; with LBUF it finishes last.
static FAST_IRQ_HANDLER void canTxArm(canDevice_e device)
{
    const uint8_t loaded = canTx[device].loaded;

    canRegs(device)->IMASK1 = CAN_IFLAG_FIFO_AVAILABLE | CAN_IFLAG_FIFO_OVERFLOW
        | (loaded ? 1u << (canTxMailbox(device) + loaded - 1u) : 0u);
}

// Mask every vector that can reach canTxKick(), both on the 11xx.
static void canIrqMask(const canDevice_t *pDev)
{
    NVIC_DisableIRQ((IRQn_Type)pDev->irq0);

    if (pDev->irq1 != pDev->irq0) {
        NVIC_DisableIRQ((IRQn_Type)pDev->irq1);
    }
}

static void canIrqUnmask(const canDevice_t *pDev)
{
    // Keep the window state stores ahead of the re-enable.
    __asm volatile ("" ::: "memory");

    NVIC_EnableIRQ((IRQn_Type)pDev->irq0);

    if (pDev->irq1 != pDev->irq0) {
        NVIC_EnableIRQ((IRQn_Type)pDev->irq1);
    }
}

// Reference manual bring-up order: leave low power mode, confirmed freeze, seed
// the ECC RAM (with WRMFRZ set only while seeding), soft reset.
static bool canResetModule(CAN_Type *can, const canCapability_t *cap)
{
    can->MCR &= ~CAN_MCR_MDIS_MASK;

    if (!canWaitMcr(can, CAN_MCR_LPMACK_MASK, false)) {
        return false;
    }

    // Several MCR and CTRL2 fields are only written in confirmed freeze.
    if (!canSetFreeze(can, true)) {
        return false;
    }

    if (cap->eccInit) {
        can->CTRL2 |= CAN_CTRL2_WRMFRZ_MASK;

        canZeroRam(can, CAN_ECC_RAM1_START, CAN_ECC_RAM1_END);
        canZeroRam(can, CAN_ECC_RAM2_START, CAN_ECC_RAM2_END);

        can->CTRL2 &= ~CAN_CTRL2_WRMFRZ_MASK;
    }

    // Soft reset leaves the module enabled and in freeze.
    can->MCR |= CAN_MCR_SOFTRST_MASK;

    return canWaitMcr(can, CAN_MCR_SOFTRST_MASK, false);
}

static bool canBringUp(canDevice_e device, uint32_t presdiv)
{
    CAN_Type *const can = canRegs(device);
    const canCapability_t *const cap = &canCapability[device];
    const uint32_t txMb = canTxMailbox(device);

    if (!canResetModule(can, cap)) {
        return false;
    }

    /*
     *   FRZ and HALT   stay in freeze for the rest of the sequence
     *   SRXDIS         do not receive our own frames back
     *   IRMQ           each filter element gets its own mask in RXIMR
     *   AEN            transmit abort is a handshake, which canTxAbort() needs
     *   RFEN           the legacy receive FIFO
     *   WRNEN          the error counter warning flags
     *   IDAM           one full identifier per filter element
     *   MAXMB          the highest buffer in use, the top of the window
     */
    can->MCR = CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK
             | CAN_MCR_SRXDIS_MASK | CAN_MCR_IRMQ_MASK | CAN_MCR_AEN_MASK
             | CAN_MCR_RFEN_MASK | CAN_MCR_WRNEN_MASK
             | CAN_MCR_IDAM(CAN_IDAM_FORMAT_A)
             | CAN_MCR_MAXMB(txMb + CAN_TX_WINDOW - 1u);

    // RRS stores remote frames, EACEN matches IDE and RTR, RFFN zero gives eight filters.
    can->CTRL2 = CAN_CTRL2_RRS_MASK | CAN_CTRL2_EACEN_MASK | CAN_CTRL2_RFFN(0);

    // Confirm freeze again before the bit timing goes in.
    if (!canSetFreeze(can, true)) {
        return false;
    }

    // LBUF sends the lowest numbered buffer first. CLKSRC zero is the 80 MHz root
    // on the 11xx and absent on the 10xx.
    can->CTRL1 = CAN_CTRL1_PRESDIV(presdiv)
               | CAN_CTRL1_PROPSEG(CAN_PROPSEG)
               | CAN_CTRL1_PSEG1(CAN_PSEG1)
               | CAN_CTRL1_PSEG2(CAN_PSEG2)
               | CAN_CTRL1_RJW(CAN_RJW)
               | CAN_CTRL1_LBUF_MASK;

    // Accept everything: all mask bits zero.
    can->RXFGMASK = 0;

    for (uint32_t i = 0; i < CAN_FIFO_FILTER_COUNT; i++) {
        can->RXIMR[i] = 0;
    }

    // ERR005829: reserve the first transmit buffer. Code 0x3 is used instead of
    // the inactive code so ESR2[LPTM] never picks it; ESR2 is not read here.
    if (cap->txErrataMb) {
        can->MB[CAN_FIRST_FREE_MB].CS = CAN_CS_CODE(CAN_CODE_TX_ERRATA);
    }

    for (uint32_t mb = txMb; mb < txMb + CAN_TX_WINDOW; mb++) {
        can->MB[mb].CS = CAN_CS_CODE(CAN_CODE_TX_INACTIVE);
        can->MB[mb].ID = 0;
        can->MB[mb].WORD0 = 0;
        can->MB[mb].WORD1 = 0;
    }

    // Clear stale flags before arming the interrupts.
    canClearFlags(can, 0xFFFFFFFFu);
    canTxArm(device);

    return canSetFreeze(can, false);
}

// The mailbox must be inactive while its identifier and payload are written.
static FAST_IRQ_HANDLER void canTxLoadMb(CAN_Type *can, uint32_t mb, const canTxFrame_t *frame)
{
    can->MB[mb].CS = CAN_CS_CODE(CAN_CODE_TX_INACTIVE);

    can->MB[mb].ID = frame->isExtended
        ? (frame->id & CAN_ID_EXTENDED_MASK)
        : CAN_ID_STD(frame->id & CAN_ID_STANDARD_MASK);

    can->MB[mb].WORD0 = canDataToWord(frame->data, frame->length, 0);
    can->MB[mb].WORD1 = canDataToWord(frame->data, frame->length, 4);

    uint32_t cs = CAN_CS_CODE(CAN_CODE_TX_DATA) | CAN_CS_DLC(frame->length);

    if (frame->isExtended) {
        // SRR must be recessive in an extended frame.
        cs |= CAN_CS_IDE_MASK | CAN_CS_SRR_MASK;
    }

    can->MB[mb].CS = cs;
}

// Frames go only into the slot above every filled one and the window rewinds
// only when empty, which keeps submission order. Sole consumer of txTail; runs
// from the interrupt or from task context with this device's vectors masked.
static FAST_IRQ_HANDLER void canTxKick(canDevice_e device)
{
    canDevice_t *const pDev = &canDevice[device];
    CAN_Type *const can = canRegs(device);
    const uint32_t txMb = canTxMailbox(device);

    while (canTx[device].loaded < CAN_TX_WINDOW && pDev->txTail != pDev->txHead) {
        // Keep the frame reads after the txHead test.
        __asm volatile ("" ::: "memory");

        const uint8_t slot = canTx[device].loaded;

        canTxLoadMb(can, txMb + slot, &pDev->txRing[pDev->txTail]);

        // ERR005829 step 8: write the reserved buffer's code twice after every load.
        if (canCapability[device].txErrataMb) {
            can->MB[CAN_FIRST_FREE_MB].CS = CAN_CS_CODE(CAN_CODE_TX_ERRATA);
            can->MB[CAN_FIRST_FREE_MB].CS = CAN_CS_CODE(CAN_CODE_TX_ERRATA);
        }

        pDev->txTail = (pDev->txTail + 1u) & CAN_TX_RING_MASK;
        canTx[device].busy |= 1u << slot;
        canTx[device].loaded = slot + 1u;
    }

    canTxArm(device);
}

// Abort the transmit window lowest mailbox first (a handshake with MCR[AEN] set);
// false if a mailbox does not answer.
static bool canTxAbort(canDevice_e device)
{
    CAN_Type *const can = canRegs(device);
    const uint32_t txMb = canTxMailbox(device);
    bool emptied = true;

    for (uint8_t slot = 0; slot < canTx[device].loaded; slot++) {
        if (!(canTx[device].busy & (1u << slot))) {
            continue;
        }

        const uint32_t flag = 1u << (txMb + slot);

        can->MB[txMb + slot].CS = CAN_CS_CODE(CAN_CODE_TX_ABORT);

        if (!canWaitFlag(can, flag)) {
            emptied = false;
            break;
        }

        canClearFlags(can, flag);
        can->MB[txMb + slot].CS = CAN_CS_CODE(CAN_CODE_TX_INACTIVE);
        canTx[device].busy &= (uint8_t)~(1u << slot);
    }

    if (emptied) {
        canTx[device].loaded = 0;
    }

    canTxArm(device);

    return emptied;
}

// A full ring with no completions means nothing acknowledges; empty the window
// and drop the backlog.
static void canTxRecoverStall(canDevice_e device, uint8_t head)
{
    canDevice_t *const pDev = &canDevice[device];
    const timeUs_t now = micros();

    if (pDev->txCompletions != pDev->txCompletionsSeen) {
        pDev->txCompletionsSeen = pDev->txCompletions;
        pDev->txStallSinceUs = now;

        return;
    }

    if (cmpTimeUs(now, pDev->txStallSinceUs) < (timeDelta_t)CAN_TX_STALL_TIMEOUT_US) {
        return;
    }

    canIrqMask(pDev);

    // Rewind only on a clean sweep, or frames could be reordered.
    if (canTxAbort(device)) {
        pDev->txTail = head;
        pDev->txStallRecoveries++;
    }

    pDev->txStallSinceUs = now;

    canIrqUnmask(pDev);
}

// Queue a classic CAN frame. False on invalid parameters, an uninitialised
// device or a full ring. Task context only (single producer).
bool canTransmit(canDevice_e device, uint32_t identifier, bool isExtended,
                 const uint8_t *data, uint8_t length)
{
    if (device < 0 || device >= CANDEV_COUNT) {
        return false;
    }

    canDevice_t *const pDev = &canDevice[device];

    if (!pDev->initialized || length > CAN_CLASSIC_MAX_DLC) {
        return false;
    }

    if (length > 0 && data == NULL) {
        return false;
    }

    const uint8_t head = pDev->txHead;
    const uint8_t next = (head + 1u) & CAN_TX_RING_MASK;

    if (next == pDev->txTail) {
        canTxRecoverStall(device, head);
        pDev->txRingOverflows++;

        return false;
    }

    canTxFrame_t *const frame = &pDev->txRing[head];

    frame->id = identifier;
    frame->isExtended = isExtended;
    frame->length = length;

    for (uint8_t i = 0; i < length; i++) {
        frame->data[i] = data[i];
    }

    // Fill the slot before advancing head.
    __asm volatile ("" ::: "memory");
    pDev->txHead = next;

    canIrqMask(pDev);
    canTxKick(device);
    canIrqUnmask(pDev);

    return true;
}

// The FIFO shows one frame at a time in buffer 0; clearing its flag pops it.
static FAST_IRQ_HANDLER void canDrainRxFifo(canDevice_e device)
{
    canDevice_t *const pDev = &canDevice[device];
    CAN_Type *const can = canRegs(device);

    while (can->IFLAG1 & CAN_IFLAG_FIFO_AVAILABLE) {
        const uint32_t cs = can->MB[CAN_FIFO_OUTPUT_MB].CS;
        const uint32_t id = can->MB[CAN_FIFO_OUTPUT_MB].ID;
        const uint32_t word0 = can->MB[CAN_FIFO_OUTPUT_MB].WORD0;
        const uint32_t word1 = can->MB[CAN_FIFO_OUTPUT_MB].WORD1;

        const bool isExtended = (cs & CAN_CS_IDE_MASK) != 0;
        const uint32_t identifier = isExtended
            ? (id & CAN_ID_EXTENDED_MASK)
            : ((id & CAN_ID_STD_MASK) >> CAN_ID_STD_SHIFT);

        uint8_t length = (cs & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT;

        if (length > CAN_CLASSIC_MAX_DLC) {
            length = CAN_CLASSIC_MAX_DLC;
        }

        uint8_t payload[CAN_CLASSIC_MAX_DLC];

        canWordToData(word0, payload, length, 0);
        canWordToData(word1, payload, length, 4);

        canClearFlags(can, CAN_IFLAG_FIFO_AVAILABLE);

        if (pDev->rxCallback) {
            pDev->rxCallback(identifier, isExtended, payload, length);
        }
    }

    // Reading TIMER releases the mailbox lock taken by reading CS.
    (void)can->TIMER;
}

// Handler for both vectors; no error source is unmasked, so the 11xx error vector stays quiet.
FAST_IRQ_HANDLER void canIrqHandler(canDevice_e device)
{
    canDevice_t *const pDev = &canDevice[device];

    if (!pDev->reg) {
        return;
    }

    CAN_Type *const can = canRegs(device);
    const uint32_t flags = can->IFLAG1;

    if (flags & CAN_IFLAG_FIFO_OVERFLOW) {
        canClearFlags(can, CAN_IFLAG_FIFO_OVERFLOW);
        pDev->rxOverruns++;
    }

    if (flags & CAN_IFLAG_FIFO_AVAILABLE) {
        canDrainRxFifo(device);
    }

    // Clear every finished window mailbox, not only the unmasked topmost one.
    const uint32_t done = flags & canTxWindowFlags(device);

    if (done) {
        canClearFlags(can, done);
        canTx[device].busy &= (uint8_t)~(done >> canTxMailbox(device));
        pDev->txCompletions += (uint32_t)__builtin_popcount(done);

        if (canTx[device].busy == 0) {
            canTx[device].loaded = 0;
        }

        canTxKick(device);
    }
}

static void canEnableInterrupt(uint8_t irq)
{
    NVIC_SetPriority((IRQn_Type)irq, NVIC_PRIORITY_LEVEL(NVIC_PRIO_CAN));

    NVIC_ClearPendingIRQ((IRQn_Type)irq);
    NVIC_EnableIRQ((IRQn_Type)irq);
}

static void canClaimPins(canDevice_e device, const nxpPinMux_t *txMux, const nxpPinMux_t *rxMux)
{
    canDevice_t *const pDev = &canDevice[device];

    IOInit(IOGetByTag(pDev->tx), OWNER_CAN_TX, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(pDev->rx), OWNER_CAN_RX, RESOURCE_INDEX(device));

    // Pull receive up so it idles recessive with no transceiver.
    nxpIomuxConfigPin(txMux, IOCFG_AF_PP);
    nxpIomuxConfigPin(rxMux, IOCFG_AF_PP_UP);

    // Transceiver silent pin low is normal operation.
    if (pDev->silent) {
        const IO_t silentIO = IOGetByTag(pDev->silent);

        IOInit(silentIO, OWNER_CAN_SILENT, RESOURCE_INDEX(device));
        IOConfigGPIO(silentIO, IOCFG_OUT_PP);
        IOLo(silentIO);
    }
}

// Only the DroneCAN device is brought up; without DroneCAN every wired one is.
static bool canDeviceSelected(canDevice_e device)
{
#if ENABLE_DRONECAN
    return device == (canDevice_e)CAN_CFG_TO_DEV(dronecanConfig()->device);
#else
    UNUSED(device);

    return true;
#endif
}

// Put an unused wired controller in disable mode (MCR[MDIS]); the clock gate must
// be open for the write.
static void canQuiesce(canDevice_e device)
{
    CAN_Type *const can = canRegs(device);

    if (!nxpClockGateEnable(canCapability[device].clk)) {
        return;
    }

    can->MCR |= CAN_MCR_MDIS_MASK;

    (void)canWaitMcr(can, CAN_MCR_LPMACK_MASK, true);
}

void canInitDevice(canDevice_e device, uint32_t bitrate)
{
    if (device < 0 || device >= CANDEV_COUNT) {
        return;
    }

    canDevice_t *const pDev = &canDevice[device];

    if (!pDev->reg || !pDev->tx || !pDev->rx) {
        return;
    }

    if (!canDeviceSelected(device)) {
        pDev->initialized = false;
        canQuiesce(device);

        return;
    }

    // Check the configuration before touching hardware.
    const nxpPinMux_t *const txMux = canPadMux(device, false, pDev->tx);
    const nxpPinMux_t *const rxMux = canPadMux(device, true, pDev->rx);

    if (!txMux || !rxMux) {
        return;
    }

    uint32_t presdiv;

    if (!canPrescaler(device, bitrate, &presdiv)) {
        return;
    }

    if (!nxpClockGateEnable(canCapability[device].clk)) {
        return;
    }

    canClaimPins(device, txMux, rxMux);

    pDev->txHead = 0;
    pDev->txTail = 0;
    pDev->txRingOverflows = 0;
    pDev->txCompletions = 0;
    pDev->txCompletionsSeen = 0;
    pDev->txStallSinceUs = micros();
    pDev->txStallRecoveries = 0;
    pDev->rxOverruns = 0;
    canTx[device].busy = 0;
    canTx[device].loaded = 0;

    if (!canBringUp(device, presdiv)) {
        return;
    }

    canEnableInterrupt(pDev->irq0);

    if (pDev->irq1 != pDev->irq0) {
        canEnableInterrupt(pDev->irq1);
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

// Override the weak handlers in the family startup file.
FAST_IRQ_HANDLER void CAN1_IRQHandler(void)
{
    canIrqHandler(CANDEV_1);
}

FAST_IRQ_HANDLER void CAN2_IRQHandler(void)
{
    canIrqHandler(CANDEV_2);
}

FAST_IRQ_HANDLER void CAN3_IRQHandler(void)
{
    canIrqHandler(CANDEV_3);
}

#if defined(IMXRT11XX)

FAST_IRQ_HANDLER void CAN1_ERROR_IRQHandler(void)
{
    canIrqHandler(CANDEV_1);
}

FAST_IRQ_HANDLER void CAN2_ERROR_IRQHandler(void)
{
    canIrqHandler(CANDEV_2);
}

FAST_IRQ_HANDLER void CAN3_ERROR_IRQHandler(void)
{
    canIrqHandler(CANDEV_3);
}

#endif // IMXRT11XX

#endif // ENABLE_CAN
