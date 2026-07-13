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

// X32M7 StdPeriph names the FDCAN register block FDCAN_Module; STM32 HAL
// calls it FDCAN_GlobalTypeDef.  Both map the same Bosch M_CAN layout.
#if defined(X32M7) && !defined(FDCAN_GlobalTypeDef)
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
// SRAM5: Bank1 (0x30050000) for FDCAN1-4, Bank2 (0x30054000) for FDCAN5-8.
//-----------------------------------------------------------------------------

#define CAN_X32_RF0_NBR             (3U)
#define CAN_X32_TFQ_NBR             (3U)
#define CAN_X32_ELEMENT_WORDS       (4U)
#define CAN_X32_PER_INSTANCE_WORDS  (64U)
#define CAN_X32_PER_BANK_INSTANCES  (4U)
#define CAN_X32_RF0_WORD_OFFSET     (0U)
#define CAN_X32_TFQ_WORD_OFFSET     (CAN_X32_RF0_WORD_OFFSET + \
                                     CAN_X32_RF0_NBR * CAN_X32_ELEMENT_WORDS)

#define CAN_X32_BANK1_BASE          (0x30050000UL)
#define CAN_X32_BANK2_BASE          (0x30054000UL)

#define CAN_X32_ESC_CODE_8B         (0U)

#define CAN_RX_ELEMENT_BYTES        (CAN_X32_ELEMENT_WORDS * 4U)
#define CAN_TX_ELEMENT_BYTES        (CAN_X32_ELEMENT_WORDS * 4U)

#define FDCAN_SIDFC_FLSSA_Pos       (2U)
#define FDCAN_SIDFC_FLSSA_Msk       FDCAN_SIDFC_FLSSA
#define FDCAN_XIDFC_FLESA_Pos       (2U)
#define FDCAN_XIDFC_FLESA_Msk       FDCAN_XIDFC_FLESA
#define FDCAN_RXF0C_F0SA_Pos        (2U)
#define FDCAN_RXF0C_F0SA_Msk        FDCAN_RXF0C_F0SA
#define FDCAN_RXF0C_F0S_Pos         (16U)
#define FDCAN_RXF0C_F0S_Msk         FDCAN_RXF0C_F0S
#define FDCAN_TXBC_TBSA_Pos         (2U)
#define FDCAN_TXBC_TBSA_Msk         FDCAN_TXBC_TBSA
#define FDCAN_TXBC_TFQS_Pos         (24U)
#define FDCAN_TXBC_TFQS_Msk         FDCAN_TXBC_TFQS
#define FDCAN_RXESC_F0DS_Pos        (0U)
#define FDCAN_RXESC_F0DS_Msk        FDCAN_RXESC_F0DS
#define FDCAN_RXESC_F1DS_Pos        (4U)
#define FDCAN_RXESC_F1DS_Msk        FDCAN_RXESC_F1DS
#define FDCAN_RXESC_RBDS_Pos        (8U)
#define FDCAN_RXESC_RBDS_Msk        FDCAN_RXESC_RBDS
#define FDCAN_TXESC_TBDS_Pos        (0U)
#define FDCAN_TXESC_TBDS_Msk        FDCAN_TXESC_TBDS

static uint32_t canMessageRamBase(canDevice_e device)
{
    if ((uint32_t)device < CAN_X32_PER_BANK_INSTANCES) {
        return CAN_X32_BANK1_BASE
             + (uint32_t)device * CAN_X32_PER_INSTANCE_WORDS * 4U;
    } else {
        return CAN_X32_BANK2_BASE
             + ((uint32_t)device - CAN_X32_PER_BANK_INSTANCES)
               * CAN_X32_PER_INSTANCE_WORDS * 4U;
    }
}

static uint32_t canRxElementAddress(canDevice_e device, uint32_t index)
{
    return canMessageRamBase(device)
         + CAN_X32_RF0_WORD_OFFSET * 4U
         + index * CAN_RX_ELEMENT_BYTES;
}

static uint32_t canTxElementAddress(canDevice_e device, uint32_t index)
{
    return canMessageRamBase(device)
         + CAN_X32_TFQ_WORD_OFFSET * 4U
         + index * CAN_TX_ELEMENT_BYTES;
}

//-----------------------------------------------------------------------------
// Bit timing
//
// canNominalBitTiming() probes the kernel clock at init time and searches
// for a (prescaler, tseg1, tseg2) triple that hits the requested bitrate
// with a sample point near 75 %.  Returns false if no valid setting exists.
//-----------------------------------------------------------------------------

#define CAN_NBTP_NSJW_POS       (25U)
#define CAN_NBTP_NBRP_POS       (16U)
#define CAN_NBTP_NTSEG1_POS     (8U)
#define CAN_NBTP_NTSEG2_POS     (0U)

#define CAN_NBRP_MAX            (512U)
#define CAN_NTSEG1_MAX          (256U)
#define CAN_NTSEG2_MAX          (128U)
#define CAN_TOTAL_TQ_MIN        (8U)
#define CAN_TOTAL_TQ_MAX        (25U)

#define CAN_SAMPLE_POINT_PERMILLE (750U)

static uint32_t canPackNominalBitTiming(uint32_t nbrp, uint32_t ntseg1,
                                        uint32_t ntseg2, uint32_t nsjw)
{
    return ((nsjw - 1U)   << CAN_NBTP_NSJW_POS)
         | ((nbrp - 1U)   << CAN_NBTP_NBRP_POS)
         | ((ntseg1 - 1U) << CAN_NBTP_NTSEG1_POS)
         | ((ntseg2 - 1U) << CAN_NBTP_NTSEG2_POS);
}

static uint32_t canGetKernelClockHz(void)
{
    uint32_t kersel = (RCC->APB1SEL1 & RCC_APB1SEL1_FDCAN1KERSEL) >> 16;
    uint32_t srcFreq;

    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);

    if (kersel == 0) {
        srcFreq = clocks.SysBusDivClkFreq;
    } else {
        srcFreq = clocks.SysBusDivClkFreq;
    }

    uint32_t divRaw = (RCC->APB1DIV1 & RCC_APB1DIV1_APB1FDCANDIV) >> 4;
    static const uint32_t divTable[8] = {1, 1, 1, 1, 2, 4, 8, 16};
    uint32_t divisor = divTable[divRaw & 0x7U];

    return srcFreq / divisor;
}
static bool canNominalBitTiming(uint32_t bitrate, uint32_t *nbtp)
{
    if (bitrate == 0U || nbtp == NULL) {
        return false;
    }

    const uint32_t kernelHz = canGetKernelClockHz();

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

        const uint32_t nsjw = (ntseg2 < 4U) ? ntseg2 : 4U;
        *nbtp = canPackNominalBitTiming(nbrp, ntseg1, ntseg2, nsjw);
        return true;
    }

    return false;
}

//-----------------------------------------------------------------------------
// Tx / Rx element header encoding
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
// Low-level helpers
//-----------------------------------------------------------------------------

static FDCAN_GlobalTypeDef *canRegs(canDevice_e device)
{
    return (FDCAN_GlobalTypeDef *)canDevice[device].reg;
}

#define CAN_INIT_SPIN_MAX       (100000U)

static bool canEnterConfigMode(FDCAN_GlobalTypeDef *regs)
{
    regs->CCCR |= FDCAN_CCCR_INIT;

    uint32_t timeout = CAN_INIT_SPIN_MAX;
    while (!(regs->CCCR & FDCAN_CCCR_INIT) && timeout--) {
    }
    if (!(regs->CCCR & FDCAN_CCCR_INIT)) {
        return false;
    }

    regs->CCCR |= FDCAN_CCCR_CCE;
    return true;
}

static bool canExitConfigMode(FDCAN_GlobalTypeDef *regs)
{
    regs->CCCR &= ~FDCAN_CCCR_CCE;
    regs->CCCR &= ~FDCAN_CCCR_INIT;

    uint32_t timeout = CAN_INIT_SPIN_MAX;
    while ((regs->CCCR & FDCAN_CCCR_INIT) && timeout--) {
    }

    return (regs->CCCR & FDCAN_CCCR_INIT) == 0U;
}

static void canClearMessageRam(canDevice_e device)
{
    uint32_t base = canMessageRamBase(device);
    uint32_t bytes = CAN_X32_PER_INSTANCE_WORDS * 4U;

    for (uint32_t addr = base; addr < base + bytes; addr += 4) {
        *(volatile uint32_t *)addr = 0;
    }
}

static void canEnableInterrupt(canDevice_e device, uint8_t irq)
{
    UNUSED(device);
    NVIC_SetPriority(irq, NVIC_PRIO_CAN);
    NVIC_EnableIRQ(irq);
}

static void canConfigureMessageRam(canDevice_e device, FDCAN_GlobalTypeDef *regs)
{
    const uint32_t bankIndex = ((uint32_t)device < CAN_X32_PER_BANK_INSTANCES)
        ? (uint32_t)device
        : ((uint32_t)device - CAN_X32_PER_BANK_INSTANCES);
    const uint32_t instanceWords = bankIndex * CAN_X32_PER_INSTANCE_WORDS;
    const uint32_t rf0SA = instanceWords + CAN_X32_RF0_WORD_OFFSET;
    const uint32_t tfqSA = instanceWords + CAN_X32_TFQ_WORD_OFFSET;
    const uint32_t escCode  = CAN_X32_ESC_CODE_8B;
    const uint32_t rf0Nbr   = CAN_X32_RF0_NBR;
    const uint32_t tfqNbr   = CAN_X32_TFQ_NBR;

    regs->SIDFC = (instanceWords << FDCAN_SIDFC_FLSSA_Pos) & FDCAN_SIDFC_FLSSA_Msk;
    regs->XIDFC = (instanceWords << FDCAN_XIDFC_FLESA_Pos) & FDCAN_XIDFC_FLESA_Msk;

    regs->RXF0C = ((rf0SA << FDCAN_RXF0C_F0SA_Pos) & FDCAN_RXF0C_F0SA_Msk)
                | ((rf0Nbr << FDCAN_RXF0C_F0S_Pos) & FDCAN_RXF0C_F0S_Msk);
    regs->RXF1C = 0;
    regs->RXBC  = 0;

    regs->RXESC = (escCode << FDCAN_RXESC_F0DS_Pos)
                | (escCode << FDCAN_RXESC_F1DS_Pos)
                | (escCode << FDCAN_RXESC_RBDS_Pos);

    regs->TXBC = ((tfqSA << FDCAN_TXBC_TBSA_Pos) & FDCAN_TXBC_TBSA_Msk)
               | ((tfqNbr << FDCAN_TXBC_TFQS_Pos) & FDCAN_TXBC_TFQS_Msk);
    regs->TXEFC = 0;
    regs->TXESC = (escCode << FDCAN_TXESC_TBDS_Pos);
}

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

    RCC_ClockCmd(pDev->rcc, ENABLE);

    IO_t txIO = IOGetByTag(pDev->tx);
    IO_t rxIO = IOGetByTag(pDev->rx);

    IOInit(txIO, OWNER_CAN_TX, RESOURCE_INDEX(device));
    IOInit(rxIO, OWNER_CAN_RX, RESOURCE_INDEX(device));

    IOConfigGPIOAF(txIO, IOCFG_AF_PP, pDev->txAF);
    IOConfigGPIOAF(rxIO, IOCFG_AF_PP_UP, pDev->rxAF);

    if (!canEnterConfigMode(regs)) {
        return;
    }

    if ((uint32_t)device >= CAN_X32_PER_BANK_INSTANCES) {
        regs->TTSS |= FDCAN_TTSS_RAMSEL;
    } else {
        regs->TTSS &= ~FDCAN_TTSS_RAMSEL;
    }

    regs->CCCR &= ~(FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE);

    uint32_t nbtp;
    if (!canNominalBitTiming(bitrate, &nbtp)) {
        return;
    }
    regs->NBTP = nbtp;

    canClearMessageRam(device);

    regs->GFC &= ~(FDCAN_GFC_ANFS | FDCAN_GFC_ANFE);
    canConfigureMessageRam(device, regs);

    // Pre-fill Tx FIFO slots so they don't transmit as ID=0x000 phantom frames.
    for (uint32_t i = 0; i < 3U; i++) {
        volatile uint32_t *txSlot =
            (volatile uint32_t *)canTxElementAddress(device, i);
        txSlot[0] = CAN_TX_WORD0_XTD | 0x1801552AUL;
        txSlot[1] = 0;
        txSlot[2] = 0;
        txSlot[3] = 0;
    }

    SCB_CleanDCache_by_Addr(
        (uint32_t *)canTxElementAddress(device, 0),
        3U * CAN_TX_ELEMENT_BYTES);

    regs->IE  |= FDCAN_IE_RF0NE | FDCAN_IE_RF0LE;
    regs->ILE |= FDCAN_ILE_EINT0;

    canEnableInterrupt(device, pDev->irq0);

    if (!canExitConfigMode(regs)) {
        return;
    }

    regs->TXBCR = 0xFFFFFFFFU;

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

    if (length > 0 && data == NULL) {
        return false;
    }

    FDCAN_GlobalTypeDef *regs = canRegs(device);

    uint32_t txfqs = regs->TXFQS;
    if (txfqs & FDCAN_TXFQS_TFQF) {
        return false;
    }

    uint32_t putIndex = (txfqs & FDCAN_TXFQS_TFQPI) >> 16;
    volatile uint32_t *slot = (volatile uint32_t *)canTxElementAddress(device, putIndex);

    if (isExtended) {
        slot[0] = CAN_TX_WORD0_XTD | (identifier & 0x1FFFFFFFUL);
    } else {
        slot[0] = (identifier & 0x7FFUL) << 18;
    }

    slot[1] = (uint32_t)length << CAN_TX_WORD1_DLC_POS;

    uint32_t words[2] = { 0, 0 };
    for (uint8_t i = 0; i < length; i++) {
        words[i >> 2] |= ((uint32_t)data[i]) << ((i & 3U) * 8U);
    }
    slot[2] = words[0];
    slot[3] = words[1];

    __DSB();
    SCB_CleanDCache_by_Addr((uint32_t *)slot, CAN_TX_ELEMENT_BYTES);

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

    while (regs->RXF0S & FDCAN_RXF0S_F0FL) {
        uint32_t getIndex = (regs->RXF0S & FDCAN_RXF0S_F0GI) >> 8;

        volatile const uint32_t *slot =
            (volatile const uint32_t *)canRxElementAddress(device, getIndex);

        SCB_InvalidateDCache_by_Addr((uint32_t *)slot, CAN_RX_ELEMENT_BYTES);

        uint32_t w0 = slot[0];
        uint32_t w1 = slot[1];

        bool isExtended = (w0 & CAN_RX_WORD0_XTD) != 0;
        uint32_t identifier = isExtended
            ? (w0 & CAN_RX_WORD0_EXTID_MASK)
            : ((w0 & CAN_RX_WORD0_STDID_MASK) >> CAN_RX_WORD0_STDID_POS);

        uint8_t dlc = (w1 & CAN_RX_WORD1_DLC_MASK) >> CAN_RX_WORD1_DLC_POS;
        if (dlc > CAN_CLASSIC_MAX_DLC) {
            dlc = CAN_CLASSIC_MAX_DLC;
        }

        uint8_t payload[CAN_CLASSIC_MAX_DLC];
        uint32_t w2 = slot[2];
        uint32_t w3 = slot[3];
        for (uint8_t i = 0; i < dlc; i++) {
            uint32_t word = (i < 4) ? w2 : w3;
            payload[i] = (uint8_t)(word >> ((i & 3U) * 8U));
        }

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
        regs->IR = FDCAN_IR_RF0N;
        canDispatchRx(device);
    }

    if (ir & FDCAN_IR_RF0L) {
        regs->IR = FDCAN_IR_RF0L;
        canDevice[device].rxOverruns++;
    }
}

void FDCAN1_INT0_IRQHandler(void) { canIrqHandler(CANDEV_1); }
void FDCAN2_INT0_IRQHandler(void) { canIrqHandler(CANDEV_2); }
void FDCAN3_INT0_IRQHandler(void) { canIrqHandler(CANDEV_3); }
void FDCAN4_INT0_IRQHandler(void) { canIrqHandler(CANDEV_4); }
void FDCAN5_INT0_IRQHandler(void) { canIrqHandler(CANDEV_5); }
void FDCAN6_INT0_IRQHandler(void) { canIrqHandler(CANDEV_6); }
void FDCAN7_INT0_IRQHandler(void) { canIrqHandler(CANDEV_7); }
void FDCAN8_INT0_IRQHandler(void) { canIrqHandler(CANDEV_8); }

#endif // ENABLE_CAN
