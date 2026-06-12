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

/*
 * Authors:
 * jflyper - Refactoring, cleanup and made pin-configurable
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "build/debug.h"

#ifdef USE_UART

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/inverter.h"
#include "drivers/dma.h"
#include "platform/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

static inline bool uartCanTx(const uartPort_t *uartPort)
{
    const uartDevice_t *uartDevice = container_of(uartPort, uartDevice_t, port);
    return (uartPort->port.mode & MODE_TX) && uartDevice->tx.pin;
}

// Disable every UART IT-enable bit before HAL_UART_DeInit / re-init. HAL clears
// CR1/CR3 during DeInit, but the window between entering DeInit and that clear
// is interruptable; a pending IRQ that fires while CR1.UE is being torn down
// can re-enter uartIrqHandler with stale ISR flags. Pre-clearing the IT-enable
// bits combined with the per-branch CR1/CR3 IT-enable gating in uartIrqHandler
// closes that window. Caller must hold the NVIC line for this UART disabled.
static inline void uartDisableIrqSources(uartPort_t *uartPort)
{
    CLEAR_BIT(uartPort->USARTx->CR1,
              USART_CR1_PEIE | USART_CR1_RXNEIE | USART_CR1_IDLEIE |
              USART_CR1_TXEIE | USART_CR1_TCIE);
    CLEAR_BIT(uartPort->USARTx->CR3, USART_CR3_EIE);
}

static void usartConfigurePinInversion(uartPort_t *uartPort)
{
    bool inverted = uartPort->port.options & SERIAL_INVERTED;

    if (inverted)
    {
        if (uartPort->port.mode & MODE_RX)
        {
            uartPort->Handle.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_RXINVERT_INIT;
            uartPort->Handle.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
        }
        if (uartPort->port.mode & MODE_TX)
        {
            uartPort->Handle.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_TXINVERT_INIT;
            uartPort->Handle.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
        }
    }
}

#if UART_TRAIT_PINSWAP
static void uartConfigurePinSwap(uartPort_t *uartPort)
{
    uartDevice_t *uartDevice = container_of(uartPort, uartDevice_t, port);
    if (uartDevice->pinSwap) {
        uartDevice->port.Handle.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_SWAP_INIT;
        uartDevice->port.Handle.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
    }
}
#endif

// XXX uartReconfigure does not handle resource management properly.

void uartReconfigure(uartPort_t *uartPort)
{
    const uartDevice_t *uartDevice = container_of(uartPort, uartDevice_t, port);
    const IRQn_Type irqn = (IRQn_Type)uartDevice->hardware->irqn;
    const bool irqWasEnabled = NVIC_GetEnableIRQ(irqn) != 0;
    const bool canTx = uartCanTx(uartPort);

    // Quiesce the UART IRQ line before tearing down the peripheral. HAL_UART_DeInit
    // clears CR1.UE early but leaves the IT-enable bits set for a few instructions;
    // a UART IRQ taken during that window with stale TXE/TC ISR flags is what
    // causes the reconfigure-time freeze on H7 + real GPS (see #15218 follow-up).
    HAL_NVIC_DisableIRQ(irqn);
    uartDisableIrqSources(uartPort);
    HAL_NVIC_ClearPendingIRQ(irqn);
    __DSB();
    __ISB();

    HAL_UART_DeInit(&uartPort->Handle);
    uartPort->Handle.Init.BaudRate = uartPort->port.baudRate;
    // according to the stm32 documentation wordlen has to be 9 for parity bits
    // this does not seem to matter for rx but will give bad data on tx!
    uartPort->Handle.Init.WordLength = (uartPort->port.options & SERIAL_PARITY_EVEN) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
    uartPort->Handle.Init.StopBits = (uartPort->port.options & SERIAL_STOPBITS_2) ? USART_STOPBITS_2 : USART_STOPBITS_1;
    uartPort->Handle.Init.Parity = (uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_PARITY_EVEN : USART_PARITY_NONE;
    uartPort->Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uartPort->Handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uartPort->Handle.Init.Mode = 0;
#if defined(STM32G4) || defined(STM32H7)
    if (uartPort->Handle.Instance == LPUART1) {
        uartPort->Handle.Init.ClockPrescaler = UART_PRESCALER_DIV8;
    }
#endif

    if (uartPort->port.mode & MODE_RX)
        uartPort->Handle.Init.Mode |= UART_MODE_RX;
    if (canTx)
        uartPort->Handle.Init.Mode |= UART_MODE_TX;

    usartConfigurePinInversion(uartPort);
#if !(defined(STM32F1) || defined(STM32F4))
    uartConfigurePinSwap(uartPort);
#endif

#ifdef TARGET_USART_CONFIG
    void usartTargetConfigure(uartPort_t *);
    usartTargetConfigure(uartPort);
#endif

    if (uartPort->port.options & SERIAL_BIDIR) {
        HAL_HalfDuplex_Init(&uartPort->Handle);
    } else {
        HAL_UART_Init(&uartPort->Handle);
    }

    // Receive DMA or IRQ
    if (uartPort->port.mode & MODE_RX)
    {
#ifdef USE_DMA
        if (uartPort->rxDMAResource)
        {
            uartPort->rxDMAHandle.Instance = (DMA_ARCH_TYPE *)uartPort->rxDMAResource;
#if !(defined(STM32H7) || defined(STM32G4))
            uartPort->rxDMAHandle.Init.Channel = uartPort->rxDMAChannel;
#else
            uartPort->txDMAHandle.Init.Request = uartPort->rxDMAChannel;
#endif
            uartPort->rxDMAHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
            uartPort->rxDMAHandle.Init.PeriphInc = DMA_PINC_DISABLE;
            uartPort->rxDMAHandle.Init.MemInc = DMA_MINC_ENABLE;
            uartPort->rxDMAHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            uartPort->rxDMAHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            uartPort->rxDMAHandle.Init.Mode = DMA_CIRCULAR;
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
            uartPort->rxDMAHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            uartPort->rxDMAHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
            uartPort->rxDMAHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
            uartPort->rxDMAHandle.Init.MemBurst = DMA_MBURST_SINGLE;
#endif
            uartPort->rxDMAHandle.Init.Priority = DMA_PRIORITY_MEDIUM;

            HAL_DMA_DeInit(&uartPort->rxDMAHandle);
            HAL_DMA_Init(&uartPort->rxDMAHandle);
            /* Associate the initialized DMA handle to the UART handle */
            __HAL_LINKDMA(&uartPort->Handle, hdmarx, uartPort->rxDMAHandle);

            HAL_UART_Receive_DMA(&uartPort->Handle, (uint8_t*)uartPort->port.rxBuffer, uartPort->port.rxBufferSize);

            uartPort->rxDMAPos = __HAL_DMA_GET_COUNTER(&uartPort->rxDMAHandle);
        } else
#endif
        {
            /* Enable the UART Parity Error Interrupt */
            SET_BIT(uartPort->USARTx->CR1, USART_CR1_PEIE);

            /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
            SET_BIT(uartPort->USARTx->CR3, USART_CR3_EIE);

            /* Enable the UART Data Register not empty Interrupt */
            SET_BIT(uartPort->USARTx->CR1, USART_CR1_RXNEIE);

            /* Enable Idle Line detection */
            SET_BIT(uartPort->USARTx->CR1, USART_CR1_IDLEIE);
        }
    }

    // Transmit DMA or IRQ
    if (canTx) {
#ifdef USE_DMA
        if (uartPort->txDMAResource) {
            uartPort->txDMAHandle.Instance = (DMA_ARCH_TYPE *)uartPort->txDMAResource;
#if !(defined(STM32H7) || defined(STM32G4))
            uartPort->txDMAHandle.Init.Channel = uartPort->txDMAChannel;
#else
            uartPort->txDMAHandle.Init.Request = uartPort->txDMAChannel;
#endif
            uartPort->txDMAHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
            uartPort->txDMAHandle.Init.PeriphInc = DMA_PINC_DISABLE;
            uartPort->txDMAHandle.Init.MemInc = DMA_MINC_ENABLE;
            uartPort->txDMAHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            uartPort->txDMAHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            uartPort->txDMAHandle.Init.Mode = DMA_NORMAL;
#if !defined(STM32G4)
            // G4's DMA is channel based, and does not have FIFO
            uartPort->txDMAHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            uartPort->txDMAHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
            uartPort->txDMAHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
            uartPort->txDMAHandle.Init.MemBurst = DMA_MBURST_SINGLE;
#endif
            uartPort->txDMAHandle.Init.Priority = DMA_PRIORITY_MEDIUM;

            HAL_DMA_DeInit(&uartPort->txDMAHandle);
            HAL_StatusTypeDef status = HAL_DMA_Init(&uartPort->txDMAHandle);
            if (status != HAL_OK)
            {
                while (1);
            }
            /* Associate the initialized DMA handle to the UART handle */
            __HAL_LINKDMA(&uartPort->Handle, hdmatx, uartPort->txDMAHandle);

            __HAL_DMA_SET_COUNTER(&uartPort->txDMAHandle, 0);
        } else
#endif
        {

            /* Enable the UART Transmit Data Register Empty Interrupt */
            SET_BIT(uartPort->USARTx->CR1, USART_CR1_TXEIE);
            SET_BIT(uartPort->USARTx->CR1, USART_CR1_TCIE);
        }
    }

    if (irqWasEnabled) {
        HAL_NVIC_ClearPendingIRQ(irqn);
        HAL_NVIC_EnableIRQ(irqn);
    }
}

bool checkUsartTxOutput(uartPort_t *s)
{
    uartDevice_t *uart = container_of(s, uartDevice_t, port);
    IO_t txIO = IOGetByTag(uart->tx.pin);

    if ((uart->txPinState == TX_PIN_MONITOR) && txIO) {
        if (IORead(txIO)) {
            // TX is high so we're good to transmit

            // Enable USART TX output
            uart->txPinState = TX_PIN_ACTIVE;
            IOConfigGPIOAF(txIO, IOCFG_AF_PP, uart->tx.af);

            // Enable the UART transmitter
            SET_BIT(s->Handle.Instance->CR1, USART_CR1_TE);

            return true;
        } else {
            // TX line is pulled low so don't enable USART TX
            return false;
        }
    }

    return true;
}

void uartTxMonitor(uartPort_t *s)
{
    uartDevice_t *uart = container_of(s, uartDevice_t, port);

    if (uart->txPinState == TX_PIN_ACTIVE) {
        IO_t txIO = IOGetByTag(uart->tx.pin);

        // Disable the UART transmitter
        CLEAR_BIT(s->Handle.Instance->CR1, USART_CR1_TE);

        // Switch TX to an input with pullup so it's state can be monitored
        uart->txPinState = TX_PIN_MONITOR;
        IOConfigGPIO(txIO, IOCFG_IPU);
    }
}

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
    ATOMIC_BLOCK(NVIC_PRIO_SERIALUART_TXDMA) {
        if (!uartCanTx(s)) {
            s->txDMAEmpty = true;
            return;
        }

        if (IS_DMA_ENABLED(s->txDMAResource)) {
            // DMA is already in progress
            return;
        }

        HAL_UART_StateTypeDef state = HAL_UART_GetState(&s->Handle);
        if ((state & HAL_UART_STATE_BUSY_TX) == HAL_UART_STATE_BUSY_TX) {
            // UART is still transmitting
            return;
        }

        if (s->port.txBufferHead == s->port.txBufferTail) {
            // No more data to transmit
            s->txDMAEmpty = true;
            return;
        }

        uint16_t size;
        uint32_t fromwhere = s->port.txBufferTail;

        if (s->port.txBufferHead > s->port.txBufferTail) {
            size = s->port.txBufferHead - s->port.txBufferTail;
            s->port.txBufferTail = s->port.txBufferHead;
        } else {
            size = s->port.txBufferSize - s->port.txBufferTail;
            s->port.txBufferTail = 0;
        }
        s->txDMAEmpty = false;

        HAL_UART_Transmit_DMA(&s->Handle, (uint8_t *)&s->port.txBuffer[fromwhere], size);
    }
}

static void handleUsartTxDma(uartPort_t *s)
{
    uartDevice_t *uart = container_of(s, uartDevice_t, port);

    uartTryStartTxDMA(s);

    if (s->txDMAEmpty && (uart->txPinState != TX_PIN_IGNORE)) {
        // Switch TX to an input with pullup so it's state can be monitored
        uartTxMonitor(s);
    }
}

void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = &(((uartDevice_t*)(descriptor->userParam))->port);

    HAL_DMA_IRQHandler(&s->txDMAHandle);

#ifdef STM32G4
    // G4's DMA HAL turns on half transfer interrupt.
    // Only detect the transfer complete interrupt by checking remaining transfer count.
    // XXX TODO Consider using HAL's XferCpltCallback facility to do this.

    if (s->txDMAHandle.Instance->CNDTR == 0) {

        // Unlike other stream based DMA implementations (F4, F7 and H7),
        // G4's DMA implementation does not clear EN bit upon completion of a transfer,
        // and it is neccesary to clear the EN bit explicitly here for IS_DMA_ENABLED macro
        // used in uartTryStartTxDMA() to continue working with G4.

        __HAL_DMA_DISABLE(&s->txDMAHandle);
    }
#endif
}
#endif

// NOSONAR cognitive complexity: this handler is a flat sequence of independent
// USART flag-clear branches; extracting per-event helpers would add ITCM call overhead
// for a FAST_IRQ_HANDLER and obscure the linear flag-handling structure.
//
// Every branch is double-gated: the cached cr1/cr3 IT-enable bits AND the ISR flag.
// HAL's __HAL_UART_GET_IT reads only the ISR (no enable-bit check), unlike master's
// LL handler which uses LL_USART_IsEnabledIT_* + LL_USART_IsActiveFlag_*. Without
// the explicit cr1/cr3 gate, an IRQ taken with stale ISR flags after the IT-enable
// bits are cleared (e.g. mid-reconfigure on a port with valid TX pin) re-fires the
// branch and storms the handler. Caching cr1/cr3 at entry preserves the original
// behaviour where the RXNE path disables PEIE/EIE for subsequent IRQs without
// affecting flag-handling on the current invocation.
FAST_IRQ_HANDLER void uartIrqHandler(uartPort_t *s)
{
    UART_HandleTypeDef *huart = &s->Handle;
    const bool canTx = uartCanTx(s);
    const uint32_t cr1 = huart->Instance->CR1;
    const uint32_t cr3 = huart->Instance->CR3;

    /* UART in mode Receiver ---------------------------------------------------*/
    if ((cr1 & USART_CR1_RXNEIE) && (__HAL_UART_GET_IT(huart, UART_IT_RXNE) != RESET)) {
        uint8_t rbyte = (uint8_t)(huart->Instance->RDR & (uint8_t) 0xff);

        if (s->port.rxCallback) {
            s->port.rxCallback(rbyte, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = rbyte;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_PEIE));

        /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    }

    /* UART parity error interrupt occurred -------------------------------------*/
    if ((cr1 & USART_CR1_PEIE) && (__HAL_UART_GET_IT(huart, UART_IT_PE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if ((cr3 & USART_CR3_EIE) && (__HAL_UART_GET_IT(huart, UART_IT_FE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if ((cr3 & USART_CR3_EIE) && (__HAL_UART_GET_IT(huart, UART_IT_NE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    // ORE asserts the USART IRQ when CR1.RXNEIE is set even with CR3.EIE clear
    // (RM0433/RM0410; ST HAL gates ORE on RXNEIE || EIE). Betaflight's RXNE path
    // clears CR3.EIE on every received byte, so gating the clear on EIE alone
    // leaves ORE permanently set -> unbounded IRQ re-entry (FC freeze on save).
    if (((cr1 & USART_CR1_RXNEIE) || (cr3 & USART_CR3_EIE)) &&
        (__HAL_UART_GET_IT(huart, UART_IT_ORE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);
    }

    // Pinless / RX-only / half-duplex-without-TX defence-in-depth: TC stays asserted
    // when the TX peripheral never sends, and the per-branch TCIE gate alone won't
    // catch a port that was opened with canTx==false but somehow has TCIE set in
    // cached cr1 (race against another writer). Clearing TXEIE/TCIE here is
    // idempotent and ensures the storm cannot re-arm.
    if (!canTx) {
        CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE | USART_CR1_TCIE);
        if (__HAL_UART_GET_IT(huart, UART_IT_TC)) {
            __HAL_UART_CLEAR_IT(huart, UART_CLEAR_TCF);
        }
    }

    // UART transmission completed
    if (canTx && (cr1 & USART_CR1_TCIE) && (__HAL_UART_GET_IT(huart, UART_IT_TC) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_TCF);

        // Switch TX to an input with pull-up so it's state can be monitored
        uartTxMonitor(s);

#ifdef USE_DMA
        if (s->txDMAResource) {
            handleUsartTxDma(s);
        }
#endif
    }

    if (canTx &&
#ifdef USE_DMA
        !s->txDMAResource &&
#endif
        (cr1 & USART_CR1_TXEIE) &&
        (__HAL_UART_GET_IT(huart, UART_IT_TXE) != RESET)) {
        /* Check that a Tx process is ongoing */
        if (s->port.txBufferTail == s->port.txBufferHead) {
            /* Disable the UART Transmit Data Register Empty Interrupt */
            CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);
        } else {
            if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE)) {
                huart->Instance->TDR = (((uint16_t) s->port.txBuffer[s->port.txBufferTail]) & (uint16_t) 0x01FFU);
            } else {
                huart->Instance->TDR = (uint8_t)(s->port.txBuffer[s->port.txBufferTail]);
            }
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        }
    }

    // UART reception idle detected
    if ((cr1 & USART_CR1_IDLEIE) && __HAL_UART_GET_IT(huart, UART_IT_IDLE)) {
        if (s->port.idleCallback) {
            s->port.idleCallback();
        }

        __HAL_UART_CLEAR_IDLEFLAG(huart);
    }

}
#endif // USE_UART
