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
#include "platform/dma.h"
#include "platform/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "platform/serial_uart_hal.h"

struct uartHalHandle_s uartHalHandles[UARTDEV_COUNT];
struct dmaHalHandle_s uartRxDmaHalHandles[UARTDEV_COUNT];
struct dmaHalHandle_s uartTxDmaHalHandles[UARTDEV_COUNT];

static void usartConfigurePinInversion(uartPort_t *uartPort)
{
    bool inverted = uartPort->port.options & SERIAL_INVERTED;

    if (inverted)
    {
        if (uartPort->port.mode & MODE_RX)
        {
            uartPort->halHandle->hal.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_RXINVERT_INIT;
            uartPort->halHandle->hal.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
        }
        if (uartPort->port.mode & MODE_TX)
        {
            uartPort->halHandle->hal.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_TXINVERT_INIT;
            uartPort->halHandle->hal.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
        }
    }
}

#if UART_TRAIT_PINSWAP
static void uartConfigurePinSwap(uartPort_t *uartPort)
{
    uartDevice_t *uartDevice = container_of(uartPort, uartDevice_t, port);
    if (uartDevice->pinSwap) {
        uartDevice->port.halHandle->hal.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_SWAP_INIT;
        uartDevice->port.halHandle->hal.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
    }
}
#endif

// XXX uartReconfigure does not handle resource management properly.

void uartReconfigure(uartPort_t *uartPort)
{
    HAL_UART_DeInit(&uartPort->halHandle->hal);
    uartPort->halHandle->hal.Init.BaudRate = uartPort->port.baudRate;
    // according to the stm32 documentation wordlen has to be 9 for parity bits
    // this does not seem to matter for rx but will give bad data on tx!
    uartPort->halHandle->hal.Init.WordLength = (uartPort->port.options & SERIAL_PARITY_EVEN) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
    uartPort->halHandle->hal.Init.StopBits = (uartPort->port.options & SERIAL_STOPBITS_2) ? USART_STOPBITS_2 : USART_STOPBITS_1;
    uartPort->halHandle->hal.Init.Parity = (uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_PARITY_EVEN : USART_PARITY_NONE;
    uartPort->halHandle->hal.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uartPort->halHandle->hal.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uartPort->halHandle->hal.Init.Mode = 0;
#if defined(STM32G4) || defined(STM32H7) || defined(STM32N6)
    if (uartPort->halHandle->hal.Instance == LPUART1) {
        uartPort->halHandle->hal.Init.ClockPrescaler = UART_PRESCALER_DIV8;
    }
#endif

    if (uartPort->port.mode & MODE_RX)
        uartPort->halHandle->hal.Init.Mode |= UART_MODE_RX;
    if (uartPort->port.mode & MODE_TX)
        uartPort->halHandle->hal.Init.Mode |= UART_MODE_TX;

    usartConfigurePinInversion(uartPort);
#if !(defined(STM32F1) || defined(STM32F4))
    uartConfigurePinSwap(uartPort);
#endif

#ifdef TARGET_USART_CONFIG
    void usartTargetConfigure(uartPort_t *);
    usartTargetConfigure(uartPort);
#endif

    if (uartPort->port.options & SERIAL_BIDIR) {
        HAL_HalfDuplex_Init(&uartPort->halHandle->hal);
    } else {
        HAL_UART_Init(&uartPort->halHandle->hal);
    }

    // Receive DMA or IRQ
    if (uartPort->port.mode & MODE_RX)
    {
#ifdef USE_DMA
        if (uartPort->rxDMAResource)
        {
            uartPort->rxDmaHalHandle->hal.Instance = (DMA_ARCH_TYPE *)uartPort->rxDMAResource;
#if defined(STM32N6)
            // TODO: STM32N6 GPDMA init for UART RX
            uartPort->rxDmaHalHandle->hal.Init.Request = uartPort->rxDMAChannel;
            uartPort->rxDmaHalHandle->hal.Init.Direction = DMA_PERIPH_TO_MEMORY;
            uartPort->rxDmaHalHandle->hal.Init.SrcInc = DMA_SINC_FIXED;
            uartPort->rxDmaHalHandle->hal.Init.DestInc = DMA_DINC_INCREMENTED;
            uartPort->rxDmaHalHandle->hal.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
            uartPort->rxDmaHalHandle->hal.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
            uartPort->rxDmaHalHandle->hal.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
            uartPort->rxDmaHalHandle->hal.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
#elif !(defined(STM32H7) || defined(STM32G4))
            uartPort->rxDmaHalHandle->hal.Init.Channel = uartPort->rxDMAChannel;
#else
            uartPort->txDmaHalHandle->hal.Init.Request = uartPort->rxDMAChannel;
#endif
#if !defined(STM32N6)
            uartPort->rxDmaHalHandle->hal.Init.Direction = DMA_PERIPH_TO_MEMORY;
            uartPort->rxDmaHalHandle->hal.Init.PeriphInc = DMA_PINC_DISABLE;
            uartPort->rxDmaHalHandle->hal.Init.MemInc = DMA_MINC_ENABLE;
            uartPort->rxDmaHalHandle->hal.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            uartPort->rxDmaHalHandle->hal.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            uartPort->rxDmaHalHandle->hal.Init.Mode = DMA_CIRCULAR;
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
            uartPort->rxDmaHalHandle->hal.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            uartPort->rxDmaHalHandle->hal.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
            uartPort->rxDmaHalHandle->hal.Init.PeriphBurst = DMA_PBURST_SINGLE;
            uartPort->rxDmaHalHandle->hal.Init.MemBurst = DMA_MBURST_SINGLE;
#endif
            uartPort->rxDmaHalHandle->hal.Init.Priority = DMA_PRIORITY_MEDIUM;
#endif

            HAL_DMA_DeInit(&uartPort->rxDmaHalHandle->hal);
            HAL_DMA_Init(&uartPort->rxDmaHalHandle->hal);
            /* Associate the initialized DMA handle to the UART handle */
            __HAL_LINKDMA(&uartPort->halHandle->hal, hdmarx, uartPort->rxDmaHalHandle->hal);

            HAL_UART_Receive_DMA(&uartPort->halHandle->hal, (uint8_t*)uartPort->port.rxBuffer, uartPort->port.rxBufferSize);

            uartPort->rxDMAPos = __HAL_DMA_GET_COUNTER(&uartPort->rxDmaHalHandle->hal);
        } else
#endif
        {
            /* Enable the UART Parity Error Interrupt */
            SET_BIT(((USART_TypeDef *)uartPort->USARTx)->CR1, USART_CR1_PEIE);

            /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
            SET_BIT(((USART_TypeDef *)uartPort->USARTx)->CR3, USART_CR3_EIE);

            /* Enable the UART Data Register not empty Interrupt */
            SET_BIT(((USART_TypeDef *)uartPort->USARTx)->CR1, USART_CR1_RXNEIE);

            /* Enable Idle Line detection */
            SET_BIT(((USART_TypeDef *)uartPort->USARTx)->CR1, USART_CR1_IDLEIE);
        }
    }

    // Transmit DMA or IRQ
    if (uartPort->port.mode & MODE_TX) {
#ifdef USE_DMA
        if (uartPort->txDMAResource) {
            uartPort->txDmaHalHandle->hal.Instance = (DMA_ARCH_TYPE *)uartPort->txDMAResource;
#if defined(STM32N6)
            // TODO: STM32N6 GPDMA init for UART TX
            uartPort->txDmaHalHandle->hal.Init.Request = uartPort->txDMAChannel;
            uartPort->txDmaHalHandle->hal.Init.Direction = DMA_MEMORY_TO_PERIPH;
            uartPort->txDmaHalHandle->hal.Init.SrcInc = DMA_SINC_INCREMENTED;
            uartPort->txDmaHalHandle->hal.Init.DestInc = DMA_DINC_FIXED;
            uartPort->txDmaHalHandle->hal.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
            uartPort->txDmaHalHandle->hal.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
            uartPort->txDmaHalHandle->hal.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
            uartPort->txDmaHalHandle->hal.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
#elif !(defined(STM32H7) || defined(STM32G4))
            uartPort->txDmaHalHandle->hal.Init.Channel = uartPort->txDMAChannel;
#else
            uartPort->txDmaHalHandle->hal.Init.Request = uartPort->txDMAChannel;
#endif
#if !defined(STM32N6)
            uartPort->txDmaHalHandle->hal.Init.Direction = DMA_MEMORY_TO_PERIPH;
            uartPort->txDmaHalHandle->hal.Init.PeriphInc = DMA_PINC_DISABLE;
            uartPort->txDmaHalHandle->hal.Init.MemInc = DMA_MINC_ENABLE;
            uartPort->txDmaHalHandle->hal.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            uartPort->txDmaHalHandle->hal.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            uartPort->txDmaHalHandle->hal.Init.Mode = DMA_NORMAL;
#if !defined(STM32G4)
            // G4's DMA is channel based, and does not have FIFO
            uartPort->txDmaHalHandle->hal.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            uartPort->txDmaHalHandle->hal.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
            uartPort->txDmaHalHandle->hal.Init.PeriphBurst = DMA_PBURST_SINGLE;
            uartPort->txDmaHalHandle->hal.Init.MemBurst = DMA_MBURST_SINGLE;
#endif
            uartPort->txDmaHalHandle->hal.Init.Priority = DMA_PRIORITY_MEDIUM;
#endif

            HAL_DMA_DeInit(&uartPort->txDmaHalHandle->hal);
            HAL_StatusTypeDef status = HAL_DMA_Init(&uartPort->txDmaHalHandle->hal);
            if (status != HAL_OK)
            {
                while (1);
            }
            /* Associate the initialized DMA handle to the UART handle */
            __HAL_LINKDMA(&uartPort->halHandle->hal, hdmatx, uartPort->txDmaHalHandle->hal);

            __HAL_DMA_SET_COUNTER(&uartPort->txDmaHalHandle->hal, 0);
        } else
#endif
        {

            /* Enable the UART Transmit Data Register Empty Interrupt */
            SET_BIT(((USART_TypeDef *)uartPort->USARTx)->CR1, USART_CR1_TXEIE);
            SET_BIT(((USART_TypeDef *)uartPort->USARTx)->CR1, USART_CR1_TCIE);
        }
    }
    return;
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
            SET_BIT(s->halHandle->hal.Instance->CR1, USART_CR1_TE);

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
        CLEAR_BIT(s->halHandle->hal.Instance->CR1, USART_CR1_TE);

        // Switch TX to an input with pullup so it's state can be monitored
        uart->txPinState = TX_PIN_MONITOR;
        IOConfigGPIO(txIO, IOCFG_IPU);
    }
}

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
    ATOMIC_BLOCK(NVIC_PRIO_SERIALUART_TXDMA) {
        if (IS_DMA_ENABLED(s->txDMAResource)) {
            // DMA is already in progress
            return;
        }

        HAL_UART_StateTypeDef state = HAL_UART_GetState(&s->halHandle->hal);
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

        HAL_UART_Transmit_DMA(&s->halHandle->hal, (uint8_t *)&s->port.txBuffer[fromwhere], size);
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

    HAL_DMA_IRQHandler(&s->txDmaHalHandle->hal);

#ifdef STM32G4
    // G4's DMA HAL turns on half transfer interrupt.
    // Only detect the transfer complete interrupt by checking remaining transfer count.
    // XXX TODO Consider using HAL's XferCpltCallback facility to do this.

    if (s->txDmaHalHandle->hal.Instance->CNDTR == 0) {

        // Unlike other stream based DMA implementations (F4, F7 and H7),
        // G4's DMA implementation does not clear EN bit upon completion of a transfer,
        // and it is neccesary to clear the EN bit explicitly here for IS_DMA_ENABLED macro
        // used in uartTryStartTxDMA() to continue working with G4.

        __HAL_DMA_DISABLE(&s->txDmaHalHandle->hal);
    }
#endif
}
#endif

FAST_IRQ_HANDLER void uartIrqHandler(uartPort_t *s)
{
    UART_HandleTypeDef *huart = &s->halHandle->hal;
    /* UART in mode Receiver ---------------------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_RXNE) != RESET)) {
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
    if ((__HAL_UART_GET_IT(huart, UART_IT_PE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_FE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_NE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_ORE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);
    }

    // UART transmission completed
    if ((__HAL_UART_GET_IT(huart, UART_IT_TC) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_TCF);

        // Switch TX to an input with pull-up so it's state can be monitored
        uartTxMonitor(s);

#ifdef USE_DMA
        if (s->txDMAResource) {
            handleUsartTxDma(s);
        }
#endif
    }

    if (
#ifdef USE_DMA
        !s->txDMAResource &&
#endif
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

    if (__HAL_UART_GET_IT(huart, UART_IT_IDLE)) {
        if (s->port.idleCallback) {
            s->port.idleCallback();
        }

        __HAL_UART_CLEAR_IDLEFLAG(huart);
    }

}
#endif // USE_UART
