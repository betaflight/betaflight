#pragma once

// FIXME since serial ports can be used for any function these buffer sizes probably need normalising.
// Code is optimal when buffer sizes are powers of 2 due to use of % and / operators.
#define UART1_RX_BUFFER_SIZE    256
#define UART1_TX_BUFFER_SIZE    256
#define UART2_RX_BUFFER_SIZE    128
#define UART2_TX_BUFFER_SIZE    64

typedef struct {
    serialPort_t port;

    // FIXME these are uart specific and do not belong in here
    DMA_Channel_TypeDef *rxDMAChannel;
    DMA_Channel_TypeDef *txDMAChannel;

    uint32_t rxDMAIrq;
    uint32_t txDMAIrq;

    uint32_t rxDMAPos;
    bool txDMAEmpty;

    uint32_t txDMAPeripheralBaseAddr;
    uint32_t rxDMAPeripheralBaseAddr;

    USART_TypeDef *USARTx;
} uartPort_t;

extern const struct serialPortVTable uartVTable[];

serialPort_t *uartOpen(USART_TypeDef *USARTx, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, serialInversion_e inversion);

// serialPort API
void uartWrite(serialPort_t *instance, uint8_t ch);
uint8_t uartTotalBytesWaiting(serialPort_t *instance);
uint8_t uartRead(serialPort_t *instance);
void uartSetBaudRate(serialPort_t *s, uint32_t baudRate);
bool isUartTransmitBufferEmpty(serialPort_t *s);
