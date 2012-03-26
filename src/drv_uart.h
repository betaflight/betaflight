#pragma once

void uartInit(void);
uint16_t uartAvailable(void);
uint8_t uartRead(void);
uint8_t uartReadPoll(void);
void uartWrite(uint8_t ch);
void uartPrint(char *str);
void uart2Init(uint32_t speed, uartReceiveCallbackPtr func);
