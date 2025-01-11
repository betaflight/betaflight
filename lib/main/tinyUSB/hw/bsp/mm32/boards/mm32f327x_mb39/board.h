#ifndef BOARD_H
#define BOARD_H

// GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_15); //Disable JTDI   AF to  AF15
#define LED_PORT              GPIOA
#define LED_PIN               GPIO_Pin_15
#define LED_STATE_ON          1

//#define BUTTON_PORT           GPIOC
//#define BUTTON_PIN            GPIO_PIN_13
//#define BUTTON_STATE_ACTIVE   1

#define UART_DEV              UART1
#define UART_GPIO_PORT        GPIOA
#define UART_GPIO_AF          GPIO_AF_7
#define UART_TX_PIN           9
#define UART_RX_PIN           10

#endif
