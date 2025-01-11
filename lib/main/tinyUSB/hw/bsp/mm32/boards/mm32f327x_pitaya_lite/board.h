#ifndef BOARD_H
#define BOARD_H

// GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_15); //Disable JTDI   AF to  AF15
#define LED_PORT              GPIOA
#define LED_PIN               GPIO_Pin_1
#define LED_STATE_ON          1

#define BUTTON_PORT           GPIOA
#define BUTTON_PIN            GPIO_Pin_0
#define BUTTON_STATE_ACTIVE   0


#endif
