#ifndef BOARD_H_
#define BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

#define LED_PORT       GPIOA
#define LED_PIN        GPIO_Pin_10
#define LED_STATE_ON   0

#define BUTTON_PORT           GPIOA
#define BUTTON_PIN            GPIO_Pin_1
#define BUTTON_STATE_ACTIVE   0

#ifdef __cplusplus
}
#endif

#endif
