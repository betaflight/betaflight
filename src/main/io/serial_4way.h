/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 * Author: 4712
*/

#include <platform.h>
#if (defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER) || defined(USE_SERIAL_4WAY_SK_BOOTLOADER))
#include "drivers/serial.h"
#include "drivers/buf_writer.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_mapping.h"
#include "drivers/light_led.h"
#include "io/serial_msp.h"


#define imC2 0
#define imSIL_BLB 1
#define imATM_BLB 2
#define imSK 3

#define RX_LED_OFF  LED0_OFF
#define RX_LED_ON   LED0_ON
#define TX_LED_OFF  LED1_OFF
#define TX_LED_ON   LED1_ON

typedef struct {
    GPIO_TypeDef* gpio;
    uint16_t pinpos;
    uint16_t pin;
    gpio_config_t gpio_config_INPUT;
    gpio_config_t gpio_config_OUTPUT;
} escHardware_t;

uint8_t selected_esc;

escHardware_t escHardware[MAX_PWM_MOTORS];

#define ESC_IS_HI   digitalIn(escHardware[selected_esc].gpio, escHardware[selected_esc].pin) != Bit_RESET
#define ESC_IS_LO   digitalIn(escHardware[selected_esc].gpio, escHardware[selected_esc].pin) == Bit_RESET
#define ESC_SET_HI  digitalHi(escHardware[selected_esc].gpio, escHardware[selected_esc].pin)
#define ESC_SET_LO  digitalLo(escHardware[selected_esc].gpio, escHardware[selected_esc].pin)

#define ESC_INPUT   gpioInit(escHardware[selected_esc].gpio, &escHardware[selected_esc].gpio_config_INPUT)
#define ESC_OUTPUT  gpioInit(escHardware[selected_esc].gpio, &escHardware[selected_esc].gpio_config_OUTPUT)



void delay_us(uint32_t delay);

union __attribute__ ((packed))  uint8_16u
{
    uint8_t bytes[2];
    uint16_t word;
};

union __attribute__ ((packed))  uint8_32u
{
    uint8_t bytes[4];
    uint16_t words[2];
    uint32_t dword;
};

//-----------------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------------
uint8_t D_NUM_BYTES;
uint8_t D_FLASH_ADDR_H;
uint8_t D_FLASH_ADDR_L;
uint8_t *D_PTR_I;

#define DeviceInfoSize 4

union uint8_32u DeviceInfo;

#define IsMcuConnected (DeviceInfo.bytes[0] > 0)

uint8_t Initialize4WayInterface(void);
void Process4WayInterface(mspPort_t *mspPort, bufWriter_t *bufwriter);
void DeInitialize4WayInterface(void);

#endif
