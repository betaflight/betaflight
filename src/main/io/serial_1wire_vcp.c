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
 *
 * Author 4712
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <platform.h>

#ifdef USE_SERIAL_1WIRE_VCP
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/system.h"
#include "io/serial_1wire_vcp.h"
#include "io/beeper.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_output.h"
#include "flight/mixer.h"
#include "io/serial_msp.h"
#include "drivers/buf_writer.h"
#include "drivers/serial_usb_vcp.h"

uint8_t escCount;

static escHardware_t escHardware[MAX_PWM_MOTORS];

static uint32_t GetPinPos(uint32_t pin) {
    uint32_t pinPos;
    for (pinPos = 0; pinPos < 16; pinPos++) {
        uint32_t pinMask = (0x1 << pinPos);
        if (pin & pinMask) {
            return pinPos;
        }
    }
    return 0;
}
static uint8_t selected_esc;

#define ESC_IS_HI   digitalIn(escHardware[selected_esc].gpio, escHardware[selected_esc].pin) != Bit_RESET
#define ESC_IS_LO   digitalIn(escHardware[selected_esc].gpio, escHardware[selected_esc].pin) == Bit_RESET
#define ESC_SET_HI  digitalHi(escHardware[selected_esc].gpio, escHardware[selected_esc].pin)
#define ESC_SET_LO  digitalLo(escHardware[selected_esc].gpio, escHardware[selected_esc].pin)

#define ESC_INPUT   gpioInit(escHardware[selected_esc].gpio, &escHardware[selected_esc].gpio_config_INPUT)
#define ESC_OUTPUT  gpioInit(escHardware[selected_esc].gpio, &escHardware[selected_esc].gpio_config_OUTPUT)

void usb1WireInitializeVcp(void)
{
    pwmDisableMotors();
    selected_esc = 0;
    memset(&escHardware, 0, sizeof(escHardware));
    pwmIOConfiguration_t *pwmIOConfiguration = pwmGetOutputConfiguration();
    for (volatile uint8_t i = 0; i < pwmIOConfiguration->ioCount; i++) {
      if ((pwmIOConfiguration->ioConfigurations[i].flags & PWM_PF_MOTOR) == PWM_PF_MOTOR) {
          if(motor[pwmIOConfiguration->ioConfigurations[i].index] > 0) {
              escHardware[selected_esc].gpio = pwmIOConfiguration->ioConfigurations[i].timerHardware->gpio;
              escHardware[selected_esc].pin = pwmIOConfiguration->ioConfigurations[i].timerHardware->pin;
              escHardware[selected_esc].pinpos = GetPinPos(escHardware[selected_esc].pin);
              escHardware[selected_esc].gpio_config_INPUT.pin = escHardware[selected_esc].pin;
              escHardware[selected_esc].gpio_config_INPUT.speed = Speed_2MHz; // see pwmOutConfig()
              escHardware[selected_esc].gpio_config_INPUT.mode = Mode_IPU;
              escHardware[selected_esc].gpio_config_OUTPUT = escHardware[selected_esc].gpio_config_INPUT;
              escHardware[selected_esc].gpio_config_OUTPUT.mode = Mode_Out_PP;
              ESC_INPUT;
              ESC_SET_HI;
              selected_esc++;
          }
      }
    }
    escCount = selected_esc;
    selected_esc = 0;
}

void usb1WireDeInitializeVcp(void){
    for (selected_esc = 0; selected_esc < (escCount); selected_esc++) {
        escHardware[selected_esc].gpio_config_OUTPUT.mode = Mode_AF_PP; // see pwmOutConfig()
        ESC_OUTPUT;
        ESC_SET_LO;
    }
    escCount = 0;
    pwmEnableMotors();
}

#define START_BIT_TIMEOUT_MS 2

#define BIT_TIME (52)  //52uS
#define BIT_TIME_HALVE (BIT_TIME >> 1) //26uS
#define START_BIT_TIME (BIT_TIME_HALVE + 1)
#define STOP_BIT_TIME ((BIT_TIME * 9) + BIT_TIME_HALVE)

static void suart_putc_(uint8_t tx_b)
{
    uint32_t btime;
    ESC_SET_LO; // 0 = start bit
    btime = BIT_TIME + micros();
    while (micros() < btime);
    for(uint8_t bit = 0; bit <8; bit++)
    {
        if(tx_b & 1)
        {
            ESC_SET_HI; // 1
        }
        else
        {
            ESC_SET_LO; // 0
        }
        btime = btime + BIT_TIME;
        tx_b = (tx_b >> 1);
        while (micros() < btime);
    }
    ESC_SET_HI; // 1 = stop bit
    btime = btime + BIT_TIME;
    while (micros() < btime);
}


static uint8_t suart_getc_(uint8_t *bt)
{
    uint32_t btime;
    uint32_t start_time;
    uint32_t stop_time;
    uint32_t wait_start;

    *bt = 0;

    wait_start = millis() + START_BIT_TIMEOUT_MS;
    while (ESC_IS_HI) {
        // check for start bit begin
        if (millis() > wait_start) {
            return 0;
        }
    }
    // start bit
    start_time = micros();
    btime = start_time + START_BIT_TIME;
    stop_time = start_time + STOP_BIT_TIME;

    while (micros() < btime);

    if (ESC_IS_HI) return 0; // check start bit
    for (uint8_t bit=0;bit<8;bit++)
    {
        btime = btime + BIT_TIME;
        while (micros() < btime);
        if (ESC_IS_HI)
        {
             *bt |= (1 << bit); // 1 else 0
        }
    }
    while (micros() < stop_time);

    if (ESC_IS_LO) return 0;  // check stop bit

    return 1; // OK
}
#define USE_TXRX_LED

#ifdef  USE_TXRX_LED
#define RX_LED_OFF LED0_OFF
#define RX_LED_ON LED0_ON
#ifdef  LED1
#define TX_LED_OFF LED1_OFF
#define TX_LED_ON LED1_ON
#else
#define TX_LED_OFF LED0_OFF
#define TX_LED_ON LED0_ON
#endif
#else
#define RX_LED_OFF
#define RX_LED_ON
#define TX_LED_OFF
#define TX_LED_ON
#endif

// This method translates 2 wires (a tx and rx line) to 1 wire, by letting the
// RX line control when data should be read or written from the single line
void usb1WirePassthroughVcp(mspPort_t *mspPort, bufWriter_t *bufwriter, uint8_t escIndex)
{
#ifdef BEEPER
    // fix for buzzer often starts beeping continuously when the ESCs are read
    // switch beeper silent here
    // TODO (4712) do we need beeperSilence()?
    // beeperSilence();
#endif

    selected_esc = escIndex;
    // TODO (4712) check all possible baud rate ok?
    // uint32_t baudrate = serialGetBaudRate(mspPort->port);
    // serialSetBaudRate(mspPort->port, 19200);

    while(usbVcpGetBaudRate(mspPort->port) != 4800) {
        // esc line is in Mode_IPU by default
        static uint8_t bt;

        if (suart_getc_(&bt)) {
            RX_LED_ON;
            serialBeginWrite(mspPort->port);
            bufWriterAppend(bufwriter, bt);
            while (suart_getc_(&bt)){
               bufWriterAppend(bufwriter, bt);
            }
            serialEndWrite(mspPort->port);
            bufWriterFlush(bufwriter);
            RX_LED_OFF;
        }
        if (serialRxBytesWaiting(mspPort->port)) {
            ESC_OUTPUT;
            TX_LED_ON;
            do {
                bt = serialRead(mspPort->port);
                suart_putc_(bt);
            }  while(serialRxBytesWaiting(mspPort->port));
            ESC_INPUT;
            TX_LED_OFF;
        }
    }
    // serialSetBaudRate(mspPort->port, baudrate);
    return;
}
#endif

