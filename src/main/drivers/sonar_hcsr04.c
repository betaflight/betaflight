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
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "system.h"
#include "gpio.h"

#include "sonar_hcsr04.h"

#ifdef SONAR

/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When trigged it sends out a series of 40KHz ultrasonic pulses and receives
 * echo froman object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */

static uint32_t last_measurement;
static volatile int32_t *distance_ptr = 0;

extern int16_t debug[4];

static sonarHardware_t const *sonarHardware;

void ECHO_EXTI_IRQHandler(void)
{
    static uint32_t timing_start;
    uint32_t timing_stop;
    if (digitalIn(GPIOB, sonarHardware->echo_pin) != 0) {
        timing_start = micros();
    } else {
        timing_stop = micros();
        if (timing_stop > timing_start) {
            // The speed of sound is 340 m/s or approx. 29 microseconds per centimeter.
            // The ping travels out and back, so to find the distance of the
            // object we take half of the distance traveled.
            //
            // 340 m/s = 0.034 cm/microsecond = 29.41176471 *2 = 58.82352941 rounded to 59
            int32_t distance = (timing_stop - timing_start) / 59;
            // this sonar range is up to 4meter , but 3meter is the safe working range (+tilted and roll)
            if (distance > 300)
                distance = -1;

            if (distance_ptr) {
                *distance_ptr = distance;
            }
        }
    }

    EXTI_ClearITPendingBit(sonarHardware->exti_line);
}

void EXTI1_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void hcsr04_init(const sonarHardware_t *initialSonarHardware)
{
    gpio_config_t gpio;
    EXTI_InitTypeDef EXTIInit;

    sonarHardware = initialSonarHardware;

    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // tp - trigger pin
    gpio.pin = sonarHardware->trigger_pin;
    gpio.mode = Mode_Out_PP;
    gpio.speed = Speed_2MHz;
    gpioInit(GPIOB, &gpio);

    // ep - echo pin
    gpio.pin = sonarHardware->echo_pin;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(GPIOB, &gpio);

    // setup external interrupt on echo pin
    gpioExtiLineConfig(GPIO_PortSourceGPIOB, sonarHardware->exti_pin_source);

    EXTI_ClearITPendingBit(sonarHardware->exti_line);

    EXTIInit.EXTI_Line = sonarHardware->exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_EnableIRQ(sonarHardware->exti_irqn);

    last_measurement = millis() - 60; // force 1st measurement in hcsr04_get_distance()
}

// distance calculation is done asynchronously, using interrupt
void hcsr04_get_distance(volatile int32_t *distance)
{
    uint32_t current_time = millis();

    if (current_time < (last_measurement + 60)) {
        // the repeat interval of trig signal should be greater than 60ms
        // to avoid interference between connective measurements.
        return;
    }

    last_measurement = current_time;
    distance_ptr = distance;

#if 1
    debug[0] = *distance;
#endif

    digitalHi(GPIOB, sonarHardware->trigger_pin);
    //  The width of trig signal must be greater than 10us
    delayMicroseconds(11);
    digitalLo(GPIOB, sonarHardware->trigger_pin);
}
#endif
