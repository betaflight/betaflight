#include "board.h"
#include "mw.h"

#ifdef SONAR

/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When trigged it sends out a series of 40KHz ultrasonic pulses and receives
 * echo froman object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */

static uint16_t trigger_pin;
static uint16_t echo_pin;
static uint32_t exti_line;
static uint8_t exti_pin_source;
static IRQn_Type exti_irqn;

static uint32_t last_measurement;
static volatile int32_t *distance_ptr;

void ECHO_EXTI_IRQHandler(void)
{
    static uint32_t timing_start;
    uint32_t timing_stop;
    if (digitalIn(GPIOB, echo_pin) != 0) {
        timing_start = micros();
    } else {
        timing_stop = micros();
        if (timing_stop > timing_start) {
            // The speed of sound is 340 m/s or approx. 29 microseconds per centimeter.
            // The ping travels out and back, so to find the distance of the
            // object we take half of the distance traveled.
            //
            // 340 m/s = 0.034 cm/microsecond = 29.41176471 *2 = 58.82352941 rounded to 59
            int32_t pulse_duration = timing_stop - timing_start;
            *distance_ptr = pulse_duration / 59;
        }
    }

    EXTI_ClearITPendingBit(exti_line);
}

void EXTI1_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void hcsr04_init(sonar_config_t config)
{
    gpio_config_t gpio;
    EXTI_InitTypeDef EXTIInit;

    // enable AFIO for EXTI support - already done is drv_system.c
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph, ENABLE);

    switch(config) {
        case sonar_pwm56:
            trigger_pin = Pin_8;   // PWM5 (PB8) - 5v tolerant
            echo_pin = Pin_9;      // PWM6 (PB9) - 5v tolerant
            exti_line = EXTI_Line9;
            exti_pin_source = GPIO_PinSource9;
            exti_irqn = EXTI9_5_IRQn;
            break;
        case sonar_rc78:
            trigger_pin = Pin_0;   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
            echo_pin = Pin_1;      // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
            exti_line = EXTI_Line1;
            exti_pin_source = GPIO_PinSource1;
            exti_irqn = EXTI1_IRQn;
            break;
    }

    // tp - trigger pin
    gpio.pin = trigger_pin;
    gpio.mode = Mode_Out_PP;
    gpio.speed = Speed_2MHz;
    gpioInit(GPIOB, &gpio);

    // ep - echo pin
    gpio.pin = echo_pin;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(GPIOB, &gpio);

    // setup external interrupt on echo pin
    gpioExtiLineConfig(GPIO_PortSourceGPIOB, exti_pin_source);

    EXTI_ClearITPendingBit(exti_line);

    EXTIInit.EXTI_Line = exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_EnableIRQ(exti_irqn);

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

    digitalHi(GPIOB, trigger_pin);
    //  The width of trig signal must be greater than 10us
    delayMicroseconds(11);
    digitalLo(GPIOB, trigger_pin);
}
#endif
