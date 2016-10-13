
#pragma once

typedef enum {
    OWNER_FREE = 0,
    OWNER_PWMINPUT,
    OWNER_PPMINPUT,
    OWNER_PWMOUTPUT_MOTOR,
    OWNER_PWMOUTPUT_FAST,
    OWNER_PWMOUTPUT_ONESHOT,
    OWNER_PWMOUTPUT_SERVO,
    OWNER_SOFTSERIAL_RX,
    OWNER_SOFTSERIAL_TX,
    OWNER_SOFTSERIAL_RXTX,        // bidirectional pin for softserial
    OWNER_SOFTSERIAL_AUXTIMER,    // timer channel is used for softserial. No IO function on pin
    OWNER_ADC,
    OWNER_SERIAL_RX,
    OWNER_SERIAL_TX,
    OWNER_SERIAL_RXTX,
    OWNER_PINDEBUG,
    OWNER_TIMER,
    OWNER_SONAR,
    OWNER_SYSTEM,
} resourceOwner_t;


// Currently TIMER should be shared resource (softserial dualtimer and timerqueue needs to allocate timer channel, but pin can be used for other function)
// with mode switching (shared serial ports, ...) this will need some improvement
typedef enum {
    RESOURCE_INPUT = 1 << 0,
    RESOURCE_OUTPUT = 1<< 1,
    RESOURCE_IO = RESOURCE_INPUT | RESOURCE_OUTPUT,
    RESOURCE_TIMER = 1 << 2,
    RESOURCE_TIMER_DUAL = 1 << 3, // channel used in dual-capture, other channel will be allocated too
    RESOURCE_USART = 1 << 4,
    RESOURCE_ADC = 1 << 5,
    RESOURCE_EXTI = 1 << 6,
} resourceType_t;
