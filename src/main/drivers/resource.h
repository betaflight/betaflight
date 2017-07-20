
#pragma once

#define RESOURCE_INDEX(x) (x + 1)

typedef enum {
    OWNER_FREE = 0,
    OWNER_PWMINPUT,
    OWNER_PPMINPUT,
    OWNER_MOTOR,
    OWNER_SERVO,
    OWNER_SOFTSERIAL,
    OWNER_ADC,
    OWNER_SERIAL,
    OWNER_PINDEBUG,
    OWNER_TIMER,
    OWNER_RANGEFINDER,
    OWNER_SYSTEM,
    OWNER_SPI,
    OWNER_I2C,
    OWNER_SDCARD,
    OWNER_FLASH,
    OWNER_USB,
    OWNER_BEEPER,
    OWNER_OSD,
    OWNER_BARO,
    OWNER_MPU,
    OWNER_INVERTER,
    OWNER_LED_STRIP,
    OWNER_LED,
    OWNER_RX,
    OWNER_TX,
    OWNER_SOFTSPI,
    OWNER_RX_SPI,
    OWNER_VTX,
    OWNER_TOTAL_COUNT
} resourceOwner_t;

extern const char * const ownerNames[OWNER_TOTAL_COUNT];

// Currently TIMER should be shared resource (softserial dualtimer and timerqueue needs to allocate timer channel, but pin can be used for other function)
// with mode switching (shared serial ports, ...) this will need some improvement
typedef enum {
    RESOURCE_NONE       = 0,
    RESOURCE_INPUT, RESOURCE_OUTPUT, RESOURCE_IO,
    RESOURCE_TIMER,
    RESOURCE_UART_TX, RESOURCE_UART_RX, RESOURCE_UART_TXRX,
    RESOURCE_EXTI,
    RESOURCE_I2C_SCL, RESOURCE_I2C_SDA,
    RESOURCE_SPI_SCK, RESOURCE_SPI_MOSI, RESOURCE_SPI_MISO, RESOURCE_SPI_CS,
    RESOURCE_ADC_CH1, RESOURCE_ADC_CH2, RESOURCE_ADC_CH3, RESOURCE_ADC_CH4,
    RESOURCE_RX_CE,
    RESOURCE_TOTAL_COUNT
} resourceType_t;

extern const char * const resourceNames[RESOURCE_TOTAL_COUNT];
