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

#define TARGET_BOARD_IDENTIFIER "CC3D" // CopterControl 3D

#define LED0                    PB3

#define INVERTER_PIN_UART1      PB2 // PB2 (BOOT1) used as inverter select GPIO

#define BEEPER                  PA15
#define BEEPER_OPT              PB2

#define USE_EXTI
#define MPU_INT_EXTI            PA3
#define USE_MPU_DATA_READY_SIGNAL
//#define DEBUG_MPU_DATA_READY_INTERRUPT

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2) // Flex port - SCL/PB10, SDA/PB11
#define I2C_DEVICE_SHARES_UART3

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

#define M25P16_CS_PIN           PB12
#define M25P16_SPI_INSTANCE     SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN CW270_DEG

#define ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN CW270_DEG

// External I2C BARO
#define BARO
//#define USE_BARO_MS5611
#define USE_BARO_BMP085
#define USE_BARO_BMP280

// External I2C MAG
#define MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
//#define USE_MAG_AK8975
//#define USE_MAG_MAG3110

#define USB_IO
#define USE_VCP
#define USE_UART1
#define USE_UART3

#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10


#if defined(CC3D_NRF24)
#define USE_RX_NRF24
#endif

#ifdef USE_RX_NRF24
#define USE_RX_SPI
#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI
#define DEFAULT_FEATURES        FEATURE_SOFTSPI
#define USE_RX_SYMA
//#define USE_RX_V202
#define USE_RX_CX10
//#define USE_RX_H8_3D
#define USE_RX_INAV
#define RX_SPI_DEFAULT_PROTOCOL  NRF24RX_SYMA_X5C
//#define RX_SPI_DEFAULT_PROTOCOL NRF24RX_V202_1M
//#define RX_SPI_DEFAULT_PROTOCOL NRF24RX_H8_3D

#define USE_SOFTSPI
#define USE_RX_SOFTSPI

// RC pinouts
// RC1              GND
// RC2              power
// RC3  PB6/TIM4    unused
// RC4  PB5/TIM3    SCK / softserial1 TX / HC-SR04 trigger
// RC5  PB0/TIM3    MISO / softserial1 RX / HC-SR04 echo / RSSI ADC
// RC6  PB1/TIM3    MOSI / current
// RC7  PA0/TIM2    CSN / battery voltage
// RC8  PA1/TIM2    CE / RX_PPM

// Nordic Semiconductor uses 'CSN', STM uses 'NSS'
#define RX_NSS_GPIO_CLK_PERIPHERAL  RCC_APB2Periph_GPIOA
#define RX_NSS_PIN                  PA0
#define RX_CE_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA
#define RX_CE_PIN                   PA1
#define RX_NSS_PIN                  PA0
#define RX_SCK_PIN                  PB5
#define RX_MOSI_PIN                 PB1
#define RX_MISO_PIN                 PB0

#define SERIAL_PORT_COUNT 3

#else

#define USE_SOFTSERIAL1
#define SERIAL_PORT_COUNT       4

#ifdef USE_UART1_RX_DMA
#undef USE_UART1_RX_DMA
#endif

#define SOFTSERIAL_1_TX_PIN     PB5
#define SOFTSERIAL_1_RX_PIN     PB0

#define DEFAULT_RX_FEATURE FEATURE_RX_PPM

#endif // USE_RX_NRF24


#define USE_ADC
#define ADC_CHANNEL_1_PIN               PA0
#define ADC_CHANNEL_2_PIN               PB1

#ifdef CC3D_PPM1
#define ADC_CHANNEL_3_PIN               PA1
#else
#define ADC_CHANNEL_3_PIN               PB0
#endif

#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

// LED strip is on PWM5 output pin
//#define LED_STRIP
#define WS2811_PIN                      PB4
#define WS2811_TIMER                    TIM3
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC6
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH6_HANDLER

#define SPEKTRUM_BIND
// USART3, PB11 (Flexport)
#define BIND_PIN                PB11

//#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// #define USE_RANGEFINDER
//#define USE_RANGEFINDER_HCSR04
//#define USE_RANGEFINDER_SRF10
#define RANGEFINDER_HCSR04_ECHO_PIN          PB0
#define RANGEFINDER_HCSR04_TRIGGER_PIN       PB5

//#define NAV_AUTO_MAG_DECLINATION
//#define NAV_GPS_GLITCH_DETECTION
#undef BLACKBOX
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define DISABLE_UNCOMMON_MIXERS

#ifdef USE_RX_NRF24
#undef USE_RX_PWM
#undef USE_RX_PPM
#undef SERIAL_RX
#undef SPEKTRUM_BIND
#endif

#define TARGET_MOTOR_COUNT 4

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    11

// DEBUG
//#define USE_ASSERT          // include assertion support code
//#define USE_ASSERT_FULL     // Provide file information
//#define USE_ASSERT_STOP   // stop on failed assertion
//#define USE_ASSERT_CHECK    // include assertion check code (should in general a per-file define)

//#define HIL
//#define USE_FAKE_MAG
//#define USE_FAKE_BARO
//#define USE_FAKE_GPS

//#undef TELEMETRY_FRSKY
//#define TELEMETRY_MAVLINK
//#define USE_SERIALRX_SUMD
//#define TELEMETRY_HOTT

// CC3D is widely used for airplanes - enable fw_autotune
#define AUTOTUNE_FIXED_WING

// IO - from schematics
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         ( BIT(14) )

#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
