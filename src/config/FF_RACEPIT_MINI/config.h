/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#define FC_TARGET_MCU     STM32F405

#define BOARD_NAME        FF_RACEPIT_MINI
#define MANUFACTURER_ID   FFPV

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

#define LED0_PIN             PB9
#define LED1_PIN             PB8
#define BEEPER_PIN           PC3
#define BEEPER_INVERTED
#define PINIO1_PIN           PC0
#define PINIO2_PIN           PC8

#define I2C3_SCL_PIN         PA8
#define I2C3_SDA_PIN         PC9
//TODO #define I2C3_PULLUP ON

#define SPI1_SCK_PIN         PA5
#define SPI1_MISO_PIN        PA6
#define SPI1_MOSI_PIN        PA7

#define SPI2_SCK_PIN         PB13
#define SPI2_MISO_PIN        PB14
#define SPI2_MOSI_PIN        PB15

#define SPI3_SCK_PIN         PB3
#define SPI3_MISO_PIN        PB4
#define SPI3_MOSI_PIN        PB5

#define GYRO_1_CS_PIN        PA4
#define GYRO_1_EXTI_PIN      PC4
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW180_DEG

#define FLASH_SPI_INSTANCE SPI3
#define FLASH_CS_PIN         PA15

#define MAX7456_SPI_CS_PIN   PB12
#define MAX7456_SPI_INSTANCE SPI2

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB0 , 2, 0 ) \
    TIMER_PIN_MAP( 1, PB1 , 2, 0 ) \
    TIMER_PIN_MAP( 2, PB11, 1, 1 ) \
    TIMER_PIN_MAP( 3, PB10, 1, 0 ) \
    TIMER_PIN_MAP( 4, PA10, 1, -1) \
    TIMER_PIN_MAP( 5, PB6 , 1, 0 )


#define MOTOR1_PIN           PB0
#define MOTOR2_PIN           PB1
#define MOTOR3_PIN           PB11
#define MOTOR4_PIN           PB10
#define LED_STRIP_PIN        PB6

#define ADC2_DMA_OPT        1

#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
//TODO #define MOTOR_PWM_PROTOCOL Dshot600

#define UART1_TX_PIN         PA9
#define UART1_RX_PIN         PB7

#define UART2_TX_PIN         PA2
#define UART2_RX_PIN         PA3

#define UART3_TX_PIN         PC10
#define UART3_RX_PIN         PC11
#define INVERTER3_PIN        PC15

#define UART4_TX_PIN         PA0
#define UART4_RX_PIN         PA1

#define UART5_TX_PIN         PC12
#define UART5_RX_PIN         PD2

#define UART6_TX_PIN         PC6
#define UART6_RX_PIN         PC7

#define ADC_VBAT_PIN         PC2
#define ADC_CURR_PIN         PC1

#define ESCSERIAL_PIN        PB0

//TODO #define PINIO_CONFIG 1,129,1,1
//TODO #define PINIO_BOX 40,41,255,255
//TODO #define TLM_HALFDUPLEX OFF
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#define USE_ADC
#define ADC_INSTANCE ADC2
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
