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

/*
   This file has been auto generated from unified-targets repo.

   The auto generation is transitional only, please ensure you update unified targets and not this file until the transition has complete.
*/

#define FC_TARGET_MCU     STM32F411

#define BOARD_NAME        NOX
#define MANUFACTURER_ID   AIRB

#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500
#define USE_BARO_SPI_BMP280
#define USE_FLASH_M25P16
#define USE_MAX7456

#define BEEPER_PIN           PC13
#define MOTOR1_PIN           PA1
#define MOTOR2_PIN           PA7
#define MOTOR3_PIN           PB8
#define MOTOR4_PIN           PB1
#define RX_PPM_PIN           PB10
#define LED_STRIP_PIN        PA0
#define UART1_TX_PIN         PB6
#define UART2_TX_PIN         PA2
#define UART1_RX_PIN         PB7
#define UART2_RX_PIN         PA3
#define INVERTER2_PIN        PC14
#define LED1_PIN             PA4
#define SPI1_SCK_PIN         PB3
#define SPI2_SCK_PIN         PB13
#define SPI1_MISO_PIN        PB4
#define SPI2_MISO_PIN        PB14
#define SPI1_MOSI_PIN        PB5
#define SPI2_MOSI_PIN        PB15
#define ESCSERIAL_PIN        PB10
#define ADC_BATT_PIN         PA5
#define BARO_CS_PIN          PA9
#define FLASH_CS_PIN         PA15
#define OSD_CS_PIN           PA10
#define GYRO_1_EXTI_PIN      PNONE
#define GYRO_1_CS_PIN        PB12

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB10, 1,  0) \
    TIMER_PIN_MAP( 1, PA1 , 2,  0) \
    TIMER_PIN_MAP( 2, PA7 , 1,  0) \
    TIMER_PIN_MAP( 3, PB8 , 1,  0) \
    TIMER_PIN_MAP( 4, PB1 , 2,  0) \
    TIMER_PIN_MAP( 5, PA0 , 1,  0) \
    TIMER_PIN_MAP( 6, PA2 , 3, -1) \
    TIMER_PIN_MAP( 7, PA3 , 3, -1) \



#define ADC1_DMA_OPT        1

#define BARO_SPI_INSTANCE SPI2
#define BEEPER_INVERTED
#define MAX7456_SPI_INSTANCE SPI2
#define FLASH_SPI_INSTANCE SPI1
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI2
