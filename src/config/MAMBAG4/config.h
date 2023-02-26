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

#define FC_TARGET_MCU     STM32G47X

#define BOARD_NAME        MAMBAG4
#define MANUFACTURER_ID   DIAT

#define USE_ACC_SPI_MPU6000
#define USE_BARO_DPS310
#define USE_GYRO_SPI_MPU6000
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

#define BEEPER_PIN           PA15
#define MOTOR1_PIN           PA0
#define MOTOR2_PIN           PA1
#define MOTOR3_PIN           PA2
#define MOTOR4_PIN           PA3
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PB3
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PC10
#define UART11_TX_PIN        PB9
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PB4
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PC11
#define I2C1_SCL_PIN         PA13
#define I2C1_SDA_PIN         PA14
#define LED0_PIN             PC15
#define LED1_PIN             PC14
#define ADC_VBAT_PIN         PB2
#define ADC_CURR_PIN         PB1
#define FLASH_CS_PIN         PC6
#define MAX7456_SPI_CS_PIN   PA8
#define GYRO_1_EXTI_PIN      PC4
#define GYRO_2_EXTI_PIN      PB12
#define GYRO_1_CS_PIN        PA4
#define GYRO_2_CS_PIN        PB0

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA0 , 1,  1) \
    TIMER_PIN_MAP( 1, PA1 , 1,  2) \
    TIMER_PIN_MAP( 2, PA2 , 1,  3) \
    TIMER_PIN_MAP( 3, PA3 , 1,  4) \
    TIMER_PIN_MAP( 4, PB6 , 3,  9) \



#define ADC1_DMA_OPT        1
#define TIMUP2_DMA_OPT      0
#define TIMUP8_DMA_OPT      0

#define BARO_I2C_INSTANCE (I2CDEV_1)
#define BEEPER_INVERTED
#define MAX7456_SPI_INSTANCE SPI2
#define FLASH_SPI_INSTANCE SPI2
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW180_DEG
#define GYRO_1_ALIGN_YAW 1800
#define GYRO_2_SPI_INSTANCE SPI1
#define GYRO_2_ALIGN CW270_DEG
#define GYRO_2_ALIGN_YAW 2700
