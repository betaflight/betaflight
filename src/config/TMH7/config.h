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

#define FC_TARGET_MCU     STM32H743

#define BOARD_NAME        TMH7
#define MANUFACTURER_ID   TMTR

#define USE_ACCGYRO_BMI270
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_FLASH_M25P16
#define USE_MAX7456

#define BEEPER_PIN           PE3
#define MOTOR1_PIN           PA0
#define MOTOR2_PIN           PA1
#define MOTOR3_PIN           PA2
#define MOTOR4_PIN           PA3
#define MOTOR5_PIN           PB0
#define MOTOR6_PIN           PB1
#define MOTOR7_PIN           PC8
#define MOTOR8_PIN           PC9
#define RX_PPM_PIN           PB9
#define LED_STRIP_PIN        PA8
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PD5
#define UART3_TX_PIN         PD8
#define UART4_TX_PIN         PD1
#define UART5_TX_PIN         PC12
#define UART6_TX_PIN         PC6
#define UART7_TX_PIN         PE8
#define UART8_TX_PIN         PE1
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PD6
#define UART3_RX_PIN         PD9
#define UART4_RX_PIN         PD0
#define UART5_RX_PIN         PD2
#define UART6_RX_PIN         PC7
#define UART7_RX_PIN         PE7
#define UART8_RX_PIN         PE0
#define I2C1_SCL_PIN         PB6
#define I2C1_SDA_PIN         PB7
#define LED1_PIN             PE5
#define LED2_PIN             PE4
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PC10
#define SPI4_SCK_PIN         PE12
#define SPI1_MISO_PIN        PA6
#define SPI2_MISO_PIN        PB14
#define SPI3_MISO_PIN        PC11
#define SPI4_MISO_PIN        PE13
#define SPI1_MOSI_PIN        PA7
#define SPI2_MOSI_PIN        PB15
#define SPI3_MOSI_PIN        PB2
#define SPI4_MOSI_PIN        PE14
#define CAMERA_CONTROL_PIN   PB8
#define ADC_VBAT_PIN         PC1
#define ADC_RSSI_PIN         PC2
#define ADC_CURR_PIN         PC3
#define ADC_EXTERNAL1_PIN    PC0
#define FLASH_CS_PIN         PA15
#define MAX7456_SPI_CS_PIN   PB12
#define GYRO_1_EXTI_PIN      PC4
#define GYRO_2_EXTI_PIN      PE15
#define GYRO_1_CS_PIN        PA4
#define GYRO_2_CS_PIN        PE11

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA8 , 1,  0) \
    TIMER_PIN_MAP( 1, PB8 , 1,  0) \
    TIMER_PIN_MAP( 2, PB9 , 1,  0) \
    TIMER_PIN_MAP( 3, PA0 , 2,  0) \
    TIMER_PIN_MAP( 4, PA1 , 2,  0) \
    TIMER_PIN_MAP( 5, PA2 , 2,  0) \
    TIMER_PIN_MAP( 6, PA3 , 2,  0) \
    TIMER_PIN_MAP( 7, PB0 , 2,  0) \
    TIMER_PIN_MAP( 8, PB1 , 2,  0) \
    TIMER_PIN_MAP( 9, PC8 , 2,  0) \
    TIMER_PIN_MAP(10, PC9 , 2,  0) \



#define ADC1_DMA_OPT        8
#define ADC3_DMA_OPT        9
#define TIMUP1_DMA_OPT      0
#define TIMUP3_DMA_OPT      0
#define TIMUP5_DMA_OPT      0
#define TIMUP8_DMA_OPT      0

#define MAG_I2C_INSTANCE I2C1
#define BARO_I2C_INSTANCE I2C1
#define BEEPER_INVERTED
#define MAX7456_SPI_INSTANCE SPI2
#define FLASH_SPI_INSTANCE SPI3
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW270_DEG
#define GYRO_1_ALIGN_YAW 1800
#define GYRO_2_SPI_INSTANCE SPI4
#define GYRO_2_ALIGN CW270_DEG
#define GYRO_2_ALIGN_YAW 2700
