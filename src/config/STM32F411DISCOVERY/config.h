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

#define BOARD_NAME        STM32F411DISCOVERY
#define MANUFACTURER_ID   STMI

#define USE_ACC_LSM303DLHC
#define USE_GYRO_L3GD20
#define MPU_I2C_INSTANCE I2CDEV_1

#define BEEPER_PIN           PD12
#define MOTOR1_PIN           PB1
#define MOTOR2_PIN           PB0
#define MOTOR3_PIN           PA2
#define MOTOR4_PIN           PA3
#define MOTOR5_PIN           PA10
#define MOTOR6_PIN           PA8
#define RX_PPM_PIN           PB8
#define LED_STRIP_PIN        PB8
#define UART1_TX_PIN         PA15
#define UART2_TX_PIN         PA2
#define UART6_TX_PIN         PC6
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART6_RX_PIN         PC7
#define I2C1_SCL_PIN         PB6
#define I2C1_SDA_PIN         PB9
#define LED1_PIN             PD15
#define LED2_PIN             PD13
#define SPI1_SCK_PIN         PA5
#define SPI1_MISO_PIN        PA6
#define SPI1_MOSI_PIN        PA7
#define ESCSERIAL_PIN        PB8
#define ADC_BATT_PIN         PC1
#define ADC_CURR_PIN         PC2
#define GYRO_1_EXTI_PIN      PE1
#define GYRO_1_CS_PIN        PE3
#define USB_DETECT_PIN       PA9

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB8 , 1,  0) \
    TIMER_PIN_MAP( 1, PD12, 1,  0) \
    TIMER_PIN_MAP( 2, PB1 , 2,  0) \
    TIMER_PIN_MAP( 3, PB0 , 2,  0) \
    TIMER_PIN_MAP( 4, PA2 , 1,  0) \
    TIMER_PIN_MAP( 5, PA3 , 1,  1) \
    TIMER_PIN_MAP( 6, PA10, 1,  1) \
    TIMER_PIN_MAP( 7, PA8 , 1,  1) \



#define ADC1_DMA_OPT        1

#define MAG_I2C_INSTANCE I2C1
#define BEEPER_PWM_HZ 2000
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW180_DEG
