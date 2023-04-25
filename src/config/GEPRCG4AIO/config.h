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

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_ICM42688P
#define USE_BARO
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

#define BOARD_NAME GEPRCG4AIO 
#define MANUFACTURER_ID GEPR

#define BEEPER_PIN PA15
#define MOTOR1_PIN PA0
#define MOTOR2_PIN PA1
#define MOTOR3_PIN PA2
#define MOTOR4_PIN PA3
#define LED_STRIP_PIN PB6
#define UART1_TX_PIN PA9
#define UART2_TX_PIN PB3
#define UART3_TX_PIN PB9
#define UART4_TX_PIN PC10
#define UART1_TX_PIN1 PB10
#define UART1_RX_PIN PA10
#define UART2_RX_PIN PB4
#define UART4_RX_PIN PC11
#define UART1_RX_PIN1 PB11
#define I2C1_SCL_PIN PA13
#define I2C1_SDA_PIN PA14
#define LED0_PIN PB7
#define SPI1_SCK_PIN PA5
#define SPI2_SCK_PIN PB13
#define SPI3_SCK_PIN PB3
#define SPI1_SDI_PIN PA6
#define SPI2_SDI_PIN PB14
#define SPI3_SDI_PIN PB4
#define SPI1_SDO_PIN PA7
#define SPI2_SDO_PIN PB15
#define SPI3_SDO_PIN PB5
#define ADC_VBAT_PIN PB2
#define ADC_CURR_PIN PB1
#define FLASH_CS_PIN PC6
#define MAX7456_SPI_CS_PIN PA8
#define GYRO_1_EXTI_PIN PA4
#define GYRO_1_CS_PIN PB0

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA0 , 1,  1) \
    TIMER_PIN_MAP( 1, PA1 , 1,  2) \
    TIMER_PIN_MAP( 2, PA2 , 1,  3) \
    TIMER_PIN_MAP( 3, PA3 , 1,  4) \
    TIMER_PIN_MAP( 4, PB6 , 1,  9)



#define ADC1_DMA_OPT 1
#define ADC2_DMA_OPT 8
#define TIMUP2_DMA_OPT 0
#define TIMUP8_DMA_OPT 0

#define DEFAULT_GYRO_TO_USE GYRO_CONFIG_USE_GYRO_1
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_FLASH
#define MAX7456_SPI_INSTANCE SPI2
#define BEEPER_INVERTED
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 120
#define FLASH_SPI_INSTANCE SPI2 
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW270_DEG
