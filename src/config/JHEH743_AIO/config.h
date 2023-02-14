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

   The auto generation is transitional only.
*/

#define FC_TARGET_MCU     STM32H743

#define BOARD_NAME        JHEH743_AIO
#define MANUFACTURER_ID   JHEF

#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000
#define USE_BARO_BMP280
#define USE_MAX7456
#define USE_FLASH_M25P16

#define BEEPER_PIN           PD15
#define MOTOR1_PIN           PE9
#define MOTOR2_PIN           PB0
#define MOTOR3_PIN           PE11
#define MOTOR4_PIN           PB1
#define RX_PPM_PIN           PA3
#define LED_STRIP_PIN        PD12
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PA0
#define UART6_TX_PIN         PC6
#define UART7_TX_PIN         PE8
#define UART8_TX_PIN         PE1
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PA1
#define UART6_RX_PIN         PC7
#define UART7_RX_PIN         PE7
#define UART8_RX_PIN         PE0
#define I2C1_SCL_PIN         PB8
#define I2C2_SCL_PIN         PB10
#define I2C1_SDA_PIN         PB9
#define I2C2_SDA_PIN         PB11
#define LED1_PIN             PC13
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PC10
#define SPI4_SCK_PIN         PE2
#define SPI1_MISO_PIN        PA6
#define SPI2_MISO_PIN        PB14
#define SPI3_MISO_PIN        PC11
#define SPI4_MISO_PIN        PE5
#define SPI1_MOSI_PIN        PA7
#define SPI2_MOSI_PIN        PB15
#define SPI3_MOSI_PIN        PC12
#define SPI4_MOSI_PIN        PE6
#define CAMERA_CONTROL_PIN   PC8
#define ADC_BATT_PIN         PC3
#define ADC_RSSI_PIN         PC5
#define ADC_CURR_PIN         PC2
#define ADC_EXT_PIN          PC1
#define FLASH_CS_PIN         PA15
#define OSD_CS_PIN           PE4
#define GYRO_1_EXTI_PIN      PD0
#define GYRO_2_EXTI_PIN      PD8
#define GYRO_1_CS_PIN        PA4
#define GYRO_2_CS_PIN        PB12

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA8 , 1, 10) \
    TIMER_PIN_MAP( 1, PB3 , 1,  0) \
    TIMER_PIN_MAP( 2, PB0 , 2,  0) \
    TIMER_PIN_MAP( 3, PB1 , 2,  1) \
    TIMER_PIN_MAP( 4, PD12, 1, 10) \
    TIMER_PIN_MAP( 5, PD13, 1,  5) \
    TIMER_PIN_MAP( 6, PC8 , 2,  0) \
    TIMER_PIN_MAP( 7, PC9 , 2,  7) \
    TIMER_PIN_MAP( 8, PA0 , 2,  0) \
    TIMER_PIN_MAP( 9, PA1 , 2,  0) \
    TIMER_PIN_MAP(10, PA2 , 2,  0) \
    TIMER_PIN_MAP(11, PA3 , 1,  0) \
    TIMER_PIN_MAP(12, PB10, 1,  0) \
    TIMER_PIN_MAP(13, PB11, 1,  0) \
    TIMER_PIN_MAP(14, PE9 , 1,  2) \
    TIMER_PIN_MAP(15, PE11, 1,  3) \



#define ADC1_DMA_OPT        8
#define ADC3_DMA_OPT        9
#define TIMUP1_DMA_OPT      0
#define TIMUP2_DMA_OPT      0
#define TIMUP3_DMA_OPT      0
#define TIMUP4_DMA_OPT      0
#define TIMUP8_DMA_OPT      0

#define MAG_I2C_INSTANCE I2C1
#define BARO_I2C_INSTANCE I2C1
#define BEEPER_INVERTED
#define MAX7456_SPI_INSTANCE SPI4
#define FLASH_SPI_INSTANCE SPI3
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_2_SPI_INSTANCE SPI2
