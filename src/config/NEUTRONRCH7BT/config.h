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

   The auto generation is transitional only, please remove this comment once the file is edited.
*/

#define FC_TARGET_MCU     STM32H743

#define BOARD_NAME        NEUTRONRCH7BT
#define MANUFACTURER_ID   NERC

#define USE_ACCGYRO_BMI270
#define USE_FLASH_W25M02G
#define USE_MAX7456

#define BEEPER_PIN           PA15
#define MOTOR1_PIN           PB0
#define MOTOR2_PIN           PB1
#define MOTOR3_PIN           PA0
#define MOTOR4_PIN           PA1
#define MOTOR5_PIN           PA2
#define MOTOR6_PIN           PA3
#define MOTOR7_PIN           PD12
#define MOTOR8_PIN           PD13
#define LED_STRIP_PIN        PA8
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PD5
#define UART3_TX_PIN         PD8
#define UART4_TX_PIN         PB9
#define UART6_TX_PIN         PC6
#define UART7_TX_PIN         PE8
#define UART8_TX_PIN         PE1
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PD6
#define UART3_RX_PIN         PD9
#define UART4_RX_PIN         PB8
#define UART6_RX_PIN         PC7
#define UART7_RX_PIN         PE7
#define UART8_RX_PIN         PE0
#define I2C1_SCL_PIN         PB6
#define I2C2_SCL_PIN         PB10
#define I2C1_SDA_PIN         PB7
#define I2C2_SDA_PIN         PB11
#define LED0_PIN             PE3
#define LED1_PIN             PE4
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PB3
#define SPI4_SCK_PIN         PE12
#define SPI1_MISO_PIN        PA6
#define SPI2_MISO_PIN        PB14
#define SPI3_MISO_PIN        PB4
#define SPI4_MISO_PIN        PE13
#define SPI1_MOSI_PIN        PD7
#define SPI2_MOSI_PIN        PB15
#define SPI3_MOSI_PIN        PB5
#define SPI4_MOSI_PIN        PE14
#define ADC_VBAT_PIN         PC0
#define ADC_RSSI_PIN         PC5
#define ADC_CURR_PIN         PC1
#define ADC_EXTERNAL1_PIN    PC4
#define SDIO_CK_PIN          PC12
#define SDIO_CMD_PIN         PD2
#define SDIO_D0_PIN          PC8
#define SDIO_D1_PIN          PC9
#define SDIO_D2_PIN          PC10
#define SDIO_D3_PIN          PC11
#define PINIO1_PIN           PC3
#define PINIO2_PIN           PD10
#define PINIO3_PIN           PD11
#define MAX7456_SPI_CS_PIN   PB12
#define GYRO_1_EXTI_PIN      PB2
#define GYRO_2_EXTI_PIN      PE15
#define GYRO_1_CS_PIN        PC15
#define GYRO_2_CS_PIN        PE11
#define USB_DETECT_PIN       PE2

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB0 , 2,  0) \
    TIMER_PIN_MAP( 1, PB1 , 2,  1) \
    TIMER_PIN_MAP( 2, PA0 , 2,  2) \
    TIMER_PIN_MAP( 3, PA1 , 2,  3) \
    TIMER_PIN_MAP( 4, PA2 , 2,  4) \
    TIMER_PIN_MAP( 5, PA3 , 2,  5) \
    TIMER_PIN_MAP( 6, PD12, 1,  6) \
    TIMER_PIN_MAP( 7, PD13, 1,  7) \
    TIMER_PIN_MAP( 8, PD14, 1, 12) \
    TIMER_PIN_MAP( 9, PD15, 1, -1) \
    TIMER_PIN_MAP(10, PE5 , 1,  0) \
    TIMER_PIN_MAP(11, PE6 , 1, -1) \
    TIMER_PIN_MAP(12, PA8 , 1, 14) \
    TIMER_PIN_MAP(13, PA15, 1,  0) \
    TIMER_PIN_MAP(14, PC7 , 2,  0) \
    TIMER_PIN_MAP(15, PC6 , 2,  0) \
    TIMER_PIN_MAP(16, PB8 , 1,  0) \
    TIMER_PIN_MAP(17, PB9 , 1,  0) \



#define ADC1_DMA_OPT        8
#define ADC3_DMA_OPT        9
#define TIMUP1_DMA_OPT      0
#define TIMUP2_DMA_OPT      0
#define TIMUP3_DMA_OPT      2
#define TIMUP4_DMA_OPT      1
#define TIMUP5_DMA_OPT      0
#define TIMUP8_DMA_OPT      0

//TODO #define GYRO_TO_USE BOTH
//TODO #define MAG_BUSTYPE I2C
#define MAG_I2C_INSTANCE (I2CDEV_1)
//TODO #define BARO_BUSTYPE I2C
#define BARO_I2C_INSTANCE (I2CDEV_2)
//TODO #define BLACKBOX_DEVICE SDCARD
//TODO #define CURRENT_METER ADC
//TODO #define BATTERY_METER ADC
#define BEEPER_INVERTED
//TODO #define BEEPER_OD OFF
//TODO #define SDIO_USE_4BIT_WIDTH ON
//TODO #define SDIO_DEVICE 1
#define MAX7456_SPI_INSTANCE SPI2
//TODO #define PINIO_CONFIG 129,1,1,1
//TODO #define PINIO_BOX 0,40,41,42
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW270_DEG
#define GYRO_1_ALIGN_YAW 2700
#define USE_SPI_GYRO
#define GYRO_2_SPI_INSTANCE SPI4
