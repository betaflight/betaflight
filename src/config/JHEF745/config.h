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

#define FC_TARGET_MCU     STM32F745

#define BOARD_NAME        JHEF745
#define MANUFACTURER_ID   JHEF

#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000
#define USE_BARO_BMP280
#define USE_MAX7456
#define USE_FLASH_M25P16

#define BEEPER_PIN           PD15
#define MOTOR1_PIN           PB0
#define MOTOR2_PIN           PB1
#define MOTOR3_PIN           PE9
#define MOTOR4_PIN           PE11
#define MOTOR5_PIN           PC9
#define MOTOR6_PIN           PA3
#define RX_PPM_PIN           PE13
#define LED_STRIP_PIN        PD12
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PD5
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PA0
#define UART6_TX_PIN         PC6
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PD6
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PA1
#define UART6_RX_PIN         PC7
#define UART7_RX_PIN         PE7
#define UART7_TX_PIN         PE8
#define I2C1_SCL_PIN         PB6
#define I2C1_SDA_PIN         PB7
#define LED0_PIN             PA2
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI4_SCK_PIN         PE2
#define SPI1_MISO_PIN        PA6
#define SPI2_MISO_PIN        PB14
#define SPI4_MISO_PIN        PE5
#define SPI1_MOSI_PIN        PA7
#define SPI2_MOSI_PIN        PB15
#define SPI4_MOSI_PIN        PE6
#define CAMERA_CONTROL_PIN   PB3
#define ADC_VBAT_PIN         PC3
#define ADC_RSSI_PIN         PC5
#define ADC_CURR_PIN         PC2
#define FLASH_CS_PIN         PA4
#define MAX7456_SPI_CS_PIN   PB12
#define GYRO_1_EXTI_PIN      PE1
#define GYRO_1_CS_PIN        PE4
#define USB_DETECT_PIN       PA8

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PE13, 1,  1) \
    TIMER_PIN_MAP( 1, PB0 , 2,  0) \
    TIMER_PIN_MAP( 2, PB1 , 2,  0) \
    TIMER_PIN_MAP( 3, PE9 , 1,  2) \
    TIMER_PIN_MAP( 4, PE11, 1,  1) \
    TIMER_PIN_MAP( 5, PC9 , 2,  0) \
    TIMER_PIN_MAP( 6, PA3 , 2,  0) \
    TIMER_PIN_MAP( 7, PD12, 1,  0) \
    TIMER_PIN_MAP( 8, PB3 , 1,  0) \



#define ADC1_DMA_OPT        1

//TODO #define MAG_BUSTYPE I2C
#define MAG_I2C_INSTANCE (I2CDEV_1)
//TODO #define BARO_BUSTYPE I2C
#define BARO_I2C_INSTANCE (I2CDEV_1)
//TODO #define SERIALRX_PROVIDER SBUS
//TODO #define BLACKBOX_DEVICE SPIFLASH
//TODO #define CURRENT_METER ADC
//TODO #define BATTERY_METER ADC
//TODO #define IBATA_SCALE 275
#define BEEPER_INVERTED
//TODO #define BEEPER_OD OFF
//TODO #define OSD_VBAT_POS 14570
//TODO #define OSD_AH_SBAR_POS 14542
//TODO #define OSD_AH_POS 14414
#define MAX7456_SPI_INSTANCE SPI2
//TODO #define DASHBOARD_I2C_BUS 1
#define FLASH_SPI_INSTANCE SPI1
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI4
#define GYRO_1_ALIGN CW270_DEG
