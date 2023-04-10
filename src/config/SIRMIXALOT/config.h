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

#pragma once

#define FC_TARGET_MCU     STM32F7X2

#define BOARD_NAME        SIRMIXALOT
#define MANUFACTURER_ID   CUST

#define BEEPER_PIN           PC11
#define MOTOR1_PIN           PB6
#define MOTOR2_PIN           PB7
#define MOTOR3_PIN           PB8
#define MOTOR4_PIN           PA15
#define LED_STRIP_PIN        PB1
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART5_TX_PIN         PC12
#define UART6_TX_PIN         PC6
#define UART11_TX_PIN        PA0
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART4_RX_PIN         PA1
#define UART5_RX_PIN         PD2
#define UART6_RX_PIN         PC7
#define UART11_RX_PIN        PB11
#define I2C1_SCL_PIN         PA8
#define I2C1_SDA_PIN         PC9
#define LED0_PIN             PB12
#define LED1_PIN             PC8
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PB3
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI3_SDI_PIN         PB4
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define SPI3_SDO_PIN         PB5
#define CAMERA_CONTROL_PIN   PC10
#define ADC_VBAT_PIN         PC2
#define ADC_RSSI_PIN         PC3
#define ADC_CURR_PIN         PC1
#define BARO_CS_PIN          PB9
#define FLASH_CS_PIN         PC0
#define MAX7456_SPI_CS_PIN   PC14
#define GYRO_1_EXTI_PIN      PC4
#define GYRO_1_CS_PIN        PA4
#define USB_DETECT_PIN       PB2

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB6 , 1,  0) \
    TIMER_PIN_MAP( 1, PB7 , 1,  0) \
    TIMER_PIN_MAP( 2, PB8 , 1,  0) \
    TIMER_PIN_MAP( 3, PB1 , 2,  0) \
    TIMER_PIN_MAP( 4, PA15, 1,  0)


#define ADC1_DMA_OPT        1

#define USE_BARO
#define BARO_SPI_INSTANCE SPI3
//TODO #define SERIALRX_PROVIDER FPORT
//TODO #define SERIALRX_INVERTED ON
//TODO #define SERIALRX_HALFDUPLEX ON
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
//TODO #define DSHOT_BIDIR OFF
//TODO #define MOTOR_PWM_PROTOCOL DSHOT600
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE 80
#define DEFAULT_CURRENT_METER_SCALE 210
#define BEEPER_INVERTED
#define DEFAULT_PID_PROCESS_DENOM 1
//TODO #define LEDSTRIP_VISUAL_BEEPER ON
//TODO #define OSD_VBAT_POS 2403
//TODO #define OSD_RSSI_POS 2436
//TODO #define OSD_RSSI_DBM_POS 66
//TODO #define OSD_TIM_1_POS 2071
//TODO #define OSD_TIM_2_POS 2103
//TODO #define OSD_FLYMODE_POS 2424
//TODO #define OSD_VTX_CHANNEL_POS 480
//TODO #define OSD_CRAFT_NAME_POS 2058
//TODO #define OSD_GPS_SPEED_POS 2359
//TODO #define OSD_GPS_SATS_POS 2392
//TODO #define OSD_HOME_DIR_POS 2095
//TODO #define OSD_ALTITUDE_POS 2135
//TODO #define OSD_WARNINGS_POS 14698
//TODO #define OSD_DISARMED_POS 2250
//TODO #define VTX_BAND 1
//TODO #define VTX_CHANNEL 3
//TODO #define VTX_POWER 1
//TODO #define VTX_FREQ 5825
#define MAX7456_SPI_INSTANCE SPI3
//TODO #define LED_INVERSION 3
#define FLASH_SPI_INSTANCE SPI2
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW180_DEG
#define GYRO_1_ALIGN_YAW 1800
