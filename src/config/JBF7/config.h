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

#define BOARD_NAME        JBF7
#define MANUFACTURER_ID   IFRC

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_FLASH
#define USE_FLASH_M25P16
#define USE_BARO_BMP280
#define USE_MAX7456
#define USE_SDCARD

#define BEEPER_PIN           PC15
#define MOTOR1_PIN           PC8
#define MOTOR2_PIN           PC6
#define MOTOR3_PIN           PC9
#define MOTOR4_PIN           PC7
#define MOTOR5_PIN           PB6
#define MOTOR6_PIN           PB7
#define MOTOR7_PIN           PB1
#define MOTOR8_PIN           PB0
#define RX_PPM_PIN           PA3
#define LED_STRIP_PIN        PA1
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PC10
#define UART5_TX_PIN         PC12
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PC11
#define UART5_RX_PIN         PD2
#define I2C2_SCL_PIN         PB10
#define I2C2_SDA_PIN         PB11
#define LED0_PIN             PC4
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PB3
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI3_SDI_PIN         PB4
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define SPI3_SDO_PIN         PB5
#define CAMERA_CONTROL_PIN   PA0
#define ADC_VBAT_PIN         PC1
#define ADC_RSSI_PIN         PC0
#define ADC_CURR_PIN         PC2
#define SDCARD_SPI_CS_PIN    PA4
#define PINIO1_PIN           PC13
#define PINIO2_PIN           PC14
#define PINIO3_PIN           PB8
#define FLASH_CS_PIN         PB9
#define MAX7456_SPI_CS_PIN   PB12
#define GYRO_1_EXTI_PIN      PA8
#define GYRO_2_EXTI_PIN      PB2
#define GYRO_1_CS_PIN        PA15
#define GYRO_2_CS_PIN        PC3

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA0 , 2,  0) \
    TIMER_PIN_MAP( 1, PA3 , 3, -1) \
    TIMER_PIN_MAP( 2, PC8 , 2,  1) \
    TIMER_PIN_MAP( 3, PC6 , 2,  0) \
    TIMER_PIN_MAP( 4, PC9 , 2,  0) \
    TIMER_PIN_MAP( 5, PC7 , 2,  1) \
    TIMER_PIN_MAP( 6, PB6 , 1,  0) \
    TIMER_PIN_MAP( 7, PB7 , 1,  0) \
    TIMER_PIN_MAP( 8, PB1 , 2,  0) \
    TIMER_PIN_MAP( 9, PB0 , 2,  0) \
    TIMER_PIN_MAP(10, PA1 , 1,  0)


#define ADC3_DMA_OPT        0

#define DEFAULT_GYRO_TO_USE GYRO_CONFIG_USE_GYRO_BOTH
#define MAG_I2C_INSTANCE (I2CDEV_2)
#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_2)

#define USE_ADC
#define ADC_INSTANCE ADC3
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ESC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE 110
#define DEFAULT_CURRENT_METER_SCALE 100
#define BEEPER_INVERTED
#define USE_SDCARD_SPI
#define SDCARD_SPI_INSTANCE SPI3
//TODO #define OSD_VBAT_POS 384
//TODO #define OSD_RSSI_POS 2426
//TODO #define OSD_TIM_1_POS 53
//TODO #define OSD_TIM_2_POS 2455
//TODO #define OSD_FLYMODE_POS 2420
//TODO #define OSD_THROTTLE_POS 2400
//TODO #define OSD_VTX_CHANNEL_POS 34
//TODO #define OSD_CROSSHAIRS_POS 236
//TODO #define OSD_AH_POS 199
//TODO #define OSD_CURRENT_POS 2440
//TODO #define OSD_MAH_DRAWN_POS 2448
//TODO #define OSD_CRAFT_NAME_POS 2048
//TODO #define OSD_DISPLAY_NAME_POS 490
//TODO #define OSD_GPS_SPEED_POS 271
//TODO #define OSD_GPS_LON_POS 82
//TODO #define OSD_GPS_LAT_POS 65
//TODO #define OSD_GPS_SATS_POS 1410
//TODO #define OSD_HOME_DIR_POS 302
//TODO #define OSD_HOME_DIST_POS 303
//TODO #define OSD_COMPASS_BAR_POS 265
//TODO #define OSD_ALTITUDE_POS 246
//TODO #define OSD_PID_ROLL_POS 135
//TODO #define OSD_PID_PITCH_POS 167
//TODO #define OSD_PID_YAW_POS 199
//TODO #define OSD_DEBUG_POS 0
//TODO #define OSD_POWER_POS 320
//TODO #define OSD_PIDRATE_PROFILE_POS 344
//TODO #define OSD_WARNINGS_POS 2406
//TODO #define OSD_AVG_CELL_VOLTAGE_POS 2432
//TODO #define OSD_PIT_ANG_POS 256
//TODO #define OSD_ROL_ANG_POS 288
//TODO #define OSD_BATTERY_USAGE_POS 391
//TODO #define OSD_DISARMED_POS 75
//TODO #define OSD_NHEADING_POS 310
//TODO #define OSD_NVARIO_POS 278
//TODO #define OSD_ESC_TMP_POS 82
//TODO #define OSD_ESC_RPM_POS 83
//TODO #define OSD_STAT_MAX_SPD OFF
#define MAX7456_SPI_INSTANCE SPI2
#define PINIO1_CONFIG 129
#define PINIO1_BOX 0
#define FLASH_SPI_INSTANCE SPI3
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_2_SPI_INSTANCE SPI1
#define GYRO_2_ALIGN CW90_DEG
#define GYRO_2_ALIGN_YAW 900
