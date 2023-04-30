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

#define FC_TARGET_MCU     AT32F435M

#define BOARD_NAME        ATSTARTF435
#define MANUFACTURER_ID   ATRY

// AT-START-F435 V1.0 LED assignments to use as a default
#define LED0_PIN                PD13 // Labelled LED2 Red
#define LED1_PIN                PD14 // Labelled LED3 Amber
#define LED2_PIN                PD15 // Labelled LED4 Green

// AT-START-F435 J7 connector SPI 2
#define SPI2_SCK_PIN            PD1
#define SPI2_SDI_PIN            PC2
#define SPI2_SDO_PIN            PD4

#define J7_NSS                  PD0

#define GYRO_1_CS_PIN           J7_NSS
#define GYRO_1_SPI_INSTANCE     SPI2

#define USE_EXTI
#define USE_GYRO
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PB11

#define USE_ACC
#define USE_ACCGYRO_BMI160

// These would be useful for connecting configurator via the at-link,
// but don't seem to be active when defined here
#define UART1_RX_PIN PA10
#define UART1_TX_PIN PA9
#define USE_MSP_UART SERIAL_PORT_USART1

#define ADC1_DMA_OPT                    1

