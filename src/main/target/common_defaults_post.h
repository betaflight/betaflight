/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// pg/max7456

#ifndef DEBUG_MODE
#define DEBUG_MODE DEBUG_NONE
#endif

#ifndef I2C0_CLOCKSPEED
#define I2C0_CLOCKSPEED 800
#endif
#ifndef I2C1_CLOCKSPEED
#define I2C1_CLOCKSPEED 800
#endif
#ifndef I2C2_CLOCKSPEED
#define I2C2_CLOCKSPEED 800
#endif
#ifndef I2C3_CLOCKSPEED
#define I2C3_CLOCKSPEED 800
#endif
#ifndef I2C4_CLOCKSPEED
#define I2C4_CLOCKSPEED 800
#endif

// Default values for internal pullup

#if defined(USE_I2C_PULLUP)
#define I2C0_PULLUP true
#define I2C1_PULLUP true
#define I2C2_PULLUP true
#define I2C3_PULLUP true
#define I2C4_PULLUP true
#else
#define I2C0_PULLUP false
#define I2C1_PULLUP false
#define I2C2_PULLUP false
#define I2C3_PULLUP false
#define I2C4_PULLUP false
#endif

// Extracted from rx/rx.c and rx/rx.h

#define RX_MAPPABLE_CHANNEL_COUNT 8

#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif

#define RX_MIN_USEC 885
#define RX_MAX_USEC 2115
#define RX_MID_USEC 1500

#ifndef SPEKTRUM_BIND_PIN
#define SPEKTRUM_BIND_PIN NONE
#endif

#ifndef BINDPLUG_PIN
#define BINDPLUG_PIN NONE
#endif

// Previously there was logic here to default GYRO_1_CUSTOM_ALIGN and GYRO_2_CUSTOM_ALIGN
// to CUSTOM_ALIGN_CW0_DEG if they weren't defined in the target. The defaulting logic
// has been moved to pg/gyrodev.c to set the custom alignment based on the sensor alignment
// if a custom alignment is not applied in the target.

#ifdef USE_VCP
#ifndef USB_DETECT_PIN
#define USB_DETECT_PIN NONE
#endif
#ifndef USB_MSC_BUTTON_PIN
#define USB_MSC_BUTTON_PIN NONE
#endif
#if !defined(MSC_BUTTON_IPU)
#define MSC_BUTTON_IPU true
#endif
#endif

#ifdef USE_TIMER_MGMT
#ifndef MAX_TIMER_PINMAP_COUNT
#define MAX_TIMER_PINMAP_COUNT 21 // Largest known for F405RG (OMNINXT)
#endif
#endif

#ifndef DEFAULT_MIXER
#define DEFAULT_MIXER    MIXER_QUADX
#endif

#if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_HCSR04)
#ifndef RANGEFINDER_HCSR04_TRIGGER_PIN
#define RANGEFINDER_HCSR04_TRIGGER_PIN     NONE
#endif
#ifndef RANGEFINDER_HCSR04_ECHO_PIN
#define RANGEFINDER_HCSR04_ECHO_PIN        NONE
#endif
#endif

// Mag
#if defined(USE_MAG)
#ifndef MAG_SPI_INSTANCE
#define MAG_SPI_INSTANCE        NULL
#endif
#ifndef MAG_CS_PIN
#define MAG_CS_PIN              NONE
#endif
#ifndef MAG_I2C_INSTANCE
#define MAG_I2C_INSTANCE        I2C_DEVICE
#endif
#endif

#ifndef MAG_INT_EXTI
#define MAG_INT_EXTI            NONE
#endif

// Baro
#if defined(USE_BARO)
#ifndef BARO_SPI_INSTANCE
#define BARO_SPI_INSTANCE       NULL
#endif
#ifndef BARO_CS_PIN
#define BARO_CS_PIN             NONE
#endif
#ifndef BARO_I2C_INSTANCE
#define BARO_I2C_INSTANCE       I2C_DEVICE
#endif
#ifndef BARO_XCLR_PIN
#define BARO_XCLR_PIN           NONE
#endif
#endif

#ifdef USE_ADC

#if !defined(ADC1_DMA_OPT)
#define ADC1_DMA_OPT (DMA_OPT_UNUSED)
#endif
#if !defined(ADC2_DMA_OPT)
#define ADC2_DMA_OPT (DMA_OPT_UNUSED)
#endif
#if !defined(ADC3_DMA_OPT)
#define ADC3_DMA_OPT (DMA_OPT_UNUSED)
#endif
#if !defined(ADC4_DMA_OPT)
#define ADC4_DMA_OPT (DMA_OPT_UNUSED)
#endif
#if !defined(ADC5_DMA_OPT)
#define ADC5_DMA_OPT (DMA_OPT_UNUSED)
#endif

#endif // USE_ADC

#ifdef USE_SPI
#ifdef USE_SPI_DEVICE_0
#ifndef SPI0_TX_DMA_OPT
#define SPI0_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef SPI0_RX_DMA_OPT
#define SPI0_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#ifdef USE_SPI_DEVICE_1
#ifndef SPI1_TX_DMA_OPT
#define SPI1_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef SPI1_RX_DMA_OPT
#define SPI1_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#ifdef USE_SPI_DEVICE_2
#ifndef SPI2_TX_DMA_OPT
#define SPI2_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef SPI2_RX_DMA_OPT
#define SPI2_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#ifdef USE_SPI_DEVICE_3
#ifndef SPI3_TX_DMA_OPT
#define SPI3_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef SPI3_RX_DMA_OPT
#define SPI3_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#ifdef USE_SPI_DEVICE_4
#ifndef SPI4_TX_DMA_OPT
#define SPI4_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef SPI4_RX_DMA_OPT
#define SPI4_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#ifdef USE_SPI_DEVICE_5
#ifndef SPI5_TX_DMA_OPT
#define SPI5_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef SPI5_RX_DMA_OPT
#define SPI5_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#ifdef USE_SPI_DEVICE_6
#ifndef SPI6_TX_DMA_OPT
#define SPI6_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef SPI6_RX_DMA_OPT
#define SPI6_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#endif

#ifdef USE_UART0
#ifndef UART0_TX_DMA_OPT
#define UART0_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART0_RX_DMA_OPT
#define UART0_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART1
#ifndef UART1_TX_DMA_OPT
#define UART1_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART1_RX_DMA_OPT
#define UART1_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART2
#ifndef UART2_TX_DMA_OPT
#define UART2_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART2_RX_DMA_OPT
#define UART2_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART3
#ifndef UART3_TX_DMA_OPT
#define UART3_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART3_RX_DMA_OPT
#define UART3_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART4
#ifndef UART4_TX_DMA_OPT
#define UART4_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART4_RX_DMA_OPT
#define UART4_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART5
#ifndef UART5_TX_DMA_OPT
#define UART5_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART5_RX_DMA_OPT
#define UART5_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART6
#ifndef UART6_TX_DMA_OPT
#define UART6_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART6_RX_DMA_OPT
#define UART6_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART7
#ifndef UART7_TX_DMA_OPT
#define UART7_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART7_RX_DMA_OPT
#define UART7_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART8
#ifndef UART8_TX_DMA_OPT
#define UART8_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART8_RX_DMA_OPT
#define UART8_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART9
#ifndef UART9_TX_DMA_OPT
#define UART9_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART9_RX_DMA_OPT
#define UART9_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART10
#ifndef UART10_TX_DMA_OPT
#define UART10_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART10_RX_DMA_OPT
#define UART10_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifndef RTC6705_CS_PIN
#define RTC6705_CS_PIN NONE
#endif

#ifndef RTC6705_POWER_PIN
#define RTC6705_POWER_PIN NONE
#endif

#ifndef RTC6705_SPICLK_PIN
#define RTC6705_SPICLK_PIN NONE
#endif

#ifndef RTC6705_SPI_SDO_PIN
#define RTC6705_SPI_SDO_PIN NONE
#endif

#ifndef RTC6705_SPI_INSTANCE
#define RTC6705_SPI_INSTANCE NULL
#endif

#if defined(USE_QUAD_MIXER_ONLY)
#define MAX_SUPPORTED_MOTORS 4
#define MAX_SUPPORTED_SERVOS 1
#else
#ifndef MAX_SUPPORTED_MOTORS
#define MAX_SUPPORTED_MOTORS 8
#endif
#define MAX_SUPPORTED_SERVOS 8
#endif

#ifndef BOX_USER1_NAME
#define BOX_USER1_NAME "USER1"
#endif

#ifndef BOX_USER2_NAME
#define BOX_USER2_NAME "USER2"
#endif

#ifndef BOX_USER3_NAME
#define BOX_USER3_NAME "USER3"
#endif

#ifndef BOX_USER4_NAME
#define BOX_USER4_NAME "USER4"
#endif
