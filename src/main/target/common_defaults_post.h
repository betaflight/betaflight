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

#ifdef USE_MAX7456
#ifndef MAX7456_CLOCK_CONFIG_DEFAULT
#define MAX7456_CLOCK_CONFIG_DEFAULT    MAX7456_CLOCK_CONFIG_OC
#endif

#ifndef MAX7456_SPI_CS_PIN
#define MAX7456_SPI_CS_PIN              NONE
#endif
#endif

// pg/bus_i2c

#ifdef I2C_FULL_RECONFIGURABILITY
#ifdef USE_I2C_DEVICE_1
#define I2C1_SCL NONE
#define I2C1_SDA NONE
#endif

#ifdef USE_I2C_DEVICE_2
#define I2C2_SCL NONE
#define I2C2_SDA NONE
#endif

#ifdef USE_I2C_DEVICE_3
#define I2C3_SCL NONE
#define I2C3_SDA NONE
#endif

#ifdef USE_I2C_DEVICE_4
#define I2C4_SCL NONE
#define I2C4_SDA NONE
#endif

#else // I2C_FULL_RECONFIGURABILITY

// Backward compatibility for exisiting targets

#ifdef STM32F1
#ifndef I2C1_SCL
#define I2C1_SCL PB8
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB9
#endif
#ifndef I2C2_SCL
#define I2C2_SCL PB10
#endif
#ifndef I2C2_SDA
#define I2C2_SDA PB11
#endif
#endif // STM32F1

#ifdef STM32F3
#ifndef I2C1_SCL
#define I2C1_SCL PB6
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB7
#endif
#ifndef I2C2_SCL
#define I2C2_SCL PA9
#endif
#ifndef I2C2_SDA
#define I2C2_SDA PA10
#endif
#endif // STM32F3

#ifdef STM32F4
#ifndef I2C1_SCL
#define I2C1_SCL PB6
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB7
#endif
#ifndef I2C2_SCL
#define I2C2_SCL PB10
#endif
#ifndef I2C2_SDA
#define I2C2_SDA PB11
#endif
#ifndef I2C3_SCL
#define I2C3_SCL PA8
#endif
#ifndef I2C3_SDA
#define I2C3_SDA PC9
#endif
#endif // STM32F4

#ifdef STM32F7
#ifndef I2C1_SCL
#define I2C1_SCL PB6
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB7
#endif
#ifndef I2C2_SCL
#define I2C2_SCL PB10
#endif
#ifndef I2C2_SDA
#define I2C2_SDA PB11
#endif
#ifndef I2C3_SCL
#define I2C3_SCL PA8
#endif
#ifndef I2C3_SDA
#define I2C3_SDA PB4
#endif
#ifndef I2C4_SCL
#define I2C4_SCL PD12
#endif
#ifndef I2C4_SDA
#define I2C4_SDA PD13
#endif
#endif // STM32F7

#endif // I2C_FULL_RECONFIGURABILITY

// Default values for internal pullup

#if defined(USE_I2C_PULLUP)
#define I2C1_PULLUP true
#define I2C2_PULLUP true
#define I2C3_PULLUP true
#define I2C4_PULLUP true
#else
#define I2C1_PULLUP false
#define I2C2_PULLUP false
#define I2C3_PULLUP false
#define I2C4_PULLUP false
#endif

// pg/bus_spi

#ifdef SPI_FULL_RECONFIGURABILITY

#ifdef USE_SPI_DEVICE_1
#define SPI1_SCK_PIN    NONE
#define SPI1_MISO_PIN   NONE
#define SPI1_MOSI_PIN   NONE
#endif

#ifdef USE_SPI_DEVICE_2
#define SPI2_SCK_PIN    NONE
#define SPI2_MISO_PIN   NONE
#define SPI2_MOSI_PIN   NONE
#endif

#ifdef USE_SPI_DEVICE_3
#define SPI3_SCK_PIN    NONE
#define SPI3_MISO_PIN   NONE
#define SPI3_MOSI_PIN   NONE
#endif

#ifdef USE_SPI_DEVICE_4
#define SPI4_SCK_PIN    NONE
#define SPI4_MISO_PIN   NONE
#define SPI4_MOSI_PIN   NONE
#endif

#else

// Pin defaults for backward compatibility

#ifndef SPI1_SCK_PIN
#define SPI1_SCK_PIN    PA5
#define SPI1_MISO_PIN   PA6
#define SPI1_MOSI_PIN   PA7
#endif

#ifndef SPI2_SCK_PIN
#define SPI2_SCK_PIN    PB13
#define SPI2_MISO_PIN   PB14
#define SPI2_MOSI_PIN   PB15
#endif

#ifndef SPI3_SCK_PIN
#define SPI3_SCK_PIN    PB3
#define SPI3_MISO_PIN   PB4
#define SPI3_MOSI_PIN   PB5
#endif
#endif

// Extracted from rx/rx.c and rx/rx.h

#define RX_MAPPABLE_CHANNEL_COUNT 8

#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif
#ifndef SERIALRX_PROVIDER
#define SERIALRX_PROVIDER 0
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
