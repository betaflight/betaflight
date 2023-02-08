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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "platform.h"

#include "resource/resource_table.h"


#if defined(USE_RESOURCE_MGMT)

// Handy macros for keeping the table tidy.
// DEFS : Single entry
// DEFA : Array of uint8_t (stride = 1)
// DEFW : Wider stride case; array of structs.

#define DEFS(owner, pgn, type, member) \
    { owner, pgn, 0, offsetof(type, member), 0 }

#define DEFA(owner, pgn, type, member, max) \
    { owner, pgn, sizeof(ioTag_t), offsetof(type, member), max }

#define DEFW(owner, pgn, type, member, max) \
    { owner, pgn, sizeof(type), offsetof(type, member), max }

const resourceValue_t resourceTable_impl[] = {
#if defined(USE_BEEPER)
    DEFS( OWNER_BEEPER,        PG_BEEPER_DEV_CONFIG, beeperDevConfig_t, ioTag) ,
#endif
    DEFA( OWNER_MOTOR,         PG_MOTOR_CONFIG, motorConfig_t, dev.ioTags[0], MAX_SUPPORTED_MOTORS ),
#if defined(USE_SERVOS)
    DEFA( OWNER_SERVO,         PG_SERVO_CONFIG, servoConfig_t, dev.ioTags[0], MAX_SUPPORTED_SERVOS ),
#endif
#if defined(USE_RX_PPM)
    DEFS( OWNER_PPMINPUT,      PG_PPM_CONFIG, ppmConfig_t, ioTag ),
#endif
#if defined(USE_RX_PWM)
    DEFA( OWNER_PWMINPUT,      PG_PWM_CONFIG, pwmConfig_t, ioTags[0], PWM_INPUT_PORT_COUNT ),
#endif
#if defined(USE_RANGEFINDER_HCSR04)
    DEFS( OWNER_SONAR_TRIGGER, PG_SONAR_CONFIG, sonarConfig_t, triggerTag ),
    DEFS( OWNER_SONAR_ECHO,    PG_SONAR_CONFIG, sonarConfig_t, echoTag ),
#endif
#if defined(USE_LED_STRIP)
    DEFS( OWNER_LED_STRIP,     PG_LED_STRIP_CONFIG, ledStripConfig_t, ioTag ),
#endif
#ifdef USE_UART
    DEFA( OWNER_SERIAL_TX,     PG_SERIAL_PIN_CONFIG, serialPinConfig_t, ioTagTx[0], SERIAL_PORT_MAX_INDEX ),
    DEFA( OWNER_SERIAL_RX,     PG_SERIAL_PIN_CONFIG, serialPinConfig_t, ioTagRx[0], SERIAL_PORT_MAX_INDEX ),
#endif
#ifdef USE_INVERTER
    DEFA( OWNER_INVERTER,      PG_SERIAL_PIN_CONFIG, serialPinConfig_t, ioTagInverter[0], SERIAL_PORT_MAX_INDEX ),
#endif
#ifdef USE_I2C
    DEFW( OWNER_I2C_SCL,       PG_I2C_CONFIG, i2cConfig_t, ioTagScl, I2CDEV_COUNT ),
    DEFW( OWNER_I2C_SDA,       PG_I2C_CONFIG, i2cConfig_t, ioTagSda, I2CDEV_COUNT ),
#endif
    DEFA( OWNER_LED,           PG_STATUS_LED_CONFIG, statusLedConfig_t, ioTags[0], STATUS_LED_NUMBER ),
#ifdef USE_SPEKTRUM_BIND
    DEFS( OWNER_RX_BIND,       PG_RX_CONFIG, rxConfig_t, spektrum_bind_pin_override_ioTag ),
    DEFS( OWNER_RX_BIND_PLUG,  PG_RX_CONFIG, rxConfig_t, spektrum_bind_plug_ioTag ),
#endif
#ifdef USE_TRANSPONDER
    DEFS( OWNER_TRANSPONDER,   PG_TRANSPONDER_CONFIG, transponderConfig_t, ioTag ),
#endif
#ifdef USE_SPI
    DEFW( OWNER_SPI_SCK,       PG_SPI_PIN_CONFIG, spiPinConfig_t, ioTagSck, SPIDEV_COUNT ),
    DEFW( OWNER_SPI_MISO,      PG_SPI_PIN_CONFIG, spiPinConfig_t, ioTagMiso, SPIDEV_COUNT ),
    DEFW( OWNER_SPI_MOSI,      PG_SPI_PIN_CONFIG, spiPinConfig_t, ioTagMosi, SPIDEV_COUNT ),
#endif
#ifdef USE_ESCSERIAL
    DEFS( OWNER_ESCSERIAL,     PG_ESCSERIAL_CONFIG, escSerialConfig_t, ioTag ),
#endif
#ifdef USE_CAMERA_CONTROL
    DEFS( OWNER_CAMERA_CONTROL, PG_CAMERA_CONTROL_CONFIG, cameraControlConfig_t, ioTag ),
#endif
#ifdef USE_ADC
    DEFS( OWNER_ADC_BATT,      PG_ADC_CONFIG, adcConfig_t, vbat.ioTag ),
    DEFS( OWNER_ADC_RSSI,      PG_ADC_CONFIG, adcConfig_t, rssi.ioTag ),
    DEFS( OWNER_ADC_CURR,      PG_ADC_CONFIG, adcConfig_t, current.ioTag ),
    DEFS( OWNER_ADC_EXT,       PG_ADC_CONFIG, adcConfig_t, external1.ioTag ),
#endif
#ifdef USE_BARO
    DEFS( OWNER_BARO_CS,       PG_BAROMETER_CONFIG, barometerConfig_t, baro_spi_csn ),
    DEFS( OWNER_BARO_EOC,      PG_BAROMETER_CONFIG, barometerConfig_t, baro_eoc_tag ),
    DEFS( OWNER_BARO_XCLR,     PG_BAROMETER_CONFIG, barometerConfig_t, baro_xclr_tag ),
#endif
#ifdef USE_MAG
    DEFS( OWNER_COMPASS_CS,    PG_COMPASS_CONFIG, compassConfig_t, mag_spi_csn ),
#ifdef USE_MAG_DATA_READY_SIGNAL
    DEFS( OWNER_COMPASS_EXTI,  PG_COMPASS_CONFIG, compassConfig_t, interruptTag ),
#endif
#endif
#ifdef USE_SDCARD_SPI
    DEFS( OWNER_SDCARD_CS,     PG_SDCARD_CONFIG, sdcardConfig_t, chipSelectTag ),
#endif
#ifdef USE_SDCARD
    DEFS( OWNER_SDCARD_DETECT, PG_SDCARD_CONFIG, sdcardConfig_t, cardDetectTag ),
#endif
#if defined(STM32H7) && defined(USE_SDCARD_SDIO)
    DEFS( OWNER_SDIO_CK,       PG_SDIO_PIN_CONFIG, sdioPinConfig_t, CKPin ),
    DEFS( OWNER_SDIO_CMD,      PG_SDIO_PIN_CONFIG, sdioPinConfig_t, CMDPin ),
    DEFS( OWNER_SDIO_D0,       PG_SDIO_PIN_CONFIG, sdioPinConfig_t, D0Pin ),
    DEFS( OWNER_SDIO_D1,       PG_SDIO_PIN_CONFIG, sdioPinConfig_t, D1Pin ),
    DEFS( OWNER_SDIO_D2,       PG_SDIO_PIN_CONFIG, sdioPinConfig_t, D2Pin ),
    DEFS( OWNER_SDIO_D3,       PG_SDIO_PIN_CONFIG, sdioPinConfig_t, D3Pin ),
#endif
#ifdef USE_PINIO
    DEFA( OWNER_PINIO,         PG_PINIO_CONFIG, pinioConfig_t, ioTag, PINIO_COUNT ),
#endif
#if defined(USE_USB_MSC)
    DEFS( OWNER_USB_MSC_PIN,   PG_USB_CONFIG, usbDev_t, mscButtonPin ),
#endif
#ifdef USE_FLASH_CHIP
    DEFS( OWNER_FLASH_CS,      PG_FLASH_CONFIG, flashConfig_t, csTag ),
#endif
#ifdef USE_MAX7456
    DEFS( OWNER_OSD_CS,        PG_MAX7456_CONFIG, max7456Config_t, csTag ),
#endif
#ifdef USE_RX_SPI
    DEFS( OWNER_RX_SPI_CS,     PG_RX_SPI_CONFIG, rxSpiConfig_t, csnTag ),
    DEFS( OWNER_RX_SPI_EXTI,   PG_RX_SPI_CONFIG, rxSpiConfig_t, extiIoTag ),
    DEFS( OWNER_RX_SPI_BIND,   PG_RX_SPI_CONFIG, rxSpiConfig_t, bindIoTag ),
    DEFS( OWNER_RX_SPI_LED,    PG_RX_SPI_CONFIG, rxSpiConfig_t, ledIoTag ),
#if defined(USE_RX_CC2500) && defined(USE_RX_CC2500_SPI_PA_LNA)
    DEFS( OWNER_RX_SPI_CC2500_TX_EN,   PG_RX_CC2500_SPI_CONFIG, rxCc2500SpiConfig_t, txEnIoTag ),
    DEFS( OWNER_RX_SPI_CC2500_LNA_EN,  PG_RX_CC2500_SPI_CONFIG, rxCc2500SpiConfig_t, lnaEnIoTag ),
#if defined(USE_RX_CC2500_SPI_DIVERSITY)
    DEFS( OWNER_RX_SPI_CC2500_ANT_SEL, PG_RX_CC2500_SPI_CONFIG, rxCc2500SpiConfig_t, antSelIoTag ),
#endif
#endif
#if defined(USE_RX_EXPRESSLRS)
    DEFS( OWNER_RX_SPI_EXPRESSLRS_RESET, PG_RX_EXPRESSLRS_SPI_CONFIG, rxExpressLrsSpiConfig_t, resetIoTag ),
    DEFS( OWNER_RX_SPI_EXPRESSLRS_BUSY, PG_RX_EXPRESSLRS_SPI_CONFIG, rxExpressLrsSpiConfig_t, busyIoTag ),
#endif
#endif
    DEFW( OWNER_GYRO_EXTI,     PG_GYRO_DEVICE_CONFIG, gyroDeviceConfig_t, extiTag, MAX_GYRODEV_COUNT ),
    DEFW( OWNER_GYRO_CS,       PG_GYRO_DEVICE_CONFIG, gyroDeviceConfig_t, csnTag, MAX_GYRODEV_COUNT ),
#ifdef USE_USB_DETECT
    DEFS( OWNER_USB_DETECT,    PG_USB_CONFIG, usbDev_t, detectPin ),
#endif
#ifdef USE_VTX_RTC6705
    DEFS( OWNER_VTX_POWER,     PG_VTX_IO_CONFIG, vtxIOConfig_t, powerTag ),
    DEFS( OWNER_VTX_CS,        PG_VTX_IO_CONFIG, vtxIOConfig_t, csTag ),
    DEFS( OWNER_VTX_DATA,      PG_VTX_IO_CONFIG, vtxIOConfig_t, dataTag ),
    DEFS( OWNER_VTX_CLK,       PG_VTX_IO_CONFIG, vtxIOConfig_t, clockTag ),
#endif
#ifdef USE_PIN_PULL_UP_DOWN
    DEFA( OWNER_PULLUP,        PG_PULLUP_CONFIG,   pinPullUpDownConfig_t, ioTag, PIN_PULL_UP_DOWN_COUNT ),
    DEFA( OWNER_PULLDOWN,      PG_PULLDOWN_CONFIG, pinPullUpDownConfig_t, ioTag, PIN_PULL_UP_DOWN_COUNT ),
#endif
};

const resourceValue_t *resourceTable = &resourceTable_impl[0];

#undef DEFS
#undef DEFA
#undef DEFW

uint8_t resource_resourceTableLength(void)
{
    return ARRAYLEN(resourceTable_impl);
}

#endif // USE_RESOURCE_MGMT

#ifdef USE_DMA_SPEC

// Handy macros for keeping the table tidy.
// DEFS : Single entry
// DEFA : Array of uint8_t (stride = 1)
// DEFW : Wider stride case; array of structs.

#define DEFS(device, peripheral, pgn, type, member) \
    { device, peripheral, pgn, 0,               offsetof(type, member), 0, MASK_IGNORED }

#define DEFA(device, peripheral, pgn, type, member, max, mask) \
    { device, peripheral, pgn, sizeof(uint8_t), offsetof(type, member), max, mask }

#define DEFW(device, peripheral, pgn, type, member, max, mask) \
    { device, peripheral, pgn, sizeof(type), offsetof(type, member), max, mask }

const dmaoptEntry_t dmaoptEntryTable_impl[] = {
    DEFW("SPI_MOSI", DMA_PERIPH_SPI_MOSI, PG_SPI_PIN_CONFIG,     spiPinConfig_t,     txDmaopt, SPIDEV_COUNT,                    MASK_IGNORED),
    DEFW("SPI_MISO", DMA_PERIPH_SPI_MISO, PG_SPI_PIN_CONFIG,     spiPinConfig_t,     rxDmaopt, SPIDEV_COUNT,                    MASK_IGNORED),
    // SPI_TX/SPI_RX for backwards compatibility with unified configs defined for 4.2.x
    DEFW("SPI_TX",   DMA_PERIPH_SPI_MOSI, PG_SPI_PIN_CONFIG,     spiPinConfig_t,     txDmaopt, SPIDEV_COUNT,                    MASK_IGNORED),
    DEFW("SPI_RX",   DMA_PERIPH_SPI_MISO, PG_SPI_PIN_CONFIG,     spiPinConfig_t,     rxDmaopt, SPIDEV_COUNT,                    MASK_IGNORED),
    DEFA("ADC",      DMA_PERIPH_ADC,      PG_ADC_CONFIG,         adcConfig_t,        dmaopt,   ADCDEV_COUNT,                    MASK_IGNORED),
    DEFS("SDIO",     DMA_PERIPH_SDIO,     PG_SDIO_CONFIG,        sdioConfig_t,       dmaopt),
    DEFW("UART_TX",  DMA_PERIPH_UART_TX,  PG_SERIAL_UART_CONFIG, serialUartConfig_t, txDmaopt, UARTDEV_CONFIG_MAX,              MASK_IGNORED),
    DEFW("UART_RX",  DMA_PERIPH_UART_RX,  PG_SERIAL_UART_CONFIG, serialUartConfig_t, rxDmaopt, UARTDEV_CONFIG_MAX,              MASK_IGNORED),
#if defined(STM32H7) || defined(STM32G4)
    DEFW("TIMUP",    DMA_PERIPH_TIMUP,    PG_TIMER_UP_CONFIG,    timerUpConfig_t,    dmaopt,   HARDWARE_TIMER_DEFINITION_COUNT, TIMUP_TIMERS),
#endif
};

#undef DEFS
#undef DEFA
#undef DEFW

const dmaoptEntry_t *dmaoptEntryTable = &dmaoptEntryTable_impl[0];

uint8_t resource_dmaoptEntryTableLength(void)
{
    return ARRAYLEN(dmaoptEntryTable_impl);
}

#endif // USE_DMA_SPEC
