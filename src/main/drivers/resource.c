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

#include "platform.h"

#include "resource.h"

const resourceOwner_t resourceOwnerInvalid = { .owner = OWNER_INVALID, .index = 0 };
const resourceOwner_t resourceOwnerFree    = { .owner = OWNER_FREE,    .index = 0 };

static const char * const ownerNames[] = {
    "FREE",
    "PWM",
    "PPM",
    "MOTOR",
    "SERVO",
    "LED",
    "ADC",
    "ADC_BATT",
    "ADC_CURR",
    "ADC_EXT",
    "ADC_RSSI",
    [OWNER_SERIAL_TX] = "SERIAL_TX",
    [OWNER_SERIAL_RX] = "SERIAL_RX",
    "DEBUG",
    "TIMER",
    "SONAR_TRIGGER",
    "SONAR_ECHO",
    "SYSTEM",
    "SPI_SCK",
    "SPI_SDI",
    "SPI_SDO",
    "I2C_SCL",
    "I2C_SDA",
    "SDCARD",
    "SDIO_CK",
    "SDIO_CMD",
    "SDIO_D0",
    "SDIO_D1",
    "SDIO_D2",
    "SDIO_D3",
    "SDCARD_CS",
    "SDCARD_DETECT",
    "FLASH_CS",
    "BARO_CS",
    "GYRO_CS",
    "OSD_CS",
    "RX_SPI_CS",
    "SPI_CS",
    "GYRO_EXTI",
    "BARO_EOC",
    "COMPASS_EXTI",
    "USB",
    "USB_DETECT",
    "BEEPER",
    "OSD",
    "RX_BIND",
    "INVERTER",
    "LED_STRIP",
    "TRANSPONDER",
    "VTX_POWER",
    "VTX_CS",
    "VTX_DATA",
    "VTX_CLK",
    "COMPASS_CS",
    "RX_BIND_PLUG",
    "ESCSERIAL",
    "CAMERA_CONTROL",
    "TIMUP",
    "RANGEFINDER",
    "RX_SPI",
    "PINIO",
    "USB_MSC_PIN",
    "MCO",
    "RX_SPI_BIND",
    "RX_SPI_LED",
    "PREINIT",
    "RX_SPI_EXTI",
    "RX_SPI_CC2500_TX_EN",
    "RX_SPI_CC2500_LNA_EN",
    "RX_SPI_CC2500_ANT_SEL",
    "QSPI_CLK",
    "QSPI_BK1IO0",
    "QSPI_BK1IO1",
    "QSPI_BK1IO2",
    "QSPI_BK1IO3",
    "QSPI_BK1CS",
    "QSPI_BK2IO0",
    "QSPI_BK2IO1",
    "QSPI_BK2IO2",
    "QSPI_BK2IO3",
    "QSPI_BK2CS",
    "BARO_XCLR",
    "PULLUP",
    "PULLDOWN",
    "DSHOT_BITBANG",
    "SWD",
    "RX_SPI_EXPRESSLRS_RESET",
    "RX_SPI_EXPRESSLRS_BUSY",
    [OWNER_SOFTSERIAL_TX] = "SOFTSERIAL_TX",
    [OWNER_SOFTSERIAL_RX] = "SOFTSERIAL_RX",
    [OWNER_LPUART_TX] = "LPUART_TX",
    [OWNER_LPUART_RX] = "LPUART_RX",
    "GYRO_CLKIN",
    [OWNER_PIOUART_TX] = "PIOUART_TX",
    [OWNER_PIOUART_RX] = "PIOUART_RX",
    [OWNER_CAN_TX] = "CAN_TX",
    [OWNER_CAN_RX] = "CAN_RX",
    [OWNER_CAN_SILENT] = "CAN_SILENT",
    // Keep in sync with resourceOwner_e.
};

STATIC_ASSERT(ARRAYLEN(ownerNames) == OWNER_TOTAL_COUNT, owner_names_array_count_not_equal_to_enum_total);

const char *getOwnerName(resourceOwner_e owner)
{
    if (owner < 0 || (unsigned)owner >= ARRAYLEN(ownerNames)) {
        return "INVALID";
    }

    return ownerNames[owner];
}

// Base of the `resource` ordinal for a bus peripheral, matching its hardware
// instance name: 0 on parts that number from zero (SPI0/I2C0), 1 otherwise
// (SPI1/I2C1). Derived from the same hardware-naming signals the drivers use.
#if defined(USE_SPI_DEVICE_0)
#define SPI_RESOURCE_INDEX_BASE 0
#else
#define SPI_RESOURCE_INDEX_BASE 1
#endif
#if defined(USE_I2C_DEVICE_0)
#define I2C_RESOURCE_INDEX_BASE 0
#else
#define I2C_RESOURCE_INDEX_BASE 1
#endif

// First `resource` ordinal for an owner, so the ordinal matches that peripheral's
// hardware instance name. On targets that opt in (USE_RESOURCE_INDEX_FROM_ZERO)
// bus peripherals follow their chip numbering - UART0/SPI0/I2C0 are 0-based -
// while logical resources (motor 1, servo 1) and sensors (gyro 1) stay 1-based.
// Without the opt-in every resource is 1-based.
static int resourceInputBase(resourceOwner_e owner)
{
#if defined(USE_RESOURCE_INDEX_FROM_ZERO)
    switch (owner) {
    case OWNER_SERIAL_TX:
    case OWNER_SERIAL_RX:
#if defined(USE_INVERTER)
    case OWNER_INVERTER:
#endif
        return SERIAL_UART_FIRST_INDEX;
    case OWNER_SPI_SCK:
    case OWNER_SPI_SDI:
    case OWNER_SPI_SDO:
    case OWNER_SPI_CS:
        return SPI_RESOURCE_INDEX_BASE;
    case OWNER_I2C_SCL:
    case OWNER_I2C_SDA:
        return I2C_RESOURCE_INDEX_BASE;
#if defined(USE_PIOUART)
    case OWNER_PIOUART_TX:
    case OWNER_PIOUART_RX:
        return 0;   // PIOUART instances are always named from zero
#endif
    default:
        return 1;   // motors, servos, sensors, LED, beeper, ... stay 1-based
    }
#else
    (void)owner;
    return 1;
#endif
}

// Convert between the internal 0-based resource index and the ordinal the
// `resource` CLI shows and accepts, applying the owner's base.
int resourceIndexToInput(resourceOwner_e owner, int index)
{
    return index + resourceInputBase(owner);
}

int resourceInputToIndex(resourceOwner_e owner, int input)
{
    return input - resourceInputBase(owner);
}
