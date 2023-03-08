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

#ifdef USE_RX_SPI

#include "drivers/io.h"
#include "drivers/bus_spi.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx_spi.h"

#include "rx/rx_spi.h"

#if !defined(RX_SPI_INSTANCE)
#define RX_SPI_INSTANCE NULL
#endif

#if !defined(RX_SPI_CS_PIN)
#define RX_SPI_CS_PIN NONE
#endif

#ifndef RX_SPI_LED_PIN
#define RX_SPI_LED_PIN NONE
#endif

#if !defined(RX_SPI_EXTI_PIN)
#define RX_SPI_EXTI_PIN NONE
#endif

#if !defined(RX_SPI_BIND_PIN)
#define RX_SPI_BIND_PIN NONE
#endif

PG_REGISTER_WITH_RESET_FN(rxSpiConfig_t, rxSpiConfig, PG_RX_SPI_CONFIG, 0);

void pgResetFn_rxSpiConfig(rxSpiConfig_t *rxSpiConfig)
{
    rxSpiConfig->rx_spi_protocol = RX_SPI_DEFAULT_PROTOCOL;

    // Basic SPI
    rxSpiConfig->csnTag = IO_TAG(RX_SPI_CS_PIN);
    rxSpiConfig->spibus = SPI_DEV_TO_CFG(spiDeviceByInstance(RX_SPI_INSTANCE));

    rxSpiConfig->extiIoTag = IO_TAG(RX_SPI_EXTI_PIN);

    rxSpiConfig->bindIoTag = IO_TAG(RX_SPI_BIND_PIN);
    rxSpiConfig->ledIoTag = IO_TAG(RX_SPI_LED_PIN);
#ifdef RX_SPI_LED_INVERTED
    rxSpiConfig->ledInversion = true;
#else
    rxSpiConfig->ledInversion = false;
#endif
}
#endif
