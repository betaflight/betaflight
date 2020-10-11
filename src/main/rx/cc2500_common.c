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

#include "platform.h"

#if defined(USE_RX_FRSKY_SPI) || defined(USE_RX_SFHSS_SPI)

#include "common/maths.h"

#include "drivers/io.h"
#include "drivers/rx/rx_cc2500.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/time.h"

#include "config/config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"
#include "pg/rx_spi.h"
#include "pg/rx_spi_cc2500.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"

#include "cc2500_common.h"

#if defined(USE_RX_CC2500_SPI_PA_LNA)
static IO_t txEnPin;
static IO_t rxLnaEnPin;
#if defined(USE_RX_CC2500_SPI_DIVERSITY)
static IO_t antSelPin;
#endif

static int16_t rssiDbm;

uint16_t cc2500getRssiDbm(void)
{
    return rssiDbm;
}

void cc2500setRssiDbm(uint8_t value)
{
    if (value >= 128) {
        rssiDbm = ((((uint16_t)value) * 18) >> 5) - 82;
    } else {
        rssiDbm = ((((uint16_t)value) * 18) >> 5) + 65;
    }

    setRssi(rssiDbm << 3, RSSI_SOURCE_RX_PROTOCOL);
}

#if defined(USE_RX_CC2500_SPI_PA_LNA) && defined(USE_RX_CC2500_SPI_DIVERSITY)
void cc2500switchAntennae(void)
{
    static bool alternativeAntennaSelected = true;

    if (antSelPin) {
        if (alternativeAntennaSelected) {
            IOLo(antSelPin);
        } else {
            IOHi(antSelPin);
        }
        alternativeAntennaSelected = !alternativeAntennaSelected;
    }
}
#endif
#if defined(USE_RX_CC2500_SPI_PA_LNA)
void cc2500TxEnable(void)
{
    if (txEnPin) {
        IOHi(txEnPin);
    }
}

void cc2500TxDisable(void)
{
    if (txEnPin) {
        IOLo(txEnPin);
    }
}
#endif

static bool cc2500SpiDetect(void)
{
    const uint8_t chipPartNum = cc2500ReadReg(CC2500_30_PARTNUM | CC2500_READ_BURST); //CC2500 read registers chip part num
    const uint8_t chipVersion = cc2500ReadReg(CC2500_31_VERSION | CC2500_READ_BURST); //CC2500 read registers chip version
    if (chipPartNum == 0x80 && chipVersion == 0x03) {
        return true;
    }

    return false;
}

bool cc2500SpiInit(void)
{
    if (rxCc2500SpiConfig()->chipDetectEnabled && !cc2500SpiDetect()) {
        return false;
    }

    if (!rxSpiExtiConfigured()) {
        return false;
    }

#if defined(USE_RX_CC2500_SPI_PA_LNA)
    if (rxCc2500SpiConfig()->lnaEnIoTag) {
        rxLnaEnPin = IOGetByTag(rxCc2500SpiConfig()->lnaEnIoTag);
        IOInit(rxLnaEnPin, OWNER_RX_SPI_CC2500_LNA_EN, 0);
        IOConfigGPIO(rxLnaEnPin, IOCFG_OUT_PP);

        IOHi(rxLnaEnPin); // always on at the moment
    }
    if (rxCc2500SpiConfig()->txEnIoTag) {
        txEnPin = IOGetByTag(rxCc2500SpiConfig()->txEnIoTag);
        IOInit(txEnPin, OWNER_RX_SPI_CC2500_TX_EN, 0);
        IOConfigGPIO(txEnPin, IOCFG_OUT_PP);
    } else {
        txEnPin = IO_NONE;
    }
#if defined(USE_RX_CC2500_SPI_DIVERSITY)
    if (rxCc2500SpiConfig()->antSelIoTag) {
        antSelPin = IOGetByTag(rxCc2500SpiConfig()->antSelIoTag);
        IOInit(antSelPin, OWNER_RX_SPI_CC2500_ANT_SEL, 0);
        IOConfigGPIO(antSelPin, IOCFG_OUT_PP);

        IOHi(antSelPin);
    } else {
        antSelPin = IO_NONE;
    }
#endif
#endif // USE_RX_CC2500_SPI_PA_LNA

#if defined(USE_RX_CC2500_SPI_PA_LNA)
    cc2500TxDisable();
#endif // USE_RX_CC2500_SPI_PA_LNA

    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }

    return true;
}
#endif

void cc2500ApplyRegisterConfig(const cc2500RegisterConfigElement_t *configArrayPtr, int configSize)
{
    const int entryCount = configSize / sizeof(cc2500RegisterConfigElement_t);
    for (int i = 0; i < entryCount; i++) {
        cc2500WriteReg(configArrayPtr->registerID, configArrayPtr->registerValue);
        configArrayPtr++;
    }
}
#endif
