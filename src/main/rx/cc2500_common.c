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

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"
#include "pg/rx_spi.h"

#include "drivers/rx/rx_cc2500.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "fc/config.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"

#include "rx/cc2500_common.h"

static IO_t gdoPin;
#if defined(USE_RX_CC2500_SPI_PA_LNA)
static IO_t txEnPin;
static IO_t rxLnaEnPin;
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

bool cc2500getGdo(void)
{
    return IORead(gdoPin);
}

#if defined(USE_RX_CC2500_SPI_PA_LNA) && defined(USE_RX_CC2500_SPI_DIVERSITY)
void cc2500switchAntennae(void)
{
    static bool alternativeAntennaSelected = true;

    if (alternativeAntennaSelected) {
        IOLo(antSelPin);
    } else {
        IOHi(antSelPin);
    }
    alternativeAntennaSelected = !alternativeAntennaSelected;
}
#endif
#if defined(USE_RX_CC2500_SPI_PA_LNA)
void cc2500TxEnable(void)
{
    IOHi(txEnPin);
}

void cc2500TxDisable(void)
{
    IOLo(txEnPin);
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
#if !defined(RX_CC2500_SPI_DISABLE_CHIP_DETECTION)
    if (!cc2500SpiDetect()) {
        return false;
    }
#else
    UNUSED(cc2500SpiDetect);
#endif

    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }

    // gpio init here
    gdoPin = IOGetByTag(IO_TAG(RX_CC2500_SPI_GDO_0_PIN));
    IOInit(gdoPin, OWNER_RX_SPI, 0);
    IOConfigGPIO(gdoPin, IOCFG_IN_FLOATING);
#if defined(USE_RX_CC2500_SPI_PA_LNA)
    rxLnaEnPin = IOGetByTag(IO_TAG(RX_CC2500_SPI_LNA_EN_PIN));
    IOInit(rxLnaEnPin, OWNER_RX_SPI, 0);
    IOConfigGPIO(rxLnaEnPin, IOCFG_OUT_PP);
    IOHi(rxLnaEnPin); // always on at the moment
    txEnPin = IOGetByTag(IO_TAG(RX_CC2500_SPI_TX_EN_PIN));
    IOInit(txEnPin, OWNER_RX_SPI, 0);
    IOConfigGPIO(txEnPin, IOCFG_OUT_PP);
#if defined(USE_RX_CC2500_SPI_DIVERSITY)
    antSelPin = IOGetByTag(IO_TAG(RX_CC2500_SPI_ANT_SEL_PIN));
    IOInit(antSelPin, OWNER_RX_SPI, 0);
    IOConfigGPIO(antSelPin, IOCFG_OUT_PP);
#endif
#endif // USE_RX_CC2500_SPI_PA_LNA

#if defined(USE_RX_CC2500_SPI_PA_LNA)
#if defined(USE_RX_CC2500_SPI_DIVERSITY)
    IOHi(antSelPin);
#endif
    cc2500TxDisable();
#endif // USE_RX_CC2500_SPI_PA_LNA

    return true;
}
#endif
