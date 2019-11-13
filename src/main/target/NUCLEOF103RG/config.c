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

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "common/axis.h"
#include "common/utils.h"

#include "drivers/io.h"

#include "config/config.h"
#include "fc/rc_controls.h"
#include "fc/controlrate_profile.h"

#include "flight/failsafe.h"
#include "flight/pid.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "pg/beeper_dev.h"
#include "pg/flash.h"
#include "pg/motor.h"

#include "hardware_revision.h"

void targetConfiguration(void)
{
    if (hardwareRevision >= NAZE32_REV5) {
        // naze rev4 and below used opendrain to PNP for buzzer. Rev5 and above use PP to NPN.
        beeperDevConfigMutable()->isOpenDrain = false;
        beeperDevConfigMutable()->isInverted = true;
    } else {
        beeperDevConfigMutable()->isOpenDrain = true;
        beeperDevConfigMutable()->isInverted = false;
        flashConfigMutable()->csTag = IO_TAG_NONE;
    }

#ifdef MAG_INT_EXTI
    if (hardwareRevision < NAZE32_REV5) {
        compassConfigMutable()->interruptTag = IO_TAG(PB12);
    }
#endif
}

void targetValidateConfiguration(void)
{
    if (hardwareRevision < NAZE32_REV5 && accelerometerConfig()->acc_hardware == ACC_ADXL345) {
        accelerometerConfigMutable()->acc_hardware = ACC_NONE;
    }
}
#endif
