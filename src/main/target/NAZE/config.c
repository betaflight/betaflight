/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#ifdef TARGET_CONFIG

#include "common/axis.h"
#include "common/utils.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/controlrate_profile.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "hardware_revision.h"

void targetConfiguration(void)
{
#ifdef BEEBRAIN
    // alternative defaults settings for Beebrain target
    motorConfigMutable()->dev.motorPwmRate = 4000;
    failsafeConfigMutable()->failsafe_delay = 2;
    failsafeConfigMutable()->failsafe_off_delay = 0;

    motorConfigMutable()->minthrottle = 1049;

    gyroConfigMutable()->gyro_lpf = GYRO_LPF_188HZ;
    gyroConfigMutable()->gyro_soft_lpf_hz = 100;
    gyroConfigMutable()->gyro_soft_notch_hz_1 = 0;
    gyroConfigMutable()->gyro_soft_notch_hz_2 = 0;

    /*for (int channel = 0; channel < NON_AUX_CHANNEL_COUNT; channel++) {
        rxChannelRangeConfigsMutable(channel)->min = 1180;
        rxChannelRangeConfigsMutable(channel)->max = 1860;
    }*/

    for (int profileId = 0; profileId < MAX_PROFILE_COUNT; profileId++) {
        pidProfilesMutable(profileId)->pid[PID_ROLL].P = 60;
        pidProfilesMutable(profileId)->pid[PID_ROLL].I = 70;
        pidProfilesMutable(profileId)->pid[PID_ROLL].D = 17;
        pidProfilesMutable(profileId)->pid[PID_PITCH].P = 80;
        pidProfilesMutable(profileId)->pid[PID_PITCH].I = 90;
        pidProfilesMutable(profileId)->pid[PID_PITCH].D = 18;
        pidProfilesMutable(profileId)->pid[PID_YAW].P = 200;
        pidProfilesMutable(profileId)->pid[PID_YAW].I = 45;
        pidProfilesMutable(profileId)->pid[PID_LEVEL].P = 30;
        pidProfilesMutable(profileId)->pid[PID_LEVEL].D = 30;

        pidProfilesMutable(profileId)->dtermSetpointWeight = 200;
        pidProfilesMutable(profileId)->setpointRelaxRatio = 50;
    }

    for (int rateProfileId = 0; rateProfileId < CONTROL_RATE_PROFILE_COUNT; rateProfileId++) {
        controlRateProfilesMutable(rateProfileId)->rcRate8 = 100;
        controlRateProfilesMutable(rateProfileId)->rcYawRate8 = 110;
        controlRateProfilesMutable(rateProfileId)->rcExpo8 = 0;
        controlRateProfilesMutable(rateProfileId)->rates[FD_ROLL] = 77;
        controlRateProfilesMutable(rateProfileId)->rates[FD_PITCH] = 77;
        controlRateProfilesMutable(rateProfileId)->rates[FD_YAW] = 80;
    }
#endif

#if !defined(AFROMINI) && !defined(BEEBRAIN)
    if (hardwareRevision >= NAZE32_REV5) {
        // naze rev4 and below used opendrain to PNP for buzzer. Rev5 and above use PP to NPN.
        beeperDevConfigMutable()->isOpenDrain = false;
        beeperDevConfigMutable()->isInverted = true;
    } else {
        beeperDevConfigMutable()->isOpenDrain = true;
        beeperDevConfigMutable()->isInverted = false;
        flashConfigMutable()->csTag = IO_TAG_NONE;
    }
#endif

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
