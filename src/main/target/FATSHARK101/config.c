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

#include <platform.h>

//#ifdef USE_TARGET_CONFIG

#include "common/axis.h"

#include "fc/config.h"
#include "fc/rc_modes.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_adjustments.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "sensors/boardalignment.h"
#include "drivers/serial.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "rx/rx.h"

void targetConfiguration(void) {
    //Set the UART2 as RX serial port
    serialConfigMutable()->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
    rxConfigMutable()->serialrx_provider = 2;

    //Set the alignment for the board, as it is vertical
    boardAlignmentMutable()->rollDegrees = 90;

    //Set the motor PWM rate and minmum throttle value when ESC armed
    motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
    motorConfigMutable()->minthrottle = 1080;

    //Set the throttle range max to 2050 to keep a bit of power for the FC
    rxChannelRangeConfigsMutable(3)->max = 2050;

    //Setup same pid values for each profiles
    for (uint8_t pidProfileIndex = 0; pidProfileIndex < MAX_PROFILE_COUNT; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

        pidProfile->pid[FD_ROLL].P = 50;
        pidProfile->pid[FD_ROLL].I = 40;
        pidProfile->pid[FD_ROLL].D = 40;
        pidProfile->pid[FD_PITCH].P = 80;
        pidProfile->pid[FD_PITCH].I = 50;
        pidProfile->pid[FD_PITCH].D = 50;
        pidProfile->pid[FD_YAW].P = 90;
        pidProfile->pid[FD_YAW].I = 50;
        pidProfile->levelAngleLimit = 13;
    }

    /*setup the different rates and expo for the 3 rate profiles
     *
     */

    //Rate Profile 1
    controlRateProfilesMutable(0)->rcRate8 = 50;
    controlRateProfilesMutable(0)->rcExpo8 = 30;
    controlRateProfilesMutable(0)->rcYawRate8 = 50;
    controlRateProfilesMutable(0)->rcYawExpo8 = 30;
    controlRateProfilesMutable(0)->thrMid8 = 45;
    controlRateProfilesMutable(0)->thrExpo8 = 90;

    //Rate Profile 2
    controlRateProfilesMutable(1)->rcRate8 = 50;
    controlRateProfilesMutable(1)->rcExpo8 = 30;
    controlRateProfilesMutable(1)->rcYawRate8 = 50;
    controlRateProfilesMutable(1)->rcYawExpo8 = 30;
    controlRateProfilesMutable(1)->thrMid8 = 45;
    controlRateProfilesMutable(1)->thrExpo8 = 90;

    //Rate Profile 3
    controlRateProfilesMutable(2)->rcRate8 = 100;
    controlRateProfilesMutable(2)->rcExpo8 = 45;
    controlRateProfilesMutable(2)->rcYawRate8 = 100;
    controlRateProfilesMutable(2)->rcYawExpo8 = 35;
    controlRateProfilesMutable(2)->thrMid8 = 45;
    controlRateProfilesMutable(2)->thrExpo8 = 90;

    /* Modes configuration
     * First mode sets the range for arm/disarm
     * Second mode sets the range for angle/ acro mode selection
     */
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->auxChannelIndex = 1;
    modeActivationConditionsMutable(0)->range.startStep = 33; //Start point set at 1725
    modeActivationConditionsMutable(0)->range.endStep = 48;	//End point set at 2100

    modeActivationConditionsMutable(1)->modeId = BOXANGLE;
    modeActivationConditionsMutable(1)->auxChannelIndex = 0;
    modeActivationConditionsMutable(1)->range.startStep = 0;	//Start point set at 900
    modeActivationConditionsMutable(1)->range.endStep = 15;	//End point set at 1275

    //Setup the adjustment ranges and function to switch rate profiles via the 3 pos switch
    adjustmentRangesMutable(0)->adjustmentFunction = ADJUSTMENT_RATE_PROFILE;
    adjustmentRangesMutable(0)->range.startStep = 0;
    adjustmentRangesMutable(0)->range.endStep = 48;

    /*Attempt to configure LED strip here... quite messy uncommented code behind to kept for later
     *it's working but not exactly using Flags correctly... good enough for now
     *also need to find how to use correct colors for arm state
     */
    ledStripConfigMutable()->ledConfigs[0] = DEFINE_LED(0, 0, 3, LED_DIRECTION_NORTH, LED_FUNCTION_ARM_STATE, 32, 0);
    ledStripConfigMutable()->ledConfigs[1] = DEFINE_LED(1, 0, 3, LED_DIRECTION_NORTH, LED_FUNCTION_ARM_STATE, 32, 0);

}
//#endif
