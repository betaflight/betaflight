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
#include <string.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "common/axis.h"
#include "common/maths.h"

#include "config/feature.h"

#include "drivers/light_led.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/pwm_output.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_modes.h"
#include "fc/rc_controls.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "rx/rx.h"
#include "rx/cc2500_frsky_common.h"

#include "sensors/acceleration.h"

#include "pg/rx.h"
#include "pg/rx_spi.h"
#include "pg/rx_spi_cc2500.h"
#include "pg/motor.h"

#include "osd/osd.h"

#include "io/serial.h"
#include "io/vtx.h"
#include "io/ledstrip.h"
#include "pg/pinio.h"
#include "pg/piniobox.h"

#include "sensors/current.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"

void targetConfiguration(void)
{
    mixerConfigMutable()->yaw_motors_reversed = true;
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_DSHOT600;
    pinioConfigMutable()->config[0] = PINIO_CONFIG_OUT_INVERTED | PINIO_CONFIG_MODE_OUT_PP;
    pinioBoxConfigMutable()->permanentId[0] = 40;
    currentSensorADCConfigMutable()->scale = 255;

    modeActivationConditionsMutable(0)->modeId           = BOXARM;
    modeActivationConditionsMutable(0)->auxChannelIndex  = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(0)->range.startStep  = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(0)->range.endStep    = CHANNEL_VALUE_TO_STEP(2100);

    modeActivationConditionsMutable(1)->modeId           = BOXFLIPOVERAFTERCRASH;
    modeActivationConditionsMutable(1)->auxChannelIndex  = AUX3 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(1)->range.startStep  = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(1)->range.endStep    = CHANNEL_VALUE_TO_STEP(2100);

    pidConfigMutable()->pid_process_denom = 1; // 8kHz PID

    accelerometerConfigMutable()->acc_hardware = ACC_NONE;

    strcpy(pilotConfigMutable()->name, "Infinity");

    osdConfigMutable()->cap_alarm  = 2200;
    osdConfigMutable()->item_pos[OSD_CRAFT_NAME]        = OSD_POS(10,  11) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(20, 10) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_2]      = OSD_POS(2,  10) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_VTX_CHANNEL]       = OSD_POS(3,  11) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_CURRENT_DRAW]      = OSD_POS(19, 11) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_WARNINGS]          = OSD_POS(9,  7) | OSD_PROFILE_1_FLAG;
    serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIALMSP_UART)].functionMask = FUNCTION_MSP;

}
#endif
