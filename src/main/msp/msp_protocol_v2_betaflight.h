/*
 * This file is part of Cleanflight, Betaflight and INAV.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight, Betaflight and INAV are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#define MSP2_BETAFLIGHT_BIND                0x3000
#define MSP2_MOTOR_OUTPUT_REORDERING        0x3001
#define MSP2_SET_MOTOR_OUTPUT_REORDERING    0x3002
#define MSP2_SEND_DSHOT_COMMAND             0x3003
#define MSP2_GET_VTX_DEVICE_STATUS          0x3004
#define MSP2_GET_OSD_WARNINGS               0x3005  // returns active OSD warning message text
#define MSP2_GET_TEXT                       0x3006
#define MSP2_SET_TEXT                       0x3007
#define MSP2_GET_LED_STRIP_CONFIG_VALUES    0x3008
#define MSP2_SET_LED_STRIP_CONFIG_VALUES    0x3009
#define MSP2_SENSOR_CONFIG_ACTIVE           0x300A
#define MSP2_SENSOR_OPTICALFLOW             0x300B
#define MSP2_MCU_INFO                       0x300C
#define MSP2_GYRO_SENSOR_ACTIVE             0x300D
#define MSP2_BATTERY_PROFILE                0x300E
#define MSP2_SET_BATTERY_PROFILE            0x300F
#define MSP2_CLI_SETTING                    0x3010
#define MSP2_CLI_SETTING_INFO               0x3011
#define MSP2_CLI_COMMAND                    0x3012
#define MSP2_MOTOR_EXCITATION_STATUS        0x3013  // out: bench motor excitation session status
#define MSP2_SET_MOTOR_EXCITATION           0x3014  // in: start / keepalive / stop a bench excitation session
#define MSP2_SET_FRAME_WIZARD               0x3017  // in: start / keepalive / abort the frame wizard
#define MSP2_FRAME_WIZARD_STATUS            0x3018  // out: frame wizard progress (polling it is the keepalive)
#define MSP2_FRAME_WIZARD_RESULT            0x3019  // out: confirmed tap modes per region, confirmed peaks per motor
#define MSP2_FRAME_WIZARD_RESPONSE          0x301A  // in: base, motor; out: 3 x 128 gain bins; base 2: the spin check
#define MSP2_FRAME_WIZARD_TAPS              0x301B  // in: page; out: session taps with their peaks, tap features, tap spectrum
#define MSP2_FILTER_TUNE_STATUS             0x301C  // out: filter autotune state, harmonic strength, residual peaks, proposed / current
#define MSP2_FRAME_MAP                      0x301D  // out: the stored frame map (modes the frame wizard confirmed)
#define MSP2_FRAME_WIZARD_LISTEN            0x301E  // out: listen mode, the last tap's peaks (polling it holds the mode)

// MSP2_CLI_COMMAND response flags (byte following the u16 total-length header)
#define MSP2_CLI_COMMAND_FLAG_TRUNCATED     (1 << 0) // output exceeded the pageable buffer
#define MSP2_CLI_COMMAND_FLAG_REFUSED       (1 << 1) // command refused or paging session mismatch

// MSP2_SET_TEXT and MSP2_GET_TEXT variable types
#define MSP2TEXT_PILOT_NAME                      1
#define MSP2TEXT_CRAFT_NAME                      2
#define MSP2TEXT_PID_PROFILE_NAME                3
#define MSP2TEXT_RATE_PROFILE_NAME               4
#define MSP2TEXT_BUILDKEY                        5
#define MSP2TEXT_RELEASENAME                     6
#define MSP2TEXT_CUSTOM_MSG_0                    7  // CUSTOM_MSG_MAX_NUM  entries are allocated
#define CUSTOM_MSG_MAX_NUM                       4
#define MSP2TEXT_BATTERY_PROFILE_NAME        11
// next new variable type must be >= MSP2TEXT_BATTERY_PROFILE_NAME + 1 (12)
