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

// MSP2_SET_TEXT and MSP2_GET_TEXT variable types
#define MSP2TEXT_PILOT_NAME                      1
#define MSP2TEXT_CRAFT_NAME                      2
#define MSP2TEXT_PID_PROFILE_NAME                3
#define MSP2TEXT_RATE_PROFILE_NAME               4
