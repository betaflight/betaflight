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

#include "common/axis.h"
#include "common/maths.h"

#include "io/mavlink_attrate.h"
#include "io/serial.h"

#ifdef USE_MAVLINK_ATTRATE

// mavlink library uses unnames unions that's causes GCC to complain if -Wpedantic is used
// until this is resolved in mavlink library - ignore -Wpedantic for mavlink code
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

static serialPort_t *serialPort;

static FAST_RAM_ZERO_INIT float throttle;
static FAST_RAM_ZERO_INIT float controls[FLIGHT_DYNAMICS_INDEX_COUNT];

static FAST_RAM_ZERO_INIT volatile uint8_t mavlinkBuffer[MAVLINK_BUFFER_SIZE];
static FAST_RAM_ZERO_INIT int mavlinkBufferHead;
static FAST_RAM_ZERO_INIT int mavlinkBufferTail;

static FAST_RAM_ZERO_INIT mavlink_message_t mavMessage;
static FAST_RAM_ZERO_INIT mavlink_set_attitude_target_t attTargetMsg;

#endif // USE_MAVLINK_ATTRATE

FAST_CODE float getMavlinkThrottle()
{
#ifdef USE_MAVLINK_ATTRATE
    return throttle;
#else
    return 0.0;
#endif // USE_MAVLINK_ATTRATE
}

#ifdef USE_MAVLINK_ATTRATE

FAST_CODE float getMavlinkAttrateSetpoint(int axis)
{
    return controls[axis];
}

FAST_CODE void mavlinkAttrateUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    mavlink_status_t status_comm;

    // when mavlinkBufferTail == mavlinkBufferHead, all data has been parsed
    while (mavlinkBufferTail < mavlinkBufferHead) {
        if (mavlink_parse_char(MAVLINK_COMM_1,
                               mavlinkBuffer[mavlinkBufferTail++],
                               &mavMessage,
                               &status_comm) &&
            (MAVLINK_MSG_ID_SET_ATTITUDE_TARGET == mavMessage.msgid)) {
            mavlink_msg_set_attitude_target_decode(&mavMessage, &attTargetMsg);
            controls[FD_ROLL] = attTargetMsg.body_roll_rate / RAD;
            controls[FD_PITCH] = attTargetMsg.body_pitch_rate / RAD;
            controls[FD_YAW] = attTargetMsg.body_yaw_rate / RAD;
            throttle = attTargetMsg.thrust;
        }
        
        if (mavlinkBufferTail >= MAVLINK_BUFFER_SIZE) {
            mavlinkBufferTail = 0;
        }
    }
  
    return;
}

// Receive ISR callback
FAST_CODE static void mavlinkDataReceive(uint16_t c, void *data)
{
    UNUSED(data);
    mavlinkBuffer[mavlinkBufferHead++] = (uint8_t)c;
    if (mavlinkBufferHead >= MAVLINK_BUFFER_SIZE) {
        mavlinkBufferHead = 0;
    }
    
    // if head just overwrote tail, increment tail
    if (mavlinkBufferHead == mavlinkBufferTail) {
        mavlinkBufferTail++;
    }
    if (mavlinkBufferTail >= MAVLINK_BUFFER_SIZE) {
        mavlinkBufferTail = 0;
    }
}

bool mavlinkAttrateInit()
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MAVLINK_ATTRATE);
    if (!portConfig) {
        return false;
    }

    mavlinkBufferHead = 0;
    mavlinkBufferTail = 0;
    
    serialPort = openSerialPort(portConfig->identifier,
        FUNCTION_MAVLINK_ATTRATE,
        mavlinkDataReceive,
        NULL,
        MAVLINK_ATTRATE_BAUDRATE,
        MODE_RXTX,
        0
        );

    return serialPort != NULL;
}
#endif // USE_MAVLINK_ATTRATE
