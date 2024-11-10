/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

#include "platform.h"

#if defined(USE_GIMBAL)

#include "build/debug.h"

#include "common/crc.h"
#include "common/maths.h"

#include "drivers/time.h"

#include "fc/rc_controls.h"

#include "io/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/gimbal.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

/* The setting below accommodate both -ve and +ve setting so the gimbal can be mounted either  way up.
 *
 * |---------------------------|-----|------------|----------------------------------------------------------------------------------------|
 * | Setting                   | Def | Range      | Purpose                                                                                |
 * |---------------------------|-----|------------|----------------------------------------------------------------------------------------|
 * | gimbal_roll_rc_gain       | 40  | -100 - 100 | Adjusts amount of roll in % from RC roll channel                                       |
 * | gimbal_pitch_rc_thr_gain  | 10  | -100 - 100 | Adjusts amount of pitch increase for input in % from RC throttle channel               |
 * | gimbal_pitch_rc_low_gain  | 10  | -100 - 100 | Adjusts amount of pitch increase for low pitch input in % from RC pitch channel        |
 * | gimbal_pitch_rc_high_gain | -20 | -100 - 100 | Adjusts amount of pitch decrease for high pitch input in % from RC pitch channel       |
 * | gimbal_yaw_rc_gain        | 20  | -100 - 100 | Adjusts amount of yaw in % from RC yaw channel                                         |
 * | gimbal_roll_gain          | 100 | -100 - 100 | Adjusts amount of roll in %                                                            |
 * | gimbal_roll_offset        | 0   | -100 - 100 | Adjust roll position for neutral roll input                                            |
 * | gimbal_roll_limit         | 100 |    0 - 100 | Adjusts roll range as a % of maximum                                                   |
 * | gimbal_pitch_gain         | 50  | -100 - 100 | Adjusts amount of pitch in %                                                           |
 * | gimbal_pitch_offset       | -10 | -100 - 100 | Adjust pitch position for neutral pitch input                                          |
 * | gimbal_pitch_low_limit    | 100 |    0 - 100 | Adjusts low pitch range as a % of maximum                                              |
 * | gimbal_pitch_high_limit   | 100 |    0 - 100 | Adjusts high pitch range as a % of maximum                                             |
 * | gimbal_yaw_gain           | 50  | -100 - 100 | Adjusts amount of yaw in %                                                             |
 * | gimbal_yaw_offset         | 0   | -100 - 100 | Adjust yaw position for neutral yaw input                                              |
 * | gimbal_yaw_limit          | 100 |    0 - 100 | Adjusts yaw range as a % of maximum                                                    |
 * | gimbal_stabilisation      | 0   |    0 - 7   | 0 no stabilisation, 1 pitch stabilisation, 2 roll/pitch stabilisation, 3 - 7 reserved  |
 * | gimbal_sensitivity        | 15  |  -16 - 15  | With higher values camera more rigidly tracks quad motion                              |
 * |---------------------------|-----|------------|----------------------------------------------------------------------------------------|
 *
 * To enable the gimbal control on a port set bit 18 thus:
 *
 *  serial <port> 262144 115200 57600 0 115200
 */

#define GIMBAL_CMD_NONE               0x00  //empty order
#define GIMBAL_CMD_ACCE_CALIB         0x01  //Acceleration Calibration
#define GIMBAL_CMD_GYRO_CALIB         0x02  //Gyro Calibration
#define GIMBAL_CMD_MAGN_CALIB         0x03  //Calibration of magnetometers
#define GIMBAL_CMD_AHRS_GZERO         0x04  //Zeroing the attitude angle
#define GIMBAL_CMD_QUIT_CALIB         0x05  //Exit current calibration

// Command Code Return Macro Definition
#define GIMBAL_STATUS_ERR             0x80

// This packet is sent to control the gimbal mode, sensivity and position
#define GIMBAL_CMD_S  0x5AA5
typedef struct {
    uint16_t opcode;
    unsigned mode:3;     // Mode [0~7] Only 0 1 2 modes are supported
    signed   sens:5;     // Sensitivity [-16~15]
    unsigned padding:4;
    signed   roll:12;   // Roll angle [-2048~2047] - ±180deg
    signed   pitch:12;  // Pich angle [-2048~2047] - ±180deg
    signed   yaw:12;    // Yaw angle [-2048~2047]  - ±180deg
    uint16_t crc;
}  __attribute__ ((__packed__)) gimbalCmd_t;

/* This packet is sent by the user to the head chase to realize the head
 * chase calibration and reset function
 */
#define GIMBAL_OPCODE_L  0x6EC5
typedef struct {
    uint16_t opcode;
    uint8_t  cmd;        // Command
    unsigned pwm:5;      // Pulse width data [0~31] => [0%~100%]
    unsigned iom:3;      // GPIO mode [0~7]
    unsigned mode:3;     // Mode [0~7] Only 0 1 2 modes are supported
    signed   sens:5;     // Sensitivity [-16~15]
    struct {
        unsigned chan:3;  // Channel [0~7] [CH56,CH57,CH58,CH67,CH68,CH78,CH78,CH78]
        unsigned revs:2;  // Reverse [0~3] [Normal, Horizontal Reverse, Vertical Reverse, All Reverse]
        unsigned rngx:2;  // Range [0~3] [90 degrees, 120 degrees, 180 degrees, 360 degrees]
        unsigned rngy:2;  // Range [0~3] [60 degrees, 90 degrees, 120 degrees, 180 degrees]
        signed   zerx:6;  // Zero [-32~31] [Resolution:5us]
        signed   zery:6;  // Zero [-32~31] [Resolution:5us]
        unsigned padding:11;
    } ppm;
    uint8_t padding2[5];
    uint16_t crc;
}  __attribute__ ((__packed__)) gimbalCal_t;

// Status reponse packet
#define GIMBAL_STAT  0x913A
typedef struct {
    uint16_t opcode;
    uint8_t cmd;        // Command response status
    uint8_t ctyp;       // Calibration type [0:Idle 1:Acceleration calibration 2:Gyroscope calibration 3:Magnetometer calibration]
    uint8_t cprg;       // Calibration progress [0%~100%]
    uint8_t cerr;       // Calibration error [0%~100%]
    uint8_t padding[8];
    uint16_t crc;
}  __attribute__ ((__packed__)) gimbalCalStatus_t;

// Expected input range from RC channels
#define GIMBAL_RC_SET_MIN   -500
#define GIMBAL_RC_SET_MAX    500

// Expect input range from head-tracker
#define GIMBAL_SET_MIN      -2047
#define GIMBAL_SET_MAX      2047
#define GIMBAL_SET_ROLL_MIN -900
#define GIMBAL_SET_ROLL_MAX  900

// Output range for full scale deflection to gimbal
#define GIMBAL_ROLL_MIN     -500
#define GIMBAL_ROLL_MAX     500
#define GIMBAL_PITCH_MIN    -1150
#define GIMBAL_PITCH_MAX    1750
#define GIMBAL_YAW_MIN      -2047
#define GIMBAL_YAW_MAX      2047

// Timeout after which headtracker input is ignored
#define GIMBAL_HT_TIMEOUT_US 250000

static struct {
    union {
        gimbalCmd_t gimbalCmd;
        uint8_t     bytes[sizeof(gimbalCmd_t)];
    } u;
} gimbalCmdIn;
static uint32_t gimbalInCount = 0;

static gimbalCmd_t gimbalCmdOut = {0};
static serialPort_t *gimbalSerialPort = NULL;

static uint16_t gimbalCrc(uint8_t *buf, uint32_t size)
{
    return __builtin_bswap16(crc16_ccitt_update(0x0000, buf, size));
}

// Set the gimbal position on each axis
static bool gimbalSet(int16_t headtracker_roll, int16_t headtracker_pitch, int16_t headtracker_yaw)
{
    DEBUG_SET(DEBUG_GIMBAL, 0, headtracker_roll);
    DEBUG_SET(DEBUG_GIMBAL, 1, headtracker_pitch);
    DEBUG_SET(DEBUG_GIMBAL, 2, headtracker_yaw);

    if (!gimbalSerialPort) {
        return false;
    }

    // Scale the expected incoming range to the max values accepted by the gimbal
    int16_t roll  = scaleRange(headtracker_roll,  GIMBAL_SET_ROLL_MIN, GIMBAL_SET_ROLL_MAX,
                                                  GIMBAL_ROLL_MIN * gimbalTrackConfig()->gimbal_roll_gain / 100,
                                                  GIMBAL_ROLL_MAX * gimbalTrackConfig()->gimbal_roll_gain / 100);
    int16_t pitch = scaleRange(headtracker_pitch, GIMBAL_SET_MIN, GIMBAL_SET_MAX,
                                                  GIMBAL_PITCH_MIN * gimbalTrackConfig()->gimbal_pitch_gain / 100,
                                                  GIMBAL_PITCH_MAX * gimbalTrackConfig()->gimbal_pitch_gain / 100);
    int16_t yaw   = scaleRange(headtracker_yaw,   GIMBAL_SET_MIN, GIMBAL_SET_MAX,
                                                  GIMBAL_YAW_MIN * gimbalTrackConfig()->gimbal_yaw_gain / 100,
                                                  GIMBAL_YAW_MAX * gimbalTrackConfig()->gimbal_yaw_gain / 100);


    // Scale the RC stick inputs and add
    roll  += scaleRange(rcData[ROLL] - rxConfig()->midrc, GIMBAL_RC_SET_MIN, GIMBAL_RC_SET_MAX,
                        GIMBAL_ROLL_MIN * gimbalTrackConfig()->gimbal_roll_rc_gain / 100,
                        GIMBAL_ROLL_MAX * gimbalTrackConfig()->gimbal_roll_rc_gain / 100);
    if (rcData[PITCH] < rxConfig()->midrc) {
        pitch += scaleRange(rcData[PITCH] - rxConfig()->midrc, GIMBAL_RC_SET_MIN, 0,
                            GIMBAL_PITCH_MAX * gimbalTrackConfig()->gimbal_pitch_rc_low_gain / 100,
                            0);
    } else {
        pitch += scaleRange(rcData[PITCH] - rxConfig()->midrc, 0, GIMBAL_RC_SET_MAX,
                            0,
                            GIMBAL_PITCH_MIN * gimbalTrackConfig()->gimbal_pitch_rc_high_gain / 100);
    }
    yaw   += scaleRange(rcData[YAW] - rxConfig()->midrc, GIMBAL_RC_SET_MIN, GIMBAL_RC_SET_MAX,
                        GIMBAL_YAW_MIN * gimbalTrackConfig()->gimbal_yaw_rc_gain / 100,
                        GIMBAL_YAW_MAX * gimbalTrackConfig()->gimbal_yaw_rc_gain / 100);

    pitch += scaleRange(rcData[THROTTLE] - rxConfig()->midrc, GIMBAL_RC_SET_MIN, GIMBAL_RC_SET_MAX,
                        GIMBAL_PITCH_MIN * gimbalTrackConfig()->gimbal_pitch_rc_thr_gain / 100,
                        GIMBAL_PITCH_MAX * gimbalTrackConfig()->gimbal_pitch_rc_thr_gain / 100);

    // Apply offsets
    roll  += GIMBAL_ROLL_MAX * gimbalTrackConfig()->gimbal_roll_offset / 100;
    pitch += GIMBAL_PITCH_MAX * gimbalTrackConfig()->gimbal_pitch_offset / 100;
    yaw   += GIMBAL_YAW_MAX * gimbalTrackConfig()->gimbal_yaw_offset / 100;

    DEBUG_SET(DEBUG_GIMBAL, 3, roll);
    DEBUG_SET(DEBUG_GIMBAL, 4, pitch);
    DEBUG_SET(DEBUG_GIMBAL, 5, yaw);

    // Constrain to set limits
    gimbalCmdOut.roll  = constrain(roll, GIMBAL_ROLL_MIN * gimbalTrackConfig()->gimbal_roll_limit / 100,
                                         GIMBAL_ROLL_MAX * gimbalTrackConfig()->gimbal_roll_limit / 100);
    gimbalCmdOut.pitch = constrain(pitch, GIMBAL_PITCH_MIN * gimbalTrackConfig()->gimbal_pitch_low_limit / 100,
                                          GIMBAL_PITCH_MAX * gimbalTrackConfig()->gimbal_pitch_high_limit / 100);
    gimbalCmdOut.yaw   = constrain(yaw, GIMBAL_YAW_MIN * gimbalTrackConfig()->gimbal_yaw_limit / 100,
                                        GIMBAL_YAW_MAX * gimbalTrackConfig()->gimbal_yaw_limit / 100);

    gimbalCmdOut.mode = gimbalTrackConfig()->gimbal_stabilisation;
    gimbalCmdOut.sens = gimbalTrackConfig()->gimbal_sensitivity;

    uint16_t crc = gimbalCrc((uint8_t *)&gimbalCmdOut, sizeof(gimbalCmdOut) - 2);

    gimbalCmdOut.crc = crc;

    return true;
}

// Gimbal updates should be sent at 100Hz or the gimbal will self center after approx. 2 seconds
void gimbalUpdate(timeUs_t currentTimeUs)
{
    static enum {GIMBAL_OP1, GIMBAL_OP2, GIMBAL_CMD} gimbalParseState = GIMBAL_OP1;
    static timeUs_t lastRxTimeUs = 0;

    if (!gimbalSerialPort) {
        setTaskEnabled(TASK_GIMBAL, false);
        return;
    }

    // Read bytes from the VTX gimbal serial data stream

    uint32_t bytes =  serialRxBytesWaiting(gimbalSerialPort);

    if (bytes > 0) {
        lastRxTimeUs = currentTimeUs;
        while (bytes--) {
            uint8_t inData = serialRead(gimbalSerialPort);

            // If the packet is a gimbalCmd_t structure then parse, otherwise pass through
            switch (gimbalParseState) {
            default:
            case GIMBAL_OP1:
                if (inData == (GIMBAL_CMD_S & 0xff)) {
                    gimbalParseState = GIMBAL_OP2;
                } else {
                    serialWrite(gimbalSerialPort, inData);
                }
                break;
            case GIMBAL_OP2:
                if (inData == ((GIMBAL_CMD_S >> 8) & 0xff)) {
                    gimbalParseState = GIMBAL_CMD;
                    gimbalInCount = sizeof(gimbalCmdIn.u.gimbalCmd.opcode);
                } else {
                    serialWrite(gimbalSerialPort, GIMBAL_CMD_S && 0xff);
                    serialWrite(gimbalSerialPort, inData);
                    gimbalParseState = GIMBAL_OP1;
                }
                break;
            case GIMBAL_CMD:
                gimbalCmdIn.u.bytes[gimbalInCount++] = inData;
                if (gimbalInCount == sizeof(gimbalCmdIn.u.gimbalCmd)) {
                    gimbalCmdOut = gimbalCmdIn.u.gimbalCmd;
                    gimbalSet(gimbalCmdIn.u.gimbalCmd.roll, gimbalCmdIn.u.gimbalCmd.pitch, gimbalCmdIn.u.gimbalCmd.yaw);
                    serialWriteBuf(gimbalSerialPort, (uint8_t *)&gimbalCmdOut, sizeof(gimbalCmdOut));
                    gimbalParseState = GIMBAL_OP1;
                }
                break;
            }
        }
    } else if (cmpTimeUs(currentTimeUs, lastRxTimeUs) > GIMBAL_HT_TIMEOUT_US) {
        gimbalSet(0, 0, 0);
        serialWriteBuf(gimbalSerialPort, (uint8_t *)&gimbalCmdOut, sizeof(gimbalCmdOut));
    }
}

bool gimbalInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_GIMBAL);
    if (!portConfig) {
        return false;
    }

    // Serial communications is 115200 8N1
    gimbalSerialPort = openSerialPort(portConfig->identifier, FUNCTION_GIMBAL,
                                      NULL, NULL,
                                      115200, MODE_RXTX, SERIAL_STOPBITS_1 | SERIAL_PARITY_NO);

    gimbalCmdIn.u.gimbalCmd.opcode = GIMBAL_CMD_S;
    gimbalCmdOut.opcode = GIMBAL_CMD_S;

    // Set gimbal initial position
    gimbalSet(0, 0, 0);

    return true;
}

#endif
