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
#include <string.h>

#include "platform.h"

#ifdef USE_RANGEFINDER_UPT1

#include "build/debug.h"
#include "build/build_config.h"

#include "io/serial.h"

#include "drivers/time.h"
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_upt1.h"
#ifdef USE_OPTICALFLOW
#include "drivers/opticalflow/opticalflow.h"
#include "sensors/rangefinder.h"
#endif

// UP-T1-001-Plus is an optical flow + laser ranging 2-in-1 module
// Module manual
//  https://github.com/Upixels-China/Upixels_Optical_flow/blob/main/docs/Upixels_Tx/T1%26T1_Plus/Optical%20flow%20laser%202-in-1%20module%20specification(UP-T101-Plus)%4020240315.pdf
// Protocol manual
//  https://github.com/Upixels-China/Upixels_Optical_flow/blob/main/docs/Upixels_Tx/T1%26T1_Plus/(实操英文版)Protocol%20Introduction%20User%20manualT1(001、Plus）、302GS和T2_V1.0.9_20240506.pdf
//
// The upixels protocol is selected and used as described below:
// Packet structure (14 bytes):
// Byte 1:    0xFE - Packet header
// Byte 2:    0x0A - Packet length (10 data bytes)
// Byte 3-4:  flow_x_integral (LSB, MSB) - radians*10000
// Byte 5-6:  flow_y_integral (LSB, MSB) - radians*10000
// Byte 7-8:  integration_timespan (LSB, MSB) - microseconds
// Byte 9-10: Laser distance (LSB, MSB) - millimeters
// Byte 11:   valid status (0x00=unavailable, 0xF5=available)
// Byte 12:   Laser confidence (0-100%)
// Byte 13:   XOR checksum (XOR of bytes 3-12)
// Byte 14:   0x55 - Packet end

#define UPT1_FRAME_HEADER      0xFE        // Frame start byte
#define UPT1_FRAME_LENGTH      0x0A        // Frame length indicator (10 data bytes)
#define UPT1_FRAME_FOOTER      0x55        // Frame end byte
#define UPT1_DATA_LENGTH       10          // Number of data bytes
#define UPT1_TIMEOUT_MS        200         // Timeout for frame reception
#define UPT1_BAUDRATE          115200      // UART baud rate from spec

// Range specifications from device specification
#define UPT1_RANGE_MIN         25          // Minimum range 2.5cm (25mm) in mm
#define UPT1_RANGE_MAX         4000        // Maximum range 4m (4000mm) in mm
#define UPT1_DETECTION_CONE_DECIDEGREES 50 // Field angle 5° horizontal (50 decidegrees)

static serialPort_t *upt1SerialPort = NULL;

typedef enum {
    UPT1_FRAME_WAIT_RESET,
    UPT1_FRAME_STATE_WAIT_HEADER,
    UPT1_FRAME_STATE_WAIT_LENGTH,
    UPT1_FRAME_STATE_READING_DATA,
    UPT1_FRAME_STATE_WAIT_CKSUM,
    UPT1_FRAME_STATE_WAIT_FOOTER,
} upt1FrameState_e;

// Number of bytes of incoming data before sending the command to set protocol
// This is the length of the version string
#define UPT1_STARTUP_BYTE_COUNT 22

static upt1FrameState_e upt1FrameState;
static uint8_t upt1Frame[UPT1_DATA_LENGTH];  // 10 data bytes
static uint8_t upt1ReceivePosition;

static int32_t upt1Value;
static uint16_t upt1Errors = 0;

#ifdef USE_OPTICALFLOW
#define UPT1_OPTICALFLOW_MIN_RANGE 80  // mm (adjust based on device spec)
#define UPT1_OPFLOW_MIN_QUALITY_THRESHOLD 30  // Minimum quality threshold (adjust based on spec)

static opticalflowData_t upt1OpticalflowSensorData = {0};
static bool hasUPT1RFNewData = false;
static int32_t upt1RFDistanceMm = RANGEFINDER_NO_NEW_DATA;
static uint32_t upt1RFTimestampUs = 0;
#endif

void rangefinderUPT1Init(rangefinderDev_t *dev)
{
    UNUSED(dev);

    upt1FrameState = UPT1_FRAME_WAIT_RESET ;
    upt1ReceivePosition = 0;
    upt1Value = RANGEFINDER_OUT_OF_RANGE;
}

void rangefinderUPT1Update(rangefinderDev_t *dev)
{
    UNUSED(dev);
    static uint32_t upt1_byte_count = 0;

    if (upt1SerialPort == NULL) {
        return;
    }

    while (serialRxBytesWaiting(upt1SerialPort)) {
        uint8_t c = serialRead(upt1SerialPort);
        upt1_byte_count++;

        switch (upt1FrameState) {
        case UPT1_FRAME_WAIT_RESET:
            // Wait until data is flowing
            if (upt1_byte_count >= UPT1_STARTUP_BYTE_COUNT) {
                // Send initialization command to configure device protocol
                const char *initCmd = "<set protocol upixels>";
                serialWriteBuf(upt1SerialPort, (const uint8_t *)initCmd, strlen(initCmd));
                upt1FrameState = UPT1_FRAME_STATE_WAIT_HEADER;
            }
            break;

        case UPT1_FRAME_STATE_WAIT_HEADER:
            if (c == UPT1_FRAME_HEADER) {
                upt1FrameState = UPT1_FRAME_STATE_WAIT_LENGTH;
            }
            break;

        case UPT1_FRAME_STATE_WAIT_LENGTH:
            if (c == UPT1_FRAME_LENGTH) {
                upt1FrameState = UPT1_FRAME_STATE_READING_DATA;
                upt1ReceivePosition = 0;
            } else {
                upt1FrameState = UPT1_FRAME_STATE_WAIT_HEADER;
            }
            break;

        case UPT1_FRAME_STATE_READING_DATA:
            upt1Frame[upt1ReceivePosition++] = c;
            if (upt1ReceivePosition == UPT1_DATA_LENGTH) {
                upt1FrameState = UPT1_FRAME_STATE_WAIT_CKSUM;
            }
            break;

        case UPT1_FRAME_STATE_WAIT_CKSUM:
            {
                // Calculate XOR checksum of data bytes (bytes 3-12 in the packet, which are bytes 0-9 in our buffer)
                uint8_t cksum = 0;
                for (int i = 0; i < UPT1_DATA_LENGTH; i++) {
                    cksum ^= upt1Frame[i];
                }

                if (c == cksum) {
                    // Valid checksum - proceed to wait for footer
                    upt1FrameState = UPT1_FRAME_STATE_WAIT_FOOTER;
                } else {
                    // Checksum error - discard frame
                    ++upt1Errors;
                    upt1FrameState = UPT1_FRAME_STATE_WAIT_HEADER;
                    upt1ReceivePosition = 0;
                }
            }
            break;

        case UPT1_FRAME_STATE_WAIT_FOOTER:
            if (c == UPT1_FRAME_FOOTER) {
                // Complete valid frame received - parse data
                // Byte 0-1: flow_x_integral (LSB, MSB)
                // Byte 2-3: flow_y_integral (LSB, MSB)
                // Byte 4-5: integration_timespan (LSB, MSB)
                // Byte 6-7: Laser distance (LSB, MSB) in mm
                // Byte 8:   valid status (0x00=unavailable, 0xF5=available)
                // Byte 9:   Laser confidence (0-100%)

                const int16_t flow_x = (int16_t)(upt1Frame[0] | (upt1Frame[1] << 8));
                const int16_t flow_y = (int16_t)(upt1Frame[2] | (upt1Frame[3] << 8));
                const uint16_t integration_time = upt1Frame[4] | (upt1Frame[5] << 8);
                const uint16_t distanceMm = upt1Frame[6] | (upt1Frame[7] << 8);
                const uint8_t validStatus = upt1Frame[8];
                const uint8_t confidence = upt1Frame[9];

                DEBUG_SET(DEBUG_LIDAR_TF, 0, distanceMm);
                DEBUG_SET(DEBUG_LIDAR_TF, 1, confidence);
                DEBUG_SET(DEBUG_LIDAR_TF, 2, flow_x);
                DEBUG_SET(DEBUG_LIDAR_TF, 3, flow_y);
                DEBUG_SET(DEBUG_LIDAR_TF, 4, validStatus);
                DEBUG_SET(DEBUG_LIDAR_TF, 5, integration_time);

                // Process rangefinder data
                if (distanceMm >= UPT1_RANGE_MIN && distanceMm <= UPT1_RANGE_MAX) {
                    upt1Value = distanceMm / 10;  // Convert mm to cm
#ifdef USE_OPTICALFLOW
                    upt1RFDistanceMm = distanceMm;
                    upt1RFTimestampUs = micros();
                    hasUPT1RFNewData = true;
#endif
                } else {
                    upt1Value = RANGEFINDER_OUT_OF_RANGE;
                }

#ifdef USE_OPTICALFLOW
                // Process optical flow data
                if (validStatus == 0xF5) {  // 0xF5 indicates valid optical flow data
                    upt1OpticalflowSensorData.timeStampUs = micros();
                    // Convert from radians*10000 to radians
                    upt1OpticalflowSensorData.flowRate.x = (float)flow_x / 10000.0f;
                    upt1OpticalflowSensorData.flowRate.y = (float)flow_y / 10000.0f;
                    upt1OpticalflowSensorData.quality = confidence;  // Already 0-100%

                    // Check if rangefinder distance is too close
                    if (distanceMm < UPT1_OPTICALFLOW_MIN_RANGE) {
                        upt1OpticalflowSensorData.quality = OPTICALFLOW_OUT_OF_RANGE;
                    }
                } else {
                    // Optical flow data not available
                    upt1OpticalflowSensorData.quality = OPTICALFLOW_NO_NEW_DATA;
                }
#endif
            } else {
                // Invalid footer
                ++upt1Errors;
            }

            upt1FrameState = UPT1_FRAME_STATE_WAIT_HEADER;
            upt1ReceivePosition = 0;
            break;
        }
    }
}

// Return most recent device output in cm
int32_t rangefinderUPT1GetDistance(rangefinderDev_t *dev)
{
    UNUSED(dev);

    if (hasUPT1RFNewData) {
        hasUPT1RFNewData = false;
        return upt1Value;
    } else {
        return RANGEFINDER_NO_NEW_DATA;
    }
}

bool rangefinderUPT1Detect(rangefinderDev_t *dev)
{
    // UP-T1-001-Plus typically uses UART communication
    // Check if serial port is configured for rangefinder/optical flow
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_LIDAR_TF);

    if (!portConfig) {
        return false;
    }

    // Open serial port at 115200n1 (per UP-T1-001-Plus specification)
    upt1SerialPort = openSerialPort(portConfig->identifier, FUNCTION_LIDAR_TF, NULL, NULL, UPT1_BAUDRATE, MODE_RXTX, 0);

    if (upt1SerialPort == NULL) {
        return false;
    }

    dev->delayMs = 20;  // 50Hz frame rate = 20ms period
    dev->maxRangeCm = UPT1_RANGE_MAX / 10;  // Convert mm to cm (400cm = 4m)
    dev->detectionConeDeciDegrees = UPT1_DETECTION_CONE_DECIDEGREES;
    dev->detectionConeExtendedDeciDegrees = UPT1_DETECTION_CONE_DECIDEGREES;

    dev->init = &rangefinderUPT1Init;
    dev->update = &rangefinderUPT1Update;
    dev->read = &rangefinderUPT1GetDistance;

    return true;
}

#ifdef USE_OPTICALFLOW
// Optical flow functions for UP-T1-001-Plus

static void upt1OpticalflowInit(opticalflowDev_t *dev)
{
    UNUSED(dev);
}

static void upt1OpticalflowUpdate(opticalflowDev_t *dev)
{
    UNUSED(dev);
    // Optical flow data is parsed from the same serial stream
    // This is called by the optical flow task, but actual data parsing
    // happens in rangefinderUPT1Update() which processes all incoming frames
}

static void upt1OpticalflowGetData(opticalflowDev_t *dev, opticalflowData_t *result)
{
    UNUSED(dev);

    *result = upt1OpticalflowSensorData;
}

bool upt1OpticalflowDetect(opticalflowDev_t *dev)
{
    // UP-T1-001-Plus uses the same serial port for both rangefinder and optical flow
    // Check if serial port is configured (should already be open from rangefinder detection)
    if (upt1SerialPort == NULL) {
        const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_LIDAR_TF);
        if (!portConfig) {
            return false;
        }
        upt1SerialPort = openSerialPort(portConfig->identifier, FUNCTION_LIDAR_TF, NULL, NULL, UPT1_BAUDRATE, MODE_RXTX, 0);
        if (upt1SerialPort == NULL) {
            return false;
        }
    }

    dev->delayMs = 20;  // 50Hz frame rate = 20ms period
    dev->minRangeCm = UPT1_OPTICALFLOW_MIN_RANGE / 10;  // Convert mm to cm
    dev->minQualityThreshold = UPT1_OPFLOW_MIN_QUALITY_THRESHOLD;

    dev->init = &upt1OpticalflowInit;
    dev->update = &upt1OpticalflowUpdate;
    dev->read = &upt1OpticalflowGetData;

    return true;
}

void upt1OpticalflowReceiveNewData(const uint8_t *bufferPtr)
{
    // This function is provided for compatibility but not actively used
    // Optical flow data is parsed directly in rangefinderUPT1Update()
    // from the combined rangefinder+optical flow packet
    UNUSED(bufferPtr);}
#endif // USE_OPTICALFLOW
#endif // USE_RANGEFINDER_UPT1
