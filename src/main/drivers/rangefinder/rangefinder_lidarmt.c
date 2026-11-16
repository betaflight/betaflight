/*
 * This file is part of Betaflight and INAV
 *
 * Betaflight and INAV are free software. You can
 * redistribute this software and/or modify this software under
 * the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * Betaflight and INAV are distributed in the hope that
 * they will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RANGEFINDER_MT

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/time.h"

#include "drivers/rangefinder/rangefinder_lidarmt.h"
#include "sensors/rangefinder.h"

#include "io/serial.h"

#include "drivers/opticalflow/opticalflow.h"
#include "sensors/opticalflow.h"

// MicoAir MTF-01 family, driven in the module's own Micolink protocol.
//
// See https://github.com/micoair/MTF-01_USER_MANUAL for the module itself.  It
// names the protocol but publishes no frame format, switching being a job for
// the vendor's Windows tool, so the format here is taken from a reverse
// engineered reference, https://github.com/GlassOnTin/mtf01-protocol, whose
// author verified the framing, the heartbeat exchange and the streaming
// payload against an MTF-01 on hardware 10, firmware 1.1.0.
//
// Driving it this way rather than over MSP is what gets the measurement right,
// not merely tidier:
//
//  - mspSerialProcess() takes one packet per TASK_SERIAL tick, and the module
//    fills more than that, so frames are dropped whenever that task is behind.
//  - Micolink reports flow as a velocity, already divided by the window the
//    module measured it over.  Read over MSP the rate had to be recovered by
//    dividing by the gap between arrivals, which made the flow gain move with
//    how fast the FC happened to be draining the port.
//  - Range and flow travel in the same frame, so the range always describes the
//    instant the flow was measured.
#define MT_BAUDRATE                 115200

// Frame: head, device id, system id, message id, sequence, payload length,
// payload, then a checksum that is the 8-bit sum of everything before it.
#define MICOLINK_HEAD               0xEF
#define MICOLINK_MAX_PAYLOAD        64
#define MICOLINK_HEADER_SIZE        6
#define MICOLINK_MAX_FRAME          (MICOLINK_HEADER_SIZE + MICOLINK_MAX_PAYLOAD + 1)

#define MICOLINK_DEV_PC_GCS         1
#define MICOLINK_MSG_HEARTBEAT      1
#define MICOLINK_MSG_RANGE_SENSOR   81

// RANGE_SENSOR payload, 20 bytes, little-endian:
//   0  time_ms       uint32
//   4  distance      uint32, millimetres; 0 means no reading
//   8  strength      uint8
//   9  precision     uint8
//  10  dis_status    uint8, 1 when the distance is usable
//  11  reserved
//  12  flow_vel_x    int16, cm/s at 1m of height
//  14  flow_vel_y    int16, cm/s at 1m of height
//  16  flow_quality  uint8
//  17  flow_status   uint8, 1 when the flow is usable
//  18  reserved
#define MICOLINK_RANGE_SENSOR_LEN   20
#define MICOLINK_STATUS_OK          1

// A velocity quoted at one metre of height is an angular rate in disguise: at
// that height ground speed and angular rate are numerically equal, so dividing
// out the centimetres leaves radians per second, whatever the real height.
#define MT_FLOW_CM_S_PER_RAD_S      100.0f

// The module picks its output protocol from the traffic it sees when left at
// the factory AUTO setting, which is why it answered MSP for as long as this
// driver spoke MSP at it.  Heartbeats are what ask it for Micolink instead; the
// reference found a unit answered a rapid burst rather than the vendor tool's
// 1Hz, so one goes out per task cycle until the module starts streaming.
// Resumed if it ever falls silent for this long, so that a module restarting
// under a running FC is asked again.
#define MT_HEARTBEAT_SILENCE_MS     1000
#define MT_HEARTBEAT_PAYLOAD_LEN    13
// What the vendor tool puts in a host heartbeat; only the model hint is
// anything but a constant, and the module answers a heartbeat whatever it says.
#define MT_HEARTBEAT_MODEL_HINT     30

#define MT_OPTICALFLOW_MIN_RANGE 80  // mm
#define MT_OPFLOW_MIN_QUALITY_THRESHOLD 30
// Sensor latency: a flow sample reports motion from this long ago (integration
// window, image processing and transport), so the rotation compensation uses a
// gyro reading of that age. It is a property of the sensor pipeline, so it is
// an absolute time rather than a count of frames or samples, which would make
// it depend on the rate the device or the flow task happens to run at. One gyro
// reading is kept per flow sample, so the history spans
// MT_OPFLOW_GYRO_DELAY_US / frame period entries.
//
// Measured on an MTF01 by correlating reported flow against gyro over
// rotation-only moves read over this protocol: the flow best matches the gyro
// averaged over the 10ms window ending 25-30ms ago, so the centre of what it
// reports is about 30ms old, and replaying the matching below agrees at 30ms
// with little to choose between 25ms and 40ms.
//
// An earlier 65000 came from the same measurement made over MSP, where frames
// queued in the port and arrived stale; that queueing, not the sensor, was most
// of the difference, exactly as it was on the PB04.
#define MT_OPFLOW_GYRO_DELAY_US 30000

static opticalflowData_t opticalflowSensorData = {0};

static bool hasRFNewData = false;
static mtRangefinderData_t rfSensorData = {RANGEFINDER_NO_NEW_DATA, 0};
static const MTRangefinderConfig * deviceConf = NULL;

static serialPort_t *mtSerialPort = NULL;
static timeMs_t mtLastFrameMs = 0;
static uint8_t mtHeartbeatSeq = 0;

static uint8_t mtFrameBuf[MICOLINK_MAX_FRAME];
static unsigned mtFramePos = 0;
static unsigned mtFrameSize = 0;

// Flow arrives as a rate already, so frames are averaged rather than summed:
// several can be waiting when a task runs late, and each is its own estimate of
// the same quantity.
static int32_t flowSumX = 0;
static int32_t flowSumY = 0;
static uint32_t flowQualitySum = 0;
static unsigned flowFrames = 0;
static unsigned flowPackets = 0;
static timeUs_t flowLatestUs = 0;

// Initialize the table with values for each rangefinder type
static const MTRangefinderConfig rangefinderConfigs[] = {
    { .deviceType = RANGEFINDER_MTF01,  .delayMs = 10, .maxRangeCm = 800  },
    { .deviceType = RANGEFINDER_MTF02,  .delayMs = 20, .maxRangeCm = 250  },
    { .deviceType = RANGEFINDER_MTF01P, .delayMs = 10, .maxRangeCm = 1200 },
    { .deviceType = RANGEFINDER_MTF02P, .delayMs = 20, .maxRangeCm = 600  },
};

const MTRangefinderConfig* getMTRangefinderDeviceConf(rangefinderType_e mtRangefinderToUse) {
    for (const MTRangefinderConfig* cfg =  rangefinderConfigs; cfg < ARRAYEND(rangefinderConfigs); cfg++) {
        if (cfg->deviceType == mtRangefinderToUse) {
            return cfg;
        }
    }
    return NULL;
}

static uint32_t mtReadU32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int16_t mtReadS16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/*
 * Ask the module to speak Micolink.
 *
 * Nothing is written once it is streaming; the heartbeat is only how a module
 * sitting in AUTO is told what is listening to it.
 */
static void mtSendHeartbeat(void) {
    uint8_t frame[MICOLINK_HEADER_SIZE + MT_HEARTBEAT_PAYLOAD_LEN + 1];
    const uint32_t nowMs = millis();
    unsigned i = 0;

    frame[i++] = MICOLINK_HEAD;
    frame[i++] = MICOLINK_DEV_PC_GCS;
    frame[i++] = 0;                         // system id
    frame[i++] = MICOLINK_MSG_HEARTBEAT;
    frame[i++] = mtHeartbeatSeq++;
    frame[i++] = MT_HEARTBEAT_PAYLOAD_LEN;

    frame[i++] = nowMs & 0xFF;
    frame[i++] = (nowMs >> 8) & 0xFF;
    frame[i++] = (nowMs >> 16) & 0xFF;
    frame[i++] = (nowMs >> 24) & 0xFF;
    frame[i++] = MT_HEARTBEAT_MODEL_HINT;
    frame[i++] = 0;
    frame[i++] = 1;
    frame[i++] = 0;
    frame[i++] = 1;
    frame[i++] = 0;                         // trailing uint32, always zero
    frame[i++] = 0;
    frame[i++] = 0;
    frame[i++] = 0;

    uint8_t checksum = 0;
    for (unsigned k = 0; k < i; k++) {
        checksum += frame[k];
    }
    frame[i++] = checksum;

    serialWriteBuf(mtSerialPort, frame, i);
}

static void mtHandleRangeSensor(const uint8_t *payload) {
    const timeUs_t currentTimeUs = micros();

    const uint32_t distanceMm = mtReadU32(&payload[4]);
    const bool distanceValid = (payload[10] == MICOLINK_STATUS_OK)
                               && (distanceMm > 0)
                               && (distanceMm <= (uint32_t)deviceConf->maxRangeCm * 10);

    rfSensorData.distanceMm = distanceValid ? (int32_t)distanceMm : -1;
    rfSensorData.timestampUs = currentTimeUs;
    hasRFNewData = true;

    flowLatestUs = currentTimeUs;
    flowPackets++;

    // An unusable frame contributes nothing rather than a zero, which would
    // otherwise be averaged in as though the craft had stopped moving.
    if (payload[17] == MICOLINK_STATUS_OK) {
        flowSumX += mtReadS16(&payload[12]);
        flowSumY += mtReadS16(&payload[14]);
        flowQualitySum += payload[16];
        flowFrames++;
    }
}

static void mtParseByte(uint8_t c) {
    if (mtFramePos == 0 && c != MICOLINK_HEAD) {
        return;  // hunting for a frame head
    }

    mtFrameBuf[mtFramePos++] = c;

    if (mtFramePos == MICOLINK_HEADER_SIZE) {
        const uint8_t payloadLength = mtFrameBuf[5];
        if (payloadLength > MICOLINK_MAX_PAYLOAD) {
            mtFramePos = 0;  // longer than the protocol allows, so not a head
            return;
        }
        mtFrameSize = MICOLINK_HEADER_SIZE + payloadLength + 1;
    }

    if (mtFramePos < MICOLINK_HEADER_SIZE || mtFramePos < mtFrameSize) {
        return;
    }
    mtFramePos = 0;

    uint8_t checksum = 0;
    for (unsigned i = 0; i < mtFrameSize - 1; i++) {
        checksum += mtFrameBuf[i];
    }
    if (checksum != mtFrameBuf[mtFrameSize - 1]) {
        return;
    }

    mtLastFrameMs = millis();

    if (mtFrameBuf[3] == MICOLINK_MSG_RANGE_SENSOR && mtFrameBuf[5] >= MICOLINK_RANGE_SENSOR_LEN) {
        mtHandleRangeSensor(&mtFrameBuf[MICOLINK_HEADER_SIZE]);
    }
}

/*
 * Drain whatever the module has sent.
 *
 * Both halves of the module arrive in the one frame on the one UART, so
 * whichever of the two tasks runs first reads for both.  Taking the whole
 * buffer each time is what keeps this independent of how often that happens.
 */
static void mtPoll(void) {
    if (!mtSerialPort) {
        return;
    }

    while (serialRxBytesWaiting(mtSerialPort)) {
        mtParseByte(serialRead(mtSerialPort));
    }

    if (cmp32(millis(), mtLastFrameMs + MT_HEARTBEAT_SILENCE_MS) >= 0) {
        mtSendHeartbeat();
    }
}

/*
 * Both halves are the one module on the one UART, so whichever is detected
 * first opens the port and the other reuses it: openSerialPort() rejects a port
 * already in use and would otherwise clear the handle.
 */
static bool mtOpenPort(serialPortIdentifier_e identifier) {
    if (mtSerialPort) {
        return true;
    }

    if (identifier == SERIAL_PORT_NONE) {
        return false;
    }

    mtSerialPort = openSerialPort(identifier, FUNCTION_LIDAR, NULL, NULL,
                                  MT_BAUDRATE, MODE_RXTX, SERIAL_NOT_INVERTED);
    if (!mtSerialPort) {
        return false;
    }

    mtLastFrameMs = millis() - MT_HEARTBEAT_SILENCE_MS;  // ask straight away

    return true;
}

static void mtRangefinderInit(rangefinderDev_t * dev) {
    UNUSED(dev);
}

static void mtRangefinderUpdate(rangefinderDev_t * dev) {
    UNUSED(dev);

    mtPoll();
}

static int32_t mtRangefinderGetDistance(rangefinderDev_t * dev) {
    UNUSED(dev);
    if (hasRFNewData) {
        hasRFNewData = false;
        return (rfSensorData.distanceMm >= 0) ? (rfSensorData.distanceMm / 10) : RANGEFINDER_OUT_OF_RANGE;
    } else {
        return RANGEFINDER_NO_NEW_DATA;
    }
}

bool mtRangefinderDetect(rangefinderDev_t * dev, rangefinderType_e mtRangefinderToUse) {
    deviceConf = getMTRangefinderDeviceConf(mtRangefinderToUse);
    if (!deviceConf) {
        return false;
    }

    if (!mtOpenPort((serialPortIdentifier_e)rangefinderConfig()->rangefinder_uart)) {
        return false;
    }

    dev->delayMs    = deviceConf->delayMs;
    dev->maxRangeCm = deviceConf->maxRangeCm;

    dev->detectionConeDeciDegrees = RANGEFINDER_MT_DETECTION_CONE_DECIDEGREES;
    dev->detectionConeExtendedDeciDegrees = RANGEFINDER_MT_DETECTION_CONE_DECIDEGREES;

    dev->init   = &mtRangefinderInit;
    dev->update = &mtRangefinderUpdate;
    dev->read   = &mtRangefinderGetDistance;
    return true;
}

static void mtOpticalflowPublish(void);

static void mtOpticalflowInit(opticalflowDev_t * dev) {
    UNUSED(dev);

    flowSumX = 0;
    flowSumY = 0;
    flowQualitySum = 0;
    flowFrames = 0;
    flowPackets = 0;
    flowLatestUs = micros();
    opticalflowSensorData.timeStampUs = flowLatestUs;
}

// Called once per flow task cycle, immediately before the data is read.
static void mtOpticalflowUpdate(opticalflowDev_t * dev) {
    UNUSED(dev);

    mtPoll();
    mtOpticalflowPublish();
}

static void mtOpticalflowGetData(opticalflowDev_t * dev, opticalflowData_t * result) {
    UNUSED(dev);
    *result = opticalflowSensorData;
}

bool mtOpticalflowDetect(opticalflowDev_t * dev, rangefinderType_e mtRangefinderToUse) {
    deviceConf = getMTRangefinderDeviceConf(mtRangefinderToUse);
    if (!deviceConf) {
        return false;
    }

    if (!mtOpenPort((serialPortIdentifier_e)opticalflowConfig()->opticalflow_uart)) {
        return false;
    }

    dev->delayMs = deviceConf->delayMs;
    dev->minRangeCm = MT_OPTICALFLOW_MIN_RANGE / 10;
    dev->minQualityThreshold = MT_OPFLOW_MIN_QUALITY_THRESHOLD;
    dev->gyroDelayUs = MT_OPFLOW_GYRO_DELAY_US;

    dev->init   = &mtOpticalflowInit;
    dev->update = &mtOpticalflowUpdate;
    dev->read   = &mtOpticalflowGetData;

    return true;
}

static void mtOpticalflowPublish(void) {
    if (flowPackets == 0) {
        return;  // nothing arrived; the previous sample and its timestamp stand
    }

    const timeUs_t windowEndUs = flowLatestUs;
    // Elapsed time no longer scales the flow - the module has already done that
    // - so it serves only as a staleness guard.
    const float dt_s = (float)cmp32(windowEndUs, opticalflowSensorData.timeStampUs) / 1e6f;

    const bool dtValid = (dt_s > 0.0f && dt_s < 1.0f);
    const bool flowValid = (flowFrames > 0);
    // The range travels in the same frame as the flow, so it always describes
    // the moment the flow was measured; only its value has to be checked.
    const bool rangeValid = (rfSensorData.distanceMm >= MT_OPTICALFLOW_MIN_RANGE);

    opticalflowSensorData.timeStampUs = windowEndUs;
    opticalflowSensorData.flowDtValid = dtValid;
    opticalflowSensorData.rangeValid  = rangeValid;
    opticalflowSensorData.flowValid   = dtValid && flowValid;

    if (dtValid && flowValid) {
        opticalflowSensorData.flowRate.x = (float)flowSumX / flowFrames / MT_FLOW_CM_S_PER_RAD_S;
        opticalflowSensorData.flowRate.y = (float)flowSumY / flowFrames / MT_FLOW_CM_S_PER_RAD_S;
        opticalflowSensorData.quality    = (flowQualitySum / flowFrames) * 100 / 255;
    } else {
        opticalflowSensorData.flowRate.x = 0;
        opticalflowSensorData.flowRate.y = 0;
        opticalflowSensorData.quality    = 0;
    }

    if (!rangeValid) {
        opticalflowSensorData.quality = OPTICALFLOW_OUT_OF_RANGE;
    }

    flowSumX = 0;
    flowSumY = 0;
    flowQualitySum = 0;
    flowFrames = 0;
    flowPackets = 0;
}

#endif // USE_RANGEFINDER_MT
