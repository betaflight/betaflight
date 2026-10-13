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

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RANGEFINDER_PB04

#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_pb04.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "sensors/rangefinder.h"

#ifdef USE_OPTICALFLOW_PB04
#include "drivers/opticalflow/opticalflow.h"
#include "sensors/opticalflow.h"
#endif

// Shenzhen Bamu PB04 optical flow + dTOF dual function module, driven in its
// own BAMU protocol.  "PB04 Optical Flow TOF Dual Function Module" datasheet
// V1.1.3, section 5.1.1.
//
// This driver opens the module's UART itself and reads it, rather than letting
// the MSP subsystem parse the port.  The module also speaks MSPV2, and driving
// it that way looks tempting because the MTF-01 message ids carry the same
// payloads, but it costs more than it saves:
//
//  - mspSerialProcess() takes one packet per TASK_SERIAL tick, and the module
//    fills two of those every 20ms, so at the default serial_update_rate_hz
//    half the frames are dropped.  What is then read has queued in the receive
//    buffer and is several frames stale, which no driver can detect or correct.
//  - MSPV2 omits the integration window the native frame carries, leaving the
//    span the motion was measured over to be assumed rather than read.
//  - The native frame carries range and flow together, so the range always
//    describes the same instant as the flow it is divided by.
//
// Read here, every frame is taken whatever else the scheduler is doing, and
// each one states the window it covers.
#define PB04_BAUDRATE           115200

// Protocol selection, datasheet section 5.  Saved in the module, so it survives
// a power cycle and a module already in this protocol needs no asking.  Sent
// once per boot all the same, so that one arriving set to MSPV2 or MAVLINK -
// how it left the factory, or how a previous owner left it - comes up working
// rather than silent.
#define PB04_PROTOCOL_COMMAND   "<Agreement:0>"
// How long to allow for the module to initialise before asking blind.  The
// datasheet gives it up to 3 seconds and says anything arriving before then is
// dropped; the wait only applies when its ready line was missed, which is the
// case when the module was already running before this board came up.
#define PB04_COMMAND_DELAY_MS   3500

// Ranging limits, datasheet section 2.2: 0.02m blind spot, 6m against an 88%
// reflectance target indoors.
#define PB04_RANGE_MIN_MM       20
#define PB04_RANGE_MAX_MM       6000

// The TOF field of view is 21 degrees, but as with the other optical flow
// rangefinders the cone is what the tilt compensation is allowed to work over
// rather than the beam width, so it matches them.
#define PB04_DETECTION_CONE_DECIDEGREES 900

// Frame layout, little-endian throughout:
//   0  0xFE   header
//   1  0x0E   payload byte count, fixed
//   2  flow_x_integral          int32, micro-radians
//   6  flow_y_integral          int32, micro-radians
//  10  integration_timespan     uint16, microseconds
//  12  laser distance           uint16, millimetres
//  14  valid                    uint8, PB04_FLOW_VALID when the flow is usable
//  15  laser ranging confidence uint8, percent
//  16  XOR of bytes 2..15
//  17  0x55   tail
//
// Datasheet section 5.1.1 describes a shorter frame - a 0x0A payload with the
// flow as int16 counts - which the module does not send.  Taken from a capture
// of the module itself instead: 144 consecutive frames at 50Hz, no checksum
// failure, every flow value an exact multiple of 1928.  That multiple is what
// identifies the unit, being the 0.001928023 rad the two protocols were
// cross-calibrated to, so the field counts micro-radians rather than flow
// counts.  A module on older firmware sending the documented frame would be
// rejected here; the debug counters below say so, by showing headers arriving
// while no frame ever completes.
#define PB04_FRAME_HEADER       0xFE
#define PB04_FRAME_PAYLOAD_LEN  0x0E
#define PB04_FRAME_TAIL         0x55
#define PB04_FRAME_SIZE         18
#define PB04_FRAME_PAYLOAD_POS  2
#define PB04_FRAME_CHECKSUM_POS 16

// Datasheet section 5.1.1: the valid byte takes only these two values.
#define PB04_FLOW_VALID         0xF5

typedef struct {
    int32_t distanceMm;     // negative for out of range
    timeUs_t timestampUs;
} pb04RangefinderData_t;

static pb04RangefinderData_t rfSensorData = { RANGEFINDER_NO_NEW_DATA, 0 };
static bool hasRFNewData = false;

static serialPort_t *pb04SerialPort = NULL;
static bool pb04ProtocolSelected = false;
static bool pb04CommandSent = false;
static timeMs_t pb04PortOpenedMs = 0;

static uint8_t pb04FrameBuf[PB04_FRAME_SIZE];
static unsigned pb04FramePos = 0;

// On power-up the module announces itself in ASCII before it streams anything,
// ending with "<Ready Agreement:BAMU>" or the equivalent for whichever protocol
// it has saved.  Only the fixed part is matched, so the line is recognised
// whatever mode the module is in - which is the point, since the modes this
// command exists to rescue it from are exactly the ones whose frames mean
// nothing to the parser below.  It marks the one moment the module is known to
// be listening rather than talking, and is where the command is sent from.
#define PB04_BANNER_MARKER      "<Ready"
static uint8_t pb04BannerMatch = 0;
static bool pb04CommandDue = false;

#ifdef USE_OPTICALFLOW_PB04
// Flow is accumulated as frames are read and turned into a rate once per task
// cycle.  Several frames can be waiting when a task runs late, and each states
// the window it covers, so summing the motion alongside the windows it was
// measured over gives the right rate however many arrive at once - no frame is
// discarded, and none is scaled by a span it did not cover.
static int32_t flowSumX = 0;
static int32_t flowSumY = 0;
static uint32_t flowIntegrationUs = 0;  // summed windows of the frames above
static unsigned flowFrames = 0;         // frames summed, i.e. usable ones
static unsigned flowPackets = 0;        // frames seen, usable or not
static timeUs_t flowLatestUs = 0;       // arrival of the most recent frame
#endif

static uint16_t pb04ReadU16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static int32_t pb04ReadS32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8)
                     | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/*
 * Ask the module for its native protocol.
 *
 * Nothing is sent until the module is known to be up: the datasheet allows it
 * up to 3 seconds to initialise and anything that arrives before then is
 * dropped.  Retried until a frame arrives, so a lost command is not fatal, and
 * stopped once one does, because the module is then already in this protocol -
 * which is also the case, and the common one, when it was never in another.
 */
static void pb04SelectProtocol(void)
{
    // Asked once per boot, and only while the module has yet to be understood.
    // A module already answering in this protocol - which is the usual case,
    // the choice being saved in its non-volatile memory - is left alone.
    if (pb04CommandSent || pb04ProtocolSelected || !pb04SerialPort) {
        return;
    }

    // Preferably on the back of its ready line, which is sent whatever protocol
    // the module is set to and marks the moment it is listening rather than
    // streaming.  Failing that, once it has had time to initialise.
    if (!pb04CommandDue
        && cmp32(millis(), pb04PortOpenedMs + PB04_COMMAND_DELAY_MS) < 0) {
        return;
    }

    serialWriteBuf(pb04SerialPort, (const uint8_t *)PB04_PROTOCOL_COMMAND,
                   strlen(PB04_PROTOCOL_COMMAND));

    pb04CommandSent = true;
}

/*
 * Watch the inbound ASCII for the module's ready line.
 *
 * Seeing it means the module has just restarted and has not begun streaming,
 * so the protocol command is queued for the end of this pass.  The match is a
 * rolling one over the marker rather than a buffer, since the line only has to
 * be recognised, not read.
 */
static void pb04WatchForBanner(uint8_t c)
{
    static const char marker[] = PB04_BANNER_MARKER;

    if (c == marker[pb04BannerMatch]) {
        pb04BannerMatch++;
        if (marker[pb04BannerMatch] == '\0') {
            pb04BannerMatch = 0;
            pb04CommandDue = true;
        }
    } else {
        // A mismatch can still be the start of the marker.
        pb04BannerMatch = (c == marker[0]) ? 1 : 0;
    }
}

static void pb04HandleFrame(const uint8_t *payload)
{
    const timeUs_t currentTimeUs = micros();

    // A well formed frame is the only confirmation the module answers in this
    // protocol; the reply to the command itself is not distinguishable.
    pb04ProtocolSelected = true;

    // Offsets within the payload, i.e. the frame layout above less its two
    // header bytes.
    const uint16_t distanceMm = pb04ReadU16(&payload[10]);
    const uint8_t rangeQuality = payload[13];

    DEBUG_SET(DEBUG_PB04, 0, distanceMm);     //!< Distance [unit:0.001m]
    DEBUG_SET(DEBUG_PB04, 1, rangeQuality);   //!< Confidence [unit:%]

    // Beyond its range the module substitutes a fixed 8m reading rather than
    // flagging it (datasheet section 5.1.1), so the module's own limits are the
    // only test available.
    const bool distanceInRange = (distanceMm >= PB04_RANGE_MIN_MM) && (distanceMm <= PB04_RANGE_MAX_MM);

    rfSensorData.distanceMm = distanceInRange ? (int32_t)distanceMm : -1;
    rfSensorData.timestampUs = currentTimeUs;
    hasRFNewData = true;

    // -99 marks the sample as out of range
    DEBUG_SET(DEBUG_PB04, 6, distanceInRange ? (int32_t)(distanceMm / 10) : -99); //!< Reported Distance [unit:cm]

#ifdef USE_OPTICALFLOW_PB04
    flowLatestUs = currentTimeUs;
    flowPackets++;

    DEBUG_SET(DEBUG_PB04, 4, payload[12]);    //!< Flow Valid Status

    // An unusable frame contributes neither its motion nor its window, which is
    // what keeps it from biasing the rate: the divisor sums only the windows of
    // the frames actually added, so dropping one costs a sample, not accuracy.
    if (payload[12] == PB04_FLOW_VALID) {
        flowSumX += pb04ReadS32(&payload[0]);
        flowSumY += pb04ReadS32(&payload[4]);
        flowIntegrationUs += pb04ReadU16(&payload[8]);
        flowFrames++;
    }
#endif
}

static void pb04ParseByte(uint8_t c)
{
    switch (pb04FramePos) {
    case 0:
        // Hunting for a header; anything else is the tail of a frame that was
        // already rejected, or the module's reply to the protocol command.
        if (c != PB04_FRAME_HEADER) {
            return;
        }
        break;

    case 1:
        if (c != PB04_FRAME_PAYLOAD_LEN) {
            // Not a frame after all.  This byte can still be the header of the
            // next one, so it is offered to the hunt rather than dropped.
            pb04FramePos = 0;
            if (c == PB04_FRAME_HEADER) {
                pb04FrameBuf[pb04FramePos++] = c;
            }
            return;
        }
        break;

    default:
        break;
    }

    pb04FrameBuf[pb04FramePos++] = c;

    if (pb04FramePos < PB04_FRAME_SIZE) {
        return;
    }
    pb04FramePos = 0;

    if (pb04FrameBuf[PB04_FRAME_SIZE - 1] != PB04_FRAME_TAIL) {
        return;
    }

    uint8_t checksum = 0;
    for (unsigned i = PB04_FRAME_PAYLOAD_POS; i < PB04_FRAME_CHECKSUM_POS; i++) {
        checksum ^= pb04FrameBuf[i];
    }
    if (checksum != pb04FrameBuf[PB04_FRAME_CHECKSUM_POS]) {
        return;
    }

    pb04HandleFrame(&pb04FrameBuf[PB04_FRAME_PAYLOAD_POS]);
}

/*
 * Drain whatever the module has sent.
 *
 * Both halves of the module arrive in the one frame on the one UART, so
 * whichever of the two tasks runs first reads for both.  Taking the whole
 * buffer each time is what keeps this independent of how often that happens.
 */
static void pb04Poll(void)
{
    if (!pb04SerialPort) {
        return;
    }

    while (serialRxBytesWaiting(pb04SerialPort)) {
        const uint8_t c = serialRead(pb04SerialPort);

        if (!pb04ProtocolSelected) {
            pb04WatchForBanner(c);
        }

        pb04ParseByte(c);
    }

    // Sent after the drain rather than before it, so a banner seen on this pass
    // is answered on this pass instead of a task period later.
    pb04SelectProtocol();
}

/*
 * Both halves are the one module on the one UART, so whichever is detected
 * first opens the port and the other reuses it: openSerialPort() rejects a port
 * already in use and would otherwise clear the handle.
 */
static bool pb04OpenPort(serialPortIdentifier_e identifier)
{
    if (pb04SerialPort) {
        return true;
    }

    if (identifier == SERIAL_PORT_NONE) {
        return false;
    }

    pb04SerialPort = openSerialPort(identifier, FUNCTION_LIDAR, NULL, NULL,
                                    PB04_BAUDRATE, MODE_RXTX, SERIAL_NOT_INVERTED);
    if (!pb04SerialPort) {
        return false;
    }

    // The module's initialisation time is counted from here rather than from
    // boot, since this is the first moment it could have been heard.
    pb04PortOpenedMs = millis();

    return true;
}

static void pb04RangefinderInit(rangefinderDev_t *dev)
{
    UNUSED(dev);
}

static void pb04RangefinderUpdate(rangefinderDev_t *dev)
{
    UNUSED(dev);

    pb04Poll();
}

// Return most recent device output in cm
static int32_t pb04RangefinderGetDistance(rangefinderDev_t *dev)
{
    UNUSED(dev);

    if (hasRFNewData) {
        hasRFNewData = false;
        return (rfSensorData.distanceMm >= 0) ? (rfSensorData.distanceMm / 10) : RANGEFINDER_OUT_OF_RANGE;
    }

    return RANGEFINDER_NO_NEW_DATA;
}

bool pb04RangefinderDetect(rangefinderDev_t *dev)
{
    if (!pb04OpenPort((serialPortIdentifier_e)rangefinderConfig()->rangefinder_uart)) {
        return false;
    }

    dev->delayMs = RANGEFINDER_PB04_TASK_PERIOD_MS;
    dev->maxRangeCm = PB04_RANGE_MAX_MM / 10;
    dev->detectionConeDeciDegrees = PB04_DETECTION_CONE_DECIDEGREES;
    dev->detectionConeExtendedDeciDegrees = PB04_DETECTION_CONE_DECIDEGREES;

    dev->init = &pb04RangefinderInit;
    dev->update = &pb04RangefinderUpdate;
    dev->read = &pb04RangefinderGetDistance;

    return true;
}

#ifdef USE_OPTICALFLOW_PB04

// Datasheet section 5.1.3: the lens has to be at least 80mm off the surface.
#define PB04_OPTICALFLOW_MIN_RANGE_MM       80
#define PB04_OPFLOW_MIN_QUALITY_THRESHOLD   30
// Sensor latency: a flow sample reports motion from this long ago (integration
// window, image processing and transport), so the rotation compensation uses a
// gyro reading of that age.
//
// Measured, as was done for the MTF01, by correlating reported flow against
// gyro over rotation-only moves read over this protocol: the flow best matches
// the gyro averaged over the 20ms window ending 20ms ago, so the centre of what
// it reports is 30ms old, and replaying the matching below agrees at 30ms.
// That is the budget one would expect - 10ms to the centre of the window, the
// module's own processing and transport, and up to a task period here.
//
// An earlier 45000 came from driving the module over MSPV2, where frames queued
// in the port and arrived stale; that queueing, not the sensor, was most of the
// difference.
#define PB04_OPFLOW_GYRO_DELAY_US           30000

static opticalflowData_t opticalflowSensorData = {0};

static void pb04OpticalflowPublish(void);

static void pb04OpticalflowInit(opticalflowDev_t *dev)
{
    UNUSED(dev);

    flowSumX = 0;
    flowSumY = 0;
    flowIntegrationUs = 0;
    flowFrames = 0;
    flowPackets = 0;
    flowLatestUs = micros();
    // Without this the first publish measures its elapsed time from zero and
    // discards a sample for being impossibly stale.
    opticalflowSensorData.timeStampUs = flowLatestUs;
}

// Called once per flow task cycle, immediately before the data is read.
static void pb04OpticalflowUpdate(opticalflowDev_t *dev)
{
    UNUSED(dev);

    pb04Poll();
    pb04OpticalflowPublish();
}

static void pb04OpticalflowGetData(opticalflowDev_t *dev, opticalflowData_t *result)
{
    UNUSED(dev);

    *result = opticalflowSensorData;
}

bool pb04OpticalflowDetect(opticalflowDev_t *dev)
{
    if (!pb04OpenPort((serialPortIdentifier_e)opticalflowConfig()->opticalflow_uart)) {
        return false;
    }

    dev->delayMs = RANGEFINDER_PB04_TASK_PERIOD_MS;
    dev->minRangeCm = PB04_OPTICALFLOW_MIN_RANGE_MM / 10;
    dev->minQualityThreshold = PB04_OPFLOW_MIN_QUALITY_THRESHOLD;
    dev->gyroDelayUs = PB04_OPFLOW_GYRO_DELAY_US;

    dev->init = &pb04OpticalflowInit;
    dev->update = &pb04OpticalflowUpdate;
    dev->read = &pb04OpticalflowGetData;

    return true;
}

static void pb04OpticalflowPublish(void)
{
    if (flowPackets == 0) {
        return;  // nothing arrived; the previous sample and its timestamp stand
    }

    const timeUs_t windowEndUs = flowLatestUs;
    // The span the summed motion covers is the sum of the windows the module
    // reported for those frames - not the wall clock since the last publish,
    // which would also count any gap between them.
    //
    // No scale factor appears here because the module leaves none to apply: the
    // flow is micro-radians and the window microseconds, so dividing one by the
    // other is already radians per second.
    // Elapsed time is still worth knowing, as a staleness guard and to show in
    // the debug how far apart the publishes are falling.
    const float dt_s = (float)cmp32(windowEndUs, opticalflowSensorData.timeStampUs) / 1e6f;

    const bool dtValid = (dt_s > 0.0f && dt_s < 1.0f);
    const bool flowValid = (flowFrames > 0) && (flowIntegrationUs > 0);
    // The range travels in the same frame as the flow, so it always describes
    // the moment the flow was measured; only its value has to be checked.
    const bool rangeValid = (rfSensorData.distanceMm >= PB04_OPTICALFLOW_MIN_RANGE_MM);

    opticalflowSensorData.timeStampUs = windowEndUs;
    opticalflowSensorData.flowDtValid = dtValid;
    opticalflowSensorData.rangeValid  = rangeValid;
    opticalflowSensorData.flowValid   = dtValid && flowValid;

    DEBUG_SET(DEBUG_PB04, 5, lrintf(dt_s * 1e6f));  //!< Frame Gap [unit:us]

    if (dtValid && flowValid) {
        opticalflowSensorData.flowRate.x = (float)flowSumX / (float)flowIntegrationUs;
        opticalflowSensorData.flowRate.y = (float)flowSumY / (float)flowIntegrationUs;
        // The frame reports only whether the flow is usable, not how good it
        // is, so a usable frame is published at full quality.
        opticalflowSensorData.quality = 100;
    } else {
        opticalflowSensorData.flowRate.x = 0;
        opticalflowSensorData.flowRate.y = 0;
        opticalflowSensorData.quality = 0;
    }

    DEBUG_SET(DEBUG_PB04, 2, lrintf(opticalflowSensorData.flowRate.x * 1000.0f)); //!< Flow Rate X [unit:0.001rad/s]
    DEBUG_SET(DEBUG_PB04, 3, lrintf(opticalflowSensorData.flowRate.y * 1000.0f)); //!< Flow Rate Y [unit:0.001rad/s]

    if (!rangeValid) {
        opticalflowSensorData.quality = OPTICALFLOW_OUT_OF_RANGE;
    }

    flowSumX = 0;
    flowSumY = 0;
    flowIntegrationUs = 0;
    flowFrames = 0;
    flowPackets = 0;
}

#endif // USE_OPTICALFLOW_PB04
#endif // USE_RANGEFINDER_PB04
