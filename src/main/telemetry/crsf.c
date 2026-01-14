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
#include <math.h>
#include <limits.h>

#include "platform.h"

#ifdef USE_TELEMETRY_CRSF

#include "build/atomic.h"
#include "build/build_config.h"
#include "build/version.h"

#include "cms/cms.h"

#include "config/config.h"
#include "config/feature.h"

#include "common/crc.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/streambuf.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/persistent.h"

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/position.h"

#include "io/displayport_crsf.h"
#include "io/gps.h"
#include "io/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/crsf.h"
#include "rx/crsf_protocol.h"

#include "sensors/battery.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"
#include "telemetry/msp_shared.h"

#include "crsf.h"

#define CRSF_CYCLETIME_US                   100000 // 100ms, 10 Hz -- all telemetry frames are inserted in this timeslice
#define CRSF_DEVICEINFO_VERSION             0x01
#define CRSF_DEVICEINFO_PARAMETER_COUNT     0

#define CRSF_MSP_BUFFER_SIZE 96
#define CRSF_MSP_LENGTH_OFFSET 1

static bool crsfTelemetryEnabled;
static bool deviceInfoReplyPending;
#if defined(USE_CRSF_V3)
static bool telemetryResponsePending;
#endif
static uint8_t crsfFrame[CRSF_FRAME_SIZE_MAX];
#ifdef USE_GPS
static uint32_t lastGpsSolnTime;
#endif
#ifdef USE_BARO
static uint32_t lastBaroTime;
#endif

#if defined(USE_MSP_OVER_TELEMETRY)
typedef struct mspBuffer_s {
    uint8_t bytes[CRSF_MSP_BUFFER_SIZE];
    int len;
} mspBuffer_t;

static mspBuffer_t mspRxBuffer;

#if defined(USE_CRSF_V3)

#define CRSF_TELEMETRY_FRAME_INTERVAL_MAX_US 20000U // 20ms

#if defined(USE_CRSF_CMS_TELEMETRY)
#define CRSF_LINK_TYPE_CHECK_US 250000 // 250 ms
#define CRSF_ELRS_DISLAYPORT_CHUNK_INTERVAL_US 75000 // 75 ms

typedef enum {
    CRSF_LINK_UNKNOWN,
    CRSF_LINK_ELRS,
    CRSF_LINK_NOT_ELRS
} crsfLinkType_t;

static crsfLinkType_t crsfLinkType = CRSF_LINK_UNKNOWN;
static timeDelta_t crsfDisplayPortChunkIntervalUs = 0;
#endif

static bool isCrsfV3Running = false;
typedef struct {
    uint8_t hasPendingReply:1;
    uint8_t isNewSpeedValid:1;
    uint8_t portID:3;
    uint8_t index;
    uint32_t confirmationTime;
} crsfSpeedControl_s;

static crsfSpeedControl_s crsfSpeed = {0};

uint32_t getCrsfCachedBaudrate(void)
{
    uint32_t crsfCachedBaudrate = persistentObjectRead(PERSISTENT_OBJECT_SERIALRX_BAUD);
    // check if valid first. return default baudrate if not
    for (unsigned i = 0; i < BAUD_COUNT; i++) {
        if (crsfCachedBaudrate == baudRates[i] && baudRates[i] >= CRSF_BAUDRATE) {
            return crsfCachedBaudrate;
        }
    }
    return CRSF_BAUDRATE;
}

static bool checkCrsfCustomizedSpeed(void)
{
    return crsfSpeed.index < BAUD_COUNT ? true : false;
}

uint32_t getCrsfDesiredSpeed(void)
{
    return checkCrsfCustomizedSpeed() ? baudRates[crsfSpeed.index] : CRSF_BAUDRATE;
}

void setCrsfDefaultSpeed(void)
{
    crsfSpeed.hasPendingReply = false;
    crsfSpeed.isNewSpeedValid = false;
    crsfSpeed.confirmationTime = 0;
    crsfSpeed.index = BAUD_COUNT;
    isCrsfV3Running = false;
    crsfRxUpdateBaudrate(getCrsfDesiredSpeed());
}

bool crsfBaudNegotiationInProgress(void)
{
    return crsfSpeed.hasPendingReply || crsfSpeed.isNewSpeedValid;
}
#endif

void initCrsfMspBuffer(void)
{
    mspRxBuffer.len = 0;
}

bool bufferCrsfMspFrame(uint8_t *frameStart, int frameLength)
{
    if (mspRxBuffer.len + CRSF_MSP_LENGTH_OFFSET + frameLength > CRSF_MSP_BUFFER_SIZE) {
        return false;
    } else {
        uint8_t *p = mspRxBuffer.bytes + mspRxBuffer.len;
        *p++ = frameLength;
        memcpy(p, frameStart, frameLength);
        mspRxBuffer.len += CRSF_MSP_LENGTH_OFFSET + frameLength;
        return true;
    }
}

bool handleCrsfMspFrameBuffer(mspResponseFnPtr responseFn)
{
    static bool replyPending = false;
    if (replyPending) {
        if (crsfRxIsTelemetryBufEmpty()) {
            replyPending = sendMspReply(CRSF_FRAME_TX_MSP_FRAME_SIZE, responseFn);
        }
        return replyPending;
    }
    if (!mspRxBuffer.len) {
        return false;
    }
    int pos = 0;
    while (true) {
        const uint8_t mspFrameLength = mspRxBuffer.bytes[pos];
        if (handleMspFrame(&mspRxBuffer.bytes[CRSF_MSP_LENGTH_OFFSET + pos], mspFrameLength, NULL)) {
            if (crsfRxIsTelemetryBufEmpty()) {
                replyPending = sendMspReply(CRSF_FRAME_TX_MSP_FRAME_SIZE, responseFn);
            } else {
                replyPending = true;
            }
        }
        pos += CRSF_MSP_LENGTH_OFFSET + mspFrameLength;
        ATOMIC_BLOCK(NVIC_PRIO_SERIALUART1) {
            if (pos >= mspRxBuffer.len) {
                mspRxBuffer.len = 0;
                return replyPending;
            }
        }
    }
    return replyPending;
}
#endif

static void crsfInitializeFrame(sbuf_t *dst)
{
    dst->ptr = crsfFrame;
    dst->end = ARRAYEND(crsfFrame);

    sbufWriteU8(dst, CRSF_SYNC_BYTE);
}

static void crsfFinalize(sbuf_t *dst)
{
    crc8_dvb_s2_sbuf_append(dst, &crsfFrame[2]); // start at byte 2, since CRC does not include device address and frame length
    sbufSwitchToReader(dst, crsfFrame);
    // write the telemetry frame to the receiver.
    crsfRxWriteTelemetryData(sbufPtr(dst), sbufBytesRemaining(dst));
}

#if defined(USE_CRSF_ACCGYRO_TELEMETRY)
static bool crsfAccGyroEnabled(void)
{
    return telemetryConfig()->crsf_tlm_accgyro;
}
#endif

/*
CRSF frame has the structure:
<Device address> <Frame length> <Type> <Payload> <CRC>
Device address: (uint8_t)
Frame length:   length in  bytes including Type (uint8_t)
Type:           (uint8_t)
CRC:            (uint8_t), crc of <Type> and <Payload>
*/

#if defined(USE_GPS)
/*
0x02 GPS
Payload:
int32_t     Latitude ( degree / 10`000`000 )
int32_t     Longitude (degree / 10`000`000 )
uint16_t    Groundspeed ( km/h / 10 )
uint16_t    GPS heading ( degree / 100 )
uint16      Altitude ( meter ­1000m offset )
uint8_t     Satellites in use ( counter )
*/
MAYBE_UNUSED static void crsfFrameGps(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, CRSF_FRAME_GPS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    sbufWriteU8(dst, CRSF_FRAMETYPE_GPS);
    sbufWriteU32BigEndian(dst, gpsSol.llh.lat); // CRSF and betaflight use same units for degrees
    sbufWriteU32BigEndian(dst, gpsSol.llh.lon);
    sbufWriteU16BigEndian(dst, (gpsSol.groundSpeed * 36 + 50) / 100); // gpsSol.groundSpeed is in cm/s
    sbufWriteU16BigEndian(dst, gpsSol.groundCourse * 10); // gpsSol.groundCourse is degrees * 10
    sbufWriteU16BigEndian(dst, constrain(gpsSol.llh.altCm / 100 + 1000, 0, 5000)); // constrain altitude from 0 to 5,000m
    sbufWriteU8(dst, gpsSol.numSat);
}

/*
0x03 GPS Time
Payload:
    int16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
*/
void crsfFrameGpsTime(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAME_GPS_TIME_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    sbufWriteU8(dst, CRSF_FRAMETYPE_GPS_TIME);

#ifdef USE_RTC_TIME
    dateTime_t dt;
    memset(&dt, 0, sizeof(dateTime_t));
    rtcGetDateTime(&dt);
    sbufWriteU16BigEndian(dst, (int16_t)dt.year);
    sbufWriteU8(dst, dt.month);
    sbufWriteU8(dst, dt.day);
    sbufWriteU8(dst, dt.hours);
    sbufWriteU8(dst, dt.minutes);
    sbufWriteU8(dst, dt.seconds);
    sbufWriteU16BigEndian(dst, dt.millis);
#else
    // No RTC - send zeros
    sbufWriteU16BigEndian(dst, 0);
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, 0);
    sbufWriteU16BigEndian(dst, 0);
#endif
}

/*
0x06 GPS Extended
Payload:
    uint8_t fix_type;       // Current GPS fix quality
    int16_t n_speed;        // Northward (north = positive) Speed [cm/sec]
    int16_t e_speed;        // Eastward (east = positive) Speed [cm/sec]
    int16_t v_speed;        // Vertical (up = positive) Speed [cm/sec]
    int16_t h_speed_acc;    // Horizontal Speed accuracy cm/sec
    int16_t track_acc;      // Heading accuracy in degrees scaled with 1e-1 degrees times 10)
    int16_t alt_ellipsoid;  // Meters Height above GPS Ellipsoid (not MSL)
    int16_t h_acc;          // horizontal accuracy in cm
    int16_t v_acc;          // vertical accuracy in cm
    uint8_t reserved;
    uint8_t hDOP;           // Horizontal dilution of precision,Dimensionless in nits of.1.
    uint8_t vDOP;           // vertical dilution of precision, Dimensionless in nits of .1.
*/
void crsfFrameGpsExtended(sbuf_t *dst)
{
    // Write Frame Header
    // Length includes Type (1) + Payload (20) + CRC (1) usually,
    // but Cleanflight sbuf logic usually takes Payload Size + Overhead constant.
    sbufWriteU8(dst, CRSF_FRAME_GPS_EXTENDED_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    sbufWriteU8(dst, CRSF_FRAMETYPE_GPS_EXTENDED);

    // 1. Fix Type
    // Map existing sats/status to CRSF fix types (1=No Fix, 2=2D, 3=3D)
    // gpsSol doesn't have an explicit fixType field, so we derive it from satellite count.
    uint8_t gpsFixType = 0;

    if (!STATE(GPS_FIX)) {
        gpsFixType = 1;
    } else {
        if (gpsSol.numSat < GPS_MIN_SAT_COUNT) {
            gpsFixType = 2;
        } else {
            gpsFixType = 3;
        }
    }
    sbufWriteU8(dst, gpsFixType);

    sbufWriteU16BigEndian(dst, gpsSol.velned.velN);
    sbufWriteU16BigEndian(dst, gpsSol.velned.velE);
    sbufWriteU16BigEndian(dst, -gpsSol.velned.velD);

    // 4. Horizontal Speed Accuracy (cm/s)
    // gpsSol.acc.sAcc is in mm/s -> divide by 10 for cm/s
    sbufWriteU16BigEndian(dst, gpsSol.acc.sAcc / 10);

    // 5. Track/Heading Accuracy (0.1 degree units)
    // gpsSol.acc.headAcc is in 1e-5 degrees -> divide by 10000 for 0.1 degree units
    sbufWriteU16BigEndian(dst, gpsSol.acc.headAcc / 10000);

    // 6. Ellipsoid Altitude (m)
    // gpsSol.llh.altCm is in cm -> divide by 100 for meters.
    // Note: gpsSol altitude is typically MSL. If Ellipsoid is strictly required
    // and geoid separation is unknown, MSL is the standard fallback.
    sbufWriteU16BigEndian(dst, gpsSol.llh.altCm / 100);

    // 7. Horizontal Accuracy (cm)
    // gpsSol.acc.hAcc is in mm -> divide by 10 for cm
    sbufWriteU16BigEndian(dst, gpsSol.acc.hAcc / 10);

    // 8. Vertical Accuracy (cm)
    // gpsSol.acc.vAcc is in mm -> divide by 10 for cm
    sbufWriteU16BigEndian(dst, gpsSol.acc.vAcc / 10);

    // 9. Reserved
    sbufWriteU8(dst, 0);

    // 10. Horizontal DOP (0.1 units)
    // gpsSol.dop.hdop is scaled by 100. Divide by 10 to get 0.1 units.
    sbufWriteU8(dst, gpsSol.dop.hdop / 10);

    // 11. Vertical DOP (0.1 units)
    // gpsSol.dop.vdop is scaled by 100. Divide by 10 to get 0.1 units.
    sbufWriteU8(dst, gpsSol.dop.vdop / 10);
}
#endif // USE_GPS

/*
0x07 Vario sensor
Payload:
int16_t     Vertical speed ( cm/s )
*/
MAYBE_UNUSED static void crsfFrameVarioSensor(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, CRSF_FRAME_VARIO_SENSOR_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    sbufWriteU8(dst, CRSF_FRAMETYPE_VARIO_SENSOR);
    sbufWriteU16BigEndian(dst, getEstimatedVario()); // vario, cm/s(Z));
}

/*
0x11 Baro
Payload:
int32_t     pressure pa
int32_t     temperature centidegrees
*/
#if defined(USE_BARO)
void crsfFrameBaro(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, CRSF_FRAME_BARO_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    sbufWriteU8(dst, CRSF_FRAMETYPE_BARO);
    sbufWriteU32BigEndian(dst, baro.pressure);
    sbufWriteU32BigEndian(dst, baro.temperature); // CRSF and betaflight use same units for degrees
}
#endif

#if defined(USE_MAG)
/*
0x12 Mag
Payload:
int16_t     field x mgauss * 3
int16_t     field y mgauss * 3
int16_t     field z mgauss * 3
*/
void crsfFrameMag(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, CRSF_FRAME_MAG_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    sbufWriteU8(dst, CRSF_FRAMETYPE_MAG);
    sbufWriteU16BigEndian(dst, (int16_t)(mag.magADC.v[X] * 3.0f));
    sbufWriteU16BigEndian(dst, (int16_t)(mag.magADC.v[Y] * 3.0f));
    sbufWriteU16BigEndian(dst, (int16_t)(mag.magADC.v[Z] * 3.0f));
}
#endif

/*
0x08 Battery sensor
Payload:
uint16_t    Voltage ( mV * 100 )
uint16_t    Current ( mA * 100 )
uint24_t    Fuel ( drawn mAh )
uint8_t     Battery remaining ( percent )
*/
static void crsfFrameBatterySensor(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    sbufWriteU8(dst, CRSF_FRAMETYPE_BATTERY_SENSOR);
    if (telemetryConfig()->report_cell_voltage) {
        sbufWriteU16BigEndian(dst, (getBatteryAverageCellVoltage() + 5) / 10); // vbat is in units of 0.01V
    } else {
        sbufWriteU16BigEndian(dst, getLegacyBatteryVoltage());
    }
    sbufWriteU16BigEndian(dst, getAmperage() / 10);
    const uint32_t mAhDrawn = getMAhDrawn();
    const uint8_t batteryRemainingPercentage = calculateBatteryPercentageRemaining();
    sbufWriteU8(dst, (mAhDrawn >> 16));
    sbufWriteU8(dst, (mAhDrawn >> 8));
    sbufWriteU8(dst, (uint8_t)mAhDrawn);
    sbufWriteU8(dst, batteryRemainingPercentage);
}

#if defined(USE_BARO) && defined(USE_VARIO)
// pack altitude in decimeters into a 16-bit value.
// Due to strange OpenTX behavior of count any 0xFFFF value as incorrect, the maximum sending value is limited to 0xFFFE (32766 meters)
// in order to have both precision and range in 16-bit
// value of altitude is packed with different precision depending on highest-bit value.
// on receiving side:
// if MSB==0, altitude is sent in decimeters as uint16 with -1000m base. So, range is -1000..2276m.
// if MSB==1, altitude is sent in meters with 0 base. So, range is 0..32766m (MSB must be zeroed).
// altitude lower  than -1000m is sent as zero   (should be displayed as "<-1000m" or something).
// altitude higher than 32767m is sent as 0xfffe (should be displayed as ">32766m" or something).
// range from 0 to 2276m might be sent with dm- or m-precision. But this function always use dm-precision.
static inline uint16_t calcAltitudePacked(int32_t altitude_dm)
{
    static const int ALT_DM_OFFSET = 10000;
    int valDm = altitude_dm + ALT_DM_OFFSET;

    if (valDm < 0) return 0;   // too low, return minimum
    if (valDm < 0x8000) return valDm;  // 15 bits to return dm value with offset

    return MIN((altitude_dm + 5) / 10, 0x7fffe) | 0x8000; // positive 15bit value in meters, with OpenTX limit
}

static inline int8_t calcVerticalSpeedPacked(int16_t verticalSpeed) // Vertical speed in m/s (meters per second)
{
    // linearity coefficient.
    // Bigger values lead to more linear output i.e., less precise smaller values and more precise big values.
    // Decreasing the coefficient increases nonlinearity, i.e., more precise small values and less precise big values.
    static const float Kl = 100.0f;

    // Range coefficient is calculated as result_max / log(verticalSpeedMax * LinearityCoefficient + 1);
    // but it must be set manually (not calculated) for equality of packing and unpacking
    static const float Kr = .026f;

    int8_t sign = verticalSpeed < 0 ? -1 : 1;
    const int result32 = lrintf(log_approx(verticalSpeed * sign / Kl + 1) / Kr) * sign;
    int8_t result8 = constrain(result32, SCHAR_MIN, SCHAR_MAX);
    return result8;

    // for unpacking the following function might be used:
    // int unpacked = lrintf((expf(result8 * sign * Kr) - 1) * Kl) * sign;
    // lrint might not be used depending on integer or floating output.
}

// pack barometric altitude
static void crsfFrameAltitude(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, CRSF_FRAME_BARO_ALTITUDE_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    sbufWriteU8(dst, CRSF_FRAMETYPE_BARO_ALTITUDE);
    sbufWriteU16BigEndian(dst, calcAltitudePacked((baro.altitude + 5) / 10));
    sbufWriteU8(dst, calcVerticalSpeedPacked(getEstimatedVario()));
}
#endif

/*
0x0B Heartbeat
Payload:
int16_t    origin_add ( Origin Device address )
*/

MAYBE_UNUSED static void crsfFrameHeartbeat(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAME_HEARTBEAT_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    sbufWriteU8(dst, CRSF_FRAMETYPE_HEARTBEAT);
    sbufWriteU16BigEndian(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
}

/*
0x28 Ping
Payload:
int8_t    destination_add ( Destination Device address )
int8_t    origin_add ( Origin Device address )
*/

MAYBE_UNUSED static void crsfFramePing(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAME_DEVICE_PING_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    sbufWriteU8(dst, CRSF_FRAMETYPE_DEVICE_PING);
    sbufWriteU8(dst, CRSF_ADDRESS_CRSF_RECEIVER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
}

typedef enum {
    CRSF_ACTIVE_ANTENNA1 = 0,
    CRSF_ACTIVE_ANTENNA2 = 1
} crsfActiveAntenna_e;

typedef enum {
    CRSF_RF_MODE_4_HZ = 0,
    CRSF_RF_MODE_50_HZ = 1,
    CRSF_RF_MODE_150_HZ = 2
} crsrRfMode_e;

typedef enum {
    CRSF_RF_POWER_0_mW = 0,
    CRSF_RF_POWER_10_mW = 1,
    CRSF_RF_POWER_25_mW = 2,
    CRSF_RF_POWER_100_mW = 3,
    CRSF_RF_POWER_500_mW = 4,
    CRSF_RF_POWER_1000_mW = 5,
    CRSF_RF_POWER_2000_mW = 6,
    CRSF_RF_POWER_250_mW = 7,
    CRSF_RF_POWER_50_mW = 8
} crsrRfPower_e;

/*
0x1E Attitude
Payload:
int16_t     Pitch angle ( rad / 10000 )
int16_t     Roll angle ( rad / 10000 )
int16_t     Yaw angle ( rad / 10000 )
*/

// convert andgle in decidegree to radians/10000 with reducing angle to +/-180 degree range
static int16_t decidegrees2Radians10000(int16_t angle_decidegree)
{
    while (angle_decidegree > 1800) {
        angle_decidegree -= 3600;
    }
    while (angle_decidegree < -1800) {
        angle_decidegree += 3600;
    }
    return (int16_t)(RAD * 1000.0f * angle_decidegree);
}

// fill dst buffer with crsf-attitude telemetry frame
static void crsfFrameAttitude(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    sbufWriteU8(dst, CRSF_FRAMETYPE_ATTITUDE);
    sbufWriteU16BigEndian(dst, decidegrees2Radians10000(attitude.values.pitch));
    sbufWriteU16BigEndian(dst, decidegrees2Radians10000(attitude.values.roll));
    sbufWriteU16BigEndian(dst, decidegrees2Radians10000(attitude.values.yaw));
}

/*
0x21 Flight mode text based
Payload:
char[]      Flight mode ( Null terminated string )
*/
static void crsfFrameFlightMode(sbuf_t *dst)
{
    // write zero for frame length, since we don't know it yet
    uint8_t *lengthPtr = sbufPtr(dst);
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, CRSF_FRAMETYPE_FLIGHT_MODE);

    // Acro is the default mode
    const char *flightMode = "ACRO";

    // Flight modes in decreasing order of importance
    if (FLIGHT_MODE(FAILSAFE_MODE) || IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
        flightMode = "!FS!";
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE) || IS_RC_MODE_ACTIVE(BOXGPSRESCUE)) {
        flightMode = "RTH";
    } else if (FLIGHT_MODE(PASSTHRU_MODE)) {
        flightMode = "PASS";
    } else if (FLIGHT_MODE(ANGLE_MODE)) {
        flightMode = "ANGL";
    } else if (FLIGHT_MODE(POS_HOLD_MODE)) {
        flightMode = "POSH";
    } else if (FLIGHT_MODE(ALT_HOLD_MODE)) {
        flightMode = "ALTH";
    } else if (FLIGHT_MODE(HORIZON_MODE)) {
        flightMode = "HOR";
    } else if (FLIGHT_MODE(CHIRP_MODE)) {
        flightMode = "CHIR";
    } else if (isAirmodeEnabled()) {
        flightMode = "AIR";
    }

    sbufWriteString(dst, flightMode);

    if (!ARMING_FLAG(ARMED) && !FLIGHT_MODE(FAILSAFE_MODE)) {
        // * = ready to arm
        // ! = arming disabled
        // ? = GPS rescue disabled
        bool isGpsRescueDisabled = false;
#ifdef USE_GPS
        isGpsRescueDisabled = featureIsEnabled(FEATURE_GPS) && gpsRescueIsConfigured() && gpsSol.numSat < gpsRescueConfig()->minSats && !STATE(GPS_FIX);
#endif
        sbufWriteU8(dst, isArmingDisabled() ? '!' : isGpsRescueDisabled ? '?' : '*');
    }

    sbufWriteU8(dst, '\0');     // zero-terminate string
    // write in the frame length
    *lengthPtr = sbufPtr(dst) - lengthPtr;
}

/*
0x29 Device Info
Payload:
uint8_t     Destination
uint8_t     Origin
char[]      Device Name ( Null terminated string )
uint32_t    Null Bytes
uint32_t    Null Bytes
uint32_t    Null Bytes
uint8_t     255 (Max MSP Parameter)
uint8_t     0x01 (Parameter version 1)
*/
static void crsfFrameDeviceInfo(sbuf_t *dst)
{
    char buff[30];
    tfp_sprintf(buff, "%s %s: %s", FC_FIRMWARE_NAME, FC_VERSION_STRING, systemConfig()->boardIdentifier);

    uint8_t *lengthPtr = sbufPtr(dst);
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, CRSF_FRAMETYPE_DEVICE_INFO);
    sbufWriteU8(dst, CRSF_ADDRESS_RADIO_TRANSMITTER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteStringWithZeroTerminator(dst, buff);
    for (unsigned int ii = 0; ii < 12; ii++) {
        sbufWriteU8(dst, 0x00);
    }
    sbufWriteU8(dst, CRSF_DEVICEINFO_PARAMETER_COUNT);
    sbufWriteU8(dst, CRSF_DEVICEINFO_VERSION);
    *lengthPtr = sbufPtr(dst) - lengthPtr;
}

#if defined(USE_CRSF_V3)
static void crsfFrameSpeedNegotiationResponse(sbuf_t *dst, bool reply)
{
    uint8_t *lengthPtr = sbufPtr(dst);
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, CRSF_FRAMETYPE_COMMAND);
    sbufWriteU8(dst, CRSF_ADDRESS_CRSF_RECEIVER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteU8(dst, CRSF_COMMAND_SUBCMD_GENERAL);
    sbufWriteU8(dst, CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_RESPONSE);
    sbufWriteU8(dst, crsfSpeed.portID);
    sbufWriteU8(dst, reply);
    crc8_poly_0xba_sbuf_append(dst, &lengthPtr[1]);
    *lengthPtr = sbufPtr(dst) - lengthPtr;
}

static void crsfProcessSpeedNegotiationCmd(const uint8_t *frameStart)
{
    uint32_t newBaudrate = frameStart[2] << 24 | frameStart[3] << 16 | frameStart[4] << 8 | frameStart[5];
    uint8_t ii = 0;
    for (ii = 0; ii < BAUD_COUNT; ++ii) {
        if (newBaudrate == baudRates[ii]) {
            break;
        }
    }
    crsfSpeed.portID = frameStart[1];
    crsfSpeed.index = ii;
}

static void crsfScheduleSpeedNegotiationResponse(void)
{
    crsfSpeed.hasPendingReply = true;
    crsfSpeed.isNewSpeedValid = false;
}

void speedNegotiationProcess(timeUs_t currentTimeUs)
{
    if (crsfSpeed.hasPendingReply) {
        bool found = ((crsfSpeed.index < BAUD_COUNT) && crsfRxUseNegotiatedBaud()) ? true : false;
        sbuf_t crsfSpeedNegotiationBuf;
        sbuf_t *dst = &crsfSpeedNegotiationBuf;
        crsfInitializeFrame(dst);
        crsfFrameSpeedNegotiationResponse(dst, found);
        crsfRxSendTelemetryData(); // prevent overwriting previous data
        crsfFinalize(dst);
        crsfRxSendTelemetryData();
        crsfSpeed.hasPendingReply = false;
        crsfSpeed.isNewSpeedValid = found;
        crsfSpeed.confirmationTime = currentTimeUs;
    } else if (crsfSpeed.isNewSpeedValid) {
        if (cmpTimeUs(currentTimeUs, crsfSpeed.confirmationTime) >= 4000) {
            // delay 4ms before applying the new baudrate
            crsfRxUpdateBaudrate(getCrsfDesiredSpeed());
            crsfSpeed.isNewSpeedValid = false;
            isCrsfV3Running = true;
        }
    } else if (!featureIsEnabled(FEATURE_TELEMETRY) && crsfRxUseNegotiatedBaud()) {
        // Send heartbeat if telemetry is disabled to allow RX to detect baud rate mismatches
        sbuf_t crsfPayloadBuf;
        sbuf_t *dst = &crsfPayloadBuf;
        crsfInitializeFrame(dst);
        crsfFrameHeartbeat(dst);
        crsfRxSendTelemetryData(); // prevent overwriting previous data
        crsfFinalize(dst);
        crsfRxSendTelemetryData();
#if defined(USE_CRSF_CMS_TELEMETRY)
    } else if (crsfLinkType == CRSF_LINK_UNKNOWN) {
        static timeUs_t lastPing;

        if ((cmpTimeUs(currentTimeUs, lastPing) > CRSF_LINK_TYPE_CHECK_US)) {
            // Send a ping, the response to which will be a device info response giving the rx serial number
            sbuf_t crsfPayloadBuf;
            sbuf_t *dst = &crsfPayloadBuf;
            crsfInitializeFrame(dst);
            crsfFramePing(dst);
            crsfRxSendTelemetryData(); // prevent overwriting previous data
            crsfFinalize(dst);
            crsfRxSendTelemetryData();

            lastPing = currentTimeUs;
        }
#endif
    }
}
#endif

#if defined(USE_CRSF_ACCGYRO_TELEMETRY)
/*
0x13 AccGyro in NEU bodyframe, samples are raw data averaged over the sample interval
Accel: +ve X = foward
       +ve Y = right
       +ve Z = up
Gyro:  +ve X = roll left
       +ve Y = pitch up
       +ve Z = yaw clockwise
Payload:
    uint32_t sample_time;       // Timestamp of the sample in us
    int16_t gyro_x;             // LSB = INT16_MAX/2000 DPS
    int16_t gyro_y;             // LSB = INT16_MAX/2000 DPS
    int16_t gyro_z;             // LSB = INT16_MAX/2000 DPS
    int16_t acc_x;              // LSB = INT16_MAX/16 G
    int16_t acc_y;              // LSB = INT16_MAX/16 G
    int16_t acc_z;              // LSB = INT16_MAX/16 G
    int16_t gyro_temp;          // C * 100
*/
bool crsfFrameAccGyro(sbuf_t *dst, timeUs_t currentTimeUs)
{
    // Capture any pending gyro samples into snapshot
    const bool hasNewGyroData = gyroStartDownsampledCycle();

    // Only send if we have gyro data
    if (!hasNewGyroData) {
        return false;
    }

    // Read gyro data from snapshot
    float gyroAverage[XYZ_AXIS_COUNT];
    for (int axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
        gyroAverage[axis] = gyroGetDownsampled(axis);
    }

#define GRAVITY_EARTH (9.80665f)

    // Capture accel if available, use previous snapshot if not
    accelStartDownsampledCycle();
    float accAverage[XYZ_AXIS_COUNT];
    for (int axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
        accAverage[axis] = accelGetDownsampled(axis) * acc.dev.acc_1G_rec * GRAVITY_EARTH;
    }

// Convert dps 'x' (max ±2000) to 16-bit integer (max ±32767), clamped to prevent overflow
#define DEGREES_TO_2KDPS16BIT(x) (int16_t)((constrainf((x), -2000.0f, 2000.0f) * INT16_MAX) / 2000.0f)
// Convert acceleration 'x' in m/s^2 (max ±16*g) to 16-bit integer (max ±32767), clamped to prevent overflow
#define ACCMSS_TO_16G16BIT(x) (int16_t)((constrainf((x), -16.0f * GRAVITY_EARTH, 16.0f * GRAVITY_EARTH) * INT16_MAX) / (16.0f * GRAVITY_EARTH))

    uint8_t *lengthPtr = sbufPtr(dst);
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, CRSF_FRAMETYPE_ACCGYRO);
    sbufWriteU32BigEndian(dst, currentTimeUs);
    sbufWriteU16BigEndian(dst, DEGREES_TO_2KDPS16BIT(gyroAverage[X]));
    sbufWriteU16BigEndian(dst, DEGREES_TO_2KDPS16BIT(-gyroAverage[Y]));
    sbufWriteU16BigEndian(dst, DEGREES_TO_2KDPS16BIT(-gyroAverage[Z]));
    sbufWriteU16BigEndian(dst, ACCMSS_TO_16G16BIT(accAverage[X]));
    sbufWriteU16BigEndian(dst, ACCMSS_TO_16G16BIT(-accAverage[Y]));
    sbufWriteU16BigEndian(dst, ACCMSS_TO_16G16BIT(-accAverage[Z]));
    // Temperature in centidegrees, clamped to int16 range (-327 to +327°C)
    sbufWriteU16BigEndian(dst, constrain(gyroGetTemperature() * 100, INT16_MIN, INT16_MAX));
    *lengthPtr = sbufPtr(dst) - lengthPtr;

    return true;
}
#endif

#if defined(USE_CRSF_CMS_TELEMETRY)
#define CRSF_DISPLAYPORT_MAX_CHUNK_LENGTH   50
#define CRSF_DISPLAYPORT_BATCH_MAX          0x3F
#define CRSF_DISPLAYPORT_FIRST_CHUNK_MASK   0x80
#define CRSF_DISPLAYPORT_LAST_CHUNK_MASK    0x40
#define CRSF_DISPLAYPORT_SANITIZE_MASK      0x60
#define CRSF_RLE_CHAR_REPEATED_MASK         0x80
#define CRSF_RLE_MAX_RUN_LENGTH             256
#define CRSF_RLE_BATCH_SIZE                 2

static uint16_t getRunLength(const void *start, const void *end)
{
    uint8_t *cursor = (uint8_t*)start;
    uint8_t c = *cursor;
    size_t runLength = 0;
    for (; cursor != end; cursor++) {
        if (*cursor == c) {
            runLength++;
        } else {
            break;
        }
    }
    return runLength;
}

static void cRleEncodeStream(sbuf_t *source, sbuf_t *dest, uint8_t maxDestLen)
{
    const uint8_t *destEnd = sbufPtr(dest) + maxDestLen;
    while (sbufBytesRemaining(source) && (sbufPtr(dest) < destEnd)) {
        const uint8_t destRemaining = destEnd - sbufPtr(dest);
        const uint8_t *srcPtr = sbufPtr(source);
        const uint16_t runLength = getRunLength(srcPtr, source->end);
        uint8_t c = *srcPtr;
        if (runLength > 1) {
            c |=  CRSF_RLE_CHAR_REPEATED_MASK;
            const uint8_t fullBatches = (runLength / CRSF_RLE_MAX_RUN_LENGTH);
            const uint8_t remainder = (runLength % CRSF_RLE_MAX_RUN_LENGTH);
            const uint8_t totalBatches = fullBatches + (remainder ? 1 : 0);
            if (destRemaining >= totalBatches * CRSF_RLE_BATCH_SIZE) {
                for (unsigned int i = 1; i <= totalBatches; i++) {
                    const uint8_t batchLength = (i < totalBatches) ? CRSF_RLE_MAX_RUN_LENGTH : remainder;
                    sbufWriteU8(dest, c);
                    sbufWriteU8(dest, batchLength);
                }
                sbufAdvance(source, runLength);
            } else {
                break;
            }
        } else if (destRemaining >= runLength) {
            sbufWriteU8(dest, c);
            sbufAdvance(source, runLength);
        }
    }
}

static void crsfFrameDisplayPortChunk(sbuf_t *dst, sbuf_t *src, uint8_t batchId, uint8_t idx)
{
    uint8_t *lengthPtr = sbufPtr(dst);
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, CRSF_FRAMETYPE_DISPLAYPORT_CMD);
    sbufWriteU8(dst, CRSF_ADDRESS_RADIO_TRANSMITTER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteU8(dst, CRSF_DISPLAYPORT_SUBCMD_UPDATE);
    uint8_t *metaPtr = sbufPtr(dst);
    sbufWriteU8(dst, batchId);
    sbufWriteU8(dst, idx);
    cRleEncodeStream(src, dst, CRSF_DISPLAYPORT_MAX_CHUNK_LENGTH);
    if (idx == 0) {
        *metaPtr |= CRSF_DISPLAYPORT_FIRST_CHUNK_MASK;
    }
    if (!sbufBytesRemaining(src)) {
        *metaPtr |= CRSF_DISPLAYPORT_LAST_CHUNK_MASK;
    }
    *lengthPtr = sbufPtr(dst) - lengthPtr;
}

static void crsfFrameDisplayPortClear(sbuf_t *dst)
{
    uint8_t *lengthPtr = sbufPtr(dst);
    sbufWriteU8(dst, CRSF_DISPLAY_PORT_COLS_MAX + CRSF_FRAME_LENGTH_EXT_TYPE_CRC);
    sbufWriteU8(dst, CRSF_FRAMETYPE_DISPLAYPORT_CMD);
    sbufWriteU8(dst, CRSF_ADDRESS_RADIO_TRANSMITTER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteU8(dst, CRSF_DISPLAYPORT_SUBCMD_CLEAR);
    *lengthPtr = sbufPtr(dst) - lengthPtr;
}

#endif

// schedule array to decide how often each type of frame is sent
typedef enum {
    CRSF_FRAME_START_INDEX = 0,
    CRSF_FRAME_ATTITUDE_INDEX = CRSF_FRAME_START_INDEX,
    CRSF_FRAME_BARO_ALTITUDE_INDEX,
    CRSF_FRAME_BATTERY_SENSOR_INDEX,
    CRSF_FRAME_FLIGHT_MODE_INDEX,
    CRSF_FRAME_MAG_INDEX,
    CRSF_FRAME_GPS_INDEX,
    CRSF_FRAME_VARIO_SENSOR_INDEX,
    CRSF_FRAME_HEARTBEAT_INDEX,
    CRSF_SCHEDULE_COUNT_MAX
} crsfFrameTypeIndex_e;

typedef enum {
    CRSF_FRAME_GPS_TIME_INDEX,
    CRSF_FRAME_GPS_EXTENDED_INDEX,
    CRSF_FRAME_BARO_INDEX,
#ifdef USE_CRSF_ACCGYRO_TELEMETRY
    CRSF_FRAME_ACCGYRO_INDEX,
#endif
    CRSF_TIMED_SCHEDULE_COUNT_MAX
} crsfTimedFrameTypeIndex_e;

static uint8_t crsfScheduleCount;
static uint16_t crsfSchedule[CRSF_SCHEDULE_COUNT_MAX];
static uint16_t crsfTimedSchedule;

#if defined(USE_MSP_OVER_TELEMETRY)

static bool mspReplyPending;
static uint8_t mspRequestOriginID = 0; // origin ID of last msp-over-crsf request. Needed to send response to the origin.

void crsfScheduleMspResponse(uint8_t requestOriginID)
{
    mspReplyPending = true;
    mspRequestOriginID = requestOriginID;
}

// sends MSP response chunk over CRSF. Must be of type mspResponseFnPtr
static void crsfSendMspResponse(uint8_t *payload, const uint8_t payloadSize)
{
    sbuf_t crsfPayloadBuf;
    sbuf_t *dst = &crsfPayloadBuf;

    crsfInitializeFrame(dst);
    sbufWriteU8(dst, payloadSize + CRSF_FRAME_LENGTH_EXT_TYPE_CRC); // size of CRSF frame (everything except sync and size itself)
    sbufWriteU8(dst, CRSF_FRAMETYPE_MSP_RESP); // CRSF type
    sbufWriteU8(dst, mspRequestOriginID);   // response destination must be the same as request origin in order to response reach proper destination.
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER); // origin is always this device
    sbufWriteData(dst, payload, payloadSize);
    crsfFinalize(dst);
}
#endif

static bool processCrsf(uint32_t currentTimeUs, uint32_t crsfLastCycleTime)
{
    sbuf_t crsfPayloadBuf;
    sbuf_t *dst = &crsfPayloadBuf;

#ifdef USE_GPS
#if defined(USE_CRSF_V3)
    if (isCrsfV3Running && crsfTimedSchedule & BIT(CRSF_FRAME_GPS_TIME_INDEX) && gpsSol.time > lastGpsSolnTime) {
        crsfInitializeFrame(dst);
        crsfFrameGpsTime(dst);
        crsfFinalize(dst);
        crsfTimedSchedule &= ~BIT(CRSF_FRAME_GPS_TIME_INDEX);
        return true;
    }
    if (isCrsfV3Running && crsfTimedSchedule & BIT(CRSF_FRAME_GPS_EXTENDED_INDEX) && gpsSol.time > lastGpsSolnTime) {
        crsfInitializeFrame(dst);
        crsfFrameGpsExtended(dst);
        crsfFinalize(dst);
        crsfTimedSchedule &= ~BIT(CRSF_FRAME_GPS_EXTENDED_INDEX);    // tick-tock between GPS frames
        return true;
    }
#endif
    if (crsfTimedSchedule & BIT(CRSF_FRAME_GPS_INDEX) && gpsSol.time > lastGpsSolnTime) {
        crsfInitializeFrame(dst);
        crsfFrameGps(dst);
        crsfFinalize(dst);
#if defined(USE_CRSF_V3)
        if (isCrsfV3Running) {
            crsfTimedSchedule |= (BIT(CRSF_FRAME_GPS_EXTENDED_INDEX) | BIT(CRSF_FRAME_GPS_TIME_INDEX));
        }
#endif
        lastGpsSolnTime = gpsSol.time;
        return true;
    }
#endif
#if defined(USE_BARO)
    if (crsfTimedSchedule & BIT(CRSF_FRAME_BARO_INDEX) && currentTimeUs > lastBaroTime + 100000U) { // 10Hz baro update
        crsfInitializeFrame(dst);
        crsfFrameBaro(dst);
        crsfFinalize(dst);
        lastBaroTime = currentTimeUs;
        return true;
    }
#endif

    if (crsfScheduleCount == 0) {
        return false;
    }

    if (currentTimeUs < crsfLastCycleTime + CRSF_CYCLETIME_US / crsfScheduleCount) {
        return false;
    }

    static uint8_t crsfScheduleIndex = 0;
    const uint16_t currentSchedule = crsfSchedule[crsfScheduleIndex];

    if (currentSchedule & BIT(CRSF_FRAME_ATTITUDE_INDEX)) {
        crsfInitializeFrame(dst);
        crsfFrameAttitude(dst);
        crsfFinalize(dst);
    }
#if defined(USE_BARO) && defined(USE_VARIO)
    // send barometric altitude
    if (currentSchedule & BIT(CRSF_FRAME_BARO_ALTITUDE_INDEX)) {
        crsfInitializeFrame(dst);
        crsfFrameAltitude(dst);
        crsfFinalize(dst);
    }
#endif
    if (currentSchedule & BIT(CRSF_FRAME_BATTERY_SENSOR_INDEX)) {
        crsfInitializeFrame(dst);
        crsfFrameBatterySensor(dst);
        crsfFinalize(dst);
    }

    if (currentSchedule & BIT(CRSF_FRAME_FLIGHT_MODE_INDEX)) {
        crsfInitializeFrame(dst);
        crsfFrameFlightMode(dst);
        crsfFinalize(dst);
    }
#if defined(USE_BARO) && !defined(USE_CRSF_V3)
    if (currentSchedule & BIT(CRSF_FRAME_BARO_INDEX)) {
        crsfInitializeFrame(dst);
        crsfFrameBaro(dst);
        crsfFinalize(dst);
    }
#endif
#if defined(USE_MAG)
    if (currentSchedule & BIT(CRSF_FRAME_MAG_INDEX)) {
        crsfInitializeFrame(dst);
        crsfFrameMag(dst);
        crsfFinalize(dst);
    }
#endif
#ifdef USE_GPS
    if (currentSchedule & BIT(CRSF_FRAME_GPS_INDEX)) {
        crsfInitializeFrame(dst);
        crsfFrameGps(dst);
        crsfFinalize(dst);
    }
#endif
#ifdef USE_VARIO
    if (currentSchedule & BIT(CRSF_FRAME_VARIO_SENSOR_INDEX)) {
        crsfInitializeFrame(dst);
        crsfFrameVarioSensor(dst);
        crsfFinalize(dst);
    }
#endif
#if defined(USE_CRSF_V3)
    if (currentSchedule & BIT(CRSF_FRAME_HEARTBEAT_INDEX)) {
        crsfInitializeFrame(dst);
        crsfFrameHeartbeat(dst);
        crsfFinalize(dst);
    }
#endif

    crsfScheduleIndex = (crsfScheduleIndex + 1) % crsfScheduleCount;

    return true;
}

void crsfScheduleDeviceInfoResponse(void)
{
    deviceInfoReplyPending = true;
}

#if defined(USE_CRSF_V3)
FAST_CODE void crsfScheduleTelemetryResponse(void)
{
    telemetryResponsePending = true;
}

bool crsfTelemetryUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    UNUSED(currentTimeUs);
    UNUSED(currentDeltaTimeUs);

    return telemetryResponsePending;
}

#endif

#if defined(USE_CRSF_CMS_TELEMETRY)
void crsfHandleDeviceInfoResponse(uint8_t *payload)
{
    // Skip over dst/src address bytes
    payload += 2;

    // Skip over first string which is the rx model/part number
    while (*payload++ != '\0');

    // Check the serial number
    if (memcmp(payload, "ELRS", 4) == 0) {
        crsfLinkType = CRSF_LINK_ELRS;
        crsfDisplayPortChunkIntervalUs = CRSF_ELRS_DISLAYPORT_CHUNK_INTERVAL_US;
    } else {
        crsfLinkType = CRSF_LINK_NOT_ELRS;
    }
}
#endif

void initCrsfTelemetry(void)
{
    // check if there is a serial port open for CRSF telemetry (ie opened by the CRSF RX)
    // and feature is enabled, if so, set CRSF telemetry enabled
    crsfTelemetryEnabled = crsfRxIsActive();

    if (!crsfTelemetryEnabled) {
        return;
    }

    deviceInfoReplyPending = false;
#if defined(USE_CRSF_V3)
    telemetryResponsePending = false;
#endif
#if defined(USE_MSP_OVER_TELEMETRY)
    mspReplyPending = false;
#endif

    uint8_t index = 0;
    if (sensors(SENSOR_ACC) && telemetryIsSensorEnabled(SENSOR_PITCH | SENSOR_ROLL | SENSOR_HEADING)) {
        crsfSchedule[index++] = BIT(CRSF_FRAME_ATTITUDE_INDEX);
    }
#if defined(USE_BARO) && defined(USE_VARIO)
    if (telemetryIsSensorEnabled(SENSOR_ALTITUDE)) {
        crsfSchedule[index++] = BIT(CRSF_FRAME_BARO_ALTITUDE_INDEX);
    }
#endif
    if ((isBatteryVoltageConfigured() && telemetryIsSensorEnabled(SENSOR_VOLTAGE))
        || (isAmperageConfigured() && telemetryIsSensorEnabled(SENSOR_CURRENT | SENSOR_FUEL))) {
        crsfSchedule[index++] = BIT(CRSF_FRAME_BATTERY_SENSOR_INDEX);
    }
    if (telemetryIsSensorEnabled(SENSOR_MODE)) {
        crsfSchedule[index++] = BIT(CRSF_FRAME_FLIGHT_MODE_INDEX);
    }
#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
#ifdef USE_CRSF_V3
        crsfTimedSchedule |= BIT(CRSF_FRAME_BARO_INDEX);
#else
        crsfSchedule[index++] = BIT(CRSF_FRAME_BARO_INDEX);
#endif
    }
#endif
#if defined(USE_MAG)
    if (sensors(SENSOR_MAG)) {
        crsfSchedule[index++] = BIT(CRSF_FRAME_MAG_INDEX);
    }
#endif
#ifdef USE_GPS
    if (featureIsEnabled(FEATURE_GPS)
       && telemetryIsSensorEnabled(SENSOR_ALTITUDE | SENSOR_LAT_LONG | SENSOR_GROUND_SPEED | SENSOR_HEADING)) {
#ifdef USE_CRSF_V3
        crsfTimedSchedule |= BIT(CRSF_FRAME_GPS_INDEX) | BIT(CRSF_FRAME_GPS_TIME_INDEX) | BIT(CRSF_FRAME_GPS_EXTENDED_INDEX);
#else
        crsfSchedule[index++] = BIT(CRSF_FRAME_GPS_INDEX);
#endif
    }
#endif
#if defined(USE_CRSF_ACCGYRO_TELEMETRY) && defined(USE_CRSF_V3)
    if (crsfAccGyroEnabled() && (sensors(SENSOR_ACC) || sensors(SENSOR_GYRO))) {
        crsfTimedSchedule |= BIT(CRSF_FRAME_ACCGYRO_INDEX);
    }
#endif
#ifdef USE_VARIO
    if ((sensors(SENSOR_BARO) || featureIsEnabled(FEATURE_GPS)) && telemetryIsSensorEnabled(SENSOR_VARIO)) {
        crsfSchedule[index++] = BIT(CRSF_FRAME_VARIO_SENSOR_INDEX);
    }
#endif

#if defined(USE_CRSF_V3)
    if (!(crsfTimedSchedule & BIT(CRSF_FRAME_ACCGYRO_INDEX))) {
        while (index < (CRSF_CYCLETIME_US / CRSF_TELEMETRY_FRAME_INTERVAL_MAX_US) && index < CRSF_SCHEDULE_COUNT_MAX) {
            // schedule heartbeat to ensure that telemetry/heartbeat frames are sent at minimum 50Hz
            crsfSchedule[index++] = BIT(CRSF_FRAME_HEARTBEAT_INDEX);
        }
    }
#endif

    crsfScheduleCount = index;

#if defined(USE_CRSF_CMS_TELEMETRY)
    crsfDisplayportRegister();
#endif
}

bool checkCrsfTelemetryState(void)
{
    return crsfTelemetryEnabled;
}

#if defined(USE_CRSF_CMS_TELEMETRY)
void crsfProcessDisplayPortCmd(uint8_t *frameStart)
{
    uint8_t cmd = *frameStart;
    switch (cmd) {
    case CRSF_DISPLAYPORT_SUBCMD_OPEN: ;
        const uint8_t rows = *(frameStart + CRSF_DISPLAYPORT_OPEN_ROWS_OFFSET);
        const uint8_t cols = *(frameStart + CRSF_DISPLAYPORT_OPEN_COLS_OFFSET);
        crsfDisplayPortSetDimensions(rows, cols);
        crsfDisplayPortMenuOpen();
        break;
    case CRSF_DISPLAYPORT_SUBCMD_CLOSE:
        crsfDisplayPortMenuExit();
        break;
    case CRSF_DISPLAYPORT_SUBCMD_POLL:
        crsfDisplayPortRefresh();
        break;
    default:
        break;
    }

}

#endif

#if defined(USE_CRSF_V3)
void crsfProcessCommand(uint8_t *frameStart)
{
    uint8_t cmd = *frameStart;
    uint8_t subCmd = frameStart[1];
    switch (cmd) {
    case CRSF_COMMAND_SUBCMD_GENERAL:
        switch (subCmd) {
        case CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL:
            crsfProcessSpeedNegotiationCmd(&frameStart[1]);
            crsfScheduleSpeedNegotiationResponse();
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}
#endif

/*
 * Called periodically by the scheduler
 */
void handleCrsfTelemetry(timeUs_t currentTimeUs)
{
    static uint32_t crsfLastCycleTime;

    if (!crsfTelemetryEnabled) {
        return;
    }

#if defined(USE_CRSF_V3)
    if (crsfBaudNegotiationInProgress()) {
        return;
    }
#endif

    // Give the receiver a chance to send any outstanding telemetry data.
    // This needs to be done at high frequency, to enable the RX to send the telemetry frame
    // in between the RX frames.
    crsfRxSendTelemetryData();

#if defined(USE_CRSF_V3)
    // only send responses on recipt of full frames, this ensures the telemetry rate is never faster
    // than the frame rate
    if (!telemetryResponsePending) {
        return;
    }
#endif

    // Send ad-hoc response frames as soon as possible
#if defined(USE_MSP_OVER_TELEMETRY)
    if (mspReplyPending) {
        mspReplyPending = handleCrsfMspFrameBuffer(&crsfSendMspResponse);
        crsfLastCycleTime = currentTimeUs; // reset telemetry timing due to ad-hoc request
#if defined(USE_CRSF_V3)
        telemetryResponsePending = false;
#endif
        return;
    }
#endif

    if (deviceInfoReplyPending) {
        sbuf_t crsfPayloadBuf;
        sbuf_t *dst = &crsfPayloadBuf;
        crsfInitializeFrame(dst);
        crsfFrameDeviceInfo(dst);
        crsfFinalize(dst);
        deviceInfoReplyPending = false;
#if defined(USE_CRSF_V3)
        telemetryResponsePending = false;
#endif
        crsfLastCycleTime = currentTimeUs; // reset telemetry timing due to ad-hoc request
        return;
    }

#if defined(USE_CRSF_CMS_TELEMETRY)
    if (crsfDisplayPortScreen()->reset) {
        crsfDisplayPortScreen()->reset = false;
        sbuf_t crsfDisplayPortBuf;
        sbuf_t *dst = &crsfDisplayPortBuf;
        crsfInitializeFrame(dst);
        crsfFrameDisplayPortClear(dst);
        crsfFinalize(dst);
        crsfLastCycleTime = currentTimeUs;
#if defined(USE_CRSF_V3)
        telemetryResponsePending = false;
#endif
        return;
    }

    if (crsfDisplayPortIsReady()) {
        static uint8_t displayPortBatchId = 0;
        static sbuf_t displayPortSbuf;
        static sbuf_t *src = NULL;
        static uint8_t batchIndex;
        static timeUs_t batchLastTimeUs;
        sbuf_t crsfDisplayPortBuf;
        sbuf_t *dst = &crsfDisplayPortBuf;

        if (crsfDisplayPortScreen()->updated) {
            crsfDisplayPortScreen()->updated = false;
            uint16_t screenSize = crsfDisplayPortScreen()->rows * crsfDisplayPortScreen()->cols;
            uint8_t *srcStart = (uint8_t*)crsfDisplayPortScreen()->buffer;
            uint8_t *srcEnd = (uint8_t*)(crsfDisplayPortScreen()->buffer + screenSize);
            src = sbufInit(&displayPortSbuf, srcStart, srcEnd);
            displayPortBatchId = (displayPortBatchId  + 1) % CRSF_DISPLAYPORT_BATCH_MAX;
            batchIndex = 0;
        }

        // Wait between successive chunks of displayport data for CMS menu display to prevent ELRS buffer over-run if necessary
        if (src && sbufBytesRemaining(src) &&
            (cmpTimeUs(currentTimeUs, batchLastTimeUs) > crsfDisplayPortChunkIntervalUs)) {
            crsfInitializeFrame(dst);
            crsfFrameDisplayPortChunk(dst, src, displayPortBatchId, batchIndex);
            crsfFinalize(dst);
            crsfRxSendTelemetryData();
            batchIndex++;
            batchLastTimeUs = currentTimeUs;

            crsfLastCycleTime = currentTimeUs;
#if defined(USE_CRSF_V3)
            telemetryResponsePending = false;
#endif

            return;
        }
    }
#endif
    if (!crsfRxIsTelemetryBufEmpty()) {
        return; // do nothing if telemetry ouptut buffer is not empty yet.
    }

    // Actual telemetry data only needs to be sent at a low frequency, ie 10Hz
    // Spread out scheduled frames evenly so each frame is sent at the same frequency.
    // speed up the telemetry rate to that configured if using accgyro data
    if (processCrsf(currentTimeUs, crsfLastCycleTime)) {
        crsfLastCycleTime = currentTimeUs;
#ifdef USE_CRSF_ACCGYRO_TELEMETRY
    } else if (crsfAccGyroEnabled()) {  // let the inbound request rate dictate the outbound rate
        sbuf_t crsfPayloadBuf;
        sbuf_t *dst = &crsfPayloadBuf;
        crsfInitializeFrame(dst);
        if (crsfFrameAccGyro(dst, currentTimeUs)) {
            crsfFinalize(dst);
        }
#endif
    }
#if defined(USE_CRSF_V3)
    telemetryResponsePending = false;
#endif
}

#if defined(UNIT_TEST) || defined(USE_RX_EXPRESSLRS)
static int crsfFinalizeBuf(sbuf_t *dst, uint8_t *frame)
{
    crc8_dvb_s2_sbuf_append(dst, &crsfFrame[2]); // start at byte 2, since CRC does not include device address and frame length
    sbufSwitchToReader(dst, crsfFrame);
    const int frameSize = sbufBytesRemaining(dst);
    for (int ii = 0; sbufBytesRemaining(dst); ++ii) {
        frame[ii] = sbufReadU8(dst);
    }
    return frameSize;
}

int getCrsfFrame(uint8_t *frame, crsfFrameType_e frameType)
{
    sbuf_t crsfFrameBuf;
    sbuf_t *sbuf = &crsfFrameBuf;

    crsfInitializeFrame(sbuf);
    switch (frameType) {
    default:
    case CRSF_FRAMETYPE_ATTITUDE:
        crsfFrameAttitude(sbuf);
        break;
    case CRSF_FRAMETYPE_BATTERY_SENSOR:
        crsfFrameBatterySensor(sbuf);
        break;
    case CRSF_FRAMETYPE_FLIGHT_MODE:
        crsfFrameFlightMode(sbuf);
        break;
#if defined(USE_BARO)
    case CRSF_FRAMETYPE_BARO:
        crsfFrameBaro(sbuf);
        break;
#endif
#if defined(USE_MAG)
    case CRSF_FRAMETYPE_MAG:
        crsfFrameMag(sbuf);
        break;
#endif
#if defined(USE_GPS)
    case CRSF_FRAMETYPE_GPS:
        crsfFrameGps(sbuf);
        break;
    case CRSF_FRAMETYPE_GPS_EXTENDED:
        crsfFrameGpsExtended(sbuf);
        break;
#endif
#if defined(USE_VARIO)
    case CRSF_FRAMETYPE_VARIO_SENSOR:
        crsfFrameVarioSensor(sbuf);
        break;
#endif
#if defined(USE_MSP_OVER_TELEMETRY)
    case CRSF_FRAMETYPE_DEVICE_INFO:
        crsfFrameDeviceInfo(sbuf);
        break;
#endif
    }
    const int frameSize = crsfFinalizeBuf(sbuf, frame);
    return frameSize;
}

#if defined(USE_MSP_OVER_TELEMETRY)
int getCrsfMspFrame(uint8_t *frame, uint8_t *payload, const uint8_t payloadSize)
{
    sbuf_t crsfFrameBuf;
    sbuf_t *sbuf = &crsfFrameBuf;

    crsfInitializeFrame(sbuf);
    sbufWriteU8(sbuf, payloadSize + CRSF_FRAME_LENGTH_EXT_TYPE_CRC);
    sbufWriteU8(sbuf, CRSF_FRAMETYPE_MSP_RESP);
    sbufWriteU8(sbuf, CRSF_ADDRESS_RADIO_TRANSMITTER);
    sbufWriteU8(sbuf, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteData(sbuf, payload, payloadSize);
    const int frameSize = crsfFinalizeBuf(sbuf, frame);
    return frameSize;
}
#endif
#endif
#endif
