/*
 * SmartPort Telemetry implementation by frank26080115
 * see https://github.com/frank26080115/cleanflight/wiki/Using-Smart-Port
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_SMARTPORT)

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/altitude.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "interface/msp.h"

#include "io/beeper.h"
#include "io/motors.h"
#include "io/gps.h"
#include "io/serial.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"
#include "telemetry/smartport.h"
#include "telemetry/msp_shared.h"

#define SMARTPORT_MIN_TELEMETRY_RESPONSE_DELAY_US 500

// these data identifiers are obtained from https://github.com/opentx/opentx/blob/master/radio/src/telemetry/frsky_hub.h
enum
{
    FSSP_DATAID_SPEED      = 0x0830 ,
    FSSP_DATAID_VFAS       = 0x0210 ,
    FSSP_DATAID_CURRENT    = 0x0200 ,
    FSSP_DATAID_RPM        = 0x050F ,
    FSSP_DATAID_ALTITUDE   = 0x0100 ,
    FSSP_DATAID_FUEL       = 0x0600 ,
    FSSP_DATAID_ADC1       = 0xF102 ,
    FSSP_DATAID_ADC2       = 0xF103 ,
    FSSP_DATAID_LATLONG    = 0x0800 ,
    FSSP_DATAID_CAP_USED   = 0x0600 ,
    FSSP_DATAID_VARIO      = 0x0110 ,
    FSSP_DATAID_CELLS      = 0x0300 ,
    FSSP_DATAID_CELLS_LAST = 0x030F ,
    FSSP_DATAID_HEADING    = 0x0840 ,
    FSSP_DATAID_ACCX       = 0x0700 ,
    FSSP_DATAID_ACCY       = 0x0710 ,
    FSSP_DATAID_ACCZ       = 0x0720 ,
    FSSP_DATAID_T1         = 0x0400 ,
    FSSP_DATAID_T2         = 0x0410 ,
    FSSP_DATAID_HOME_DIST  = 0x0420 ,
    FSSP_DATAID_GPS_ALT    = 0x0820 ,
    FSSP_DATAID_ASPD       = 0x0A00 ,
    FSSP_DATAID_A3         = 0x0900 ,
    FSSP_DATAID_A4         = 0x0910
};

const uint16_t frSkyDataIdTable[] = {
    FSSP_DATAID_SPEED     ,
    FSSP_DATAID_VFAS      ,
    FSSP_DATAID_CURRENT   ,
    //FSSP_DATAID_RPM       ,
    FSSP_DATAID_ALTITUDE  ,
    FSSP_DATAID_FUEL      ,
    //FSSP_DATAID_ADC1      ,
    //FSSP_DATAID_ADC2      ,
    FSSP_DATAID_LATLONG   ,
    FSSP_DATAID_LATLONG   , // twice
    //FSSP_DATAID_CAP_USED  ,
    FSSP_DATAID_VARIO     ,
    //FSSP_DATAID_CELLS     ,
    //FSSP_DATAID_CELLS_LAST,
    FSSP_DATAID_HEADING   ,
    FSSP_DATAID_ACCX      ,
    FSSP_DATAID_ACCY      ,
    FSSP_DATAID_ACCZ      ,
    FSSP_DATAID_T1        ,
    FSSP_DATAID_T2        ,
    FSSP_DATAID_HOME_DIST ,
    FSSP_DATAID_GPS_ALT   ,
    FSSP_DATAID_ASPD      ,
    FSSP_DATAID_A4        ,
    0
};

#define __USE_C99_MATH // for roundf()
#define SMARTPORT_BAUD 57600
#define SMARTPORT_UART_MODE MODE_RXTX
#define SMARTPORT_SERVICE_TIMEOUT_MS 1 // max allowed time to find a value to send

static serialPort_t *smartPortSerialPort = NULL; // The 'SmartPort'(tm) Port.
static serialPortConfig_t *portConfig;

static portSharing_e smartPortPortSharing;

enum
{
    TELEMETRY_STATE_UNINITIALIZED,
    TELEMETRY_STATE_INITIALIZED_SERIAL,
    TELEMETRY_STATE_INITIALIZED_EXTERNAL,
};

static uint8_t telemetryState = TELEMETRY_STATE_UNINITIALIZED;
static uint8_t smartPortIdCnt = 0;

typedef struct smartPortFrame_s {
    uint8_t  sensorId;
    smartPortPayload_t payload;
    uint8_t  crc;
} __attribute__((packed)) smartPortFrame_t;

#define SMARTPORT_MSP_PAYLOAD_SIZE (sizeof(smartPortPayload_t) - sizeof(uint8_t))

static smartPortWriteFrameFn *smartPortWriteFrame;

#if defined(USE_MSP_OVER_TELEMETRY)
static bool smartPortMspReplyPending = false;
#endif

smartPortPayload_t *smartPortDataReceive(uint16_t c, bool *clearToSend, smartPortCheckQueueEmptyFn *checkQueueEmpty, bool useChecksum)
{
    static uint8_t rxBuffer[sizeof(smartPortPayload_t)];
    static uint8_t smartPortRxBytes = 0;
    static bool skipUntilStart = true;
    static bool awaitingSensorId = false;
    static bool byteStuffing = false;
    static uint16_t checksum = 0;

    if (c == FSSP_START_STOP) {
        *clearToSend = false;
        smartPortRxBytes = 0;
        awaitingSensorId = true;
        skipUntilStart = false;

        return NULL;
    } else if (skipUntilStart) {
        return NULL;
    }

    if (awaitingSensorId) {
        awaitingSensorId = false;
        if ((c == FSSP_SENSOR_ID1) && checkQueueEmpty()) {
            // our slot is starting, no need to decode more
            *clearToSend = true;
            skipUntilStart = true;
        } else if (c == FSSP_SENSOR_ID2) {
            checksum = 0;
        } else {
            skipUntilStart = true;
        }
    } else {
        if (c == FSSP_DLE) {
            byteStuffing = true;

            return NULL;
        } else if (byteStuffing) {
            c ^= FSSP_DLE_XOR;
            byteStuffing = false;
        }

        if (smartPortRxBytes < sizeof(smartPortPayload_t)) {
            rxBuffer[smartPortRxBytes++] = (uint8_t)c;
            checksum += c;

            if (!useChecksum && (smartPortRxBytes == sizeof(smartPortPayload_t))) {
                skipUntilStart = true;

                return (smartPortPayload_t *)&rxBuffer;
            }
        } else {
            skipUntilStart = true;

            checksum += c;
            checksum = (checksum & 0xFF) + (checksum >> 8);
            if (checksum == 0xFF) {
                return (smartPortPayload_t *)&rxBuffer;
            }
        }
    }

    return NULL;
}

void smartPortSendByte(uint8_t c, uint16_t *checksum, serialPort_t *port)
{
    // smart port escape sequence
    if (c == FSSP_DLE || c == FSSP_START_STOP) {
        serialWrite(port, FSSP_DLE);
        serialWrite(port, c ^ FSSP_DLE_XOR);
    } else {
        serialWrite(port, c);
    }

    if (checksum != NULL) {
        *checksum += c;
    }
}

void smartPortWriteFrameSerial(const smartPortPayload_t *payload, serialPort_t *port, uint16_t checksum)
{
    uint8_t *data = (uint8_t *)payload;
    for (unsigned i = 0; i < sizeof(smartPortPayload_t); i++) {
        smartPortSendByte(*data++, &checksum, port);
    }
    checksum = 0xff - ((checksum & 0xff) + (checksum >> 8));
    smartPortSendByte((uint8_t)checksum, NULL, port);
}

static void smartPortWriteFrameInternal(const smartPortPayload_t *payload)
{
    smartPortWriteFrameSerial(payload, smartPortSerialPort, 0);
}

static void smartPortSendPackage(uint16_t id, uint32_t val)
{
    smartPortPayload_t payload;
    payload.frameId = FSSP_DATA_FRAME;
    payload.valueId = id;
    payload.data = val;

    smartPortWriteFrame(&payload);
}

bool initSmartPortTelemetry(void)
{
    if (telemetryState == TELEMETRY_STATE_UNINITIALIZED) {
        portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_SMARTPORT);
        if (portConfig) {
            smartPortPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_SMARTPORT);

            smartPortWriteFrame = smartPortWriteFrameInternal;

            telemetryState = TELEMETRY_STATE_INITIALIZED_SERIAL;
        }

        return true;
    }

    return false;
}

bool initSmartPortTelemetryExternal(smartPortWriteFrameFn *smartPortWriteFrameExternal)
{
    if (telemetryState == TELEMETRY_STATE_UNINITIALIZED) {
        smartPortWriteFrame = smartPortWriteFrameExternal;

        telemetryState = TELEMETRY_STATE_INITIALIZED_EXTERNAL;

        return true;
    }

    return false;
}

static void freeSmartPortTelemetryPort(void)
{
    closeSerialPort(smartPortSerialPort);
    smartPortSerialPort = NULL;
}

static void configureSmartPortTelemetryPort(void)
{
    if (portConfig) {
        portOptions_e portOptions = (telemetryConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) | (telemetryConfig()->telemetry_inverted ? SERIAL_NOT_INVERTED : SERIAL_INVERTED);

        smartPortSerialPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_SMARTPORT, NULL, NULL, SMARTPORT_BAUD, SMARTPORT_UART_MODE, portOptions);
    }
}

void checkSmartPortTelemetryState(void)
{
    if (telemetryState == TELEMETRY_STATE_INITIALIZED_SERIAL) {
        bool enableSerialTelemetry = telemetryDetermineEnabledState(smartPortPortSharing);

        if (enableSerialTelemetry && !smartPortSerialPort) {
            configureSmartPortTelemetryPort();
        } else if (!enableSerialTelemetry && smartPortSerialPort) {
            freeSmartPortTelemetryPort();
        }
    }
}

#if defined(USE_MSP_OVER_TELEMETRY)
static void smartPortSendMspResponse(uint8_t *data) {
    smartPortPayload_t payload;
    payload.frameId = FSSP_MSPS_FRAME;
    memcpy(&payload.valueId, data, SMARTPORT_MSP_PAYLOAD_SIZE);

    smartPortWriteFrame(&payload);
}
#endif

void processSmartPortTelemetry(smartPortPayload_t *payload, volatile bool *clearToSend, const uint32_t *requestTimeout)
{
    if (payload) {
        // do not check the physical ID here again
        // unless we start receiving other sensors' packets

#if defined(USE_MSP_OVER_TELEMETRY)
        if (payload->frameId == FSSP_MSPC_FRAME_SMARTPORT || payload->frameId == FSSP_MSPC_FRAME_FPORT) {
            // Pass only the payload: skip frameId
            uint8_t *frameStart = (uint8_t *)&payload->valueId;
            smartPortMspReplyPending = handleMspFrame(frameStart, SMARTPORT_MSP_PAYLOAD_SIZE);
        }
#endif
    }

    bool doRun = true;
    while (doRun && *clearToSend) {
        // Ensure we won't get stuck in the loop if there happens to be nothing available to send in a timely manner - dump the slot if we loop in there for too long.
        if (requestTimeout) {
            if (millis() >= *requestTimeout) {
                *clearToSend = false;

                return;
            }
        } else {
            doRun = false;
        }

#if defined(USE_MSP_OVER_TELEMETRY)
        if (smartPortMspReplyPending) {
            smartPortMspReplyPending = sendMspReply(SMARTPORT_MSP_PAYLOAD_SIZE, &smartPortSendMspResponse);
            *clearToSend = false;

            return;
        }
#endif

        // we can send back any data we want, our table keeps track of the order and frequency of each data type we send
        uint16_t id = frSkyDataIdTable[smartPortIdCnt];
        if (id == 0) { // end of table reached, loop back
            smartPortIdCnt = 0;
            id = frSkyDataIdTable[smartPortIdCnt];
        }
        smartPortIdCnt++;

        int32_t tmpi;
        uint32_t tmp2 = 0;
        static uint8_t t1Cnt = 0;
        static uint8_t t2Cnt = 0;

        switch (id) {
            case FSSP_DATAID_VFAS       :
                if (isBatteryVoltageConfigured()) {
                    uint16_t vfasVoltage;
                    if (telemetryConfig()->report_cell_voltage) {
                        const uint8_t cellCount = getBatteryCellCount();
                        vfasVoltage = cellCount ? getBatteryVoltage() / cellCount : 0;
                    } else {
                        vfasVoltage = getBatteryVoltage();
                    }
                    smartPortSendPackage(id, vfasVoltage * 10); // given in 0.1V, convert to volts
                    *clearToSend = false;
                }
                break;
            case FSSP_DATAID_CURRENT    :
                if (isAmperageConfigured()) {
                    smartPortSendPackage(id, getAmperage() / 10); // given in 10mA steps, unknown requested unit
                    *clearToSend = false;
                }
                break;
            //case FSSP_DATAID_RPM        :
            case FSSP_DATAID_ALTITUDE   :
                if (sensors(SENSOR_BARO)) {
                    smartPortSendPackage(id, getEstimatedAltitude()); // unknown given unit, requested 100 = 1 meter
                    *clearToSend = false;
                }
                break;
            case FSSP_DATAID_FUEL       :
                if (isAmperageConfigured()) {
                    smartPortSendPackage(id, getMAhDrawn()); // given in mAh, unknown requested unit
                    *clearToSend = false;
                }
                break;
            //case FSSP_DATAID_ADC1       :
            //case FSSP_DATAID_ADC2       :
            //case FSSP_DATAID_CAP_USED   :
            case FSSP_DATAID_VARIO      :
                if (sensors(SENSOR_BARO)) {
                    smartPortSendPackage(id, getEstimatedVario()); // unknown given unit but requested in 100 = 1m/s
                    *clearToSend = false;
                }
                break;
            case FSSP_DATAID_HEADING    :
                smartPortSendPackage(id, attitude.values.yaw * 10); // given in 10*deg, requested in 10000 = 100 deg
                *clearToSend = false;
                break;
            case FSSP_DATAID_ACCX       :
                smartPortSendPackage(id, 100 * acc.accADC[X] / acc.dev.acc_1G); // Multiply by 100 to show as x.xx g on Taranis
                *clearToSend = false;
                break;
            case FSSP_DATAID_ACCY       :
                smartPortSendPackage(id, 100 * acc.accADC[Y] / acc.dev.acc_1G);
                *clearToSend = false;
                break;
            case FSSP_DATAID_ACCZ       :
                smartPortSendPackage(id, 100 * acc.accADC[Z] / acc.dev.acc_1G);
                *clearToSend = false;
                break;
            case FSSP_DATAID_T1         :
                // we send all the flags as decimal digits for easy reading

                // the t1Cnt simply allows the telemetry view to show at least some changes
                t1Cnt++;
                if (t1Cnt >= 4) {
                    t1Cnt = 1;
                }
                tmpi = t1Cnt * 10000; // start off with at least one digit so the most significant 0 won't be cut off
                // the Taranis seems to be able to fit 5 digits on the screen
                // the Taranis seems to consider this number a signed 16 bit integer

                if (!isArmingDisabled()) {
                    tmpi += 1;
                } else {
                    tmpi += 2;
                }
                if (ARMING_FLAG(ARMED))
                    tmpi += 4;

                if (FLIGHT_MODE(ANGLE_MODE))
                    tmpi += 10;
                if (FLIGHT_MODE(HORIZON_MODE))
                    tmpi += 20;
                if (FLIGHT_MODE(UNUSED_MODE))
                    tmpi += 40;
                if (FLIGHT_MODE(PASSTHRU_MODE))
                    tmpi += 40;

                if (FLIGHT_MODE(MAG_MODE))
                    tmpi += 100;
                if (FLIGHT_MODE(BARO_MODE))
                    tmpi += 200;
                if (FLIGHT_MODE(RANGEFINDER_MODE))
                    tmpi += 400;

                if (FLIGHT_MODE(GPS_HOLD_MODE))
                    tmpi += 1000;
                if (FLIGHT_MODE(GPS_HOME_MODE))
                    tmpi += 2000;
                if (FLIGHT_MODE(HEADFREE_MODE))
                    tmpi += 4000;

                smartPortSendPackage(id, (uint32_t)tmpi);
                *clearToSend = false;
                break;
            case FSSP_DATAID_T2         :
                if (sensors(SENSOR_GPS)) {
#ifdef USE_GPS
                    // provide GPS lock status
                    smartPortSendPackage(id, (STATE(GPS_FIX) ? 1000 : 0) + (STATE(GPS_FIX_HOME) ? 2000 : 0) + gpsSol.numSat);
                    *clearToSend = false;
#endif
                } else if (feature(FEATURE_GPS)) {
                    smartPortSendPackage(id, 0);
                    *clearToSend = false;
                } else if (telemetryConfig()->pidValuesAsTelemetry) {
                    switch (t2Cnt) {
                        case 0:
                            tmp2 = currentPidProfile->pid[PID_ROLL].P;
                            tmp2 += (currentPidProfile->pid[PID_PITCH].P<<8);
                            tmp2 += (currentPidProfile->pid[PID_YAW].P<<16);
                        break;
                        case 1:
                            tmp2 = currentPidProfile->pid[PID_ROLL].I;
                            tmp2 += (currentPidProfile->pid[PID_PITCH].I<<8);
                            tmp2 += (currentPidProfile->pid[PID_YAW].I<<16);
                        break;
                        case 2:
                            tmp2 = currentPidProfile->pid[PID_ROLL].D;
                            tmp2 += (currentPidProfile->pid[PID_PITCH].D<<8);
                            tmp2 += (currentPidProfile->pid[PID_YAW].D<<16);
                        break;
                        case 3:
                            tmp2 = currentControlRateProfile->rates[FD_ROLL];
                            tmp2 += (currentControlRateProfile->rates[FD_PITCH]<<8);
                            tmp2 += (currentControlRateProfile->rates[FD_YAW]<<16);
                        break;
                    }
                    tmp2 += t2Cnt<<24;
                    t2Cnt++;
                    if (t2Cnt == 4) {
                        t2Cnt = 0;
                    }
                    smartPortSendPackage(id, tmp2);
                    *clearToSend = false;
                }
                break;
#ifdef USE_GPS
            case FSSP_DATAID_SPEED      :
                if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
                    //convert to knots: 1cm/s = 0.0194384449 knots
                    //Speed should be sent in knots/1000 (GPS speed is in cm/s)
                    uint32_t tmpui = gpsSol.groundSpeed * 1944 / 100;
                    smartPortSendPackage(id, tmpui);
                    *clearToSend = false;
                }
                break;
            case FSSP_DATAID_LATLONG    :
                if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
                    uint32_t tmpui = 0;
                    // the same ID is sent twice, one for longitude, one for latitude
                    // the MSB of the sent uint32_t helps FrSky keep track
                    // the even/odd bit of our counter helps us keep track
                    if (smartPortIdCnt & 1) {
                        tmpui = abs(gpsSol.llh.lon);  // now we have unsigned value and one bit to spare
                        tmpui = (tmpui + tmpui / 2) / 25 | 0x80000000;  // 6/100 = 1.5/25, division by power of 2 is fast
                        if (gpsSol.llh.lon < 0) tmpui |= 0x40000000;
                    }
                    else {
                        tmpui = abs(gpsSol.llh.lat);  // now we have unsigned value and one bit to spare
                        tmpui = (tmpui + tmpui / 2) / 25;  // 6/100 = 1.5/25, division by power of 2 is fast
                        if (gpsSol.llh.lat < 0) tmpui |= 0x40000000;
                    }
                    smartPortSendPackage(id, tmpui);
                    *clearToSend = false;
                }
                break;
            case FSSP_DATAID_HOME_DIST  :
                if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
                    smartPortSendPackage(id, GPS_distanceToHome);
                     *clearToSend = false;
                }
                break;
            case FSSP_DATAID_GPS_ALT    :
                if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
                    smartPortSendPackage(id, gpsSol.llh.alt * 100); // given in 0.1m , requested in 10 = 1m (should be in mm, probably a bug in opentx, tested on 2.0.1.7)
                    *clearToSend = false;
                }
                break;
#endif
            case FSSP_DATAID_A4         :
                if (isBatteryVoltageConfigured()) {
                    const uint8_t cellCount = getBatteryCellCount();
                    const uint32_t vfasVoltage = cellCount ? (getBatteryVoltage() * 10 / cellCount) : 0; // given in 0.1V, convert to volts
                    smartPortSendPackage(id, vfasVoltage);
                    *clearToSend = false;
                }
                break;
            case FSSP_DATAID_ASPD       :
                // Air speed. Not supported in BF yet.
                break;
            default:
                break;
                // if nothing is sent, hasRequest isn't cleared, we already incremented the counter, just loop back to the start
        }
    }
}

static bool serialCheckQueueEmpty(void)
{
    return (serialRxBytesWaiting(smartPortSerialPort) == 0);
}

void handleSmartPortTelemetry(void)
{
    static bool clearToSend = false;
    static volatile timeUs_t lastTelemetryFrameReceivedUs;
    static smartPortPayload_t *payload = NULL;

    const uint32_t requestTimeout = millis() + SMARTPORT_SERVICE_TIMEOUT_MS;

    if (telemetryState == TELEMETRY_STATE_INITIALIZED_SERIAL && smartPortSerialPort) {
        while (serialRxBytesWaiting(smartPortSerialPort) > 0 && !payload) {
            uint8_t c = serialRead(smartPortSerialPort);
            payload = smartPortDataReceive(c, &clearToSend, serialCheckQueueEmpty, true);
            if (payload) {
                lastTelemetryFrameReceivedUs = micros();
            }
        }

        if (cmpTimeUs(micros(), lastTelemetryFrameReceivedUs) >= SMARTPORT_MIN_TELEMETRY_RESPONSE_DELAY_US) {
            processSmartPortTelemetry(payload, &clearToSend, &requestTimeout);
            payload = NULL;
        }
    }
}
#endif
