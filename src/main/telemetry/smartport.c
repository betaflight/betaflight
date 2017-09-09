/*
 * SmartPort Telemetry implementation by frank26080115
 * see https://github.com/frank26080115/cleanflight/wiki/Using-Smart-Port
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#if defined(TELEMETRY) && defined(TELEMETRY_SMARTPORT)

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/time.h"
#include "drivers/serial.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_msp.h"

#include "flight/imu.h"

#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"

#include "navigation/navigation.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/barometer.h"
#include "sensors/pitotmeter.h"

#include "telemetry/telemetry.h"
#include "telemetry/smartport.h"


enum
{
    SPSTATE_UNINITIALIZED,
    SPSTATE_INITIALIZED,
    SPSTATE_WORKING,
};

enum
{
    FSSP_START_STOP = 0x7E,

    FSSP_DLE        = 0x7D,
    FSSP_DLE_XOR    = 0x20,

    FSSP_DATA_FRAME = 0x10,
    FSSP_MSPC_FRAME = 0x30, // MSP client frame
    FSSP_MSPS_FRAME = 0x32, // MSP server frame

    // ID of sensor. Must be something that is polled by FrSky RX
    FSSP_SENSOR_ID1 = 0x1B,
    FSSP_SENSOR_ID2 = 0x0D,
    FSSP_SENSOR_ID3 = 0x34,
    FSSP_SENSOR_ID4 = 0x67,
    // there are 32 ID's polled by smartport master
    // remaining 3 bits are crc (according to comments in openTx code)
};

// these data identifiers are obtained from https://github.com/opentx/opentx/blob/master/radio/src/telemetry/frsky.h
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
    FSSP_DATAID_A4         = 0x0910 ,
};

const uint16_t smartPortDataIdTable[] = {
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
    //FSSP_DATAID_A3        ,
    FSSP_DATAID_A4        ,
};

#define __USE_C99_MATH // for roundf()
#define SMARTPORT_BAUD 57600
#define SMARTPORT_UART_MODE MODE_RXTX
#define SMARTPORT_SERVICE_TIMEOUT_MS 1 // max allowed time to find a value to send
#define SMARTPORT_DATA_LENGTH (sizeof(smartPortDataIdTable) / sizeof(smartPortDataIdTable[0]))

static serialPort_t *smartPortSerialPort = NULL; // The 'SmartPort'(tm) Port.
static serialPortConfig_t *portConfig;

static bool smartPortTelemetryEnabled =  false;
static portSharing_e smartPortPortSharing;

char smartPortState = SPSTATE_UNINITIALIZED;
static bool smartPortHasRequest = false;
static uint8_t smartPortIdCnt = 0;
static uint32_t smartPortLastRequestTime = 0;

typedef struct smartPortFrame_s {
    uint8_t  sensorId;
    uint8_t  frameId;
    uint16_t valueId;
    uint32_t data;
    uint8_t  crc;
} __attribute__((packed)) smartPortFrame_t;

#define SMARTPORT_FRAME_SIZE  sizeof(smartPortFrame_t)
#define SMARTPORT_TX_BUF_SIZE 256

#define SMARTPORT_PAYLOAD_OFFSET offsetof(smartPortFrame_t, valueId)
#define SMARTPORT_PAYLOAD_SIZE   (SMARTPORT_FRAME_SIZE - SMARTPORT_PAYLOAD_OFFSET - 1)

static smartPortFrame_t smartPortRxBuffer;
static uint8_t smartPortRxBytes = 0;
static bool smartPortFrameReceived = false;

#define SMARTPORT_MSP_VERSION    1
#define SMARTPORT_MSP_VER_SHIFT  5
#define SMARTPORT_MSP_VER_MASK   (0x7 << SMARTPORT_MSP_VER_SHIFT)
#define SMARTPORT_MSP_VERSION_S  (SMARTPORT_MSP_VERSION << SMARTPORT_MSP_VER_SHIFT)

#define SMARTPORT_MSP_ERROR_FLAG (1 << 5)
#define SMARTPORT_MSP_START_FLAG (1 << 4)
#define SMARTPORT_MSP_SEQ_MASK   0x0F

#define SMARTPORT_MSP_RX_BUF_SIZE 64

static uint8_t smartPortMspTxBuffer[SMARTPORT_TX_BUF_SIZE];
static mspPacket_t smartPortMspReply;
static bool smartPortMspReplyPending = false;

#define SMARTPORT_MSP_RES_ERROR (-10)

enum {
    SMARTPORT_MSP_VER_MISMATCH = 0,
    SMARTPORT_MSP_CRC_ERROR    = 1,
    SMARTPORT_MSP_ERROR        = 2,
};

static void smartPortDataReceive(uint16_t c)
{
    static bool skipUntilStart = true;
    static bool byteStuffing = false;
    static uint16_t checksum = 0;

    uint32_t now = millis();

    if (c == FSSP_START_STOP) {
        smartPortRxBytes = 0;
        smartPortHasRequest = false;
        skipUntilStart = false;
        return;
    } else if (skipUntilStart)
        return;

    uint8_t* rxBuffer = (uint8_t*)&smartPortRxBuffer;
    if (smartPortRxBytes == 0) {
        if ((c == FSSP_SENSOR_ID1) && (serialRxBytesWaiting(smartPortSerialPort) == 0)) {

            // our slot is starting...
            smartPortLastRequestTime = now;
            smartPortHasRequest = true;
        } else if (c == FSSP_SENSOR_ID2) {
            rxBuffer[smartPortRxBytes++] = c;
            checksum = 0;
        } else
            skipUntilStart = true;
    }
    else {

        if (c == FSSP_DLE) {
            byteStuffing = true;
            return;
        }

        if (byteStuffing) {
            c ^= FSSP_DLE_XOR;
            byteStuffing = false;
        }

        rxBuffer[smartPortRxBytes++] = c;

        if (smartPortRxBytes == SMARTPORT_FRAME_SIZE) {
            if (c == (0xFF - checksum))
                smartPortFrameReceived = true;
            skipUntilStart = true;
        } else if (smartPortRxBytes < SMARTPORT_FRAME_SIZE) {
            checksum += c;
            checksum += checksum >> 8;
            checksum &= 0x00FF;
        }
    }
}

static void smartPortSendByte(uint8_t c, uint16_t *crcp)
{
    // smart port escape sequence
    if (c == FSSP_DLE || c == FSSP_START_STOP) {
        serialWrite(smartPortSerialPort, FSSP_DLE);
        serialWrite(smartPortSerialPort, c ^ FSSP_DLE_XOR);
    } else
        serialWrite(smartPortSerialPort, c);

    if (crcp == NULL)
        return;

    uint16_t crc = *crcp;
    crc += c;
    crc += crc >> 8;
    crc &= 0x00FF;
    *crcp = crc;
}

static void smartPortSendPackageEx(uint8_t frameId, uint8_t* data)
{
    uint16_t crc = 0;
    smartPortSendByte(frameId, &crc);
    for (unsigned i = 0; i < SMARTPORT_PAYLOAD_SIZE; i++)
        smartPortSendByte(*data++, &crc);
    smartPortSendByte(0xFF - (uint8_t)crc, NULL);
}

static void smartPortSendPackage(uint16_t id, uint32_t val)
{
    uint8_t payload[SMARTPORT_PAYLOAD_SIZE];
    uint8_t *dst = payload;
    *dst++ = id & 0xFF;
    *dst++ = id >> 8;
    *dst++ = val & 0xFF;
    *dst++ = (val >> 8) & 0xFF;
    *dst++ = (val >> 16) & 0xFF;
    *dst++ = (val >> 24) & 0xFF;

    smartPortSendPackageEx(FSSP_DATA_FRAME,payload);
    smartPortHasRequest = false;
}

void initSmartPortTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_SMARTPORT);
    smartPortPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_SMARTPORT);
}

void freeSmartPortTelemetryPort(void)
{
    closeSerialPort(smartPortSerialPort);
    smartPortSerialPort = NULL;

    smartPortState = SPSTATE_UNINITIALIZED;
    smartPortTelemetryEnabled = false;
}

void configureSmartPortTelemetryPort(void)
{
    portOptions_t portOptions;

    if (!portConfig)
        return;

    if (telemetryConfig()->smartportUartUnidirectional)
        portOptions = SERIAL_UNIDIR;
    else
        portOptions = SERIAL_BIDIR;

    if (telemetryConfig()->telemetry_inversion)
        portOptions |= SERIAL_INVERTED;

    smartPortSerialPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_SMARTPORT, NULL, SMARTPORT_BAUD, SMARTPORT_UART_MODE, portOptions);

    if (!smartPortSerialPort)
        return;

    smartPortState = SPSTATE_INITIALIZED;
    smartPortTelemetryEnabled = true;
    smartPortLastRequestTime = millis();
}

bool canSendSmartPortTelemetry(void)
{
    return smartPortSerialPort && (smartPortState == SPSTATE_INITIALIZED || smartPortState == SPSTATE_WORKING);
}

void checkSmartPortTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(smartPortPortSharing);

    if (newTelemetryEnabledValue == smartPortTelemetryEnabled)
        return;

    if (newTelemetryEnabledValue)
        configureSmartPortTelemetryPort();
    else
        freeSmartPortTelemetryPort();
}

static void initSmartPortMspReply(int16_t cmd)
{
    smartPortMspReply.buf.ptr    = smartPortMspTxBuffer;
    smartPortMspReply.buf.end    = ARRAYEND(smartPortMspTxBuffer);

    smartPortMspReply.cmd    = cmd;
    smartPortMspReply.result = 0;
}

static void processMspPacket(mspPacket_t* packet)
{
    initSmartPortMspReply(0);

    if (mspFcProcessCommand(packet, &smartPortMspReply, NULL) == MSP_RESULT_ERROR)
        sbufWriteU8(&smartPortMspReply.buf, SMARTPORT_MSP_ERROR);

    // change streambuf direction
    sbufSwitchToReader(&smartPortMspReply.buf, smartPortMspTxBuffer);
    smartPortMspReplyPending = true;
}

/**
 * Request frame format:
 * - Header: 1 byte
 *   - Reserved: 2 bits (future use)
 *   - Error-flag: 1 bit
 *   - Start-flag: 1 bit
 *   - CSeq: 4 bits
 *
 * - MSP payload:
 *   - if Error-flag == 0:
 *     - size: 1 byte
 *     - payload
 *     - CRC (request type included)
 *   - if Error-flag == 1:
 *     - size: 1 byte (== 1)
 *     - error: 1 Byte
 *       - 0: Version mismatch (type=0)
 *       - 1: Sequence number error
 *       - 2: MSP error
 *     - CRC (request type included)
 */
bool smartPortSendMspReply()
{
    static uint8_t checksum = 0;
    static uint8_t seq = 0;

    uint8_t packet[SMARTPORT_PAYLOAD_SIZE];
    uint8_t* p = packet;
    uint8_t* end = p + SMARTPORT_PAYLOAD_SIZE;

    sbuf_t* txBuf = &smartPortMspReply.buf;

    // detect first reply packet
    if (txBuf->ptr == smartPortMspTxBuffer) {

        // header
        uint8_t head = SMARTPORT_MSP_START_FLAG | (seq++ & SMARTPORT_MSP_SEQ_MASK);
        if (smartPortMspReply.result < 0)
            head |= SMARTPORT_MSP_ERROR_FLAG;
        *p++ = head;

        uint8_t size = sbufBytesRemaining(txBuf);
        *p++ = size;

        checksum = size ^ smartPortMspReply.cmd;
    } else {
        // header
        *p++ = (seq++ & SMARTPORT_MSP_SEQ_MASK);
    }

    while ((p < end) && (sbufBytesRemaining(txBuf) > 0)) {
        *p = sbufReadU8(txBuf);
        checksum ^= *p++; // MSP checksum
    }

    // to be continued...
    if (p == end) {
        smartPortSendPackageEx(FSSP_MSPS_FRAME, packet);
        return true;
    }

    // nothing left in txBuf,
    // append the MSP checksum
    *p++ = checksum;

    // pad with zeros
    while (p < end)
        *p++ = 0;

    smartPortSendPackageEx(FSSP_MSPS_FRAME, packet);
    return false;
}

void smartPortSendErrorReply(uint8_t error, int16_t cmd)
{
    initSmartPortMspReply(cmd);
    sbufWriteU8(&smartPortMspReply.buf, error);
    smartPortMspReply.result = SMARTPORT_MSP_RES_ERROR;

    sbufSwitchToReader(&smartPortMspReply.buf, smartPortMspTxBuffer);
    smartPortMspReplyPending = true;
}

/**
 * Request frame format:
 * - Header: 1 byte
 *   - Version: 3 bits
 *   - Start-flag: 1 bit
 *   - CSeq: 4 bits
 *
 * - MSP payload:
 *   - Size: 1 Byte
 *   - Type: 1 Byte
 *   - payload...
 *   - CRC
 */
void handleSmartPortMspFrame(smartPortFrame_t* sp_frame)
{
    static uint8_t mspBuffer[SMARTPORT_MSP_RX_BUF_SIZE];
    static uint8_t mspStarted = 0;
    static uint8_t lastSeq = 0;
    static uint8_t checksum = 0;
    static mspPacket_t cmd;

    // re-assemble MSP frame & forward to MSP port when complete
    uint8_t* p = ((uint8_t*)sp_frame) + SMARTPORT_PAYLOAD_OFFSET;
    uint8_t* end = p + SMARTPORT_PAYLOAD_SIZE;

    uint8_t head = *p++;
    uint8_t seq = head & SMARTPORT_MSP_SEQ_MASK;
    uint8_t version = (head & SMARTPORT_MSP_VER_MASK) >> SMARTPORT_MSP_VER_SHIFT;

    if (version != SMARTPORT_MSP_VERSION) {
        mspStarted = 0;
        smartPortSendErrorReply(SMARTPORT_MSP_VER_MISMATCH,0);
        return;
    }

    // check start-flag
    if (head & SMARTPORT_MSP_START_FLAG) {

        //TODO: if (p_size > SMARTPORT_MSP_RX_BUF_SIZE) error!
        uint8_t p_size = *p++;
        cmd.cmd = *p++;
        cmd.result = 0;

        cmd.buf.ptr = mspBuffer;
        cmd.buf.end = mspBuffer + p_size;

        checksum = p_size ^ cmd.cmd;
        mspStarted = 1;
    } else if (!mspStarted)
        return; // no start packet yet, throw this one away
    else if (((lastSeq + 1) & SMARTPORT_MSP_SEQ_MASK) != seq) {
        // packet loss detected!
        mspStarted = 0;
        return;
    }

    // copy payload bytes
    while ((p < end) && sbufBytesRemaining(&cmd.buf)) {
        checksum ^= *p;
        sbufWriteU8(&cmd.buf,*p++);
    }

    // reached end of smart port frame
    if (p == end) {
        lastSeq = seq;
        return;
    }

    // last byte must be the checksum
    if (checksum != *p) {
        mspStarted = 0;
        smartPortSendErrorReply(SMARTPORT_MSP_CRC_ERROR,cmd.cmd);
        return;
    }

    // end of MSP packet reached
    mspStarted = 0;
    sbufSwitchToReader(&cmd.buf,mspBuffer);

    processMspPacket(&cmd);
}

void handleSmartPortTelemetry(void)
{
    uint32_t smartPortLastServiceTime = millis();

    if (!smartPortTelemetryEnabled)
        return;

    if (!canSendSmartPortTelemetry())
        return;

    while (serialRxBytesWaiting(smartPortSerialPort) > 0) {
        uint8_t c = serialRead(smartPortSerialPort);
        smartPortDataReceive(c);
    }

    if (smartPortFrameReceived) {
        smartPortFrameReceived = false;
        // do not check the physical ID here again
        // unless we start receiving other sensors' packets
        if (smartPortRxBuffer.frameId == FSSP_MSPC_FRAME)
            handleSmartPortMspFrame(&smartPortRxBuffer); // Pass only the payload: skip sensorId & frameId
    }

    while (smartPortHasRequest) {
        // Ensure we won't get stuck in the loop if there happens to be nothing available to send in a timely manner - dump the slot if we loop in there for too long.
        if ((millis() - smartPortLastServiceTime) > SMARTPORT_SERVICE_TIMEOUT_MS) {
            smartPortHasRequest = false;
            return;
        }

        if (smartPortMspReplyPending) {
            smartPortMspReplyPending = smartPortSendMspReply();
            smartPortHasRequest = 0;
            return;
        }

        // we can send back any data we want, our table keeps track of the order and frequency of each data type we send
        smartPortIdCnt++;
        if (smartPortIdCnt >= SMARTPORT_DATA_LENGTH)
            smartPortIdCnt = 0; // end of table reached, loop back
        uint16_t id = smartPortDataIdTable[smartPortIdCnt];

        switch (id) {
#ifdef GPS
            case FSSP_DATAID_SPEED      :
                //convert to knots: 1cm/s = 0.0194384449 knots
                //Speed should be sent in knots/1000 (GPS speed is in cm/s)
                if (sensors(SENSOR_GPS) && STATE(GPS_FIX))
                    smartPortSendPackage(id, (uint32_t)gpsSol.groundSpeed * 1944 / 100);
                break;
#endif
            case FSSP_DATAID_VFAS       :
                if (feature(FEATURE_VBAT)) {
                    uint16_t vfasVoltage;
                    if (telemetryConfig()->frsky_vfas_cell_voltage)
                        vfasVoltage = vbat / batteryCellCount;
                    else
                        vfasVoltage = vbat;
                    smartPortSendPackage(id, vfasVoltage * 10); // given in 0.1V, convert to volts
                }
                break;
            case FSSP_DATAID_CURRENT    :
                if (feature(FEATURE_CURRENT_METER))
                    smartPortSendPackage(id, amperage / 10); // given in 10mA steps, unknown requested unit
                break;
            //case FSSP_DATAID_RPM        :
            case FSSP_DATAID_ALTITUDE   :
                if (sensors(SENSOR_BARO))
                    smartPortSendPackage(id, getEstimatedActualPosition(Z)); // unknown given unit, requested 100 = 1 meter
                break;
            case FSSP_DATAID_FUEL       :
                if (feature(FEATURE_CURRENT_METER)) {
                    if (telemetryConfig()->smartportFuelPercent && batteryConfig()->batteryCapacity > 0)
                        smartPortSendPackage(id, calculateBatteryCapacityRemainingPercentage()); // Show remaining battery % if smartport_fuel_percent=ON and battery_capacity set
                    else
                        smartPortSendPackage(id, mAhDrawn); // given in mAh, unknown requested unit
                }
                break;
            //case FSSP_DATAID_ADC1       :
            //case FSSP_DATAID_ADC2       :
#ifdef GPS
            case FSSP_DATAID_LATLONG    :
                if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
                    uint32_t tmpui = 0;
                    // the same ID is sent twice, one for longitude, one for latitude
                    // the MSB of the sent uint32_t helps FrSky keep track
                    // the even/odd bit of our counter helps us keep track
                    if (smartPortIdCnt & 1) {
                        tmpui = abs(gpsSol.llh.lon);  // now we have unsigned value and one bit to spare
                        tmpui = (tmpui + tmpui / 2) / 25 | 0x80000000;  // 6/100 = 1.5/25, division by power of 2 is fast
                        if (gpsSol.llh.lon < 0)
                            tmpui |= 0x40000000;
                    } else {
                        tmpui = abs(gpsSol.llh.lat);  // now we have unsigned value and one bit to spare
                        tmpui = (tmpui + tmpui / 2) / 25;  // 6/100 = 1.5/25, division by power of 2 is fast
                        if (gpsSol.llh.lat < 0)
                            tmpui |= 0x40000000;
                    }
                    smartPortSendPackage(id, tmpui);
                }
                break;
#endif
            case FSSP_DATAID_VARIO      :
                if (sensors(SENSOR_BARO))
                    smartPortSendPackage(id, lrintf(getEstimatedActualVelocity(Z))); // unknown given unit but requested in 100 = 1m/s
                break;
            case FSSP_DATAID_HEADING    :
                smartPortSendPackage(id, attitude.values.yaw * 10); // given in 10*deg, requested in 10000 = 100 deg
                break;
            case FSSP_DATAID_ACCX       :
                smartPortSendPackage(id, 100 * acc.accADC[X] / acc.dev.acc_1G);
                break;
            case FSSP_DATAID_ACCY       :
                smartPortSendPackage(id, 100 * acc.accADC[Y] / acc.dev.acc_1G);
                break;
            case FSSP_DATAID_ACCZ       :
                smartPortSendPackage(id, 100 * acc.accADC[Z] / acc.dev.acc_1G);
                break;
            case FSSP_DATAID_T1         :
                {
                    uint32_t tmpi = 10000; // start off with at least one digit so the most significant 0 won't be cut off

                    // ones column
                    if (!isArmingDisabled())
                        tmpi += 1;
                    else
                        tmpi += 2;

                    if (ARMING_FLAG(ARMED))
                        tmpi += 4;

                    // tens column
                    if (FLIGHT_MODE(ANGLE_MODE))
                        tmpi += 10;
                    if (FLIGHT_MODE(HORIZON_MODE))
                        tmpi += 20;
                    if (FLIGHT_MODE(PASSTHRU_MODE))
                        tmpi += 40;

                    // hundreds column
                    if (FLIGHT_MODE(HEADING_MODE))
                        tmpi += 100;
                    if (FLIGHT_MODE(NAV_ALTHOLD_MODE))
                        tmpi += 200;
                    if (FLIGHT_MODE(NAV_POSHOLD_MODE))
                        tmpi += 400;

                    // thousands column
                    if (FLIGHT_MODE(NAV_RTH_MODE))
                        tmpi += 1000;
                    if (FLIGHT_MODE(NAV_WP_MODE))
                        tmpi += 2000;
                    if (FLIGHT_MODE(HEADFREE_MODE))
                        tmpi += 4000;

                    // ten thousands column
                    if (true == false) // placeholder, would like to use this for home reset indicator
                        tmpi += 20000;
                    if (FLIGHT_MODE(FAILSAFE_MODE))
                        tmpi += 40000;

                    smartPortSendPackage(id, tmpi);
                    break;
                }
            case FSSP_DATAID_T2         :
                if (sensors(SENSOR_GPS)) {
#ifdef GPS
                    uint32_t tmpi = 0;

                    // ones and tens columns (# of satellites 0 - 99)
                    tmpi += constrain(gpsSol.numSat, 0, 99);

                    // hundreds column (satellite accuracy HDOP: 0 = worst, 9 = best)
                    tmpi += (9 - constrain(gpsSol.hdop / 1000, 0, 9)) * 100;

                    // thousands column (GPS fix status)
                    if (STATE(GPS_FIX))
                        tmpi += 1000;
                    if (STATE(GPS_FIX_HOME))
                        tmpi += 2000;

                    smartPortSendPackage(id, tmpi);
#endif
                } else if (feature(FEATURE_GPS))
                    smartPortSendPackage(id, 0);
                break;
#ifdef GPS
            case FSSP_DATAID_HOME_DIST  :
                if (sensors(SENSOR_GPS) && STATE(GPS_FIX))
                    smartPortSendPackage(id, GPS_distanceToHome);
                break;
            case FSSP_DATAID_GPS_ALT    :
                if (sensors(SENSOR_GPS) && STATE(GPS_FIX))
                    smartPortSendPackage(id, gpsSol.llh.alt); // cm
                break;
#endif
#ifdef PITOT
            case FSSP_DATAID_ASPD    :
                if (sensors(SENSOR_PITOT))
                    smartPortSendPackage(id, pitot.airSpeed * 0.194384449f); // cm/s to knots*10
                break;
#endif
            //case FSSP_DATAID_A3         :
            case FSSP_DATAID_A4         :
                if (feature(FEATURE_VBAT))
                    smartPortSendPackage(id, vbat * 10 / batteryCellCount ); // given in 0.1V, convert to volts
                break;
            default:
                break;
                // if nothing is sent, smartPortHasRequest isn't cleared, we already incremented the counter, just loop back to the start
        }
    }
}

#endif