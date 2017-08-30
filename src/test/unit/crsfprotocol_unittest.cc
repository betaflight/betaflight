#include <stdint.h>
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "common/crc.h"
    #include "common/filter.h"
    #include "common/streambuf.h"
    #include "common/utils.h"

    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"

    #include "drivers/sensor.h"

    #include "fc/config.h"
    #include "fc/controlrate_profile.h"

    #include "flight/pid.h"

    #include "interface/cli.h"
    #include "interface/crsf_protocol.h"
    #include "interface/settings.h"

    #include "rx/crsf.h"

    #include "sensors/gyro.h"
    PG_REGISTER_WITH_RESET_TEMPLATE(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);

    PG_RESET_TEMPLATE(gyroConfig_t, gyroConfig,
        .gyro_align = ALIGN_DEFAULT,
        .gyroMovementCalibrationThreshold = 48,
        .gyro_sync_denom = 8,
        .gyro_lpf = 0,
        .gyro_soft_lpf_type = FILTER_PT1,
        .gyro_soft_lpf_hz = 90,
        .gyro_use_32khz = false,
        .gyro_to_use = 0,
        .gyro_soft_notch_hz_1 = 400,
        .gyro_soft_notch_cutoff_1 = 300,
        .gyro_soft_notch_hz_2 = 200,
        .gyro_soft_notch_cutoff_2 = 100
    );
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define GYRO_SYNC_DENOM_INDEX 4
#define GYRO_NOTCH1_CUTOFF_INDEX 8

static uint8_t crsfPayload[CRSF_PAYLOAD_SIZE_MAX];
static uint8_t crsfFrame[CRSF_FRAME_SIZE_MAX];

TEST(CRSF, Indices)
{
    // check GYRO_SYNC_DENOM_INDEX indexes the correct value
    const clivalue_t *value = &valueTable[GYRO_SYNC_DENOM_INDEX];
    EXPECT_EQ(0, strcmp("gyro_sync_denom", value->name));

    // check GYRO_NOTCH1_CUTOFF_INDEX indexes the correct value
    value = &valueTable[GYRO_NOTCH1_CUTOFF_INDEX];
    EXPECT_EQ(0, strcmp("gyro_notch1_cutoff", value->name));
}

TEST(CRSF, packU8)
{
    sbuf_t crsfPayloadBuf;
    sbuf_t *sbuf = sbufInit(&crsfPayloadBuf, crsfPayload, ARRAYEND(crsfPayload));

    pgResetAll();

    const clivalue_t *value = &valueTable[GYRO_SYNC_DENOM_INDEX];
    EXPECT_EQ(1, value->config.minmax.min);
    EXPECT_EQ(32, value->config.minmax.max);
    EXPECT_EQ(8, *(uint8_t*)cliGetDefaultPointer(value)); // default
//!!    EXPECT_EQ(crsf_c::PARAM_DONT_SAVE, param->eepromAdd);
//!!    EXPECT_EQ(nullptr, param->hiddenIndex);

    crsfProtocolPackU8Cli(sbuf, value, CRSF_PARAM_SKIP_STRING);
    EXPECT_EQ(CRSF_UINT8, crsfPayload[0]);
    EXPECT_EQ(0, crsfPayload[1]); // name, zero since PARAM_SKIP_STRING used
    EXPECT_EQ(8, crsfPayload[2]); // value
    EXPECT_EQ(1, crsfPayload[3]); // min
    EXPECT_EQ(32, crsfPayload[4]); // max
    EXPECT_EQ(8, crsfPayload[5]); // default
    EXPECT_EQ(0, crsfPayload[6]); // units, zero since PARAM_SKIP_STRING used
}

TEST(CRSF, unPackU8)
{
    crsfPayload[CRSF_PARAMETER_WRITE_DATA_OFFSET] = 14;
    const clivalue_t *value = &valueTable[GYRO_SYNC_DENOM_INDEX];
    crsfProtocolUnpackU8Cli(value, crsfPayload);
    EXPECT_EQ(14, gyroConfig()->gyro_sync_denom);
}

TEST(CRSF, unPackU16)
{
    const uint16_t testValue = 1875;
    crsfPayload[CRSF_PARAMETER_WRITE_DATA_OFFSET] = testValue >> 8;
    crsfPayload[CRSF_PARAMETER_WRITE_DATA_OFFSET+1] = testValue & 0xFF;
    const clivalue_t *value = &valueTable[GYRO_NOTCH1_CUTOFF_INDEX];
    crsfProtocolUnpackU16Cli(value, crsfPayload);
    EXPECT_EQ(testValue, gyroConfig()->gyro_soft_notch_cutoff_1);
}

TEST(CRSF, parameterWrite)
{
    pgResetAll();

    sbuf_t crsfPayloadBuf;
    sbuf_t *sbuf = sbufInit(&crsfPayloadBuf, crsfPayload, ARRAYEND(crsfPayload));

    // Payload for write
    sbufWriteU8(sbuf, GYRO_SYNC_DENOM_INDEX); // parameterIndex
    sbufWriteU8(sbuf, 17); // Parameter value

    gyroConfigMutable()->gyro_sync_denom = 0;
    crsfProtocolParameterWrite(crsfPayload);
    // check the parameter has been updated by parameterWrite
    EXPECT_EQ(17, gyroConfig()->gyro_sync_denom);
}

TEST(CRSF, ParameterRead)
{
    pgResetAll();

    sbuf_t crsfPayloadBuf;
    sbuf_t *sbuf = sbufInit(&crsfPayloadBuf, crsfPayload, ARRAYEND(crsfPayload));

    // Payload for read
    sbufWriteU8(sbuf, GYRO_SYNC_DENOM_INDEX); // parameterIndex
    sbufWriteU8(sbuf, 0); // Chunk index

    gyroConfigMutable()->gyro_sync_denom = 22;

    static uint8_t crsfReadFrame[CRSF_FRAME_SIZE_MAX];
    sbuf_t crsfReadFrameBuf;
    sbuf_t * const dst = sbufInit(&crsfReadFrameBuf, crsfReadFrame, ARRAYEND(crsfReadFrame));

    // reading a parameter will return a FRAMETYPE_PARAMETER_SETTINGS_ENTRY frame
    crsfProtocolParameterRead(dst, crsfPayload, CRSF_ADDRESS_RESERVED1, CRSF_PARAM_SKIP_STRING);
    //<Frame length><Type><Destination Address><Origin Address><Payload><CRC>
    EXPECT_EQ(14, crsfReadFrame[0]); // length
    EXPECT_EQ(CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, crsfReadFrame[1]); // type
    EXPECT_EQ(CRSF_ADDRESS_RESERVED1, crsfReadFrame[2]); // destination address
    EXPECT_EQ(CRSF_ADDRESS_FLIGHT_CONTROLLER, crsfReadFrame[3]); // our address
    EXPECT_EQ(GYRO_SYNC_DENOM_INDEX, crsfReadFrame[4]); // parameter index
    EXPECT_EQ(0, crsfReadFrame[5]); // chunk
    EXPECT_EQ(1, crsfReadFrame[6]); // parent folder
    EXPECT_EQ(CRSF_UINT8, crsfReadFrame[7]); // parameter type
    EXPECT_EQ(0, crsfReadFrame[8]); // name, zero since PARAM_SKIP_STRING used
    EXPECT_EQ(22, crsfReadFrame[9]); // value = 22
    EXPECT_EQ(1, crsfReadFrame[10]); // min = 1
    EXPECT_EQ(32, crsfReadFrame[11]); // max = 32
    EXPECT_EQ(8, crsfReadFrame[12]); // default = 8
    EXPECT_EQ(0, crsfReadFrame[13]); // unit, zero since PARAM_SKIP_STRING used
    EXPECT_EQ(crc8_dvb_s2_update(0, &crsfReadFrame[1], 13), crsfReadFrame[14]); // CRC
}

TEST(CRSF, InterpretExtendedFrameWrite)
{
    pgResetAll();

    sbuf_t crsfFrameBuf;
    sbuf_t *sbuf = sbufInit(&crsfFrameBuf, crsfFrame, ARRAYEND(crsfFrame));

    // Frame
    //<Device address  or Sync Byte><Frame length><Type><Destination Address><Origin Address><Payload><CRC>
    sbufWriteU8(sbuf, CRSF_SYNC_BYTE);
    // write zero for frame length, since we don't know it yet
    uint8_t * const lengthPtr = sbuf->ptr;
    sbufWriteU8(sbuf, 0);
    sbufWriteU8(sbuf, CRSF_FRAMETYPE_PARAMETER_WRITE);
    sbufWriteU8(sbuf, CRSF_ADDRESS_FLIGHT_CONTROLLER); // destination address
    sbufWriteU8(sbuf, CRSF_ADDRESS_RADIO_TRANSMITTER); // origin address
    // Payload for write
    sbufWriteU8(sbuf, GYRO_NOTCH1_CUTOFF_INDEX); // parameterIndex
    sbufWriteU16BigEndian(sbuf, 107); // Parameter value
    // write in the frame length
    *lengthPtr = sbuf->ptr - lengthPtr;
    crc8_dvb_s2_sbuf_append(sbuf, lengthPtr + CRSF_FRAME_LENGTH_TYPE);

    static uint8_t crsfReadFrame[CRSF_FRAME_SIZE_MAX];
    sbuf_t crsfReadFrameBuf;
    sbuf_t * const dst = sbufInit(&crsfReadFrameBuf, crsfReadFrame, ARRAYEND(crsfReadFrame));

    // set the parameter to zero and then call crsfInterpretExtendedFrame
    gyroConfigMutable()->gyro_soft_notch_cutoff_1 = 0;
    crsfInterpretExtendedFrame(dst, crsfFrame);
    // check the parameter has been updated by crsfInterpretExtendedFrame
    EXPECT_EQ(107, gyroConfig()->gyro_soft_notch_cutoff_1);
}

TEST(CRSF, InterpretExtendedFrameRead)
{
    pgResetAll();

    sbuf_t crsfFrameBuf;
    sbuf_t *sbuf = sbufInit(&crsfFrameBuf, crsfFrame, ARRAYEND(crsfFrame));

    // Frame
    //<Device address  or Sync Byte><Frame length><Type><Destination Address><Origin Address><Payload><CRC>
    sbufWriteU8(sbuf, CRSF_SYNC_BYTE);
    // write zero for frame length, since we don't know it yet
    uint8_t *lengthPtr = sbuf->ptr;
    sbufWriteU8(sbuf, 0);
    sbufWriteU8(sbuf, CRSF_FRAMETYPE_PARAMETER_READ);
    sbufWriteU8(sbuf, CRSF_ADDRESS_FLIGHT_CONTROLLER); // destination address
    sbufWriteU8(sbuf, CRSF_ADDRESS_RADIO_TRANSMITTER); // origin address
    // Payload for read
    sbufWriteU8(sbuf, GYRO_NOTCH1_CUTOFF_INDEX); // parameterIndex
    sbufWriteU8(sbuf, 0); // Chunk index
    // write in the frame length
    *lengthPtr = sbuf->ptr - lengthPtr;
    crc8_dvb_s2_sbuf_append(sbuf, lengthPtr + CRSF_FRAME_LENGTH_TYPE);

    gyroConfigMutable()->gyro_soft_notch_cutoff_1 = 22;

    static uint8_t crsfReadFrame[CRSF_FRAME_SIZE_MAX];
    sbuf_t crsfReadFrameBuf;
    sbuf_t * const dst = sbufInit(&crsfReadFrameBuf, crsfReadFrame, ARRAYEND(crsfReadFrame));

    // interpret the frame we have set up and check the correct values returned
    crsfInterpretExtendedFrame(dst, crsfFrame);
    EXPECT_EQ(36, crsfReadFrame[0]); // length
    EXPECT_EQ(CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, crsfReadFrame[1]); // type
    EXPECT_EQ(CRSF_ADDRESS_RADIO_TRANSMITTER, crsfReadFrame[2]); // destination address
    EXPECT_EQ(CRSF_ADDRESS_FLIGHT_CONTROLLER, crsfReadFrame[3]); // our address
    EXPECT_EQ(GYRO_NOTCH1_CUTOFF_INDEX, crsfReadFrame[4]); // parameter index
    EXPECT_EQ(0, crsfReadFrame[5]); // chunk
    EXPECT_EQ(1, crsfReadFrame[6]); // parent folder
    EXPECT_EQ(CRSF_UINT16, crsfReadFrame[7]); // parameter type
    // gyro_notch1_cutoff
    EXPECT_EQ('g', crsfReadFrame[8]); // name
    EXPECT_EQ('y', crsfReadFrame[9]);
    EXPECT_EQ('r', crsfReadFrame[10]);
    EXPECT_EQ('o', crsfReadFrame[11]);
    EXPECT_EQ('_', crsfReadFrame[12]);
    EXPECT_EQ('n', crsfReadFrame[13]);
    EXPECT_EQ('o', crsfReadFrame[14]);
    EXPECT_EQ('t', crsfReadFrame[15]);
    EXPECT_EQ('c', crsfReadFrame[16]);
    EXPECT_EQ('h', crsfReadFrame[17]);
    EXPECT_EQ('1', crsfReadFrame[18]);
    EXPECT_EQ('_', crsfReadFrame[19]);
    EXPECT_EQ('c', crsfReadFrame[20]);
    EXPECT_EQ('u', crsfReadFrame[21]);
    EXPECT_EQ('t', crsfReadFrame[22]);
    EXPECT_EQ('o', crsfReadFrame[23]);
    EXPECT_EQ('f', crsfReadFrame[24]);
    EXPECT_EQ('f', crsfReadFrame[25]);
    EXPECT_EQ(0, crsfReadFrame[26]);
    EXPECT_EQ(0, crsfReadFrame[27]); // value
    EXPECT_EQ(22, crsfReadFrame[28]);
    EXPECT_EQ(0, crsfReadFrame[29]); // min
    EXPECT_EQ(0, crsfReadFrame[30]);
    EXPECT_EQ(0x3E, crsfReadFrame[31]); // max 16000 = 0x3E80
    EXPECT_EQ(0x80, crsfReadFrame[32]);
    EXPECT_EQ(0x01, crsfReadFrame[33]); // default 300 = 0x012C
    EXPECT_EQ(0x2C, crsfReadFrame[34]);
    EXPECT_EQ(0, crsfReadFrame[35]); // unit, zero since units not currently supported
    EXPECT_EQ(crc8_dvb_s2_update(0, &crsfReadFrame[1], 35), crsfReadFrame[36]); // CRC
}


extern "C" {
// STUBS
uint8_t getCurrentPidProfileIndex(void) {return 0;}
uint8_t getCurrentControlRateProfileIndex(void) {return 0;}

static uint16_t cliGetValueOffset(const clivalue_t *value)
{
    switch (value->type & VALUE_SECTION_MASK) {
    case MASTER_VALUE:
        return value->offset;
    case PROFILE_VALUE:
        return value->offset + sizeof(pidProfile_t) * getCurrentPidProfileIndex();
    case PROFILE_RATE_VALUE:
        return value->offset + sizeof(controlRateConfig_t) * getCurrentControlRateProfileIndex();
    }
    return 0;
}

void *cliGetValuePointer(const clivalue_t *value)
{
    const pgRegistry_t* pg = pgFind(value->pgn);
    return CONST_CAST(void *, pg->address + cliGetValueOffset(value));
}

const void *cliGetDefaultPointer(const clivalue_t *value)
{
    const pgRegistry_t* pg = pgFind(value->pgn);
    pgResetInstance(pg, pg->copy);
    const void *copy =  pg->copy + cliGetValueOffset(value);
    return copy;
}

}
