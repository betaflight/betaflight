/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/crc.h"
#include "common/maths.h"
#include "common/streambuf.h"

#include "drivers/time.h"

#include "io/serial.h"

#include "rcdevice.h"

#ifdef USE_RCDEVICE

typedef enum {
    RCDP_SETTING_PARSE_WAITING_ID,
    RCDP_SETTING_PARSE_WAITING_NAME,
    RCDP_SETTING_PARSE_WAITING_VALUE,
} runcamDeviceSettingParseStep_e;

typedef struct runcamDeviceExpectedResponseLength_s {
    uint8_t command;
    uint8_t reponseLength;
} runcamDeviceExpectedResponseLength_t;

static runcamDeviceExpectedResponseLength_t expectedResponsesLength[] = {
    { RCDEVICE_PROTOCOL_COMMAND_GET_SETTINGS,               0xFF},
    { RCDEVICE_PROTOCOL_COMMAND_READ_SETTING_DETAIL,        0xFF},
    { RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO,            5},
    { RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_PRESS,      2},
    { RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_RELEASE,    2},
    { RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION,            3},
    { RCDEVICE_PROTOCOL_COMMAND_WRITE_SETTING,              4},
};

static uint8_t runcamDeviceGetResponseLength(uint8_t command)
{
    for (unsigned int i = 0; i < ARRAYLEN(expectedResponsesLength); i++) {
        if (expectedResponsesLength[i].command == command) {
            return expectedResponsesLength[i].reponseLength;
        }
    }

    return 0;
}

// Verify the response data has received done, return true if the data is still receiving or it was received done.
// return false if the packet has incorrect
static uint8_t runcamDeviceIsResponseReceiveDone(uint8_t command, uint8_t *data, uint8_t dataLen, bool *isDone)
{
    uint8_t expectedResponseDataLength = runcamDeviceGetResponseLength(command);
    if (expectedResponseDataLength == 0xFF) {
        uint8_t settingDataLength = 0x00;
        // get setting datalen first
        if (dataLen >= 3) {
            settingDataLength = data[2];
            if (dataLen >= (settingDataLength + 4)) {
                *isDone = true;
                return true;
            }
        }

        if (settingDataLength > 60) {
            return false;
        }
    } else if (dataLen >= expectedResponseDataLength) {
        *isDone = true;
        return true;
    }
    return true;
}

// a common way to receive packet and verify it
static uint8_t runcamDeviceReceivePacket(runcamDevice_t *device, uint8_t command, uint8_t *data, int timeoutms)
{
    uint8_t dataPos = 0;
    uint8_t crc = 0;
    uint8_t responseDataLen = 0;

    // wait for reply until timeout(specialy by timeoutms)
    timeMs_t timeout = millis() + timeoutms;
    bool isWaitingHeader = true;
    while (millis() < timeout) {
        if (serialRxBytesWaiting(device->serialPort) > 0) {
            uint8_t c = serialRead(device->serialPort);
            crc = crc8_dvb_s2(crc, c);

            if (data) {
                data[dataPos] = c;
            }
            dataPos++;

            if (isWaitingHeader) {
                if (c == RCDEVICE_PROTOCOL_HEADER) {
                    isWaitingHeader = false;
                }
            } else {
                bool isDone = false;
                if (!runcamDeviceIsResponseReceiveDone(command, data, dataPos, &isDone)) {
                    return 0;
                }
    
                if (isDone) {
                    responseDataLen = dataPos;
                    break;
                }
            }
        }
    }

    // check crc
    if (crc != 0) {
        return 0;
    }

    return responseDataLen;
}

// every time send packet to device, and want to get something from device,
// it'd better call the method to clear the rx buffer before the packet send,
// else may be the useless data in rx buffer will cause the response decoding
// failed.
static void runcamDeviceFlushRxBuffer(runcamDevice_t *device)
{
    while (serialRxBytesWaiting(device->serialPort) > 0) {
        serialRead(device->serialPort);
    }
}

// a common way to send packet to device
static void runcamDeviceSendPacket(runcamDevice_t *device, uint8_t command, uint8_t *paramData, int paramDataLen)
{
    // is this device open?
    if (!device->serialPort) {
        return;
    }

    sbuf_t buf;
    // prepare pointer
    buf.ptr = device->buffer;
    buf.end = ARRAYEND(device->buffer);

    sbufWriteU8(&buf, RCDEVICE_PROTOCOL_HEADER);
    sbufWriteU8(&buf, command);

    if (paramData) {
        sbufWriteData(&buf, paramData, paramDataLen);
    }

    // add crc over (all) data
    crc8_dvb_s2_sbuf_append(&buf, device->buffer);

    // switch to reader
    sbufSwitchToReader(&buf, device->buffer);

    // send data if possible
    serialWriteBuf(device->serialPort, sbufPtr(&buf), sbufBytesRemaining(&buf));
}

// a common way to send a packet to device, and get response from the device.
static bool runcamDeviceSendRequestAndWaitingResp(runcamDevice_t *device, uint8_t commandID, uint8_t *paramData, uint8_t paramDataLen, uint8_t *outputBuffer, uint8_t *outputBufferLen)
{
    int max_retries = 1;
    // here using 1000ms as timeout, because the response from 5 key simulation command need a long time about >= 600ms, 
    // so set a max value to ensure we can receive the response
    int timeoutMs = 1000; 

    // only the command sending on initializing step need retry logic, 
    // otherwise, the timeout of 1000 ms is enough for the response from device
    if (commandID == RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO) {
        max_retries = 3;
        timeoutMs = 100; // we have test some device, 100ms as timeout, and retry times be 3, it's stable for most case
    }

    while (max_retries--) {
        // flush rx buffer
        runcamDeviceFlushRxBuffer(device);

        // send packet
        runcamDeviceSendPacket(device, commandID, paramData, paramDataLen);

        // waiting response
        uint8_t responseLength = runcamDeviceReceivePacket(device, commandID, outputBuffer, timeoutMs);
        if (responseLength) {
            if (outputBufferLen) {
                *outputBufferLen = responseLength;
            }

            return true;
        }
    }

    return false;
}

// get the device info(firmware version, protocol version and features, see the
// definition of runcamDeviceInfo_t to know more)
static bool runcamDeviceGetDeviceInfo(runcamDevice_t *device, uint8_t *outputBuffer) 
{
    return runcamDeviceSendRequestAndWaitingResp(device, RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO, NULL, 0, outputBuffer, NULL); 
}

static bool runcamDeviceSend5KeyOSDCableConnectionEvent(runcamDevice_t *device, uint8_t operation, uint8_t *outActionID, uint8_t *outErrorCode)
{
    uint8_t outputDataLen = RCDEVICE_PROTOCOL_MAX_PACKET_SIZE;
    uint8_t respBuf[RCDEVICE_PROTOCOL_MAX_PACKET_SIZE];
    if (!runcamDeviceSendRequestAndWaitingResp(device, RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION, &operation, sizeof(uint8_t), respBuf, &outputDataLen)) {
        return false;
    }

    // the high 4 bits is the operationID that we sent
    // the low 4 bits is the result code
    uint8_t operationID = (respBuf[1] & 0xF0) >> 4;
    bool errorCode = (respBuf[1] & 0x0F);
    if (outActionID) {
        *outActionID = operationID;
    }

    if (outErrorCode) {
        *outErrorCode = errorCode;
    }

    return true;
}

// init the runcam device, it'll search the UART port with FUNCTION_RCDEVICE id
// this function will delay 400ms in the first loop to wait the device prepared,
// as we know, there are has some camera need about 200~400ms to initialization,
// and then we can send/receive from it.
bool runcamDeviceInit(runcamDevice_t *device)
{
    serialPortFunction_e portID = FUNCTION_RCDEVICE;
    serialPortConfig_t *portConfig = findSerialPortConfig(portID);
    if (portConfig != NULL) {
        device->serialPort = openSerialPort(portConfig->identifier, portID, NULL, 115200, MODE_RXTX, SERIAL_NOT_INVERTED);

        if (device->serialPort != NULL) {
            // send RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO to device to retrive
            // device info, e.g protocol version, supported features
            uint8_t respBuf[RCDEVICE_PROTOCOL_MAX_PACKET_SIZE];
            if (runcamDeviceGetDeviceInfo(device, respBuf)) {
                device->info.protocolVersion = respBuf[1];

                uint8_t featureLowBits = respBuf[2];
                uint8_t featureHighBits = respBuf[3];
                device->info.features = (featureHighBits << 8) | featureLowBits;

                return true;
            }

            closeSerialPort(device->serialPort);
        }
    }

    device->serialPort = NULL;
    return false;
}

bool runcamDeviceSimulateCameraButton(runcamDevice_t *device, uint8_t operation)
{
    runcamDeviceSendPacket(device, RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL, &operation, sizeof(operation));
    return true;
}

// every time start to control the OSD menu of camera, must call this method to
// camera
bool runcamDeviceOpen5KeyOSDCableConnection(runcamDevice_t *device)
{
    uint8_t actionID = 0xFF;
    uint8_t code = 0xFF;
    bool r = runcamDeviceSend5KeyOSDCableConnectionEvent(device, RCDEVICE_PROTOCOL_5KEY_CONNECTION_OPEN, &actionID, &code);
    return r && (code == 1) && (actionID == RCDEVICE_PROTOCOL_5KEY_CONNECTION_OPEN);
}

// when the control was stop, must call this method to the camera to disconnect
// with camera.
bool runcamDeviceClose5KeyOSDCableConnection(runcamDevice_t *device, uint8_t *resultCode)
{
    uint8_t actionID = 0xFF;
    uint8_t code = 0xFF;
    bool r = runcamDeviceSend5KeyOSDCableConnectionEvent(device, RCDEVICE_PROTOCOL_5KEY_CONNECTION_CLOSE, &actionID, &code);
    if (resultCode) {
        *resultCode = code;
    }
    return r;
}

// simulate button press event of 5 key osd cable with special button
bool runcamDeviceSimulate5KeyOSDCableButtonPress(runcamDevice_t *device, uint8_t operation)
{
    if (operation == RCDEVICE_PROTOCOL_5KEY_SIMULATION_NONE) {
        return false;
    }

    if (runcamDeviceSendRequestAndWaitingResp(device, RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_PRESS, &operation, sizeof(uint8_t), NULL, NULL)) {
        return true;
    }

    return false;
}

// simulate button release event of 5 key osd cable
bool runcamDeviceSimulate5KeyOSDCableButtonRelease(runcamDevice_t *device)
{
    return runcamDeviceSendRequestAndWaitingResp(device, RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_RELEASE, NULL, 0, NULL, NULL);
}

// fill a region with same char on screen, this is used to DisplayPort feature
// support
void runcamDeviceDispFillRegion(runcamDevice_t *device, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t c)
{
    uint8_t paramsBuf[5];

    // fill parameters buf
    paramsBuf[0] = x;
    paramsBuf[1] = y;
    paramsBuf[2] = width;
    paramsBuf[3] = height;
    paramsBuf[4] = c;

    runcamDeviceSendPacket(device, RCDEVICE_PROTOCOL_COMMAND_DISP_FILL_REGION, paramsBuf, sizeof(paramsBuf));
}

// draw a single char on special position on screen, this is used to DisplayPort
// feature support
void runcamDeviceDispWriteChar(runcamDevice_t *device, uint8_t x, uint8_t y, uint8_t c)
{
    uint8_t paramsBuf[3];

    // fill parameters buf
    paramsBuf[0] = x;
    paramsBuf[1] = y;
    paramsBuf[2] = c;

    runcamDeviceSendPacket(device, RCDEVICE_PROTOCOL_COMMAND_DISP_WRITE_CHAR, paramsBuf, sizeof(paramsBuf));
}

static void runcamDeviceDispWriteString(runcamDevice_t *device, uint8_t x, uint8_t y, const char *text, bool isHorizontal)
{
    uint8_t textLen = strlen(text);
    if (textLen > 60) { // if text len more then 60 chars, cut it to 60
        textLen = 60;
    }

    uint8_t paramsBufLen = 3 + textLen;
    uint8_t paramsBuf[RCDEVICE_PROTOCOL_MAX_DATA_SIZE];

    paramsBuf[0] = paramsBufLen - 1;
    paramsBuf[1] = x;
    paramsBuf[2] = y;
    memcpy(paramsBuf + 3, text, textLen);

    uint8_t command = isHorizontal ? RCDEVICE_PROTOCOL_COMMAND_DISP_WRITE_HORIZONTAL_STRING : RCDEVICE_PROTOCOL_COMMAND_DISP_WRITE_VERTICAL_STRING;
    runcamDeviceSendPacket(device, command, paramsBuf, paramsBufLen);
}

// draw a string on special position on screen, this is used to DisplayPort
// feature support
void runcamDeviceDispWriteHorizontalString(runcamDevice_t *device, uint8_t x, uint8_t y, const char *text)
{
    runcamDeviceDispWriteString(device, x, y, text, true);
}

void runcamDeviceDispWriteVerticalString(runcamDevice_t *device, uint8_t x, uint8_t y, const char *text)
{
    runcamDeviceDispWriteString(device, x, y, text, false);
}

void runcamDeviceDispWriteChars(runcamDevice_t *device, uint8_t *data, uint8_t datalen)
{
    uint8_t adjustedDataLen = datalen;
    if (adjustedDataLen > 60) { // if data len more then 60 chars, cut it to 60
        adjustedDataLen = 60;
    }

    uint8_t paramsBufLen = adjustedDataLen + 1;
    uint8_t paramsBuf[RCDEVICE_PROTOCOL_MAX_DATA_SIZE];

    paramsBuf[0] = adjustedDataLen;
    memcpy(paramsBuf + 1, data, adjustedDataLen);

    runcamDeviceSendPacket(device, RCDEVICE_PROTOCOL_COMMAND_DISP_WRITE_CHARS, paramsBuf, paramsBufLen);
}

static bool runcamDeviceDecodeSettings(sbuf_t *buf, runcamDeviceSetting_t *outSettingList, int maxSettingItemCount)
{
    if (outSettingList == NULL) {
        return false;
    }

    if (maxSettingItemCount > RCDEVICE_PROTOCOL_MAX_MENUITEM_PER_PAGE)
        maxSettingItemCount = RCDEVICE_PROTOCOL_MAX_MENUITEM_PER_PAGE;

    runcamDeviceSettingParseStep_e parseStep = RCDP_SETTING_PARSE_WAITING_ID;
    memset(outSettingList, 0, maxSettingItemCount * sizeof(runcamDeviceSetting_t));
    runcamDeviceSetting_t *settingIterator = outSettingList;
    while (sbufBytesRemaining(buf) > 0) {
        if (settingIterator >= (outSettingList + maxSettingItemCount)) {
            break;
        }

        switch (parseStep) {
        case RCDP_SETTING_PARSE_WAITING_ID: {
            settingIterator->id = sbufReadU8(buf);
            parseStep = RCDP_SETTING_PARSE_WAITING_NAME;
        } 
            break;
        case RCDP_SETTING_PARSE_WAITING_NAME: {
            const char *str = (const char *)sbufConstPtr(buf);
            uint8_t nameLen = strlen(str) + 1;
            memset(settingIterator->name, 0, RCDEVICE_PROTOCOL_MAX_SETTING_NAME_LENGTH);
            strncpy(settingIterator->name, str, RCDEVICE_PROTOCOL_MAX_SETTING_NAME_LENGTH);
            sbufAdvance(buf, nameLen);
            
            parseStep = RCDP_SETTING_PARSE_WAITING_VALUE;
        } 
            break;
        case RCDP_SETTING_PARSE_WAITING_VALUE: {
            const char *str = (const char *)sbufConstPtr(buf);
            uint8_t valueLen = strlen(str) + 1;
            memset(settingIterator->value, 0, RCDEVICE_PROTOCOL_MAX_SETTING_VALUE_LENGTH);
            strcpy(settingIterator->value, str);
            sbufAdvance(buf, valueLen);
            parseStep = RCDP_SETTING_PARSE_WAITING_ID;

            settingIterator++;
        } 
            break;
        }
    }

    if (RCDP_SETTING_PARSE_WAITING_ID != parseStep) {
        return false;
    }

    return true;
}

static bool runcamDeviceGetResponseWithMultipleChunk(runcamDevice_t *device, uint8_t command, uint8_t settingID, uint8_t *responseData, uint16_t *responseDatalen)
{
    if (responseData == NULL || responseDatalen == NULL) {
        return false;
    }

    // fill parameters buf
    uint8_t paramsBuf[2];
    uint8_t chunkIndex = 0;
    paramsBuf[0] = settingID; // parent setting id
    paramsBuf[1] = chunkIndex; // chunk index

    uint8_t outputBufLen = RCDEVICE_PROTOCOL_MAX_PACKET_SIZE;
    uint8_t outputBuf[RCDEVICE_PROTOCOL_MAX_PACKET_SIZE];
    bool result = runcamDeviceSendRequestAndWaitingResp(device, command, paramsBuf, sizeof(paramsBuf), outputBuf, &outputBufLen);
    if (!result) {
        return false;
    }

    uint8_t remainingChunk = outputBuf[1];
    // Every response chunk count must less than or equal to RCDEVICE_PROTOCOL_MAX_CHUNK_PER_RESPONSE
    if (remainingChunk >= RCDEVICE_PROTOCOL_MAX_CHUNK_PER_RESPONSE) {
        return false;
    }

    // save setting data to sbuf_t object
    const uint16_t maxDataLen = RCDEVICE_PROTOCOL_MAX_CHUNK_PER_RESPONSE * RCDEVICE_PROTOCOL_MAX_DATA_SIZE;
    // uint8_t data[maxDataLen];
    sbuf_t dataBuf;
    dataBuf.ptr = responseData;
    dataBuf.end = responseData + maxDataLen;
    sbufWriteData(&dataBuf, outputBuf + 3, outputBufLen - 4);

    // get the remaining chunks
    while (remainingChunk > 0) {
        paramsBuf[1] = ++chunkIndex; // chunk index

        outputBufLen = RCDEVICE_PROTOCOL_MAX_PACKET_SIZE;
        result = runcamDeviceSendRequestAndWaitingResp(device, command, paramsBuf, sizeof(paramsBuf), outputBuf, &outputBufLen);

        if (!result) {
            return false;
        }

        // append the trailing chunk to the sbuf_t object,
        // but only append the actually setting data
        sbufWriteData(&dataBuf, outputBuf + 3, outputBufLen - 4);

        remainingChunk--;
    }

    sbufSwitchToReader(&dataBuf, responseData);
    *responseDatalen = sbufBytesRemaining(&dataBuf);

    return true;
}

// get settings with parent setting id, the type of parent setting must be a
// FOLDER after this function called, the settings will fill into outSettingList
// argument
bool runcamDeviceGetSettings(runcamDevice_t *device, uint8_t parentSettingID, runcamDeviceSetting_t *outSettingList, int maxSettingItemCount)
{
    if (outSettingList == NULL) {
        return false;
    }

    uint16_t responseDataLength = 0;
    uint8_t data[RCDEVICE_PROTOCOL_MAX_CHUNK_PER_RESPONSE * RCDEVICE_PROTOCOL_MAX_DATA_SIZE];
    if (!runcamDeviceGetResponseWithMultipleChunk(device, RCDEVICE_PROTOCOL_COMMAND_GET_SETTINGS, parentSettingID, data, &responseDataLength)) {
        return false;
    }

    sbuf_t dataBuf;
    dataBuf.ptr = data;
    dataBuf.end = data + responseDataLength;

    // parse the settings data and convert them into a runcamDeviceSetting_t list
    if (!runcamDeviceDecodeSettings(&dataBuf, outSettingList, maxSettingItemCount)) {
        return false;
    }
    
    return true;
}

static bool runcamDeviceDecodeSettingDetail(sbuf_t *buf, runcamDeviceSettingDetail_t *outSettingDetail)
{
    if (outSettingDetail == NULL || sbufBytesRemaining(buf) == 0) {
        return false;
    }

    rcdeviceSettingType_e settingType = sbufReadU8(buf);
    outSettingDetail->type = settingType;
    switch (settingType) {
    case RCDEVICE_PROTOCOL_SETTINGTYPE_UINT8:
    case RCDEVICE_PROTOCOL_SETTINGTYPE_INT8:
        outSettingDetail->value = sbufReadU8(buf);
        outSettingDetail->minValue = sbufReadU8(buf);
        outSettingDetail->maxValue = sbufReadU8(buf);
        outSettingDetail->stepSize = sbufReadU8(buf);
        break;
    case RCDEVICE_PROTOCOL_SETTINGTYPE_UINT16:
    case RCDEVICE_PROTOCOL_SETTINGTYPE_INT16: 
        outSettingDetail->value = sbufReadU16(buf);
        outSettingDetail->minValue = sbufReadU16(buf);
        outSettingDetail->maxValue = sbufReadU16(buf);
        outSettingDetail->stepSize = sbufReadU8(buf);
        break;
    case RCDEVICE_PROTOCOL_SETTINGTYPE_FLOAT: 
        outSettingDetail->value = sbufReadU32(buf);
        outSettingDetail->minValue = sbufReadU32(buf);
        outSettingDetail->maxValue = sbufReadU32(buf);
        outSettingDetail->decimalPoint = sbufReadU8(buf);
        outSettingDetail->stepSize = sbufReadU32(buf);
        break;
    case RCDEVICE_PROTOCOL_SETTINGTYPE_TEXT_SELECTION: {
        outSettingDetail->value = sbufReadU8(buf);

        const char *tmp = (const char *)sbufConstPtr(buf);
        const uint16_t maxLen = RCDEVICE_PROTOCOL_MAX_DATA_SIZE * RCDEVICE_PROTOCOL_MAX_TEXT_SELECTIONS;
        char textSels[maxLen];
        memset(textSels, 0, maxLen);
        strncpy(textSels, tmp, maxLen);
        char delims[] = ";";
        char *result = strtok(textSels, delims);
        int i = 0;
        runcamDeviceSettingTextSelection_t *iterator = outSettingDetail->textSelections;
        while (result != NULL) {
            if (i >= RCDEVICE_PROTOCOL_MAX_TEXT_SELECTIONS) {
                break;
            }

            memset(iterator->text, 0, RCDEVICE_PROTOCOL_MAX_SETTING_VALUE_LENGTH);
            strncpy(iterator->text, result, RCDEVICE_PROTOCOL_MAX_SETTING_VALUE_LENGTH);
            iterator++;
            result = strtok(NULL, delims);
            i++;
        }
    } 
        break;
    case RCDEVICE_PROTOCOL_SETTINGTYPE_STRING: {
        const char *tmp = (const char *)sbufConstPtr(buf);
        strncpy(outSettingDetail->stringValue, tmp, RCDEVICE_PROTOCOL_MAX_STRING_LENGTH);
        sbufAdvance(buf, strlen(tmp) + 1);

        outSettingDetail->maxStringSize = sbufReadU8(buf);
    } 
        break;
    case RCDEVICE_PROTOCOL_SETTINGTYPE_FOLDER:
        break;
    case RCDEVICE_PROTOCOL_SETTINGTYPE_INFO: {
        const char *tmp = (const char *)sbufConstPtr(buf);
        strncpy(outSettingDetail->stringValue, tmp, RCDEVICE_PROTOCOL_MAX_STRING_LENGTH);
        sbufAdvance(buf, strlen(outSettingDetail->stringValue) + 1);
    }
        break;
    case RCDEVICE_PROTOCOL_SETTINGTYPE_UNKNOWN:
        break;
    }

    return true;
}

// get the setting details with setting id
// after this function called, the setting detail will fill into
// outSettingDetail argument
bool runcamDeviceGetSettingDetail(runcamDevice_t *device, uint8_t settingID, runcamDeviceSettingDetail_t *outSettingDetail)
{
    if (outSettingDetail == NULL)
        return false;

    uint16_t responseDataLength = 0;
    uint8_t data[RCDEVICE_PROTOCOL_MAX_CHUNK_PER_RESPONSE * RCDEVICE_PROTOCOL_MAX_DATA_SIZE];
    if (!runcamDeviceGetResponseWithMultipleChunk(device, RCDEVICE_PROTOCOL_COMMAND_READ_SETTING_DETAIL, settingID, data, &responseDataLength)) {
        return false;
    }

    sbuf_t dataBuf;
    dataBuf.ptr = data;
    dataBuf.end = data + responseDataLength;

    // parse the settings data and convert them into a runcamDeviceSettingDetail_t
    if (!runcamDeviceDecodeSettingDetail(&dataBuf, outSettingDetail)) {
        return false;
    }

    return true;
}

// write new value with to the setting
bool runcamDeviceWriteSetting(runcamDevice_t *device, uint8_t settingID, uint8_t *paramData, uint8_t paramDataLen, runcamDeviceWriteSettingResponse_t *response)
{
    if (response == NULL || paramDataLen > (RCDEVICE_PROTOCOL_MAX_DATA_SIZE - 1)) {
        return false;
    }

    memset(response, 0, sizeof(runcamDeviceWriteSettingResponse_t));
    response->resultCode = 1; // initialize the result code to failed 

    uint8_t paramsBufLen = sizeof(uint8_t) + paramDataLen;
    uint8_t paramsBuf[RCDEVICE_PROTOCOL_MAX_DATA_SIZE];
    paramsBuf[0] = settingID;
    memcpy(paramsBuf + 1, paramData, paramDataLen);

    uint8_t outputBufLen = RCDEVICE_PROTOCOL_MAX_PACKET_SIZE;
    uint8_t outputBuf[RCDEVICE_PROTOCOL_MAX_PACKET_SIZE];
    bool result = runcamDeviceSendRequestAndWaitingResp(device, RCDEVICE_PROTOCOL_COMMAND_WRITE_SETTING, paramsBuf, paramsBufLen, outputBuf, &outputBufLen);
    if (!result) {
        return false;
    }

    response->resultCode = outputBuf[1];
    response->needUpdateMenuItems = outputBuf[2];

    return true;
}

#endif
