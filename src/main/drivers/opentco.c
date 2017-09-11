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


#include "common/maths.h"
#include "common/streambuf.h"
#include "drivers/time.h"
#include "common/crc.h"
#include "opentco.h"

#include <stdbool.h>
#include <string.h>

#if 1 //defined(USE_OPENTCO)

static bool opentcoDetectDevice(opentcoDevice_t *device);
static void opentcoRegisterDevice(opentcoDevice_t *device);
static opentcoDevice_t *firstDevice = NULL;

// openTCO allows multiple (virtual) devices to share a single cpu and uart
// - or -
// use an indivual uart for every device
//
// for now only ONE device of each class (camera, vtx, osd) is supported
//
// this function scans all serialports configured as FUNCTION_OPENTCO
// for a given deviceid and returns the FIRST successful hit
bool opentcoInit(opentcoDevice_t *device)
{
    // first: iterate over the open opentcoDevice list in order
    // to reuse an already opened port:
    // find attachment point
    opentcoDevice_t *currentDevice = firstDevice;
    while(currentDevice != NULL) {
        // temporarily set serial port
        device->serialPort = currentDevice->serialPort;

        // try to detect this device on bus
        if (opentcoDetectDevice(device)) {
            // found device on this port, store this device
            opentcoRegisterDevice(device);

            // and done
            return true;
        }
        // next device
        currentDevice = currentDevice->next;
    }

    // not found in the current device list, scan all serial ports
    // (this will skip already opened ports, this is why we need
    // to scan the list above

    // start with the first one:
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_OPENTCO);

    while (portConfig != NULL) {
        // extract baudrate
        uint32_t baudrate = baudRates[portConfig->blackbox_baudrateIndex];

        // try tp open the serial port
        device->serialPort = openSerialPort(portConfig->identifier, FUNCTION_OPENTCO, NULL, baudrate, MODE_RXTX, SERIAL_NOT_INVERTED);

        if (device->serialPort != NULL) {
            // try to detect the given device:
            if (opentcoDetectDevice(device)) {
                opentcoRegisterDevice(device);
                return true;
            }
            // device not found, close port
            closeSerialPort(device->serialPort);
        }

        // find next portConfig
        portConfig = findNextSerialPortConfig(FUNCTION_OPENTCO);
    }

    // clear serialport
    device->serialPort = NULL;
    return false;
}

// device detected on bus?
static bool opentcoDetectDevice(opentcoDevice_t *device)
{
    uint16_t tmp;
    return opentcoReadRegisterUint16(device, 0, &tmp);
}

// keep a linked list of devices
static void opentcoRegisterDevice(opentcoDevice_t *device)
{
    // new device, should not have next pointer
    device->next = NULL;

    if (firstDevice == NULL) {
        // no active device in chain, store this
        firstDevice = device;
        return;
    }

    // find attachment point
    opentcoDevice_t *currentDevice = firstDevice;
    while(currentDevice->next != NULL) {
        currentDevice = currentDevice->next;
    }

    // add device to list
    currentDevice->next = device;
}

#define OPENTCO_RESPONSE_TYPE_UINT16 2
#define OPENTCO_RESPONSE_TYPE_TEXTSELECTION 9

static bool opentcoDecodeResponseUint16(opentcoDevice_t *device, uint8_t requested_reg, uint16_t *reply)
{
    // header has been checked beforehand, test the remaining n-1 bytes:
    // ... [DEVICE:4|CMD:4] [REGISTER:8] [LENGTH:8] [TYPE:8] [VALUE_LO:8] [VALUE_HI:8] [CRC:8]

    // prepare crc calc
    uint8_t crc = crc8_dvb_s2(0, OPENTCO_PROTOCOL_HEADER);

    // fetch data (serial buffer already contains enough bytes)
    uint8_t data[8];
    for(int i = 0; i < 8; i++) {
        uint8_t rx = serialRead(device->serialPort);
        data[i] = rx;
        crc = crc8_dvb_s2(crc, rx);
    }

    // check crc
    if (crc != 0) return false;

    // check device and command
    uint8_t valid_devcmd = ((OPENTCO_DEVICE_RESPONSE | device->id) << 4) | OPENTCO_OSD_COMMAND_REGISTER_ACCESS;
    if (data[0] != valid_devcmd) return false;

    // response to our request? reply should contain register id WITHOUT read bit set
    uint8_t valid_reg = requested_reg & ~OPENTCO_REGISTER_ACCESS_MODE_READ;
    if (data[1] != valid_reg) return false;

    // valid response length?
    if (data[2] != 3) return false;

    // valid response type?
    if (data[3] != OPENTCO_RESPONSE_TYPE_UINT16) return false;

    // return value
    *reply = (data[4] << 8) | data[5];

    return true;
}

static void opentcoReadRegisterProcessPayloadUint16(uint8_t *data, uint8_t length, uint16_t *target) {
    UNUSED(length);
    *target = (data[0] << 8) | data[1];
}

static void opentcoReadRegisterProcessPayloadTextselection(uint8_t *data, uint8_t length, uint8_t *target) {
    // copy data to target string
    strncpy((char *)target, (const char*)data, length);
}

void opentcoReadRegisterProcessPayload(uint8_t type, uint8_t *data, uint8_t length, void *target) {
    // process response
    switch (type)
    {
    default:
        // invalid -> ignore
        break;

    case OPENTCO_RESPONSE_TYPE_UINT16:
        opentcoReadRegisterProcessPayloadUint16(data, length, target);
        break;

    case(OPENTCO_RESPONSE_TYPE_TEXTSELECTION):
        opentcoReadRegisterProcessPayloadTextselection(data, length, target);
        break;
    }
}

static bool opentcoReadRegister(opentcoDevice_t *device, uint8_t reg, void *target)
{
    uint32_t max_retries = 3;

    while (max_retries--) {
        // flush rx buffer
        while (serialRxBytesWaiting(device->serialPort)) {
            serialRead(device->serialPort);
        }

        // send read request
        opentcoWriteRegisterUint16(device, reg | OPENTCO_REGISTER_ACCESS_MODE_READ, 0);

        // wait 100ms for reply
        timeMs_t timeout = millis() + 100;

        // decode packet head: header + device + register + length
        uint8_t valid_devcmd = ((OPENTCO_DEVICE_RESPONSE | device->id) << 4) | OPENTCO_OSD_COMMAND_REGISTER_ACCESS;
        uint8_t valid_reg = reg & ~OPENTCO_REGISTER_ACCESS_MODE_READ;

        uint32_t decoder_state = 0;
        uint8_t data_length = 0;
        uint8_t data_pos = 0;
        uint8_t data_type = 0;
        uint8_t data[OPENTCO_MAX_DATA_LENGTH];
        uint8_t crc = crc8_dvb_s2(0, OPENTCO_PROTOCOL_HEADER);

        while (millis() < timeout) {
            if (serialRxBytesWaiting(device->serialPort) > 0) {
                uint8_t rx = serialRead(device->serialPort);
                crc = crc8_dvb_s2(crc, rx);

                switch (decoder_state)
                {
                default:
                case 0:
                    // expect HEADER
                    if (rx == OPENTCO_PROTOCOL_HEADER) {
                        decoder_state = 1;
                    }
                    break;

                case 1:
                    // expect device id + matching cmd
                    if (rx == valid_devcmd) {
                        // got valid device and cmd
                        decoder_state = 2;
                        // reinit crc
                        crc = crc8_dvb_s2(0, OPENTCO_PROTOCOL_HEADER);
                    } else if (rx == OPENTCO_PROTOCOL_HEADER) {
                        // this looks like a header, retry decoding on next byte
                    } else {
                        // not a header, restart next time
                        decoder_state = 0;
                    }
                    break;

                case 2:
                    // expect register
                    if (rx == valid_reg) {
                        decoder_state = 3;
                    } else {
                        decoder_state = 0;
                    }
                    break;

                case 3:
                    // decode length
                    if (rx <= OPENTCO_MAX_DATA_LENGTH) {
                        // valid length!
                        decoder_state = 4;
                        data_length = rx;
                        data_pos = 0;
                    } else {
                        decoder_state = 0;
                    }
                    break;

                case 4:
                    // decode data type
                    data_type = rx;
                    decoder_state = 5;
                    break;

                case 5:
                    // fetch data
                    data[data_pos] = rx;
                    if (data_pos != data_length) {
                        decoder_state = 6;
                    }
                    data_pos++;
                    break;

                case 6:
                    // check crc
                    if (crc == 0) {
                        // fine! safe to decode payload
                        opentcoReadRegisterProcessPayload(data_type, data, data_length, target);
                    }
                    decoder_state = 0;
                }
            }
        }
    }

    // failed n times
    return false;
}

// read a register and return a 16 bit value register
bool opentcoReadRegisterUint16(opentcoDevice_t *device, uint8_t reg, uint16_t *val) {
    return opentcoReadRegister(device, reg, (void*)val);
}


// read a register and return a string
bool opentcoReadRegisterString(opentcoDevice_t *device, uint8_t reg, char *val)
{
    return opentcoReadRegister(device, reg, (void*)val);
}


/*
static bool opentcoDecodeResponseStringArray(opentcoDevice_t *device, uint8_t requested_reg, uint8_t *index, uint8_t *max, char *val)
{
    // header has been checked beforehand, test the remaining 6 + OPENTCO_MAX_STRING_LENGTH -1 bytes:
    // ... [DEVICE:4|CMD:4] [REGISTER:8] [MAX_INDEX:4|INDEX:4] [STRING:OPENTCO_MAX_STRING_LENGTH * 8] [CRC:8]

    // prepare crc calc
    uint8_t crc = crc8_dvb_s2(0, OPENTCO_PROTOCOL_HEADER);

    // fetch data (serial buffer already contains enough bytes)
    uint8_t data[5 + OPENTCO_MAX_STRING_LENGTH - 1];
    for(int i = 0; i < 20; i++) {
        uint8_t rx = serialRead(device->serialPort);
        data[i] = rx;
        crc = crc8_dvb_s2(crc, rx);
    }

    // check crc
    if (crc != 0) return false;

    // check device and command
    uint8_t valid_devcmd = ((OPENTCO_DEVICE_RESPONSE | device->id) << 4) | OPENTCO_OSD_COMMAND_REGISTER_ACCESS;
    if (data[0] != valid_devcmd) return false;

    // response to our request? reply should contain register id WITHOUT read bit set
    uint8_t valid_reg = requested_reg & ~OPENTCO_REGISTER_ACCESS_MODE_READ;
    if (data[1] != valid_reg) return false;

    // decode index and max index:
    *max = (data[2] >> 4) & 0x0F;
    *index = data[2] & 0x0F;

    // copy string to return value:
    uint32_t todo = OPENTCO_MAX_STRING_LENGTH - 1;
    char *sptr = (char *)&data[3];
    while(todo--) {
        *val++ = *sptr++;
    }
    // loop iterates SIZE-1, make sure to zero terminate string
    *val = 0;

    return true;
}*/

bool opentcoWriteRegisterUint16(opentcoDevice_t *device, uint8_t reg, uint16_t val)
{
    // start frame
    opentcoInitializeFrame(device, OPENTCO_OSD_COMMAND_REGISTER_ACCESS);

    // add register
    sbufWriteU8(device->sbuf, reg);

    // add value
    sbufWriteU16(device->sbuf, val);

    // send
    opentcoSendFrame(device);

    //FIXME: check if actually written (read response)
    return true;
}

void opentcoInitializeFrame(opentcoDevice_t *device, uint8_t command)
{
    // point to the buffer
    device->sbuf = &device->streamBuffer;

    // prepare pointer
    device->sbuf->ptr = device->buffer;
    device->sbuf->end = ARRAYEND(device->buffer);

    // add frame header
    sbufWriteU8(device->sbuf, OPENTCO_PROTOCOL_HEADER);

    // add device & command
    sbufWriteU8(device->sbuf, ((device->id & 0x0F)<<4) | (command & 0x0F));
}


void opentcoSendFrame(opentcoDevice_t *device)
{
    // is this device open?
    if (!device->serialPort) {
        return;
    }

    // add crc over (all) data
    crc8_dvb_s2_sbuf_append(device->sbuf, device->buffer);

    // switch to reader
    sbufSwitchToReader(device->sbuf, device->buffer);

    // send data if possible
    serialPort_t *serialPort = device->serialPort;
    if (!serialPort->locked) {
        serialPort->locked = true;
        serialWriteBuf(device->serialPort, sbufPtr(device->sbuf), sbufBytesRemaining(device->sbuf));
        serialPort->locked = false;
    }
}

#endif  // defined(USE_OPENTCO)

