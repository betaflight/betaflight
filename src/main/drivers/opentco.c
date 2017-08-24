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

#if defined(USE_OPENTCO)

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
    return opentcoReadRegister(device, 0, &tmp);
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

static bool opentcoDecodeResponse(opentcoDevice_t *device, uint8_t requested_reg, uint16_t *reply)
{
    // header has been checked beforehand, test the remaining 5 bytes:
    // ... [DEVICE:4|CMD:4] [REGISTER:8] [VALUE_LO:8] [VALUE_HI:8] [CRC:8]

    // prepare crc calc
    uint8_t crc = crc8_dvb_s2(0, OPENTCO_PROTOCOL_HEADER);

    // fetch data (serial buffer already contains enough bytes)
    uint8_t data[5];
    for(int i = 0; i < 5; i++) {
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

    // return value
    *reply = (data[3] << 8) | data[2];

    return true;
}

bool opentcoReadRegister(opentcoDevice_t *device, uint8_t reg, uint16_t *val)
{
    uint32_t max_retries = 3;

    while (max_retries--) {
        // send read request
        opentcoWriteRegister(device, reg | OPENTCO_REGISTER_ACCESS_MODE_READ, 0);

        // wait 100ms for reply
        timeMs_t timeout = millis() + 100;

        bool header_received = false;
        while (millis() < timeout) {
            // register request replies will contain 6 bytes:
            // [HEADER:8] [DEVICE:4|CMD:4] [REGISTER:8] [VALUE_LO:8] [VALUE_HI:8] [CRC:8]
            if (!header_received) {
                // read serial bytes until we find a header:
                if (serialRxBytesWaiting(device->serialPort) > 0) {
                    uint8_t rx = serialRead(device->serialPort);
                    if (rx == OPENTCO_PROTOCOL_HEADER) {
                        header_received = true;
                    }
                }
            } else {
                // header found, now wait for the remaining bytes to arrive
                if (serialRxBytesWaiting(device->serialPort) >= 5) {
                    // try to decode this packet
                    if (!opentcoDecodeResponse(device, reg, val)) {
                        // received broken / bad response
                        break;
                    }

                    // received valid data
                    return true;
                }
            }
        }
    }

    // failed n times
    return false;
}

bool opentcoWriteRegister(opentcoDevice_t *device, uint8_t reg, uint16_t val)
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

