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
 * Author: 4712
*/

#define USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#define USE_SERIAL_4WAY_SK_BOOTLOADER

typedef enum {
    imC2      = 0,
    imSIL_BLB = 1,
    imATM_BLB = 2,
    imSK      = 3,
} esc4wayInterfaceMode_e;

typedef enum {
// Test Interface still present
    cmd_InterfaceTestAlive  = 0x30,  // '0' alive
// RETURN: ACK

// get Protocol Version Number 01..255
    cmd_ProtocolGetVersion  = 0x31,  // '1' version
// RETURN: uint8_t VersionNumber + ACK

// get Version String
    cmd_InterfaceGetName    = 0x32,  // '2' name
// RETURN: String + ACK

//get Version Number 01..255
    cmd_InterfaceGetVersion = 0x33,  // '3' version
// RETURN: uint16_t VersionNumber + ACK

// Exit / Restart Interface - can be used to switch to Box Mode
    cmd_InterfaceExit       = 0x34,  // '4' exit
// RETURN: ACK

// Reset the Device connected to the Interface
    cmd_DeviceReset         = 0x35,  // '5' reset
// PARAM: uint8_t escId
// RETURN: ACK

// Get the Device ID connected
//     cmd_DeviceGetID      = 0x36,  // '6' device id; removed since 06/106
// RETURN: uint8_t DeviceID + ACK

// Initialize Flash Access for Device connected
// Autodetects interface protocol; retruns device signature and protocol
    cmd_DeviceInitFlash     = 0x37,  // '7' init flash access
// PARAM: uint8_t escId
// RETURN: uint8_t deviceInfo[4] + ACK

// Erase the whole Device Memory of connected Device
    cmd_DeviceEraseAll      = 0x38,  // '8' erase all
// RETURN: ACK

// Erase one Page of Device Memory of connected Device
    cmd_DevicePageErase     = 0x39,  // '9' page erase
// PARAM: uint8_t PageNumber (512B pages)
// RETURN: APageNumber ACK

// Read to Buffer from FLASH Memory of connected Device
// Buffer Len is Max 256 Bytes, 0 means 256 Bytes
    cmd_DeviceRead          = 0x3A,  // ':' read Device
// PARAM: [ADRESS] uint8_t BuffLen
// RETURN: [ADRESS, len] Buffer[0..256] ACK

// Write Buffer to FLASH Memory of connected Device
// Buffer Len is Max 256 Bytes, 0 means 256 Bytes
    cmd_DeviceWrite         = 0x3B,  // ';' write
// PARAM: [ADRESS + BuffLen] Buffer[1..256]
// RETURN: ACK

// Set C2CK low infinite - permanent Reset state (unimplemented)
    cmd_DeviceC2CK_LOW      = 0x3C,  // '<'
// RETURN: ACK

// Read to Buffer from EEPROM Memory of connected Device
// Buffer Len is Max 256 Bytes, 0 means 256 Bytes
    cmd_DeviceReadEEprom    = 0x3D,  // '=' read Device
// PARAM: [ADRESS] uint8_t BuffLen
// RETURN: [ADRESS + BuffLen] + Buffer[1..256] ACK

// Write Buffer to EEPROM Memory of connected Device
// Buffer Len is Max 256 Bytes, 0 means 256 Bytes
    cmd_DeviceWriteEEprom   = 0x3E,  // '>' write
// PARAM: [ADRESS + BuffLen] Buffer[1..256]
// RETURN: ACK

// Set Interface Mode
    cmd_InterfaceSetMode   = 0x3F,   // '?'
// PARAM: uint8_t Mode (interfaceMode_e)
// RETURN: ACK
} esc4wayCmd_e;

// responses
typedef enum {
    esc4wayAck_OK =                 0x00,
//  esc4wayAck_I_UNKNOWN_ERROR =    0x01,
    esc4wayAck_I_INVALID_CMD =      0x02,
    esc4wayAck_I_INVALID_CRC =      0x03,
    esc4wayAck_I_VERIFY_ERROR =     0x04,
//  esc4wayAck_D_INVALID_COMMAND =  0x05,
//  esc4wayAck_D_COMMAND_FAILED =   0x06,
//  esc4wayAck_D_UNKNOWN_ERROR =    0x07,
    esc4wayAck_I_INVALID_CHANNEL =  0x08,
    esc4wayAck_I_INVALID_PARAM =    0x09,
    esc4wayAck_D_GENERAL_ERROR =    0x0f,
} esc4wayAck_e;

typedef struct escDeviceInfo_s {
    uint16_t signature;        // lower 16 bit of signature
    uint8_t  signature2;       // top 8 bit of signature for SK / BootMsg last char from BL
    uint8_t interfaceMode;
} escDeviceInfo_t;

bool esc4wayExitRequested;     // flag that exit was requested. Set by esc4wayProcessCmd, used internally by esc4wayProcess

int esc4wayInit(void);
void esc4wayStart(void);
void esc4wayRelease(void);
void esc4wayProcess(serialPort_t *serial);
esc4wayAck_e esc4wayProcessCmd(esc4wayCmd_e command, uint16_t addr, uint8_t *data, int inLen, int *outLen);
