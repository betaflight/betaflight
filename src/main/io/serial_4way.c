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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include <platform.h>

#ifdef  USE_SERIAL_4WAY_BLHELI_INTERFACE

#include "config/parameter_group.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"
#include "drivers/system.h"
#include "flight/mixer.h"
#include "io/beeper.h"
#include "io/serial_msp.h"
#include "io/serial_msp.h"
#include "io/serial_4way.h"

#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
# include "io/serial_4way_avrootloader.h"
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
# include "io/serial_4way_stk500v2.h"
#endif

#define USE_TXRX_LED

#if defined(USE_TXRX_LED) && defined(LED0)
# define RX_LED_OFF   LED0_OFF
# define RX_LED_ON    LED0_ON
# ifdef  LED1
#  define TX_LED_OFF  LED1_OFF
#  define TX_LED_ON   LED1_ON
# else
#  define TX_LED_OFF  LED0_OFF
#  define TX_LED_ON   LED0_ON
# endif
#else
# define RX_LED_OFF   do {} while(0)
# define RX_LED_ON    do {} while(0)
# define TX_LED_OFF   do {} while(0)
# define TX_LED_ON    do {} while(0)
#endif

#define SERIAL_4WAY_INTERFACE_NAME_STR "m4wFCIntf"
// *** change to adapt Revision
#define SERIAL_4WAY_VER_MAIN  14
#define SERIAL_4WAY_VER_SUB_1 ((uint8_t)4)
#define SERIAL_4WAY_VER_SUB_2 ((uint8_t)4)

#define SERIAL_4WAY_PROTOCOL_VER 106
// *** end

#if (SERIAL_4WAY_VER_MAIN > 24)
# error "beware of SERIAL_4WAY_VER_SUB_1 is uint8_t"
#endif

#define SERIAL_4WAY_VERSION ((uint16_t)((SERIAL_4WAY_VER_MAIN * 1000) + (SERIAL_4WAY_VER_SUB_1 * 100) + SERIAL_4WAY_VER_SUB_2))

#define SERIAL_4WAY_VERSION_HI (uint8_t) (SERIAL_4WAY_VERSION / 100)
#define SERIAL_4WAY_VERSION_LO (uint8_t) (SERIAL_4WAY_VERSION % 100)

static uint8_t escCount;
uint8_t escSelected;

escHardware_t escHardware[MAX_PWM_MOTORS];

static deviceInfo_t deviceInfo;

bool isMcuConnected(void)
{
    return deviceInfo.signature != 0;
}

static void setDisconnected(void) {
    deviceInfo.signature = 0;
}

bool isEscHi(uint8_t selEsc)
{
    return (digitalIn(escHardware[selEsc].gpio, escHardware[selEsc].pin) != Bit_RESET);
}

bool isEscLo(uint8_t selEsc)
{
    return (digitalIn(escHardware[selEsc].gpio, escHardware[selEsc].pin) == Bit_RESET);
}

void setEscHi(uint8_t selEsc)
{
    digitalHi(escHardware[selEsc].gpio, escHardware[selEsc].pin);
}

void setEscLo(uint8_t selEsc)
{
    digitalLo(escHardware[selEsc].gpio, escHardware[selEsc].pin);
}

void setEscInput(uint8_t selEsc)
{
    gpioInit(escHardware[selEsc].gpio, &escHardware[selEsc].gpio_config_INPUT);
}

void setEscOutput(uint8_t selEsc)
{
    gpioInit(escHardware[selEsc].gpio, &escHardware[selEsc].gpio_config_OUTPUT);
}

static uint32_t GetPinPos(uint32_t pin)
{
    for (int pinPos = 0; pinPos < 16; pinPos++) {
        uint32_t pinMask = (0x1 << pinPos);
        if (pin & pinMask) {
            return pinPos;
        }
    }
    return 0;
}

int Initialize4WayInterface(void)
{
    // StopPwmAllMotors();
    pwmDisableMotors();
    memset(&escHardware, 0, sizeof(escHardware));
    pwmIOConfiguration_t *pwmIOConfiguration = pwmGetOutputConfiguration();
    int escIdx = 0;
    for (int i = 0; i < pwmIOConfiguration->ioCount; i++) {
        if ((pwmIOConfiguration->ioConfigurations[i].flags & PWM_PF_MOTOR) == PWM_PF_MOTOR) {
            if(motor[pwmIOConfiguration->ioConfigurations[i].index] > 0) {
                escHardware[escIdx].gpio = pwmIOConfiguration->ioConfigurations[i].timerHardware->gpio;
                escHardware[escIdx].pin = pwmIOConfiguration->ioConfigurations[i].timerHardware->pin;
                escHardware[escIdx].pinpos = GetPinPos(escHardware[escIdx].pin);
                escHardware[escIdx].gpio_config_INPUT.pin = escHardware[escIdx].pin;
                escHardware[escIdx].gpio_config_INPUT.speed = Speed_2MHz; // see pwmOutConfig()
                escHardware[escIdx].gpio_config_INPUT.mode = Mode_IPU;
                escHardware[escIdx].gpio_config_OUTPUT = escHardware[escIdx].gpio_config_INPUT;
                escHardware[escIdx].gpio_config_OUTPUT.mode = Mode_Out_PP;
                setEscInput(escIdx);
                setEscHi(escIdx);
                escIdx++;
            }
        }
    }
    escCount = escIdx;
    return escCount;
}

void DeInitialize4WayInterface(void)
{
    for(int i = 0; i < escCount; i++) {
        escHardware[i].gpio_config_OUTPUT.mode = Mode_AF_PP; // see pwmOutConfig() // TODO
        setEscOutput(i);
        setEscLo(i);
    }
    escCount = 0;
    pwmEnableMotors();
}

// Interface related only
// establish and test connection to the Interface

// Send Structure
// ESC CMD ADDR_H ADDR_L PARAM_LEN [PARAM (if len > 0)] CRC16_Hi CRC16_Lo
// Return
// ESC CMD ADDR_H ADDR_L PARAM_LEN [PARAM (if len > 0)] + ACK (uint8_t OK or ERR) + CRC16_Hi CRC16_Lo

typedef enum {
    cmd_Remote_Escape       = 0x2E, // '.'
    cmd_Local_Escape        = 0x2F, // '/'
} esc_4way_e;

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
//     cmd_DeviceGetID      = 0x36,  // '6' device id removed since 06/106
// RETURN: uint8_t DeviceID + ACK

// Initialize Flash Access for Device connected
    cmd_DeviceInitFlash     = 0x37,  // '7' init flash access
// PARAM: uint8_t escId
// RETURN: uint8_t deviceInfo[4] + ACK

// Erase the whole Device Memory of connected Device
    cmd_DeviceEraseAll      = 0x38,  // '8' erase all
// RETURN: ACK

// Erase one Page of Device Memory of connected Device
    cmd_DevicePageErase     = 0x39,  // '9' page erase
// PARAM: uint8_t APageNumber (512B pages)
// RETURN: APageNumber ACK

// Read to Buffer from Device Memory of connected Device
// Buffer Len is Max 256 Bytes, 0 means 256 Bytes
    cmd_DeviceRead          = 0x3A,  // ':' read Device
// PARAM: [ADRESS] uint8_t BuffLen
// RETURN: [ADRESS, len] Buffer[0..256] ACK

// Write to Buffer for Device Memory of connected Device
// Buffer Len is Max 256 Bytes, 0 means 256 Bytes
    cmd_DeviceWrite         = 0x3B,  // ';' write
// PARAM: [ADRESS + BuffLen] Buffer[1..256]
// RETURN: ACK

// Set C2CK low infinite ) permanent Reset state (unimplemented)
    cmd_DeviceC2CK_LOW      = 0x3C,  // '<'
// RETURN: ACK

// Read to Buffer from Device Memory of connected Device
// Buffer Len is Max 256 Bytes, 0 means 256 Bytes
    cmd_DeviceReadEEprom    = 0x3D,  // '=' read Device
// PARAM: [ADRESS] uint8_t BuffLen
// RETURN: [ADRESS + BuffLen] + Buffer[1..256] ACK

// Write to Buffer for Device Memory of connected Device
// Buffer Len is Max 256 Bytes, 0 means 256 Bytes
    cmd_DeviceWriteEEprom   = 0x3E,  // '>' write
// PARAM: [ADRESS + BuffLen] Buffer[1..256]
// RETURN: ACK

// Set Interface Mode
    cmd_InterfaceSetMode   = 0x3F,   // '?'
// PARAM: uint8_t Mode (interfaceMode_e)
// RETURN: ACK or ACK_I_INVALID_PARAM
} cmd_4way_e;

// responses
#define ACK_OK                  0x00
// #define ACK_I_UNKNOWN_ERROR   0x01
#define ACK_I_INVALID_CMD       0x02
#define ACK_I_INVALID_CRC       0x03
#define ACK_I_VERIFY_ERROR      0x04
// #define ACK_D_INVALID_COMMAND 0x05
// #define ACK_D_COMMAND_FAILED  0x06
// #define ACK_D_UNKNOWN_ERROR   0x07

#define ACK_I_INVALID_CHANNEL   0x08
#define ACK_I_INVALID_PARAM     0x09
#define ACK_D_GENERAL_ERROR     0x0F

/* Copyright (c) 2002, 2003, 2004  Marek Michalkiewicz
   Copyright (c) 2005, 2007 Joerg Wunsch
   Copyright (c) 2013 Dave Hylands
   Copyright (c) 2013 Frederic Nadeau
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */
uint16_t _crc_xmodem_update (uint16_t crc, uint8_t data) {
        int i;

        crc = crc ^ ((uint16_t)data << 8);
        for (i = 0; i < 8; i++){
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
        return crc;
}
// * End copyright

static uint16_t signaturesAtmel[] =  {0x9307, 0x930A, 0x930F, 0x940B, 0};
static uint16_t signaturesSilabs[] = {0xF310, 0xF330, 0xF410, 0xF390, 0xF850, 0xE8B1, 0xE8B2, 0};

static bool signatureMatch(uint16_t signature, uint16_t *list)
{
    for(; *list; list++)
        if(signature == *list)
            return true;
    return false;
}

static uint8_t currentInterfaceMode;

static uint8_t Connect(deviceInfo_t *pDeviceInfo)
{
    for (int i = 0; i < 3; i++) {    // TODO - probaly useless
#if defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
        if (Stk_ConnectEx(pDeviceInfo)) {
            currentInterfaceMode = imSK;
            if(signatureMatch(pDeviceInfo->signature, signaturesAtmel))
                return 1;
        }
#endif
#if defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER)
        if (BL_ConnectEx(pDeviceInfo)) {
            if(signatureMatch(pDeviceInfo->signature, signaturesSilabs)) {
                currentInterfaceMode = imSIL_BLB;
                return 1;
            }
            if(signatureMatch(pDeviceInfo->signature, signaturesAtmel)) {
                currentInterfaceMode = imATM_BLB;
                return 1;
            }
        }
#endif
    }
    return 0;
}

static serialPort_t *port;
static uint16_t crcIn, crcOut;

static uint8_t ReadByte(void)
{
    // need timeout?
    while (!serialRxBytesWaiting(port));
    return serialRead(port);
}

static uint8_t ReadByteCrc(void)
{
    uint8_t b = ReadByte();
    crcIn = _crc_xmodem_update(crcIn, b);
    return b;
}

static void WriteByte(uint8_t b)
{
    serialWrite(port, b);
}

static void WriteByteCrc(uint8_t b)
{
    WriteByte(b);
    crcOut = _crc_xmodem_update(crcOut, b);
}

void Process4WayInterface(serialPort_t *serial) {
    uint8_t paramBuf[256];
    uint8_t paramInLen;
    uint8_t CMD;
    uint8_t ACK_OUT;
    uint8_16_u Dummy;
    uint8_t O_PARAM_LEN;
    uint8_t *O_PARAM;
    ioMem_t ioMem;

    port = serial;

    // Start here  with UART Main loop
#ifdef BEEPER
    // fix for buzzer often starts beeping continuously when the ESCs are read
    // switch beeper silent here
    beeperSilence();
#endif
    bool isExitScheduled = false;

    while(1) {
        // restart looking for new sequence from host
        crcIn = 0;
        uint8_t esc = ReadByteCrc();
        if(esc != cmd_Local_Escape)
            continue;  // wait for sync character

        RX_LED_ON;

        Dummy.word = 0;
        O_PARAM = &Dummy.bytes[0];
        O_PARAM_LEN = 1;
        CMD = ReadByteCrc();
        ioMem.D_FLASH_ADDR_H = ReadByteCrc();
        ioMem.D_FLASH_ADDR_L = ReadByteCrc();
        paramInLen = ReadByteCrc();

        for(int i = 0; i < paramInLen; i++)
            paramBuf[i] = ReadByteCrc();

        uint16_t crc;
        crc = ReadByte() << 8;
        crc |= ReadByte();

        RX_LED_OFF;

        ACK_OUT = ACK_OK;

        if(crcIn != crc) {
            ACK_OUT = ACK_I_INVALID_CRC;
        }

        if (ACK_OUT == ACK_OK) {
            // wtf.D_FLASH_ADDR_H=Adress_H;
            // wtf.D_FLASH_ADDR_L=Adress_L;
            ioMem.D_PTR_I = paramBuf;

            switch(CMD) {
                // ******* Interface related stuff *******
                case cmd_InterfaceTestAlive:
                    if (isMcuConnected()) {
                        switch(currentInterfaceMode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                            case imATM_BLB:
                            case imSIL_BLB:
                                if (!BL_SendCMDKeepAlive()) { // SetStateDisconnected() included
                                    ACK_OUT = ACK_D_GENERAL_ERROR;
                                }
                                break;
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                            case imSK:
                                if (!Stk_SignOn()) { // SetStateDisconnected();
                                    ACK_OUT = ACK_D_GENERAL_ERROR;
                                }
                                break;
#endif
                            default:
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                        }
                        if (ACK_OUT != ACK_OK)
                            setDisconnected();
                    }
                    break;

                case cmd_ProtocolGetVersion:
                    // Only interface itself, no matter what Device
                    Dummy.bytes[0] = SERIAL_4WAY_PROTOCOL_VER;
                    break;

                case cmd_InterfaceGetName:
                    // Only interface itself, no matter what Device
                    // O_PARAM_LEN=16;
                    O_PARAM_LEN = strlen(SERIAL_4WAY_INTERFACE_NAME_STR);
                    O_PARAM = (uint8_t *)SERIAL_4WAY_INTERFACE_NAME_STR;
                    break;

                case cmd_InterfaceGetVersion:
                    // Only interface itself, no matter what Device
                    // Dummy = iUart_res_InterfVersion;
                    O_PARAM_LEN = 2;
                    Dummy.bytes[0] = SERIAL_4WAY_VERSION_HI;
                    Dummy.bytes[1] = SERIAL_4WAY_VERSION_LO;
                    break;

                case cmd_InterfaceExit:
                    isExitScheduled = true;
                    break;

                case cmd_InterfaceSetMode:
                    switch(paramBuf[0]) {
#if defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER)
                        case imSIL_BLB:
                        case imATM_BLB:
#endif
#if defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
                        case imSK:
#endif
                            currentInterfaceMode = paramBuf[0];
                            break;
                        default:
                            ACK_OUT = ACK_I_INVALID_PARAM;
                    }
                    break;

                case cmd_DeviceReset:
                    if(paramBuf[0] >= escCount) {
                        ACK_OUT = ACK_I_INVALID_CHANNEL;
                        break;
                    }
                    // Channel may change here
                    escSelected = paramBuf[0];
                    switch (currentInterfaceMode) {
                        case imSIL_BLB:            // TODO!
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                        case imATM_BLB:
                            BL_SendCMDRunRestartBootloader(&deviceInfo);
                            break;
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                        case imSK:
                            break;
#endif
                    }
                    setDisconnected();
                    break;

                case cmd_DeviceInitFlash:
                    setDisconnected();
                    if (paramBuf[0] >= escCount) {
                        ACK_OUT = ACK_I_INVALID_CHANNEL;
                        break;
                    }
                    //Channel may change here
                    //ESC_LO or ESC_HI; Halt state for prev channel
                    escSelected = paramBuf[0];
                    O_PARAM_LEN = sizeof(deviceInfo);
                    O_PARAM = (uint8_t*)&deviceInfo;
                    if(!Connect(&deviceInfo)) {
                        setDisconnected();
                        ACK_OUT = ACK_D_GENERAL_ERROR;
                    }
                    deviceInfo.interfaceMode = currentInterfaceMode;
                    break;

#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                case cmd_DeviceEraseAll:
                    switch(currentInterfaceMode) {
                        case imSK:
                            if (!Stk_Chip_Erase())
                                ACK_OUT=ACK_D_GENERAL_ERROR;
                            break;
                        default:
                            ACK_OUT = ACK_I_INVALID_CMD;
                    }
                    break;
#endif

#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                case cmd_DevicePageErase:
                    switch (currentInterfaceMode) {
                        case imSIL_BLB:
                            Dummy.bytes[0] = paramBuf[0];
                            //Address = Page * 512
                            ioMem.D_FLASH_ADDR_H = (Dummy.bytes[0] << 1);
                            ioMem.D_FLASH_ADDR_L = 0;
                            if (!BL_PageErase(&ioMem))
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                            break;
                        default:
                            ACK_OUT = ACK_I_INVALID_CMD;
                    }
                    break;
#endif

                    //*** Device Memory Read Ops ***
                case cmd_DeviceRead:
                    ioMem.D_NUM_BYTES = paramBuf[0];
                    switch(currentInterfaceMode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                        case imSIL_BLB:
                        case imATM_BLB:
                            if(!BL_ReadFlash(currentInterfaceMode, &ioMem))
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                            break;
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                        case imSK:
                            if(!Stk_ReadFlash(&ioMem))
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                            break;
#endif
                        default:
                            ACK_OUT = ACK_I_INVALID_CMD;
                    }
                    if (ACK_OUT == ACK_OK) {
                        O_PARAM_LEN = ioMem.D_NUM_BYTES;
                        O_PARAM = (uint8_t *)&paramBuf;
                    }
                    break;

                case cmd_DeviceReadEEprom:
                    ioMem.D_NUM_BYTES = paramBuf[0];
                    switch (currentInterfaceMode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                        case imATM_BLB:
                            if (!BL_ReadEEprom(&ioMem))
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                            break;
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                        case imSK:
                            if (!Stk_ReadEEprom(&ioMem))
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                            break;
#endif
                        default:
                            ACK_OUT = ACK_I_INVALID_CMD;
                    }
                    if(ACK_OUT == ACK_OK) {
                        O_PARAM_LEN = ioMem.D_NUM_BYTES;
                        O_PARAM = (uint8_t *)&paramBuf;
                    }
                    break;

                    //*** Device Memory Write Ops ***
                case cmd_DeviceWrite:
                    ioMem.D_NUM_BYTES = paramInLen;
                    switch (currentInterfaceMode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                        case imSIL_BLB:
                        case imATM_BLB:
                            if (!BL_WriteFlash(&ioMem))
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                            break;
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                        case imSK:
                            if (!Stk_WriteFlash(&ioMem))
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                            break;
#endif
                        default:
                            ACK_OUT = ACK_I_INVALID_CMD;
                    }
                    break;

                case cmd_DeviceWriteEEprom:
                    ioMem.D_NUM_BYTES = paramInLen;
                    switch (currentInterfaceMode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                        case imSIL_BLB:
                            ACK_OUT = ACK_I_INVALID_CMD;
                            break;
                        case imATM_BLB:
                            if (!BL_WriteEEprom(&ioMem))
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                            break;
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                        case imSK:
                            if (!Stk_WriteEEprom(&ioMem))
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                            break;
#endif
                        default:
                            ACK_OUT = ACK_I_INVALID_CMD;
                    }
                    break;
                default:
                    ACK_OUT = ACK_I_INVALID_CMD;
            }
        }

        crcOut = 0;

        TX_LED_ON;
        serialBeginWrite(port);
        WriteByteCrc(cmd_Remote_Escape);
        WriteByteCrc(CMD);
        WriteByteCrc(ioMem.D_FLASH_ADDR_H);
        WriteByteCrc(ioMem.D_FLASH_ADDR_L);
        WriteByteCrc(O_PARAM_LEN);

        for(int i = 0; i < O_PARAM_LEN; i++)
            WriteByteCrc(O_PARAM[i]);

        WriteByteCrc(ACK_OUT);
        WriteByte(crcOut >> 8);
        WriteByte(crcOut & 0xff);
        serialEndWrite(port);
        TX_LED_OFF;
        if (isExitScheduled) {
            DeInitialize4WayInterface();
            return;
        }
    }
}

#endif
