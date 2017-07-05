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

#include "platform.h"

#ifdef  USE_SERIAL_4WAY_BLHELI_INTERFACE

#include "drivers/buf_writer.h"
#include "drivers/io.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"
#include "drivers/system.h"

#include "flight/mixer.h"

#include "io/beeper.h"
#include "io/serial_4way.h"

#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#include "io/serial_4way_avrootloader.h"
#endif
#if defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
#include "io/serial_4way_stk500v2.h"
#endif

#if defined(USE_HAL_DRIVER)
#define Bit_RESET GPIO_PIN_RESET
#endif

#define USE_TXRX_LED

#ifdef  USE_TXRX_LED
#define RX_LED_OFF LED0_OFF
#define RX_LED_ON LED0_ON
#ifdef  LED1
#define TX_LED_OFF LED1_OFF
#define TX_LED_ON LED1_ON
#else
#define TX_LED_OFF LED0_OFF
#define TX_LED_ON LED0_ON
#endif
#else
#define RX_LED_OFF
#define RX_LED_ON
#define TX_LED_OFF
#define TX_LED_ON
#endif

#define SERIAL_4WAY_INTERFACE_NAME_STR "m4wFCIntf"
// *** change to adapt Revision
#define SERIAL_4WAY_VER_MAIN 14
#define SERIAL_4WAY_VER_SUB_1 (uint8_t) 4
#define SERIAL_4WAY_VER_SUB_2 (uint8_t) 04

#define SERIAL_4WAY_PROTOCOL_VER 106
// *** end

#if (SERIAL_4WAY_VER_MAIN > 24)
#error "beware of SERIAL_4WAY_VER_SUB_1 is uint8_t"
#endif

#define SERIAL_4WAY_VERSION (uint16_t) ((SERIAL_4WAY_VER_MAIN * 1000) + (SERIAL_4WAY_VER_SUB_1 * 100) + SERIAL_4WAY_VER_SUB_2)

#define SERIAL_4WAY_VERSION_HI (uint8_t) (SERIAL_4WAY_VERSION / 100)
#define SERIAL_4WAY_VERSION_LO (uint8_t) (SERIAL_4WAY_VERSION % 100)

static uint8_t escCount;

escHardware_t escHardware[MAX_SUPPORTED_MOTORS];

uint8_t selected_esc;

uint8_32_u DeviceInfo;

#define DeviceInfoSize 4

inline bool isMcuConnected(void)
{
    return (DeviceInfo.bytes[0] > 0);
}

inline bool isEscHi(uint8_t selEsc)
{
    return (IORead(escHardware[selEsc].io) != Bit_RESET);
}
inline bool isEscLo(uint8_t selEsc)
{
    return (IORead(escHardware[selEsc].io) == Bit_RESET);
}

inline void setEscHi(uint8_t selEsc)
{
    IOHi(escHardware[selEsc].io);
}

inline void setEscLo(uint8_t selEsc)
{
    IOLo(escHardware[selEsc].io);
}

inline void setEscInput(uint8_t selEsc)
{
    IOConfigGPIO(escHardware[selEsc].io, IOCFG_IPU);
}

inline void setEscOutput(uint8_t selEsc)
{
    IOConfigGPIO(escHardware[selEsc].io, IOCFG_OUT_PP);
}

uint8_t esc4wayInit(void)
{
    // StopPwmAllMotors();
    pwmDisableMotors();
    escCount = 0;
    memset(&escHardware, 0, sizeof(escHardware));
    pwmIOConfiguration_t *pwmIOConfiguration = pwmGetOutputConfiguration();
    for (volatile uint8_t i = 0; i < pwmIOConfiguration->ioCount; i++) {
        if ((pwmIOConfiguration->ioConfigurations[i].flags & PWM_PF_MOTOR) == PWM_PF_MOTOR) {
            if (motor[pwmIOConfiguration->ioConfigurations[i].index] > 0) {
                escHardware[escCount].io = IOGetByTag(pwmIOConfiguration->ioConfigurations[i].timerHardware->tag);
                setEscInput(escCount);
                setEscHi(escCount);
                escCount++;
            }
        }
    }
    return escCount;
}

void esc4wayRelease(void)
{
    while (escCount > 0) {
        escCount--;
        IOConfigGPIO(escHardware[escCount].io, IOCFG_AF_PP);
        setEscLo(escCount);
    }
    pwmEnableMotors();
}


#define SET_DISCONNECTED DeviceInfo.words[0] = 0

#define INTF_MODE_IDX 3  // index for DeviceInfostate

// Interface related only
// establish and test connection to the Interface

// Send Structure
// ESC + CMD PARAM_LEN [PARAM (if len > 0)] CRC16_Hi CRC16_Lo
// Return
// ESC CMD PARAM_LEN [PARAM (if len > 0)] + ACK (uint8_t OK or ERR) + CRC16_Hi CRC16_Lo

#define cmd_Remote_Escape 0x2E // '.'
#define cmd_Local_Escape  0x2F // '/'

// Test Interface still present
#define cmd_InterfaceTestAlive 0x30 // '0' alive
// RETURN: ACK

// get Protocol Version Number 01..255
#define cmd_ProtocolGetVersion 0x31  // '1' version
// RETURN: uint8_t VersionNumber + ACK

// get Version String
#define cmd_InterfaceGetName 0x32 // '2' name
// RETURN: String + ACK

//get Version Number 01..255
#define cmd_InterfaceGetVersion 0x33  // '3' version
// RETURN: uint8_t AVersionNumber + ACK


// Exit / Restart Interface - can be used to switch to Box Mode
#define cmd_InterfaceExit 0x34       // '4' exit
// RETURN: ACK

// Reset the Device connected to the Interface
#define cmd_DeviceReset 0x35        // '5' reset
// RETURN: ACK

// Get the Device ID connected
// #define cmd_DeviceGetID 0x36      //'6' device id removed since 06/106
// RETURN: uint8_t DeviceID + ACK

// Initialize Flash Access for Device connected
#define cmd_DeviceInitFlash 0x37    // '7' init flash access
// RETURN: ACK

// Erase the whole Device Memory of connected Device
#define cmd_DeviceEraseAll 0x38     // '8' erase all
// RETURN: ACK

// Erase one Page of Device Memory of connected Device
#define cmd_DevicePageErase 0x39    // '9' page erase
// PARAM: uint8_t APageNumber
// RETURN: ACK

// Read to Buffer from Device Memory of connected Device // Buffer Len is Max 256 Bytes
// BuffLen = 0 means 256 Bytes
#define cmd_DeviceRead 0x3A  // ':' read Device
// PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BuffLen[0..255]
// RETURN: PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255] ACK

// Write to Buffer for Device Memory of connected Device // Buffer Len is Max 256 Bytes
// BuffLen = 0 means 256 Bytes
#define cmd_DeviceWrite 0x3B    // ';' write
// PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255]
// RETURN: ACK

// Set C2CK low infinite ) permanent Reset state
#define cmd_DeviceC2CK_LOW 0x3C // '<'
// RETURN: ACK

// Read to Buffer from Device Memory of connected Device //Buffer Len is Max 256 Bytes
// BuffLen = 0 means 256 Bytes
#define cmd_DeviceReadEEprom 0x3D  // '=' read Device
// PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BuffLen[0..255]
// RETURN: PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255] ACK

// Write to Buffer for Device Memory of connected Device // Buffer Len is Max 256 Bytes
// BuffLen = 0 means 256 Bytes
#define cmd_DeviceWriteEEprom 0x3E  // '>' write
// PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255]
// RETURN: ACK

// Set Interface Mode
#define cmd_InterfaceSetMode 0x3F   // '?'
// #define imC2 0
// #define imSIL_BLB 1
// #define imATM_BLB 2
// #define imSK 3
// PARAM: uint8_t Mode
// RETURN: ACK or ACK_I_INVALID_CHANNEL

// responses
#define ACK_OK                  0x00
// #define ACK_I_UNKNOWN_ERROR       0x01
#define ACK_I_INVALID_CMD       0x02
#define ACK_I_INVALID_CRC       0x03
#define ACK_I_VERIFY_ERROR      0x04
// #define ACK_D_INVALID_COMMAND 0x05
// #define ACK_D_COMMAND_FAILED  0x06
// #define ACK_D_UNKNOWN_ERROR       0x07

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
        for (i=0; i < 8; i++){
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
        return crc;
}
// * End copyright


#define ATMEL_DEVICE_MATCH ((pDeviceInfo->words[0] == 0x9307) || (pDeviceInfo->words[0] == 0x930A) || \
        (pDeviceInfo->words[0] == 0x930F) || (pDeviceInfo->words[0] == 0x940B))

#define SILABS_DEVICE_MATCH ((pDeviceInfo->words[0] == 0xF310)||(pDeviceInfo->words[0] ==0xF330) || \
        (pDeviceInfo->words[0] == 0xF410) || (pDeviceInfo->words[0] == 0xF390) || \
        (pDeviceInfo->words[0] == 0xF850) || (pDeviceInfo->words[0] == 0xE8B1) || \
        (pDeviceInfo->words[0] == 0xE8B2))

static uint8_t CurrentInterfaceMode;

static uint8_t Connect(uint8_32_u *pDeviceInfo)
{
    for (uint8_t I = 0; I < 3; ++I) {
        #if (defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER) && defined(USE_SERIAL_4WAY_SK_BOOTLOADER))
        if (Stk_ConnectEx(pDeviceInfo) && ATMEL_DEVICE_MATCH) {
            CurrentInterfaceMode = imSK;
            return 1;
        } else {
            if (BL_ConnectEx(pDeviceInfo)) {
                if  SILABS_DEVICE_MATCH {
                    CurrentInterfaceMode = imSIL_BLB;
                    return 1;
                } else if ATMEL_DEVICE_MATCH {
                    CurrentInterfaceMode = imATM_BLB;
                    return 1;
                }
            }
        }
        #elif defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER)
        if (BL_ConnectEx(pDeviceInfo)) {
            if SILABS_DEVICE_MATCH {
                CurrentInterfaceMode = imSIL_BLB;
                return 1;
            } else if ATMEL_DEVICE_MATCH {
                CurrentInterfaceMode = imATM_BLB;
                return 1;
            }
        }
        #elif defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
        if (Stk_ConnectEx(pDeviceInfo)) {
            CurrentInterfaceMode = imSK;
            if ATMEL_DEVICE_MATCH return 1;
        }
        #endif
    }
    return 0;
}

static serialPort_t *port;

static uint8_t ReadByte(void)
{
    // need timeout?
    while (!serialRxBytesWaiting(port));
    return serialRead(port);
}

static uint8_16_u CRC_in;
static uint8_t ReadByteCrc(void)
{
    uint8_t b = ReadByte();
    CRC_in.word = _crc_xmodem_update(CRC_in.word, b);
    return b;
}

static void WriteByte(uint8_t b)
{
    serialWrite(port, b);
}

static uint8_16_u CRCout;
static void WriteByteCrc(uint8_t b)
{
    WriteByte(b);
    CRCout.word = _crc_xmodem_update(CRCout.word, b);
}

void esc4wayProcess(serialPort_t *mspPort)
{

    uint8_t ParamBuf[256];
    uint8_t ESC;
    uint8_t I_PARAM_LEN;
    uint8_t CMD;
    uint8_t ACK_OUT;
    uint8_16_u CRC_check;
    uint8_16_u Dummy;
    uint8_t O_PARAM_LEN;
    uint8_t *O_PARAM;
    uint8_t *InBuff;
    ioMem_t ioMem;

    port = mspPort;

    // Start here  with UART Main loop
    #ifdef BEEPER
    // fix for buzzer often starts beeping continuously when the ESCs are read
    // switch beeper silent here
    beeperSilence();
    #endif
    bool isExitScheduled = false;

    while (1) {
        // restart looking for new sequence from host
        do {
            CRC_in.word = 0;
            ESC = ReadByteCrc();
        } while (ESC != cmd_Local_Escape);

        RX_LED_ON;

        Dummy.word = 0;
        O_PARAM = &Dummy.bytes[0];
        O_PARAM_LEN = 1;
        CMD = ReadByteCrc();
        ioMem.D_FLASH_ADDR_H = ReadByteCrc();
        ioMem.D_FLASH_ADDR_L = ReadByteCrc();
        I_PARAM_LEN = ReadByteCrc();

        InBuff = ParamBuf;
        uint8_t i = I_PARAM_LEN;
        do {
          *InBuff = ReadByteCrc();
          InBuff++;
          i--;
        } while (i != 0);

        CRC_check.bytes[1] = ReadByte();
        CRC_check.bytes[0] = ReadByte();

        if (CRC_check.word == CRC_in.word) {
            ACK_OUT = ACK_OK;
        } else {
            ACK_OUT = ACK_I_INVALID_CRC;
        }

        TX_LED_ON;

        if (ACK_OUT == ACK_OK)
        {
            // wtf.D_FLASH_ADDR_H=Adress_H;
            // wtf.D_FLASH_ADDR_L=Adress_L;
            ioMem.D_PTR_I = ParamBuf;


            switch (CMD) {
                // ******* Interface related stuff *******
                case cmd_InterfaceTestAlive:
                {
                    if (isMcuConnected()){
                        switch (CurrentInterfaceMode)
                        {
                            #ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                            case imATM_BLB:
                            case imSIL_BLB:
                            {
                                if (!BL_SendCMDKeepAlive()) { // SetStateDisconnected() included
                                    ACK_OUT = ACK_D_GENERAL_ERROR;
                                }
                                break;
                            }
                            #endif
                            #ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                            case imSK:
                            {
                                if (!Stk_SignOn()) { // SetStateDisconnected();
                                    ACK_OUT = ACK_D_GENERAL_ERROR;
                                }
                                break;
                            }
                            #endif
                            default:
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                        }
                        if ( ACK_OUT != ACK_OK) SET_DISCONNECTED;
                    }
                    break;
                }
                case cmd_ProtocolGetVersion:
                {
                    // Only interface itself, no matter what Device
                    Dummy.bytes[0] = SERIAL_4WAY_PROTOCOL_VER;
                    break;
                }

                case cmd_InterfaceGetName:
                {
                    // Only interface itself, no matter what Device
                    // O_PARAM_LEN=16;
                    O_PARAM_LEN = strlen(SERIAL_4WAY_INTERFACE_NAME_STR);
                    O_PARAM = (uint8_t *)SERIAL_4WAY_INTERFACE_NAME_STR;
                    break;
                }

                case cmd_InterfaceGetVersion:
                {
                    // Only interface itself, no matter what Device
                    // Dummy = iUart_res_InterfVersion;
                    O_PARAM_LEN = 2;
                    Dummy.bytes[0] = SERIAL_4WAY_VERSION_HI;
                    Dummy.bytes[1] = SERIAL_4WAY_VERSION_LO;
                    break;
                }
                case cmd_InterfaceExit:
                {
                    isExitScheduled = true;
                    break;
                }
                case cmd_InterfaceSetMode:
                {
#if defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER) && defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
                    if ((ParamBuf[0] <= imSK) && (ParamBuf[0] >= imSIL_BLB)) {
#elif defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER)
                    if ((ParamBuf[0] <= imATM_BLB) && (ParamBuf[0] >= imSIL_BLB)) {
#elif defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
                    if (ParamBuf[0] == imSK) {
#endif
                        CurrentInterfaceMode = ParamBuf[0];
                    } else {
                        ACK_OUT = ACK_I_INVALID_PARAM;
                    }
                    break;
                }

                case cmd_DeviceReset:
                {
                    if (ParamBuf[0] < escCount) {
                        // Channel may change here
                        selected_esc = ParamBuf[0];
                    }
                    else {
                        ACK_OUT = ACK_I_INVALID_CHANNEL;
                        break;
                    }
                    switch (CurrentInterfaceMode)
                    {
                    case imSIL_BLB:
                        #ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                        case imATM_BLB:
                        {
                            BL_SendCMDRunRestartBootloader(&DeviceInfo);
                            break;
                        }
                        #endif
                        #ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                        case imSK:
                        {
                            break;
                        }
                        #endif
                    }
                    SET_DISCONNECTED;
                    break;
                }
                case cmd_DeviceInitFlash:
                {
                    SET_DISCONNECTED;
                    if (ParamBuf[0] < escCount) {
                        //Channel may change here
                        //ESC_LO or ESC_HI; Halt state for prev channel
                        selected_esc = ParamBuf[0];
                    } else {
                        ACK_OUT = ACK_I_INVALID_CHANNEL;
                        break;
                    }
                    O_PARAM_LEN = DeviceInfoSize; //4
                    O_PARAM = (uint8_t *)&DeviceInfo;
                    if (Connect(&DeviceInfo)) {
                        DeviceInfo.bytes[INTF_MODE_IDX] = CurrentInterfaceMode;
                    } else {
                        SET_DISCONNECTED;
                        ACK_OUT = ACK_D_GENERAL_ERROR;
                    }
                    break;
                }

                #ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                case cmd_DeviceEraseAll:
                {
                    switch (CurrentInterfaceMode)
                    {
                        case imSK:
                        {
                            if (!Stk_Chip_Erase()) ACK_OUT=ACK_D_GENERAL_ERROR;
                            break;
                        }
                        default:
                            ACK_OUT = ACK_I_INVALID_CMD;
                    }
                    break;
                }
                #endif

                #ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                case cmd_DevicePageErase:
                {
                    switch (CurrentInterfaceMode)
                    {
                        case imSIL_BLB:
                        {
                            Dummy.bytes[0] = ParamBuf[0];
                            //Address = Page * 512
                            ioMem.D_FLASH_ADDR_H = (Dummy.bytes[0] << 1);
                            ioMem.D_FLASH_ADDR_L = 0;
                            if (!BL_PageErase(&ioMem)) ACK_OUT = ACK_D_GENERAL_ERROR;
                            break;
                        }
                        default:
                            ACK_OUT = ACK_I_INVALID_CMD;
                    }
                    break;
                }
                #endif

                //*** Device Memory Read Ops ***
                case cmd_DeviceRead:
                {
                    ioMem.D_NUM_BYTES = ParamBuf[0];
                    /*
                    wtf.D_FLASH_ADDR_H=Adress_H;
                    wtf.D_FLASH_ADDR_L=Adress_L;
                    wtf.D_PTR_I = BUF_I;
                    */
                    switch (CurrentInterfaceMode)
                    {
                        #ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                        case imSIL_BLB:
                        case imATM_BLB:
                        {
                            if (!BL_ReadFlash(CurrentInterfaceMode, &ioMem))
                            {
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                            }
                            break;
                        }
                        #endif
                        #ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                        case imSK:
                        {
                            if (!Stk_ReadFlash(&ioMem))
                            {
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                            }
                            break;
                        }
                        #endif
                        default:
                            ACK_OUT = ACK_I_INVALID_CMD;
                    }
                    if (ACK_OUT == ACK_OK)
                    {
                        O_PARAM_LEN = ioMem.D_NUM_BYTES;
                        O_PARAM = (uint8_t *)&ParamBuf;
                    }
                    break;
                }

                case cmd_DeviceReadEEprom:
                {
                    ioMem.D_NUM_BYTES = ParamBuf[0];
                    /*
                    wtf.D_FLASH_ADDR_H = Adress_H;
                    wtf.D_FLASH_ADDR_L = Adress_L;
                    wtf.D_PTR_I = BUF_I;
                    */
                    switch (CurrentInterfaceMode)
                    {
                        #ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                        case imATM_BLB:
                        {
                            if (!BL_ReadEEprom(&ioMem))
                            {
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                            }
                            break;
                        }
                        #endif
                        #ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                        case imSK:
                        {
                            if (!Stk_ReadEEprom(&ioMem))
                            {
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                            }
                            break;
                        }
                        #endif
                        default:
                            ACK_OUT = ACK_I_INVALID_CMD;
                    }
                    if (ACK_OUT == ACK_OK)
                    {
                        O_PARAM_LEN = ioMem.D_NUM_BYTES;
                        O_PARAM = (uint8_t *)&ParamBuf;
                    }
                    break;
                }

                //*** Device Memory Write Ops ***
                case cmd_DeviceWrite:
                {
                    ioMem.D_NUM_BYTES = I_PARAM_LEN;
                    /*
                    wtf.D_FLASH_ADDR_H=Adress_H;
                    wtf.D_FLASH_ADDR_L=Adress_L;
                    wtf.D_PTR_I = BUF_I;
                    */
                    switch (CurrentInterfaceMode)
                    {
                        #ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                        case imSIL_BLB:
                        case imATM_BLB:
                        {
                            if (!BL_WriteFlash(&ioMem)) {
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                            }
                            break;
                        }
                        #endif
                        #ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                        case imSK:
                        {
                            if (!Stk_WriteFlash(&ioMem))
                            {
                                ACK_OUT = ACK_D_GENERAL_ERROR;
                            }
                            break;
                        }
                        #endif
                    }
                    break;
                }

                case cmd_DeviceWriteEEprom:
                {
                    ioMem.D_NUM_BYTES = I_PARAM_LEN;
                    ACK_OUT = ACK_D_GENERAL_ERROR;
                    /*
                    wtf.D_FLASH_ADDR_H=Adress_H;
                    wtf.D_FLASH_ADDR_L=Adress_L;
                    wtf.D_PTR_I = BUF_I;
                    */
                    switch (CurrentInterfaceMode)
                    {
                        #ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                        case imSIL_BLB:
                        {
                            ACK_OUT = ACK_I_INVALID_CMD;
                            break;
                        }
                        case imATM_BLB:
                        {
                            if (BL_WriteEEprom(&ioMem))
                            {
                                ACK_OUT = ACK_OK;
                            }
                            break;
                        }
                        #endif
                        #ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                        case imSK:
                        {
                            if (Stk_WriteEEprom(&ioMem))
                            {
                                ACK_OUT = ACK_OK;
                            }
                            break;
                        }
                        #endif
                    }
                    break;
                }
                default:
                {
                    ACK_OUT = ACK_I_INVALID_CMD;
                }
            }
        }

        CRCout.word = 0;

        RX_LED_OFF;

        serialBeginWrite(port);
        WriteByteCrc(cmd_Remote_Escape);
        WriteByteCrc(CMD);
        WriteByteCrc(ioMem.D_FLASH_ADDR_H);
        WriteByteCrc(ioMem.D_FLASH_ADDR_L);
        WriteByteCrc(O_PARAM_LEN);

        i=O_PARAM_LEN;
        do {
            while (!serialTxBytesFree(port));

            WriteByteCrc(*O_PARAM);
            O_PARAM++;
            i--;
        } while (i > 0);

        WriteByteCrc(ACK_OUT);
        WriteByte(CRCout.bytes[1]);
        WriteByte(CRCout.bytes[0]);
        serialEndWrite(port);

        TX_LED_OFF;
        if (isExitScheduled) {
            esc4wayRelease();
            return;
        }
    };
}



#endif
