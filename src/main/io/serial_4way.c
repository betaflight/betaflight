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
//#include "io/serial_msp.h"
#include "io/serial_4way.h"
#include "io/serial_4way_impl.h"
#include "io/serial_4way_avrootloader.h"
#include "io/serial_4way_stk500v2.h"

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
#define SERIAL_4WAY_VER_MAIN  14
#define SERIAL_4WAY_VER_SUB_1 4
#define SERIAL_4WAY_VER_SUB_2 4
#define SERIAL_4WAY_PROTOCOL_VER 106

#if SERIAL_4WAY_VER_MAIN > 24
# error "SERIAL_4WAY_VER_MAIN * 10 + SERIAL_4WAY_VER_SUB_1 must fit in uint8_t"
#endif
#if SERIAL_4WAY_VER_SUB_1 >= 10
# warning "SERIAL_4WAY_VER_SUB_1 should be 0-9"
#endif

#if SERIAL_4WAY_VER_SUB_2 >= 100
# warning "SERIAL_4WAY_VER_SUB_2 should be <= 99 (9.9)"
#endif

#define SERIAL_4WAY_VERSION_HI (uint8_t)(SERIAL_4WAY_VER_MAIN * 10 + SERIAL_4WAY_VER_SUB_1)
#define SERIAL_4WAY_VERSION_LO (uint8_t)(SERIAL_4WAY_VER_SUB_2)

static uint8_t escCount;
uint8_t escSelected;
escHardware_t escHardware[MAX_PWM_MOTORS];

bool esc4wayExitRequested = false;

static escDeviceInfo_t deviceInfo;

static bool isMcuConnected(void)
{
    return deviceInfo.signature != 0;
}

static void setDisconnected(void)
{
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

static uint32_t getPinPos(uint32_t pin)
{
    for (int pinPos = 0; pinPos < 16; pinPos++) {
        uint32_t pinMask = (0x1 << pinPos);
        if (pin & pinMask) {
            return pinPos;
        }
    }
    return 0;
}

// Initialize 4way ESC interface
// initializes internal structures
// returns number of ESCs available
int esc4wayInit(void)
{
    // StopPwmAllMotors();
    memset(&escHardware, 0, sizeof(escHardware));
    pwmIOConfiguration_t *pwmIOConfiguration = pwmGetOutputConfiguration();
    int escIdx = 0;
    for (int i = 0; i < pwmIOConfiguration->ioCount; i++) {
        if ((pwmIOConfiguration->ioConfigurations[i].flags & PWM_PF_MOTOR) == PWM_PF_MOTOR) {
            if(motor[pwmIOConfiguration->ioConfigurations[i].index] > 0) {
                escHardware[escIdx].gpio = pwmIOConfiguration->ioConfigurations[i].timerHardware->gpio;
                escHardware[escIdx].pin = pwmIOConfiguration->ioConfigurations[i].timerHardware->pin;
                escHardware[escIdx].pinpos = getPinPos(escHardware[escIdx].pin);
                escHardware[escIdx].gpio_config_INPUT.pin = escHardware[escIdx].pin;
                escHardware[escIdx].gpio_config_INPUT.speed = Speed_2MHz; // see pwmOutConfig()
                escHardware[escIdx].gpio_config_INPUT.mode = Mode_IPU;
                escHardware[escIdx].gpio_config_OUTPUT = escHardware[escIdx].gpio_config_INPUT;
                escHardware[escIdx].gpio_config_OUTPUT.mode = Mode_Out_PP;
                escIdx++;
            }
        }
    }
    escCount = escIdx;
    return escCount;
}

// stat BLHeli 4way interface
// sets all ESC lines as output + hi
void esc4wayStart(void)
{
    pwmDisableMotors();    // prevent updating PWM registers
    for (int i = 0; i < escCount; i++) {
        setEscInput(i);
        setEscHi(i);
    }
}

// stops BLHeli 4way interface
// returns all claimed pins back to PWM drivers, reenables PWM
void esc4wayRelease(void)
{
    for(int i = 0; i < escCount; i++) {
        escHardware[i].gpio_config_OUTPUT.mode = Mode_AF_PP; // see pwmOutConfig()
        setEscOutput(i);
        setEscLo(i);
    }
    escCount = 0;
    pwmEnableMotors();
}

// BLHeliSuite packet framing
// for reference, see 'Manuals/BLHeliSuite 4w-if protocol.pdf' from BLHeliSuite
// Send Structure
// ESC CMD ADDR_H ADDR_L PARAM_LEN PARAM (256B if len == 0) CRC16_Hi CRC16_Lo
// Return
// ESC CMD ADDR_H ADDR_L PARAM_LEN PARAM (256B if len == 0) + ACK (uint8_t OK or ERR) + CRC16_Hi CRC16_Lo

// esc4wayCmd_e in public header

typedef enum {
    // not commands, but keep naming consistent with BLHeli suite
    cmd_Remote_Escape       = 0x2E,  // '.'
    cmd_Local_Escape        = 0x2F,  // '/'
} syn_4way_e;


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
uint16_t _crc_xmodem_update (uint16_t crc, uint8_t data)
{
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

// Try connecting to device
// 3 attempts are made, trying both STK and BL protocols.
static uint8_t connect(escDeviceInfo_t *pDeviceInfo)
{
    for (int try = 0; try < 3; try++) {
#if defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
        if (Stk_ConnectEx(pDeviceInfo) && signatureMatch(pDeviceInfo->signature, signaturesAtmel)) {
            currentInterfaceMode = imSK;
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

static uint8_t readByte(void)
{
    // need timeout?
    while (!serialRxBytesWaiting(port));
    return serialRead(port);
}

static uint8_t readByteCrc(void)
{
    uint8_t b = readByte();
    crcIn = _crc_xmodem_update(crcIn, b);
    return b;
}

static void writeByte(uint8_t b)
{
    serialWrite(port, b);
}

static void writeByteCrc(uint8_t b)
{
    writeByte(b);
    crcOut = _crc_xmodem_update(crcOut, b);
}

// handle 4way interface on serial port
// esc4wayStart / esc4wayRelease in called internally
// 256 bytes buffer is allocated on stack
void esc4wayProcess(serialPort_t *serial)
{
    uint8_t command;
    uint16_t addr;
    int inLen;
    int outLen;
    uint8_t paramBuf[256];
    uint8_t replyAck;

    esc4wayStart();

    port = serial;

#ifdef BEEPER
    // fix for buzzer often starts beeping continuously when the ESCs are read
    // switch beeper silent here
    beeperSilence();
#endif

    esc4wayExitRequested = false;
    while(!esc4wayExitRequested) {
        // restart looking for new sequence from host
        crcIn = 0;
        uint8_t esc = readByteCrc();
        if(esc != cmd_Local_Escape)
            continue;                          // wait for sync character

        RX_LED_ON;

        command = readByteCrc();
        addr    = readByteCrc() << 8;
        addr   |= readByteCrc();

        inLen   = readByteCrc();
        if(inLen == 0)
            inLen = 0x100;                     // len ==0 -> param is 256B

        for(int i = 0; i < inLen; i++)
            paramBuf[i] = readByteCrc();

        readByteCrc(); readByteCrc();         // update input CRC
        RX_LED_OFF;

        outLen = 0;                           // output handling code will send single zero byte if necessary
        replyAck = esc4wayAck_OK;

        if(crcIn != 0)                        // CRC of correct message == 0
            replyAck = esc4wayAck_I_INVALID_CRC;

        if (replyAck == esc4wayAck_OK)
            replyAck = esc4wayProcessCmd(command, addr, paramBuf, inLen, &outLen);

        // send single '\0' byte is output when length is zero (len ==0 -> 256 bytes)
        if(!outLen) {
            paramBuf[0] = 0;
            outLen = 1;
        }

        crcOut = 0;
        TX_LED_ON;
        serialBeginWrite(port);
        writeByteCrc(cmd_Remote_Escape);
        writeByteCrc(command);
        writeByteCrc(addr >> 8);
        writeByteCrc(addr & 0xff);
        writeByteCrc(outLen & 0xff);          // only low byte is send, 0x00 -> 256B
        for(int i = 0; i < outLen; i++)
            writeByteCrc(paramBuf[i]);
        writeByteCrc(replyAck);
        writeByte(crcOut >> 8);
        writeByte(crcOut & 0xff);
        serialEndWrite(port);
        TX_LED_OFF;
    }

    esc4wayRelease();
}

// handle 4Way interface command
// command - received command, will be sent back in reply
// addr - from received header
// data - buffer used both for received parameters and returned data.
//   Should be 256B long ; TODO - implement limited buffer size
// inLen - received input length
// outLen - size of data to return, max 256, initialized to zero
//          single '\0' byte will be send if outLen is zero (protocol limitation)
esc4wayAck_e esc4wayProcessCmd(esc4wayCmd_e command, uint16_t addr, uint8_t *data, int inLen, int *outLen)
{
    ioMem_t ioMem;
    ioMem.addr = addr;                     // default flash operation address
    ioMem.data = data;                     // command data buffer is used for read and write commands

    switch(command) {
// ******* Interface related stuff *******
        case cmd_InterfaceTestAlive:
            if (!isMcuConnected())
                return esc4wayAck_OK;

            switch(currentInterfaceMode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                case imATM_BLB:
                case imSIL_BLB:
                    if (BL_SendCMDKeepAlive())
                        return esc4wayAck_OK;
                    break;
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                case imSK:
                    if (Stk_SignOn())
                        return esc4wayAck_OK;
                    break;
#endif
            }
            setDisconnected();
            return esc4wayAck_D_GENERAL_ERROR;

        case cmd_ProtocolGetVersion:
            // Only interface itself, no matter what Device
            data[0] = SERIAL_4WAY_PROTOCOL_VER;
            *outLen = 1;
            return esc4wayAck_OK;

        case cmd_InterfaceGetName:
            // Only interface itself, no matter what Device
            // outLen=16;
            memcpy(data, SERIAL_4WAY_INTERFACE_NAME_STR, strlen(SERIAL_4WAY_INTERFACE_NAME_STR));
            *outLen = strlen(SERIAL_4WAY_INTERFACE_NAME_STR);
            return esc4wayAck_OK;

        case cmd_InterfaceGetVersion:
            // Only interface itself, no matter what Device
            data[0] = SERIAL_4WAY_VERSION_HI;
            data[1] = SERIAL_4WAY_VERSION_LO;
            *outLen = 2;
            return esc4wayAck_OK;

        case cmd_InterfaceExit:
            esc4wayExitRequested = true;
            return esc4wayAck_OK;

        case cmd_InterfaceSetMode:
            switch(data[0]) {
#if defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER)
                case imSIL_BLB:
                case imATM_BLB:
#endif
#if defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
                case imSK:
#endif
                    currentInterfaceMode = data[0];
                    break;
                default:
                    return esc4wayAck_I_INVALID_PARAM;
            }
            return esc4wayAck_OK;

        case cmd_DeviceReset:
            if(data[0] >= escCount)
                return esc4wayAck_I_INVALID_CHANNEL;
            // Channel may change here
            escSelected = data[0];
            switch (currentInterfaceMode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                case imSIL_BLB:
                case imATM_BLB:
                    BL_SendCMDRunRestartBootloader();
                    break;
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                case imSK:
                    break;
#endif
            }
            setDisconnected();
            return esc4wayAck_OK;

        case cmd_DeviceInitFlash: {
            setDisconnected();
            if (data[0] >= escCount)
                return  esc4wayAck_I_INVALID_CHANNEL;
            //Channel may change here
            //ESC_LO or ESC_HI; Halt state for prev channel
            int replyAck = esc4wayAck_OK;
            escSelected = data[0];
            if(!connect(&deviceInfo)) {
                setDisconnected();
                replyAck = esc4wayAck_D_GENERAL_ERROR;
            }
            deviceInfo.interfaceMode = currentInterfaceMode;
            memcpy(data, &deviceInfo, sizeof(deviceInfo));
            *outLen = sizeof(deviceInfo);
            return replyAck;
        }

#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
        case cmd_DeviceEraseAll:
            switch(currentInterfaceMode) {
                case imSK:
                    if (!Stk_Chip_Erase())
                        return esc4wayAck_D_GENERAL_ERROR;
                    break;
                default:
                    return esc4wayAck_I_INVALID_CMD;
            }
            return esc4wayAck_OK;
#endif

#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
        case cmd_DevicePageErase:
            switch (currentInterfaceMode) {
                case imSIL_BLB:
                    *outLen = 1;  // return block number (from incoming packet)
                    // Address = Page * 512
                    ioMem.addr = data[0] << 9;
                    if (!BL_PageErase(&ioMem))
                        return esc4wayAck_D_GENERAL_ERROR;
                    break;
                default:
                    return esc4wayAck_I_INVALID_CMD;
            }
            return esc4wayAck_OK;
#endif

//*** Device Memory Read Ops ***

// macros to mix interface with (single bit) memory type for switch statement
#define M_FLASH 0
#define M_EEPROM 1
#define INTFMEM(interface, memory) (((interface) << 1) | (memory))

        case cmd_DeviceReadEEprom:
        case cmd_DeviceRead: {
            int len = data[0];
            if(len == 0)
                len = 0x100;
            ioMem.len = len;
            switch(INTFMEM(currentInterfaceMode, (command == cmd_DeviceRead) ? M_FLASH : M_EEPROM)) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                case INTFMEM(imSIL_BLB, M_FLASH):
                    if(!BL_ReadFlashSIL(&ioMem))
                        return esc4wayAck_D_GENERAL_ERROR;
                    break;
                case INTFMEM(imATM_BLB, M_FLASH):
                    if(!BL_ReadFlashATM(&ioMem))
                        return esc4wayAck_D_GENERAL_ERROR;
                    break;
                case INTFMEM(imATM_BLB, M_EEPROM):
                    if(!BL_ReadEEprom(&ioMem))
                        return esc4wayAck_D_GENERAL_ERROR;
                    break;
                // INTFMEM(imSIL_BLB, M_EEPROM): no eeprom on Silabs
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                case INTFMEM(imSK, M_FLASH):
                    if(!Stk_ReadFlash(&ioMem))
                        return esc4wayAck_D_GENERAL_ERROR;
                    break;
                case INTFMEM(imSK, M_EEPROM):
                    if (!Stk_ReadEEprom(&ioMem))
                        return esc4wayAck_D_GENERAL_ERROR;
                    break;
#endif
                default:
                    return esc4wayAck_I_INVALID_CMD;
            }
            *outLen = ioMem.len;
            return esc4wayAck_OK;
        }

//*** Device Memory Write Ops ***
        case cmd_DeviceWrite:
        case cmd_DeviceWriteEEprom:
            ioMem.len = inLen;
            switch (INTFMEM(currentInterfaceMode, (command == cmd_DeviceWrite) ? M_FLASH : M_EEPROM)) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
                case INTFMEM(imSIL_BLB, M_FLASH):
                case INTFMEM(imATM_BLB, M_FLASH):
                    if (!BL_WriteFlash(&ioMem))
                        return esc4wayAck_D_GENERAL_ERROR;
                    break;
                case INTFMEM(imATM_BLB, M_EEPROM):
                    if (!BL_WriteEEprom(&ioMem))
                        return esc4wayAck_D_GENERAL_ERROR;
                    break;
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
                case INTFMEM(imSK, M_FLASH):
                    if (!Stk_WriteFlash(&ioMem))
                        return esc4wayAck_D_GENERAL_ERROR;
                    break;
                case INTFMEM(imSK, M_EEPROM):
                    if (!Stk_WriteEEprom(&ioMem))
                        return esc4wayAck_D_GENERAL_ERROR;
                    break;
#endif
                default:
                    return esc4wayAck_I_INVALID_CMD;
            }
            return esc4wayAck_OK;
#undef M_FLASH
#undef M_EEPROM
#undef INTFMEM
        default:
            return esc4wayAck_I_INVALID_CMD;
    }
    // should not get here
}

#endif
