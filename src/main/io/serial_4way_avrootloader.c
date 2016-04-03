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
 * for info about Hagens AVRootloader:
 * http://www.mikrocontroller.net/topic/avr-bootloader-mit-verschluesselung
*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <platform.h>

#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#include "drivers/system.h"
#include "io/serial_4way_avrootloader.h"
#include "io/serial_4way.h"

// Bootloader commands
// RunCmd
#define RestartBootloader 0
#define ExitBootloader 1

#define CMD_RUN           0x00
#define CMD_PROG_FLASH    0x01
#define CMD_ERASE_FLASH   0x02
#define CMD_READ_FLASH_SIL 0x03
#define CMD_VERIFY_FLASH  0x03
#define CMD_READ_EEPROM   0x04
#define CMD_PROG_EEPROM   0x05
#define CMD_READ_SRAM     0x06
#define CMD_READ_FLASH_ATM 0x07
#define CMD_KEEP_ALIVE    0xFD
#define CMD_SET_ADDRESS   0xFF
#define CMD_SET_BUFFER    0xFE

#define CMD_BOOTINIT      0x07
#define CMD_BOOTSIGN      0x08

// Bootloader result codes

#define brSUCCESS         0x30
#define brERRORCOMMAND    0xC1
#define brERRORCRC        0xC2
#define brNONE            0xFF

static union uint8_16u CRC_16;

static uint8_t cb;
static uint8_t suart_timeout;

#define WaitStartBitTimeoutms 2

#define BitTime (52)  //52uS
#define BitHalfTime (BitTime >> 1) //26uS
#define StartBitTime (BitHalfTime + 1)
#define StopBitTime ((BitTime * 9) + BitHalfTime)



static uint8_t suart_getc_(void)
{
    uint8_t bt=0;
    uint32_t btime;
    uint32_t bstop;
    uint32_t start_time;

    suart_timeout = 1;
    uint32_t wait_time = millis() + WaitStartBitTimeoutms;
    while (ESC_IS_HI) {
        // check for Startbit begin
        if (millis() >= wait_time) {
            return 0;
        }
    }
    // Startbit
    start_time = micros();

    btime = start_time + StartBitTime;
    bstop= start_time + StopBitTime;

    while (micros() < btime);
    if (ESC_IS_HI) {
        return 0;
    }
    for (uint8_t bit = 0; bit < 8; bit++)
    {
        btime = btime + BitTime;
        while (micros() < btime);
        if (ESC_IS_HI)
        {
             bt |= (1 << bit);
        }
    }
    while (micros() < bstop);
    // check Stoppbit
    if (ESC_IS_LO) {
        return 0;
    }
    suart_timeout = 0;
    return (bt);
}

static void suart_putc_(uint8_t TXbyte)
{
    uint32_t btime;
    ESC_SET_LO; // Set low = StartBit
    btime = BitTime + micros();
    while (micros() < btime);
    for(uint8_t bit = 0; bit < 8; bit++)
    {
        if(TXbyte & 1)
        {
            ESC_SET_HI; // 1
        }
        else
        {
            ESC_SET_LO; // 0
        }
        btime = btime + BitTime;
        TXbyte = (TXbyte >> 1);
        while (micros() < btime);
    }
    ESC_SET_HI; //Set high = Stoppbit
    btime = btime + BitTime;
    while (micros() < btime);
}

static union uint8_16u LastCRC_16;

static void ByteCrc(void)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (((cb & 0x01) ^ (CRC_16.word & 0x0001)) !=0 )
        {
            CRC_16.word = CRC_16.word >> 1;
            CRC_16.word = CRC_16.word ^ 0xA001;
        }
        else
        {
            CRC_16.word = CRC_16.word >> 1;
        }
        cb = cb >> 1;
    }
}

static uint8_t BL_ReadBuf(uint8_t *pstring, uint8_t len)
{
    //Todo CRC in case of timeout?
    //len 0 means 256
    CRC_16.word = 0;
    LastCRC_16.word = 0;
    uint8_t  LastACK = brNONE;
    do {
        cb = suart_getc_();
        if(suart_timeout) goto timeout;
        *pstring = cb;
        ByteCrc();
        pstring++;
        len--;
    } while(len > 0);

    if(IsMcuConnected) {
        //With CRC read 3 more
        LastCRC_16.bytes[0] = suart_getc_();
        if(suart_timeout) goto timeout;
        LastCRC_16.bytes[1] = suart_getc_();
        if(suart_timeout) goto timeout;
        LastACK = suart_getc_();
        if (CRC_16.word != LastCRC_16.word) {
            LastACK = brERRORCRC;
        }
    } else {
        //TODO check here LastACK
        LastACK = suart_getc_();
    }
timeout:
    return (LastACK == brSUCCESS);
}

static void BL_SendBuf(uint8_t *pstring, uint8_t len)
{
    ESC_OUTPUT;
    // wait some us
    delayMicroseconds(50);
    CRC_16.word=0;
    do {
        cb = *pstring;
        pstring++;
        suart_putc_(cb);
        ByteCrc();
        len--;
    } while (len > 0);
    
    if (IsMcuConnected) {
        suart_putc_(CRC_16.bytes[0]);
        suart_putc_(CRC_16.bytes[1]);
    }
    ESC_INPUT;
}

uint8_t BL_ConnectEx(void)
{
    #define BootMsgLen 4
    #define DevSignHi (BootMsgLen)
    #define DevSignLo (BootMsgLen+1)

    //DeviceInfo.dword=0; is set before
    uint8_t BootInfo[9];
    uint8_t BootMsg[BootMsgLen-1] = "471";
    // x * 0 + 9
#if defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
    uint8_t BootInit[] = {0,0,0,0,0,0,0,0,0,0,0,0,0x0D,'B','L','H','e','l','i',0xF4,0x7D};
    BL_SendBuf(BootInit, 21);
#else
    uint8_t BootInit[] = {0,0,0,0,0,0,0,0,0x0D,'B','L','H','e','l','i',0xF4,0x7D};
    BL_SendBuf(BootInit, 17);
#endif
    if (!BL_ReadBuf(BootInfo, BootMsgLen + 4)) {
        return 0;
    }
    // BootInfo has no CRC  (ACK byte already analyzed... )
    // Format = BootMsg("471c") SIGNATURE_001, SIGNATURE_002, BootVersion (always 6), BootPages (,ACK)
    for (uint8_t i = 0; i < (BootMsgLen - 1); i++) { // Check only the first 3 letters -> 471x OK
        if (BootInfo[i] != BootMsg[i]) {
            return (0);
        }
    }

    //only 2 bytes used $1E9307 -> 0x9307
    DeviceInfo.bytes[2] = BootInfo[BootMsgLen - 1];
    DeviceInfo.bytes[1] = BootInfo[DevSignHi];
    DeviceInfo.bytes[0] = BootInfo[DevSignLo];
    return (1);
}

static uint8_t BL_GetACK(uint32_t Timeout)
{
    uint8_t LastACK;
    do {
        LastACK = suart_getc_();
        Timeout--;
    } while ((suart_timeout) && (Timeout));

    if(suart_timeout) {
        LastACK = brNONE;
    }
    return (LastACK);
}


uint8_t BL_SendCMDKeepAlive(void) 
{
    uint8_t sCMD[] = {CMD_KEEP_ALIVE, 0};
    BL_SendBuf(sCMD, 2);
    if (BL_GetACK(1) != brERRORCOMMAND) {
        return 0;
    }
    return 1;
}

void BL_SendCMDRunRestartBootloader(void)
{
    uint8_t sCMD[] = {RestartBootloader, 0};
    DeviceInfo.bytes[0] = 1;
    BL_SendBuf(sCMD, 2); //sends simply 4 x 0x00 (CRC =00)
    return;
}

static uint8_t BL_SendCMDSetAddress(void) //supports only 16 bit Adr
{
    // skip if adr == 0xFFFF
    if((D_FLASH_ADDR_H == 0xFF) && (D_FLASH_ADDR_L == 0xFF)) return 1;
    uint8_t sCMD[] = {CMD_SET_ADDRESS, 0, D_FLASH_ADDR_H, D_FLASH_ADDR_L };
    BL_SendBuf(sCMD, 4);
    return (BL_GetACK(2) == brSUCCESS);
}

static uint8_t BL_SendCMDSetBuffer(void)
{
    uint8_t sCMD[] = {CMD_SET_BUFFER, 0, 0, D_NUM_BYTES};
    if (D_NUM_BYTES == 0) {
        // set high byte
        sCMD[2] = 1;
    }
    BL_SendBuf(sCMD, 4);
    if (BL_GetACK(2) != brNONE) return 0;
    BL_SendBuf(D_PTR_I, D_NUM_BYTES);
    return (BL_GetACK(40) == brSUCCESS);
}

static uint8_t BL_ReadA(uint8_t cmd)
{
    if (BL_SendCMDSetAddress()) {
        uint8_t sCMD[] = {cmd, D_NUM_BYTES};
        BL_SendBuf(sCMD, 2);
        return (BL_ReadBuf(D_PTR_I, D_NUM_BYTES ));
    }
    return 0;
}

static uint8_t BL_WriteA(uint8_t cmd, uint32_t timeout)
{
    if (BL_SendCMDSetAddress()) {
        if (!BL_SendCMDSetBuffer()) return 0;
        uint8_t sCMD[] = {cmd, 0x01};
        BL_SendBuf(sCMD, 2);
        return (BL_GetACK(timeout) == brSUCCESS);
    }
    return 0;
}


uint8_t BL_ReadFlash(uint8_t interface_mode)
{
    if(interface_mode == imATM_BLB) {
        return BL_ReadA(CMD_READ_FLASH_ATM);
    } else {
        return BL_ReadA(CMD_READ_FLASH_SIL);
    }
}
 
 
uint8_t BL_ReadEEprom(void)
{
    return BL_ReadA(CMD_READ_EEPROM);
}

uint8_t BL_PageErase(void)
{
    if (BL_SendCMDSetAddress()) {
        uint8_t sCMD[] = {CMD_ERASE_FLASH, 0x01};
        BL_SendBuf(sCMD, 2);
        return (BL_GetACK((40 / WaitStartBitTimeoutms)) == brSUCCESS);
    }
    return 0;
}

uint8_t BL_WriteEEprom(void)
{
    return BL_WriteA(CMD_PROG_EEPROM, (3000 / WaitStartBitTimeoutms));
}

uint8_t BL_WriteFlash(void)
{
    return BL_WriteA(CMD_PROG_FLASH, (40 / WaitStartBitTimeoutms));
}

#endif
