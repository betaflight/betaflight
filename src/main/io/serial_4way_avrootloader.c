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

#ifdef  USE_SERIAL_4WAY_BLHELI_INTERFACE
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/buf_writer.h"
#include "drivers/pwm_mapping.h"
#include "drivers/gpio.h"
#include "io/serial.h"
#include "io/serial_msp.h"
#include "io/serial_4way.h"
#include "io/serial_4way_avrootloader.h"
#ifdef  USE_SERIAL_4WAY_BLHELI_BOOTLOADER


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


#define START_BIT_TIMEOUT_MS 2

#define BIT_TIME (52)  //52uS
#define BIT_TIME_HALVE (BIT_TIME >> 1) //26uS
#define START_BIT_TIME (BIT_TIME_HALVE + 1)
//#define STOP_BIT_TIME ((BIT_TIME * 9) + BIT_TIME_HALVE)

static uint8_t suart_getc_(uint8_t *bt)
{
    uint32_t btime;
    uint32_t start_time;

    uint32_t wait_time = millis() + START_BIT_TIMEOUT_MS;
    while (ESC_IS_HI) {
        // check for startbit begin
        if (millis() >= wait_time) {
            return 0;
        }
    }
    // start bit
    start_time = micros();
    btime = start_time + START_BIT_TIME;
    uint16_t bitmask = 0;
    uint8_t bit = 0;
    while (micros() < btime);
    while(1) {
        if (ESC_IS_HI)
        {
            bitmask |= (1 << bit);
        }
        btime = btime + BIT_TIME;
        bit++;
        if (bit == 10) break;
        while (micros() < btime);
    }
    // check start bit and stop bit
    if ((bitmask & 1) || (!(bitmask & (1 << 9)))) {
        return 0;
    }
    *bt = bitmask >> 1;
    return 1;
}

static void suart_putc_(uint8_t *tx_b)
{
    // shift out stopbit first
    uint16_t bitmask = (*tx_b << 2) | 1 | (1 << 10);
    uint32_t btime = micros();
    while(1) {
        if(bitmask & 1) {
            ESC_SET_HI; // 1
        }
        else {
            ESC_SET_LO; // 0
        }
        btime = btime + BIT_TIME;
        bitmask = (bitmask >> 1);
        if (bitmask == 0) break; // stopbit shifted out - but don't wait
        while (micros() < btime);
    }
}

static uint8_16_u CRC_16;
static uint8_16_u LastCRC_16;

static void ByteCrc(uint8_t *bt)
{
    uint8_t xb = *bt;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (((xb & 0x01) ^ (CRC_16.word & 0x0001)) !=0 ) {
            CRC_16.word = CRC_16.word >> 1;
            CRC_16.word = CRC_16.word ^ 0xA001;
        } else {
            CRC_16.word = CRC_16.word >> 1;
        }
        xb = xb >> 1;
    }
}

static uint8_t BL_ReadBuf(uint8_t *pstring, uint8_t len)
{
    // len 0 means 256
    CRC_16.word = 0;
    LastCRC_16.word = 0;
    uint8_t  LastACK = brNONE;
    do {
        if(!suart_getc_(pstring)) goto timeout;
        ByteCrc(pstring);
        pstring++;
        len--;
    } while(len > 0);

    if(isMcuConnected()) {
        //With CRC read 3 more
        if(!suart_getc_(&LastCRC_16.bytes[0])) goto timeout;
        if(!suart_getc_(&LastCRC_16.bytes[1])) goto timeout;
        if(!suart_getc_(&LastACK)) goto timeout;
        if (CRC_16.word != LastCRC_16.word) {
            LastACK = brERRORCRC;
        }
    } else {
        if(!suart_getc_(&LastACK)) goto timeout;
    }
timeout:
    return (LastACK == brSUCCESS);
}

static void BL_SendBuf(uint8_t *pstring, uint8_t len)
{
    ESC_OUTPUT;
    CRC_16.word=0;
    do {
        suart_putc_(pstring);
        ByteCrc(pstring);
        pstring++;
        len--;
    } while (len > 0);
    
    if (isMcuConnected()) {
        suart_putc_(&CRC_16.bytes[0]);
        suart_putc_(&CRC_16.bytes[1]);
    }
    ESC_INPUT;
}

uint8_t BL_ConnectEx(uint8_32_u *pDeviceInfo)
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
    pDeviceInfo->bytes[2] = BootInfo[BootMsgLen - 1];
    pDeviceInfo->bytes[1] = BootInfo[DevSignHi];
    pDeviceInfo->bytes[0] = BootInfo[DevSignLo];
    return (1);
}

static uint8_t BL_GetACK(uint32_t Timeout)
{
    uint8_t LastACK = brNONE;
    while (!(suart_getc_(&LastACK)) && (Timeout)) {
        Timeout--;
    } ;
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

void BL_SendCMDRunRestartBootloader(uint8_32_u *pDeviceInfo)
{
    uint8_t sCMD[] = {RestartBootloader, 0};
    pDeviceInfo->bytes[0] = 1;
    BL_SendBuf(sCMD, 2); //sends simply 4 x 0x00 (CRC =00)
    return;
}

static uint8_t BL_SendCMDSetAddress(ioMem_t *pMem) //supports only 16 bit Adr
{
    // skip if adr == 0xFFFF
    if((pMem->D_FLASH_ADDR_H == 0xFF) && (pMem->D_FLASH_ADDR_L == 0xFF)) return 1;
    uint8_t sCMD[] = {CMD_SET_ADDRESS, 0, pMem->D_FLASH_ADDR_H, pMem->D_FLASH_ADDR_L };
    BL_SendBuf(sCMD, 4);
    return (BL_GetACK(2) == brSUCCESS);
}

static uint8_t BL_SendCMDSetBuffer(ioMem_t *pMem)
{
    uint8_t sCMD[] = {CMD_SET_BUFFER, 0, 0, pMem->D_NUM_BYTES};
    if (pMem->D_NUM_BYTES == 0) {
        // set high byte
        sCMD[2] = 1;
    }
    BL_SendBuf(sCMD, 4);
    if (BL_GetACK(2) != brNONE) return 0;
    BL_SendBuf(pMem->D_PTR_I, pMem->D_NUM_BYTES);
    return (BL_GetACK(40) == brSUCCESS);
}

static uint8_t BL_ReadA(uint8_t cmd, ioMem_t *pMem)
{
    if (BL_SendCMDSetAddress(pMem)) {
        uint8_t sCMD[] = {cmd, pMem->D_NUM_BYTES};
        BL_SendBuf(sCMD, 2);
        return (BL_ReadBuf(pMem->D_PTR_I, pMem->D_NUM_BYTES ));
    }
    return 0;
}

static uint8_t BL_WriteA(uint8_t cmd, ioMem_t *pMem, uint32_t timeout)
{
    if (BL_SendCMDSetAddress(pMem)) {
        if (!BL_SendCMDSetBuffer(pMem)) return 0;
        uint8_t sCMD[] = {cmd, 0x01};
        BL_SendBuf(sCMD, 2);
        return (BL_GetACK(timeout) == brSUCCESS);
    }
    return 0;
}


uint8_t BL_ReadFlash(uint8_t interface_mode, ioMem_t *pMem)
{
    if(interface_mode == imATM_BLB) {
        return BL_ReadA(CMD_READ_FLASH_ATM, pMem);
    } else {
        return BL_ReadA(CMD_READ_FLASH_SIL, pMem);
    }
}
 
 
uint8_t BL_ReadEEprom(ioMem_t *pMem)
{
    return BL_ReadA(CMD_READ_EEPROM, pMem);
}

uint8_t BL_PageErase(ioMem_t *pMem)
{
    if (BL_SendCMDSetAddress(pMem)) {
        uint8_t sCMD[] = {CMD_ERASE_FLASH, 0x01};
        BL_SendBuf(sCMD, 2);
        return (BL_GetACK((40 / START_BIT_TIMEOUT_MS)) == brSUCCESS);
    }
    return 0;
}

uint8_t BL_WriteEEprom(ioMem_t *pMem)
{
    return BL_WriteA(CMD_PROG_EEPROM, pMem, (3000 / START_BIT_TIMEOUT_MS));
}

uint8_t BL_WriteFlash(ioMem_t *pMem)
{
    return BL_WriteA(CMD_PROG_FLASH, pMem, (40 / START_BIT_TIMEOUT_MS));
}

#endif
#endif
