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

#include "common/utils.h"
#include "config/parameter_group.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/buf_writer.h"
#include "drivers/pwm_mapping.h"
#include "drivers/gpio.h"
#include "io/serial.h"
//#include "io/serial_msp.h"
#include "io/serial_4way.h"
#include "io/serial_4way_impl.h"
#include "io/serial_4way_avrootloader.h"

#if defined(USE_SERIAL_4WAY_BLHELI_INTERFACE) && defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER)

// Bootloader commands
// RunCmd
#define RestartBootloader 0
#define ExitBootloader    1

#define CMD_RUN            0x00
#define CMD_PROG_FLASH     0x01
#define CMD_ERASE_FLASH    0x02
#define CMD_READ_FLASH_SIL 0x03
#define CMD_VERIFY_FLASH   0x03
#define CMD_READ_EEPROM    0x04
#define CMD_PROG_EEPROM    0x05
#define CMD_READ_SRAM      0x06
#define CMD_READ_FLASH_ATM 0x07
#define CMD_KEEP_ALIVE     0xFD
#define CMD_SET_ADDRESS    0xFF
#define CMD_SET_BUFFER     0xFE

#define CMD_BOOTINIT       0x07
#define CMD_BOOTSIGN       0x08

// Bootloader result codes

#define BR_SUCCESS          0x30
#define BR_ERRORCOMMAND     0xC1
#define BR_ERRORCRC         0xC2
#define BR_NONE             0xFF

#define START_BIT_TIMEOUT 2000                     // 2ms

#define BIT_TIME          52                       // 52uS
#define BIT_TIME_HALVE    (BIT_TIME >> 1)          // 26uS
#define BIT_TIME_3_4      (BIT_TIME_HALVE + (BIT_TIME_HALVE >> 1))   // 39uS
#define START_BIT_TIME    (BIT_TIME_3_4)

static int suart_getc(void)
{
    uint32_t btime;
    uint32_t start_time;

    uint32_t wait_time = micros() + START_BIT_TIMEOUT;
    while (ESC_IS_HI) {
        // check for startbit begin
        if (micros() >= wait_time) {
            return -1;
        }
    }
    // start bit
    start_time = micros();
    btime = start_time + START_BIT_TIME;
    uint16_t bitmask = 0;
    for(int bit = 0; bit < 10; bit++) {
        while (cmp32(micros(), btime) < 0);
        if (ESC_IS_HI)
            bitmask |= (1 << bit);
        btime = btime + BIT_TIME;
    }
    // check start bit and stop bit
    if ((bitmask & (1 << 0)) || (!(bitmask & (1 << 9)))) {
        return -1;
    }
    return bitmask >> 1;
}

static void suart_putc(uint8_t byte)
{
    // send one idle bit first (stopbit from previous byte)
    uint16_t bitmask = (byte << 2) | (1  << 0) | (1 << 10);
    uint32_t btime = micros();
    while(1) {
        if(bitmask & 1)
            ESC_SET_HI; // 1
        else
            ESC_SET_LO; // 0
        btime = btime + BIT_TIME;
        bitmask >>= 1;
        if (bitmask == 0)
            break; // stopbit shifted out - but don't wait
        while (cmp32(micros(), btime) < 0);
    }
}

static uint16_t crc16Byte(uint16_t from, uint8_t byte)
{
    uint16_t crc16 = from;
    for (int i = 0; i < 8; i++) {
        if (((byte & 0x01) ^ (crc16 & 0x0001)) != 0) {
            crc16 >>= 1;
            crc16 ^= 0xA001;
        } else {
            crc16 >>= 1;
        }
        byte >>= 1;
    }
    return crc16;
}

static uint8_t BL_ReadBuf(uint8_t *pstring, int len, bool checkCrc)
{
    int crc = 0;
    int c;

    uint8_t  lastACK = BR_NONE;
    for(int i = 0; i < len; i++) {
        int c;
        if ((c = suart_getc()) < 0) goto timeout;
        crc = crc16Byte(crc, c);
        pstring[i] = c;
    }

    if(checkCrc) {
        // With CRC read 3 more
        for(int i = 0; i < 2; i++) {  // checksum 2 CRC bytes
            if ((c = suart_getc()) < 0) goto timeout;
            crc = crc16Byte(crc, c);
        }
        if((c = suart_getc()) < 0) goto timeout;
        lastACK = c;
        if (crc != 0)   // CRC of correct message is 0
            lastACK = BR_ERRORCRC;
    } else {
        if((c = suart_getc()) < 0) goto timeout;
        lastACK = c;
    }
timeout:
    return (lastACK == BR_SUCCESS);
}

static void BL_SendBuf(uint8_t *pstring, int len, bool appendCrc)
{
    ESC_OUTPUT;
    uint16_t crc = 0;
    for(int i = 0; i < len; i++) {
        suart_putc(pstring[i]);
        crc = crc16Byte(crc, pstring[i]);
    }
    if (appendCrc) {
        suart_putc(crc & 0xff);
        suart_putc(crc >> 8);
    }
    ESC_INPUT;
}

uint8_t BL_ConnectEx(escDeviceInfo_t *pDeviceInfo)
{
#define BOOT_MSG_LEN 4
#define DevSignHi (BOOT_MSG_LEN)
#define DevSignLo (BOOT_MSG_LEN + 1)

    memset(pDeviceInfo, 0, sizeof(*pDeviceInfo));
    uint8_t bootInfo[BOOT_MSG_LEN + 4];
    static const uint8_t bootMsgCheck[BOOT_MSG_LEN - 1] = "471";
    // x * 0 + 9
#if defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
    // SK message was sent during autodetection, use longer preamble
    uint8_t bootInit[] = {0,0,0,0,0,0,0,0,0,0,0,0,0x0D,'B','L','H','e','l','i',0xF4,0x7D};
#else
    uint8_t bootInit[] = {        0,0,0,0,0,0,0,0,0x0D,'B','L','H','e','l','i',0xF4,0x7D};
#endif
    BL_SendBuf(bootInit, sizeof(bootInit), false);
    if (!BL_ReadBuf(bootInfo, sizeof(bootInfo), false))
        return 0;
    // BootInfo has no CRC  (ACK byte already analyzed... )
    // Format = BootMsg("471c") SIGNATURE_001, SIGNATURE_002, BootVersion (always 6), BootPages (,ACK)
    if(memcmp(bootInfo, bootMsgCheck, sizeof(bootMsgCheck)) != 0) // Check only the first 3 letters -> 471x OK
        return 0;

    pDeviceInfo->signature2 = bootInfo[BOOT_MSG_LEN - 1]; // taken from bootloaderMsg part, ascii 'c' now
    pDeviceInfo->signature = (bootInfo[DevSignHi] << 8) | bootInfo[DevSignLo]; // SIGNATURE_001, SIGNATURE_002
    return 1;
}

static uint8_t BL_GetACK(int timeout)
{
    int c;
    while ((c = suart_getc()) < 0)
        if(--timeout < 0)    // timeout=1 -> 1 retry
            return BR_NONE;
    return c;
}

uint8_t BL_SendCMDKeepAlive(void)
{
    uint8_t sCMD[] = {CMD_KEEP_ALIVE, 0};
    BL_SendBuf(sCMD, sizeof(sCMD), true);
    if (BL_GetACK(1) != BR_ERRORCOMMAND)
        return 0;
    return 1;
}

void BL_SendCMDRunRestartBootloader(void)
{
    uint8_t sCMD[] = {RestartBootloader, 0};
    BL_SendBuf(sCMD, sizeof(sCMD), true); // sends simply 4 x 0x00 (CRC = 00)
    return;
}

static uint8_t BL_SendCMDSetAddress(ioMem_t *pMem) //supports only 16 bit Adr
{
    // skip if adr == 0xFFFF
    if((pMem->addr == 0xffff))
        return 1;
    uint8_t sCMD[] = {CMD_SET_ADDRESS, 0, pMem->addr >> 8, pMem->addr & 0xff };
    BL_SendBuf(sCMD, sizeof(sCMD), true);
    return BL_GetACK(2) == BR_SUCCESS;
}

static uint8_t BL_SendCMDSetBuffer(ioMem_t *pMem)
{
    uint16_t len = pMem->len;
    uint8_t sCMD[] = {CMD_SET_BUFFER, 0, len >> 8, len & 0xff};
    BL_SendBuf(sCMD, sizeof(sCMD), true);
    if (BL_GetACK(2) != BR_NONE)
        return 0;
    BL_SendBuf(pMem->data, len, true);
    return BL_GetACK(40) == BR_SUCCESS;
}

static uint8_t BL_ReadA(uint8_t cmd, ioMem_t *pMem)
{
    if(!BL_SendCMDSetAddress(pMem))
        return 0;
    unsigned len = pMem->len;
    uint8_t sCMD[] = {cmd, len & 0xff};    // 0x100 is sent a 0x00 here
    BL_SendBuf(sCMD, sizeof(sCMD), true);
    return BL_ReadBuf(pMem->data, len, true);
}

static uint8_t BL_WriteA(uint8_t cmd, ioMem_t *pMem, uint32_t timeout)
{
    if(!BL_SendCMDSetAddress(pMem))
        return 0;
    if (!BL_SendCMDSetBuffer(pMem))
        return 0;
    uint8_t sCMD[] = {cmd, 0x01};
    BL_SendBuf(sCMD, sizeof(sCMD), true);
    return BL_GetACK(timeout) == BR_SUCCESS;
}


uint8_t BL_ReadFlashATM(ioMem_t *pMem)
{
    return BL_ReadA(CMD_READ_FLASH_ATM, pMem);
}

uint8_t BL_ReadFlashSIL(ioMem_t *pMem)
{
    return BL_ReadA(CMD_READ_FLASH_SIL, pMem);
}


uint8_t BL_ReadEEprom(ioMem_t *pMem)
{
    return BL_ReadA(CMD_READ_EEPROM, pMem);
}

uint8_t BL_PageErase(ioMem_t *pMem)
{
    if(!BL_SendCMDSetAddress(pMem))
        return 0;

    uint8_t sCMD[] = {CMD_ERASE_FLASH, 0x01};
    BL_SendBuf(sCMD, sizeof(sCMD), true);
    return BL_GetACK(40 * 1000 / START_BIT_TIMEOUT) == BR_SUCCESS;
}

uint8_t BL_WriteEEprom(ioMem_t *pMem)
{
    return BL_WriteA(CMD_PROG_EEPROM, pMem, 3000 * 1000 / START_BIT_TIMEOUT);
}

uint8_t BL_WriteFlash(ioMem_t *pMem)
{
    return BL_WriteA(CMD_PROG_FLASH, pMem, 40 * 1000 / START_BIT_TIMEOUT);
}

#endif
