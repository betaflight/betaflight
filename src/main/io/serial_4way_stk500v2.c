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
 * have a look at https://github.com/sim-/tgy/blob/master/boot.inc
 * for info about the stk500v2 implementation
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <platform.h>
#include "common/utils.h"
#include "drivers/gpio.h"
#include "drivers/buf_writer.h"
#include "drivers/pwm_mapping.h"
#include "drivers/serial.h"
#include "drivers/system.h"
#include "config/config.h"
#include "io/serial.h"
#include "io/serial_4way.h"
#include "io/serial_4way_impl.h"
#include "io/serial_4way_stk500v2.h"

#if defined(USE_SERIAL_4WAY_BLHELI_INTERFACE) && defined(USE_SERIAL_4WAY_SK_BOOTLOADER)


#define BIT_LO_US 32                               //32uS
#define BIT_HI_US (2 * BIT_LO_US)

static uint8_t stkInBuf[16];

#define STK_BIT_TIMEOUT 250                        // micro seconds
#define STK_WAIT_TICKS (1000 / STK_BIT_TIMEOUT)    // per ms
#define STK_WAITCYLCES (STK_WAIT_TICKS * 35)       // 35ms
#define STK_WAITCYLCES_START (STK_WAIT_TICKS / 2)  // 0.5ms
#define STK_WAITCYLCES_EXT (STK_WAIT_TICKS * 5000) // 5s

#define  WaitPinLo  while (ESC_IS_HI) { if (cmp32(micros(), timeout_timer) > 0) goto timeout; }
#define  WaitPinHi  while (ESC_IS_LO) { if (cmp32(micros(), timeout_timer) > 0) goto timeout; }

static uint32_t lastBitTime;
static uint32_t hiLoTsh;

static uint8_t SeqNumber;
static uint8_t StkCmd;
static uint8_t ckSumIn;
static uint8_t ckSumOut;

// used STK message constants from ATMEL AVR - Application note
#define MESSAGE_START           0x1B
#define TOKEN                   0x0E

#define CMD_SIGN_ON             0x01
#define CMD_LOAD_ADDRESS        0x06
#define CMD_CHIP_ERASE_ISP      0x12
#define CMD_PROGRAM_FLASH_ISP   0x13
#define CMD_READ_FLASH_ISP      0x14
#define CMD_PROGRAM_EEPROM_ISP  0x15
#define CMD_READ_EEPROM_ISP     0x16
#define CMD_READ_SIGNATURE_ISP  0x1B
#define CMD_SPI_MULTI           0x1D

#define STATUS_CMD_OK           0x00

#define CmdFlashEepromRead      0xA0
#define EnterIspCmd1            0xAC
#define EnterIspCmd2            0x53
#define SPI_SIGNATURE_READ      0x30

#define delay_us(x) delayMicroseconds(x)
#define IRQ_OFF // dummy
#define IRQ_ON  // dummy

static void StkSendByte(uint8_t dat)
{
    ckSumOut ^= dat;
    for (uint8_t i = 0; i < 8; i++) {
        if (dat & 0x01) {
            // 1-bits are encoded as 64.0us high, 72.8us low (135.8us total).
            ESC_SET_HI;
            delay_us(BIT_HI_US);
            ESC_SET_LO;
            delay_us(BIT_HI_US);
        } else {
            // 0-bits are encoded as 27.8us high, 34.5us low, 34.4us high, 37.9 low (134.6us total)
            ESC_SET_HI;
            delay_us(BIT_LO_US);
            ESC_SET_LO;
            delay_us(BIT_LO_US);
            ESC_SET_HI;
            delay_us(BIT_LO_US);
            ESC_SET_LO;
            delay_us(BIT_LO_US);
        }
        dat >>= 1;
    }
}

static void StkSendPacketHeader(uint16_t len)
{
    IRQ_OFF;
    ESC_OUTPUT;
    StkSendByte(0xFF);
    StkSendByte(0xFF);
    StkSendByte(0x7F);
    ckSumOut = 0;
    StkSendByte(MESSAGE_START);
    StkSendByte(++SeqNumber);
    StkSendByte(len >> 8);
    StkSendByte(len & 0xff);
    StkSendByte(TOKEN);
}

static void StkSendPacketFooter(void)
{
    StkSendByte(ckSumOut);
    ESC_SET_HI;
    delay_us(BIT_LO_US);
    ESC_INPUT;
    IRQ_ON;
}

static int8_t ReadBit(void)
{
    uint32_t btimer = micros();
    uint32_t timeout_timer = btimer + STK_BIT_TIMEOUT;
    WaitPinLo;
    WaitPinHi;
    lastBitTime = micros() - btimer;
    if (lastBitTime <= hiLoTsh) {
        timeout_timer = timeout_timer + STK_BIT_TIMEOUT;
        WaitPinLo;
        WaitPinHi;
        //lo-bit
        return 0;
    } else {
        return 1;
    }
timeout:
    return -1;
}

static int ReadByte(void)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        int8_t bit = ReadBit();
        if (bit < 0)
            return -1;  // timeout
        byte |= bit << i;
    }
    ckSumIn ^= byte;
    return byte;
}

static uint8_t StkReadLeader(void)
{

    // Reset learned timing
    hiLoTsh = BIT_HI_US + BIT_LO_US;

    // Wait for the first bit
    int waitcycl; //250uS each

    if((StkCmd == CMD_PROGRAM_EEPROM_ISP) || (StkCmd == CMD_CHIP_ERASE_ISP)) {
         waitcycl = STK_WAITCYLCES_EXT;
    } else if(StkCmd == CMD_SIGN_ON) {
        waitcycl = STK_WAITCYLCES_START;
    } else {
        waitcycl= STK_WAITCYLCES;
    }
    while(ReadBit() < 0 && --waitcycl > 0);
    if (waitcycl <= 0)
        goto timeout;

    // Skip the first bits
    for (int i = 0; i < 10; i++) {
        if (ReadBit() < 0)
            goto timeout;
    }

    // learn timing (0.75 * lastBitTime)
    hiLoTsh = (lastBitTime >> 1) + (lastBitTime >> 2);

    // Read until we get a 0 bit
    int bit;
    do {
        bit = ReadBit();
        if (bit < 0)
            goto timeout;
    } while (bit > 0);
    return 1;
timeout:
    return 0;
}

static uint8_t StkRcvPacket(uint8_t *pstring, int maxLen)
{
    int byte;
    int len;

    IRQ_OFF;
    if (!StkReadLeader()) goto Err;
    ckSumIn=0;
    if ((byte = ReadByte()) < 0 || (byte != MESSAGE_START)) goto Err;
    if ((byte = ReadByte()) < 0 || (byte != SeqNumber)) goto Err;
    len = ReadByte() << 8;
    len |= ReadByte();
    if(len < 1 || len >= 256 + 4)  // will catch timeout too; limit length to max expected size
        goto Err;
    if ((byte = ReadByte()) < 0 || (byte != TOKEN)) goto Err;
    if ((byte = ReadByte()) < 0 || (byte != StkCmd)) goto Err;
    if ((byte = ReadByte()) < 0 || (byte != STATUS_CMD_OK)) goto Err;
    for (int i = 0; i < len - 2; i++) {
        if ((byte = ReadByte()) < 0) goto Err;
        if(i < maxLen)           // limit saved length (buffer is only 256B, but memory read reply contains additional status + 1 unknown byte)
        pstring[i] = byte;
    }
    ReadByte();                  // read checksum
    if (ckSumIn != 0) goto Err;
    IRQ_ON;
    return 1;
Err:
    IRQ_ON;
    return 0;
}

static uint8_t _CMD_SPI_MULTI_EX(uint8_t * resByte, uint8_t subcmd, uint16_t addr)
{
    StkCmd = CMD_SPI_MULTI;
    StkSendPacketHeader(8);
    StkSendByte(CMD_SPI_MULTI);
    StkSendByte(4);             // NumTX
    StkSendByte(4);             // NumRX
    StkSendByte(0);             // RxStartAdr
    StkSendByte(subcmd);        // {TxData} Cmd
    StkSendByte(addr >> 8);     // {TxData} AdrHi
    StkSendByte(addr & 0xff);   // {TxData} AdrLow
    StkSendByte(0);             // {TxData} 0
    StkSendPacketFooter();
    if (StkRcvPacket(stkInBuf, sizeof(stkInBuf))) { // NumRX + 3
        if ((stkInBuf[0] == 0x00)
            && ((stkInBuf[1] == subcmd) || (stkInBuf[1] == 0x00 /* ignore  zero returns */))
            && (stkInBuf[2] == 0x00)) {
            *resByte = stkInBuf[3];
        }
        return 1;
    }
    return 0;
}

static uint8_t _CMD_LOAD_ADDRESS(ioMem_t *pMem)
{
    // ignore 0xFFFF
    // assume address is set before and we read or write the immediately following memory
    if((pMem->addr == 0xffff))
        return 1;
    StkCmd = CMD_LOAD_ADDRESS;
    StkSendPacketHeader(5);
    StkSendByte(CMD_LOAD_ADDRESS);
    StkSendByte(0);
    StkSendByte(0);
    StkSendByte(pMem->addr >> 8);
    StkSendByte(pMem->addr & 0xff);
    StkSendPacketFooter();
    return StkRcvPacket(stkInBuf, sizeof(stkInBuf));
}

static uint8_t _CMD_READ_MEM_ISP(ioMem_t *pMem)
{
    StkSendPacketHeader(4);
    StkSendByte(StkCmd);
    StkSendByte(pMem->len >> 8);
    StkSendByte(pMem->len & 0xff);
    StkSendByte(CmdFlashEepromRead);
    StkSendPacketFooter();
    return StkRcvPacket(pMem->data, pMem->len);
}

static uint8_t _CMD_PROGRAM_MEM_ISP(ioMem_t *pMem)
{
    StkSendPacketHeader(pMem->len + 10);
    StkSendByte(StkCmd);
    StkSendByte(pMem->len >> 8);
    StkSendByte(pMem->len & 0xff);
    StkSendByte(0); // mode
    StkSendByte(0); // delay
    StkSendByte(0); // cmd1
    StkSendByte(0); // cmd2
    StkSendByte(0); // cmd3
    StkSendByte(0); // poll1
    StkSendByte(0); // poll2
    for(int i = 0; i < pMem->len; i++)
        StkSendByte(pMem->data[i]);
    StkSendPacketFooter();
    return StkRcvPacket(stkInBuf, sizeof(stkInBuf));
}

uint8_t Stk_SignOn(void)
{
    StkCmd = CMD_SIGN_ON;
    StkSendPacketHeader(1);
    StkSendByte(CMD_SIGN_ON);
    StkSendPacketFooter();
    return StkRcvPacket(stkInBuf, sizeof(stkInBuf));
}

uint8_t Stk_ConnectEx(escDeviceInfo_t *pDeviceInfo)
{
    if (!Stk_SignOn())
        return 0;
    uint8_t signature[3];    // device signature, MSB first
    for(unsigned i = 0; i < sizeof(signature); i++) {
        if (!_CMD_SPI_MULTI_EX(&signature[i], SPI_SIGNATURE_READ, i))
            return 0;
    }
    // convert signature to little endian
    pDeviceInfo->signature = (signature[1] << 8) | signature[2];
    pDeviceInfo->signature2 = signature[0];
    return 1;
}

uint8_t Stk_Chip_Erase(void)
{
    StkCmd = CMD_CHIP_ERASE_ISP;
    StkSendPacketHeader(7);
    StkSendByte(StkCmd);
    StkSendByte(20); // ChipErase_eraseDelay atmega8
    StkSendByte(0);  // ChipErase_pollMethod atmega8
    StkSendByte(0xAC);
    StkSendByte(0x88);
    StkSendByte(0x13);
    StkSendByte(0x76);
    StkSendPacketFooter();
    return StkRcvPacket(stkInBuf, sizeof(stkInBuf));
}

uint8_t Stk_ReadFlash(ioMem_t *pMem)
{
    if (!_CMD_LOAD_ADDRESS(pMem))
        return 0;
    StkCmd = CMD_READ_FLASH_ISP;
    return _CMD_READ_MEM_ISP(pMem);
}


uint8_t Stk_ReadEEprom(ioMem_t *pMem)
{
    if (!_CMD_LOAD_ADDRESS(pMem))
        return 0;
    StkCmd = CMD_READ_EEPROM_ISP;
    return _CMD_READ_MEM_ISP(pMem);
}

uint8_t Stk_WriteFlash(ioMem_t *pMem)
{
    if (!_CMD_LOAD_ADDRESS(pMem))
        return 0;
    StkCmd = CMD_PROGRAM_FLASH_ISP;
    return _CMD_PROGRAM_MEM_ISP(pMem);
}

uint8_t Stk_WriteEEprom(ioMem_t *pMem)
{
    if (!_CMD_LOAD_ADDRESS(pMem))
        return 0;
    StkCmd = CMD_PROGRAM_EEPROM_ISP;
    return _CMD_PROGRAM_MEM_ISP(pMem);
}

#endif
