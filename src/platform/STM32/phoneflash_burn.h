/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * phone-flash burn engine: shared state and byte helpers. included by each per-mcu wrapper
 * (phoneflash_burn_<mcu>.c) after it defines the RF/RFI macros, before the wrapper's flash ops that
 * fold the running crc. the mcu-independent bulk of the engine (usb, ncm, ip, protocol) lives in
 * phoneflash_burn_impl.c, included after the flash ops. not a standalone translation unit.
 */

#pragma once

enum { ST_WAIT_HELLO = 0, ST_READY, ST_FLASHING, ST_DONE };

static uint8_t  hostMac[6];
static uint32_t hostIp;
static uint16_t hostPort;
static uint16_t txSeq;

static uint8_t  state;
static uint32_t totalSize;
static uint32_t imageCrc;
static uint32_t writtenUpTo;
static uint32_t programmedBytes;
static uint32_t crcState;
static uint32_t erasedMask;
static uint8_t  rebootReq;
static uint8_t  abortReq;

RFI uint32_t rd32(const uint8_t *p)  { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
RFI uint16_t rd16(const uint8_t *p)  { return (uint16_t)(p[0] | p[1] << 8); }
RFI void     wr32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
RFI void     wr16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
RFI uint32_t rd32be(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | (uint32_t)p[3]; }
RFI void     wr32be(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
RFI void     wr16be(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v; }

// crc-32 (reflected, poly 0xedb88320) fold of one verified byte, matches the host's whole-image crc
RFI void rfCrc32Byte(uint8_t b)
{
    crcState ^= b;
    for (uint32_t k = 0; k < 8; k++) {
        crcState = (crcState >> 1) ^ (0xEDB88320U & (uint32_t)-(int32_t)(crcState & 1U));
    }
}

static RF void rfMemcpy(uint8_t *d, const uint8_t *s, uint32_t n)
{
    while (n--) {
        *d++ = *s++;
    }
}

static RF void rfReboot(void)
{
    __DSB();
    SCB->AIRCR = (0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
    __DSB();
    for (;;) { }
}
