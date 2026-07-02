/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * phone-flash: firmware update over the phone-config usb-ncm link, so an iphone can reflash the fc over
 * a cable without dfu (ios can't drive usb dfu). the host streams the image to a udp service, a routine
 * copied into ram erases and programs the flash while all flash access is stalled. see phoneflash_burn.
 *
 * wire format is little-endian to match the arm core. every packet starts with the 4-byte magic and an
 * opcode, replies echo the opcode with the high bit set.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define PHONE_FLASH_UDP_PORT     5762
#define PHONE_FLASH_MAGIC        0x48534C46U   // "FLSH"
#define PHONE_FLASH_REPLY_BIT    0x80U

// max firmware bytes per PF_WRITE. keeps the ncm datagram under the 2048-byte ntb limit.
#define PHONE_FLASH_CHUNK        1024U

// ncm block buffer size for the ram burn engine, matches the class' NCM_NTB_OUT_MAX_SIZE
#define NCM_BURN_NTB             2048U

enum {
    PF_HELLO   = 0x01,   // host -> fc: request flash. reply carries the flash geometry
    PF_BEGIN   = 0x02,   // host -> fc: {u32 totalSize, u32 imageCrc}
    PF_WRITE   = 0x03,   // host -> fc: {u32 offset, u16 len, bytes[len]} offset from 0x08000000
    PF_END     = 0x04,   // host -> fc: verify the readback crc over totalSize
    PF_REBOOT  = 0x05,   // host -> fc: reset into the freshly flashed firmware
    PF_ABORT   = 0x06,   // host -> fc: give up, reboot back to normal (only valid before any erase)
};

enum {
    PF_ERR_NONE       = 0,
    PF_ERR_BADMAGIC   = 1,
    PF_ERR_BADOP      = 2,
    PF_ERR_RANGE      = 3,   // offset/len outside the writable firmware region
    PF_ERR_ALIGN      = 4,   // offset or len not word aligned
    PF_ERR_SEQ        = 5,   // non-sequential write (host must stream in order)
    PF_ERR_FLASH      = 6,   // erase/program/verify failed
    PF_ERR_CRC        = 7,   // whole-image crc mismatch at PF_END
    PF_ERR_STATE      = 8,   // opcode not valid in the current state
};

// flash geometry reported in the PF_HELLO reply, so the host does not hard-code the target.
// {u32 magic}{u8 op|reply}{u8 err}{u32 flashBase}{u32 writableBase}{u32 writableSize}{u16 maxChunk}{u16 mcu}
#define PHONE_FLASH_FC_BASE       0x08000000U

// note a PF_HELLO seen by the flash-resident lwip listener, so the phone-config task can hand off to the
// ram burn engine. implemented in phoneflash_burn_<mcu>.c.
void phoneFlashNoteHello(const uint8_t *payload, uint16_t len);
bool phoneFlashPending(void);
void phoneFlashRun(void);
