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
 * phone-flash burn engine, stm32h7 wrapper: the register-level flash driver plus the mcu constants for
 * the shared engine in phoneflash_burn_impl.c. same design as the f7 wrapper (the whole receive-and-
 * program path runs from a copy in .ram_flash, interrupts masked, polled usb, no flash/hal/libc
 * mid-burn); the h7 differences are the dual-bank flash controller (per-bank registers, 128k sectors)
 * and the 256-bit program word, which forces staging bytes into a 32-byte row before each program.
 */

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_PHONE_CONFIG

#include "drivers/persistent.h"

#include "io/phoneflash.h"

// h7 parts whose flash geometry and usb instance are known here. others get inert stubs.
#if defined(STM32H743xx)

// burn code must never become a memcpy/memset call back into flash, hence no-tree-loop-distribute-patterns
#define RF __attribute__((section(".ram_flash"), noinline, used, \
                          optimize("no-tree-loop-distribute-patterns")))

// must never exist out-of-line in flash, the burn path calls them mid-erase
#define RFI static inline __attribute__((always_inline))

#define PF_REPLY   PHONE_FLASH_REPLY_BIT

#define OUR_IP     0xC0A80701U     // 192.168.7.1
#define OUR_MAC_5  0x02U           // fc mac low byte, differs from the host's ..:01
#define PF_OUT_EPNUM   1U
#define RF_HZ           480000000U     // h743 core clock
#define RF_IDLE_TIMEOUT (RF_HZ * 8U)   // host-idle reboot timeout, below the u32 cyccnt wrap at ~8.9s

#define PF_FLASH_SIZE  (2048U * 1024U)
#define PF_CHIP_ID     0x0743U

// config sector bounds (__config_start/__config_end, declared in common_post.h) come from the
// linker script. bank 1 sector 1.

// FLASH_KEY1 / FLASH_KEY2 come from the hal flash header
#define PF_SR_ERRMASK (FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | FLASH_SR_INCERR | \
                       FLASH_SR_OPERR | FLASH_SR_RDPERR | FLASH_SR_RDSERR | FLASH_SR_SNECCERR | \
                       FLASH_SR_DBECCERR)
#define PF_SR_CLEAR   (PF_SR_ERRMASK | FLASH_SR_EOP)   // ccr bits mirror the sr layout

// vcp runs on usb2 otg-fs (pa11/pa12) on the h743
#define OTG          ((USB_OTG_GlobalTypeDef *)USB2_OTG_FS_PERIPH_BASE)
#define OTG_INEP(i)  ((USB_OTG_INEndpointTypeDef *)(USB2_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (i) * USB_OTG_EP_REG_SIZE))
#define OTG_OUTEP(i) ((USB_OTG_OUTEndpointTypeDef *)(USB2_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (i) * USB_OTG_EP_REG_SIZE))
#define OTG_FIFO(i)  (*(volatile uint32_t *)(USB2_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + (i) * USB_OTG_FIFO_SIZE))

static uint8_t rxNtb[NCM_BURN_NTB];
static uint8_t txNtb[NCM_BURN_NTB];
static uint8_t txFrame[1536];
static uint8_t replyBuf[64];

// 256-bit program word staging
static uint8_t  rowBuf[32];
static uint32_t rowAddr;
static uint32_t rowFill;

extern uint32_t _siram_flash[];
extern uint32_t _sram_flash[];
extern uint32_t _eram_flash[];

#include "phoneflash_burn.h"

// keep the watchdog fed through a multi-second erase, where nothing else runs
static RF void rfKickWdg(void)
{
    IWDG1->KR = 0x0000AAAAU;
}

// per-bank flash registers, sectors 0-7 bank 1, 8-15 bank 2
RFI volatile uint32_t *rfCrReg(uint32_t sec)  { return (sec >= 8U) ? &FLASH->CR2 : &FLASH->CR1; }
RFI volatile uint32_t *rfSrReg(uint32_t sec)  { return (sec >= 8U) ? &FLASH->SR2 : &FLASH->SR1; }
RFI volatile uint32_t *rfCcrReg(uint32_t sec) { return (sec >= 8U) ? &FLASH->CCR2 : &FLASH->CCR1; }

// ---- flash driver (register level, no hal) ----

static RF void rfWaitIdle(uint32_t sec)
{
    uint32_t start = DWT->CYCCNT;
    while (*rfSrReg(sec) & (FLASH_SR_BSY | FLASH_SR_QW)) {
        rfKickWdg();
        if ((uint32_t)(DWT->CYCCNT - start) > RF_HZ * 8U) {
            rfReboot();
        }
    }
}

static RF uint32_t rfSectorOf(uint32_t addr)
{
    uint32_t off = addr - PHONE_FLASH_FC_BASE;
    if (off >= PF_FLASH_SIZE) {
        return 0xFF;
    }
    return off >> 17;   // 128k sectors. sector 1 holds the config, never erased
}

static RF void rfFlashUnlock(void)
{
    if (FLASH->CR1 & FLASH_CR_LOCK) {
        FLASH->KEYR1 = FLASH_KEY1;
        FLASH->KEYR1 = FLASH_KEY2;
    }
    if (FLASH->CR2 & FLASH_CR_LOCK) {
        FLASH->KEYR2 = FLASH_KEY1;
        FLASH->KEYR2 = FLASH_KEY2;
    }
}

static RF void rfFlashLock(void)
{
    FLASH->CR1 |= FLASH_CR_LOCK;
    FLASH->CR2 |= FLASH_CR_LOCK;
}

static RF uint32_t rfEraseSector(uint32_t sector)
{
    volatile uint32_t *cr = rfCrReg(sector);
    rfWaitIdle(sector);
    *rfCcrReg(sector) = PF_SR_CLEAR;
    *cr &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
    *cr |= FLASH_CR_SER | (2U << FLASH_CR_PSIZE_Pos)                     // x32 program/erase, needs 2.7v+
           | ((sector & 7U) << FLASH_CR_SNB_Pos) | FLASH_CR_START;
    rfWaitIdle(sector);
    *cr &= ~(FLASH_CR_SER | FLASH_CR_SNB);
    return (*rfSrReg(sector) & PF_SR_ERRMASK) ? 1U : 0U;
}

// program one 32-byte flash word. the h7 write buffer fills from 8 consecutive word stores and
// programs automatically on the 8th
static RF uint32_t rfProgramRow(uint32_t addr, const uint8_t *data)
{
    uint32_t sec = rfSectorOf(addr);
    volatile uint32_t *cr = rfCrReg(sec);
    rfWaitIdle(sec);
    *rfCcrReg(sec) = PF_SR_CLEAR;
    *cr &= ~FLASH_CR_PSIZE;
    *cr |= (2U << FLASH_CR_PSIZE_Pos);
    *cr |= FLASH_CR_PG;
    __ISB();
    __DSB();
    volatile uint32_t *dst = (volatile uint32_t *)addr;
    for (uint32_t i = 0; i < 8U; i++) {
        dst[i] = rd32(data + 4U * i);
    }
    __ISB();
    __DSB();
    rfWaitIdle(sec);
    *cr &= ~FLASH_CR_PG;
    return (*rfSrReg(sec) & PF_SR_ERRMASK) ? 1U : 0U;
}

// program the staged row, erasing its sector on first touch and verifying by readback. only the
// first `valid` bytes came from the host: they feed the running crc, the 0xff pad just gets verified
static RF uint32_t rfFlushRow(uint32_t valid)
{
    uint32_t sec = rfSectorOf(rowAddr);
    if (sec == 1U || sec == 0xFFU) {
        return 1;
    }
    for (uint32_t i = valid; i < 32U; i++) {
        rowBuf[i] = 0xFF;
    }
    if (!(erasedMask & (1U << sec))) {
        if (rfEraseSector(sec)) {
            return 1;
        }
        erasedMask |= (1U << sec);
    }
    if (rfProgramRow(rowAddr, rowBuf)) {
        return 1;
    }
    for (uint32_t i = 0; i < 32U; i++) {
        uint8_t rb = *(volatile uint8_t *)(rowAddr + i);
        if (rb != rowBuf[i]) {
            return 1;
        }
        if (i < valid) {
            rfCrc32Byte(rb);
        }
    }
    rowAddr += 32U;
    rowFill = 0;
    return 0;
}

// stage word-aligned len at addr into 32-byte rows. addr is sequential except for the config-region
// jump (validated by the protocol layer), which must land on a fresh row: any partially staged row is
// finished 0xff-padded first. returns non-zero on failure.
static RF uint32_t rfProgram(uint32_t addr, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t sec = rfSectorOf(addr + i);
        if (sec == 1U || sec == 0xFFU) {
            return 1;                                  // never touch config or run off the end
        }
    }
    if (rowFill != 0 && addr != rowAddr + rowFill) {
        if (addr < rowAddr + 32U) {
            return 1;                                  // a row can only be programmed once
        }
        if (rfFlushRow(rowFill)) {
            return 1;
        }
    }
    if (rowFill == 0) {
        if (addr & 31U) {
            return 1;
        }
        rowAddr = addr;
    }
    for (uint32_t i = 0; i < len; i++) {
        rowBuf[rowFill++] = data[i];
        if (rowFill == 32U) {
            if (rfFlushRow(32U)) {
                return 1;
            }
        }
    }
    return 0;
}

static RF void rfProgramBegin(void)
{
    rowFill = 0;
}

// flush the final partial row, 0xff padded, at the end of the image
static RF uint32_t rfProgramFinish(void)
{
    if (rowFill != 0) {
        return rfFlushRow(rowFill);
    }
    return 0;
}

#include "phoneflash_burn_impl.c"

#else // unsupported h7 part

void phoneFlashNoteHello(const uint8_t *payload, uint16_t len)
{
    (void)payload;
    (void)len;
}

bool phoneFlashPending(void)
{
    return false;
}

void phoneFlashRun(void)
{
}

#endif // supported h7 part

#endif // USE_PHONE_CONFIG
