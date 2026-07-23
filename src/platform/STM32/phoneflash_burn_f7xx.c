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
 * phone-flash burn engine, stm32f7 wrapper: the register-level flash driver plus the mcu constants for
 * the shared engine in phoneflash_burn_impl.c. these parts are single-bank, so all flash access stalls
 * during an erase or program and the code being erased is the code we run from; the whole receive-and-
 * program path therefore runs from a copy in .ram_flash with interrupts masked and polled usb, touching
 * no flash/hal/libc mid-burn. nothing is erased until the host completes a live round trip against this
 * ram code, so a broken build reboots to the intact firmware; rom dfu is the final backstop.
 */

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_PHONE_CONFIG

#include "drivers/persistent.h"

#include "io/phoneflash.h"

// single-bank f7 parts whose sector geometry is known here. other f7 targets get inert stubs.
#if defined(STM32F722xx) || defined(STM32F745xx) || defined(STM32F746xx)

// burn code must never become a memcpy/memset call back into flash, hence no-tree-loop-distribute-patterns
#define RF __attribute__((section(".ram_flash"), noinline, used, \
                          optimize("no-tree-loop-distribute-patterns")))

// must never exist out-of-line in flash, the burn path calls them mid-erase
#define RFI static inline __attribute__((always_inline))

#define PF_REPLY   PHONE_FLASH_REPLY_BIT

#define OUR_IP     0xC0A80701U     // 192.168.7.1
#define OUR_MAC_5  0x02U           // fc mac low byte, differs from the host's ..:01
#define PF_OUT_EPNUM   1U
#define RF_HZ           216000000U      // f7 core clock
#define RF_IDLE_TIMEOUT (RF_HZ * 15U)   // host-idle reboot timeout, well below the u32 cyccnt wrap at ~19s

#if defined(STM32F722xx)
#define PF_FLASH_SIZE  (512U * 1024U)
#define PF_CHIP_ID     0xF722U
#elif defined(STM32F745xx)
#define PF_FLASH_SIZE  (1024U * 1024U)
#define PF_CHIP_ID     0xF745U
#else
#define PF_FLASH_SIZE  (1024U * 1024U)
#define PF_CHIP_ID     0xF746U
#endif

// config sector bounds (__config_start/__config_end, declared in common_post.h) come from the
// linker script. sector 1 on every supported part.

// FLASH_KEY1 / FLASH_KEY2 come from the hal flash header
#define FLASH_SR_ERRORS 0x000000F2U   // OPERR|WRPERR|PGAERR|PGPERR|ERSERR
#define FLASH_SR_CLEAR  0x000000F3U   // above plus EOP, write-1-to-clear

#define OTG          ((USB_OTG_GlobalTypeDef *)USB_OTG_FS_PERIPH_BASE)
#define OTG_INEP(i)  ((USB_OTG_INEndpointTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (i) * USB_OTG_EP_REG_SIZE))
#define OTG_OUTEP(i) ((USB_OTG_OUTEndpointTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (i) * USB_OTG_EP_REG_SIZE))
#define OTG_FIFO(i)  (*(volatile uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + (i) * USB_OTG_FIFO_SIZE))

// buffers sit in sram2 (unused on this target) to spare the tight sram1
#define BURN_BUF __attribute__((section(".sram2")))

static BURN_BUF uint8_t rxNtb[NCM_BURN_NTB];
static BURN_BUF uint8_t txNtb[NCM_BURN_NTB];
static BURN_BUF uint8_t txFrame[1536];
static BURN_BUF uint8_t replyBuf[64];

extern uint32_t _siram_flash[];
extern uint32_t _sram_flash[];
extern uint32_t _eram_flash[];

#include "phoneflash_burn.h"

// keep the watchdog fed through a multi-second erase, where nothing else runs
static RF void rfKickWdg(void)
{
    IWDG->KR = 0x0000AAAAU;
}

// ---- flash driver (register level, no hal) ----

static RF void rfWaitBsy(void)
{
    uint32_t start = DWT->CYCCNT;
    while (FLASH->SR & FLASH_SR_BSY) {
        rfKickWdg();
        if ((uint32_t)(DWT->CYCCNT - start) > RF_HZ * 8U) {   // f74x worst-case 256k sector erase is ~4s
            rfReboot();
        }
    }
}

static RF uint32_t rfSectorOf(uint32_t addr)
{
    uint32_t off = addr - PHONE_FLASH_FC_BASE;
#if defined(STM32F722xx)
    if (off < 0x04000) return 0;   // 16k, vector table
    if (off < 0x08000) return 1;   // 16k, config (never erased)
    if (off < 0x0C000) return 2;   // 16k
    if (off < 0x10000) return 3;   // 16k
    if (off < 0x20000) return 4;   // 64k
    if (off < 0x40000) return 5;   // 128k
    if (off < 0x60000) return 6;   // 128k
    if (off < 0x80000) return 7;   // 128k
#else // f745/f746
    if (off < 0x08000) return 0;   // 32k, vector table
    if (off < 0x10000) return 1;   // 32k, config (never erased)
    if (off < 0x18000) return 2;   // 32k
    if (off < 0x20000) return 3;   // 32k
    if (off < 0x40000) return 4;   // 128k
    if (off < 0x80000) return 5;   // 256k
    if (off < 0xC0000) return 6;   // 256k
    if (off < 0x100000) return 7;  // 256k
#endif
    return 0xFF;
}

static RF void rfFlashUnlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}

static RF void rfFlashLock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

static RF uint32_t rfEraseSector(uint32_t sector)
{
    rfWaitBsy();
    FLASH->SR = FLASH_SR_CLEAR;
    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= (2U << FLASH_CR_PSIZE_Pos);          // x32 program/erase, needs 2.7v+
    FLASH->CR &= ~FLASH_CR_SNB;
    FLASH->CR |= FLASH_CR_SER | (sector << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    rfWaitBsy();
    FLASH->CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);
    return (FLASH->SR & FLASH_SR_ERRORS) ? 1U : 0U;
}

static RF uint32_t rfProgramWord(uint32_t addr, uint32_t word)
{
    rfWaitBsy();
    FLASH->SR = FLASH_SR_CLEAR;
    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= (2U << FLASH_CR_PSIZE_Pos);
    FLASH->CR |= FLASH_CR_PG;
    *(volatile uint32_t *)addr = word;
    __DSB();
    rfWaitBsy();
    FLASH->CR &= ~FLASH_CR_PG;
    return (FLASH->SR & FLASH_SR_ERRORS) ? 1U : 0U;
}

// program word-aligned len at addr, erasing each sector on first touch and verifying every word by
// readback into the running crc. returns non-zero on failure.
static RF uint32_t rfProgram(uint32_t addr, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t sec = rfSectorOf(addr + i);
        if (sec == 1 || sec == 0xFF) {
            return 1;                                  // never touch config or run off the end
        }
    }
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t a = addr + i;
        uint32_t sec = rfSectorOf(a);
        if (!(erasedMask & (1U << sec))) {
            if (rfEraseSector(sec)) {
                return 1;
            }
            erasedMask |= (1U << sec);
        }
        uint32_t w = rd32(data + i);
        if (rfProgramWord(a, w)) {
            return 1;
        }
        uint32_t rb = *(volatile uint32_t *)a;
        if (rb != w) {
            return 1;
        }
        for (uint32_t b = 0; b < 4; b++) {
            rfCrc32Byte((rb >> (8 * b)) & 0xFF);
        }
    }
    return 0;
}

// f7 programs a word at a time, so there is no partial-row staging to reset or flush
static RF void rfProgramBegin(void) { }
static RF uint32_t rfProgramFinish(void) { return 0; }

#include "phoneflash_burn_impl.c"

#else // unsupported f7 part

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

#endif // supported f7 part

#endif // USE_PHONE_CONFIG
