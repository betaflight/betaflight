/*
 * Override of CubeN6's usbd_dfu_if.c — Betaflight DFU memory descriptor.
 *
 * The DfuSe descriptor string advertises a memory layout to the host
 * (start address, sector count, sector size, type code). We expose two
 * different layouts depending on which mode obl_app.c entered:
 *
 *   Normal mode (obl_recovery_mode == false):
 *     @Betaflight  /0x70100000/0100*64Kg
 *     ↑ starts at the BF slot, covers 16 MiB. The OBL slot
 *       (0x70000000..0x700FFFFF) is NOT exposed, so a host upload
 *       can't accidentally trash a working OBL.
 *
 *   Recovery mode (obl_recovery_mode == true, nor0 0x0 had no valid
 *                  signed-FSBL header):
 *     @Betaflight  /0x70000000/0800*64Kg
 *     ↑ starts at nor0 0x0, covers 128 MiB. Used for the bricked-FC
 *       recovery flow where the host needs to write OBL itself + BF.
 *
 * 64 KiB sector size matches the MX66UW1G45G 4-byte block-erase opcode
 * (0xDC) and minimises DFU round-trip overhead vs the chip's 4 KiB
 * sector erase. Type 'g' = read+erase+write.
 *
 * The pStrDesc pointer in USBD_DFU_Media_fops is updated before
 * OpenBootloader_Init() runs (see obl_app.c::run_obl_loop), so by the
 * time the host enumerates the DFU descriptor reflects the active mode.
 */

#include <stdbool.h>
#include <string.h>

#include "usbd_dfu_if.h"
#include "openbl_usb_cmd.h"
#include "usb_interface.h"
#include "openbl_mem.h"
#include "common_interface.h"
#include "openbootloader_conf.h"
#include "flash_iface.h"

#define MEDIA_DESC_STR_NORMAL       "@Betaflight  /0x70100000/0100*064Kg"
#define MEDIA_DESC_STR_RECOVERY     "@Betaflight  /0x70000000/0800*064Kg"

/* @DBGRAM alt — exposes a fixed AXISRAM region OBL uses to publish
 * boot-decision diagnostics. AXISRAM survives soft reset and is
 * reachable from the DFU upload context. */
#define DBG_DESC_STR                "@DBGRAM /0x24100000/1*512Ba"
#define DBG_RAM_BASE                0x24100000U
#define DBG_RAM_SIZE                0x00000200U

/* Referenced from our patched usbd_dfu.c::USBD_DFU_GetUsrStringDesc
 * for the alt-1 iInterface descriptor. */
const char obl_tamp_desc_str[] = DBG_DESC_STR;

/* MX66UW1G45G program/erase windows — used by the host's DFU GET_STATUS
 * to bound the polling timeout it announces in the device-state machine.
 * Generous; real hardware completes faster but the host doesn't care
 * about under-reporting. */
#define MEDIA_ERASE_TIME            (uint16_t)400U   /* ms per 64 KiB block */
#define MEDIA_PROGRAM_TIME          (uint16_t)5U     /* ms per page         */

extern bool obl_recovery_mode;
extern USBD_HandleTypeDef hUsbDeviceFS;

static uint16_t USB_DFU_If_Init(void);
static uint16_t USB_DFU_If_Erase(uint32_t Add);
static uint16_t USB_DFU_If_Write(uint8_t *pSrc, uint32_t alt, uint32_t Len, uint32_t BlockNumber);
static uint8_t *USB_DFU_If_Read(uint32_t alt, uint8_t *pDest, uint32_t Len, uint32_t BlockNumber);
static uint16_t USB_DFU_If_DeInit(void);
static uint16_t USB_DFU_If_GetStatus(uint32_t Add, uint8_t Cmd, uint8_t *buffer);

/* USBD_DFU_Media_fops::pStrDesc is patched at runtime by
 * obl_dfu_apply_mode() (called from obl_app.c) before
 * OpenBootloader_Init enables USB enumeration. The default points at
 * the recovery string so a stray enumeration before mode-set fails safe
 * (recovery is the more permissive of the two — better than the host
 * trying to write the OBL slot through a normal-mode descriptor that
 * doesn't cover it). */
__ALIGN_BEGIN USBD_DFU_MediaTypeDef USBD_DFU_Media_fops __ALIGN_END =
{
    (uint8_t *)MEDIA_DESC_STR_RECOVERY,
    USB_DFU_If_Init,
    USB_DFU_If_DeInit,
    USB_DFU_If_Erase,
    USB_DFU_If_Write,
    USB_DFU_If_Read,
    USB_DFU_If_GetStatus
};

void obl_dfu_apply_mode(bool recovery)
{
    USBD_DFU_Media_fops.pStrDesc = (uint8_t *)(recovery
        ? MEDIA_DESC_STR_RECOVERY
        : MEDIA_DESC_STR_NORMAL);
}

static uint16_t USB_DFU_If_Init(void)
{
    return 0;
}

static uint16_t USB_DFU_If_DeInit(void)
{
    return 0;
}

static uint16_t USB_DFU_If_Erase(uint32_t Add)
{
    /* @DBGRAM range is read-only; reject erase. */
    if (Add >= DBG_RAM_BASE && Add < DBG_RAM_BASE + DBG_RAM_SIZE) {
        return 1U;
    }

    if (OPENBL_MEM_GetAddressArea((uint32_t)Add) == AREA_ERROR) {
        return OPENBL_USB_SendAddressNack(&hUsbDeviceFS);
    }

    /* CubeN6's OPENBL_MEM_Erase is a stub that always returns ERROR
     * (Modules/Mem/openbl_mem.c). Bypass the middleware and erase a
     * single descriptor sector directly via the flash driver. The DFU
     * descriptor advertises EXT_MEMORY_SECTOR_SIZE-byte sectors so the
     * host issues exactly one Erase per sector before writing into it. */
    if (Add < EXT_MEMORY_START_ADDRESS) {
        return 1U;
    }
    const uint32_t offset = Add - EXT_MEMORY_START_ADDRESS;
    if (!flash_erase_range(offset, EXT_MEMORY_SECTOR_SIZE)) {
        return 1U;
    }
    return 0U;
}

/* DfuSe address pointer captured from SET_ADDRESS_POINTER. Defaults
 * mirror the descriptor's advertised start so a data block before any
 * SET_ADDR still lands at the slot's base. */
static uint32_t dfuse_addr_ptr_normal   = 0x70100000U;
static uint32_t dfuse_addr_ptr_recovery = 0x70000000U;

static uint32_t dfuse_get_base(void)
{
    return obl_recovery_mode ? dfuse_addr_ptr_recovery : dfuse_addr_ptr_normal;
}

static uint16_t USB_DFU_If_Write(uint8_t *pSrc, uint32_t alt, uint32_t Len, uint32_t BlockNumber)
{
    /* alt 1 (@DBGRAM) is read-only. DfuSe upload anchors via a
     * DOWNLOAD wValue=0,wLength=5 SET_ADDRESS_POINTER which we can
     * silently accept; refuse anything that looks like a data block. */
    if (alt == 1U) {
        if (BlockNumber < 2U) {
            return 0U;
        }
        return 1U;
    }

    /* DfuSe extension commands (bcdDFUVersion=0x011A) arrive as
     * BlockNumber=0, wLength==5, command byte at pSrc[0]:
     *   0x21 SET_ADDRESS_POINTER  pSrc[1..4] = LE target address
     *   0x41 ERASE_PAGE           pSrc[1..4] = LE sector address
     *   0x91 READ_UNPROTECT       (ignored)
     *   0x92 ERASE_ALL            (ignored — host issues per-sector)
     *
     * The vendored usbd_dfu.c routes every DOWNLOAD here without
     * decoding DfuSe, so this is the only point where these sub-
     * commands get dispatched. */
    if (BlockNumber == 0U && Len >= 5U) {
        const uint8_t cmd = pSrc[0];
        const uint32_t addr = (uint32_t)pSrc[1]
                            | ((uint32_t)pSrc[2] << 8)
                            | ((uint32_t)pSrc[3] << 16)
                            | ((uint32_t)pSrc[4] << 24);
        switch (cmd) {
        case 0x21U:
            if (obl_recovery_mode) {
                dfuse_addr_ptr_recovery = addr;
            } else {
                dfuse_addr_ptr_normal = addr;
            }
            return 0U;
        case 0x41U: {
            if (addr < EXT_MEMORY_START_ADDRESS) {
                return 1U;
            }
            const uint32_t offset = addr - EXT_MEMORY_START_ADDRESS;
            if (!flash_erase_range(offset, EXT_MEMORY_SECTOR_SIZE)) {
                return 1U;
            }
            return 0U;
        }
        default:
            return 0U;
        }
    }
    if (BlockNumber < 2U) {
        return 0U;
    }

    /* Block 2+ data writes — addressed relative to the most recent
     * SET_ADDRESS_POINTER (or the descriptor default if none arrived). */
    const uint32_t addr = dfuse_get_base() + (BlockNumber - 2U) * USBD_DFU_XFER_SIZE;
    if (addr < EXT_MEMORY_START_ADDRESS) {
        return 1U;
    }
    const uint32_t offset = addr - EXT_MEMORY_START_ADDRESS;
    if (!flash_program(offset, pSrc, Len)) {
        return 1U;
    }
    return 0U;
}

static uint8_t *USB_DFU_If_Read(uint32_t alt, uint8_t *pDest, uint32_t Len, uint32_t BlockNumber)
{
    if (alt == 1U) {
        /* @DBGRAM read from AXISRAM2 at DBG_RAM_BASE (0x24100000). ST
         * DfuSe upload addressing: BlockNumber 0/1 reserved (SET_ADDR /
         * GET_CMD), 2+ carry data at (BlockNumber - 2) * xfer_size from
         * DBG_RAM_BASE. */
        const uint32_t offset = (BlockNumber >= 2U)
                              ? (BlockNumber - 2U) * USBD_DFU_XFER_SIZE
                              : 0U;
        if (offset >= DBG_RAM_SIZE) {
            memset(pDest, 0, Len);
            return pDest;
        }
        uint32_t copy_len = Len;
        if (offset + copy_len > DBG_RAM_SIZE) {
            copy_len = DBG_RAM_SIZE - offset;
        }
        memcpy(pDest, (const uint8_t *)(DBG_RAM_BASE + offset), copy_len);
        if (copy_len < Len) {
            memset(pDest + copy_len, 0, Len - copy_len);
        }
        return pDest;
    }
    /* alt 0: memcpy from XSPI memory-mapped flash, anchored at the
     * most recent DfuSe SET_ADDRESS_POINTER. flash_memmap_on() is
     * idempotent and brings the controller back from indirect mode
     * if a prior Write() flipped it off. */
    if (BlockNumber < 2U) {
        memset(pDest, 0, Len);
        return pDest;
    }
    if (!flash_memmap_on()) {
        memset(pDest, 0, Len);
        return pDest;
    }
    const uint32_t addr = dfuse_get_base() + (BlockNumber - 2U) * USBD_DFU_XFER_SIZE;
    memcpy(pDest, (const uint8_t *)addr, Len);
    return pDest;
}

static uint16_t USB_DFU_If_GetStatus(uint32_t Add, uint8_t Cmd, uint8_t *pBuffer)
{
    (void)Add;
    switch (Cmd) {
    case DFU_MEDIA_PROGRAM:
        pBuffer[1] = (uint8_t)(MEDIA_PROGRAM_TIME & 0xFFU);
        pBuffer[2] = (uint8_t)((MEDIA_PROGRAM_TIME >> 8U) & 0xFFU);
        pBuffer[3] = 0U;
        break;
    case DFU_MEDIA_ERASE:
    default:
        pBuffer[1] = (uint8_t)(MEDIA_ERASE_TIME & 0xFFU);
        pBuffer[2] = (uint8_t)((MEDIA_ERASE_TIME >> 8U) & 0xFFU);
        pBuffer[3] = 0U;
        break;
    }
    return USBD_OK;
}
