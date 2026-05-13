/*
 * Override of CubeN6's usbd_conf.h.
 *
 * The stock OBL exposes 5 DFU alts (Flashlayout / FSBL-EXT / FSBL-APP /
 * <user app> / OTP) wired to the OBL middleware's phase machinery
 * (PHASE_FLASHLAYOUT / PHASE_3 / PHASE_4 / PHASE_CMD / PHASE_OTP). The
 * Betaflight OBL only needs a single memory descriptor — the BF slot at
 * 0x70100000 (or nor0 0x0 in recovery mode). Collapsing to one alt does
 * three things:
 *
 *   1. The host shows our @Betaflight memory descriptor as THE name on
 *      enumeration instead of "@Flashlayout /0x00/1*1Ke".
 *   2. dfu-util / STM32CubeProgrammer aim writes at alt 0 by default,
 *      no `-a 3` needed.
 *   3. We can bypass the OBL middleware's alt→phase logic and the
 *      OPENBL_USB_Download/EraseMemory phase-table machinery — those
 *      paths are wired for ST's signed-FSBL flow and silently no-op for
 *      a plain XIP user app at PHASE_CMD.
 *
 * Pulled in via the Makefile's `-I.` include path which precedes the
 * CubeN6 example's USB_Device/Target/.
 */

#ifndef USBD_CONF_H
#define USBD_CONF_H

#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USBD_CLASS_USER_STRING_DESC            0U

#define USBD_MAX_NUM_INTERFACES                1U
#define USBD_MAX_NUM_CONFIGURATION             1U
#define USBD_MAX_STR_DESC_SIZ                  130U
#define USBD_SUPPORT_USER_STRING_DESC          1U
#define USBD_SELF_POWERED                      1U
#define USBD_DEBUG_LEVEL                       0U

/* Two DFU alts:
 *   alt 0: @Betaflight (BF flash slot — read/erase/write)
 *   alt 1: @DBGRAM     (read-only mirror of the AXISRAM2 debug buffer
 *                       at DBG_RAM_BASE, so the host can
 *                       `dfu-util -a 1 -U dbgram.bin` to retrieve BF's
 *                       last fault snapshot + magic state after a wedge)
 * The patched usbd_dfu.c::USBD_DFU_GetUsrStringDesc routes alt 0 to
 * DfuInterface->pStrDesc and alt 1 to a fixed @DBGRAM descriptor string. */
#define USBD_DFU_MAX_ITF_NUM                   2U
#define USBD_DFU_XFER_SIZE                     1024U
#define USBD_DFU_APP_DEFAULT_ADD               0x70100000U
#define USBD_DFU_MAX_NB_OF_SECTORS             256U

#define DEVICE_FS                              0U

#define USBD_malloc                            (void *)USBD_static_malloc
#define USBD_free                              USBD_static_free
#define USBD_memset                            memset
#define USBD_memcpy                            memcpy
#define USBD_Delay                             HAL_Delay

#if (USBD_DEBUG_LEVEL > 0U)
#define USBD_UsrLog(...) printf(__VA_ARGS__); printf("\n");
#else
#define USBD_UsrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 1U)
#define USBD_ErrLog(...) printf("ERROR: "); printf(__VA_ARGS__); printf("\n");
#else
#define USBD_ErrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 2U)
#define USBD_DbgLog(...) printf("DEBUG : "); printf(__VA_ARGS__); printf("\n");
#else
#define USBD_DbgLog(...)
#endif

extern PCD_HandleTypeDef hpcd_USB_HS;

void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#endif /* USBD_CONF_H */
