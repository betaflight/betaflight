/*
 * OBL DFU loop entry points called from main.c's boot decision.
 *
 * Both run the same OBL middleware loop (OpenBootloader_Init +
 * OpenBootloader_ProtocolDetection in a busy loop). They differ in how
 * the DFU memory map is configured before the loop starts:
 *   - normal mode protects the OBL slot (advertise @External_Flash from
 *     nor0 0x100000 onwards — BF slot only);
 *   - recovery mode exposes nor0 from offset 0 so the host can rewrite
 *     OBL itself.
 *
 * The mode is communicated to USB_DFU_If_* in usbd_dfu_if.c via the
 * obl_recovery_mode flag below — checked when DFU initialises its memory
 * descriptor string.
 *
 * Once a host completes a DFU download, our patched usbd_dfu.c sets
 * dfu_leave_pending; the loop below detaches USB and triggers a system
 * reset so the freshly-flashed BF runs without manual NRST.
 */

#include "main.h"
#include "app_openbootloader.h"
#include "usbd_core.h"

#include "flash_iface.h"

bool obl_recovery_mode;

extern void obl_dfu_apply_mode(bool recovery);
extern volatile bool dfu_leave_pending;
extern USBD_HandleTypeDef hUsbDeviceFS;

/* Time for the host's final GET_STATUS to return dfuMANIFEST-WAIT-RESET
 * before the device disappears off the bus. dfu-util prints a clean
 * "File downloaded successfully" once the manifest poll loop completes
 * within ~250 ms; 400 ms is generous. */
#define DFU_LEAVE_DRAIN_MS  400U

static __attribute__((noreturn)) void run_obl_loop(bool recovery)
{
    obl_recovery_mode = recovery;
    /* Patch the DFU descriptor string before USB enumeration starts so
     * the host sees the right memory geometry on first GET_DESCRIPTOR. */
    obl_dfu_apply_mode(recovery);

    OpenBootloader_Init();

    /* IWDG is NOT started in DFU mode — host upload can take tens of
     * seconds and the BF watchdog window (~30 s) is too tight for that.
     * The watchdog is armed only when handing off to BF (iwdg_start in
     * main.c). */
    while (1) {
        OpenBootloader_ProtocolDetection();

        if (dfu_leave_pending) {
            HAL_Delay(DFU_LEAVE_DRAIN_MS);
            (void)USBD_Stop(&hUsbDeviceFS);
            (void)USBD_DeInit(&hUsbDeviceFS);
            HAL_Delay(50U);
            NVIC_SystemReset();
        }
    }
}

__attribute__((noreturn)) void OBL_run_dfu_normal(void)
{
    run_obl_loop(false);
}

__attribute__((noreturn)) void OBL_run_dfu_recovery(void)
{
    run_obl_loop(true);
}
