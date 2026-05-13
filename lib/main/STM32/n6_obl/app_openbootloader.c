/*
 * Override of CubeN6's app_openbootloader.c.
 *
 * The submodule version registers USART + USB + IWDG transports. We only
 * use USB + IWDG (FCs don't expose UART pins for bootloader purposes),
 * so dropping USART trims a few KiB of code and lets us skip building
 * the openbl_usart_cmd / usart_interface sources entirely.
 *
 * Picked up before the submodule version because -I. and the local
 * source path come first.
 */

#include "main.h"

#include "openbl_core.h"

#include "openbl_usb_cmd.h"

#include "app_openbootloader.h"
#include "usb_interface.h"
#include "iwdg_interface.h"
#include "otp_interface.h"

extern OPENBL_MemoryTypeDef RAM_Descriptor;
extern OPENBL_MemoryTypeDef RAM_FlashLoader_Descriptor;
extern OPENBL_MemoryTypeDef EXTERNAL_MEMORY_Descriptor;

static OPENBL_HandleTypeDef USB_Handle;
static OPENBL_HandleTypeDef IWDG_Handle;

static OPENBL_OpsTypeDef USB_Ops =
{
    OPENBL_USB_Configuration,
    OPENBL_USB_DeInit,
    OPENBL_USB_ProtocolDetection,
    NULL,
    NULL
};

static OPENBL_OpsTypeDef IWDG_Ops =
{
    OPENBL_IWDG_Configuration,
    NULL,
    NULL,
    NULL,
    NULL
};

uint16_t SpecialCmdList[SPECIAL_CMD_MAX_NUMBER] =
{
    SPECIAL_CMD_DEFAULT
};

uint16_t ExtendedSpecialCmdList[EXTENDED_SPECIAL_CMD_MAX_NUMBER] =
{
    SPECIAL_CMD_DEFAULT
};

void OpenBootloader_Init(void)
{
    USB_Handle.p_Ops = &USB_Ops;
    USB_Handle.p_Cmd = NULL;
    if (OPENBL_RegisterInterface(&USB_Handle) != SUCCESS) {
        Error_Handler();
    }

    IWDG_Handle.p_Ops = &IWDG_Ops;
    IWDG_Handle.p_Cmd = NULL;
    if (OPENBL_RegisterInterface(&IWDG_Handle) != SUCCESS) {
        Error_Handler();
    }

    OPENBL_Init();

    if (OPENBL_MEM_RegisterMemory(&RAM_Descriptor) != SUCCESS) {
        Error_Handler();
    }
    if (OPENBL_MEM_RegisterMemory(&RAM_FlashLoader_Descriptor) != SUCCESS) {
        Error_Handler();
    }
    if (OPENBL_MEM_RegisterMemory(&EXTERNAL_MEMORY_Descriptor) != SUCCESS) {
        Error_Handler();
    }

    OPENBL_OTP_Init();
}

void OpenBootloader_DeInit(void)
{
    System_DeInit();
}

void OpenBootloader_ProtocolDetection(void)
{
    static uint32_t interface_detected = 0U;

    if (interface_detected == 0U) {
        interface_detected = OPENBL_InterfaceDetection();
        if (interface_detected == 1U) {
            OPENBL_InterfacesDeInit();
        }
    }

    if (interface_detected == 1U) {
        OPENBL_CommandProcess();
    }
}
