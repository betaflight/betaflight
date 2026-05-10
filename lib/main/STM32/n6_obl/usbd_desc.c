/*
 * Override of CubeN6's usbd_desc.c — Betaflight-branded USB descriptors.
 *
 * VID/PID kept as ST DfuSe defaults (0x0483/0xDF11) so dfu-util,
 * STM32CubeProgrammer, and BF Configurator's existing DFU plumbing all
 * work unchanged. Strings are Betaflight-branded so users see "Betaflight
 * N6 OBL" on enumerate.
 */

#include <stdio.h>

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

/* DfuSe protocol default VID/PID. Standard tooling targets this pair. */
#define USBD_VID                      0x0483
#define USBD_PID                      0xDF11
#define USBD_LANGID_STRING            1033

#define USBD_MANUFACTURER_STRING      "Betaflight"
#define USBD_PRODUCT_STRING           "Betaflight N6 DFU"
#define USBD_CONFIGURATION_STRING     "BF N6 DFU Config"
#define USBD_INTERFACE_STRING         "BF N6 DFU Interface"

uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ];

static void Get_SerialNum(void);

uint8_t *USBD_DFU_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_DFU_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_DFU_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_DFU_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_DFU_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_DFU_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_DFU_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef DFU_Desc =
{
    USBD_DFU_DeviceDescriptor,
    USBD_DFU_LangIDStrDescriptor,
    USBD_DFU_ManufacturerStrDescriptor,
    USBD_DFU_ProductStrDescriptor,
    USBD_DFU_SerialStrDescriptor,
    USBD_DFU_ConfigStrDescriptor,
    USBD_DFU_InterfaceStrDescriptor
};

__ALIGN_BEGIN uint8_t USBD_DFU_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
    0x12,
    USB_DESC_TYPE_DEVICE,
    0x10, 0x01,                  /* USB 1.10 — DfuSe convention */
    0x00, 0x00, 0x00,
    USB_MAX_EP0_SIZE,
    LOBYTE(USBD_VID), HIBYTE(USBD_VID),
    LOBYTE(USBD_PID), HIBYTE(USBD_PID),
    0x00, 0x02,                  /* bcdDevice = 2.00 */
    USBD_IDX_MFC_STR,
    USBD_IDX_PRODUCT_STR,
    USBD_IDX_SERIAL_STR,
    USBD_MAX_NUM_CONFIGURATION
};

__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID_STRING),
    HIBYTE(USBD_LANGID_STRING)
};

__ALIGN_BEGIN uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END =
{
    USB_SIZ_STRING_SERIAL,
    USB_DESC_TYPE_STRING,
};

uint8_t *USBD_DFU_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_DFU_DeviceDesc);
    return USBD_DFU_DeviceDesc;
}

uint8_t *USBD_DFU_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_LangIDDesc);
    return USBD_LangIDDesc;
}

uint8_t *USBD_DFU_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

uint8_t *USBD_DFU_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

uint8_t *USBD_DFU_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = USB_SIZ_STRING_SERIAL;
    Get_SerialNum();
    return (uint8_t *)USBD_StringSerial;
}

uint8_t *USBD_DFU_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

uint8_t *USBD_DFU_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_INTERFACE_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

static void Get_SerialNum(void)
{
    /* Read the device unique ID from the boot ROM-published location.
     * DEVICE_ID1/2/3 are CMSIS macros pointing into system memory. */
    unsigned long s0 = *(uint32_t *)DEVICE_ID1;
    unsigned long s1 = *(uint32_t *)DEVICE_ID2;
    unsigned long s2 = *(uint32_t *)DEVICE_ID3;
    char serial_string[SIZ_STRING_SERIAL + 2U];

    snprintf(serial_string, sizeof(serial_string), "%08lX%08lX%08lX", s0, s1, s2);

    uint8_t idx = 2U;
    const char *p = serial_string;
    while (*p != '\0' && idx < (USB_SIZ_STRING_SERIAL - 1U)) {
        USBD_StringSerial[idx++] = *p++;
        USBD_StringSerial[idx++] = 0x00U;
    }
}
