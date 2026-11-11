/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Betaflight is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this software. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stddef.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_USB_MSC

#include "common/printf.h"
#include "common/utils.h"

#include "usbd_core.h"
#include "usbd_msc.h"
#include "usbd_msc_desc.h"

#define MSC_VID 0x396A
#define MSC_PID 0x1000
#define MSC_OUT_EP 0x02
#define MSC_IN_EP 0x81
#define MSC_MAX_PACKET_SIZE 64
#define MSC_CONFIG_SIZE (USB_SIZEOF_CONFIG_DESC + MSC_DESCRIPTOR_LEN)

USB_MEM_ALIGNX static const uint8_t deviceDescriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, MSC_VID, MSC_PID, 0x0200, 0x01)
};

USB_MEM_ALIGNX static const uint8_t configDescriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(MSC_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, 100),
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, MSC_MAX_PACKET_SIZE, USB_STRING_PRODUCT_INDEX)
};

// One eight-digit hexadecimal field for each of the three UID words.
static char serialNumber[3 * 8 + 1];

static const char *const stringDescriptors[] = {
    [USB_STRING_LANGID_INDEX] = (const char[]){ 0x09, 0x04 },
    [USB_STRING_MFC_INDEX] = "UMIC",
    [USB_STRING_PRODUCT_INDEX] = "UMIC MSC BF",
    [USB_STRING_SERIAL_INDEX] = serialNumber,
};

static const uint8_t *getDeviceDescriptor(uint8_t speed)
{
    UNUSED(speed);
    return deviceDescriptor;
}

static const uint8_t *getConfigDescriptor(uint8_t speed)
{
    UNUSED(speed);
    return configDescriptor;
}

static const char *getStringDescriptor(uint8_t speed, uint8_t index)
{
    UNUSED(speed);
    return index < ARRAYLEN(stringDescriptors) ? stringDescriptors[index] : NULL;
}

static const struct usb_descriptor mscDescriptor = {
    .device_descriptor_callback = getDeviceDescriptor,
    .config_descriptor_callback = getConfigDescriptor,
    .string_descriptor_callback = getStringDescriptor,
};

static void mscUsbEvent(uint8_t busId, uint8_t event)
{
    UNUSED(busId);
    UNUSED(event);
}

void mscUsbInit(uint8_t busId, uintptr_t regBase)
{
    static struct usbd_interface mscInterface;

    tfp_sprintf(serialNumber, "%08X%08X%08X", (unsigned int)U_ID_0, (unsigned int)U_ID_1, (unsigned int)U_ID_2);
    usbd_desc_register(busId, &mscDescriptor);
    usbd_add_interface(busId, usbd_msc_init_intf(busId, &mscInterface, MSC_OUT_EP, MSC_IN_EP));
    usbd_initialize(busId, regBase, mscUsbEvent);
}

#endif
