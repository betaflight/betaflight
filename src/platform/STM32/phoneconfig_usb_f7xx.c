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
 * phone-config usb bring-up (stm32f7): brings the usb device up as a cdc-ncm network gadget instead of
 * the normal cdc-acm serial port. modelled on mscStart() in usb_msc_f7xx.c.
 */

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_PHONE_CONFIG

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/usb_io.h"

#include "pg/usb.h"

#include "vcp_hal/usbd_cdc_interface.h"

#include "usbd_core.h"
#include "vcp_hal/usbd_cdc_ncm.h"

#include "drivers/phoneconfig.h"
#include "io/phoneconfig_net.h"

extern USBD_HandleTypeDef USBD_Device;

// iad/miscellaneous device descriptor with its own product id, ios is strict about this
static const uint8_t ncmDeviceDescriptor[18] = {
    0x12,                   // bLength
    0x01,                   // bDescriptorType: Device
    0x00, 0x02,             // bcdUSB 2.00
    0xEF,                   // bDeviceClass: Miscellaneous Device
    0x02,                   // bDeviceSubClass: Common Class
    0x01,                   // bDeviceProtocol: Interface Association Descriptor
    0x40,                   // bMaxPacketSize0: 64
    0x83, 0x04,             // idVendor: 0x0483 (STMicroelectronics)
    0x42, 0x57,             // idProduct: 0x5742 (distinct from serial 0x5740 / ECM 0x5741)
    0x00, 0x02,             // bcdDevice 2.00
    0x01,                   // iManufacturer
    0x02,                   // iProduct
    0x03,                   // iSerialNumber
    0x01,                   // bNumConfigurations
};

static uint8_t *ncmDeviceDescriptorCb(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(ncmDeviceDescriptor);
    return (uint8_t *)ncmDeviceDescriptor;
}

// reuse betaflight's vcp string-descriptor providers
extern uint8_t *USBD_VCP_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
extern uint8_t *USBD_VCP_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
extern uint8_t *USBD_VCP_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
extern uint8_t *USBD_VCP_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
extern uint8_t *USBD_VCP_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
extern uint8_t *USBD_VCP_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

static USBD_DescriptorsTypeDef ncmDescriptors = {
    ncmDeviceDescriptorCb,
    USBD_VCP_LangIDStrDescriptor,
    USBD_VCP_ManufacturerStrDescriptor,
    USBD_VCP_ProductStrDescriptor,
    USBD_VCP_SerialStrDescriptor,
    USBD_VCP_ConfigStrDescriptor,
    USBD_VCP_InterfaceStrDescriptor,
};

// network glue: bridge the ncm class to the lwip stack in phoneconfig_net.c
static int8_t ncmGlueInit(void) { return 0; }
static int8_t ncmGlueDeInit(void) { return 0; }
static int8_t ncmGlueReceive(uint8_t *frame, uint32_t length) { phoneConfigNetRxFrame(frame, length); return 0; }
static void   ncmGlueLinkChange(uint8_t up) { phoneConfigNetLinkChange(up); }
// the host's usb-ncm interface mac (imacaddress descriptor), becomes the phone's ethernet mac
static const uint8_t ncmMacString[] = "020000000001";
// the fc's own lwip mac. must differ from the host mac above or ios drops the gateway's arp reply as
// bogus (linux tolerates the clash, ios doesn't), so differ by the last byte.
static const uint8_t ncmMacBytes[6] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x02 };

// lwip netif tx. mspFlushTx() only calls here when the in endpoint is idle, so the ntb is taken at once
void phoneConfigUsbTransmitFrame(const uint8_t *frame, uint16_t length)
{
    USBD_CDC_NCM_TransmitFrame(&USBD_Device, frame, length);
}

bool phoneConfigUsbTxBusy(void)
{
    return USBD_CDC_NCM_IsTxBusy() != 0;
}

// drain deferred usb-ncm rx into lwip, called from the main loop
void phoneConfigUsbProcess(void)
{
    USBD_CDC_NCM_Process(&USBD_Device);
}

// the st usb low level is not re-entrant, so mask the otg-fs interrupt while the main loop drives it
void phoneConfigUsbLock(void)
{
    NVIC_DisableIRQ(OTG_FS_IRQn);
    __DSB();
    __ISB();
}

void phoneConfigUsbUnlock(void)
{
    NVIC_EnableIRQ(OTG_FS_IRQn);
}

static USBD_CDC_NCM_ItfTypeDef ncmFops = {
    ncmGlueInit,
    ncmGlueDeInit,
    ncmGlueReceive,
    ncmGlueLinkChange,
    ncmMacString,
};

uint8_t phoneConfigUsbStart(void)
{
    usbGenerateDisconnectPulse();

    IOInit(IOGetByTag(IO_TAG(PA11)), OWNER_USB, 0);
    IOInit(IOGetByTag(IO_TAG(PA12)), OWNER_USB, 0);

    USBD_Init(&USBD_Device, &ncmDescriptors, 0);

    // bf's fifo layout only allocates tx fifos for ep0/ep1 (its vcp never transmits on the notification
    // endpoint). ncm does (link-up notification on ep2), and without a tx fifo there it hard-faults, so
    // re-partition the 320-word otg-fs fifo ram to add one. words: 0x80+0x20+0x80+0x10 = 0x130.
    PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)USBD_Device.pData;
    HAL_PCDEx_SetRxFiFo(hpcd, 0x80);
    HAL_PCDEx_SetTxFiFo(hpcd, 0, 0x20);   // ep0 control
    HAL_PCDEx_SetTxFiFo(hpcd, 1, 0x80);   // ep1 bulk data (ntbs)
    HAL_PCDEx_SetTxFiFo(hpcd, 2, 0x10);   // ep2 interrupt, link notifications (was missing)

    USBD_RegisterClass(&USBD_Device, USBD_CDC_NCM_CLASS);
    USBD_CDC_NCM_RegisterInterface(&USBD_Device, &ncmFops);
    USBD_Start(&USBD_Device);

    // net stack starts later in phoneConfigNetStart(), after mspSerialInit()

    NVIC_DisableIRQ(SysTick_IRQn);
    NVIC_SetPriority(SysTick_IRQn, NVIC_BUILD_PRIORITY(0, 0));
    NVIC_EnableIRQ(SysTick_IRQn);

    return 0;
}

// bring up lwip and the msp-over-tcp server. called from init() after mspSerialInit()
void phoneConfigNetStart(void)
{
    phoneConfigNetInit(ncmMacBytes);
}

#endif // USE_PHONE_CONFIG
