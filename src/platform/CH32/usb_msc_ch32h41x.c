/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Author: Chris Hockuba (https://github.com/conkerkh)
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#if defined(USE_USB_MSC)

#include "build/build_config.h"

#include "common/utils.h"

#include "blackbox/blackbox.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/usb_msc.h"

#include "msc/usbd_storage.h"
#include "msc/usbd_storage_emfat.h"

#include "pg/sdcard.h"
#include "pg/usb.h"

// #include "usb_conf.h"
// #include "usb_core.h"
// #include "usbd_int.h"
// #include "msc_class.h"
// #include "msc_desc.h"
// #include "msc_diskio.h"

#include "usbd_core.h"
#include "usbd_msc.h"

#include "drivers/usb_io.h"

#include "usb_ch32h41x_usbhs_reg.h"

//0xE205 -- DFU
#define MSC_IN_EP  0x81
#define MSC_OUT_EP 0x01

#define USBD_VID           0x1A86
#define USBD_PID           0xFE10 
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

#define USB_CONFIG_SIZE (9 + MSC_DESCRIPTOR_LEN)

#define MSC_MAX_MPS 64

static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0200, 0x01)
};

static const uint8_t config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_SELF_POWERED, USBD_MAX_POWER),
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, MSC_MAX_MPS, 0x05)
};

static const uint8_t device_quality_descriptor[] = {
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x00,
    0x00,
};

static const char *string_descriptors[] = {
    (const char[]){ 0x09, 0x04 },                   /* Langid */
    "Betaflight",                                   /* Manufacturer */
    "Betaflight FC Mass Storage (FS Mode)",         /* Product */
    "2025123456",                                   /* Serial Number */
    "Betaflight CH32H415", 
    "Betaflight CH32H415"
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void) speed;
    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    (void) speed;
    return config_descriptor;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void) speed;
    return device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void) speed;    
    if (index > 5) {
        return NULL;
    }
    return string_descriptors[index];
}

const struct usb_descriptor msc_ram_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback
};


static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    (void) busid;
    switch (event) {
        case USBD_EVENT_RESET:
            break;
        case USBD_EVENT_CONNECTED:
            break;
        case USBD_EVENT_DISCONNECTED:
            break;
        case USBD_EVENT_RESUME:
            break;
        case USBD_EVENT_SUSPEND:
            break;
        case USBD_EVENT_CONFIGURED:
            break;
        case USBD_EVENT_SET_REMOTE_WAKEUP:
            break;
        case USBD_EVENT_CLR_REMOTE_WAKEUP:
            break;

        default:
            break;
    }
}


static struct usbd_interface intf0;

void msc_ram_init(uint8_t busid, uintptr_t reg_base)
{
    USBD_STORAGE_fops->Init(0);

    usbd_desc_register(busid, &msc_ram_descriptor);
    usbd_add_interface(busid, usbd_msc_init_intf(busid, &intf0, MSC_OUT_EP, MSC_IN_EP));
    usbd_initialize(busid, reg_base, usbd_event_handler);
}


static void msc_usb_gpio_config(void)
{
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB, ENABLE);
    //remap
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
}

static void msc_usb_clk_config(void)
{
        //disable first
        RCC_HBPeriphClockCmd(RCC_HBPeriph_USBHS, DISABLE);
        RCC_UTMIcmd(DISABLE);
        if((RCC->PLLCFGR & RCC_SYSPLL_SEL) != RCC_SYSPLL_USBHS)
        {
            RCC_USBHS_PLLCmd(DISABLE);
        }


    if((RCC->PLLCFGR & RCC_SYSPLL_SEL) != RCC_SYSPLL_USBHS)
    {
        /* Initialize USBHS 480M PLL */
        RCC_USBHS_PLLCmd(DISABLE);
        RCC_USBHSPLLCLKConfig(RCC_USBHSPLLSource_HSE);
        RCC_USBHSPLLReferConfig(RCC_USBHSPLLRefer_25M);
        RCC_USBHSPLLClockSourceDivConfig(RCC_USBHSPLL_IN_Div1);
        RCC_USBHS_PLLCmd(ENABLE);
    }
    /* Enable UTMI Clock */
    RCC_UTMIcmd(ENABLE);
    /* Enable USBHS Clock */
    RCC_HBPeriphClockCmd(RCC_HBPeriph_USBHS, ENABLE);
}

uint8_t mscStart(void)
{
    //gpio remap
    msc_usb_gpio_config( );

    usbGenerateDisconnectPulse();

    IOInit(IOGetByTag(IO_TAG(PB8)), OWNER_USB, 0);
    IOInit(IOGetByTag(IO_TAG(PB9)), OWNER_USB, 0);

    switch (blackboxConfig()->device) {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        switch (sdcardConfig()->mode) {
#ifdef USE_SDCARD_SDIO
        case SDCARD_MODE_SDIO:
            USBD_STORAGE_fops = &USBD_MSC_MICRO_SDIO_fops;
            break;
#endif
#ifdef USE_SDCARD_SPI
        case SDCARD_MODE_SPI:
            USBD_STORAGE_fops = &USBD_MSC_MICRO_SD_SPI_fops;
            break;
#endif
        default:
            return 1;
        }
        break;
#endif

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        USBD_STORAGE_fops = &USBD_MSC_EMFAT_fops;
        break;
#endif
    default:
        return 1;
    }

    //usb clk init
    msc_usb_clk_config( ); 
    usb_rxsof_handler = NULL;
    msc_ram_init(0, 0);

    NVIC_SetPriority(USBHS_IRQn,NVIC_PRIO_USB) ;
    NVIC_EnableIRQ(USBHS_IRQn);

    NVIC_DisableIRQ(SysTick1_IRQn);
    asm("fence.i");
    NVIC_SetPriority(SysTick1_IRQn, 0);
    NVIC_EnableIRQ(SysTick1_IRQn);

    return 0;
}

void mscTask(void)
{
    // Nothing to do here
}

static uint32_t g_block_size = 0;
void usbd_msc_get_cap(uint8_t busid, uint8_t lun, uint32_t *block_num, uint32_t *block_size)
{
    (void)busid;
    USBD_STORAGE_fops->GetCapacity(lun, block_num, block_size);
    g_block_size = *block_size;
}


int usbd_msc_sector_read(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
    (void)busid;
    return USBD_STORAGE_fops->Read(lun, buffer, sector, length / g_block_size);
}


int usbd_msc_sector_write(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
   (void)busid;
   return  USBD_STORAGE_fops->Write(lun, buffer, sector, length / g_block_size);
}
#endif
