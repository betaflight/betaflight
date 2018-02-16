/*
 * usb_msc_f4xx.c
 *
 *  Created on: Feb 15, 2018
 *      Author: khockuba
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "drivers/usb_msc.h"

#include "platform.h"

#ifdef USB_MSC

#include "build/build_config.h"

#include "common/utils.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/sdmmc_sdio.h"
#include "drivers/light_led.h"

#if defined(STM32F4)
#include "usb_core.h"
#include "usbd_cdc_vcp.h"
#include "usb_io.h"
#elif defined(STM32F7)
#include "vcp_hal/usbd_cdc_interface.h"
#include "usb_io.h"
USBD_HandleTypeDef USBD_Device;
#else
#include "usb_core.h"
#include "usb_init.h"
#include "hw_config.h"
#endif

#include "usbd_msc_core.h"

uint8_t startMsc(void)
{
	ledInit(statusLedConfig());

	//Start USB
	usbGenerateDisconnectPulse();

	IOInit(IOGetByTag(IO_TAG(PA11)), OWNER_USB, 0);
	IOInit(IOGetByTag(IO_TAG(PA12)), OWNER_USB, 0);
	USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &MSC_desc, &USBD_MSC_cb, &USR_cb);

	// NVIC configuration for SYSTick
	NVIC_DisableIRQ(SysTick_IRQn);
	NVIC_SetPriority(SysTick_IRQn, NVIC_BUILD_PRIORITY(0, 0));
	NVIC_EnableIRQ(SysTick_IRQn);

	return 0;
}

#endif
