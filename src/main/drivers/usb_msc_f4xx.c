/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Chris Hockuba (https://github.com/conkerkh)
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "drivers/usb_msc.h"

#include "platform.h"

#ifdef USE_USB_MSC

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

void mscButtonInit(void)
{
#ifdef MSC_BUTTON
    IO_t msc_button = IOGetByTag(IO_TAG(MSC_BUTTON));
	IOInit(msc_button, OWNER_USB, 0);
#ifdef MSC_BUTTON_IPU
	IOConfigGPIO(msc_button, IOCFG_IPU);
#else
	IOConfigGPIO(msc_button, IOCFG_IPD);
#endif
#endif
}

#ifdef MSC_BUTTON
#ifdef MSC_BUTTON_IPU
#define BUTTON_STATUS IORead(msc_button) == 0
#else
#define BUTTON_STATUS IORead(msc_button) == 1
#endif
#else
#define BUTTON_STATUS 0
#endif

void mscCheck(void)
{
	//In order to exit MSC mode simply disconnect the board
#ifdef MSC_BUTTON
	IO_t msc_button = IOGetByTag(IO_TAG(MSC_BUTTON));
#endif

	while(BUTTON_STATUS);
	while(1) {
		asm("NOP");
		if (BUTTON_STATUS) {
			*((uint32_t *)0x2001FFF0) = 0xDDDD1011;
			delay(1);
			NVIC_SystemReset();
		}
	}
}

bool mscButton(void) {
#ifdef MSC_BUTTON
	IO_t msc_button = IOGetByTag(IO_TAG(MSC_BUTTON));
#endif
	return BUTTON_STATUS;
}

#endif
