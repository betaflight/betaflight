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

#if defined(USE_USB_MSC) && defined(USE_SDCARD_SPI)

#include "common/utils.h"
#include "build/build_config.h"
#include "msc/usbd_storage.h"


#include "blackbox/blackbox.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/sdcard.h"
#include "drivers/usb_io.h"

#include "drivers/usb_msc.h"

#include "pg/sdcard.h"
#include "pg/bus_spi.h"



#define STORAGE_LUN_NBR                  1
// #define STORAGE_BLK_NBR                  0x10000
// #define STORAGE_BLK_SIZ                  0x200


static int8_t STORAGE_Init (uint8_t lun);

static int8_t STORAGE_GetCapacity (uint8_t lun,
                           uint32_t *block_num,
                           uint32_t *block_size);

static int8_t  STORAGE_IsReady (uint8_t lun);

static int8_t  STORAGE_IsWriteProtected (uint8_t lun);

static int8_t STORAGE_Read (uint8_t lun,
                        uint8_t *buf,
                        uint32_t blk_addr,
                        uint16_t blk_len);


static int8_t STORAGE_Write (uint8_t lun,
                        uint8_t *buf,
                        uint32_t blk_addr,
                        uint16_t blk_len);


                        static int8_t STORAGE_GetMaxLun (void);


static uint8_t  STORAGE_Inquirydata[] = {//36

  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (USBD_STD_INQUIRY_LENGTH - 5),
  0x00,
  0x00,
  0x00,
  'W', 'C', 'H', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'd', 'u', 't', ' ', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1',                     /* Version      : 4 Bytes */
};


USBD_STORAGE_cb_TypeDef USBD_MSC_MICRO_SD_SPI_fops =
{
  STORAGE_Init,
  STORAGE_GetCapacity,
  STORAGE_IsReady,
  STORAGE_IsWriteProtected,
  STORAGE_Read,
  STORAGE_Write,
  STORAGE_GetMaxLun,
  (int8_t*)STORAGE_Inquirydata,
};


static int8_t STORAGE_Init (uint8_t lun)
{
	UNUSED(lun);
	LED0_OFF;
	sdcard_init(sdcardConfig());
	while (sdcard_poll() == 0);
    mscSetActive();
	return 0;
}


static int8_t STORAGE_GetCapacity (uint8_t lun, uint32_t *block_num, uint32_t *block_size)
{
	UNUSED(lun);
  
  *block_num = sdcard_getMetadata()->numBlocks;
	*block_size = 512;
	return (0);
}

static int8_t  STORAGE_IsReady (uint8_t lun)
{
	UNUSED(lun);
	int8_t ret = -1;
	if (sdcard_poll()) {
        ret = 0;
	}
	return ret;
}


static int8_t  STORAGE_IsWriteProtected (uint8_t lun)
{
  UNUSED(lun);
  return  0;
}



static int8_t STORAGE_Read (uint8_t lun,
                 uint8_t *buf,
                 uint32_t blk_addr,
                 uint16_t blk_len)
{
	UNUSED(lun);
	for (int i = 0; i < blk_len; i++) {
		while (sdcard_readBlock(blk_addr + i, buf + (512 * i), NULL, 0) == 0);
		while (sdcard_poll() == 0);
	}
    mscSetActive();
	return 0;
}


static int8_t STORAGE_Write (uint8_t lun,
                  uint8_t *buf,
                  uint32_t blk_addr,
                  uint16_t blk_len)
{
	UNUSED(lun);
	for (int i = 0; i < blk_len; i++) {
		while (sdcard_writeBlock(blk_addr + i, buf + (i * 512), NULL, 0) != SDCARD_OPERATION_IN_PROGRESS) {
			sdcard_poll();
		}
		while (sdcard_poll() == 0);
	}
    mscSetActive();
	return 0;
}

static int8_t STORAGE_GetMaxLun (void)
{
  return (STORAGE_LUN_NBR - 1);
}


#endif
