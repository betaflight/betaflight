/*
 * Derived from github.com/fetisov/emfat/project/StorageMode.c
 */

/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 by Sergey Fetisov <fsenok@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdbool.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/light_led.h"
#include "drivers/time.h"
#include "drivers/flash.h"

#include "io/flashfs.h"

#include "pg/flash.h"

#include "usbd_storage.h"
#include "usbd_storage_emfat.h"
#include "emfat_file.h"


#define STORAGE_LUN_NBR 1

static const uint8_t STORAGE_Inquirydata[] = 
{
    0x00, 0x80, 0x02, 0x02,
#ifdef USE_HAL_DRIVER
    (STANDARD_INQUIRY_DATA_LEN - 5),
#else
    (USBD_STD_INQUIRY_LENGTH - 5),
#endif
    0x00, 0x00, 0x00,
    'B', 'E', 'T', 'A', 'F', 'L', 'T', ' ', // Manufacturer : 8 bytes
    'O', 'n', 'b', 'o', 'a', 'r', 'd', ' ', // Product      : 16 Bytes
    'F', 'l', 'a', 's', 'h', ' ', ' ', ' ', //
    ' ', ' ', ' ' ,' ',                     // Version      : 4 Bytes
};

static int8_t STORAGE_Init(uint8_t lun)
{
    UNUSED(lun);

    LED0_ON;

#ifdef USE_FLASHFS 
#ifdef USE_FLASH_CHIP
    flashInit(flashConfig());
#endif
    flashfsInit();
#endif
    emfat_init_files();

    delay(1000);

    LED0_OFF;

    return 0;
}

#ifdef USE_HAL_DRIVER
static int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
#else
static int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num, uint32_t *block_size)
#endif
{
    UNUSED(lun);
    *block_size = 512;
    *block_num = emfat.disk_sectors;
    return 0;
}

static int8_t STORAGE_IsReady(uint8_t lun)
{
    UNUSED(lun);
    return 0;
}

static int8_t STORAGE_IsWriteProtected(uint8_t lun)
{
    UNUSED(lun);
    return 1;
}

static int8_t STORAGE_Read(
    uint8_t lun,        // logical unit number
    uint8_t *buf,       // Pointer to the buffer to save data
    uint32_t blk_addr,  // address of 1st block to be read
    uint16_t blk_len)   // nmber of blocks to be read
{
    UNUSED(lun);
    LED0_ON;
    emfat_read(&emfat, buf, blk_addr, blk_len);
    LED0_OFF;
    return 0;
}

static int8_t STORAGE_Write(uint8_t lun,
    uint8_t *buf,
    uint32_t blk_addr,
    uint16_t blk_len)
{
    UNUSED(lun);
    UNUSED(buf);
    UNUSED(blk_addr);
    UNUSED(blk_len);

    return 1;
}

static int8_t STORAGE_GetMaxLun(void)
{
  return (STORAGE_LUN_NBR - 1);
}

#ifdef USE_HAL_DRIVER
USBD_StorageTypeDef
#else
USBD_STORAGE_cb_TypeDef
#endif
USBD_MSC_EMFAT_fops =
{
    STORAGE_Init,
    STORAGE_GetCapacity,
    STORAGE_IsReady,
    STORAGE_IsWriteProtected,
    STORAGE_Read,
    STORAGE_Write,
    STORAGE_GetMaxLun,
    (int8_t *)STORAGE_Inquirydata,
};
