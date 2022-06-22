/**
  ******************************************************************************
  * @file    usbd_storage_template.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    09-November-2015
  * @brief   Memory management layer
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdbool.h>

#include "platform.h"
#include "common/utils.h"

#include "usbd_storage.h"

#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/sdcard.h"
#include "drivers/usb_msc.h"

#include "pg/sdcard.h"
#include "pg/bus_spi.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USB NVIC Priority has to be lower than both DMA and SDIO priority,
 * otherwise SDIO won't be able to preempt USB.
 */

#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  0x10000
#define STORAGE_BLK_SIZ                  0x200

static int8_t STORAGE_Init (uint8_t lun);

#ifdef USE_HAL_DRIVER
static int8_t STORAGE_GetCapacity (uint8_t lun,
                           uint32_t *block_num,
                           uint16_t *block_size);
#else
static int8_t STORAGE_GetCapacity (uint8_t lun,
                           uint32_t *block_num,
                           uint32_t *block_size);
#endif

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

/* USB Mass storage Standard Inquiry Data */
static uint8_t  STORAGE_Inquirydata[] = {//36

  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
#ifdef USE_HAL_DRIVER
  (STANDARD_INQUIRY_DATA_LEN - 5),
#else
  (USBD_STD_INQUIRY_LENGTH - 5),
#endif
  0x00,
  0x00,
  0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'd', 'u', 't', ' ', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1',                     /* Version      : 4 Bytes */
};

#ifdef USE_HAL_DRIVER
USBD_StorageTypeDef USBD_MSC_MICRO_SD_SPI_fops =
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
#else
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
#endif

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the microSD card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static int8_t STORAGE_Init (uint8_t lun)
{
	UNUSED(lun);
	LED0_OFF;
	sdcard_init(sdcardConfig());
	while (sdcard_poll() == 0);
    mscSetActive();
	return 0;
}

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
#ifdef USE_HAL_DRIVER
static int8_t STORAGE_GetCapacity (uint8_t lun, uint32_t *block_num, uint16_t *block_size)
#else
static int8_t STORAGE_GetCapacity (uint8_t lun, uint32_t *block_num, uint32_t *block_size)
#endif
{
	UNUSED(lun);
	*block_num = sdcard_getMetadata()->numBlocks;
	*block_size = 512;
	return (0);
}

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static int8_t  STORAGE_IsReady (uint8_t lun)
{
	UNUSED(lun);
	int8_t ret = -1;
	if (sdcard_poll()) {
        ret = 0;
	}
	return ret;
}

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static int8_t  STORAGE_IsWriteProtected (uint8_t lun)
{
  UNUSED(lun);
  return  0;
}

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
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
/*******************************************************************************
* Function Name  : Write_Memory
* Description    : Handle the Write operation to the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
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
/*******************************************************************************
* Function Name  : Write_Memory
* Description    : Handle the Write operation to the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static int8_t STORAGE_GetMaxLun (void)
{
  return (STORAGE_LUN_NBR - 1);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

