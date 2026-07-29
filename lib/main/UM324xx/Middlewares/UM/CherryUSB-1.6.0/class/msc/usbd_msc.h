/*
 * Copyright (c) 2022, sakumisu
 * Copyright (c) 2024, zhihong chen
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef USBD_MSC_H
#define USBD_MSC_H

#include "usb_msc.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup USB_CORE_Exported_Types
  * @{
  */
typedef struct _USBD_STORAGE
{
  int8_t (* Init)(uint8_t lun);
  int8_t (* GetCapacity)(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
  int8_t (* IsReady)(uint8_t lun);
  int8_t (* IsWriteProtected)(uint8_t lun);
  int8_t (* Read)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* Write)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* GetMaxLun)(void);
  int8_t *pInquiry;

} USBD_StorageTypeDef;
	
/* Init msc interface driver */
struct usbd_interface *usbd_msc_init_intf(uint8_t busid, struct usbd_interface *intf,
                                          const uint8_t out_ep,
                                          const uint8_t in_ep);

void usbd_msc_get_cap(uint8_t busid, uint8_t lun, uint32_t *block_num, uint32_t *block_size);
int usbd_msc_sector_read(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length);
int usbd_msc_sector_write(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length);

void usbd_msc_set_readonly(uint8_t busid, bool readonly);
bool usbd_msc_get_popup(uint8_t busid);

void usbd_msc_polling(uint8_t busid);

#ifdef __cplusplus
}
#endif

#endif /* USBD_MSC_H */
