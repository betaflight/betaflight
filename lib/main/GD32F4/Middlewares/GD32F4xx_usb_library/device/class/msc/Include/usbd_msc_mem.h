/*!
    \file    usbd_msc_mem.h
    \brief   header file for storage memory

    \version 2024-12-20, V3.3.1, firmware for GD32F4xx
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#ifndef USBD_MSC_MEM_H
#define USBD_MSC_MEM_H

#include "usbd_conf.h"

#define USBD_STD_INQUIRY_LENGTH          36U               /*!< standard inquiry length */

typedef struct {
    int8_t (*mem_init)(uint8_t lun);
    int8_t (*mem_getcapacity) (uint8_t lun, uint32_t *block_num, uint32_t *block_size);
    int8_t (*mem_ready)(uint8_t lun);
    int8_t (*mem_protected)(uint8_t lun);
    int8_t (*mem_read)(uint8_t lun, uint8_t *buf, uint32_t block_addr, uint16_t block_len);
    int8_t (*mem_write)(uint8_t lun, uint8_t *buf, uint32_t block_addr, uint16_t block_len);
    int8_t (*mem_maxlun)(void);

    uint8_t *mem_toc_data;                                 /*!< memory TOC command data pointer */
    uint8_t *mem_inquiry_data[MEM_LUN_NUM];                /*!< memory inquiry data buff */
    uint32_t mem_block_size[MEM_LUN_NUM];                  /*!< memory block size buff */
    uint32_t mem_block_len[MEM_LUN_NUM];                   /*!< memory block length buff */                 /*!< memory block length buff */
}usbd_mem_cb;

extern usbd_mem_cb *usbd_mem_fops;

typedef struct _USBD_STORAGE
{
  int8_t (* Init) (uint8_t lun);
  int8_t (* GetCapacity) (uint8_t lun, uint32_t *block_num, uint32_t *block_size);
  int8_t (* IsReady) (uint8_t lun);
  int8_t (* IsWriteProtected) (uint8_t lun);
  int8_t (* Read) (uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* Write)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* GetMaxLun)(void);
  int8_t *pInquiry;
  
}USBD_STORAGE_cb_TypeDef;

#endif /* USBD_MSC_MEM_H */
