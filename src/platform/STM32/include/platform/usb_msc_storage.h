#pragma once

#include <stdbool.h>

#ifdef USE_HAL_DRIVER
#include "usbd_msc.h"
typedef USBD_StorageTypeDef USBD_MSC_StorageType;
#define USBD_MSC_INQUIRY_DATA_LEN STANDARD_INQUIRY_DATA_LEN
typedef uint16_t usbd_msc_block_size_t;
#else
#include "usbd_msc_mem.h"
#include "usbd_msc_core.h"
typedef USBD_STORAGE_cb_TypeDef USBD_MSC_StorageType;
#define USBD_MSC_INQUIRY_DATA_LEN USBD_STD_INQUIRY_LENGTH
typedef uint32_t usbd_msc_block_size_t;
#endif

bool mscSdioInitDma(void);
