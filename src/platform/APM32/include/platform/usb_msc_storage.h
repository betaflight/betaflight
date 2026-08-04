#pragma once

#include <stdbool.h>

#include "usbd_msc.h"
typedef USBD_StorageTypeDef USBD_MSC_StorageType;
#define USBD_MSC_INQUIRY_DATA_LEN STANDARD_INQUIRY_DATA_LEN
typedef uint16_t usbd_msc_block_size_t;

bool mscSdioInitDma(void);
