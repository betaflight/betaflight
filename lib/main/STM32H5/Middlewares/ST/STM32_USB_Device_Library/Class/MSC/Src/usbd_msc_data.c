/**
  ******************************************************************************
  * @file    usbd_msc_data.c
  * @author  MCD Application Team
  * @brief   This file provides all the vital inquiry pages and sense data.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}{adafruit}_sd.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_msc_data.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup MSC_DATA
  * @brief Mass storage info/data module
  * @{
  */

/** @defgroup MSC_DATA_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup MSC_DATA_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup MSC_DATA_Private_Macros
  * @{
  */
/**
  * @}
  */


/** @defgroup MSC_DATA_Private_Variables
  * @{
  */

/* USB Mass storage Page 0 Inquiry Data */
uint8_t MSC_Page00_Inquiry_Data[LENGTH_INQUIRY_PAGE00] =
{
  0x00,
  0x00,
  0x00,
  (LENGTH_INQUIRY_PAGE00 - 4U),
  0x00,
  0x80
};

/* USB Mass storage VPD Page 0x80 Inquiry Data for Unit Serial Number */
uint8_t MSC_Page80_Inquiry_Data[LENGTH_INQUIRY_PAGE80] =
{
  0x00,
  0x80,
  0x00,
  LENGTH_INQUIRY_PAGE80,
  0x20,     /* Put Product Serial number */
  0x20,
  0x20,
  0x20
};

/* USB Mass storage sense 6 Data */
uint8_t MSC_Mode_Sense6_data[MODE_SENSE6_LEN] =
{
  0x22,
  0x00,
  0x00,
  0x00,
  0x08,
  0x12,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00
};


/* USB Mass storage sense 10  Data */
uint8_t MSC_Mode_Sense10_data[MODE_SENSE10_LEN] =
{
  0x00,
  0x26,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x08,
  0x12,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00
};
/**
  * @}
  */


/** @defgroup MSC_DATA_Private_FunctionPrototypes
  * @{
  */
/**
  * @}
  */


/** @defgroup MSC_DATA_Private_Functions
  * @{
  */

/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

