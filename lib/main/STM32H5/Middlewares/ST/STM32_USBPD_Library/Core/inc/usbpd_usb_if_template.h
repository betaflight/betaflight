/**
  ******************************************************************************
  * @file    usbpd_usb_if.h
  * @author  MCD Application Team
  * @brief   This file contains the usb if API.
    ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifndef USBPD_USBIF_H_
#define USBPD_USBIF_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/** @addtogroup STM32_USBPD_USB_IF
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup USBPD_CORE_USBIF_Exported_Functions USBPD USB IF Exported Functions
  * @{
  */

/**
  * @brief  Initialize the usb layer
  * @param  None
  * @retval 0 if initialization is success
  */
int32_t USBPD_USBIF_Init(void);

/**
  * @brief  start the USB device
  * @param  PortNum Index of current used port
  * @retval None
  */
void USBPD_USBIF_DeviceStart(uint32_t PortNum);
/**
  * @brief  stop the USB device
  * @param  PortNum Index of current used port
  * @retval None
  */
void USBPD_USBIF_DeviceStop(uint32_t PortNum);

/**
  * @brief  start the USB host
  * @param  PortNum Index of current used port
  * @retval None
  */
void USBPD_USBIF_HostStart(uint32_t PortNum);

/**
  * @brief  stop the USB host
  * @param  PortNum Index of current used port
  * @retval None
  */
void USBPD_USBIF_HostStop(uint32_t PortNum);

/**
  * @brief  start the billboard class
  * @param  PortNum Index of current used port
  * @retval None
  */
void USBPD_USBIF_DeviceBillboard(uint32_t PortNum);

/**
  * @brief  swap from device to host
  * @param  PortNum Index of current used port
  * @retval None
  */
void USBPD_USBIF_Swap2Host(uint32_t PortNum);

/**
  * @brief  swap from host to device
  * @param  PortNum Index of current used port
  * @retval None
  */
void USBPD_USBIF_Swap2Device(uint32_t PortNum);

/**
  * @brief  update the BOS information of the device
  * @param  PortNum Index of current used port
  * @param  DataPtr pointer on the data
  * @retval None
  */
void USBPD_USBIF_DeviceSetBOSInfo(uint32_t PortNum, void *DataPtr);

/**
  * @brief  update the VDM information presented by the billboard class
  * @param  PortNum Index of current used port
  * @param  DataPtr pointer on the data
  * @retval None
  */
void USBPD_USBIF_DeviceSetVDMInfo(uint32_t PortNum, void *DataPtr);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* USBPD_USBIF_H_ */

