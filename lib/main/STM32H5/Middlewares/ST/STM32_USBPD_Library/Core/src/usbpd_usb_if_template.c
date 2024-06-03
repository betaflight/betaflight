/**
  ******************************************************************************
  * @file    usbpd_usb_if.c
  * @author  MCD Application Team
  * @brief   This file contains the usb if functions.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbpd_usb_if.h"
#if defined(_TRACE)
#include "usbpd_trace.h"
#include <stdio.h>
#endif /* _TRACE */
/* USB include files ----------------------------------------------------------*/

/** @addtogroup STM32_USBPD_USBIF
  * @{
    @note this file contains all the API to use if you manage the USB data.
	The API call is managed through the notification function (USBPD_DPM_Notification)
	so you will need to add the following code inside your application.

	void USBPD_DPM_Notification(uint8_t PortNum, USBPD_NotifyEventValue_TypeDef EventVal)
	{
		switch(EventVal)
		{
    ....
		case USBPD_NOTIFY_USBSTACK_START:
		{
			if (USBPD_PORTDATAROLE_DFP == DPM_Params[PortNum].PE_DataRole)
			{
				USBPD_USBIF_HostStart(PortNum);
			}
			else
			{
				USBPD_USBIF_DeviceStart(PortNum);
			}
		break;
		}
		case USBPD_NOTIFY_USBSTACK_STOP:
		{
		 if (USBPD_PORTDATAROLE_DFP == DPM_Params[PortNum].PE_DataRole)
		 {
			 USBPD_USBIF_HostStop(PortNum);
		 }
		 else
		 {
		    USBPD_USBIF_DeviceStop(PortNum);
		 }
		 break;
		}
    case USBPD_NOTIFY_DATAROLESWAP_DFP :
		{
		  USBPD_USBIF_Swap2Host(PortNum);
      break;
		}
	  case USBPD_NOTIFY_DATAROLESWAP_UFP :
		{
		  USBPD_USBIF_Swap2Device(PortNum);
		  break;
		}
	}
  */

/* Private enums -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#if defined(_TRACE)
USBPD_USBIF_TRACE(__PORT__, __DATA__) \
  USBPD_TRACE_Add(USBPD_TRACE_DEBUG, (__PORT__), 0, (uint8_t*)(__DATA__), sizeof(__DATA__));
#else
USBPD_USBIF_TRACE(__PORT__, __DATA__)
#endif /* _TRACE */
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
/**
  * @brief  Initialize billboard driver
  * @param  None
  * @retval status
  */
int32_t USBPD_USBIF_Init(void)
{
  /* USER CODE BEGIN USBPD_USBIF_Init */	
  USBPD_USBIF_TRACE(0, "USB_IF init");
  return 0;
  /* USER END BEGIN USBPD_USBIF_Init */
}

/** @addtogroup USBPD_CORE_USBIF_Exported_Functions
  * @{
  */
void USBPD_USBIF_DeviceStart(uint32_t PortNum)
{
  /* USER CODE BEGIN USBPD_USBIF_DeviceStart */	
  USBPD_USBIF_TRACE(PortNum, "USBIF Device start");
  /* USER END BEGIN USBPD_USBIF_DeviceStart */
}

void USBPD_USBIF_DeviceStop(uint32_t PortNum)
{
  /* USER CODE BEGIN USBPD_USBIF_DeviceStop */	
  USBPD_USBIF_TRACE(PortNum, "USBIF Device stop");
  /* USER END BEGIN USBPD_USBIF_DeviceStop */
}

void USBPD_USBIF_HostStart(uint32_t PortNum)
{
  /* USER CODE BEGIN USBPD_USBIF_HostStart */	
  USBPD_USBIF_TRACE(PortNum, "USBIF host start");
  /* USER END BEGIN USBPD_USBIF_HostStart */
}

void USBPD_USBIF_HostStop(uint32_t PortNum)
{
  /* USER CODE BEGIN USBPD_USBIF_HostStop */	
  USBPD_USBIF_TRACE(PortNum, "USBIF host stop");
  /* USER END BEGIN USBPD_USBIF_HostStop */
}

void USBPD_USBIF_DeviceBillboard(uint32_t PortNum)
{
  /* USER CODE BEGIN USBPD_USBIF_DeviceBillboard */	
  USBPD_USBIF_TRACE(PortNum, "USBIF device billboard");
  /* USER END BEGIN USBPD_USBIF_DeviceBillboard */
}

void USBPD_USBIF_Swap2Host(uint32_t PortNum)
{
  /* USER CODE BEGIN USBPD_USBIF_Swap2Host */	
  USBPD_USBIF_TRACE(PortNum, "USBIF swap to host");
  /* USER END BEGIN USBPD_USBIF_Swap2Host */
}

void USBPD_USBIF_Swap2Device(uint32_t PortNum)
{
  /* USER CODE BEGIN USBPD_USBIF_Swap2Device */	
  USBPD_USBIF_TRACE(PortNum, "USBIF swap to device");
  /* USER END BEGIN USBPD_USBIF_Swap2Device */
}

void USBPD_USBIF_DeviceSetBOSInfo(uint32_t PortNum, void *DataPtr)
{
  /* USER CODE BEGIN USBPD_USBIF_DeviceSetBOSInfo */	
  USBPD_USBIF_TRACE(PortNum, "USBIF set BOS info");
  /* USER END BEGIN USBPD_USBIF_DeviceSetBOSInfo */
}

void USBPD_USBIF_DeviceSetVDMInfo(uint32_t PortNum, void *DataPtr)
{
  /* USER CODE BEGIN USBPD_USBIF_DeviceSetVDMInfo */	
  USBPD_USBIF_TRACE(PortNum, "USBIF set VDM info");
  /* USER END BEGIN USBPD_USBIF_DeviceSetVDMInfo */
}

/**
  * @}
  */

/** @addtogroup USBPD_CORE_USBIF_Private_Functions
  * @{
  */

/**
  * @}
  */

