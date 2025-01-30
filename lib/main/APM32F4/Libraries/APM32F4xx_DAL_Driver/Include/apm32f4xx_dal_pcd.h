/**
  *
  * @file    apm32f4xx_dal_pcd.h
  * @brief   Header file of PCD DAL module.
  *
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without modification, 
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
  * OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The original code has been modified by Geehy Semiconductor.
  *
  * Copyright (c) 2016 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_PCD_H
#define APM32F4xx_DAL_PCD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_ddl_usb.h"

#if defined (USB_OTG_FS) || defined (USB_OTG_HS)

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup PCD
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup PCD_Exported_Types PCD Exported Types
  * @{
  */

/**
  * @brief  PCD State structure definition
  */
typedef enum
{
  DAL_PCD_STATE_RESET   = 0x00,
  DAL_PCD_STATE_READY   = 0x01,
  DAL_PCD_STATE_ERROR   = 0x02,
  DAL_PCD_STATE_BUSY    = 0x03,
  DAL_PCD_STATE_TIMEOUT = 0x04
} PCD_StateTypeDef;

/* Device LPM suspend state */
typedef enum
{
  LPM_L0 = 0x00, /* on */
  LPM_L1 = 0x01, /* LPM L1 sleep */
  LPM_L2 = 0x02, /* suspend */
  LPM_L3 = 0x03, /* off */
} PCD_LPM_StateTypeDef;

typedef enum
{
  PCD_LPM_L0_ACTIVE = 0x00, /* on */
  PCD_LPM_L1_ACTIVE = 0x01, /* LPM L1 sleep */
} PCD_LPM_MsgTypeDef;

typedef enum
{
  PCD_BCD_ERROR                     = 0xFF,
  PCD_BCD_CONTACT_DETECTION         = 0xFE,
  PCD_BCD_STD_DOWNSTREAM_PORT       = 0xFD,
  PCD_BCD_CHARGING_DOWNSTREAM_PORT  = 0xFC,
  PCD_BCD_DEDICATED_CHARGING_PORT   = 0xFB,
  PCD_BCD_DISCOVERY_COMPLETED       = 0x00,

} PCD_BCD_MsgTypeDef;

#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
typedef USB_OTG_GlobalTypeDef  PCD_TypeDef;
typedef USB_OTG_CfgTypeDef     PCD_InitTypeDef;
typedef USB_OTG_EPTypeDef      PCD_EPTypeDef;
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */

/**
  * @brief  PCD Handle Structure definition
  */
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
typedef struct __PCD_HandleTypeDef
#else
typedef struct
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
{
  PCD_TypeDef             *Instance;   /*!< Register base address             */
  PCD_InitTypeDef         Init;        /*!< PCD required parameters           */
  __IO uint8_t            USB_Address; /*!< USB Address                       */
  PCD_EPTypeDef           IN_ep[16];   /*!< IN endpoint parameters            */
  PCD_EPTypeDef           OUT_ep[16];  /*!< OUT endpoint parameters           */
  DAL_LockTypeDef         Lock;        /*!< PCD peripheral status             */
  __IO PCD_StateTypeDef   State;       /*!< PCD communication state           */
  __IO  uint32_t          ErrorCode;   /*!< PCD Error code                    */
  uint32_t                Setup[12];   /*!< Setup packet buffer               */
  PCD_LPM_StateTypeDef    LPM_State;   /*!< LPM State                         */
  uint32_t                BESL;
  uint32_t                FrameNumber; /*!< Store Current Frame number        */


  uint32_t lpm_active;                 /*!< Enable or disable the Link Power Management .
                                       This parameter can be set to ENABLE or DISABLE        */

  uint32_t battery_charging_active;    /*!< Enable or disable Battery charging.
                                       This parameter can be set to ENABLE or DISABLE        */
  void                    *pData;      /*!< Pointer to upper stack Handler */

#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
  void (* SOFCallback)(struct __PCD_HandleTypeDef *hpcd);                              /*!< USB OTG PCD SOF callback                */
  void (* SetupStageCallback)(struct __PCD_HandleTypeDef *hpcd);                       /*!< USB OTG PCD Setup Stage callback        */
  void (* ResetCallback)(struct __PCD_HandleTypeDef *hpcd);                            /*!< USB OTG PCD Reset callback              */
  void (* SuspendCallback)(struct __PCD_HandleTypeDef *hpcd);                          /*!< USB OTG PCD Suspend callback            */
  void (* ResumeCallback)(struct __PCD_HandleTypeDef *hpcd);                           /*!< USB OTG PCD Resume callback             */
  void (* ConnectCallback)(struct __PCD_HandleTypeDef *hpcd);                          /*!< USB OTG PCD Connect callback            */
  void (* DisconnectCallback)(struct __PCD_HandleTypeDef *hpcd);                       /*!< USB OTG PCD Disconnect callback         */

  void (* DataOutStageCallback)(struct __PCD_HandleTypeDef *hpcd, uint8_t epnum);      /*!< USB OTG PCD Data OUT Stage callback     */
  void (* DataInStageCallback)(struct __PCD_HandleTypeDef *hpcd, uint8_t epnum);       /*!< USB OTG PCD Data IN Stage callback      */
  void (* ISOOUTIncompleteCallback)(struct __PCD_HandleTypeDef *hpcd, uint8_t epnum);  /*!< USB OTG PCD ISO OUT Incomplete callback */
  void (* ISOINIncompleteCallback)(struct __PCD_HandleTypeDef *hpcd, uint8_t epnum);   /*!< USB OTG PCD ISO IN Incomplete callback  */
  void (* BCDCallback)(struct __PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg);      /*!< USB OTG PCD BCD callback                */
  void (* LPMCallback)(struct __PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg);      /*!< USB OTG PCD LPM callback                */

  void (* MspInitCallback)(struct __PCD_HandleTypeDef *hpcd);                          /*!< USB OTG PCD Msp Init callback           */
  void (* MspDeInitCallback)(struct __PCD_HandleTypeDef *hpcd);                        /*!< USB OTG PCD Msp DeInit callback         */
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
} PCD_HandleTypeDef;

/**
  * @}
  */

/* Include PCD DAL Extended module */
#include "apm32f4xx_dal_pcd_ex.h"

/* Exported constants --------------------------------------------------------*/
/** @defgroup PCD_Exported_Constants PCD Exported Constants
  * @{
  */

/** @defgroup PCD_Speed PCD Speed
  * @{
  */
#define PCD_SPEED_HIGH               USBD_HS_SPEED
#define PCD_SPEED_HIGH_IN_FULL       USBD_HSINFS_SPEED
#define PCD_SPEED_FULL               USBD_FS_SPEED
/**
  * @}
  */

/** @defgroup PCD_PHY_Module PCD PHY Module
  * @{
  */
#define PCD_PHY_ULPI                 1U
#define PCD_PHY_EMBEDDED             2U
#define PCD_PHY_UTMI                 3U
/**
  * @}
  */

/** @defgroup PCD_Error_Code_definition PCD Error Code definition
  * @brief  PCD Error Code definition
  * @{
  */
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
#define  DAL_PCD_ERROR_INVALID_CALLBACK                        (0x00000010U)    /*!< Invalid Callback error  */
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup PCD_Exported_Macros PCD Exported Macros
  *  @brief macros to handle interrupts and specific clock configurations
  * @{
  */
#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
#define __DAL_PCD_ENABLE(__HANDLE__)                       (void)USB_EnableGlobalInt ((__HANDLE__)->Instance)
#define __DAL_PCD_DISABLE(__HANDLE__)                      (void)USB_DisableGlobalInt ((__HANDLE__)->Instance)

#define __DAL_PCD_GET_FLAG(__HANDLE__, __INTERRUPT__) \
  ((USB_ReadInterrupts((__HANDLE__)->Instance) & (__INTERRUPT__)) == (__INTERRUPT__))

#define __DAL_PCD_CLEAR_FLAG(__HANDLE__, __INTERRUPT__)    (((__HANDLE__)->Instance->GCINT) &=  (__INTERRUPT__))
#define __DAL_PCD_IS_INVALID_INTERRUPT(__HANDLE__)         (USB_ReadInterrupts((__HANDLE__)->Instance) == 0U)

#define __DAL_PCD_UNGATE_PHYCLOCK(__HANDLE__) \
  *(__IO uint32_t *)((uint32_t)((__HANDLE__)->Instance) + USB_OTG_PCGCCTL_BASE) &= ~(USB_OTG_PCGCCTL_STOPCLK)

#define __DAL_PCD_GATE_PHYCLOCK(__HANDLE__) \
  *(__IO uint32_t *)((uint32_t)((__HANDLE__)->Instance) + USB_OTG_PCGCCTL_BASE) |= USB_OTG_PCGCCTL_STOPCLK

#define __DAL_PCD_IS_PHY_SUSPENDED(__HANDLE__) \
  ((*(__IO uint32_t *)((uint32_t)((__HANDLE__)->Instance) + USB_OTG_PCGCCTL_BASE)) & 0x10U)

#define __DAL_USB_OTG_HS_WAKEUP_EINT_ENABLE_IT()    EINT->IMASK |= (USB_OTG_HS_WAKEUP_EINT_LINE)
#define __DAL_USB_OTG_HS_WAKEUP_EINT_DISABLE_IT()   EINT->IMASK &= ~(USB_OTG_HS_WAKEUP_EINT_LINE)
#define __DAL_USB_OTG_HS_WAKEUP_EINT_GET_FLAG()     EINT->IPEND & (USB_OTG_HS_WAKEUP_EINT_LINE)
#define __DAL_USB_OTG_HS_WAKEUP_EINT_CLEAR_FLAG()   EINT->IPEND = (USB_OTG_HS_WAKEUP_EINT_LINE)

#define __DAL_USB_OTG_HS_WAKEUP_EINT_ENABLE_RISING_EDGE() \
  do { \
    EINT->FTEN &= ~(USB_OTG_HS_WAKEUP_EINT_LINE); \
    EINT->RTEN |= USB_OTG_HS_WAKEUP_EINT_LINE; \
  } while(0U)
#define __DAL_USB_OTG_FS_WAKEUP_EINT_ENABLE_IT()    EINT->IMASK |= USB_OTG_FS_WAKEUP_EINT_LINE
#define __DAL_USB_OTG_FS_WAKEUP_EINT_DISABLE_IT()   EINT->IMASK &= ~(USB_OTG_FS_WAKEUP_EINT_LINE)
#define __DAL_USB_OTG_FS_WAKEUP_EINT_GET_FLAG()     EINT->IPEND & (USB_OTG_FS_WAKEUP_EINT_LINE)
#define __DAL_USB_OTG_FS_WAKEUP_EINT_CLEAR_FLAG()   EINT->IPEND = USB_OTG_FS_WAKEUP_EINT_LINE

#define __DAL_USB_OTG_FS_WAKEUP_EINT_ENABLE_RISING_EDGE() \
  do { \
    EINT->FTEN &= ~(USB_OTG_FS_WAKEUP_EINT_LINE); \
    EINT->RTEN |= USB_OTG_FS_WAKEUP_EINT_LINE; \
  } while(0U)
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup PCD_Exported_Functions PCD Exported Functions
  * @{
  */

/* Initialization/de-initialization functions  ********************************/
/** @addtogroup PCD_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
DAL_StatusTypeDef DAL_PCD_Init(PCD_HandleTypeDef *hpcd);
DAL_StatusTypeDef DAL_PCD_DeInit(PCD_HandleTypeDef *hpcd);
void DAL_PCD_MspInit(PCD_HandleTypeDef *hpcd);
void DAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd);

#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
/** @defgroup DAL_PCD_Callback_ID_enumeration_definition DAL USB OTG PCD Callback ID enumeration definition
  * @brief  DAL USB OTG PCD Callback ID enumeration definition
  * @{
  */
typedef enum
{
  DAL_PCD_SOF_CB_ID          = 0x01,      /*!< USB PCD SOF callback ID          */
  DAL_PCD_SETUPSTAGE_CB_ID   = 0x02,      /*!< USB PCD Setup Stage callback ID  */
  DAL_PCD_RESET_CB_ID        = 0x03,      /*!< USB PCD Reset callback ID        */
  DAL_PCD_SUSPEND_CB_ID      = 0x04,      /*!< USB PCD Suspend callback ID      */
  DAL_PCD_RESUME_CB_ID       = 0x05,      /*!< USB PCD Resume callback ID       */
  DAL_PCD_CONNECT_CB_ID      = 0x06,      /*!< USB PCD Connect callback ID      */
  DAL_PCD_DISCONNECT_CB_ID   = 0x07,      /*!< USB PCD Disconnect callback ID   */

  DAL_PCD_MSPINIT_CB_ID      = 0x08,      /*!< USB PCD MspInit callback ID      */
  DAL_PCD_MSPDEINIT_CB_ID    = 0x09       /*!< USB PCD MspDeInit callback ID    */

} DAL_PCD_CallbackIDTypeDef;
/**
  * @}
  */

/** @defgroup DAL_PCD_Callback_pointer_definition DAL USB OTG PCD Callback pointer definition
  * @brief  DAL USB OTG PCD Callback pointer definition
  * @{
  */

typedef void (*pPCD_CallbackTypeDef)(PCD_HandleTypeDef *hpcd);                                   /*!< pointer to a common USB OTG PCD callback function  */
typedef void (*pPCD_DataOutStageCallbackTypeDef)(PCD_HandleTypeDef *hpcd, uint8_t epnum);        /*!< pointer to USB OTG PCD Data OUT Stage callback     */
typedef void (*pPCD_DataInStageCallbackTypeDef)(PCD_HandleTypeDef *hpcd, uint8_t epnum);         /*!< pointer to USB OTG PCD Data IN Stage callback      */
typedef void (*pPCD_IsoOutIncpltCallbackTypeDef)(PCD_HandleTypeDef *hpcd, uint8_t epnum);        /*!< pointer to USB OTG PCD ISO OUT Incomplete callback */
typedef void (*pPCD_IsoInIncpltCallbackTypeDef)(PCD_HandleTypeDef *hpcd, uint8_t epnum);         /*!< pointer to USB OTG PCD ISO IN Incomplete callback  */
typedef void (*pPCD_LpmCallbackTypeDef)(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg);        /*!< pointer to USB OTG PCD LPM callback                */
typedef void (*pPCD_BcdCallbackTypeDef)(PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg);        /*!< pointer to USB OTG PCD BCD callback                */

/**
  * @}
  */

DAL_StatusTypeDef DAL_PCD_RegisterCallback(PCD_HandleTypeDef *hpcd, DAL_PCD_CallbackIDTypeDef CallbackID,
                                           pPCD_CallbackTypeDef pCallback);

DAL_StatusTypeDef DAL_PCD_UnRegisterCallback(PCD_HandleTypeDef *hpcd, DAL_PCD_CallbackIDTypeDef CallbackID);

DAL_StatusTypeDef DAL_PCD_RegisterDataOutStageCallback(PCD_HandleTypeDef *hpcd,
                                                       pPCD_DataOutStageCallbackTypeDef pCallback);

DAL_StatusTypeDef DAL_PCD_UnRegisterDataOutStageCallback(PCD_HandleTypeDef *hpcd);

DAL_StatusTypeDef DAL_PCD_RegisterDataInStageCallback(PCD_HandleTypeDef *hpcd,
                                                      pPCD_DataInStageCallbackTypeDef pCallback);

DAL_StatusTypeDef DAL_PCD_UnRegisterDataInStageCallback(PCD_HandleTypeDef *hpcd);

DAL_StatusTypeDef DAL_PCD_RegisterIsoOutIncpltCallback(PCD_HandleTypeDef *hpcd,
                                                       pPCD_IsoOutIncpltCallbackTypeDef pCallback);

DAL_StatusTypeDef DAL_PCD_UnRegisterIsoOutIncpltCallback(PCD_HandleTypeDef *hpcd);

DAL_StatusTypeDef DAL_PCD_RegisterIsoInIncpltCallback(PCD_HandleTypeDef *hpcd,
                                                      pPCD_IsoInIncpltCallbackTypeDef pCallback);

DAL_StatusTypeDef DAL_PCD_UnRegisterIsoInIncpltCallback(PCD_HandleTypeDef *hpcd);

DAL_StatusTypeDef DAL_PCD_RegisterBcdCallback(PCD_HandleTypeDef *hpcd, pPCD_BcdCallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_PCD_UnRegisterBcdCallback(PCD_HandleTypeDef *hpcd);

DAL_StatusTypeDef DAL_PCD_RegisterLpmCallback(PCD_HandleTypeDef *hpcd, pPCD_LpmCallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_PCD_UnRegisterLpmCallback(PCD_HandleTypeDef *hpcd);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
/**
  * @}
  */

/* I/O operation functions  ***************************************************/
/* Non-Blocking mode: Interrupt */
/** @addtogroup PCD_Exported_Functions_Group2 Input and Output operation functions
  * @{
  */
DAL_StatusTypeDef DAL_PCD_Start(PCD_HandleTypeDef *hpcd);
DAL_StatusTypeDef DAL_PCD_Stop(PCD_HandleTypeDef *hpcd);
void DAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd);
void DAL_PCD_WKUP_IRQHandler(PCD_HandleTypeDef *hpcd);

void DAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd);
void DAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd);
void DAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd);
void DAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd);
void DAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd);
void DAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd);
void DAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd);

void DAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void DAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void DAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void DAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
/**
  * @}
  */

/* Peripheral Control functions  **********************************************/
/** @addtogroup PCD_Exported_Functions_Group3 Peripheral Control functions
  * @{
  */
DAL_StatusTypeDef DAL_PCD_DevConnect(PCD_HandleTypeDef *hpcd);
DAL_StatusTypeDef DAL_PCD_DevDisconnect(PCD_HandleTypeDef *hpcd);
DAL_StatusTypeDef DAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address);
DAL_StatusTypeDef DAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type);
DAL_StatusTypeDef DAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
DAL_StatusTypeDef DAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
DAL_StatusTypeDef DAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
DAL_StatusTypeDef DAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
DAL_StatusTypeDef DAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
uint8_t DAL_PCD_EP_ReadStallStatus(PCD_HandleTypeDef *hpcd, uint8_t epAddr);
DAL_StatusTypeDef DAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
DAL_StatusTypeDef DAL_PCD_EP_Abort(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
DAL_StatusTypeDef DAL_PCD_ActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);
DAL_StatusTypeDef DAL_PCD_DeActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);
DAL_StatusTypeDef DAL_PCD_SetTestMode(PCD_HandleTypeDef *hpcd, uint8_t testmode);

uint32_t          DAL_PCD_EP_GetRxCount(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
/**
  * @}
  */

/* Peripheral State functions  ************************************************/
/** @addtogroup PCD_Exported_Functions_Group4 Peripheral State functions
  * @{
  */
PCD_StateTypeDef DAL_PCD_GetState(PCD_HandleTypeDef *hpcd);
/**
  * @}
  */

/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup PCD_Private_Constants PCD Private Constants
  * @{
  */
/** @defgroup USB_EINT_Line_Interrupt USB EINT line interrupt
  * @{
  */
#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
#define USB_OTG_FS_WAKEUP_EINT_LINE                                   (0x1U << 18)  /*!< USB FS EINT Line WakeUp Interrupt */
#define USB_OTG_HS_WAKEUP_EINT_LINE                                   (0x1U << 20)  /*!< USB HS EINT Line WakeUp Interrupt */
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */


/**
  * @}
  */
/**
  * @}
  */

#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
#ifndef USB_OTG_DOEPINT_OTEPSPR
#define USB_OTG_DOEPINT_OTEPSPR                (0x1UL << 5)      /*!< Status Phase Received interrupt */
#endif /* defined USB_OTG_DOEPINT_OTEPSPR */

#ifndef USB_OTG_DOUTIMASK_OTEPSPRM
#define USB_OTG_DOUTIMASK_OTEPSPRM               (0x1UL << 5)      /*!< Setup Packet Received interrupt mask */
#endif /* defined USB_OTG_DOUTIMASK_OTEPSPRM */

#ifndef USB_OTG_DOEPINT_NAK
#define USB_OTG_DOEPINT_NAK                    (0x1UL << 13)      /*!< NAK interrupt */
#endif /* defined USB_OTG_DOEPINT_NAK */

#ifndef USB_OTG_DOUTIMASK_NAKM
#define USB_OTG_DOUTIMASK_NAKM                   (0x1UL << 13)      /*!< OUT Packet NAK interrupt mask */
#endif /* defined USB_OTG_DOUTIMASK_NAKM */

#ifndef USB_OTG_DOEPINT_STPKTRX
#define USB_OTG_DOEPINT_STPKTRX                (0x1UL << 15)      /*!< Setup Packet Received interrupt */
#endif /* defined USB_OTG_DOEPINT_STPKTRX */

#ifndef USB_OTG_DOUTIMASK_NYETM
#define USB_OTG_DOUTIMASK_NYETM                  (0x1UL << 14)      /*!< Setup Packet Received interrupt mask */
#endif /* defined USB_OTG_DOUTIMASK_NYETM */
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */

/* Private macros ------------------------------------------------------------*/
/** @defgroup PCD_Private_Macros PCD Private Macros
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
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DAL_PCD_H */
