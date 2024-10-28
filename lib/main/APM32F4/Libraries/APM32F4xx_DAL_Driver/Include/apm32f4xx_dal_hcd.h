/**
  *
  * @file    apm32f4xx_dal_hcd.h
  * @brief   Header file of HCD DAL module.
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
#ifndef APM32F4xx_DAL_HCD_H
#define APM32F4xx_DAL_HCD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_ddl_usb.h"

#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup HCD HCD
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup HCD_Exported_Types HCD Exported Types
  * @{
  */

/** @defgroup HCD_Exported_Types_Group1 HCD State Structure definition
  * @{
  */
typedef enum
{
  DAL_HCD_STATE_RESET    = 0x00,
  DAL_HCD_STATE_READY    = 0x01,
  DAL_HCD_STATE_ERROR    = 0x02,
  DAL_HCD_STATE_BUSY     = 0x03,
  DAL_HCD_STATE_TIMEOUT  = 0x04
} HCD_StateTypeDef;

typedef USB_OTG_GlobalTypeDef   HCD_TypeDef;
typedef USB_OTG_CfgTypeDef      HCD_InitTypeDef;
typedef USB_OTG_HCTypeDef       HCD_HCTypeDef;
typedef USB_OTG_URBStateTypeDef HCD_URBStateTypeDef;
typedef USB_OTG_HCStateTypeDef  HCD_HCStateTypeDef;
/**
  * @}
  */

/** @defgroup HCD_Exported_Types_Group2 HCD Handle Structure definition
  * @{
  */
#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
typedef struct __HCD_HandleTypeDef
#else
typedef struct
#endif /* USE_DAL_HCD_REGISTER_CALLBACKS */
{
  HCD_TypeDef               *Instance;  /*!< Register base address    */
  HCD_InitTypeDef           Init;       /*!< HCD required parameters  */
  HCD_HCTypeDef             hc[16];     /*!< Host channels parameters */
  DAL_LockTypeDef           Lock;       /*!< HCD peripheral status    */
  __IO HCD_StateTypeDef     State;      /*!< HCD communication state  */
  __IO  uint32_t            ErrorCode;  /*!< HCD Error code           */
  void                      *pData;     /*!< Pointer Stack Handler    */
#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
  void (* SOFCallback)(struct __HCD_HandleTypeDef *hhcd);                               /*!< USB OTG HCD SOF callback                */
  void (* ConnectCallback)(struct __HCD_HandleTypeDef *hhcd);                           /*!< USB OTG HCD Connect callback            */
  void (* DisconnectCallback)(struct __HCD_HandleTypeDef *hhcd);                        /*!< USB OTG HCD Disconnect callback         */
  void (* PortEnabledCallback)(struct __HCD_HandleTypeDef *hhcd);                       /*!< USB OTG HCD Port Enable callback        */
  void (* PortDisabledCallback)(struct __HCD_HandleTypeDef *hhcd);                      /*!< USB OTG HCD Port Disable callback       */
  void (* HC_NotifyURBChangeCallback)(struct __HCD_HandleTypeDef *hhcd, uint8_t chnum,
                                      HCD_URBStateTypeDef urb_state);                   /*!< USB OTG HCD Host Channel Notify URB Change callback  */

  void (* MspInitCallback)(struct __HCD_HandleTypeDef *hhcd);                           /*!< USB OTG HCD Msp Init callback           */
  void (* MspDeInitCallback)(struct __HCD_HandleTypeDef *hhcd);                         /*!< USB OTG HCD Msp DeInit callback         */
#endif /* USE_DAL_HCD_REGISTER_CALLBACKS */
} HCD_HandleTypeDef;
/**
  * @}
  */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup HCD_Exported_Constants HCD Exported Constants
  * @{
  */

/** @defgroup HCD_Speed HCD Speed
  * @{
  */
#define HCD_SPEED_HIGH               USBH_HS_SPEED
#define HCD_SPEED_FULL               USBH_FSLS_SPEED
#define HCD_SPEED_LOW                USBH_FSLS_SPEED
/**
  * @}
  */

/** @defgroup HCD_Device_Speed HCD Device Speed
  * @{
  */
#define HCD_DEVICE_SPEED_HIGH               0U
#define HCD_DEVICE_SPEED_FULL               1U
#define HCD_DEVICE_SPEED_LOW                2U
/**
  * @}
  */

/** @defgroup HCD_PHY_Module HCD PHY Module
  * @{
  */
#define HCD_PHY_ULPI                 1U
#define HCD_PHY_EMBEDDED             2U
/**
  * @}
  */

/** @defgroup HCD_Error_Code_definition HCD Error Code definition
  * @brief  HCD Error Code definition
  * @{
  */
#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
#define  DAL_HCD_ERROR_INVALID_CALLBACK                        (0x00000010U)    /*!< Invalid Callback error  */
#endif /* USE_DAL_HCD_REGISTER_CALLBACKS */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup HCD_Exported_Macros HCD Exported Macros
  *  @brief macros to handle interrupts and specific clock configurations
  * @{
  */
#define __DAL_HCD_ENABLE(__HANDLE__)                   (void)USB_EnableGlobalInt ((__HANDLE__)->Instance)
#define __DAL_HCD_DISABLE(__HANDLE__)                  (void)USB_DisableGlobalInt ((__HANDLE__)->Instance)

#define __DAL_HCD_GET_FLAG(__HANDLE__, __INTERRUPT__)      ((USB_ReadInterrupts((__HANDLE__)->Instance)\
                                                             & (__INTERRUPT__)) == (__INTERRUPT__))
#define __DAL_HCD_CLEAR_FLAG(__HANDLE__, __INTERRUPT__)    (((__HANDLE__)->Instance->GCINT) = (__INTERRUPT__))
#define __DAL_HCD_IS_INVALID_INTERRUPT(__HANDLE__)         (USB_ReadInterrupts((__HANDLE__)->Instance) == 0U)

#define __DAL_HCD_CLEAR_HC_INT(chnum, __INTERRUPT__)  (USBx_HC(chnum)->HCHINT = (__INTERRUPT__))
#define __DAL_HCD_MASK_HALT_HC_INT(chnum)             (USBx_HC(chnum)->HCHIMASK &= ~USB_OTG_HCHIMASK_TSFCMPANM)
#define __DAL_HCD_UNMASK_HALT_HC_INT(chnum)           (USBx_HC(chnum)->HCHIMASK |= USB_OTG_HCHIMASK_TSFCMPANM)
#define __DAL_HCD_MASK_ACK_HC_INT(chnum)              (USBx_HC(chnum)->HCHIMASK &= ~USB_OTG_HCHIMASK_RXTXACKM)
#define __DAL_HCD_UNMASK_ACK_HC_INT(chnum)            (USBx_HC(chnum)->HCHIMASK |= USB_OTG_HCHIMASK_RXTXACKM)
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HCD_Exported_Functions HCD Exported Functions
  * @{
  */

/** @defgroup HCD_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
DAL_StatusTypeDef DAL_HCD_Init(HCD_HandleTypeDef *hhcd);
DAL_StatusTypeDef DAL_HCD_DeInit(HCD_HandleTypeDef *hhcd);
DAL_StatusTypeDef DAL_HCD_HC_Init(HCD_HandleTypeDef *hhcd, uint8_t ch_num,
                                  uint8_t epnum, uint8_t dev_address,
                                  uint8_t speed, uint8_t ep_type, uint16_t mps);

DAL_StatusTypeDef DAL_HCD_HC_Halt(HCD_HandleTypeDef *hhcd, uint8_t ch_num);
void              DAL_HCD_MspInit(HCD_HandleTypeDef *hhcd);
void              DAL_HCD_MspDeInit(HCD_HandleTypeDef *hhcd);

#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
/** @defgroup DAL_HCD_Callback_ID_enumeration_definition DAL USB OTG HCD Callback ID enumeration definition
  * @brief  DAL USB OTG HCD Callback ID enumeration definition
  * @{
  */
typedef enum
{
  DAL_HCD_SOF_CB_ID            = 0x01,       /*!< USB HCD SOF callback ID           */
  DAL_HCD_CONNECT_CB_ID        = 0x02,       /*!< USB HCD Connect callback ID       */
  DAL_HCD_DISCONNECT_CB_ID     = 0x03,       /*!< USB HCD Disconnect callback ID    */
  DAL_HCD_PORT_ENABLED_CB_ID   = 0x04,       /*!< USB HCD Port Enable callback ID   */
  DAL_HCD_PORT_DISABLED_CB_ID  = 0x05,       /*!< USB HCD Port Disable callback ID  */

  DAL_HCD_MSPINIT_CB_ID        = 0x06,       /*!< USB HCD MspInit callback ID       */
  DAL_HCD_MSPDEINIT_CB_ID      = 0x07        /*!< USB HCD MspDeInit callback ID     */

} DAL_HCD_CallbackIDTypeDef;
/**
  * @}
  */

/** @defgroup DAL_HCD_Callback_pointer_definition DAL USB OTG HCD Callback pointer definition
  * @brief  DAL USB OTG HCD Callback pointer definition
  * @{
  */

typedef void (*pHCD_CallbackTypeDef)(HCD_HandleTypeDef *hhcd);                   /*!< pointer to a common USB OTG HCD callback function  */
typedef void (*pHCD_HC_NotifyURBChangeCallbackTypeDef)(HCD_HandleTypeDef *hhcd,
                                                       uint8_t epnum,
                                                       HCD_URBStateTypeDef urb_state);   /*!< pointer to USB OTG HCD host channel  callback */
/**
  * @}
  */

DAL_StatusTypeDef DAL_HCD_RegisterCallback(HCD_HandleTypeDef *hhcd,
                                           DAL_HCD_CallbackIDTypeDef CallbackID,
                                           pHCD_CallbackTypeDef pCallback);

DAL_StatusTypeDef DAL_HCD_UnRegisterCallback(HCD_HandleTypeDef *hhcd,
                                             DAL_HCD_CallbackIDTypeDef CallbackID);

DAL_StatusTypeDef DAL_HCD_RegisterHC_NotifyURBChangeCallback(HCD_HandleTypeDef *hhcd,
                                                             pHCD_HC_NotifyURBChangeCallbackTypeDef pCallback);

DAL_StatusTypeDef DAL_HCD_UnRegisterHC_NotifyURBChangeCallback(HCD_HandleTypeDef *hhcd);
#endif /* USE_DAL_HCD_REGISTER_CALLBACKS */
/**
  * @}
  */

/* I/O operation functions  ***************************************************/
/** @addtogroup HCD_Exported_Functions_Group2 Input and Output operation functions
  * @{
  */
DAL_StatusTypeDef DAL_HCD_HC_SubmitRequest(HCD_HandleTypeDef *hhcd, uint8_t ch_num,
                                           uint8_t direction, uint8_t ep_type,
                                           uint8_t token, uint8_t *pbuff,
                                           uint16_t length, uint8_t do_ping);

/* Non-Blocking mode: Interrupt */
void DAL_HCD_IRQHandler(HCD_HandleTypeDef *hhcd);
void DAL_HCD_WKUP_IRQHandler(HCD_HandleTypeDef *hhcd);
void DAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd);
void DAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd);
void DAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd);
void DAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef *hhcd);
void DAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef *hhcd);
void DAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum,
                                         HCD_URBStateTypeDef urb_state);
/**
  * @}
  */

/* Peripheral Control functions  **********************************************/
/** @addtogroup HCD_Exported_Functions_Group3 Peripheral Control functions
  * @{
  */
DAL_StatusTypeDef DAL_HCD_ResetPort(HCD_HandleTypeDef *hhcd);
DAL_StatusTypeDef DAL_HCD_Start(HCD_HandleTypeDef *hhcd);
DAL_StatusTypeDef DAL_HCD_Stop(HCD_HandleTypeDef *hhcd);
/**
  * @}
  */

/* Peripheral State functions  ************************************************/
/** @addtogroup HCD_Exported_Functions_Group4 Peripheral State functions
  * @{
  */
HCD_StateTypeDef        DAL_HCD_GetState(HCD_HandleTypeDef *hhcd);
HCD_URBStateTypeDef     DAL_HCD_HC_GetURBState(HCD_HandleTypeDef *hhcd, uint8_t chnum);
HCD_HCStateTypeDef      DAL_HCD_HC_GetState(HCD_HandleTypeDef *hhcd, uint8_t chnum);
uint32_t                DAL_HCD_HC_GetXferCount(HCD_HandleTypeDef *hhcd, uint8_t chnum);
uint32_t                DAL_HCD_GetCurrentFrame(HCD_HandleTypeDef *hhcd);
uint32_t                DAL_HCD_GetCurrentSpeed(HCD_HandleTypeDef *hhcd);
void                    DAL_HCD_ConfigToggle(HCD_HandleTypeDef* hhcd, uint8_t pipe, uint8_t toggle);
uint8_t                 DAL_HCD_ReadToggle(HCD_HandleTypeDef* hhcd, uint8_t pipe);
/**
  * @}
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup HCD_Private_Macros HCD Private Macros
  * @{
  */
/**
  * @}
  */
/* Private functions prototypes ----------------------------------------------*/

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

#endif /* APM32F4xx_DAL_HCD_H */
