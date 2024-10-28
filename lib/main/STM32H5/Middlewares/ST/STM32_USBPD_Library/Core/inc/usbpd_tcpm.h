/**
  ******************************************************************************
  * @file    usbpd_tcpm.h
  * @author  MCD Application Team
  * @brief   Header file containing functions prototypes of USBPD TCPM library.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBPD_TCPM_H
#define __USBPD_TCPM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#if defined(USBPDCORE_TCPM_SUPPORT)

#include "string.h"
#include "usbpd_def.h"
#include "tcpc.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_CORE
  * @{
  */

/** @addtogroup USBPD_CORE_TCPM
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/** @defgroup USBPD_CORE_TCPM_Exported_TypesDefinitions USBPD CORE TCPM Exported Types Definitions
  * @{
  */
/**
  * @brief CallBacks exposed by the @ref USBPD_CORE_TCPM to the @ref USBPD_CORE_PRL
  */
typedef struct
{
  /**
    * @brief  Reports that a message has been received on a specified port.
    * @param  hport:    The handle of the port
    * @param  Type:    The type of the message received
    * @retval None
    * @note Received data are stored inside hport->pRxBuffPtr
    */
  void (*USBPD_TCPM_MessageReceived)(uint8_t hport, USBPD_SOPType_TypeDef Type);

  /**
    * @brief  Reports to the PRL that a Reset received from channel.
    * @param  hport:    The handle of the port
    * @param  Type:    The type of reset performed
    * @retval None
    */
  void (*USBPD_TCPM_ResetIndication)(uint8_t hport, USBPD_SOPType_TypeDef Type);

  /**
    * @brief  Reports to the PRL that a Reset operation has been completed.
    * @param  hport:    The handle of the port
    * @param  Type:    The type of reset performed
    * @retval None
    */
  void (*USBPD_TCPM_ResetCompleted)(uint8_t hport, USBPD_SOPType_TypeDef Type);

  /**
    * @brief  Reports to the PRL that a Bist operation has been completed.
    * @param  hport:    The handle of the port
    * @param  Type:    The type of Bist performed
    * @retval None
    */
  void (*USBPD_TCPM_BistCompleted)(uint8_t hport, USBPD_BISTMsg_TypeDef bistmode);

  /**
    * @brief  USB-PD message sent callback from TCPC
    * @param  PortNum port number value
    * @param  Status Status of the transmission
    * @retval None
    */
  void (*USBPD_TCPM_MessageReceivedTC)(uint8_t PortNum, uint32_t status);

  /**
    * @brief  Reports to the PRL that an FRS has been detected.
    * @param  PortNum:    The handle of the port
    * @retval None
    */
  void (*USBPD_PHY_FastRoleSwapReception)(uint8_t PortNum);

} USBPD_PHY_Callbacks;

/**
  * @brief Initialization structure exposed by the @ref USBPD_CORE_TCPM to the @ref USBPD_CORE_PRL
  */
typedef struct
{
  uint8_t   *pRxBuffer;             /*!< Pointer to @ref USBPD_CORE_PRL RX Buffer for the current port */
  const USBPD_PHY_Callbacks *pCallbacks;  /*!< TCPM Callbacks */
} USBPD_TCPM_HandleTypeDef;
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup USBPD_CORE_TCPM_Exported_Functions
  * @{
  */

/** @defgroup USBPD_CORE_TCPM_Exported_Functions_Grp2 USBPD CORE TCPM Exported Functions to PRL
  * @{
  */
/**
  * @brief  Initialize TCPC devices
  * @param  PortNum     Number of the port
  * @param  pCallbacks  TCPM callbacks
  * @param  pRxBuffer   Pointer on the RX buffer
  * @param  PowerRole   Power role can be one of the following values:
  *         @arg @ref USBPD_PORTPOWERROLE_SNK
  *         @arg @ref USBPD_PORTPOWERROLE_SRC
  * @param  SupportedSOP  Supported SOP
  * @retval HAL status
  */
USBPD_StatusTypeDef  USBPD_PHY_Init(uint8_t PortNum, const USBPD_PHY_Callbacks *pCallbacks, uint8_t *pRxBuffer,
                                    USBPD_PortPowerRole_TypeDef PowerRole, uint32_t SupportedSOP);

/**
  * @brief  Reset the PHY of a specified port.
  * @param  PortNum    Number of the port.
  * @retval None
  */
void                 USBPD_PHY_Reset(uint8_t PortNum);

/**
  * @brief  function to set the supported SOP
  * @param  PortNum       Number of the port.
  * @param  SOPSupported  List of the supported SOP
  * @retval None.
  */
void                 USBPD_PHY_SOPSupported(uint8_t PortNum, uint32_t SOPSupported);

/**
  * @brief  De-initialize TCPC devices
  * @param  PortNum Number of the port
  * @retval None
  */
void                 USBPD_TCPM_DeInit(uint8_t PortNum);

/**
  * @brief  Get CC line for PD connection
  * @param  PortNum Number of the port
  * @param  CC1_Level Pointer of status of the CC1 line
  * @param  CC2_Level Pointer of status of the CC2 line
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_TCPM_get_cc(uint32_t PortNum, uint32_t *CC1_Level, uint32_t *CC2_Level);

/**
  * @brief  Set the polarity of the CC lines
  * @param  PortNum   Number of the port
  * @param  Polarity  Polarity
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_TCPM_set_polarity(uint32_t PortNum, uint32_t Polarity);

/**
  * @brief  Set power and data role et PD message header
  * @param  PortNum        Number of the port
  * @param  PowerRole      Power role
  * @param  DataRole       Data role
  * @param  Specification  PD Specification version
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_TCPM_set_msg_header(uint32_t PortNum, USBPD_PortPowerRole_TypeDef PowerRole,
                                               USBPD_PortDataRole_TypeDef DataRole,
                                               USBPD_SpecRev_TypeDef Specification);

/**
  * @brief  Enable or disable PD reception
  * @param  PortNum       Number of the port
  * @param  Pull          Value of the CC pin to configure based on @ref TCPC_CC_Pull_TypeDef
  * @param  State         Activation or deactivation of RX
  * @param  SupportedSOP  Supported SOP by PRL
  * @param  HardReset     Hard reset status based on @ref TCPC_hard_reset
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_TCPM_set_rx_state(uint32_t PortNum, TCPC_CC_Pull_TypeDef Pull, USBPD_FunctionalState State,
                                             uint32_t SupportedSOP, TCPC_hard_reset HardReset);

/**
  * @brief  Retrieve the PD message
  * @param  PortNum Number of the port
  * @param  Payload Pointer on the payload
  * @param  Type    Pointer on the message type
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_TCPM_get_message(uint32_t PortNum, uint8_t *Payload, uint8_t *Type);

/**
  * @brief  Transmit the PD message
  * @param  PortNum     Number of the port
  * @param  Type        Message type
  * @param  pData       Pointer on the data message
  * @param  RetryNumber Number of retry
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_TCPM_transmit(uint32_t PortNum, USBPD_SOPType_TypeDef Type, const uint8_t *pData,
                                         uint32_t RetryNumber);

/**
  * @brief  Send bist pattern.
  * @param  PortNum    Number of the port
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_PHY_Send_BIST_Pattern(uint32_t PortNum);

/**
  * @brief  Request a Reset on a specified port.
  * @param  PortNum   Number of the port
  * @param  Type      The type of reset (hard or cable reset).
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_PHY_ResetRequest(uint8_t PortNum, USBPD_SOPType_TypeDef Type);

/**
  * @brief  Request TCPC to enter a specific BIST test mode.
  * @param  PortNum  Number of the port
  * @param  State    Enable BIST carrier mode 2
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_TCPM_Send_BIST_Pattern(uint8_t PortNum, USBPD_FunctionalState State);

/**
  * @brief  function to set the SinkTxNg
  * @param  PortNum  Number of the port.
  * @retval none.
  */
void                 USBPD_PHY_SetResistor_SinkTxNG(uint8_t PortNum);

/**
  * @brief  function to set the SinkTxOK
  * @param  PortNum  Number of the port.
  * @retval none.
  */
void                 USBPD_PHY_SetResistor_SinkTxOK(uint8_t PortNum);

/**
  * @brief  function to check if SinkTxOK
  * @param  PortNum  Number of the port.
  * @retval USBPD_TRUE or USBPD_FALSE
  */
uint8_t               USBPD_PHY_IsResistor_SinkTxOk(uint8_t PortNum);

/**
  * @brief  Trigger in Fast role swap signalling
  * @param  PortNum  Number of the port.
  * @retval None
  */
void                  USBPD_PHY_FastRoleSwapSignalling(uint8_t PortNum);

/**
  * @brief  Enable RX
  * @param  PortNum    Number of the port.
  * @retval None
  */
void                  USBPD_PHY_EnableRX(uint8_t PortNum);

/**
  * @brief  Disable RX
  * @param  PortNum    Number of the port.
  * @retval None
  */
void                  USBPD_PHY_DisableRX(uint8_t PortNum);

/**
  * @}
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

/**
  * @}
  */

#endif /* USBPDCORE_TCPM_SUPPORT */

#ifdef __cplusplus
}
#endif


#endif /* __USBPD_TCPM_H */

