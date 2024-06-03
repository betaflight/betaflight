/**
  ******************************************************************************
  * @file    usbpd_cad_hw_if.c
  * @author  MCD Application Team
  * @brief   This file contains power hardware interface cad functions.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#define USBPD_CAD_HW_IF_C
#include "usbpd_devices_conf.h"
#include "usbpd_hw.h"
#include "usbpd_def.h"
#include "usbpd_cad_hw_if.h"
#include "usbpd_hw_if.h"
#if defined(_TRACE)
#include "usbpd_trace.h"
#endif /* _TRACE*/
#include "string.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_CAD_HW_IF
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief CAD State Machine function pointer
  * @{
  */
typedef uint32_t CAD_StateMachinePtr(uint8_t PortNum, USBPD_CAD_EVENT *Event, CCxPin_TypeDef *CCXX);

/**
  * @brief CAD State value @ref USBPD_DEVICE_CAD_HW_IF
  * @{
  */
typedef enum
{
  USBPD_CAD_STATE_RESET                   = 0U,  /*!< USBPD CAD State Reset                              */
  USBPD_CAD_STATE_DETACHED                = 1U,  /*!< USBPD CAD State No cable detected                  */
  USBPD_CAD_STATE_ATTACHED_WAIT           = 2U,  /*!< USBPD CAD State Port partner detected              */
  USBPD_CAD_STATE_ATTACHED                = 3U,  /*!< USBPD CAD State Port partner attached              */
#if defined(_DRP) || defined(_SRC)
  USBPD_CAD_STATE_EMC                     = 4U,  /*!< USBPD CAD State Electronically Marked Cable detected */
  USBPD_CAD_STATE_ATTEMC                  = 5U,  /*!< USBPD CAD State Port Partner detected through EMC  */
#if defined(_ACCESSORY_SRC)
  USBPD_CAD_STATE_DEBUG                   = 7U,  /*!< USBPD CAD State Debug detected                     */
  USBPD_CAD_STATE_ACCESSORY               = 6U,  /*!< USBPD CAD State Accessory detected                 */
#endif /* _ACCESSORY_SRC */
#endif /* _DRP || _SRC */
  USBPD_CAD_STATE_SWITCH_TO_SRC           = 8U,  /*!< USBPD CAD State switch to Source                   */
  USBPD_CAD_STATE_SWITCH_TO_SNK           = 9U,  /*!< USBPD CAD State switch to Sink                     */
  USBPD_CAD_STATE_UNKNOW                  = 10U, /*!< USBPD CAD State unknown                            */
  USBPD_CAD_STATE_DETACH_SRC              = 11U,
  USBPD_CAD_STATE_ERRORRECOVERY           = 12U, /*!< USBPD CAD State error recovery                     */
  USBPD_CAD_STATE_ERRORRECOVERY_EXIT      = 13U, /*!< USBPD CAD State to exit error recovery             */
#if defined(_SNK) && defined(_ACCESSORY_SNK)
  USBPD_CAD_STATE_UNATTACHED_ACCESSORY    = 14U,  /*!< USBPD CAD State Unattached.Accessory              */
  USBPD_CAD_STATE_AUDIO_ACCESSORY         = 15U,  /*!< USBPD CAD State Wait detached after Accessory detection */
#endif /* _SNK && _ACCESSORY_SNK */
  USBPD_CAD_STATE_ATTACHED_ACCESSORY_WAIT = 16U,
#if defined(USBPDCORE_VPD)
  USBPD_CAD_STATE_POWERED_ACCESSORY       = 17U,
  USBPD_CAD_STATE_UNSUPPORTED_ACCESSORY   = 18U,
  USBPD_CAD_STATE_CTVPD_UNATTACHED        = 19U,
  USBPD_CAD_STATE_CTVPD_ATTACHED          = 20U
#endif /* USBPDCORE_VPD */
} USBPD_CAD_STATE;
/**
  * @}
  */

/**
  * @brief USB PD CC lines HW condition
  */
typedef enum
{
  HW_Detachment                        = 0x00UL,   /*!< Nothing attached   */
  HW_Attachment                        = 0x01UL,   /*!< Sink attached   */
  HW_PwrCable_NoSink_Attachment        = 0x02UL,   /*!< Powered cable without Sink attached   */
  HW_PwrCable_Sink_Attachment          = 0x03UL,   /*!< Powered cable with Sink or VCONN-powered Accessory attached */
  HW_Debug_Attachment                  = 0x04UL,   /*!< Debug Accessory Mode attached   */
  HW_AudioAdapter_Attachment           = 0x05UL,   /*!< Audio Adapter Accessory Mode attached   */
} CAD_HW_Condition_TypeDef;

/**
  * @brief CAD State value @ref USBPD_DEVICE_CAD_HW_IF
  * @{
  */
typedef struct
{
  CCxPin_TypeDef                     cc                     : 2;
  CAD_HW_Condition_TypeDef           CurrentHWcondition     : 3;
  uint32_t                           CAD_tDebounce_flag     : 1;
  uint32_t                           CAD_tDebounceAcc_flag  : 1;
  uint32_t                           CAD_ErrorRecoveryflag  : 1;
  uint32_t                           CAD_ResistorUpdateflag : 1;
  USBPD_CAD_STATE                    cstate                 : 5; /* current state  */
  uint32_t                           CAD_Accessory_SRC      : 1;
  uint32_t                           CAD_Accessory_SNK      : 1;
  uint32_t                           reserved               : 1;
  USBPD_CAD_STATE                    pstate                 : 5; /* previous state */
#if defined(USBPDCORE_VPD)
  uint32_t                           CAD_VPD_SRC            : 1;
  uint32_t                           CAD_VPD_SNK            : 1;
  uint32_t                           reserved2              : 8;
#else
  uint32_t                           reserved2              : 10;
#endif /* USBPDCORE_VPD */

#if defined(_DRP) || defined(_ACCESSORY_SNK)
  uint32_t                           CAD_tToggle_start;
#endif /* _DRP */
  uint32_t                           CAD_tDebounce_start;   /* Variable used for attach or detach debounce timers */
  CAD_StateMachinePtr                *CAD_PtrStateMachine;
} CAD_HW_HandleTypeDef;
/**
  * @}
  */

/* Private define ------------------------------------------------------------*/
#define CAD_TPDDEBOUCE_THRESHOLD         12u          /**< tPDDebounce threshold between 10 to 20ms           */
#define CAD_TCCDEBOUCE_THRESHOLD         120u         /**< tCCDebounce threshold between 100 to 200ms         */
#define CAD_TSRCDISCONNECT_THRESHOLD     2u           /**< tSRCDisconnect detach threshold between 0 to 20ms  */
#define CAD_INFINITE_TIME                0xFFFFFFFFu  /**< infinite time to wait a new interrupt event        */
#define CAD_TERROR_RECOVERY_TIME         26u          /**< tErrorRecovery min 25ms                            */
#define CAD_DEFAULT_TIME                 2u           /**< default transition timing of the state machine     */
#define CAD_ACCESSORY_TOGGLE             40u          /**< toggle time for snk accessory detection            */
#define CAD_TVPDDETACH                   10u          /**< tVPDDetach timing between 10 and 20 ms             */

#if defined(_DRP) || defined(_SRC)
#define CAD_DETACH_POLLING               40u
#elif defined(_SNK)
#define CAD_DETACH_POLLING               100u
#endif /* _DRP || _SRC */

#if defined(_LOW_POWER)
#define CAD_VBUS_POLLING_TIME            38u
#else
#define CAD_VBUS_POLLING_TIME            10u
#endif /* _LOW_POWER */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* handle to manage the detection state machine */
static CAD_HW_HandleTypeDef CAD_HW_Handles[USBPD_PORT_COUNT];

/* Private function prototypes -----------------------------------------------*/
/** @defgroup USBPD_DEVICE_CAD_HW_IF_Private_Functions USBPD DEVICE_CAD HW IF Private Functions
  * @{
  */
#if defined(_DRP) || defined(_SRC)
static void CAD_Check_HW_SRC(uint8_t PortNum);
#endif /* _DRP || _SRC */

#if defined(_DRP) || defined(_SNK)
static uint32_t ManageStateAttachedWait_SNK(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX);
static uint32_t ManageStateAttached_SNK(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX);
static uint32_t ManageStateDetached_SNK(uint8_t PortNum);
static void CAD_Check_HW_SNK(uint8_t PortNum);
#endif /* _DRP || _SNK */

#if defined(_DRP)
static uint32_t ManageStateAttachedWait_DRP(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX);
static uint32_t ManageStateAttached_DRP(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX);
#endif /* _DRP */

#if defined(_DRP) || defined(_SRC)
static uint32_t ManageStateAttachedWait_SRC(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX);
#endif /* _DRP || _SRC || (_ACCESSORY && _SNK) */

#if defined(_SRC) || defined(_DRP)
static uint32_t ManageStateDetached_SRC(uint8_t PortNum);
#endif /* _SRC */

#if defined(_DRP) || defined(_SRC)
static uint32_t ManageStateEMC(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX);
static uint32_t ManageStateAttached_SRC(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX);
#endif /* _DRP || _SRC */

#if defined(_DRP)
static uint32_t ManageStateDetached_DRP(uint8_t PortNum);
#endif /* _DRP */


#if defined(_SNK)
static uint32_t CAD_StateMachine_SNK(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX);
#endif /* _SNK */

#if defined(_SRC)
static uint32_t CAD_StateMachine_SRC(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX);
#endif /* _SRC */

#if defined(_DRP)
static uint32_t CAD_StateMachine_DRP(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX);
#endif /* _DRP */


#if defined(TCPP0203_SUPPORT)
void CAD_HW_IF_VBUSDetectCallback(uint32_t PortNum,
                                  USBPD_PWR_VBUSConnectionStatusTypeDef VBUSConnectionStatus);
#endif /* TCPP0203_SUPPORT */

/**
  * @}
  */

/** @defgroup USBPD_DEVICE_CAD_HW_IF_Exported_Functions USBPD DEVICE_CAD HW IF Exported Functions
  * @{
  */

/**
  * @brief  function to initialize the cable detection state machine
  * @param  PortNum       port
  * @param  pSettings     Pointer on PD settings based on @ref USBPD_SettingsTypeDef
  * @param  pParams       Pointer on PD params based on @ref USBPD_ParamsTypeDef
  * @param  WakeUp        Wake-up callback function used for waking up CAD
  * @retval None
  */
void CAD_Init(uint8_t PortNum, USBPD_SettingsTypeDef *pSettings, USBPD_ParamsTypeDef *pParams,  void (*WakeUp)(void))
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  LL_UCPD_InitTypeDef settings;

  Ports[PortNum].params = pParams;
  Ports[PortNum].settings = pSettings;
  Ports[PortNum].params->RpResistor = Ports[PortNum].settings->CAD_DefaultResistor;
  Ports[PortNum].params->SNKExposedRP_AtAttach = vRd_Undefined;

  memset(_handle, 0, sizeof(CAD_HW_HandleTypeDef));

  Ports[PortNum].USBPD_CAD_WakeUp = WakeUp;

  /* Initialize the USBPD_IP */
  Ports[PortNum].husbpd = USBPD_HW_GetUSPDInstance(PortNum);

  /* Initialize usbpd */
  LL_UCPD_StructInit(&settings);
  (void)LL_UCPD_Init(Ports[PortNum].husbpd, &settings);
  LL_UCPD_SetRxOrderSet(Ports[PortNum].husbpd,
                        LL_UCPD_ORDERSET_SOP | LL_UCPD_ORDERSET_SOP1 | LL_UCPD_ORDERSET_SOP2 |
                        LL_UCPD_ORDERSET_CABLERST | LL_UCPD_ORDERSET_HARDRST);
  /* Controls whether pull-ups and pull-downs controls related to ANAMODE and ANASUBMODE
     should be applied to CC1 and CC2 analog PHYs */
  /* Should be done when UCPDEN is 1 */
  LL_UCPD_SetccEnable(Ports[PortNum].husbpd, LL_UCPD_CCENABLE_CC1CC2);

#ifdef _LOW_POWER
  LL_UCPD_WakeUpEnable(Ports[PortNum].husbpd);
#endif /* _LOW_POWER */

  HAL_PWREx_DisableUCPDDeadBattery();

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /* Set by default UCPD1_CC1 & UCPD1_CC2 in analog mode */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_ANALOG); /* PB13 mode = GP analog => CC1 */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_ANALOG); /* PB14 mode = GP analog => CC2 */

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_13, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_14, LL_GPIO_PULL_NO);

  /* Init power */
  BSP_USBPD_PWR_Init(PortNum);

#if defined(TCPP0203_SUPPORT)
  /* Register VBUS detect callback */
  BSP_USBPD_PWR_RegisterVBUSDetectCallback(PortNum, CAD_HW_IF_VBUSDetectCallback);
#endif /* TCPP0203_SUPPORT */

  /* Enable USBPD IP */
  LL_UCPD_Enable(Ports[PortNum].husbpd);

#if defined(_SRC) || defined(_DRP)
  /* Initialize usbpd interrupt */
  if (USBPD_PORTPOWERROLE_SRC == Ports[PortNum].params->PE_PowerRole)
  {
    USBPDM1_AssertRp(PortNum);
#if defined(TCPP0203_SUPPORT)
    /* Switch to Low Power mode */
    BSP_USBPD_PWR_SetPowerMode(PortNum, USBPD_PWR_MODE_LOWPOWER);
#endif /* TCPP0203_SUPPORT */
  }
#endif /* _SRC || _DRP */
#if defined(_DRP)
  else
#endif /* _DRP */
#if defined(_SNK) || defined(_DRP)
  {
    USBPDM1_AssertRd(PortNum);
#if defined(TCPP0203_SUPPORT)
    /* Switch to Low Power mode */
    BSP_USBPD_PWR_SetPowerMode(PortNum, USBPD_PWR_MODE_LOWPOWER);
#endif /* TCPP0203_SUPPORT */
  }
#endif /* _SNK || _DRP */

  /* Set the state machine according the SW configuration */
#if !defined(USBPDCORE_LIB_NO_PD)
#if defined(_DRP)
  if (Ports[PortNum].settings->CAD_RoleToggle == USBPD_TRUE)
  {
    _handle->CAD_PtrStateMachine = CAD_StateMachine_DRP;
    _handle->CAD_Accessory_SRC = Ports[PortNum].settings->CAD_AccesorySupport;
#if defined(USBPDCORE_VPD)
    _handle->CAD_VPD_SRC = Ports[PortNum].settings->CAD_VPDSupport;
#endif /* USBPDCORE_VPD */
  }
  else
#endif /* _DRP */
  {
#if defined(_SRC)
    if (USBPD_PORTPOWERROLE_SRC == Ports[PortNum].settings->PE_DefaultRole)
    {
      _handle->CAD_PtrStateMachine = CAD_StateMachine_SRC;
      _handle->CAD_Accessory_SRC = Ports[PortNum].settings->CAD_AccesorySupport;
#if defined(USBPDCORE_VPD)
      _handle->CAD_VPD_SRC = Ports[PortNum].settings->CAD_VPDSupport;
#endif /* USBPDCORE_VPD */
    }
    else
#endif /* _SRC */
    {
#if defined(_SNK)
      _handle->CAD_PtrStateMachine = CAD_StateMachine_SNK;
      _handle->CAD_Accessory_SNK = Ports[PortNum].settings->CAD_AccesorySupport;
#if defined(USBPDCORE_VPD)
      _handle->CAD_VPD_SNK = Ports[PortNum].settings->VPDSupport;
#endif /* USBPDCORE_VPD */
#endif /* _SNK */
    }
  }
#else  /* USBPDCORE_LIB_NO_PD */
#if defined(_SRC)
  if (USBPD_PORTPOWERROLE_SRC == Ports[PortNum].settings->PE_DefaultRole)
  {
    _handle->CAD_PtrStateMachine = CAD_StateMachine_SRC;
#if defined(USBPDCORE_VPD)
    _handle->CAD_VPD_SRC = Ports[PortNum].settings->CAD_VPDSupport;
#endif /* USBPDCORE_VPD */
  }
  else
#endif /* _SRC */
  {
#if defined(_SNK)
    _handle->CAD_PtrStateMachine = CAD_StateMachine_SNK;
#if defined(USBPDCORE_VPD)
    _handle->CAD_VPD_SNK = Ports[PortNum].settings->CAD_VPDSupport;
#endif /* USBPDCORE_VPD */
#endif /* _SNK */
  }
#endif  /* USBPDCORE_LIB_NO_PD */
}

/**
  * @brief  function to force CAD state machine into error recovery state
  * @param  PortNum Index of current used port
  * @retval None
  */
void CAD_Enter_ErrorRecovery(uint8_t PortNum)
{
  /* remove the ucpd resistor */
  USBPDM1_EnterErrorRecovery(PortNum);
  /* set the error recovery flag to allow the stack to switch into errorRecovery Flag */
  CAD_HW_Handles[PortNum].CAD_ErrorRecoveryflag = USBPD_TRUE;
  Ports[PortNum].USBPD_CAD_WakeUp();
}

#if defined(USBPDCORE_DRP) || defined(USBPDCORE_SRC)
/**
  * @brief  function to force the value of the RP resistor
  * @note   Must be called only if you want change the settings value
  * @param  PortNum Index of current used port
  * @param  RpValue RP value to set in devices based on @ref CAD_RP_Source_Current_Adv_Typedef
  * @retval 0 success else error
  */
uint32_t CAD_SRC_Set_ResistorRp(uint8_t PortNum, CAD_RP_Source_Current_Adv_Typedef RpValue)
{
  /* update the information about the default resistor value presented in detach mode */
  Ports[PortNum].params->RpResistor = RpValue;

  /* inform state machine about a resistor update */
  CAD_HW_Handles[PortNum].CAD_ResistorUpdateflag = USBPD_TRUE;
  Ports[PortNum].USBPD_CAD_WakeUp();
  return 0;
}

/* Keep for legacy */
uint32_t CAD_Set_ResistorRp(uint8_t PortNum, CAD_RP_Source_Current_Adv_Typedef RpValue)
{
  return CAD_SRC_Set_ResistorRp(PortNum, RpValue);
}
#endif /* USBPDCORE_DRP || USBPDCORE_SRC */


/**
  * @brief  CAD State machine
  * @param  PortNum Port
  * @param  pEvent  Pointer on CAD event based on @ref USBPD_CAD_EVENT
  * @param  pCCXX   Pointer on CC Pin based on @ref CCxPin_TypeDef
  * @retval Timeout value
  */
#if defined(_SNK)
/* function to handle SNK and SNK  + ACCESSORY OPTION */
uint32_t CAD_StateMachine_SNK(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  uint32_t _timing = CAD_DEFAULT_TIME;

#if defined(USBPDM1_VCC_FEATURE_ENABLED)
  BSP_USBPD_PWR_VCCSetState(PortNum, 1);
#endif /* USBPDM1_VCC_FEATURE_ENABLED */

  /*Check CAD STATE*/
  switch (_handle->cstate)
  {
    case USBPD_CAD_STATE_DETACHED:
    {
      _timing = ManageStateDetached_SNK(PortNum);
      break;
    }

    case USBPD_CAD_STATE_ATTACHED_WAIT:
      _timing = ManageStateAttachedWait_SNK(PortNum, pEvent, pCCXX);
      break;

    case USBPD_CAD_STATE_ATTACHED:
      _timing = ManageStateAttached_SNK(PortNum, pEvent, pCCXX);
      break;

#if defined(_ACCESSORY_SNK)
    case USBPD_CAD_STATE_UNATTACHED_ACCESSORY:
    {
      uint32_t cc;
      cc = Ports[PortNum].husbpd->SR & (UCPD_SR_TYPEC_VSTATE_CC1 | UCPD_SR_TYPEC_VSTATE_CC2);

      _handle->CAD_tDebounce_flag = USBPD_FALSE;
      if ((USBPD_TRUE == _handle->CAD_Accessory_SNK) && (cc == (LL_UCPD_SRC_CC1_VRA | LL_UCPD_SRC_CC2_VRA)))
      {
        /* Get the time of this event */
        _handle->CAD_tDebounce_start = HAL_GetTick();
        _handle->cstate = USBPD_CAD_STATE_ATTACHED_ACCESSORY_WAIT;
      }
#if defined(USBPDCORE_VPD)
      else if ((USBPD_TRUE == _handle->CAD_VPD_SNK) &&
               ((cc == (LL_UCPD_SRC_CC1_VRD | LL_UCPD_SRC_CC2_VRA)
                 || ((LL_UCPD_SRC_CC1_VRA | LL_UCPD_SRC_CC2_VRD) == cc)))
              )
      {
        _handle->CAD_tDebounce_start = HAL_GetTick();
        _handle->cstate = USBPD_CAD_STATE_ATTACHED_ACCESSORY_WAIT;
      }
#endif /* USBPDCORE_VPD */
      else if (
        ((cc == (LL_UCPD_SRC_CC1_VRD | LL_UCPD_SRC_CC2_VRA)
          || ((LL_UCPD_SRC_CC1_VRA | LL_UCPD_SRC_CC2_VRD) == cc)))
      )
      {
        if ((HAL_GetTick() - _handle->CAD_tToggle_start) > 200)
        {
          _handle->CAD_tToggle_start = HAL_GetTick();
          _handle->cstate = USBPD_CAD_STATE_DETACHED;
          USBPDM1_AssertRd(PortNum);
        }
      }
      else
      {
        /* tOnePortToggleConnect between 0 and 80ms */
        if ((HAL_GetTick() - _handle->CAD_tToggle_start) > CAD_ACCESSORY_TOGGLE)
        {
          _handle->CAD_tToggle_start = HAL_GetTick();
          _handle->cstate = USBPD_CAD_STATE_DETACHED;
          USBPDM1_AssertRd(PortNum);
        }
      }
      break;
    }

    case USBPD_CAD_STATE_ATTACHED_ACCESSORY_WAIT :
    {
      uint32_t cc;
      cc = Ports[PortNum].husbpd->SR & (UCPD_SR_TYPEC_VSTATE_CC1 | UCPD_SR_TYPEC_VSTATE_CC2);
      _handle->CAD_tDebounce_flag = USBPD_FALSE;
      switch (cc)
      {
        case LL_UCPD_SRC_CC1_VRA | LL_UCPD_SRC_CC2_VRA :      /* Audio accessory */
        {
          /* check if the device is still connected after the debouce timing */
          if (HAL_GetTick() - _handle->CAD_tDebounce_start > CAD_TCCDEBOUCE_THRESHOLD)
          {
            _handle->cstate = USBPD_CAD_STATE_AUDIO_ACCESSORY;
            *pEvent = USBPD_CAD_EVENT_ACCESSORY;
          }
          break;
        }
#if defined(USBPDCORE_VPD)
        case  LL_UCPD_SRC_CC1_VRD | LL_UCPD_SRC_CC2_VRA:   /* VPD CC1 */
        case  LL_UCPD_SRC_CC1_VRA | LL_UCPD_SRC_CC2_VRD:   /* VPD CC2 */
        {
          if (HAL_GetTick() - _handle->CAD_tDebounce_start > CAD_TCCDEBOUCE_THRESHOLD)
          {
            _handle->cstate = USBPD_CAD_STATE_POWERED_ACCESSORY;
            *pEvent = USPPD_CAD_EVENT_VPD;
            Ports[PortNum].params->PE_VPDStatus = VPD_UNKNOWN;
            Ports[PortNum].params->CAD_VPDStatus = VPD_NONE;
            if ((LL_UCPD_SRC_CC1_VRD | LL_UCPD_SRC_CC2_VRA) == cc)
            {
              _handle->cc = CC1;
            }
            else
            {
              _handle->cc = CC2;
            }
            *pCCXX = _handle->cc;
            HW_SignalAttachement(PortNum, _handle->cc);
          }
          break;
        }
#endif /* USBPDCORE_VPD */
        default :
        {
          if ((HAL_GetTick() - _handle->CAD_tDebounce_start) > CAD_TCCDEBOUCE_THRESHOLD)
          {
            /* Get the time of this event */
            _handle->CAD_tToggle_start = HAL_GetTick();
            _handle->cstate = USBPD_CAD_STATE_DETACHED;
            USBPDM1_AssertRd(PortNum);
          }
          break;
        }
      }
      break;
    }

    case USBPD_CAD_STATE_AUDIO_ACCESSORY:
    {
      /* check if the device is still connected after the debouce timing */
      if ((LL_UCPD_SRC_CC1_VRA != (Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC1))
          && (LL_UCPD_SRC_CC2_VRA != (Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC2)))
      {
        if (USBPD_FALSE == _handle->CAD_tDebounce_flag)
        {
          _handle->CAD_tDebounce_start = HAL_GetTick();
          _handle->CAD_tDebounce_flag = USBPD_TRUE;
          _timing = CAD_TCCDEBOUCE_THRESHOLD + 49U;
        }
        else
        {
          if ((HAL_GetTick() - _handle->CAD_tDebounce_start) > (CAD_TCCDEBOUCE_THRESHOLD + 50U))
          {
            _handle->CAD_tToggle_start = HAL_GetTick();
            _handle->CAD_tDebounce_flag = USBPD_FALSE;
            _handle->cstate = USBPD_CAD_STATE_DETACHED;
            _handle->cc = CCNONE;
            USBPDM1_AssertRd(PortNum);
            *pEvent = USBPD_CAD_EVENT_DETACHED;
            *pCCXX = CCNONE;
          }
        }
      }
      else
      {
        _handle->CAD_tDebounce_flag = USBPD_FALSE;
        _timing = CAD_INFINITE_TIME;
      }
      break;
    }

#if defined(USBPDCORE_VPD)
    case USBPD_CAD_STATE_POWERED_ACCESSORY:
    {
      uint32_t cc;
      cc = Ports[PortNum].husbpd->SR & (UCPD_SR_TYPEC_VSTATE_CC1 | UCPD_SR_TYPEC_VSTATE_CC2);

      /* if SRC.Open detected on the monitored PIN switch to Unattached.SNK */
      if (((CC1 == _handle->cc) && (LL_UCPD_SRC_CC1_OPEN == (cc & UCPD_SR_TYPEC_VSTATE_CC1))) ||
          ((CC2 == _handle->cc) && (LL_UCPD_SRC_CC2_OPEN == (cc & UCPD_SR_TYPEC_VSTATE_CC2))))
      {
        _handle->CAD_tToggle_start = HAL_GetTick();
        _handle->cstate = USBPD_CAD_STATE_DETACHED;
        Ports[PortNum].params->PE_VPDStatus = VPD_NONE;
        *pEvent = USBPD_CAD_EVENT_DETACHED;
        USBPDM1_AssertRd(PortNum);
      }
      else
      {
        /*  if the port is not PD or a SRC it must switch to try.SNK */
        if (VPD_NOPD == Ports[PortNum].params->CAD_VPDStatus)
        {
          _handle->CAD_tToggle_start = HAL_GetTick();
          _handle->cstate = USBPD_CAD_STATE_DETACHED;
          *pEvent = USBPD_CAD_EVENT_DETACHED;
          USBPDM1_AssertRd(PortNum);
          Ports[PortNum].params->PE_VPDStatus = VPD_NONE;
        }
        /* if the port is PD but doesn't enter Alternate mode within tAMETimeout switch to Unsupported.Accessory */
        else if (VPD_FAILED_ENTER_ALTERNATE == Ports[PortNum].params->CAD_VPDStatus)
        {
          _handle->cstate = USBPD_CAD_STATE_UNSUPPORTED_ACCESSORY;
        }
        /* if Powered USB DEvice confirmed transition to CTUnattached.SNK */
        else if (VPD_DETECTED == Ports[PortNum].params->CAD_VPDStatus)
        {
          _handle->cstate = USBPD_CAD_STATE_CTVPD_UNATTACHED;
        }
      }
      break;
    }

    case USBPD_CAD_STATE_UNSUPPORTED_ACCESSORY:
    {
      uint32_t cc;
      cc = Ports[PortNum].husbpd->SR & (UCPD_SR_TYPEC_VSTATE_CC1 | UCPD_SR_TYPEC_VSTATE_CC2);

      /* if SRC.Open detected on the monitored PIN switch to Unattached.SNK */
      if ((LL_UCPD_SRC_CC1_OPEN == (cc | UCPD_SR_TYPEC_VSTATE_CC1)) ||
          (LL_UCPD_SRC_CC2_OPEN == (cc | UCPD_SR_TYPEC_VSTATE_CC2)))
      {
        _handle->CAD_tToggle_start = HAL_GetTick();
        _handle->cstate = USBPD_CAD_STATE_DETACHED;
        *pEvent = USBPD_CAD_EVENT_DETACHED;
        USBPDM1_AssertRp(PortNum);
      }
      break;
    }

    case USBPD_CAD_STATE_CTVPD_UNATTACHED:
    {
      if (USBPD_TRUE == USBPD_PWR_IF_GetVBUSStatus(PortNum, USBPD_PWR_VSAFE5V)) /* Check if Vbus is on */
      {
        /* the charger/SRC has been plug */
        _handle->cstate = USBPD_CAD_STATE_CTVPD_ATTACHED;
      }
      else
      {
        uint32_t cc;
        cc = Ports[PortNum].husbpd->SR & (UCPD_SR_TYPEC_VSTATE_CC1 | UCPD_SR_TYPEC_VSTATE_CC2);

        /* if VBUS is VSafe0V and CC low for tVPDDetach = 10/20ms */
        if (((LL_UCPD_SRC_CC1_OPEN == (cc | UCPD_SR_TYPEC_VSTATE_CC1)) ||
             (LL_UCPD_SRC_CC2_OPEN == (cc | UCPD_SR_TYPEC_VSTATE_CC2))) &&
            (USBPD_TRUE == USBPD_PWR_IF_GetVBUSStatus(PortNum, USBPD_PWR_BELOWVSAFE0V)))
        {
          if (USBPD_TRUE == _handle->CAD_tDebounce_flag)
          {
            _handle->CAD_tDebounce_start = HAL_GetTick();
          }
          else
          {
            if (HAL_GetTick() - _handle->CAD_tDebounce_start > CAD_TVPDDETACH)
            {
              _handle->CAD_tToggle_start = HAL_GetTick();
              _handle->cstate = USBPD_CAD_STATE_DETACHED;
              *pEvent = USBPD_CAD_EVENT_DETACHED;
              USBPDM1_AssertRp(PortNum);
            }
          }
        }
        else
        {
          _handle->CAD_tDebounce_flag = USBPD_FALSE;
        }
      }
      break;
    }

    case USBPD_CAD_STATE_CTVPD_ATTACHED:
    {
      /* if VBUS removed */
      if (USBPD_TRUE == USBPD_PWR_IF_GetVBUSStatus(PortNum, USBPD_PWR_SNKDETACH))
      {
        /* the charger/SRC has been unplug */
        _handle->cstate = USBPD_CAD_STATE_CTVPD_UNATTACHED;
      }
      break;
    }
#endif /* USBPDCORE_VPD */
#endif /* _ACCESSORY_SNK */

    default:
    {
      break;
    }
  }

#if defined(USBPDM1_VCC_FEATURE_ENABLED)
  switch (_handle->pstate)
  {
    case USBPD_CAD_STATE_ATTACHED_WAIT:
    case USBPD_CAD_STATE_ATTACHED:
    case USBPD_CAD_STATE_ERRORRECOVERY :
    case USBPD_CAD_STATE_ERRORRECOVERY_EXIT:
      /* nothing to do, the VCC must stay high */
      break;
    default :
      BSP_USBPD_PWR_VCCSetState(PortNum, 0);
      break;
  }
#endif /* USBPDM1_VCC_FEATURE_ENABLED */

  return _timing;
}
#endif /* _SNK */

#if defined(_SRC)
/* function to handle SRC */
uint32_t CAD_StateMachine_SRC(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  uint32_t _timing = CAD_DEFAULT_TIME;

  /*Check CAD STATE*/
  switch (_handle->cstate)
  {
    case USBPD_CAD_STATE_DETACH_SRC :
    {
#if defined(_VCONN_SUPPORT)
      /* DeInitialize Vconn management */
      (void)BSP_USBPD_PWR_VCONNDeInit(PortNum, (Ports[PortNum].CCx == CC1) ? 1u : 2u);
#endif /* _VCONN_SUPPORT */
      /* DeInitialise VBUS power */
      (void)BSP_USBPD_PWR_VBUSDeInit(PortNum);
      /* Reset the resistor */
      USBPDM1_AssertRp(PortNum);
      _handle->cstate = USBPD_CAD_STATE_DETACHED;
      _timing = 0;
      break;
    }

    case USBPD_CAD_STATE_SWITCH_TO_SNK :
    case USBPD_CAD_STATE_DETACHED:
    {
      _timing = ManageStateDetached_SRC(PortNum);
      break;
    }

    case USBPD_CAD_STATE_ATTACHED_WAIT:
    {
      _timing = ManageStateAttachedWait_SRC(PortNum, pEvent, pCCXX);
      break;
    }

#if defined(_ACCESSORY_SRC)
    case USBPD_CAD_STATE_ACCESSORY:
    {
      _timing = CAD_INFINITE_TIME;
      CAD_Check_HW_SRC(PortNum);
      if ((HW_AudioAdapter_Attachment != _handle->CurrentHWcondition)
          && (HW_PwrCable_Sink_Attachment != _handle->CurrentHWcondition))
      {
        _handle->cstate = USBPD_CAD_STATE_DETACH_SRC;
        *pCCXX = CCNONE;
        _handle->cc = CCNONE;
        *pEvent = USBPD_CAD_EVENT_DETACHED;
      }
      break;
    }

    case USBPD_CAD_STATE_DEBUG:
    {
      _timing = CAD_INFINITE_TIME;
      CAD_Check_HW_SRC(PortNum);
      if (_handle->CurrentHWcondition != HW_Debug_Attachment)
      {
        _handle->cstate = USBPD_CAD_STATE_DETACH_SRC;
        *pCCXX = CCNONE;
        _handle->cc = CCNONE;
        *pEvent = USBPD_CAD_EVENT_DETACHED;
      }
      break;
    }
#endif /* _ACCESSORY_SRC */

    case USBPD_CAD_STATE_EMC :
    {
      _timing = ManageStateEMC(PortNum, pEvent, pCCXX);
      break;
    }

    /*CAD electronic cable with Sink ATTACHED*/
    case USBPD_CAD_STATE_ATTEMC:
    case USBPD_CAD_STATE_ATTACHED:
    {
      _timing = ManageStateAttached_SRC(PortNum, pEvent, pCCXX);
      break;
    }

    default :
    {
      break;
    }
  }

  return _timing;
}
#endif /* _SRC */

#if defined(_DRP)
/* function to handle DRP */
uint32_t CAD_StateMachine_DRP(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  uint32_t _timing = CAD_DEFAULT_TIME;

  /*Check CAD STATE*/
  switch (_handle->cstate)
  {
    case USBPD_CAD_STATE_DETACH_SRC :
    {
#if defined(_VCONN_SUPPORT)
      /* DeInitialize Vconn management */
      (void)BSP_USBPD_PWR_VCONNDeInit(PortNum, (Ports[PortNum].CCx == CC1) ? 1u : 2u);
#endif /* _VCONN_SUPPORT */
      /* DeInitialise VBUS power */
      (void)BSP_USBPD_PWR_VBUSDeInit(PortNum);
      _timing = 0;
      _handle->cstate = USBPD_CAD_STATE_SWITCH_TO_SNK;
      break;
    }

    case USBPD_CAD_STATE_SWITCH_TO_SRC:
    case USBPD_CAD_STATE_SWITCH_TO_SNK:
    {
      LL_UCPD_RxDisable(Ports[PortNum].husbpd);
      if (USBPD_CAD_STATE_SWITCH_TO_SRC == _handle->cstate)
      {
        USBPDM1_AssertRp(PortNum);
        Ports[PortNum].params->PE_PowerRole = USBPD_PORTPOWERROLE_SRC;
        Ports[PortNum].params->PE_DataRole = USBPD_PORTDATAROLE_DFP;
        _timing = Ports[PortNum].settings->CAD_SRCToggleTime;
      }
      if (USBPD_CAD_STATE_SWITCH_TO_SNK == _handle->cstate)
      {
        USBPDM1_AssertRd(PortNum);
        Ports[PortNum].params->PE_PowerRole = USBPD_PORTPOWERROLE_SNK;
        Ports[PortNum].params->PE_DataRole = USBPD_PORTDATAROLE_UFP;
        _timing = Ports[PortNum].settings->CAD_SNKToggleTime;
      }
      _handle->CAD_tToggle_start = HAL_GetTick();
      _handle->cstate = USBPD_CAD_STATE_DETACHED;
    }
    break;

    case USBPD_CAD_STATE_DETACHED:
      _timing = ManageStateDetached_DRP(PortNum);
      break;

    /*CAD STATE ATTACHED WAIT*/
    case USBPD_CAD_STATE_ATTACHED_WAIT:
      _timing = ManageStateAttachedWait_DRP(PortNum, pEvent, pCCXX);
      break;

#if defined(_ACCESSORY_SRC)
    case USBPD_CAD_STATE_ACCESSORY:
    {
      CAD_Check_HW_SRC(PortNum);
      if ((HW_AudioAdapter_Attachment != _handle->CurrentHWcondition)
          && (HW_PwrCable_Sink_Attachment != _handle->CurrentHWcondition))
      {
        _handle->cstate = USBPD_CAD_STATE_DETACH_SRC;
        *pCCXX = CCNONE;
        _handle->cc = CCNONE;
        *pEvent = USBPD_CAD_EVENT_DETACHED;
      }
      else
      {
        _timing = CAD_INFINITE_TIME;
      }
      break;
    }

    case USBPD_CAD_STATE_DEBUG:
    {
      _timing = CAD_INFINITE_TIME;
      CAD_Check_HW_SRC(PortNum);
      if (_handle->CurrentHWcondition != HW_Debug_Attachment)
      {
        _handle->cstate = USBPD_CAD_STATE_DETACH_SRC;
        *pCCXX = CCNONE;
        _handle->cc = CCNONE;
        *pEvent = USBPD_CAD_EVENT_DETACHED;
      }
      break;
    }
#endif /* _ACCESSORY_SRC */

    /* CAD ELECTRONIC CABLE ATTACHED */
    case USBPD_CAD_STATE_EMC :
      _timing = ManageStateEMC(PortNum, pEvent, pCCXX);
      break;

    /*CAD electronic cable with Sink ATTACHED*/
    case USBPD_CAD_STATE_ATTEMC:
    case USBPD_CAD_STATE_ATTACHED:
      _timing = ManageStateAttached_DRP(PortNum, pEvent, pCCXX);
      break;

    default :
      break;
  }

  return _timing;
}
#endif /* _DRP */

#if !defined(USBPDCORE_LIB_NO_PD)
uint32_t CAD_StateMachine(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  uint32_t _timing = CAD_DEFAULT_TIME;

  /* set by default event to none */
  *pEvent = USBPD_CAD_EVENT_NONE;

  if (USBPD_TRUE == Ports[PortNum].params->PE_SwapOngoing)
  {
    return _timing;
  }

  if (_handle->CAD_ErrorRecoveryflag == USBPD_TRUE)
  {
    /* Force the state error recovery */
    _handle->CAD_ErrorRecoveryflag = USBPD_FALSE;
    _handle->cstate = USBPD_CAD_STATE_ERRORRECOVERY;
#if defined(_TRACE)
    USBPD_TRACE_Add(USBPD_TRACE_CAD_LOW, PortNum, (uint8_t)_handle->cstate, NULL, 0);
#endif /* _TRACE */
  }

  switch (_handle->cstate)
  {
    case USBPD_CAD_STATE_RESET:
    {
#if !defined(_LOW_POWER) && !defined(USBPDM1_VCC_FEATURE_ENABLED)
      LL_UCPD_EnableIT_TypeCEventCC2(Ports[PortNum].husbpd);
      LL_UCPD_EnableIT_TypeCEventCC1(Ports[PortNum].husbpd);
#elif defined(_LOW_POWER)
      if (USBPD_PORTPOWERROLE_SRC == Ports[PortNum].params->PE_PowerRole)
      {
        LL_UCPD_EnableIT_TypeCEventCC2(Ports[PortNum].husbpd);
        LL_UCPD_EnableIT_TypeCEventCC1(Ports[PortNum].husbpd);
      }
#endif /* !_LOW_POWER && !USBPDM1_VCC_FEATURE_ENABLED */

      UCPD_INSTANCE0_ENABLEIRQ;
#if defined(_DRP) || defined(_ACCESSORY_SNK)
      _handle->CAD_tToggle_start = HAL_GetTick();
#endif /* _DRP || _ACCESSORY_SNK */
      _handle->cstate = USBPD_CAD_STATE_DETACHED;
      break;
    }

    case USBPD_CAD_STATE_ERRORRECOVERY :
    {
      /* Remove the resistor */
      /* Enter recovery = Switch to SRC with no resistor */
      USBPDM1_EnterErrorRecovery(PortNum);

      /* forward detach event to DPM */
      Ports[PortNum].CCx = CCNONE;
      *pCCXX = CCNONE;
      _handle->cc = CCNONE;
      *pEvent = USBPD_CAD_EVENT_DETACHED;

      /* start tErrorRecovery timeout */
      _handle->CAD_tDebounce_start = HAL_GetTick();
      _timing = CAD_TERROR_RECOVERY_TIME;
      _handle->cstate = USBPD_CAD_STATE_ERRORRECOVERY_EXIT;
      break;
    }

    case USBPD_CAD_STATE_ERRORRECOVERY_EXIT :
    {
      if ((HAL_GetTick() - _handle->CAD_tDebounce_start) >  CAD_TERROR_RECOVERY_TIME)
      {
        /* reconfigure the port
        port source  to src
        port snk     to snk
        port drp     to src   */

#if defined(_SRC) || defined(_DRP)
        if (USBPD_PORTPOWERROLE_SRC == Ports[PortNum].params->PE_PowerRole)
        {
          USBPDM1_AssertRp(PortNum);
        }
#endif /* _SRC || _DRP */
#if defined(_DRP)
        else
#endif /* _DRP */
#if defined(_SNK) || defined(_DRP)
        {
          USBPDM1_AssertRd(PortNum);
        }
#endif /* _SNK || _DRP */
        /* switch to state detach */
#if defined(_DRP) || defined(_ACCESSORY_SNK)
        _handle->CAD_tToggle_start = HAL_GetTick();
#endif /* _DRP || _ACCESSORY_SNK */
        _handle->cstate = USBPD_CAD_STATE_DETACHED;
      }
      break;
    }

    default:
    {
      /* call the state machine corresponding to the port SNK or SRC or DRP */
      _timing = _handle->CAD_PtrStateMachine(PortNum, pEvent, pCCXX);
      break;
    }
  }

#if defined(_TRACE)
  if (_handle->cstate != _handle->pstate)
  {
    _handle->pstate = _handle->cstate;
    USBPD_TRACE_Add(USBPD_TRACE_CAD_LOW, PortNum, (uint8_t)_handle->cstate, NULL, 0);
#if defined(CAD_DEBUG_TRACE)
    if (USBPD_PORTPOWERROLE_SRC == Ports[PortNum].params->PE_PowerRole)
    {
      USBPD_TRACE_Add(USBPD_TRACE_CAD_LOW, PortNum, (uint8_t)(0xC0 | _handle->CurrentHWcondition), NULL, 0);
    }
    else
    {
      USBPD_TRACE_Add(USBPD_TRACE_CAD_LOW, PortNum,
                      (uint8_t)(0x80 | (Ports[PortNum].params->SNKExposedRP_AtAttach << 4) |
                                _handle->CurrentHWcondition), NULL, 0);
    }
#endif /* CAD_DEBUG_TRACE */
  }
#endif /* _TRACE */

  return _timing;
}

#else
/* USBPDCORE_LIB_NO_PD */
uint32_t CAD_StateMachine(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  uint32_t _timing = CAD_DEFAULT_TIME;

  /* set by default event to none */
  *pEvent = USBPD_CAD_EVENT_NONE;

  switch (_handle->cstate)
  {
    case USBPD_CAD_STATE_RESET:
    {
#ifndef _LOW_POWER
      LL_UCPD_EnableIT_TypeCEventCC2(Ports[PortNum].husbpd);
      LL_UCPD_EnableIT_TypeCEventCC1(Ports[PortNum].husbpd);
#else
      if (USBPD_PORTPOWERROLE_SRC == Ports[PortNum].params->PE_PowerRole)
      {
        LL_UCPD_EnableIT_TypeCEventCC2(Ports[PortNum].husbpd);
        LL_UCPD_EnableIT_TypeCEventCC1(Ports[PortNum].husbpd);
      }
#endif /* !_LOW_POWER */

      UCPD_INSTANCE0_ENABLEIRQ;
#if defined(_DRP) || defined(_ACCESSORY_SNK)
      _handle->CAD_tToggle_start = HAL_GetTick();
#endif /* _DRP || _ACCESSORY_SNK */
      _handle->cstate = USBPD_CAD_STATE_DETACHED;
      break;
    }

    default:
    {
      /* call the state machine corresponding to the port SNK or SRC or DRP */
      _timing = _handle->CAD_PtrStateMachine(PortNum, pEvent, pCCXX);
      break;
    }
  }

#if defined(_TRACE)
  if (_handle->cstate != _handle->pstate)
  {
    _handle->pstate = _handle->cstate;
    USBPD_TRACE_Add(USBPD_TRACE_CAD_LOW, PortNum, (uint8_t)_handle->cstate, NULL, 0);
#if defined(CAD_DEBUG_TRACE)
    if (USBPD_PORTPOWERROLE_SRC == Ports[PortNum].params->PE_PowerRole)
    {
      USBPD_TRACE_Add(USBPD_TRACE_CAD_LOW, PortNum, (uint8_t)(0xC0 | _handle->CurrentHWcondition), NULL, 0);
    }
    else
    {
      USBPD_TRACE_Add(USBPD_TRACE_CAD_LOW, PortNum, (uint8_t)(0x80 |
                                                              (Ports[PortNum].params->SNKExposedRP_AtAttach << 4) |
                                                              _handle->CurrentHWcondition), NULL, 0);
    }
#endif /* CAD_DEBUG_TRACE */
  }
#endif /* _TRACE */

  return _timing;
}
#endif /* !USBPDCORE_LIB_NO_PD */

/**
  * @}
  */

/** @addtogroup USBPD_DEVICE_CAD_HW_IF_Private_Functions
  * @{
  */

/**
  * @brief  Check CCx HW condition
  * @param  PortNum                     port
  * @retval none
  */
#if defined(_DRP) || defined(_SNK)
void CAD_Check_HW_SNK(uint8_t PortNum)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  /* done to prevent code optimization issue with GCC */
  uint32_t CC1_value;
  uint32_t CC2_value;

  /*
  ----------------------------------------------------------------------------
  | ANAMODE   |  ANASUBMODE[1:0]  |  Notes      |  TYPEC_VSTATE_CCx[1:0]      |
  |           |                   |             |  00  |  01  |  10  |  11    |
  ----------------------------------------------------------------------------
  | 0: Source | 00: Disabled      |Disabled N/A |         NA                  |
  |           | 01: Default USB Rp|             |vRaDef|vRdDef|vOPENDef|      |
  |           | 10: 1.5A Rp       |             |vRa1.5|vRd1.5|vOPEN1.5| NA   |
  |           | 11: 3.0A Rp       |             |vRa3.0|vRd3.0|vOPEN3.0| NA   |
  -----------------------------------------------------------------------------
  | 1: Sink   |                   |             |xx vRa|vRdUSB| vRd1.5 |vRd3.0|
  -----------------------------------------------------------------------------
  */

#if defined(_LOW_POWER) || defined(USBPDM1_VCC_FEATURE_ENABLED)
  /* Enable type C state machine */
  CLEAR_BIT(Ports[PortNum].husbpd->CR, UCPD_CR_CC1TCDIS | UCPD_CR_CC2TCDIS);

  for (int32_t index = 0; index < CAD_DELAY_READ_CC_STATUS; index++)
  {
    __DSB();
  };

  /* Read the CC line */
  CC1_value = Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC1;
  CC2_value = Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC2;

  /* Disable the C state machine */
  SET_BIT(Ports[PortNum].husbpd->CR, UCPD_CR_CC1TCDIS | UCPD_CR_CC2TCDIS);
#else
  CC1_value = Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC1;
  CC2_value = Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC2;
#endif /* _LOW_POWER || USBPDM1_VCC_FEATURE_ENABLED */

  _handle->cc  = CCNONE;
  _handle->CurrentHWcondition     = HW_Detachment;

  if ((CC1_value != LL_UCPD_SNK_CC1_VOPEN) && (CC2_value == LL_UCPD_SNK_CC2_VOPEN))
  {
    _handle->CurrentHWcondition = HW_Attachment;
    _handle->cc = CC1;
    Ports[PortNum].params->SNKExposedRP_AtAttach = CC1_value >> UCPD_SR_TYPEC_VSTATE_CC1_Pos;
  }

  if ((CC1_value == LL_UCPD_SNK_CC1_VOPEN) && (CC2_value != LL_UCPD_SNK_CC2_VOPEN))
  {
    _handle->CurrentHWcondition = HW_Attachment;
    _handle->cc = CC2;
    Ports[PortNum].params->SNKExposedRP_AtAttach = CC2_value >> UCPD_SR_TYPEC_VSTATE_CC2_Pos;;
  }
}
#endif /* _DRP || _SNK */

#if defined(_DRP) || defined(_SRC)
void CAD_Check_HW_SRC(uint8_t PortNum)
{
#if !defined(_RTOS)
  uint32_t CC1_value_temp;
  uint32_t CC2_value_temp;
#endif /* !_RTOS */
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  /* done to prevent code optimization issue with GCC */
  uint32_t CC1_value;
  uint32_t CC2_value;

  /*
  ----------------------------------------------------------------------------
  | ANAMODE   |  ANASUBMODE[1:0]  |  Notes      |  TYPEC_VSTATE_CCx[1:0]      |
  |           |                   |             |  00  |  01  |  10  |  11    |
  ----------------------------------------------------------------------------
  | 0: Source | 00: Disabled      |Disabled N/A |         NA                  |
  |           | 01: Default USB Rp|             |vRaDef|vRdDef|vOPENDef|      |
  |           | 10: 1.5A Rp       |             |vRa1.5|vRd1.5|vOPEN1.5| NA   |
  |           | 11: 3.0A Rp       |             |vRa3.0|vRd3.0|vOPEN3.0| NA   |
  -----------------------------------------------------------------------------
  | 1: Sink   |                   |             |xx vRa|vRdUSB| vRd1.5 |vRd3.0|
  -----------------------------------------------------------------------------
  */

  CC1_value = (Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC1) >> UCPD_SR_TYPEC_VSTATE_CC1_Pos;
  CC2_value = (Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC2) >> UCPD_SR_TYPEC_VSTATE_CC2_Pos;

#if !defined(_RTOS)
  /* Workaround linked to issue with Ellisys test TD.PC.E5
  - it seems that in NRTOS version, we detect a glitch during DRP transition SNK to SRC */
  CC1_value_temp = (Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC1) >> UCPD_SR_TYPEC_VSTATE_CC1_Pos;
  CC2_value_temp = (Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC2) >> UCPD_SR_TYPEC_VSTATE_CC2_Pos;
  if ((CC1_value_temp != CC1_value) || (CC2_value_temp != CC2_value))
  {
    return;
  }
#endif /* !_RTOS */

  const CCxPin_TypeDef table_cc[] =
  {
    CCNONE,  CC2,       CC2,
    CC1,     CCNONE,   CC1,
    CC1,     CC2,       CCNONE
  };

  const CAD_HW_Condition_TypeDef table_CurrentHWcondition[] =
  {
    HW_AudioAdapter_Attachment,  HW_PwrCable_Sink_Attachment, HW_PwrCable_NoSink_Attachment,
    HW_PwrCable_Sink_Attachment,         HW_Debug_Attachment,                 HW_Attachment,
    HW_PwrCable_NoSink_Attachment,             HW_Attachment,                 HW_Detachment
  };

  if (CC1_value * 3 + CC2_value < 9)
  {
    _handle->cc  = table_cc[CC1_value * 3 + CC2_value];
    _handle->CurrentHWcondition     = table_CurrentHWcondition[CC1_value * 3 + CC2_value];
  }
}
#endif /* _DRP || _SRC */

#if defined(_DRP) || defined(_SNK)
static uint32_t ManageStateDetached_SNK(uint8_t PortNum)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  uint32_t _timing = CAD_DEFAULT_TIME;

  CAD_Check_HW_SNK(PortNum);
  /* Change the status on the basis of the HW event given by CAD_Check_HW() */
  if (_handle->CurrentHWcondition == HW_Detachment)
  {
#if defined(_LOW_POWER)
    /* value returned by a SRC or a SINK */
    _timing = CAD_DETACH_POLLING; /* 100ms in the sink cases */
#elif defined(USBPDM1_VCC_FEATURE_ENABLED)
    _timing = CAD_DEFAULT_TIME;
#else
    _timing = CAD_INFINITE_TIME;
#endif /* _LOW_POWER */

#if defined(_ACCESSORY_SNK)
    if ((USBPD_TRUE == _handle->CAD_Accessory_SNK)
#if defined(USBPDCORE_VPD)
        || (USBPD_TRUE == _handle->CAD_VPD_SNK)
#endif /* USBPDCORE_VPD */
       )
    {
      /* A Sink with Accessory support shall transition to Unattached.Accessory within tDRPTransition
      after the state of both the CC1 and CC2 pins is SNK.Open for tDRP - dcSRC.DRP * tDRP, or if directed.*/
      if ((HAL_GetTick() - _handle->CAD_tToggle_start) > CAD_ACCESSORY_TOGGLE)
      {
        _handle->cstate = USBPD_CAD_STATE_UNATTACHED_ACCESSORY;
        _handle->CAD_tToggle_start = HAL_GetTick();
        USBPDM1_AssertRp(PortNum);
      }
      _timing = CAD_DEFAULT_TIME;
    }
#endif /* _ACCESSORY_SNK */
  }
  else
  {
    /* Get the time of this event */
    _handle->CAD_tDebounce_start = HAL_GetTick();
    _handle->cstate = USBPD_CAD_STATE_ATTACHED_WAIT;

    /* Temporary patch for test TD.PD 4.5.2 + rework for Patch TP.PD.C.E5 */
    HAL_Delay(1);
    CAD_Check_HW_SNK(PortNum);

    if (_handle->CurrentHWcondition == HW_Detachment)
    {
      _handle->cstate = USBPD_CAD_STATE_DETACHED;
    }
    else
    {
      BSP_USBPD_PWR_VBUSInit(PortNum);
    }
  }
  return _timing;
}
#endif /* _DRP || _SNK */

#if defined(_SRC) || defined(_DRP)
static uint32_t ManageStateDetached_SRC(uint8_t PortNum)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  uint32_t _timing = CAD_DEFAULT_TIME;

  if (_handle->CAD_ResistorUpdateflag == USBPD_TRUE)
  {
    /* update the resistor value */
    USBPDM1_AssertRp(PortNum);
    _handle->CAD_ResistorUpdateflag = USBPD_FALSE;

    /* let time to internal state machine update */
    HAL_Delay(1);
  }

  CAD_Check_HW_SRC(PortNum);
  /* Change the status on the basis of the HW event given by CAD_Check_HW() */
  if (_handle->CurrentHWcondition == HW_Detachment)
  {
#ifdef _LOW_POWER
    /* value returned for a SRC */
    _timing = CAD_DETACH_POLLING;
#else
    _timing = CAD_INFINITE_TIME;
#endif /* _LOW_POWER */
  }
  else
  {
    if (_handle->CurrentHWcondition == HW_PwrCable_NoSink_Attachment)
    {
      _handle->cstate = USBPD_CAD_STATE_EMC;
    }
    else
    {
      /* Get the time of this event */
      _handle->CAD_tDebounce_start = HAL_GetTick();
      _handle->cstate = USBPD_CAD_STATE_ATTACHED_WAIT;

      BSP_USBPD_PWR_VBUSInit(PortNum);
    }
  }
  return _timing;
}
#endif /* _SRC || _DRP */

#if defined(_DRP)
static uint32_t ManageStateDetached_DRP(uint8_t PortNum)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  uint32_t _timing = CAD_DEFAULT_TIME;

  if (USBPD_PORTPOWERROLE_SRC == Ports[PortNum].params->PE_PowerRole)
  {
    ManageStateDetached_SRC(PortNum);
  }
  else
  {
    ManageStateDetached_SNK(PortNum);
  }

  /* Manage the toggle */
  if (_handle->CurrentHWcondition == HW_Detachment)
  {
    switch (Ports[PortNum].params->PE_PowerRole)
    {
      case USBPD_PORTPOWERROLE_SRC :
        if ((HAL_GetTick() - _handle->CAD_tToggle_start) > Ports[PortNum].settings->CAD_SRCToggleTime)
        {
          _handle->CAD_tToggle_start = HAL_GetTick();
          Ports[PortNum].params->PE_PowerRole = USBPD_PORTPOWERROLE_SNK;
          Ports[PortNum].params->PE_DataRole = USBPD_PORTDATAROLE_UFP;
          _timing = Ports[PortNum].settings->CAD_SNKToggleTime;
          USBPDM1_AssertRd(PortNum);
        }
        break;
      case USBPD_PORTPOWERROLE_SNK :
        if ((HAL_GetTick() - _handle->CAD_tToggle_start) > Ports[PortNum].settings->CAD_SNKToggleTime)
        {
          _handle->CAD_tToggle_start = HAL_GetTick();
          Ports[PortNum].params->PE_PowerRole = USBPD_PORTPOWERROLE_SRC;
          Ports[PortNum].params->PE_DataRole = USBPD_PORTDATAROLE_DFP;
          _timing = Ports[PortNum].settings->CAD_SRCToggleTime;
          USBPDM1_AssertRp(PortNum);
        }
        break;
      default:
        break;
    }
  }

  return _timing;
}
#endif /* _DRP */

#if defined(_DRP) || defined(_SRC)
static uint32_t ManageStateAttachedWait_SRC(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  uint32_t _timing = 2;

  /* Evaluate elapsed time in Attach_Wait state */
  uint32_t CAD_tDebounce = HAL_GetTick() - _handle->CAD_tDebounce_start;
  CAD_Check_HW_SRC(PortNum);

  if ((_handle->CurrentHWcondition != HW_Detachment) && (_handle->CurrentHWcondition != HW_PwrCable_NoSink_Attachment))
  {
    if (USBPD_FALSE == USBPD_PWR_IF_GetVBUSStatus(PortNum, USBPD_PWR_BELOWVSAFE0V))
    {
      /* reset the timing because VBUS threshold not yet reach */
      _handle->CAD_tDebounce_start = HAL_GetTick();
      return CAD_TCCDEBOUCE_THRESHOLD;
    }

    /* Check tCCDebounce */
    if (CAD_tDebounce > CAD_TCCDEBOUCE_THRESHOLD)
    {
      switch (_handle->CurrentHWcondition)
      {
        case HW_Attachment:
          HW_SignalAttachement(PortNum, _handle->cc);
          _handle->cstate = USBPD_CAD_STATE_ATTACHED;
          *pEvent = USBPD_CAD_EVENT_ATTACHED;
          *pCCXX = _handle->cc;
          break;

        case HW_PwrCable_Sink_Attachment:
          HW_SignalAttachement(PortNum, _handle->cc);
          _handle->cstate = USBPD_CAD_STATE_ATTEMC;
          *pEvent = USBPD_CAD_EVENT_ATTEMC;
          break;

        case HW_PwrCable_NoSink_Attachment:
          BSP_USBPD_PWR_VBUSDeInit(PortNum);
          _handle->cstate = USBPD_CAD_STATE_EMC;
          *pEvent = USBPD_CAD_EVENT_EMC;
          *pCCXX = _handle->cc;
          break;

#if defined(_ACCESSORY_SRC)
        case HW_Debug_Attachment:
          _handle->cstate = USBPD_CAD_STATE_DEBUG;
          *pEvent = USBPD_CAD_EVENT_DEBUG;
          break;

        case HW_AudioAdapter_Attachment:
          _handle->cstate = USBPD_CAD_STATE_ACCESSORY;
          *pEvent = USBPD_CAD_EVENT_ACCESSORY;
          break;
#endif /* _ACCESSORY_SRC */

        case HW_Detachment:
        default:
#if !defined(_ACCESSORY_SRC)
          _handle->cstate             = USBPD_CAD_STATE_DETACH_SRC;
#endif /* _ACCESSORY_SRC */
          break;
      } /* end of switch */
      *pCCXX = _handle->cc;
      _timing = 2;
    }
    /* reset the flag for CAD_tDebounce */
    _handle->CAD_tDebounce_flag = USBPD_FALSE;
  }
  else /* CAD_HW_Condition[PortNum] = HW_Detachment */
  {
    /* start counting of CAD_tDebounce */
    if (USBPD_FALSE == _handle->CAD_tDebounce_flag)
    {
      _handle->CAD_tDebounce_start  = HAL_GetTick();
      _handle->CAD_tDebounce_flag   = USBPD_TRUE;
      _timing                       = CAD_TSRCDISCONNECT_THRESHOLD;
    }
    else /* CAD_tDebounce already running */
    {
      /* evaluate CAD_tDebounce */
      CAD_tDebounce = HAL_GetTick() - _handle->CAD_tDebounce_start;
      if (CAD_tDebounce > CAD_TSRCDISCONNECT_THRESHOLD)
      {
        _handle->CAD_tDebounce_flag = USBPD_FALSE;
        _handle->cstate             = USBPD_CAD_STATE_DETACH_SRC;
        _timing = 0;
      }
    }
  }
  return _timing;
}
#endif /* _DRP || _SRC || (_ACCESSORY && _SNK) */

#if defined(_DRP) || defined(_SRC)
static uint32_t ManageStateEMC(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX)
{
  uint32_t _timing = CAD_INFINITE_TIME;
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];

  CAD_Check_HW_SRC(PortNum);
  /* Change the status on the basis of the HW event given by CAD_Check_HW() */
  switch (_handle->CurrentHWcondition)
  {
    case HW_Detachment :
      _handle->cstate = USBPD_CAD_STATE_SWITCH_TO_SNK;
      _timing = 1;
      break;
    case HW_PwrCable_Sink_Attachment:
    case HW_Attachment :
      _handle->cstate = USBPD_CAD_STATE_ATTACHED_WAIT;
      _handle->CAD_tDebounce_start = HAL_GetTick() - 5u;  /* this is only to check cable presence */
      BSP_USBPD_PWR_VBUSInit(PortNum);
      _timing = 2;
      break;
    case HW_PwrCable_NoSink_Attachment:
    default :
      /* nothing to do still the same status */
#if defined(_DRP)
      if (USBPD_TRUE == Ports[PortNum].settings->CAD_RoleToggle)
      {
        if ((HAL_GetTick() - _handle->CAD_tToggle_start) > Ports[PortNum].settings->CAD_SRCToggleTime)
        {
          _handle->cstate = USBPD_CAD_STATE_SWITCH_TO_SNK;
        }
        _timing = 0;
      }
#else
      _timing = 2;
#endif /* _DRP */
      break;
  }
  return _timing;
}
#endif /* _DRP || _SRC */

#if defined(_DRP)
static uint32_t ManageStateAttached_DRP(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX)
{
  if (USBPD_PORTPOWERROLE_SRC == Ports[PortNum].params->PE_PowerRole)
  {
    return ManageStateAttached_SRC(PortNum, pEvent, pCCXX);
  }
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  uint32_t _timing = ManageStateAttached_SNK(PortNum, pEvent, pCCXX);

  /* Toggle management */
  if (_handle->CurrentHWcondition == HW_Detachment)
  {
    _handle->cstate = USBPD_CAD_STATE_SWITCH_TO_SRC;
    _timing = 0;
  }
  return _timing;
}
#endif /* _DRP */

#if defined(_DRP) || (defined(_ACCESSORY) && defined(_SNK))
static uint32_t ManageStateAttachedWait_DRP(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX)
{
  if (USBPD_PORTPOWERROLE_SRC == Ports[PortNum].params->PE_PowerRole)
  {
    return ManageStateAttachedWait_SRC(PortNum, pEvent, pCCXX);
  }
  return ManageStateAttachedWait_SNK(PortNum, pEvent, pCCXX);
}
#endif /* _DRP || (_ACCESSORY && _SNK) */

#if defined(_SRC) || defined(_DRP)
static uint32_t ManageStateAttached_SRC(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  uint32_t _timing = CAD_DEFAULT_TIME;

  uint32_t ccx  = (Ports[PortNum].CCx == CC1) ? (Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC1) :
                  (Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC2);
  uint32_t comp = (Ports[PortNum].CCx == CC1) ? LL_UCPD_SRC_CC1_VRD : LL_UCPD_SRC_CC2_VRD;

  /* Check if CC lines is opened or switch to debug accessory */
  if (comp != ccx)
  {
    /* start counting of CAD_tDebounce */
    if (USBPD_FALSE == _handle->CAD_tDebounce_flag)
    {
      _handle->CAD_tDebounce_flag   = USBPD_TRUE;
      _handle->CAD_tDebounce_start  = HAL_GetTick();
      _timing                       = CAD_TSRCDISCONNECT_THRESHOLD;
    }
    else /* CAD_tDebounce already running */
    {
      /* evaluate CAD_tDebounce */
      uint32_t CAD_tDebounce = HAL_GetTick() - _handle->CAD_tDebounce_start;
      if (CAD_tDebounce > CAD_TSRCDISCONNECT_THRESHOLD)
      {
        HW_SignalDetachment(PortNum);
#ifdef _DRP
        if (USBPD_TRUE == Ports[PortNum].settings->CAD_RoleToggle)
        {
          USBPDM1_AssertRd(PortNum);
        }
#endif /* _DRP */
        _handle->CAD_tDebounce_flag = USBPD_FALSE;
        /* move inside state DETACH to avoid wrong VCONN level*/
        _handle->cstate             = USBPD_CAD_STATE_DETACH_SRC;
        *pEvent                     = USBPD_CAD_EVENT_DETACHED;
        *pCCXX                      = CCNONE;
        _timing                     = 0;
      }
    }
  }
  else
  {
    /* Reset tPDDebounce flag*/
    _handle->CAD_tDebounce_flag   = USBPD_FALSE;
    _timing = CAD_INFINITE_TIME;
  }

  return _timing;
}
#endif /* _SRC || _DRP */

#if defined(_SNK) || defined(_DRP)
static uint32_t ManageStateAttachedWait_SNK(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  uint32_t _timing = CAD_DEFAULT_TIME;

  uint32_t CAD_tDebounce = HAL_GetTick() - _handle->CAD_tDebounce_start;
  CAD_Check_HW_SNK(PortNum);
  if (_handle->CurrentHWcondition == HW_Attachment)
  {
    if (CAD_tDebounce > CAD_TCCDEBOUCE_THRESHOLD)
    {
      if (USBPD_TRUE == USBPD_PWR_IF_GetVBUSStatus(PortNum, USBPD_PWR_VSAFE5V)) /* Check if Vbus is on */
      {
        HW_SignalAttachement(PortNum, _handle->cc);
        _handle->cstate = USBPD_CAD_STATE_ATTACHED;
        *pEvent = USBPD_CAD_EVENT_ATTACHED;
        *pCCXX = _handle->cc;
      }
    }
    _handle->CAD_tDebounce_flag = USBPD_FALSE;
  }
  else
  {
    /* start counting of CAD_tDebounce */
    if (USBPD_FALSE == _handle->CAD_tDebounce_flag)
    {
      _handle->CAD_tDebounce_start = HAL_GetTick();
      _handle->CAD_tDebounce_flag = USBPD_TRUE;
      _timing = CAD_TPDDEBOUCE_THRESHOLD;
    }
    else /* CAD_tDebounce already running */
    {
      /* evaluate CAD_tDebounce */
      if ((HAL_GetTick() - _handle->CAD_tDebounce_start > CAD_TPDDEBOUCE_THRESHOLD))
      {
        _handle->CAD_tDebounce_flag = USBPD_FALSE;
        _handle->cstate             = USBPD_CAD_STATE_DETACHED;
        BSP_USBPD_PWR_VBUSDeInit(PortNum);
#if defined(_ACCESSORY_SNK)
        if (USBPD_TRUE ==  _handle->CAD_Accessory_SNK)
        {
          _handle->CAD_tToggle_start = HAL_GetTick();
        }
#endif /* _ACCESSORY_SNK */
      }
    }
  }
  return _timing;
}

static uint32_t ManageStateAttached_SNK(uint8_t PortNum, USBPD_CAD_EVENT *pEvent, CCxPin_TypeDef *pCCXX)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  uint32_t _timing = CAD_DEFAULT_TIME;

  uint32_t ccx;
  uint32_t comp = (Ports[PortNum].CCx == CC1) ? LL_UCPD_SNK_CC1_VOPEN : LL_UCPD_SNK_CC2_VOPEN;

#if defined(_LOW_POWER) || defined(USBPDM1_VCC_FEATURE_ENABLED)
  /* Enable type C state machine */
  CLEAR_BIT(Ports[PortNum].husbpd->CR, UCPD_CR_CC1TCDIS | UCPD_CR_CC2TCDIS);

  for (int32_t index = 0; index < CAD_DELAY_READ_CC_STATUS; index++)
  {
    __DSB();
  };
#endif /* _LOW_POWER || USBPDM1_VCC_FEATURE_ENABLED */

  ccx  = (Ports[PortNum].CCx == CC1) ? (Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC1)
         : (Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC2);
  if ((USBPD_TRUE == USBPD_PWR_IF_GetVBUSStatus(PortNum,
                                                USBPD_PWR_SNKDETACH)) /* Check if Vbus is below disconnect threshold */
      &&
      (comp == ccx)                                                   /* Confirm that there is no RP */
     )
  {
    HW_SignalDetachment(PortNum);
    /* Restart the toggle time */
    _handle->CurrentHWcondition = HW_Detachment;
    _handle->cstate             = USBPD_CAD_STATE_DETACHED;
#if defined(_ACCESSORY_SNK)
    if (USBPD_TRUE ==  _handle->CAD_Accessory_SNK)
    {
      _handle->CAD_tToggle_start = HAL_GetTick();
    }
#endif /* _ACCESSORY_SNK */
    *pEvent = USBPD_CAD_EVENT_DETACHED;
    *pCCXX = CCNONE;
    _timing = 0;
  }
  else
  {
    _timing = CAD_VBUS_POLLING_TIME;
  }

#if defined(_LOW_POWER) || defined(USBPDM1_VCC_FEATURE_ENABLED)
  /* Disable type C state machine */
  SET_BIT(Ports[PortNum].husbpd->CR, UCPD_CR_CC1TCDIS | UCPD_CR_CC2TCDIS);
#endif /* _LOW_POWER || USBPDM1_VCC_FEATURE_ENABLED */

  return _timing;
}
#endif /* _SNK || _DRP */

#if defined(TCPP0203_SUPPORT)
void CAD_HW_IF_VBUSDetectCallback(uint32_t PortNum,
                                  USBPD_PWR_VBUSConnectionStatusTypeDef VBUSConnectionStatus)
{
  if (VBUSConnectionStatus == VBUS_CONNECTED)
  {
#if defined(_TRACE)
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0,
                    (uint8_t *)"-- USBPD_PWR_VBUSDetectCallback : VBUS_CONNECTED --", 51);
#endif /* _TRACE */
  }
  else
  {
#if defined(_TRACE)
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0,
                    (uint8_t *)"-- USBPD_PWR_VBUSDetectCallback : VBUS_NOT_CONNECTED --", 55);
#endif /* _TRACE */

    /* VBUS_NOT_CONNECTED indications could be caused by false OCP/OVP errors detected at BSP level.
       If reported here, it is assumed that it might be possible to recover from error.
       If error could not be recovered, or is assumed to be related to a true safety issue, it will not be notified
       by BSP */
#if defined(USBPDCORE_LIB_NO_PD)
    CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];

    /* VBUS_NOT_CONNECTED indication management in case of NO_PD configuration */
    if (Ports[PortNum].params->PE_PowerRole == USBPD_PORTPOWERROLE_SRC)
    {
      /* Current role is SRC when VBUS_NOT_CONNECTED signal is received */
      HW_SignalDetachment(PortNum);
      _handle->CAD_tDebounce_flag = USBPD_FALSE;
      _handle->cstate             = USBPD_CAD_STATE_DETACH_SRC;
    }
    else
    {
      /* Current role is SNK when VBUS_NOT_CONNECTED signal is received */
      HW_SignalDetachment(PortNum);
      _handle->CurrentHWcondition = HW_Detachment;
      _handle->cstate             = USBPD_CAD_STATE_DETACHED;
    }
#else
    /* VBUS_NOT_CONNECTED indication management : Error has to be handled through a Detach/Attach procedure.
       Handled using ErrorRecovery mechanism */
    CAD_Enter_ErrorRecovery(PortNum);
#endif /* USBPDCORE_LIB_NO_PD */
  }
}
#endif /* TCPP0203_SUPPORT */

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

