/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usbpd_dpm_core.c
  * @author  MCD Application Team
  * @brief   USBPD dpm core file
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
/* USER CODE END Header */

#define __USBPD_DPM_CORE_C

/* Includes ------------------------------------------------------------------*/
#include "usbpd_core.h"
#include "usbpd_trace.h"
#include "usbpd_dpm_core.h"
#include "usbpd_dpm_conf.h"

#if defined(_LOW_POWER)
#include "usbpd_lowpower.h"
#endif /* _LOW_POWER */

#if defined(USBPD_TCPM_MODULE_ENABLED)
#if defined(USBPD_STUSB1605)
#include "p-nucleo-usb002.h"
#else
#include "usbpd_tcpci.h"
#endif /*USBPD_STUSB1605*/
#endif /* USBPD_TCPM_MODULE_ENABLED */

#if defined(USBPD_TCPM_MODULE_ENABLED)
#include "usbpd_timersserver.h"
#endif /* USBPD_TCPM_MODULE_ENABLED */

/* OS management */
#include "usbpd_os_port.h"

#if defined(_FWUPDATE_RESPONDER)
#include "usbpd_pdfu_responder.h"
#endif /* _FWUPDATE_RESPONDER */

/* Private definition -------------------------------------------------------*/
/* function import prototypes -----------------------------------------------*/
/* Generic STM32 prototypes */
extern uint32_t HAL_GetTick(void);

/* Private function prototypes -----------------------------------------------*/
#if defined(_RTOS) || defined(USBPD_THREADX)
DEF_TASK_FUNCTION(USBPD_TaskUser);
DEF_TASK_FUNCTION(USBPD_CAD_Task);
DEF_TASK_FUNCTION(USBPD_PE_CableTask);

#if !defined(USBPDCORE_LIB_NO_PD)
DEF_TASK_FUNCTION(USBPD_PE_Task);
#endif /* !USBPDCORE_LIB_NO_PD */

#if defined(USBPD_TCPM_MODULE_ENABLED)
DEF_TASK_FUNCTION(USBPD_ALERT_Task);
#endif /* USBPD_TCPM_MODULE_ENABLED */
#else
void USBPD_CAD_Task(void);
void USBPD_TaskUser(void);
#endif /* _RTOS || USBPD_THREADX */


#if defined(USE_STM32_UTILITY_OS)
void TimerCADfunction(void *);
#endif /* USE_STM32_UTILITY_OS */

#if defined(USE_STM32_UTILITY_OS)
void USBPD_PE_Task_P0(void);
void USBPD_PE_Task_P1(void);
void TimerPE0function(void *pArg);
void TimerPE1function(void *pArg);
#endif /* USE_STM32_UTILITY_OS */

/* Private typedef -----------------------------------------------------------*/
#if defined(_RTOS)
#if (osCMSIS < 0x20000U)
#define DPM_STACK_SIZE_ADDON_FOR_CMSIS              1
#else
#define DPM_STACK_SIZE_ADDON_FOR_CMSIS              4
#endif /* osCMSIS < 0x20000U */

#define OS_PE_PRIORITY                    osPriorityAboveNormal
#if defined(_VDM)
#define OS_PE_STACK_SIZE                  (350 * DPM_STACK_SIZE_ADDON_FOR_CMSIS)
#elif defined(__AUTHENTICATION__)
#define OS_PE_STACK_SIZE                  (350 * DPM_STACK_SIZE_ADDON_FOR_CMSIS)
#else
#define OS_PE_STACK_SIZE                  (200 * DPM_STACK_SIZE_ADDON_FOR_CMSIS)
#endif /*_VDM*/

#if defined(USBPD_TCPM_MODULE_ENABLED)
#define OS_ALERT_PRIORITY                 osPriorityRealtime
#define OS_ALERT_STACK_SIZE               (240 * DPM_STACK_SIZE_ADDON_FOR_CMSIS)
#endif /* USBPD_TCPM_MODULE_ENABLED */
#define OS_CAD_PRIORITY                   osPriorityRealtime
#define OS_CAD_STACK_SIZE                 (300 * DPM_STACK_SIZE_ADDON_FOR_CMSIS)

#if defined(USBPD_TCPM_MODULE_ENABLED)
OS_DEFINE_TASK(ALERT, USBPD_ALERT_Task, OS_ALERT_PRIORITY, OS_ALERT_STACK_SIZE, NULL);
#endif /* USBPD_TCPM_MODULE_ENABLED */

#elif defined(USBPD_THREADX)

#define OS_PE_PRIORITY                    1
#define OS_PE_STACK_SIZE                  1024

#if defined(USBPD_TCPM_MODULE_ENABLED)
#define OS_ALERT_PRIORITY                 1
#define OS_ALERT_STACK_SIZE               1024
#endif /* USBPD_TCPM_MODULE_ENABLED */

#define OS_CAD_PRIORITY                   1
#define OS_CAD_STACK_SIZE                 1024

#else /* !_RTOS */
#if defined(USE_STM32_UTILITY_OS)
UTIL_TIMER_Object_t TimerCAD;
#if !defined(USBPDCORE_LIB_NO_PD)
UTIL_TIMER_Object_t TimerPE0, TimerPE1;
#endif /* !USBPDCORE_LIB_NO_PD */
#endif /* USE_STM32_UTILITY_OS */
#endif /* _RTOS */

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define CHECK_PE_FUNCTION_CALL(_function_)  do{                                     \
                                                _retr = _function_;                  \
                                               if(USBPD_OK != _retr) {goto error;}   \
                                              } while(0);

#define CHECK_CAD_FUNCTION_CALL(_function_) if(USBPD_CAD_OK != _function_)      \
  {                                   \
    _retr = USBPD_ERROR;              \
    goto error;                       \
  }

#if defined(_DEBUG_TRACE)
#define DPM_CORE_DEBUG_TRACE(_PORTNUM_, __MESSAGE__)  \
  USBPD_TRACE_Add(USBPD_TRACE_DEBUG, _PORTNUM_, 0u, (uint8_t *)(__MESSAGE__), sizeof(__MESSAGE__) - 1u);
#else
#define DPM_CORE_DEBUG_TRACE(_PORTNUM_, __MESSAGE__)
#endif /* _DEBUG_TRACE */

/* Private variables ---------------------------------------------------------*/
#if defined(USBPD_THREADX)
static TX_BYTE_POOL *USBPD_memory_pool;
#endif /* USBPD_TREADX */
#if defined(_RTOS) || defined(USBPD_THREADX)
#if !defined(USBPDCORE_LIB_NO_PD)
static OS_TASK_ID DPM_PEThreadId_Table[USBPD_PORT_COUNT];
#if defined(USBPD_TCPM_MODULE_ENABLED)
OS_QUEUE_ID  AlarmMsgBox;
OS_TASK_ID ThreadAlert;
static OS_TASK_ID ALAThread;
#else
static OS_QUEUE_ID CADQueueId;
static OS_TASK_ID CADThread;
#endif /* USBPD_TCPM_MODULE_ENABLED */
static OS_QUEUE_ID PEQueueId[USBPD_PORT_COUNT];
#else
static OS_QUEUE_ID CADQueueId;
static OS_TASK_ID CADThread;
#endif /* USBPDCORE_LIB_NO_PD */
#else
#if !defined(USE_STM32_UTILITY_OS)
#if defined(USBPD_TCPM_MODULE_ENABLED)
#define OFFSET_CAD 0U
#else
#define OFFSET_CAD 1U
#endif /* USBPD_TCPM_MODULE_ENABLED */
static uint32_t DPM_Sleep_time[USBPD_PORT_COUNT + OFFSET_CAD];
static uint32_t DPM_Sleep_start[USBPD_PORT_COUNT + OFFSET_CAD];
#endif /* !USE_STM32_UTILITY_OS */
#endif /* _RTOS || USBPD_THREADX */

USBPD_ParamsTypeDef   DPM_Params[USBPD_PORT_COUNT];
/* Private function prototypes -----------------------------------------------*/
#if !defined(USBPDCORE_LIB_NO_PD)
static void USBPD_PE_TaskWakeUp(uint8_t PortNum);
static void DPM_StartPETask(uint8_t PortNum);
#endif /* USBPDCORE_LIB_NO_PD */

void USBPD_DPM_CADCallback(uint8_t PortNum, USBPD_CAD_EVENT State, CCxPin_TypeDef Cc);

#if !defined(USBPD_TCPM_MODULE_ENABLED)
static void USBPD_DPM_CADTaskWakeUp(void);
#endif /* !USBPD_TCPM_MODULE_ENABLED */

/**
  * @brief  Initialize the core stack (port power role, PWR_IF, CAD and PE Init procedures)
  * @retval USBPD status
  */
#if defined(USBPDCORE_LIB_NO_PD)
USBPD_StatusTypeDef USBPD_DPM_InitCore(void)
{
  /* variable to get dynamique memory allocated by usbpd stack */
  uint32_t stack_dynamemsize;
  USBPD_StatusTypeDef _retr = USBPD_OK;

  /* Check the lib selected */
  if (USBPD_TRUE != USBPD_PE_CheckLIB(LIB_ID))
  {
    _retr = USBPD_ERROR;
    goto error;
  }

  /* to get how much memory are dynamically allocated by the stack
     the memory return is corresponding to 2 ports so if the application
     managed only one port divide the value return by 2                   */
  stack_dynamemsize = USBPD_PE_GetMemoryConsumption();

  /* done to avoid warning */
  (void)stack_dynamemsize;

#if defined(_TRACE) || defined(_GUI_INTERFACE)
  /* Initialise the TRACE */
  USBPD_TRACE_Init();
#endif /* _TRACE || _GUI_INTERFACE */

  DPM_Params[USBPD_PORT_0].PE_PowerRole     = DPM_Settings[USBPD_PORT_0].PE_DefaultRole;

  {
    static const USBPD_CAD_Callbacks CAD_cbs = { USBPD_DPM_CADCallback, USBPD_DPM_CADTaskWakeUp };
    /* Init CAD */
    CHECK_CAD_FUNCTION_CALL(USBPD_CAD_Init(USBPD_PORT_0, &CAD_cbs, (USBPD_SettingsTypeDef *)&DPM_Settings[USBPD_PORT_0],
                                           &DPM_Params[USBPD_PORT_0]));

    /* Enable CAD on Port 0 */
    USBPD_CAD_PortEnable(USBPD_PORT_0, USBPD_CAD_ENABLE);
  }

error :
  return _retr;
}
#else
USBPD_StatusTypeDef USBPD_DPM_InitCore(void)
{
  /* variable to get dynamique memory allocated by usbpd stack */
  uint32_t stack_dynamemsize;
  USBPD_StatusTypeDef _retr = USBPD_OK;

  /* CAD callback definition */
  static const USBPD_PE_Callbacks dpmCallbacks =
  {
#if defined(_SRC) || defined(_DRP)
    USBPD_DPM_SetupNewPower,
#else
    NULL,
#endif /*_SRC || DRP */
    USBPD_DPM_HardReset,
#if defined(_DRP)
    USBPD_DPM_EvaluatePowerRoleSwap,
#else
    NULL,
#endif /* DRP */
    USBPD_DPM_Notification,
#ifdef USBPD_REV30_SUPPORT
    USBPD_DPM_ExtendedMessageReceived,
#else
    NULL,
#endif /* USBPD_REV30_SUPPORT */
    USBPD_DPM_GetDataInfo,
    USBPD_DPM_SetDataInfo,
#if defined(_SRC) || defined(_DRP)
    USBPD_DPM_EvaluateRequest,
#else
    NULL,
#endif /*_SRC || DRP */
#if defined(_SNK) || defined(_DRP)
    USBPD_DPM_SNK_EvaluateCapabilities,
#else
    NULL,
#endif /*_SNK || DRP */
#if defined(_DRP)
    USBPD_DPM_PowerRoleSwap,
#else
    NULL,
#endif /*  _DRP  */
    USBPD_PE_TaskWakeUp,
#if defined(_VCONN_SUPPORT)
    USBPD_DPM_EvaluateVconnSwap,
    USBPD_DPM_PE_VconnPwr,
#else
    NULL,
    NULL,
#endif /* _VCONN_SUPPORT */
#if defined(_ERRORRECOVERY_NOTSUPPORTED)
    NULL,
#else
    USBPD_DPM_EnterErrorRecovery,
#endif /* _ERRORRECOVERY_NOTSUPPORTED */
    USBPD_DPM_EvaluateDataRoleSwap,
    USBPD_DPM_IsPowerReady
  };

  static const USBPD_CAD_Callbacks CAD_cbs =
  {
    USBPD_DPM_CADCallback,
#if defined(USBPD_TCPM_MODULE_ENABLED)
    NULL
#else  /* USBPD_TCPM_MODULE_ENABLED */
    USBPD_DPM_CADTaskWakeUp
#endif /* USBPD_TCPM_MODULE_ENABLED */
  };

#if !defined(_SIMULATOR)
  /* Check the lib selected */
  if (USBPD_TRUE != USBPD_PE_CheckLIB(LIB_ID))
  {
    _retr = USBPD_ERROR;
    goto error;
  }
#endif /* _SIMULATOR */

  /* to get how much memory are dynamically allocated by the stack
     the memory return is corresponding to 2 ports so if the application
     managed only one port divide the value return by 2                   */
  stack_dynamemsize = USBPD_PE_GetMemoryConsumption();

  /* done to avoid warning */
  (void)stack_dynamemsize;

#if defined(_TRACE) || defined(_GUI_INTERFACE)
  /* Initialise the TRACE */
  USBPD_TRACE_Init();
#endif /* _TRACE || _GUI_INTERFACE */

#if defined(USBPD_TCPM_MODULE_ENABLED)
#ifndef _RTOS
  USBPD_TIM_Init();
#endif /* !_RTOS*/
#if !defined(USBPD_STUSB1605)
  USBPD_TCPCI_Init();
#endif /* !USBPD_STUSB1605 */
#endif /* USBPD_TCPM_MODULE_ENABLED */

  for (uint8_t _port_index = 0; _port_index < USBPD_PORT_COUNT; ++_port_index)
  {
    /* Variable to be sure that DPM is correctly initialized */
    DPM_Params[_port_index].DPM_Initialized = USBPD_FALSE;

    /* check the stack settings */
    DPM_Params[_port_index].PE_SpecRevision  = DPM_Settings[_port_index].PE_SpecRevision;
    DPM_Params[_port_index].PE_PowerRole     = DPM_Settings[_port_index].PE_DefaultRole;
    DPM_Params[_port_index].PE_SwapOngoing   = USBPD_FALSE;
    DPM_Params[_port_index].ActiveCCIs       = CCNONE;
    DPM_Params[_port_index].VconnCCIs        = CCNONE;
    DPM_Params[_port_index].VconnStatus      = USBPD_FALSE;

#if defined(USBPD_TCPM_MODULE_ENABLED)
    {
      TCPC_DrvTypeDef *tcpc_driver;
      USBPD_TCPCI_GetDevicesDrivers(_port_index, &tcpc_driver);
      USBPD_TCPM_HWInit(_port_index, DPM_Settings[_port_index].CAD_RoleToggle, &DPM_Params[_port_index],
                        (USBPD_CAD_Callbacks *)&CAD_cbs, tcpc_driver);
    }
#else
    /* CAD SET UP : Port 0 */
    CHECK_CAD_FUNCTION_CALL(USBPD_CAD_Init(_port_index,
                                           &CAD_cbs,
                                           &DPM_Settings[_port_index],
                                           &DPM_Params[_port_index]));
#endif /* USBPD_TCPM_MODULE_ENABLED */

    /* PE SET UP : Port 0 */
    CHECK_PE_FUNCTION_CALL(USBPD_PE_Init(_port_index, (USBPD_SettingsTypeDef *)&DPM_Settings[_port_index],
                                         &DPM_Params[_port_index], &dpmCallbacks));

    /* DPM is correctly initialized */
    DPM_Params[_port_index].DPM_Initialized = USBPD_TRUE;

#if defined(USBPD_TCPM_MODULE_ENABLED)
#else
    /* Enable CAD on Port 0 */
    USBPD_CAD_PortEnable(_port_index, USBPD_CAD_ENABLE);
#endif /* USBPD_TCPM_MODULE_ENABLED */
  }

#if !defined(_RTOS)
#if defined(USE_STM32_UTILITY_OS)
  /* initialise timer server */
  UTIL_TIMER_Init();

  /* initialize the sequencer */
  UTIL_SEQ_Init();
#endif /* USE_STM32_UTILITY_OS */
#endif /* !_RTOS */

#ifdef _LOW_POWER
  USBPD_LOWPOWER_Init();
#endif /* _LOW_POWER */

error :
  return _retr;
}
#endif /* USBPDCORE_LIB_NO_PD */
/**
  * @brief  Initialize the OS parts (task, queue,... )
  * @retval USBPD status
  */
#if defined(USBPD_THREADX)
uint32_t USBPD_DPM_InitOS(VOID *MemoryPtr)
#else
USBPD_StatusTypeDef USBPD_DPM_InitOS(void)
#endif
{
  OS_INIT();
#if defined(_RTOS) || defined(USBPD_THREADX)
#if defined(USBPD_TCPM_MODULE_ENABLED)
  {
    OS_CREATE_QUEUE(AlarmMsgBox, "QAlarme", TCPM_ALARMBOX_MESSAGES_MAX, OS_ELEMENT_SIZE);
    OS_CREATE_TASK(ALAThread, ALERT, USBPD_ALERT_Task, OS_ALERT_PRIORITY, OS_ALERT_STACK_SIZE, NULL);
  }
#else
  {
    OS_CREATE_QUEUE(CADQueueId, "QCAD", USBPD_PORT_COUNT, OS_ELEMENT_SIZE);
    OS_DEFINE_TASK(CAD, USBPD_CAD_Task, OS_CAD_PRIORITY, OS_CAD_STACK_SIZE, NULL);
    OS_CREATE_TASK(CADThread, CAD, USBPD_CAD_Task,  OS_CAD_PRIORITY, OS_CAD_STACK_SIZE, NULL);
  }
#endif /* USBPD_TCPM_MODULE_ENABLED */

#if !defined(USBPDCORE_LIB_NO_PD)
  /* Create the queue corresponding to PE task */
  for (uint32_t index = 0; index < USBPD_PORT_COUNT; index++)
  {
    OS_CREATE_QUEUE(PEQueueId[index], "QPE", 1, OS_ELEMENT_SIZE);

    if (index == USBPD_PORT_0)
    {
      /* Tasks definition */
      OS_DEFINE_TASK(PE_0, USBPD_PE_Task, OS_PE_PRIORITY,  OS_PE_STACK_SIZE,  USBPD_PORT_0);
      OS_CREATE_TASK(DPM_PEThreadId_Table[USBPD_PORT_0], PE_0, USBPD_PE_Task,
                     OS_PE_PRIORITY, OS_PE_STACK_SIZE, index);
    }
#if USBPD_PORT_COUNT > 1
    if (index == USBPD_PORT_1)
    {
      /* Tasks definition */
      OS_DEFINE_TASK(PE_1, USBPD_PE_Task, OS_PE_PRIORITY,  OS_PE_STACK_SIZE,  USBPD_PORT_1);
      OS_CREATE_TASK(DPM_PEThreadId_Table[USBPD_PORT_1], PE_1, USBPD_PE_Task,
                     OS_PE_PRIORITY, OS_PE_STACK_SIZE, index);
    }
#endif /* USBPD_PORT_COUNT > 1*/
  }
#endif /* !USBPDCORE_LIB_NO_PD */
#endif /* _RTOS || USBPD_THREADX */

#if defined(USBPD_TCPM_MODULE_ENABLED)
  USBPD_TCPI_AlertInit();
#endif /* USBPD_TCPM_MODULE_ENABLED */

#if defined(_RTOS) || defined(USBPD_THREADX)
error:
#endif
  return _retr;
}

/**
  * @brief  Initialize the OS parts (port power role, PWR_IF, CAD and PE Init procedures)
  * @retval None
  */
#ifdef _RTOS
void USBPD_DPM_Run(void)
{
  OS_KERNEL_START();
}
#elif defined(USBPD_THREADX)
/* Nothing to do kernel run is managed on CubeMX side */
/* this is obsolete in threadX context void USBPD_DPM_Run(void) */
#else /* NRTOS */
#if defined(USE_STM32_UTILITY_OS)
/**
  * @brief  Task for CAD processing
  * @retval None
  */
void USBPD_CAD_Task(void)
{
  UTIL_TIMER_Stop(&TimerCAD);
  uint32_t _timing = USBPD_CAD_Process();
  UTIL_TIMER_SetPeriod(&TimerCAD, _timing);
  UTIL_TIMER_Start(&TimerCAD);
}

/**
  * @brief  timer function to wakeup CAD Task
  * @param pArg Pointer on an argument
  * @retval None
  */
void TimerCADfunction(void *pArg)
{
  UTIL_SEQ_SetTask(TASK_CAD, 0);
}

#if !defined(USBPDCORE_LIB_NO_PD)
/**
  * @brief  timer function to wakeup PE_0 Task
  * @param pArg Pointer on an argument
  * @retval None
  */
void TimerPE0function(void *pArg)
{
  UTIL_SEQ_SetTask(TASK_PE_0, 0);
}

/**
  * @brief  timer function to wakeup PE_1 Task
  * @param pArg Pointer on an argument
  * @retval None
  */
void TimerPE1function(void *pArg)
{
  UTIL_SEQ_SetTask(TASK_PE_1, 0);
}

/**
  * @brief  Task for PE_0 processing
  * @retval None
  */
void USBPD_PE_Task_P0(void)
{
  UTIL_TIMER_Stop(&TimerPE0);
  uint32_t _timing =
#ifdef _DRP
    USBPD_PE_StateMachine_DRP(USBPD_PORT_0);
#elif _SRC
    USBPD_PE_StateMachine_SRC(USBPD_PORT_0);
#elif _SNK
    USBPD_PE_StateMachine_SNK(USBPD_PORT_0);
#endif /* _DRP */
  if (_timing != 0xFFFFFFFF)
  {
    UTIL_TIMER_SetPeriod(&TimerPE0, _timing);
    UTIL_TIMER_Start(&TimerPE0);
  }
}

/**
  * @brief  Task for PE_1 processing
  * @retval None
  */
void USBPD_PE_Task_P1(void)
{
  UTIL_TIMER_Stop(&TimerPE1);
  uint32_t _timing =
#ifdef _DRP
    USBPD_PE_StateMachine_DRP(USBPD_PORT_1);
#elif _SRC
    USBPD_PE_StateMachine_SRC(USBPD_PORT_1);
#elif _SNK
    USBPD_PE_StateMachine_SNK(USBPD_PORT_1);
#endif /* _DRP */
  if (_timing != 0xFFFFFFFF)
  {
    UTIL_TIMER_SetPeriod(&TimerPE1, _timing);
    UTIL_TIMER_Start(&TimerPE1);
  }
}
#endif /* !USBPDCORE_LIB_NO_PD */

/**
  * @brief  Task for DPM_USER processing
  * @retval None
  */
void USBPD_TaskUser(void)
{
  USBPD_DPM_UserExecute(NULL);
}
#endif /* USE_STM32_UTILITY_OS */

void USBPD_DPM_Run(void)
{
#if defined(USE_STM32_UTILITY_OS)
#if !defined(USBPD_TCPM_MODULE_ENABLED)
  UTIL_SEQ_RegTask(TASK_CAD,  0, USBPD_CAD_Task);
  UTIL_SEQ_SetTask(TASK_CAD,  0);
  UTIL_TIMER_Create(&TimerCAD, 10, UTIL_TIMER_ONESHOT, TimerCADfunction, NULL);
#endif /* !USBPD_TCPM_MODULE_ENABLED */

#if !defined(USBPDCORE_LIB_NO_PD)
  UTIL_SEQ_RegTask(TASK_PE_0, 0,  USBPD_PE_Task_P0);
  UTIL_SEQ_PauseTask(TASK_PE_0);
  UTIL_TIMER_Create(&TimerPE0, 10, UTIL_TIMER_ONESHOT, TimerPE0function, NULL);
#if USBPD_PORT_COUNT == 2
  UTIL_SEQ_RegTask(TASK_PE_1, 0,  USBPD_PE_Task_P1);
  UTIL_SEQ_PauseTask(TASK_PE_1);
  UTIL_TIMER_Create(&TimerPE1, 10, UTIL_TIMER_ONESHOT, TimerPE1function, NULL);
#endif /* USBPD_PORT_COUNT == 2 */
#endif /* !USBPDCORE_LIB_NO_PD */

  UTIL_SEQ_RegTask(TASK_USER, 0, USBPD_TaskUser);
  UTIL_SEQ_SetTask(TASK_USER,  0);

  do
  {
    UTIL_SEQ_Run(~0);
  } while (1u == 1u);
#else /* !USE_STM32_UTILITY_OS */
  do
  {
#if !defined(USBPD_TCPM_MODULE_ENABLED)
    if ((HAL_GetTick() - DPM_Sleep_start[USBPD_PORT_COUNT]) >= DPM_Sleep_time[USBPD_PORT_COUNT])
    {
      DPM_Sleep_time[USBPD_PORT_COUNT] = USBPD_CAD_Process();
      DPM_Sleep_start[USBPD_PORT_COUNT] = HAL_GetTick();
    }
#endif /* USBPD_TCPM_MODULE_ENABLED */

#if !defined(USBPDCORE_LIB_NO_PD)
    uint32_t port = 0;

    for (port = 0; port < USBPD_PORT_COUNT; port++)
    {
      if ((HAL_GetTick() - DPM_Sleep_start[port]) >= DPM_Sleep_time[port])
      {
        DPM_Sleep_time[port] =
#ifdef _DRP
          USBPD_PE_StateMachine_DRP(port);
#elif _SRC
          USBPD_PE_StateMachine_SRC(port);
#elif _SNK
          USBPD_PE_StateMachine_SNK(port);
#endif /* _DRP */
        DPM_Sleep_start[port] = HAL_GetTick();
      }
    }
#endif /* USBPDCORE_LIB_NO_PD */

    USBPD_DPM_UserExecute(NULL);

#if defined(_SIMULATOR)
    return;
#endif /* _SIMULATOR */
  } while (1u == 1u);
#endif /* USE_STM32_UTILITY_OS */
}

#endif /* _RTOS */


#if !defined(USBPDCORE_LIB_NO_PD)
/**
  * @brief  Initialize DPM (port power role, PWR_IF, CAD and PE Init procedures)
  * @retval USBPD status
  */
void USBPD_DPM_TimerCounter(void)
{
  /* Call PE/PRL timers functions only if DPM is initialized */
  if (USBPD_TRUE == DPM_Params[USBPD_PORT_0].DPM_Initialized)
  {
    USBPD_DPM_UserTimerCounter(USBPD_PORT_0);
    USBPD_PE_TimerCounter(USBPD_PORT_0);
    USBPD_PRL_TimerCounter(USBPD_PORT_0);
#if defined(_FWUPDATE_RESPONDER)
    USBPD_PDFU_TimerCounter(USBPD_PORT_0);
#endif /* _FWUPDATE_RESPONDER */
  }
#if USBPD_PORT_COUNT==2
  if (USBPD_TRUE == DPM_Params[USBPD_PORT_1].DPM_Initialized)
  {
    USBPD_DPM_UserTimerCounter(USBPD_PORT_1);
    USBPD_PE_TimerCounter(USBPD_PORT_1);
    USBPD_PRL_TimerCounter(USBPD_PORT_1);
#if defined(_FWUPDATE_RESPONDER)
    USBPD_PDFU_TimerCounter(USBPD_PORT_1);
#endif /* _FWUPDATE_RESPONDER */
  }
#endif /* USBPD_PORT_COUNT == 2 */

#if defined(_RTOS)
#if (osCMSIS >= 0x20000U)
  /* SysTick Handler now fully handled on CMSIS OS V2 side */
#else
  /* check to avoid count before OSKernel Start */
  if (uxTaskGetNumberOfTasks() != 0u)
  {
    osSystickHandler();
  }
#endif /* osCMSIS >= 0x20000U */
#endif /* _RTOS */
}

/**
  * @brief  WakeUp PE task
  * @param  PortNum port number
  * @retval None
  */
static void USBPD_PE_TaskWakeUp(uint8_t PortNum)
{
#if defined(_RTOS) || defined(USBPD_THREADX)
  OS_PUT_MESSAGE_QUEUE(PEQueueId[PortNum], 0xFFFFU, 0U);
#else
#if defined(USE_STM32_UTILITY_OS)
  UTIL_SEQ_SetTask(PortNum == 0 ? TASK_PE_0 : TASK_PE_1, 0);
#else
  DPM_Sleep_time[PortNum] = 0;
#endif /* USE_STM32_UTILITY_OS */
#endif /* _RTOS || USBPD_THREADX */
}

#endif /* USBPDCORE_LIB_NO_PD */

#if !defined(USBPD_TCPM_MODULE_ENABLED)
/**
  * @brief  WakeUp CAD task
  * @retval None
  */
static void USBPD_DPM_CADTaskWakeUp(void)
{
#if defined(_RTOS) || defined(USBPD_THREADX)
  OS_PUT_MESSAGE_QUEUE(CADQueueId, 0xFFFF, 0);
#else
#if defined(USE_STM32_UTILITY_OS)
  UTIL_SEQ_SetTask(TASK_CAD, 0);
#else
  DPM_Sleep_time[USBPD_PORT_COUNT] = 0;
#endif /* USE_STM32_UTILITY_OS */
#endif /* _RTOS || USBPD_THREADX */
}
#endif /* !USBPD_TCPM_MODULE_ENABLED */

#if defined(_RTOS) || defined(USBPD_THREADX)
#if !defined(USBPDCORE_LIB_NO_PD)
/**
  * @brief  Main task for PE layer
  * @param  argument Not used
  * @retval None
  */
DEF_TASK_FUNCTION(USBPD_PE_Task)
{
  uint8_t _port = (uint32_t)argument;
  uint32_t _timing;

#ifdef _LOW_POWER
  UTIL_LPM_SetOffMode(0 == _port ? LPM_PE_0 : LPM_PE_1, UTIL_LPM_DISABLE);
#endif /* _LOW_POWER */

  for (;;)
  {
    if (DPM_Params[_port].PE_IsConnected == USBPD_FALSE)
    {
      /* if the port is no more connected, suspend the PE thread */
      OS_TASK_SUSPEND(OS_TASK_GETID());
    }

#if defined(_DRP) || (defined(_SRC) && defined(_SNK))
    _timing = USBPD_PE_StateMachine_DRP(_port);
#elif defined(_SRC)
    _timing = USBPD_PE_StateMachine_SRC(_port);
#elif defined(_SNK)
    _timing = USBPD_PE_StateMachine_SNK(_port);
#else
#error "a type of state machine should be supported"
#endif /* _DRP || ( _SRC && _SNK) */

    OS_GETMESSAGE_QUEUE(PEQueueId[_port], _timing);

#if defined(USBPD_TCPM_MODULE_ENABLED)
    /* During SRC tests, VBUS is disabled by the FUSB but the detection is not
       well done */
    if ((DPM_Params[_port].PE_SwapOngoing == 0) && (USBPD_ERROR == USBPD_TCPM_VBUS_IsVsafe5V(_port)))
    {
      (void)osMessagePut(AlarmMsgBox, (_port << 8 | 2), osWaitForever);
    }
#endif /* USBPD_TCPM_MODULE_ENABLED */
  }
}
#endif /* !USBPDCORE_LIB_NO_PD */
#if defined(USBPD_TCPM_MODULE_ENABLED)
/**
  * @brief  Main task for ALERT layer
  * @param  argument: Not used
  * @retval None
  */
DEF_TASK_FUNCTION(USBPD_ALERT_Task)
{
  osMessageQId  queue = *(osMessageQId *)argument;
  uint8_t port;
  for (;;)
  {
#if (osCMSIS < 0x20000U)
    osEvent event = osMessageGet(queue, osWaitForever);
    port = (event.value.v >> 8);
#else
    (void)osMessageQueueGet(queue, &event, NULL, osWaitForever);
    port = (event >> 8);
#endif /* osCMSIS < 0x20000U */
#if defined(_TRACE)
    USBPD_TRACE_Add(USBPD_TRACE_TCPM, port, TCPM_TRACE_CORE_ALERT, (uint8_t *)(&(event.value.v)), 4);
#endif /* _TRACE */
#if (osCMSIS < 0x20000U)
    USBPD_TCPM_alert(event.value.v);
#else
    USBPD_TCPM_alert(event);
#endif /* osCMSIS < 0x20000U */
    HAL_NVIC_EnableIRQ(ALERT_GPIO_IRQHANDLER(port));
  }
}
#else /* !USBPD_TCPM_MODULE_ENABLED */
/**
  * @brief  Main task for CAD layer
  * @param  argument Not used
  * @retval None
  */
DEF_TASK_FUNCTION(USBPD_CAD_Task)
{
  uint32_t _timing;
#ifdef _LOW_POWER
  UTIL_LPM_SetOffMode(LPM_CAD, UTIL_LPM_DISABLE);
#endif /* _LOW_POWER */
  for (;;)
  {
    _timing = USBPD_CAD_Process();
    OS_GETMESSAGE_QUEUE(CADQueueId, _timing);
  }
}
#endif /* USBPD_TCPM_MODULE_ENABLED */
#endif /* _RTOS || USBPD_THREADX */

/**
  * @brief  CallBack reporting events on a specified port from CAD layer.
  * @param  PortNum   The handle of the port
  * @param  State     CAD state
  * @param  Cc        The Communication Channel for the USBPD communication
  * @retval None
  */
#if defined(USBPDCORE_LIB_NO_PD)
void USBPD_DPM_CADCallback(uint8_t PortNum, USBPD_CAD_EVENT State, CCxPin_TypeDef Cc)
{
#ifdef _TRACE
  USBPD_TRACE_Add(USBPD_TRACE_CADEVENT, PortNum, (uint8_t)State, NULL, 0);
#endif /* _TRACE */

  switch (State)
  {
#if defined(USBPDCORE_VPD)
    case USPPD_CAD_EVENT_VPD :
#endif /* USBPDCORE_VPD */
    case USBPD_CAD_EVENT_ATTEMC :
    case USBPD_CAD_EVENT_ATTACHED :
    {
      USBPD_DPM_UserCableDetection(PortNum, State);
      USBPD_DPM_Notification(PortNum, USBPD_NOTIFY_USBSTACK_START);
      break;
    }
    case USBPD_CAD_EVENT_DETACHED :
    case USBPD_CAD_EVENT_EMC :
    {
      USBPD_DPM_UserCableDetection(PortNum, State);
      USBPD_DPM_Notification(PortNum, USBPD_NOTIFY_USBSTACK_STOP);
      break;
    }
    default :
      /* nothing to do */
      break;
  }
}
#else
void USBPD_DPM_CADCallback(uint8_t PortNum, USBPD_CAD_EVENT State, CCxPin_TypeDef Cc)
{
#ifdef _TRACE
  USBPD_TRACE_Add(USBPD_TRACE_CADEVENT, PortNum, (uint8_t)State, NULL, 0);
#endif /* _TRACE */
  (void)(Cc);
  switch (State)
  {

    case USBPD_CAD_EVENT_ATTEMC :
    {
#if defined(_VCONN_SUPPORT)
      DPM_Params[PortNum].VconnStatus = USBPD_TRUE;
#endif /* _VCONN_SUPPORT */
      USBPD_DPM_UserCableDetection(PortNum, USBPD_CAD_EVENT_ATTEMC);
      DPM_StartPETask(PortNum);
      break;
    }
    case USBPD_CAD_EVENT_ATTACHED :
      USBPD_DPM_UserCableDetection(PortNum, USBPD_CAD_EVENT_ATTACHED);
      DPM_StartPETask(PortNum);
      break;

    case USBPD_CAD_EVENT_DETACHED :
    case USBPD_CAD_EVENT_EMC :
    {
      /* Terminate PE task */
#if defined(_RTOS) || defined(USBPD_THREADX)
      uint8_t _timeout = 0;
#ifdef _LOW_POWER
      UTIL_LPM_SetStopMode(0 == PortNum ? LPM_PE_0 : LPM_PE_1, UTIL_LPM_ENABLE);
      UTIL_LPM_SetOffMode(0 == PortNum ? LPM_PE_0 : LPM_PE_1, UTIL_LPM_ENABLE);
#endif /* _LOW_POWER */
      /* WakeUp PE task to let him enter suspend mode */
      USBPD_PE_TaskWakeUp(PortNum);
      /* Wait PE Let time to PE to complete the ongoing action */
      while (!OS_TASK_IS_SUSPENDED(DPM_PEThreadId_Table[PortNum]))
      {
        (void)OS_DELAY(1u);
        _timeout++;
        if (_timeout > 30u)
        {
          /* Kill the PE task */
          (void)OS_TASK_KILL(DPM_PEThreadId_Table[PortNum]);
#if defined(_RTOS)
          DPM_PEThreadId_Table[PortNum] = 0;
#elif defined(USBPD_THREADX)
          DPM_PEThreadId_Table[PortNum].tx_thread_id = 0;
#endif /* _RTOS */
          break;
        }
      };
      /* Stop the PE state machine */
      USBPD_PE_StateMachine_Stop(PortNum);
#else
#if defined(USE_STM32_UTILITY_OS)
      UTIL_SEQ_PauseTask(PortNum == 0 ? TASK_PE_0 : TASK_PE_1);
#else
      DPM_Sleep_time[PortNum] = 0xFFFFFFFFU;
#endif /* USE_STM32_UTILITY_OS */
#endif /* _RTOS || USBPD_THREADX */
      DPM_Params[PortNum].PE_SwapOngoing = USBPD_FALSE;
      DPM_Params[PortNum].PE_Power   = USBPD_POWER_NO;
      USBPD_DPM_UserCableDetection(PortNum, State);
#ifdef _VCONN_SUPPORT
      DPM_Params[PortNum].VconnStatus = USBPD_FALSE;
#endif /* _VCONN_SUPPORT */
      break;
    }
    default :
      /* nothing to do */
      break;
  }
}

static void DPM_StartPETask(uint8_t PortNum)
{
  USBPD_PE_StateMachine_Reset(PortNum);
#if defined(_RTOS) || defined(USBPD_THREADX)
#if defined(USBPD_THREADX)
  uint32_t _retr = TX_SUCCESS;
#elif defined(_RTOS)
  USBPD_StatusTypeDef _retr = USBPD_OK; /* Added for comptibility with the macro OS_CreateTask */
#endif /* USBPD_THREADX */
  /* Resume the PE task */
  switch (PortNum)
  {
    case USBPD_PORT_0:
    case USBPD_PORT_1:
    {
#if defined(_RTOS)
      if (DPM_PEThreadId_Table[PortNum] != 0)
#elif defined(USBPD_THREADX)
      if (DPM_PEThreadId_Table[PortNum].tx_thread_id != 0)
#endif /* _RTOS */
      {
        OS_TASK_RESUME(DPM_PEThreadId_Table[PortNum]);
      }
      else
      {
        OS_CREATE_TASK(DPM_PEThreadId_Table[PortNum], PE_0, USBPD_PE_Task, OS_PE_PRIORITY, OS_PE_STACK_SIZE, PortNum);
      }
      break;
    }
    default :
    {
      USBPD_DPM_ErrorHandler();
      break;
    }
  }
error :
   (void)_retr;
#else
#if defined(USE_STM32_UTILITY_OS)
  /* Resume the task */
  UTIL_SEQ_ResumeTask(PortNum == 0 ? TASK_PE_0 : TASK_PE_1);
  /* Enable task execution */
  UTIL_SEQ_SetTask(PortNum == 0 ? TASK_PE_0 : TASK_PE_1, 0);
#else
  DPM_Sleep_time[PortNum] = 0U;
#endif /* USE_STM32_UTILITY_OS */
#endif /* _RTOS || USBPD_THREADX */
}
#endif /* USBPDCORE_LIB_NO_PD */

__WEAK void USBPD_DPM_ErrorHandler(void)
{
  /* This function is called to block application execution
     in case of an unexpected behavior
     another solution could be to reset application */
  while (1u == 1u) {};
}
