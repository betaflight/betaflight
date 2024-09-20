/**
  ******************************************************************************
  * @file    usbpd_os_port.h
  * @author  MCD Application Team
  * @brief   This file contains the core os portability macro definition.
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
#ifndef USBPD_CORE_OSPORT_H_
#define USBPD_CORE_OSPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#if defined(USBPD_THREADX)
#include "tx_api.h"
#else
#if defined(_RTOS)
#include "cmsis_os.h"
#if (osCMSIS >= 0x20000U)
#include "task.h"
#endif /* osCMSIS >= 0x20000U */
#else
#if defined(USE_STM32_UTILITY_OS)
#include "utilities_conf.h"
#endif /* USE_STM32_UTILITY_OS */
#endif /* _RTOS */
#endif /* USBPD_THREADX */

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_CORE_OS
  * @{
  */

/** @addtogroup USBPD_CORE_OS_Macro
  * @{
  */
/* Exported define -----------------------------------------------------------*/
/**
  * @brief macro definition used to define the task function
  */
#if defined(USBPD_THREADX)

#define DEF_TASK_FUNCTION(__FUNCTION__)   void (__FUNCTION__)(ULONG argument)
#else

#if (osCMSIS < 0x20000U)

#define DEF_TASK_FUNCTION(__FUNCTION__)   void (__FUNCTION__)(void const *argument)

#else

#define DEF_TASK_FUNCTION(__FUNCTION__)   void (__FUNCTION__)(void *argument)

#endif /* osCMSIS < 0x20000U */
#endif /* USBPD_THREADX */

/**
  * @brief macro definition used to initialize the OS environment
  */
#if defined(USBPD_THREADX)

#define OS_INIT()                                        \
    USBPD_memory_pool = (TX_BYTE_POOL*)MemoryPtr;        \
    uint32_t _retr = TX_SUCCESS;

#else

#define OS_INIT()   USBPD_StatusTypeDef _retr = USBPD_OK;

#endif /* USBPD_THREADX */

/**
  * @brief macro definition the define a queue type
  */
#if defined(USBPD_THREADX)

#define OS_QUEUE_ID TX_QUEUE

#else

#define OS_QUEUE_ID osMessageQId

#endif /* USBPD_THREADX */

/**
  * @brief macro definition the define a queue type
  */
#if defined(USBPD_THREADX)

#define OS_ELEMENT_SIZE TX_1_ULONG

#else

#define OS_ELEMENT_SIZE sizeof(uint16_t)

#endif /* USBPD_THREADX */

/**
  * @brief macro definition used to define a queue
  */
#if defined(USBPD_THREADX)

#define OS_CREATE_QUEUE(_ID_,_NAME_, _ELT_,_ELTSIZE_)                                                         \
  do{                                                                                                         \
    char *ptr;                                                                                                \
    _retr = tx_byte_allocate(USBPD_memory_pool, (VOID **) &ptr,(_ELT_)*sizeof(ULONG)*(_ELTSIZE_),TX_NO_WAIT); \
    if(_retr != TX_SUCCESS)                                                                                   \
    {                                                                                                         \
      goto error;                                                                                             \
    }                                                                                                         \
    _retr = tx_queue_create(&(_ID_),(_NAME_), (_ELTSIZE_), ptr ,(_ELT_)*sizeof(ULONG)*(_ELTSIZE_));           \
    if(_retr != TX_SUCCESS)                                                                                   \
    {                                                                                                         \
      goto error;                                                                                             \
    }                                                                                                         \
  } while(0);

#else

#if (osCMSIS < 0x20000U)

#define OS_CREATE_QUEUE(_ID_,_NAME_,_ELT_,_ELTSIZE_)        \
  do {                                                      \
       osMessageQDef(queuetmp, (_ELT_), (_ELTSIZE_));       \
       (_ID_) = osMessageCreate(osMessageQ(queuetmp), NULL);\
       if((_ID_) == 0)                                      \
       {                                                    \
         _retr = USBPD_ERROR;                               \
         goto error;                                        \
       }                                                    \
  } while(0)

#else

#define OS_CREATE_QUEUE(_ID_,_NAME_,_ELT_,_ELTSIZE_)         \
  do {                                                       \
       (_ID_) = osMessageQueueNew((_ELT_),(_ELTSIZE_), NULL);\
  }while(0)

#endif /* osCMSIS < 0x20000U */
#endif /* USBPD_THREADX */

/**
  * @brief macro definition used to read a queue message
  */
#if defined(USBPD_THREADX)

#define OS_GETMESSAGE_QUEUE(_ID_, _TIME_)                                      \
  do {                                                                         \
    ULONG value;                                                               \
    tx_queue_receive(&(_ID_), (void*)&value, (_TIME_));                        \
  } while(0)

#else
#if (osCMSIS < 0x20000U)

#define OS_GETMESSAGE_QUEUE(_ID_, _TIME_)  osMessageGet((_ID_),(_TIME_))

#else

#define OS_GETMESSAGE_QUEUE(_ID_, _TIME_)                   \
  do {                                                      \
       uint32_t event;                                      \
       (void)osMessageQueueGet((_ID_),&event,NULL,(_TIME_));\
  } while(0)

#endif /* osCMSIS < 0x20000U */
#endif /* USBPD_THREADX */

/**
  * @brief macro definition used to define put a message inside the queue
  */
#if defined(USBPD_THREADX)

#define OS_PUT_MESSAGE_QUEUE(_ID_,_MSG_,_TIMEOUT_)                               \
  do{                                                                            \
    ULONG _msg = _MSG_;                                                          \
    (void)tx_queue_send(&(_ID_), &_msg,(_TIMEOUT_));                             \
  }while(0)
#else

#if (osCMSIS < 0x20000U)
#define OS_PUT_MESSAGE_QUEUE(_ID_,_MSG_,_TIMEOUT_)   \
  do{                                                \
      (void)osMessagePut((_ID_),(_MSG_),(_TIMEOUT_));\
  }while(0)
#else
#define OS_PUT_MESSAGE_QUEUE(_ID_,_MSG_,_TIMEOUT_)             \
  do {                                                         \
       uint32_t event = (_MSG_);                               \
       (void)osMessageQueuePut((_ID_), &event, 0U,(_TIMEOUT_));\
  } while(0)
#endif /* osCMSIS < 0x20000U */
#endif /* USBPD_THREADX */

/**
  * @brief macro definition used to define a task
  */
#if defined(USBPD_THREADX)

#define OS_DEFINE_TASK(_NAME_,_FUNC_,_PRIORITY_,_STACK_SIZE_, _PARAM_)
#else
#if (osCMSIS < 0x20000U)

#define OS_DEFINE_TASK(_NAME_,_FUNC_,_PRIORITY_,_STACK_SIZE_, _PARAM_)

#else

#define OS_DEFINE_TASK(_NAME_,_FUNC_,_PRIORITY_,_STACK_SIZE_, _PARAM_)
#endif /* osCMSIS < 0x20000U */
#endif /* USBPD_THREADX */

/**
  * @brief macro definition of the TASK id
  */
#if defined(USBPD_THREADX)

#define OS_TASK_ID   TX_THREAD
#else

#define OS_TASK_ID   osThreadId
#endif /* USBPD_THREADX */

/**
  * @brief macro definition used to create a task
  */
#if defined(USBPD_THREADX)

#define OS_CREATE_TASK(_ID_,_NAME_,_FUNC_,_PRIORITY_,_STACK_SIZE_, _PARAM_)                  \
  do {                                                                                       \
    char *ptr;                                                                               \
    _retr = tx_byte_allocate(USBPD_memory_pool, (VOID **)&ptr,(_STACK_SIZE_),TX_NO_WAIT);    \
    if(_retr != TX_SUCCESS)                                                                  \
    {                                                                                        \
      goto error;                                                                            \
    }                                                                                        \
    _retr = tx_thread_create(&(_ID_),#_NAME_,(_FUNC_), (int)_PARAM_,                         \
                         ptr,(_STACK_SIZE_),                                                 \
                         _PRIORITY_, 1, TX_NO_TIME_SLICE,                                    \
                         TX_AUTO_START);                                                     \
    if(_retr != TX_SUCCESS)                                                                  \
    {                                                                                        \
      goto error;                                                                            \
    }                                                                                        \
  } while(0);

#else
#if (osCMSIS < 0x20000U)

#define OS_CREATE_TASK(_ID_,_NAME_,_FUNC_,_PRIORITY_,_STACK_SIZE_, _PARAM_) \
  do {                                                           \
    osThreadDef(_NAME_, _FUNC_, _PRIORITY_, 0, _STACK_SIZE_);    \
    (_ID_) = osThreadCreate(osThread(_NAME_), (void *)(_PARAM_));\
    if (NULL == (_ID_))                                          \
    {                                                            \
      _retr = USBPD_ERROR;                                       \
      goto error;                                                \
    }                                                            \
  } while(0)
#else

#define OS_CREATE_TASK(_ID_,_NAME_,_FUNC_,_PRIORITY_,_STACK_SIZE_, _PARAM_) \
  do {                                                 \
    osThreadAttr_t Thread_Atrr =                       \
    {                                                  \
      .name       = #_NAME_,                           \
      .priority   = (_PRIORITY_),                      \
      .stack_size = (_STACK_SIZE_)                     \
    };                                                 \
    (_ID_) = osThreadNew(_FUNC_, (void *)(_PARAM_),    \
                         &Thread_Atrr);                \
    if (NULL == (_ID_))                                \
    {                                                  \
      _retr = USBPD_ERROR;                             \
      goto error;                                      \
    }                                                  \
  } while(0)
#endif /* osCMSIS < 0x20000U */
#endif /* USBPD_THREADX */

/* Legacy define for typo error */
#define OS_TASK_IS_SUPENDED OS_TASK_IS_SUSPENDED

/**
  * @brief macro definition used to check is task is suspended
  */
#if defined(USBPD_THREADX)
#define OS_TASK_IS_SUSPENDED(_ID_) (TX_SUSPENDED == (_ID_).tx_thread_state)
#else
#if (osCMSIS < 0x20000U)
#define OS_TASK_IS_SUSPENDED(_ID_) (osThreadSuspended == osThreadGetState((_ID_)))
#else
#define OS_TASK_IS_SUSPENDED(_ID_) (osThreadBlocked == osThreadGetState((_ID_)))
#endif /* osCMSIS < 0x20000U */
#endif /* USBPD_THREADX */



/**
  * @brief macro definition used to get the task ID
  */
#if defined(USBPD_THREADX)
#define OS_TASK_GETID()          tx_thread_identify()
#else
#define OS_TASK_GETID()          osThreadGetId()
#endif /* USBPD_THREADX */

/**
  * @brief macro definition used to suspend a task
  */
#if defined(USBPD_THREADX)
#define OS_TASK_SUSPEND(_ID_)    tx_thread_suspend(_ID_)
#else
#define OS_TASK_SUSPEND(_ID_)    osThreadSuspend(_ID_)
#endif /* USBPD_THREADX */

/**
  * @brief macro definition used to kill a task
  */
#if defined(USBPD_THREADX)
#define OS_TASK_KILL(_ID_)    tx_thread_terminate(&_ID_)
#else
#define OS_TASK_KILL(_ID_)    osThreadTerminate(_ID_)
#endif /* USBPD_THREADX */


/**
  * @brief macro definition used to resume a task
  */
#if defined(USBPD_THREADX)

#define OS_TASK_RESUME(_ID_)     tx_thread_resume(&_ID_)
#else

#define OS_TASK_RESUME(_ID_)     osThreadResume(_ID_)
#endif /* USBPD_THREADX */

/**
  * @brief macro definition used to manage the delay
  */
#if defined(USBPD_THREADX)

#define OS_DELAY(_TIME_)   tx_thread_sleep(_TIME_)
#else

#define OS_DELAY(_TIME_)   osDelay(_TIME_)
#endif /* USBPD_THREADX */

/**
  * @brief macro definition used to start the task scheduling
  */
#if defined(USBPD_THREADX)

#define OS_KERNEL_START() /* This function is not managed at usbpd level in the case of threadX */
#else
#if (osCMSIS >= 0x20000U)
#define OS_KERNEL_START()  do { (void)osKernelInitialize(); \
                                (void)osKernelStart();      \
                              } while(0)
#else
#define OS_KERNEL_START()  (void)osKernelStart()
#endif /* osCMSIS >= 0x20000U */
#endif /* USBPD_THREADX */

/* Exported types ------------------------------------------------------------*/
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif
#endif /* USBPD_CORE_OSPORT_H_ */

