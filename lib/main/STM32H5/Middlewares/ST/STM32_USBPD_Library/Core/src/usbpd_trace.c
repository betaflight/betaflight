/**
  ******************************************************************************
  * @file    usbpd_trace.c
  * @author  MCD Application Team
  * @brief   This file contains trace control functions.
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
#define USBPD_TRACE_C

#include "usbpd_def.h"
#include "usbpd_core.h"
#include "usbpd_trace.h"
#ifdef _TRACE
#include "tracer_emb.h"
#endif /* _TRACE */

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_CORE
  * @{
  */

/** @addtogroup USBPD_CORE_TRACE
  * @{
  */

/* Private enums -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup USBPD_CORE_TRACE_Private_Defines USBPD CORE TRACE Private Defines
  * @{
  */

#define TRACE_SIZE_HEADER_TRACE   9u      /* Type + Time x 2 + PortNum + Sop + Size */

#define TRACE_PORT_BIT_POSITION   5u      /* Bit position of port number in TAG id */

#define TLV_SOF                   (uint8_t)0xFDu
#define TLV_EOF                   (uint8_t)0xA5u
#define TLV_SIZE_MAX              256u
#define TLV_HEADER_SIZE           3u /* Size of TLV header (TAG(1) + LENGTH(2)  */
#define TLV_SOF_SIZE              4u /* TLV_SOF * 4                             */
#define TLV_EOF_SIZE              4u /* TLV_EOF * 4                             */

#define DEBUG_STACK_MESSAGE       0x12u
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/** @defgroup USBPD_CORE_TRACE_Private_Macros USBPD CORE TRACE Private Macros
  * @{
  */
#define TRACE_SET_TAG_ID(_PORT_, _TAG_)         (((_PORT_) << TRACE_PORT_BIT_POSITION) | (_TAG_))

#define TRACER_EMB_WRITE_DATA(_POSITION_,_DATA_)  do {                                        \
                                                       TRACER_EMB_WriteData((_POSITION_),(_DATA_));\
                                                       (_POSITION_) = ((_POSITION_) + 1u);         \
                                                     } while(0)
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup USBPD_CORE_TRACE_Private_Variables USBPD CORE TRACE Private Variables
  * @{
  */
extern uint32_t HAL_GetTick(void);
extern void     USBPD_DPM_TraceWakeUp(void);
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @addtogroup USBPD_CORE_TRACE_Exported_Functions
  * @{
  */
void USBPD_TRACE_Init(void)
{
#if defined(_TRACE)
  static const uint8_t OverFlow_String[] =
  {
    TLV_SOF, TLV_SOF, TLV_SOF, TLV_SOF,   /* Buffer header */
    0x32,                                 /* Tag id */
    0x0, 0x18,                            /* Length */
    0x6,                                  /* Type */
    0x0, 0x0, 0x0, 0x0,                   /* Time   */
    0x0,                                  /* PortNum */
    0x0,                                  /* SOP */
    0x0, 0x0F,                                                                 /* Size */
    'T', 'R', 'A', 'C', 'E', ' ', 'O', 'V', 'E', 'R', '_', 'F', 'L', 'O', 'W', /* Data */
    TLV_EOF, TLV_EOF, TLV_EOF, TLV_EOF                                         /* Buffer end */
  };

  /* initialize tracer module */
  TRACER_EMB_Init();

  /* Initialize PE trace */
  USBPD_PE_SetTrace(USBPD_TRACE_Add, 3u);

  /* Initialize the overflow detection */
  (void)TRACER_EMB_EnableOverFlow(OverFlow_String, (uint8_t)sizeof(OverFlow_String));
#else
  return;
#endif /* _TRACE */
}

void USBPD_TRACE_DeInit(void)
{
  /* Nothing to do */
  return;
}

void  USBPD_TRACE_Add(TRACE_EVENT Type, uint8_t PortNum, uint8_t Sop, uint8_t *Ptr, uint32_t Size)
{
#if defined(_TRACE)
  uint32_t _time;
  int32_t _allocation;
  uint16_t index;

  /*  Get trace timing */
  _time = HAL_GetTick();

  TRACER_EMB_Lock();

  /* Data are encapsulate inside a TLV string*/
  /* Allocate buffer Size */
  _allocation = TRACER_EMB_AllocateBufer(Size + TRACE_SIZE_HEADER_TRACE +
                                         TLV_HEADER_SIZE + TLV_SOF_SIZE + TLV_EOF_SIZE);

  /* Check allocation */
  if (_allocation  != -1)
  {
    uint16_t _writepos = (uint16_t)_allocation;

    /* Copy SOF bytes */
    for (index = 0u; index < TLV_SOF_SIZE; index++)
    {
      TRACER_EMB_WRITE_DATA(_writepos, TLV_SOF);
    }
    /* Copy the TAG */
    TRACER_EMB_WRITE_DATA(_writepos, TRACE_SET_TAG_ID((PortNum + 1u), DEBUG_STACK_MESSAGE));
    /* Copy the LENGTH */
    TRACER_EMB_WRITE_DATA(_writepos, (uint8_t)((Size + TRACE_SIZE_HEADER_TRACE) >> 8u));
    TRACER_EMB_WRITE_DATA(_writepos, (uint8_t)(Size + TRACE_SIZE_HEADER_TRACE));

    /* Trace type */
    TRACER_EMB_WRITE_DATA(_writepos, (uint8_t)Type);

    TRACER_EMB_WRITE_DATA(_writepos, (uint8_t)_time);
    TRACER_EMB_WRITE_DATA(_writepos, (uint8_t)(_time >> 8u));
    TRACER_EMB_WRITE_DATA(_writepos, (uint8_t)(_time >> 16u));
    TRACER_EMB_WRITE_DATA(_writepos, (uint8_t)(_time >> 24u));

    TRACER_EMB_WRITE_DATA(_writepos, PortNum);
    TRACER_EMB_WRITE_DATA(_writepos, Sop);

    TRACER_EMB_WRITE_DATA(_writepos, (uint8_t)(Size >> 8u));
    TRACER_EMB_WRITE_DATA(_writepos, (uint8_t)Size);

    /* initialize the Ptr for Read/Write */
    for (index = 0u; index < Size; index++)
    {
      TRACER_EMB_WRITE_DATA(_writepos, Ptr[index]);
    }

    /* Copy EOF bytes */
    for (index = 0u; index < TLV_EOF_SIZE; index++)
    {
      TRACER_EMB_WRITE_DATA(_writepos, TLV_EOF);
    }
  }

  TRACER_EMB_UnLock();

  TRACER_EMB_SendData();
#else
  return;
#endif /* _TRACE */
}

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

