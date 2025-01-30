/**
  *
  * @file    apm32f4xx_dal_dci.h
  * @brief   Header file of DCI DAL module.
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
  * Copyright (c) 2017 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_DCI_H
#define APM32F4xx_DAL_DCI_H

#ifdef __cplusplus
 extern "C" {
#endif

#if defined(APM32F407xx) || defined(APM32F417xx)
/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/* Include DCI DAL Extended module */
/* (include on top of file since DCI structures are defined in extended file) */
#include "apm32f4xx_dal_dci_ex.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup DCI DCI
  * @brief DCI DAL module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup DCI_Exported_Types DCI Exported Types
  * @{
  */
/**
  * @brief   DCI Embedded Synchronisation CODE Init structure definition
  */
typedef struct
{
  uint8_t FrameStartUnmask; /*!< Specifies the frame start delimiter unmask. */
  uint8_t LineStartUnmask;  /*!< Specifies the line start delimiter unmask.  */
  uint8_t LineEndUnmask;    /*!< Specifies the line end delimiter unmask.    */
  uint8_t FrameEndUnmask;   /*!< Specifies the frame end delimiter unmask.   */
}DCI_SyncUnmaskTypeDef;
/**
  * @brief  DAL DCI State structures definition
  */ 
typedef enum
{
  DAL_DCI_STATE_RESET             = 0x00U,  /*!< DCI not yet initialized or disabled  */
  DAL_DCI_STATE_READY             = 0x01U,  /*!< DCI initialized and ready for use    */
  DAL_DCI_STATE_BUSY              = 0x02U,  /*!< DCI internal processing is ongoing   */
  DAL_DCI_STATE_TIMEOUT           = 0x03U,  /*!< DCI timeout state                    */
  DAL_DCI_STATE_ERROR             = 0x04U,  /*!< DCI error state                      */
  DAL_DCI_STATE_SUSPENDED         = 0x05U   /*!< DCI suspend state                    */
}DAL_DCI_StateTypeDef;

/** 
  * @brief  DCI handle Structure definition
  */
typedef struct __DCI_HandleTypeDef
{
  DCI_TypeDef                  *Instance;           /*!< DCI Register base address   */

  DCI_InitTypeDef              Init;                /*!< DCI parameters              */

  DAL_LockTypeDef               Lock;                /*!< DCI locking object          */

  __IO DAL_DCI_StateTypeDef    State;               /*!< DCI state                   */

  __IO uint32_t                 XferCount;           /*!< DMA transfer counter         */

  __IO uint32_t                 XferSize;            /*!< DMA transfer size            */

  uint32_t                      XferTransferNumber;  /*!< DMA transfer number          */

  uint32_t                      pBuffPtr;            /*!< Pointer to DMA output buffer */

  DMA_HandleTypeDef             *DMA_Handle;         /*!< Pointer to the DMA handler   */

  __IO uint32_t                 ErrorCode;           /*!< DCI Error code              */
#if (USE_DAL_DCI_REGISTER_CALLBACKS == 1)
  void    (* FrameEventCallback) ( struct __DCI_HandleTypeDef *hdci);  /*!< DCI Frame Event Callback */
  void    (* VsyncEventCallback) ( struct __DCI_HandleTypeDef *hdci);  /*!< DCI Vsync Event Callback */
  void    (* LineEventCallback ) ( struct __DCI_HandleTypeDef *hdci);  /*!< DCI Line Event Callback  */ 
  void    (* ErrorCallback)      ( struct __DCI_HandleTypeDef *hdci);  /*!< DCI Error Callback       */
  void    (* MspInitCallback)    ( struct __DCI_HandleTypeDef *hdci);  /*!< DCI Msp Init callback    */
  void    (* MspDeInitCallback)  ( struct __DCI_HandleTypeDef *hdci);  /*!< DCI Msp DeInit callback  */  
#endif  /* USE_DAL_DCI_REGISTER_CALLBACKS */ 
}DCI_HandleTypeDef;

#if (USE_DAL_DCI_REGISTER_CALLBACKS == 1)
typedef enum
{
  DAL_DCI_FRAME_EVENT_CB_ID    = 0x00U,    /*!< DCI Frame Event Callback ID */
  DAL_DCI_VSYNC_EVENT_CB_ID    = 0x01U,    /*!< DCI Vsync Event Callback ID */
  DAL_DCI_LINE_EVENT_CB_ID     = 0x02U,    /*!< DCI Line Event Callback ID  */ 
  DAL_DCI_ERROR_CB_ID          = 0x03U,    /*!< DCI Error Callback ID       */
  DAL_DCI_MSPINIT_CB_ID        = 0x04U,    /*!< DCI MspInit callback ID     */
  DAL_DCI_MSPDEINIT_CB_ID      = 0x05U     /*!< DCI MspDeInit callback ID   */
  
}DAL_DCI_CallbackIDTypeDef;
    
typedef void (*pDCI_CallbackTypeDef)(DCI_HandleTypeDef *hdci);
#endif /* USE_DAL_DCI_REGISTER_CALLBACKS */


/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup DCI_Exported_Constants DCI Exported Constants
  * @{
  */

/** @defgroup DCI_Error_Code DCI Error Code
  * @{
  */
#define DAL_DCI_ERROR_NONE      0x00000000U    /*!< No error              */
#define DAL_DCI_ERROR_OVR       0x00000001U    /*!< Overrun error         */
#define DAL_DCI_ERROR_SYNC      0x00000002U    /*!< Synchronization error */
#define DAL_DCI_ERROR_TIMEOUT   0x00000020U    /*!< Timeout error         */
#define DAL_DCI_ERROR_DMA       0x00000040U    /*!< DMA error             */
#if (USE_DAL_DCI_REGISTER_CALLBACKS == 1)
#define DAL_DCI_ERROR_INVALID_CALLBACK ((uint32_t)0x00000080U)  /*!< Invalid callback error */
#endif
/**
  * @}
  */

/** @defgroup DCI_Capture_Mode DCI Capture Mode
  * @{
  */ 
#define DCI_MODE_CONTINUOUS         0x00000000U               /*!< The received data are transferred continuously
                                                                    into the destination memory through the DMA             */
#define DCI_MODE_SNAPSHOT             ((uint32_t)DCI_CTRL_CMODE)  /*!< Once activated, the interface waits for the start of
                                                                    frame and then transfers a single frame through the DMA */
/**
  * @}
  */

/** @defgroup DCI_Synchronization_Mode DCI Synchronization Mode
  * @{
  */
#define DCI_SYNCHRO_HARDWARE        0x00000000U               /*!< Hardware synchronization data capture (frame/line start/stop)
                                                                    is synchronized with the HSYNC/VSYNC signals                  */
#define DCI_SYNCHRO_EMBEDDED        ((uint32_t)DCI_CTRL_ESYNCSEL)   /*!< Embedded synchronization data capture is synchronized with
                                                                    synchronization codes embedded in the data flow               */

/**
  * @}
  */

/** @defgroup DCI_PIXCK_Polarity DCI PIXCK Polarity
  * @{
  */
#define DCI_PCKPOLARITY_FALLING     0x00000000U                 /*!< Pixel clock active on Falling edge */
#define DCI_PCKPOLARITY_RISING      ((uint32_t)DCI_CTRL_PXCLKPOL)  /*!< Pixel clock active on Rising edge  */

/**
  * @}
  */

/** @defgroup DCI_VSYNC_Polarity DCI VSYNC Polarity
  * @{
  */
#define DCI_VSPOLARITY_LOW          0x00000000U                /*!< Vertical synchronization active Low  */
#define DCI_VSPOLARITY_HIGH         ((uint32_t)DCI_CTRL_VSYNCPOL)  /*!< Vertical synchronization active High */

/**
  * @}
  */

/** @defgroup DCI_HSYNC_Polarity DCI HSYNC Polarity
  * @{
  */ 
#define DCI_HSPOLARITY_LOW          0x00000000U                /*!< Horizontal synchronization active Low  */
#define DCI_HSPOLARITY_HIGH         ((uint32_t)DCI_CTRL_HSYNCPOL)  /*!< Horizontal synchronization active High */

/**
  * @}
  */

/** @defgroup DCI_MODE_JPEG DCI MODE JPEG
  * @{
  */
#define DCI_JPEG_DISABLE            0x00000000U               /*!< Mode JPEG Disabled  */
#define DCI_JPEG_ENABLE             ((uint32_t)DCI_CTRL_JPGFM)  /*!< Mode JPEG Enabled   */

/**
  * @}
  */

/** @defgroup DCI_Capture_Rate DCI Capture Rate
  * @{
  */
#define DCI_CR_ALL_FRAME            0x00000000U                 /*!< All frames are captured        */
#define DCI_CR_ALTERNATE_2_FRAME    ((uint32_t)DCI_CTRL_FCRCFG_0)  /*!< Every alternate frame captured */
#define DCI_CR_ALTERNATE_4_FRAME    ((uint32_t)DCI_CTRL_FCRCFG_1)  /*!< One frame in 4 frames captured */

/**
  * @}
  */

/** @defgroup DCI_Extended_Data_Mode DCI Extended Data Mode
  * @{
  */
#define DCI_EXTEND_DATA_8B     0x00000000U                                  /*!< Interface captures 8-bit data on every pixel clock  */
#define DCI_EXTEND_DATA_10B    ((uint32_t)DCI_CTRL_EXDMOD_0)                    /*!< Interface captures 10-bit data on every pixel clock */
#define DCI_EXTEND_DATA_12B    ((uint32_t)DCI_CTRL_EXDMOD_1)                    /*!< Interface captures 12-bit data on every pixel clock */
#define DCI_EXTEND_DATA_14B    ((uint32_t)(DCI_CTRL_EXDMOD_0 | DCI_CTRL_EXDMOD_1))  /*!< Interface captures 14-bit data on every pixel clock */

/**
  * @}
  */

/** @defgroup DCI_Window_Coordinate DCI Window Coordinate
  * @{
  */
#define DCI_WINDOW_COORDINATE    0x3FFFU   /*!< Window coordinate */

/**
  * @}
  */

/** @defgroup DCI_Window_Height DCI Window Height
  * @{
  */ 
#define DCI_WINDOW_HEIGHT        0x1FFFU   /*!< Window Height */

/**
  * @}
  */

/** @defgroup DCI_Window_Vertical_Line DCI Window Vertical Line
  * @{
  */
#define DCI_POSITION_CWSIZE_VLINE         (uint32_t)DCI_CROPWSIZE_VLINECNT_Pos /*!< Required left shift to set crop window vertical line count       */
#define DCI_POSITION_CWSTRT_VST           (uint32_t)DCI_CROPWSTAT_VSLINECNT_Pos   /*!< Required left shift to set crop window vertical start line count */

/**
  * @}
  */

/** @defgroup DCI_interrupt_sources  DCI interrupt sources
  * @{
  */
#define DCI_IT_FRAME    ((uint32_t)DCI_INTEN_CCINTEN)    /*!< Capture complete interrupt      */
#define DCI_IT_OVR      ((uint32_t)DCI_INTEN_OVRINTEN)      /*!< Overrun interrupt               */
#define DCI_IT_ERR      ((uint32_t)DCI_INTEN_SYNCERRINTEN)      /*!< Synchronization error interrupt */
#define DCI_IT_VSYNC    ((uint32_t)DCI_INTEN_VSYNCINTEN)    /*!< VSYNC interrupt                 */
#define DCI_IT_LINE     ((uint32_t)DCI_INTEN_LINEINTEN)     /*!< Line interrupt                  */
/**
  * @}
  */

/** @defgroup DCI_Flags DCI Flags
  * @{
  */

/** 
  * @brief   DCI SR register
  */
#define DCI_FLAG_HSYNC     ((uint32_t)DCI_STS_INDEX|DCI_STS_HSYNCSTS) /*!< HSYNC pin state (active line / synchronization between lines)   */
#define DCI_FLAG_VSYNC     ((uint32_t)DCI_STS_INDEX|DCI_STS_VSYNCSTS) /*!< VSYNC pin state (active frame / synchronization between frames) */
#define DCI_FLAG_FNE       ((uint32_t)DCI_STS_INDEX|DCI_STS_FIFONEMP)   /*!< FIFO not empty flag                                             */
/** 
  * @brief   DCI RIS register
  */ 
#define DCI_FLAG_FRAMERI    ((uint32_t)DCI_RINTSTS_CC_RINT_RIS)  /*!< Frame capture complete interrupt flag */
#define DCI_FLAG_OVRRI      ((uint32_t)DCI_RINTSTS_OVR_RINT_RIS)    /*!< Overrun interrupt flag                */
#define DCI_FLAG_ERRRI      ((uint32_t)DCI_RINTSTS_SYNCERR_RINT_RIS)    /*!< Synchronization error interrupt flag  */
#define DCI_FLAG_VSYNCRI    ((uint32_t)DCI_RINTSTS_VSYNC_RINT_RIS)  /*!< VSYNC interrupt flag                  */
#define DCI_FLAG_LINERI     ((uint32_t)DCI_RINTSTS_LINE_RINT_RIS)   /*!< Line interrupt flag                   */
/** 
  * @brief   DCI MIS register
  */ 
#define DCI_FLAG_FRAMEMI    ((uint32_t)DCI_MIS_INDEX|DCI_MIS_FRAME_MIS)  /*!< DCI Frame capture complete masked interrupt status */
#define DCI_FLAG_OVRMI      ((uint32_t)DCI_MIS_INDEX|DCI_MIS_OVR_MIS  )  /*!< DCI Overrun masked interrupt status                */
#define DCI_FLAG_ERRMI      ((uint32_t)DCI_MIS_INDEX|DCI_MIS_ERR_MIS  )  /*!< DCI Synchronization error masked interrupt status  */
#define DCI_FLAG_VSYNCMI    ((uint32_t)DCI_MIS_INDEX|DCI_MIS_VSYNC_MIS)  /*!< DCI VSYNC masked interrupt status                  */
#define DCI_FLAG_LINEMI     ((uint32_t)DCI_MIS_INDEX|DCI_MIS_LINE_MIS )  /*!< DCI Line masked interrupt status                   */
/**
  * @}
  */

/**
  * @}
  */
 
/* Exported macro ------------------------------------------------------------*/
/** @defgroup DCI_Exported_Macros DCI Exported Macros
  * @{
  */
  
/** @brief Reset DCI handle state
  * @param  __HANDLE__ specifies the DCI handle.
  * @retval None
  */
#define __DAL_DCI_RESET_HANDLE_STATE(__HANDLE__) do{                                            \
                                                     (__HANDLE__)->State = DAL_DCI_STATE_RESET; \
                                                     (__HANDLE__)->MspInitCallback = NULL;      \
                                                     (__HANDLE__)->MspDeInitCallback = NULL;    \
                                                   } while(0)

/**
  * @brief  Enable the DCI.
  * @param  __HANDLE__ DCI handle
  * @retval None
  */
#define __DAL_DCI_ENABLE(__HANDLE__)    ((__HANDLE__)->Instance->CTRL |= DCI_CTRL_DCIEN)

/**
  * @brief  Disable the DCI.
  * @param  __HANDLE__ DCI handle
  * @retval None
  */
#define __DAL_DCI_DISABLE(__HANDLE__)   ((__HANDLE__)->Instance->CTRL &= ~(DCI_CTRL_DCIEN))

/* Interrupt & Flag management */
/**
  * @brief  Get the DCI pending flag.
  * @param  __HANDLE__ DCI handle
  * @param  __FLAG__ Get the specified flag.
  *         This parameter can be one of the following values (no combination allowed)
  *            @arg DCI_FLAG_HSYNC: HSYNC pin state (active line / synchronization between lines)
  *            @arg DCI_FLAG_VSYNC: VSYNC pin state (active frame / synchronization between frames)
  *            @arg DCI_FLAG_FNE: FIFO empty flag
  *            @arg DCI_FLAG_FRAMERI: Frame capture complete flag mask
  *            @arg DCI_FLAG_OVRRI: Overrun flag mask
  *            @arg DCI_FLAG_ERRRI: Synchronization error flag mask
  *            @arg DCI_FLAG_VSYNCRI: VSYNC flag mask
  *            @arg DCI_FLAG_LINERI: Line flag mask
  *            @arg DCI_FLAG_FRAMEMI: DCI Capture complete masked interrupt status
  *            @arg DCI_FLAG_OVRMI: DCI Overrun masked interrupt status
  *            @arg DCI_FLAG_ERRMI: DCI Synchronization error masked interrupt status
  *            @arg DCI_FLAG_VSYNCMI: DCI VSYNC masked interrupt status
  *            @arg DCI_FLAG_LINEMI: DCI Line masked interrupt status
  * @retval The state of FLAG.
  */
#define __DAL_DCI_GET_FLAG(__HANDLE__, __FLAG__)\
((((__FLAG__) & (DCI_STS_INDEX|DCI_MIS_INDEX)) == 0x0U)? ((__HANDLE__)->Instance->RINTSTS & (__FLAG__)) :\
 (((__FLAG__) & DCI_STS_INDEX) == 0x0U)? ((__HANDLE__)->Instance->MINTSTS & (__FLAG__)) : ((__HANDLE__)->Instance->STS & (__FLAG__)))

/**
  * @brief  Clear the DCI pending flags.
  * @param  __HANDLE__ DCI handle
  * @param  __FLAG__ specifies the flag to clear.
  *         This parameter can be any combination of the following values:
  *            @arg DCI_FLAG_FRAMERI: Frame capture complete flag mask
  *            @arg DCI_FLAG_OVRRI: Overrun flag mask
  *            @arg DCI_FLAG_ERRRI: Synchronization error flag mask
  *            @arg DCI_FLAG_VSYNCRI: VSYNC flag mask
  *            @arg DCI_FLAG_LINERI: Line flag mask
  * @retval None
  */
#define __DAL_DCI_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->INTCLR = (__FLAG__))

/**
  * @brief  Enable the specified DCI interrupts.
  * @param  __HANDLE__    DCI handle
  * @param  __INTERRUPT__ specifies the DCI interrupt sources to be enabled.
  *         This parameter can be any combination of the following values:
  *            @arg DCI_IT_FRAME: Frame capture complete interrupt mask
  *            @arg DCI_IT_OVR: Overrun interrupt mask
  *            @arg DCI_IT_ERR: Synchronization error interrupt mask
  *            @arg DCI_IT_VSYNC: VSYNC interrupt mask
  *            @arg DCI_IT_LINE: Line interrupt mask
  * @retval None
  */
#define __DAL_DCI_ENABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->INTEN |= (__INTERRUPT__))

/**
  * @brief  Disable the specified DCI interrupts.
  * @param  __HANDLE__ DCI handle
  * @param  __INTERRUPT__ specifies the DCI interrupt sources to be enabled.
  *         This parameter can be any combination of the following values:
  *            @arg DCI_IT_FRAME: Frame capture complete interrupt mask
  *            @arg DCI_IT_OVR: Overrun interrupt mask
  *            @arg DCI_IT_ERR: Synchronization error interrupt mask
  *            @arg DCI_IT_VSYNC: VSYNC interrupt mask
  *            @arg DCI_IT_LINE: Line interrupt mask
  * @retval None
  */
#define __DAL_DCI_DISABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->INTEN &= ~(__INTERRUPT__))

/**
  * @brief  Check whether the specified DCI interrupt has occurred or not.
  * @param  __HANDLE__ DCI handle
  * @param  __INTERRUPT__ specifies the DCI interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg DCI_IT_FRAME: Frame capture complete interrupt mask
  *            @arg DCI_IT_OVR: Overrun interrupt mask
  *            @arg DCI_IT_ERR: Synchronization error interrupt mask
  *            @arg DCI_IT_VSYNC: VSYNC interrupt mask
  *            @arg DCI_IT_LINE: Line interrupt mask
  * @retval The state of INTERRUPT.
  */
#define __DAL_DCI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->MINTSTS & (__INTERRUPT__))

/**
  * @}
  */
  
/* Exported functions --------------------------------------------------------*/
/** @addtogroup DCI_Exported_Functions DCI Exported Functions
  * @{
  */

/** @addtogroup DCI_Exported_Functions_Group1 Initialization and Configuration functions
 * @{
 */
/* Initialization and de-initialization functions *****************************/
DAL_StatusTypeDef DAL_DCI_Init(DCI_HandleTypeDef *hdci);
DAL_StatusTypeDef DAL_DCI_DeInit(DCI_HandleTypeDef *hdci);
void              DAL_DCI_MspInit(DCI_HandleTypeDef* hdci);
void              DAL_DCI_MspDeInit(DCI_HandleTypeDef* hdci);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_DAL_DCI_REGISTER_CALLBACKS == 1)   
DAL_StatusTypeDef DAL_DCI_RegisterCallback(DCI_HandleTypeDef *hdci, DAL_DCI_CallbackIDTypeDef CallbackID, pDCI_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_DCI_UnRegisterCallback(DCI_HandleTypeDef *hdci, DAL_DCI_CallbackIDTypeDef CallbackID);
#endif /* USE_DAL_DCI_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup DCI_Exported_Functions_Group2 IO operation functions
 * @{
 */
/* IO operation functions *****************************************************/
DAL_StatusTypeDef DAL_DCI_Start_DMA(DCI_HandleTypeDef* hdci, uint32_t DCI_Mode, uint32_t pData, uint32_t Length);
DAL_StatusTypeDef DAL_DCI_Stop(DCI_HandleTypeDef* hdci);
DAL_StatusTypeDef DAL_DCI_Suspend(DCI_HandleTypeDef* hdci);
DAL_StatusTypeDef DAL_DCI_Resume(DCI_HandleTypeDef* hdci);
void              DAL_DCI_ErrorCallback(DCI_HandleTypeDef *hdci);
void              DAL_DCI_LineEventCallback(DCI_HandleTypeDef *hdci);
void              DAL_DCI_FrameEventCallback(DCI_HandleTypeDef *hdci);
void              DAL_DCI_VsyncEventCallback(DCI_HandleTypeDef *hdci);
void              DAL_DCI_VsyncCallback(DCI_HandleTypeDef *hdci);
void              DAL_DCI_HsyncCallback(DCI_HandleTypeDef *hdci);
void              DAL_DCI_IRQHandler(DCI_HandleTypeDef *hdci);
/**
  * @}
  */
  
/** @addtogroup DCI_Exported_Functions_Group3 Peripheral Control functions
 * @{
 */
/* Peripheral Control functions ***********************************************/
DAL_StatusTypeDef DAL_DCI_ConfigCrop(DCI_HandleTypeDef *hdci, uint32_t X0, uint32_t Y0, uint32_t XSize, uint32_t YSize);
DAL_StatusTypeDef DAL_DCI_EnableCrop(DCI_HandleTypeDef *hdci);
DAL_StatusTypeDef DAL_DCI_DisableCrop(DCI_HandleTypeDef *hdci);
DAL_StatusTypeDef DAL_DCI_ConfigSyncUnmask(DCI_HandleTypeDef *hdci, DCI_SyncUnmaskTypeDef *SyncUnmask);
/**
  * @}
  */
  
/** @addtogroup DCI_Exported_Functions_Group4 Peripheral State functions
 * @{
 */
/* Peripheral State functions *************************************************/
DAL_DCI_StateTypeDef DAL_DCI_GetState(DCI_HandleTypeDef *hdci);
uint32_t              DAL_DCI_GetError(DCI_HandleTypeDef *hdci);
/**
  * @}
  */

/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup DCI_Private_Constants DCI Private Constants
  * @{
  */
#define DCI_MIS_INDEX        0x1000U /*!< DCI MIS register index */
#define DCI_STS_INDEX         0x2000U /*!< DCI SR register index  */
/**
  * @}
  */   
/* Private macro -------------------------------------------------------------*/
/** @defgroup DCI_Private_Macros DCI Private Macros
  * @{
  */
#define IS_DCI_CAPTURE_MODE(MODE)(((MODE) == DCI_MODE_CONTINUOUS) || \
                                   ((MODE) == DCI_MODE_SNAPSHOT))

#define IS_DCI_SYNCHRO(MODE)(((MODE) == DCI_SYNCHRO_HARDWARE) || \
                              ((MODE) == DCI_SYNCHRO_EMBEDDED))
                              
#define IS_DCI_PCKPOLARITY(POLARITY)(((POLARITY) == DCI_PCKPOLARITY_FALLING) || \
                                      ((POLARITY) == DCI_PCKPOLARITY_RISING))
                                      
#define IS_DCI_VSPOLARITY(POLARITY)(((POLARITY) == DCI_VSPOLARITY_LOW) || \
                                     ((POLARITY) == DCI_VSPOLARITY_HIGH))
                                     
#define IS_DCI_HSPOLARITY(POLARITY)(((POLARITY) == DCI_HSPOLARITY_LOW) || \
                                     ((POLARITY) == DCI_HSPOLARITY_HIGH))
                                     
#define IS_DCI_MODE_JPEG(JPEG_MODE)(((JPEG_MODE) == DCI_JPEG_DISABLE) || \
                                     ((JPEG_MODE) == DCI_JPEG_ENABLE))
                                     
#define IS_DCI_CAPTURE_RATE(RATE) (((RATE) == DCI_CR_ALL_FRAME)         || \
                                    ((RATE) == DCI_CR_ALTERNATE_2_FRAME) || \
                                    ((RATE) == DCI_CR_ALTERNATE_4_FRAME))
                                    
#define IS_DCI_EXTENDED_DATA(DATA)(((DATA) == DCI_EXTEND_DATA_8B)  || \
                                    ((DATA) == DCI_EXTEND_DATA_10B) || \
                                    ((DATA) == DCI_EXTEND_DATA_12B) || \
                                    ((DATA) == DCI_EXTEND_DATA_14B))
                                    
#define IS_DCI_WINDOW_COORDINATE(COORDINATE) ((COORDINATE) <= DCI_WINDOW_COORDINATE)

#define IS_DCI_WINDOW_HEIGHT(HEIGHT) ((HEIGHT) <= DCI_WINDOW_HEIGHT)

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @addtogroup DCI_Private_Functions DCI Private Functions
  * @{
  */
  
/**
  * @}
  */
      
#endif /* APM32F407xx || APM32F417xx */

/**
  * @}
  */
    
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DAL_DCI_H */
