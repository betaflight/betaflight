/**
 ******************************************************************************
 * @file     um324xx_hal_dma.h
 * @author   MCU Team
 * @version  V1.00
 * @date     2023-04-11
 * @brief
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324XX_HAL_DMA_H__
#define __UM324XX_HAL_DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"


/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup DMA
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup DMA_Exported_Types DMA Exported Types
  * @{
  */

/**
  * @brief  DMA Configuration Structure definition
  */
typedef struct
{
    uint32_t SrcRequest;                     /*!< Specifies the request selected for the specified channel.
                                            This parameter can be a value of @ref DMA_request */
    uint32_t DstRequest;                     /*!< Specifies the request selected for the specified channel.
                                            This parameter can be a value of @ref DMA_request */

    uint32_t Direction;                   /*!< Specifies if the data will be transferred from memory to peripheral,
                                            from memory to memory or from peripheral to memory.
                                            This parameter can be a value of @ref DMA_Data_transfer_direction */

    uint32_t SrcInc;                      /*!< Specifies whether the Peripheral address register should be incremented or not.
                                           This parameter can be a value of @ref DMA_source_incremented_mode */

    uint32_t DstInc;                      /*!< Specifies whether the memory address register should be incremented or not.
                                           This parameter can be a value of @ref DMA_destination_incremented_mode */

    uint32_t SrcDataAlignment;            /*!< Specifies the Peripheral data width.
                                           This parameter can be a value of @ref DMA_source_data_size */

    uint32_t DstDataAlignment;            /*!< The Burst length of the target transmission. The number of data
                                        written each time the target peripheral Req handshake signal is available*/

    uint32_t SrcMSize;                    /*!<  Burst length of source transmission. The number of data read each time
                                        the source peripheral Req handshake signal is available*/

    uint32_t DstMSize;                    /*!<  Burst length of source transmission. The number of data read each time
                                        the source peripheral Req handshake signal is available*/

    uint32_t SrcPer;                       /*!< Source peripheral handshake signal number*/

    uint32_t DstPer;                       /*!< Target peripheral handshake signal number*/

    uint32_t SrcReload;                    /*!<  Automatically restart source transmission*/

    uint32_t DstReload;                    /*!<  Automatically restart target transmission*/

    uint32_t SrcHsSel;                     /*!<  Source transmission handshake signal selection*/

    uint32_t DstHsSel;                     /*!<  Target transmission handshake signal selection*/

    uint32_t FIFOMode;                      /*!< Initiate transmission to the target when the available data is
                                            greater than or equal to half the FIFO depth, and initiate transmission
                                            when the space is greater than or equal to half the FIFO depth; Exception
                                            at the end of a Burst or Block transmission */

    uint32_t FCMode;                      /*!<Turn off pre reading and do not transfer source data until
                                                the target transfer is completed*/

    uint32_t Priority;                  /*!< Specifies the software priority for the DMAy Channelx.
                                           This parameter can be a value of @ref DMA_Priority_level */
} DMA_InitTypeDef;

/**
  * @brief  HAL DMA State structures definition
  */
typedef enum
{
    HAL_DMA_STATE_RESET             = 0x00U,  /*!< DMA not yet initialized or disabled    */
    HAL_DMA_STATE_READY             = 0x01U,  /*!< DMA initialized and ready for use      */
    HAL_DMA_STATE_BUSY              = 0x02U,  /*!< DMA process is ongoing                 */
    HAL_DMA_STATE_TIMEOUT           = 0x03U,  /*!< DMA timeout state                     */
    HAL_DMA_STATE_ERROR             = 0x04U,  /*!< DMA error state                     */
    HAL_DMA_STATE_ABORT             = 0x05U,  /*!< DMA Abort state                     */
} HAL_DMA_StateTypeDef;

/**
  * @brief  HAL DMA Error Code structure definition
  */
typedef enum
{
    HAL_DMA_FULL_TRANSFER      = 0x00U,    /*!< Full transfer     */
    HAL_DMA_HALF_TRANSFER      = 0x01U     /*!< Half Transfer     */
} HAL_DMA_LevelCompleteTypeDef;

/**
  * @brief  HAL DMA Channel number
  */
typedef enum
{
    DMA_Channel_0  =  0U ,
    DMA_Channel_1  =  1U ,
    DMA_Channel_2  =  2U ,
    DMA_Channel_3  =  3U ,
    DMA_Channel_4  =  4U ,
    DMA_Channel_5  =  5U ,
    DMA_Channel_6  =  6U ,
    DMA_Channel_7  =  7U
} DMA_Channel_TypeDef;


/**
  * @brief  DMA handle Structure definition
  */
typedef struct __DMA_HandleTypeDef
{
#if defined(UM324xF)   
	DMA_Stream_TypeDef    *Instance1;                                                 /*!< Register base address                */
#endif	
    DMA_TypeDef           *Instance;                                                   /*!< Register base address                */

    uint32_t              DmaChannelSel;                                               /**< DMA Channel   */

    DMA_InitTypeDef       Init;                                                        /*!< DMA communication parameters         */

    HAL_LockTypeDef       Lock;                                                        /*!< DMA locking object                   */

    __IO HAL_DMA_StateTypeDef  State;                                                  /*!< DMA transfer state                   */

    void                  *Parent;                                                     /*!< Parent object state                  */

    void (* XferTfrCallback)(struct __DMA_HandleTypeDef *hdma);                        /*!< DMA transfer complete callback       */
    void (* XferBlockCallback)(struct __DMA_HandleTypeDef *hdma);                      /*!< DMA transfer complete callback       */
    void (* XferSrcTranCallback)(struct __DMA_HandleTypeDef *hdma);                    /*!< DMA transfer complete callback       */
    void (* XferDstTranCallback)(struct __DMA_HandleTypeDef *hdma);                    /*!< DMA transfer complete callback       */
    void (* XferErrorCallback)(struct __DMA_HandleTypeDef *hdma);                      /*!< DMA transfer complete callback       */

    __IO uint32_t          ErrorCode;                                           /*!< DMA Error code                       */

#if defined(UM32x42x) || defined(UM32x41x)    
    DMAMUX_TypeDef         *DmamuxBaseAddress;                                /*!< Register base address                */

    uint32_t               DmamuxChannelSel;                                  /*!< DMAMUX Channels  */

    uint32_t               DmamuxReqGenSel;                                  /*!< DMAMUX Channels  */
#endif

#if defined(UM324xH)    
    DMAMUX_TypeDef         *DmamuxBaseAddress;                                /*!< Register base address                */

    uint32_t               DmamuxChannelSel;                                  /*!< DMAMUX Channels  */

    uint32_t               DmamuxReqGenSel;                                  /*!< DMAMUX Channels  */
#endif
} DMA_HandleTypeDef;

/**
  * @}
  */

/**
  * @brief  HAL DMA Callback ID structure definition
  */

/**
  * @brief  HAL DMA Callback ID enumeration definition
  */
typedef enum
{
    HAL_DMA_TFR_CB_ID            = 0x00U,    /*!< DMA_TFR Callback ID              */
    HAL_DMA_BLOCK_CB_ID          = 0x01U,    /*!< DMA_BLOCK Callback ID              */
    HAL_DMA_SRCTRAN_CB_ID        = 0x02U,    /*!< DMA_SRCTRAN Callback ID         */
    HAL_DMA_DSTTRAN_CB_ID        = 0x03U,    /*!< DMA_DSTTRAN Callback ID         */
    HAL_DMA_ERROR_CB_ID          = 0x04U,    /*!< DMA_ERROR Callback ID               */

} HAL_DMA_CallbackIDTypeDef;

/**
  * @brief  HAL DMA Callback pointer definition
  */
typedef void (*pDMA_CallbackTypeDef)(DMA_HandleTypeDef *hdma); /*!< pointer to an DMA callback function */

/* Exported types ------------------------------------------------------------*/
/** @defgroup DMAEx_Exported_Types DMAEx Exported Types
  * @{
  */

/**
  * @brief  HAL DMA Synchro definition
  */

/**
  * @brief  HAL DMAMUX Synchronization configuration structure definition
  */
typedef struct
{
    uint32_t SyncSignalID;  /*!< Specifies the synchronization signal gating the DMA request in periodic mode.
                              This parameter can be a value of @ref DMAEx_DMAMUX_SyncSignalID_selection */

    uint32_t SyncPolarity;  /*!< Specifies the polarity of the signal on which the DMA request is synchronized.
                              This parameter can be a value of @ref DMAEx_DMAMUX_SyncPolarity_selection */

    uint32_t SyncEnable;  /*!< Specifies if the synchronization shall be enabled or disabled
                                    This parameter can take the value ENABLE or DISABLE*/


    uint32_t EventEnable;    /*!< Specifies if an event shall be generated once the RequestNumber is reached.
                                       This parameter can take the value ENABLE or DISABLE */

    uint32_t RequestNumber; /*!< Specifies the number of DMA request that will be authorized after a sync event
                               This parameter must be a number between Min_Data = 1 and Max_Data = 32 */


} HAL_DMA_MuxSyncConfigTypeDef;

/**
  * @brief  HAL DMAMUX request generator parameters structure definition
  */
typedef struct
{
    uint32_t SignalID;      /*!< Specifies the ID of the signal used for DMAMUX request generator
                              This parameter can be a value of @ref DMAEx_DMAMUX_SignalGeneratorID_selection */

    uint32_t Polarity;       /*!< Specifies the polarity of the signal on which the request is generated.
                             This parameter can be a value of @ref DMAEx_DMAMUX_RequestGeneneratorPolarity_selection */

    uint32_t RequestNumber;  /*!< Specifies the number of DMA request that will be generated after a signal event
                                This parameter must be a number between Min_Data = 1 and Max_Data = 32 */
    uint32_t GeneratorEnable;    /*!< Specifies if an Generator shall be generated once the RequestNumber is reached.
                                       This parameter can take the value ENABLE or DISABLE */

} HAL_DMA_MuxRequestGeneratorConfigTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup DMA_Exported_Constants DMA Exported Constants
  * @{
  */

/** @defgroup DMA_Error_Code DMA Error Code
  * @{
  */
#define HAL_DMA_ERROR_NONE             0x00000000U    /*!< No error                              */
#define HAL_DMA_ERROR_TE               0x00000001U    /*!< Transfer error                        */
#define HAL_DMA_ERROR_NO_XFER          0x00000004U    /*!< Abort requested with no Xfer ongoing  */
#define HAL_DMA_ERROR_TIMEOUT          0x00000020U    /*!< Timeout error                         */
#define HAL_DMA_ERROR_NOT_SUPPORTED    0x00000100U    /*!< Not supported mode                    */
#define HAL_DMA_ERROR_SYNC             0x00000200U    /*!< DMAMUX sync overrun  error              */
#define HAL_DMA_ERROR_REQGEN           0x00000400U    /*!< DMAMUX request generator overrun  error */

/**
  * @}
  */

/** @defgroup DMA_request DMA request
  * @{
  */
#if defined(UM32x42x) ||defined(UM32x41x)
#if defined(UM32x42x)
#define DMA_REQUEST_DMAMUX_REQ_GEN0          0U
#define DMA_REQUEST_DMAMUX_REQ_GEN1          1U
#define DMA_REQUEST_DMAMUX_REQ_GEN2          2U
#define DMA_REQUEST_DMAMUX_REQ_GEN3          3U

#define DMA_REQUEST_SPI0TX              4U
#define DMA_REQUEST_SPI0RX              5U
#define DMA_REQUEST_SPI1TX              6U
#define DMA_REQUEST_SPI1RX              7U
#define DMA_REQUEST_SPI2TX              8U
#define DMA_REQUEST_SPI2RX              9U

#define DMA_REQUEST_UART2TX            10U
#define DMA_REQUEST_UART2RX            11U
#define DMA_REQUEST_DAC                12U
#define DMA_REQUEST_ADC1               13U
#define DMA_REQUEST_ADC2               14U

#define DMA_REQUEST_TIM1_CH1           15U
#define DMA_REQUEST_TIM1_CH2           16U
#define DMA_REQUEST_TIM1_CH3           17U
#define DMA_REQUEST_TIM1_CH4           18U
#define DMA_REQUEST_TIM1_TRIG          19U
#define DMA_REQUEST_TIM1_UP            20U

#define DMA_REQUEST_TIM8_CH1           21U
#define DMA_REQUEST_TIM8_CH2           22U
#define DMA_REQUEST_TIM8_CH3           23U
#define DMA_REQUEST_TIM8_CH4           24U
#define DMA_REQUEST_TIM8_TRIG          25U
#define DMA_REQUEST_TIM8_UP            26U

#define DMA_REQUEST_TIM2_CH1           27U
#define DMA_REQUEST_TIM2_CH2           28U
#define DMA_REQUEST_TIM2_CH3           29U
#define DMA_REQUEST_TIM2_CH4           30U
#define DMA_REQUEST_TIM2_TRIG          31U
#define DMA_REQUEST_TIM2_UP            32U

#define DMA_REQUEST_TIM3_CH1           33U
#define DMA_REQUEST_TIM3_CH2           34U
#define DMA_REQUEST_TIM3_CH3           35U
#define DMA_REQUEST_TIM3_CH4           36U
#define DMA_REQUEST_TIM3_TRIG          37U
#define DMA_REQUEST_TIM3_UP            38U

#define DMA_REQUEST_TIM4_CH1           39U
#define DMA_REQUEST_TIM4_CH2           40U
#define DMA_REQUEST_TIM4_CH3           41U
#define DMA_REQUEST_TIM4_CH4           42U
#define DMA_REQUEST_TIM4_TRIG          43U
#define DMA_REQUEST_TIM4_UP            44U

#define DMA_REQUEST_TIM5_CH1           45U
#define DMA_REQUEST_TIM5_CH2           46U
#define DMA_REQUEST_TIM5_CH3           47U
#define DMA_REQUEST_TIM5_CH4           48U
#define DMA_REQUEST_TIM5_TRIG          49U
#define DMA_REQUEST_TIM5_UP            50U

#define DMA_REQUEST_TIM6_UP            51U

#define DMA_REQUEST_USART7_TX          52U
#define DMA_REQUEST_USART7_RX          53U
#define DMA_REQUEST_USART8_TX          54U
#define DMA_REQUEST_USART8_RX          55U

#define DMA_REQUEST_I2C2TX             56U
#define DMA_REQUEST_I2C2RX             57U
#define DMA_REQUEST_I2C3TX             58U
#define DMA_REQUEST_I2C3RX             59U

#define DMA_REQUEST_TIM9_CH1           60U
#define DMA_REQUEST_TIM9_CH2           61U
#define DMA_REQUEST_TIM9_CH3           62U
#define DMA_REQUEST_TIM9_CH4           63U
#define DMA_REQUEST_TIM9_TRIG          64U
#define DMA_REQUEST_TIM9_UP            65U

#define DMA_REQUEST_TIM10_CH1           66U
#define DMA_REQUEST_TIM10_CH2           67U
#define DMA_REQUEST_TIM10_CH3           68U
#define DMA_REQUEST_TIM10_CH4           69U
#define DMA_REQUEST_TIM10_TRIG          70U
#define DMA_REQUEST_TIM10_UP            71U

#define DMA_REQUEST_TIM14              72U
#define DMA_REQUEST_TIM15              73U
#define DMA_REQUEST_TIM16              74U

#define DMA_REQUEST_CORDIC_IN          75U
#define DMA_REQUEST_CORDIC_OUT         76U

#define DMA_REQUEST_I2STX              77U
#define DMA_REQUEST_I2SRX              78U

#define DMA_REQUEST_CANTX              79U
#define DMA_REQUEST_CANRX              80U
#endif

#if defined(UM32x41x)
#define DMA_REQUEST_DMAMUX_REQ_GEN0          0U 
#define DMA_REQUEST_DMAMUX_REQ_GEN1          1U
#define DMA_REQUEST_DMAMUX_REQ_GEN2          2U
#define DMA_REQUEST_DMAMUX_REQ_GEN3          3U
        
#define DMA_REQUEST_I2C2TX              4U
#define DMA_REQUEST_I2C2RX              5U
#define DMA_REQUEST_I2C3TX              6U
#define DMA_REQUEST_I2C3RX              7U
#define DMA_REQUEST_SPI2TX              8U
#define DMA_REQUEST_SPI2RX              9U

#define DMA_REQUEST_UART2TX            10U
#define DMA_REQUEST_UART2RX            11U
#define DMA_REQUEST_TIM9                12U
#define DMA_REQUEST_ADC1               13U
#define DMA_REQUEST_TIM10               14U

#define DMA_REQUEST_TIM1_CH1           15U
#define DMA_REQUEST_TIM1_CH2           16U
#define DMA_REQUEST_TIM1_CH3           17U
#define DMA_REQUEST_TIM1_CH4           18U
#define DMA_REQUEST_TIM1_TRIG          19U
#define DMA_REQUEST_TIM1_UP            20U

#define DMA_REQUEST_TIM8_CH1           21U
#define DMA_REQUEST_TIM8_CH2           22U
#define DMA_REQUEST_TIM8_CH3           23U
#define DMA_REQUEST_TIM8_CH4           24U
#define DMA_REQUEST_TIM8_TRIG          25U
#define DMA_REQUEST_TIM8_UP            26U

#define DMA_REQUEST_TIM2_CH1           27U
#define DMA_REQUEST_TIM2_CH2           28U
#define DMA_REQUEST_TIM2_CH3           29U
#define DMA_REQUEST_TIM2_CH4           30U
#define DMA_REQUEST_TIM2_TRIG          31U
#define DMA_REQUEST_TIM2_UP            32U

#define DMA_REQUEST_TIM3_CH1           33U
#define DMA_REQUEST_TIM3_CH2           34U
#define DMA_REQUEST_TIM3_CH3           35U
#define DMA_REQUEST_TIM3_CH4           36U
#define DMA_REQUEST_TIM3_TRIG          37U
#define DMA_REQUEST_TIM3_UP            38U

#define DMA_REQUEST_TIM4_CH1           39U
#define DMA_REQUEST_TIM4_CH2           40U
#define DMA_REQUEST_TIM4_CH3           41U
#define DMA_REQUEST_TIM4_CH4           42U
#define DMA_REQUEST_TIM4_TRIG          43U
#define DMA_REQUEST_TIM4_UP            44U

#define DMA_REQUEST_TIM5_CH1           45U
#define DMA_REQUEST_TIM5_CH2           46U
#define DMA_REQUEST_TIM5_CH3           47U
#define DMA_REQUEST_TIM5_CH4           48U
#define DMA_REQUEST_TIM5_TRIG          49U
#define DMA_REQUEST_TIM5_UP            50U

#define DMA_REQUEST_USART7_TX          51U
#define DMA_REQUEST_USART7_RX          52U


#define DMA_REQUEST_CORDIC_IN          53U
#define DMA_REQUEST_CORDIC_OUT         54U
#endif
/**
  * @}
  */

/** @defgroup DMAEx_DMAMUX_SyncSignalID_selection DMAMUX SyncSignalID selection
  * @{
  */
#define DMAMUX_SYNC_EXTI0                      0U     /*!<  Synchronization Signal is EXTI0  IT   */
#define DMAMUX_SYNC_EXTI1                      1U     /*!<  Synchronization Signal is EXTI1  IT   */
#define DMAMUX_SYNC_EXTI2                      2U     /*!<  Synchronization Signal is EXTI2  IT   */
#define DMAMUX_SYNC_EXTI3                      3U     /*!<  Synchronization Signal is EXTI3  IT   */
#define DMAMUX_SYNC_EXTI4                      4U     /*!<  Synchronization Signal is EXTI4  IT   */
#define DMAMUX_SYNC_EXTI5                      5U     /*!<  Synchronization Signal is EXTI5  IT   */
#define DMAMUX_SYNC_EXTI6                      6U     /*!<  Synchronization Signal is EXTI6  IT   */
#define DMAMUX_SYNC_EXTI7                      7U     /*!<  Synchronization Signal is EXTI7  IT   */
#define DMAMUX_SYNC_EXTI8                      8U     /*!<  Synchronization Signal is EXTI8  IT   */
#define DMAMUX_SYNC_EXTI9                      9U     /*!<  Synchronization Signal is EXTI9  IT   */
#define DMAMUX_SYNC_EXTI10                    10U     /*!<  Synchronization Signal is EXTI10 IT   */
#define DMAMUX_SYNC_EXTI11                    11U     /*!<  Synchronization Signal is EXTI11 IT   */
#define DMAMUX_SYNC_EXTI12                    12U     /*!<  Synchronization Signal is EXTI12 IT   */
#define DMAMUX_SYNC_EXTI13                    13U     /*!<  Synchronization Signal is EXTI13 IT   */
#define DMAMUX_SYNC_EXTI14                    14U     /*!<  Synchronization Signal is EXTI14 IT   */
#define DMAMUX_SYNC_EXTI15                    15U     /*!<  Synchronization Signal is EXTI15 IT   */
#define DMAMUX_SYNC_DMAMUX1_CH0_EVT           16U     /*!<  Synchronization Signal is DMAMUX1 Channel0 Event  */
#define DMAMUX_SYNC_DMAMUX1_CH1_EVT           17U     /*!<  Synchronization Signal is DMAMUX1 Channel1 Event  */
#define DMAMUX_SYNC_DMAMUX1_CH2_EVT           18U     /*!<  Synchronization Signal is DMAMUX1 Channel2 Event  */
#define DMAMUX_SYNC_DMAMUX1_CH3_EVT           19U     /*!<  Synchronization Signal is DMAMUX1 Channel3 Event  */
#define DMAMUX_SYNC_LPTIM1_OUT                20U     /*!<  Synchronization Signal is LPTIM1 OUT */
#define DMAMUX_SYNC_LPTIM1_OUT                21U     /*!<  Synchronization Signal is LPTIM1 OUT */
#define DMAMUX_SYNC_TIM6_TRGO                 22U     /*!<  Synchronization Signal is TIM6 TRGO */
#define DMAMUX_SYNC_TIM14_OUT                 23U     /*!<  Synchronization Signal is TIM14 OUT */
#define DMAMUX_SYNC_TIM15_OUT                 24U     /*!<  Synchronization Signal is TIM15 OUT */
#define DMAMUX_SYNC_TIM16_OUT                 25U     /*!<  Synchronization Signal is TIM16 OUT */

/**
  * @}
  */

/** @defgroup DMAEx_DMAMUX_SyncPolarity_selection DMAMUX SyncPolarity selection
  * @{
  */
#define HAL_DMAMUX_SYNC_NO_EVENT                        0U    /*!< block synchronization events        */
#define HAL_DMAMUX_SYNC_RISING                          ((uint32_t)DMAMUX_RG0CR_GPOL_0)    /*!< synchronize with rising edge events */
#define HAL_DMAMUX_SYNC_FALLING                         ((uint32_t)DMAMUX_RG0CR_GPOL_1)    /*!< synchronize with falling edge events */
#define HAL_DMAMUX_SYNC_RISING_FALLING                  ((uint32_t)DMAMUX_RG0CR_GPOL)  /*!< synchronize with rising and falling edge events */

/**
  * @}
  */

/** @defgroup DMAEx_DMAMUX_SignalGeneratorID_selection DMAMUX SignalGeneratorID selection
  * @{
  */
#define DMAMUX_REQ_GEN_EXTI0                      0U        /*!< Request generator Signal is EXTI0 IT    */
#define DMAMUX_REQ_GEN_EXTI1                      1U        /*!< Request generator Signal is EXTI1 IT    */
#define DMAMUX_REQ_GEN_EXTI2                      2U        /*!< Request generator Signal is EXTI2 IT    */
#define DMAMUX_REQ_GEN_EXTI3                      3U        /*!< Request generator Signal is EXTI3 IT    */
#define DMAMUX_REQ_GEN_EXTI4                      4U        /*!< Request generator Signal is EXTI4 IT    */
#define DMAMUX_REQ_GEN_EXTI5                      5U        /*!< Request generator Signal is EXTI5 IT    */
#define DMAMUX_REQ_GEN_EXTI6                      6U        /*!< Request generator Signal is EXTI6 IT    */
#define DMAMUX_REQ_GEN_EXTI7                      7U        /*!< Request generator Signal is EXTI7 IT    */
#define DMAMUX_REQ_GEN_EXTI8                      8U        /*!< Request generator Signal is EXTI8 IT    */
#define DMAMUX_REQ_GEN_EXTI9                      9U        /*!< Request generator Signal is EXTI9 IT    */
#define DMAMUX_REQ_GEN_EXTI10                    10U        /*!< Request generator Signal is EXTI10 IT   */
#define DMAMUX_REQ_GEN_EXTI11                    11U        /*!< Request generator Signal is EXTI11 IT   */
#define DMAMUX_REQ_GEN_EXTI12                    12U        /*!< Request generator Signal is EXTI12 IT   */
#define DMAMUX_REQ_GEN_EXTI13                    13U        /*!< Request generator Signal is EXTI13 IT   */
#define DMAMUX_REQ_GEN_EXTI14                    14U        /*!< Request generator Signal is EXTI14 IT   */
#define DMAMUX_REQ_GEN_EXTI15                    15U        /*!< Request generator Signal is EXTI15 IT   */
#define DMAMUX_REQ_GEN_DMAMUX1_CH0_EVT           16U        /*!< Request generator Signal is DMAMUX1 Channel0 Event */
#define DMAMUX_REQ_GEN_DMAMUX1_CH1_EVT           17U        /*!< Request generator Signal is DMAMUX1 Channel1 Event */
#define DMAMUX_REQ_GEN_DMAMUX1_CH2_EVT           18U        /*!< Request generator Signal is DMAMUX1 Channel2 Event */
#define DMAMUX_REQ_GEN_DMAMUX1_CH3_EVT           19U        /*!< Request generator Signal is DMAMUX1 Channel3 Event */
#define DMAMUX_REQ_GEN_LPTIM0_OUT                20U        /*!< Request generator Signal is LPTIM0 OUT  */
#define DMAMUX_REQ_GEN_LPTIM1_OUT                21U        /*!< Request generator Signal is LPTIM1 OUT */
#define DMAMUX_REQ_GEN_TIM6_TRGO                 22U        /*!< Request generator Signal is TIM6 TRGO */
#define DMAMUX_REQ_GEN_TIM14_OUT                 23U        /*!< Request generator Signal is TIM14 OUT */
#define DMAMUX_REQ_GEN_TIM15_OUT                 24U        /*!< Request generator Signal is TIM15 OUT */
#define DMAMUX_REQ_GEN_TIM16_OUT                 25U        /*!< Request generator Signal is TIM16 OUT */

/**
  * @}
  */

/** @defgroup DMAEx_DMAMUX_RequestGeneneratorPolarity_selection DMAMUX RequestGeneneratorPolarity selection
  * @{
  */
#define HAL_DMAMUX_REQ_GEN_NO_EVENT         0x00000000U           /*!< block request generator events        */
#define HAL_DMAMUX_REQ_GEN_RISING           DMAMUX_RG0CR_GPOL_0   /*!< generate request on rising edge events */
#define HAL_DMAMUX_REQ_GEN_FALLING          DMAMUX_RG0CR_GPOL_1   /*!< generate request on falling edge events */
#define HAL_DMAMUX_REQ_GEN_RISING_FALLING   DMAMUX_RG0CR_GPOL     /*!< generate request on rising and falling edge events */

/**
  * @}
  */
  
#endif

#if defined(UM324xH)
#define DMA_REQUEST_DMAMUX_REQ_GEN0          0U
#define DMA_REQUEST_DMAMUX_REQ_GEN1          1U
#define DMA_REQUEST_DMAMUX_REQ_GEN2          2U
#define DMA_REQUEST_DMAMUX_REQ_GEN3          3U

#define DMA_REQUEST_SPI0TX              4U
#define DMA_REQUEST_SPI0RX              5U
#define DMA_REQUEST_SPI1TX              6U
#define DMA_REQUEST_SPI1RX              7U
#define DMA_REQUEST_SPI2TX              8U
#define DMA_REQUEST_SPI2RX              9U

#define DMA_REQUEST_UART2TX            10U
#define DMA_REQUEST_UART2RX            11U
#define DMA_REQUEST_DAC                12U
#define DMA_REQUEST_ADC1               13U
#define DMA_REQUEST_ADC2               14U

#define DMA_REQUEST_TIM1_CH1           15U
#define DMA_REQUEST_TIM1_CH2           16U
#define DMA_REQUEST_TIM1_CH3           17U
#define DMA_REQUEST_TIM1_CH4           18U
#define DMA_REQUEST_TIM1_TRIG          19U
#define DMA_REQUEST_TIM1_UP            20U

#define DMA_REQUEST_TIM8_CH1           21U
#define DMA_REQUEST_TIM8_CH2           22U
#define DMA_REQUEST_TIM8_CH3           23U
#define DMA_REQUEST_TIM8_CH4           24U
#define DMA_REQUEST_TIM8_TRIG          25U
#define DMA_REQUEST_TIM8_UP            26U

#define DMA_REQUEST_TIM2_CH1           27U
#define DMA_REQUEST_TIM2_CH2           28U
#define DMA_REQUEST_TIM2_CH3           29U
#define DMA_REQUEST_TIM2_CH4           30U
#define DMA_REQUEST_TIM2_TRIG          31U
#define DMA_REQUEST_TIM2_UP            32U

#define DMA_REQUEST_TIM3_CH1           33U
#define DMA_REQUEST_TIM3_CH2           34U
#define DMA_REQUEST_TIM3_CH3           35U
#define DMA_REQUEST_TIM3_CH4           36U
#define DMA_REQUEST_TIM3_TRIG          37U
#define DMA_REQUEST_TIM3_UP            38U

#define DMA_REQUEST_TIM4_CH1           39U
#define DMA_REQUEST_TIM4_CH2           40U
#define DMA_REQUEST_TIM4_CH3           41U
#define DMA_REQUEST_TIM4_CH4           42U
#define DMA_REQUEST_TIM4_TRIG          43U
#define DMA_REQUEST_TIM4_UP            44U

#define DMA_REQUEST_TIM5_CH1           45U
#define DMA_REQUEST_TIM5_CH2           46U
#define DMA_REQUEST_TIM5_CH3           47U
#define DMA_REQUEST_TIM5_CH4           48U
#define DMA_REQUEST_TIM5_TRIG          49U
#define DMA_REQUEST_TIM5_UP            50U

#define DMA_REQUEST_TIM6_UP            51U

#define DMA_REQUEST_USART7_TX          52U
#define DMA_REQUEST_USART7_RX          53U
#define DMA_REQUEST_USART8_TX          54U
#define DMA_REQUEST_USART8_RX          55U

#define DMA_REQUEST_I2C2TX             56U
#define DMA_REQUEST_I2C2RX             57U
#define DMA_REQUEST_I2C3TX             58U
#define DMA_REQUEST_I2C3RX             59U

#define DMA_REQUEST_TIM9_CH1           60U
#define DMA_REQUEST_TIM9_CH2           61U
#define DMA_REQUEST_TIM9_CH3           62U
#define DMA_REQUEST_TIM9_CH4           63U
#define DMA_REQUEST_TIM9_TRIG          64U
#define DMA_REQUEST_TIM9_UP            65U

#define DMA_REQUEST_TIM10_CH1           66U
#define DMA_REQUEST_TIM10_CH2           67U
#define DMA_REQUEST_TIM10_CH3           68U
#define DMA_REQUEST_TIM10_CH4           69U
#define DMA_REQUEST_TIM10_TRIG          70U
#define DMA_REQUEST_TIM10_UP            71U

#define DMA_REQUEST_TIM14              72U
#define DMA_REQUEST_TIM15              73U
#define DMA_REQUEST_TIM16              74U

#define DMA_REQUEST_CORDIC_IN          75U
#define DMA_REQUEST_CORDIC_OUT         76U

#define DMA_REQUEST_I2STX              77U
#define DMA_REQUEST_I2SRX              78U

#define DMA_REQUEST_CANTX              79U
#define DMA_REQUEST_CANRX              80U

/**
  * @}
  */

/** @defgroup DMAEx_DMAMUX_SyncSignalID_selection DMAMUX SyncSignalID selection
  * @{
  */
#define DMAMUX_SYNC_EXTI0                      0U     /*!<  Synchronization Signal is EXTI0  IT   */
#define DMAMUX_SYNC_EXTI1                      1U     /*!<  Synchronization Signal is EXTI1  IT   */
#define DMAMUX_SYNC_EXTI2                      2U     /*!<  Synchronization Signal is EXTI2  IT   */
#define DMAMUX_SYNC_EXTI3                      3U     /*!<  Synchronization Signal is EXTI3  IT   */
#define DMAMUX_SYNC_EXTI4                      4U     /*!<  Synchronization Signal is EXTI4  IT   */
#define DMAMUX_SYNC_EXTI5                      5U     /*!<  Synchronization Signal is EXTI5  IT   */
#define DMAMUX_SYNC_EXTI6                      6U     /*!<  Synchronization Signal is EXTI6  IT   */
#define DMAMUX_SYNC_EXTI7                      7U     /*!<  Synchronization Signal is EXTI7  IT   */
#define DMAMUX_SYNC_EXTI8                      8U     /*!<  Synchronization Signal is EXTI8  IT   */
#define DMAMUX_SYNC_EXTI9                      9U     /*!<  Synchronization Signal is EXTI9  IT   */
#define DMAMUX_SYNC_EXTI10                    10U     /*!<  Synchronization Signal is EXTI10 IT   */
#define DMAMUX_SYNC_EXTI11                    11U     /*!<  Synchronization Signal is EXTI11 IT   */
#define DMAMUX_SYNC_EXTI12                    12U     /*!<  Synchronization Signal is EXTI12 IT   */
#define DMAMUX_SYNC_EXTI13                    13U     /*!<  Synchronization Signal is EXTI13 IT   */
#define DMAMUX_SYNC_EXTI14                    14U     /*!<  Synchronization Signal is EXTI14 IT   */
#define DMAMUX_SYNC_EXTI15                    15U     /*!<  Synchronization Signal is EXTI15 IT   */
#define DMAMUX_SYNC_DMAMUX1_CH0_EVT           16U     /*!<  Synchronization Signal is DMAMUX1 Channel0 Event  */
#define DMAMUX_SYNC_DMAMUX1_CH1_EVT           17U     /*!<  Synchronization Signal is DMAMUX1 Channel1 Event  */
#define DMAMUX_SYNC_DMAMUX1_CH2_EVT           18U     /*!<  Synchronization Signal is DMAMUX1 Channel2 Event  */
#define DMAMUX_SYNC_DMAMUX1_CH3_EVT           19U     /*!<  Synchronization Signal is DMAMUX1 Channel3 Event  */
#define DMAMUX_SYNC_LPTIM0_OUT                20U     /*!<  Synchronization Signal is LPTIM0 OUT */
#define DMAMUX_SYNC_LPTIM1_OUT                21U     /*!<  Synchronization Signal is LPTIM1 OUT */
#define DMAMUX_SYNC_TIM6_TRGO                 22U     /*!<  Synchronization Signal is TIM6 TRGO */
#define DMAMUX_SYNC_TIM14_OUT                 23U     /*!<  Synchronization Signal is TIM14 OUT */
#define DMAMUX_SYNC_TIM15_OUT                 24U     /*!<  Synchronization Signal is TIM15 OUT */
#define DMAMUX_SYNC_TIM16_OUT                 25U     /*!<  Synchronization Signal is TIM16 OUT */

/**
  * @}
  */

/** @defgroup DMAEx_DMAMUX_SyncPolarity_selection DMAMUX SyncPolarity selection
  * @{
  */
#define HAL_DMAMUX_SYNC_NO_EVENT                        0U    /*!< block synchronization events        */
#define HAL_DMAMUX_SYNC_RISING                          ((uint32_t)DMAMUX_RG0CR_GPOL_0)    /*!< synchronize with rising edge events */
#define HAL_DMAMUX_SYNC_FALLING                         ((uint32_t)DMAMUX_RG0CR_GPOL_1)    /*!< synchronize with falling edge events */
#define HAL_DMAMUX_SYNC_RISING_FALLING                  ((uint32_t)DMAMUX_RG0CR_GPOL)  /*!< synchronize with rising and falling edge events */

/**
  * @}
  */

/** @defgroup DMAEx_DMAMUX_SignalGeneratorID_selection DMAMUX SignalGeneratorID selection
  * @{
  */
#define DMAMUX_REQ_GEN_EXTI0                      0U        /*!< Request generator Signal is EXTI0 IT    */
#define DMAMUX_REQ_GEN_EXTI1                      1U        /*!< Request generator Signal is EXTI1 IT    */
#define DMAMUX_REQ_GEN_EXTI2                      2U        /*!< Request generator Signal is EXTI2 IT    */
#define DMAMUX_REQ_GEN_EXTI3                      3U        /*!< Request generator Signal is EXTI3 IT    */
#define DMAMUX_REQ_GEN_EXTI4                      4U        /*!< Request generator Signal is EXTI4 IT    */
#define DMAMUX_REQ_GEN_EXTI5                      5U        /*!< Request generator Signal is EXTI5 IT    */
#define DMAMUX_REQ_GEN_EXTI6                      6U        /*!< Request generator Signal is EXTI6 IT    */
#define DMAMUX_REQ_GEN_EXTI7                      7U        /*!< Request generator Signal is EXTI7 IT    */
#define DMAMUX_REQ_GEN_EXTI8                      8U        /*!< Request generator Signal is EXTI8 IT    */
#define DMAMUX_REQ_GEN_EXTI9                      9U        /*!< Request generator Signal is EXTI9 IT    */
#define DMAMUX_REQ_GEN_EXTI10                    10U        /*!< Request generator Signal is EXTI10 IT   */
#define DMAMUX_REQ_GEN_EXTI11                    11U        /*!< Request generator Signal is EXTI11 IT   */
#define DMAMUX_REQ_GEN_EXTI12                    12U        /*!< Request generator Signal is EXTI12 IT   */
#define DMAMUX_REQ_GEN_EXTI13                    13U        /*!< Request generator Signal is EXTI13 IT   */
#define DMAMUX_REQ_GEN_EXTI14                    14U        /*!< Request generator Signal is EXTI14 IT   */
#define DMAMUX_REQ_GEN_EXTI15                    15U        /*!< Request generator Signal is EXTI15 IT   */
#define DMAMUX_REQ_GEN_DMAMUX1_CH0_EVT           16U        /*!< Request generator Signal is DMAMUX1 Channel0 Event */
#define DMAMUX_REQ_GEN_DMAMUX1_CH1_EVT           17U        /*!< Request generator Signal is DMAMUX1 Channel1 Event */
#define DMAMUX_REQ_GEN_DMAMUX1_CH2_EVT           18U        /*!< Request generator Signal is DMAMUX1 Channel2 Event */
#define DMAMUX_REQ_GEN_DMAMUX1_CH3_EVT           19U        /*!< Request generator Signal is DMAMUX1 Channel3 Event */
#define DMAMUX_REQ_GEN_LPTIM0_OUT                20U        /*!< Request generator Signal is LPTIM0 OUT  */
#define DMAMUX_REQ_GEN_LPTIM1_OUT                21U        /*!< Request generator Signal is LPTIM1 OUT */
#define DMAMUX_REQ_GEN_TIM6_TRGO                 22U        /*!< Request generator Signal is TIM6 TRGO */
#define DMAMUX_REQ_GEN_TIM14_OUT                 23U        /*!< Request generator Signal is TIM14 OUT */
#define DMAMUX_REQ_GEN_TIM15_OUT                 24U        /*!< Request generator Signal is TIM15 OUT */
#define DMAMUX_REQ_GEN_TIM16_OUT                 25U        /*!< Request generator Signal is TIM16 OUT */

/**
  * @}
  */

/** @defgroup DMAEx_DMAMUX_RequestGeneneratorPolarity_selection DMAMUX RequestGeneneratorPolarity selection
  * @{
  */
#define HAL_DMAMUX_REQ_GEN_NO_EVENT         0x00000000U           /*!< block request generator events        */
#define HAL_DMAMUX_REQ_GEN_RISING           DMAMUX_RG0CR_GPOL_0   /*!< generate request on rising edge events */
#define HAL_DMAMUX_REQ_GEN_FALLING          DMAMUX_RG0CR_GPOL_1   /*!< generate request on falling edge events */
#define HAL_DMAMUX_REQ_GEN_RISING_FALLING   DMAMUX_RG0CR_GPOL     /*!< generate request on rising and falling edge events */

/**
  * @}
  */
#endif


/** @defgroup DMA_Data_transfer_direction DMA Data transfer direction
  * @{
  */
#define DMA_MEMORY_TO_MEMORY                0U                                                  /*!< memory to memory transfer   */
#define DMA_MEMORY_TO_PERIPH                DMA_CTL0_TT_FC_0                                   /*!< Peripheral to memory direction */
#define DMA_PERIPH_TO_MEMORY                DMA_CTL0_TT_FC_1                                   /*!< Memory to peripheral direction */
#define DMA_PERIPH_TO_PERIPH                (DMA_CTL0_TT_FC_0 | DMA_CTL0_TT_FC_1)              /*!< Memory to memory direction     */

/**
  * @}
  */

/** @defgroup DMA register channel offset
  * @{
  */
#define CHANNEL_OFFSET  0x16


/**
  * @}
  */

/** @defgroup DMA_destination transmission Burst length
  * @{
  */
#define DMA_BURST_DST_NUM_1         0x00000000U               /*!< DMA_destination transmission Burst length is 1BYTE */
#define DMA_BURST_DST_NUM_4         DMA_CTL0_DST_MSIZE_0      /*!< DMA_destination transmission Burst length is 4BYTE */
#define DMA_BURST_DST_NUM_8         DMA_CTL0_DST_MSIZE_1      /*!< DMA_destination transmission Burst length is 8BYTE */
#define DMA_BURST_DST_NUM_16        (DMA_CTL0_DST_MSIZE_0 \
                                     | DMA_CTL0_DST_MSIZE_1)  /*!< DMA_destination transmission Burst length is 16BYTE */

/**
  * @}
  */

/** @defgroup DMA_Source transmission Burst length
  * @{
  */
#define DMA_BURST_SRC_NUM_1         0x00000000U                 /*!< DMA_Source transmission Burst length is 1BYTE */
#define DMA_BURST_SRC_NUM_4         DMA_CTL0_SRC_MSIZE_0        /*!< DMA_Source transmission Burst length is 4BYTE */
#define DMA_BURST_SRC_NUM_8         DMA_CTL0_SRC_MSIZE_1        /*!< DMA_Source transmission Burst length is 8BYTE */
#define DMA_BURST_SRC_NUM_16        (DMA_CTL0_SRC_MSIZE_0 \
                                    | DMA_CTL0_SRC_MSIZE_1)     /*!< DMA_Source transmission Burst length is 16BYTE */

/**
  * @}
  */

/** @defgroup DMA_destination_incremented_mode DMA Peripheral incremented mode
  * @{
  */
#define DMA_DSTINC_INC              0x00000000U                     /*!< destination address increment */
#define DMA_DSTINC_DEC              DMA_CTL0_DINC_0                 /*!< destination address decrement */
#define DMA_DSTINC_NOC              DMA_CTL0_DINC_1                 /*!< destination address remains unchanged*/
/**
  * @}
  */

/** @defgroup DMA_source_incremented_mode DMA Memory incremented mode
  * @{
  */
#define DMA_SRCINC_INC              0x00000000U                   /*!< Source address increment  */
#define DMA_SRCINC_DEC              DMA_CTL0_SINC_0               /*!< Source address decrement */
#define DMA_SRCINC_NOC              DMA_CTL0_SINC_1               /*!< Source address remains unchanged*/
/**
  * @}
  */

/** @defgroup DMA_source_data_size DMA Memory data size
  * @{
  */
#define DMA_SRCDATAALIGN_BYTE          0x00000000U                  /*!< source data alignment : Byte     */
#define DMA_SRCDATAALIGN_HALFWORD      DMA_CTL0_SRC_TR_WIDTH_0              /*!< source data alignment : HalfWord */
#define DMA_SRCDATAALIGN_WORD          DMA_CTL0_SRC_TR_WIDTH_1              /*!< source data alignment : Word     */
/**
  * @}
  */

/** @defgroup DMA_destination_data_size DMA Memory data size
  * @{
  */
#define DMA_DSTDATAALIGN_BYTE          0x00000000U                          /*!< destination data alignment : Byte     */
#define DMA_DSTDATAALIGN_HALFWORD      DMA_CTL0_DST_TR_WIDTH_0              /*!< destination data alignment : HalfWord */
#define DMA_DSTDATAALIGN_WORD          DMA_CTL0_DST_TR_WIDTH_1              /*!< destination data alignment : Word     */
/**
  * @}
  */

/** @defgroup DMA FIFO mode
  * @{
  */
#define DMA_FIFOMODE_DISABLE         0x00000000U               /*!< FIFO Mode disable */
#define DMA_FIFOMODE_ENABLE         DMA_CFGH0_FIFO_MODE       /*!< FIFO Mode disable */

/**
  * @}
  */

/** @defgroup DMA FC mode
  * @{
  */

#define DMA_FCMODE_DISABLE         DMA_CFGH0_FCMODE                 /*!< FC Mode disable */
#define DMA_FCMODE_ENABLE         0x00000000U                       /*!< FC Mode disable */
/**
  * @}
  */

/** @defgroup DMA hardware/software handshaking
  * @{
  */
#define DMA_SRC_HS_SW           DMA_CFG0_HS_SEL_SRC     /*!< Source transmission software handshake*/
#define DMA_SRC_HS_HW           0x00000000U             /*!< Source transmission hardware handshake */

#define DMA_DST_HS_SW           DMA_CFG0_HS_SEL_DST    /*!< destination transmission software handshake*/
#define DMA_DST_HS_HW           0x00000000U            /*!< destination transmission hardware handshake */
/**
  * @}
  */

/** @defgroup DMA Source/destination reload enable
  * @{
  */
#define DMA_SRC_RELOAD_DISABLE           0x00000000U     /*!< Source transmission software handshake*/
#define DMA_SRC_RELOAD_ENABLE            DMA_CFG0_RELOAD_SRC             /*!< Source transmission hardware handshake */

#define DMA_DST_RELOAD_DISABLE           0x00000000U    /*!< destination transmission software handshake*/
#define DMA_DST_RELOAD_ENABLE            DMA_CFG0_RELOAD_DST            /*!< destination transmission hardware handshake */
/**
  * @}
  */

#if defined(UM32x42x) ||defined(UM32x41x)
/** @defgroup DMA_DEST_PER_SIGNAL
  * @{
   */
#define DMA_DST_PER_SIGNAL0                0x00000000U
#define DMA_DST_PER_SIGNAL1                (DMA_CFGH0_DEST_PER_0 )
#define DMA_DST_PER_SIGNAL2                (DMA_CFGH0_DEST_PER_1 )
#define DMA_DST_PER_SIGNAL3                (DMA_CFGH0_DEST_PER_0 | DMA_CFGH0_DEST_PER_1)
#define DMA_DST_PER_SIGNAL4                (DMA_CFGH0_DEST_PER_2 )
#define DMA_DST_PER_SIGNAL5                (DMA_CFGH0_DEST_PER_0 | DMA_CFGH0_DEST_PER_2)
#define DMA_DST_PER_SIGNAL6                (DMA_CFGH0_DEST_PER_1 | DMA_CFGH0_DEST_PER_2)
#define DMA_DST_PER_SIGNAL7                (DMA_CFGH0_DEST_PER_0 |DMA_CFGH0_DEST_PER_1 | DMA_CFGH0_DEST_PER_2)
/**
  * @}
  */

/** @defgroup DMA_SRC_PER_SIGNAL
  * @{
   */

#define DMA_SRC_PER_SIGNAL0                 0x00000000U
#define DMA_SRC_PER_SIGNAL1                 (DMA_CFGH0_SRC_PER_0 )
#define DMA_SRC_PER_SIGNAL2                 (DMA_CFGH0_SRC_PER_1 )
#define DMA_SRC_PER_SIGNAL3                 (DMA_CFGH0_SRC_PER_0 | DMA_CFGH0_SRC_PER_1)
#define DMA_SRC_PER_SIGNAL4                 (DMA_CFGH0_SRC_PER_2 )
#define DMA_SRC_PER_SIGNAL5                 (DMA_CFGH0_SRC_PER_0 | DMA_CFGH0_SRC_PER_2)
#define DMA_SRC_PER_SIGNAL6                 (DMA_CFGH0_SRC_PER_1 | DMA_CFGH0_SRC_PER_2)
#define DMA_SRC_PER_SIGNAL7                 (DMA_CFGH0_SRC_PER_0 |DMA_CFGH0_SRC_PER_1 | DMA_CFGH0_SRC_PER_2)

/**
  * @}
  */
#endif
 #if defined(UM324xF)
/** @defgroup DEST_PER src/dst per number
  * @brief    src/dst per number
  * @{ 
  */
/*************DMA0 DEST_PER**************/      //Destination handshaking number
#define DMA0_DEST_PER_HANDSHAKING_NULL          0x00000000U
#define DMA0_DEST_PER_HANDSHAKING_SIGNAL0       ((uint32_t)(0<<11))		//handshake signal 0��SPI2_RX��I2C1_RX��TIM4_CH1��I2S1_RX��UART5_RX��TIM5_CH3��TIM5_UP
#define DMA0_DEST_PER_HANDSHAKING_SIGNAL1       ((uint32_t)(1<<11))		//handshake signal 1��TIM2_UP��TIM2_CH3��UART3_RX��TIM5_CH4��TIM5_TRIG��TIM6_UP
#define DMA0_DEST_PER_HANDSHAKING_SIGNAL2       ((uint32_t)(2<<11))		//handshake signal 2��SPI2_RX��TIM7_UP��I2S1_RX��I2C3_RX��UART4_RX��TIM3_CH4��TIM3_UP��TIM5_CH1��I2C2_RX
#define DMA0_DEST_PER_HANDSHAKING_SIGNAL3       ((uint32_t)(3<<11))		//handshake signal 3��SPI1_RX��TIM4_CH2��I2S0_RX��UART3_TX��TIM5_TRIG��TIM5_CH4��I2C2_RX
#define DMA0_DEST_PER_HANDSHAKING_SIGNAL4       ((uint32_t)(4<<11))		//handshake signal 4��SPI1_TX��TIM7_UP��I2S0_TX��I2C3_TX��UART4_TX��TIM3_TRIG��TIM3_CH1��TIM5_CH2��UART3_TX
#define DMA0_DEST_PER_HANDSHAKING_SIGNAL5       ((uint32_t)(5<<11))		//handshake signal 5��SPI2_TX��I2C1_RX��I2S1_TX��TIM2_CH1��UART2_RX��TIM3_CH2��DAC0
#define DMA0_DEST_PER_HANDSHAKING_SIGNAL6       ((uint32_t)(6<<11))		//handshake signal 6��I2C1_TX��TIM4_UP��TIM2_CH2��TIM2_CH4��UART2_TX��TIM5_UP��DAC1
#define DMA0_DEST_PER_HANDSHAKING_SIGNAL7       ((uint32_t)(7<<11))		//handshake signal 7��SPI2_TX��I2C1_RX��TIM4_CH3��TIM2_CH4��TIM2_UP��UART5_TX��TIM3_CH3��I2C2_TX
#define DMA0_DEST_PER_HANDSHAKING_SIGNAL8       ((uint32_t)(8<<11))		//handshake signal 8��QSPI_RX��TIM12_CH2��TIM13_CH1��TIM14_TRIG��TIM4_CH4
#define DMA0_DEST_PER_HANDSHAKING_SIGNAL9       ((uint32_t)(9<<11))		//handshake signal 9��QSPI_TX��TIM12_CH3��TIM13_CH2��TIM14_CH1��TIM4_TRIG
#define DMA0_DEST_PER_HANDSHAKING_SIGNAL10      ((uint32_t)(10<<11))	//handshake signal 10��TIM12_CH4��TIM13_CH3��TIM14_CH2��USART7_RX��TIM1_TRIG
#define DMA0_DEST_PER_HANDSHAKING_SIGNAL11      ((uint32_t)(11<<11))	//handshake signal 11��TIM12_UP��TIM13_CH4��TIM14_CH3��USART7_TX
#define DMA0_DEST_PER_HANDSHAKING_SIGNAL12      ((uint32_t)(12<<11))	//handshake signal 12��TIM12_TRIG��TIM13_UP��TIM14_CH4
#define DMA0_DEST_PER_HANDSHAKING_SIGNAL13      ((uint32_t)(13<<11))	//handshake signal 13��TIM12_CH1��TIM13_TRIG��TIM14_UP


/*************DMA0 SRC_PER**************/       //Source Handshaking number
#define DMA0_SRC_PER_HANDSHAKING_NULL           0x00000000U
#define DMA0_SRC_PER_HANDSHAKING_SIGNAL0        ((uint32_t)(0<<7))		//handshake signal 0��SPI2_RX��I2C1_RX��TIM4_CH1��I2S1_RX��UART5_RX��TIM5_CH3��TIM5_UP
#define DMA0_SRC_PER_HANDSHAKING_SIGNAL1        ((uint32_t)(1<<7))		//handshake signal 1��TIM2_UP��TIM2_CH3��UART3_RX��TIM5_CH4��TIM5_TRIG��TIM6_UP
#define DMA0_SRC_PER_HANDSHAKING_SIGNAL2        ((uint32_t)(2<<7))		//handshake signal 2��SPI2_RX��TIM7_UP��I2S1_RX��I2C3_RX��UART4_RX��TIM3_CH4��TIM3_UP��TIM5_CH1��I2C2_RX
#define DMA0_SRC_PER_HANDSHAKING_SIGNAL3        ((uint32_t)(3<<7))		//handshake signal 3��SPI1_RX��TIM4_CH2��I2S0_RX��UART3_TX��TIM5_TRIG��TIM5_CH4��I2C2_RX
#define DMA0_SRC_PER_HANDSHAKING_SIGNAL4        ((uint32_t)(4<<7))		//handshake signal 4��SPI1_TX��TIM7_UP��I2S0_TX��I2C3_TX��UART4_TX��TIM3_TRIG��TIM3_CH1��TIM5_CH2��UART3_TX
#define DMA0_SRC_PER_HANDSHAKING_SIGNAL5        ((uint32_t)(5<<7))		//handshake signal 5��SPI2_TX��I2C1_RX��I2S1_TX��TIM2_CH1��UART2_RX��TIM3_CH2��DAC0
#define DMA0_SRC_PER_HANDSHAKING_SIGNAL6        ((uint32_t)(6<<7))		//handshake signal 6��I2C1_TX��TIM4_UP��TIM2_CH2��TIM2_CH4��UART2_TX��TIM5_UP��DAC1
#define DMA0_SRC_PER_HANDSHAKING_SIGNAL7        ((uint32_t)(7<<7))		//handshake signal 7��SPI2_TX��I2C1_RX��TIM4_CH3��TIM2_CH4��TIM2_UP��UART5_TX��TIM3_CH3��I2C2_TX
#define DMA0_SRC_PER_HANDSHAKING_SIGNAL8        ((uint32_t)(8<<7))		//handshake signal 8��QSPI_RX��TIM12_CH2��TIM13_CH1��TIM14_TRIG��TIM4_CH4
#define DMA0_SRC_PER_HANDSHAKING_SIGNAL9        ((uint32_t)(9<<7))		//handshake signal 9��QSPI_TX��TIM12_CH3��TIM13_CH2��TIM14_CH1��TIM4_TRIG
#define DMA0_SRC_PER_HANDSHAKING_SIGNAL10       ((uint32_t)(10<<7))		//handshake signal 10��TIM12_CH4��TIM13_CH3��TIM14_CH2��USART7_RX��TIM1_TRIG
#define DMA0_SRC_PER_HANDSHAKING_SIGNAL11       ((uint32_t)(11<<7))		//handshake signal 11��TIM12_UP��TIM13_CH4��TIM14_CH3��USART7_TX
#define DMA0_SRC_PER_HANDSHAKING_SIGNAL12       ((uint32_t)(12<<7))		//handshake signal 12��TIM12_TRIG��TIM13_UP��TIM14_CH4
#define DMA0_SRC_PER_HANDSHAKING_SIGNAL13       ((uint32_t)(13<<7))		//handshake signal 13��TIM12_CH1��TIM13_TRIG��TIM14_UP

/*************DMA1 DEST_PER**************/      //Destination handshaking number
#define DMA1_DEST_PER_HANDSHAKING_NULL          0x00000000U      
#define DMA1_DEST_PER_HANDSHAKING_SIGNAL0       ((uint32_t)(0<<11))		//handshake signal 0��ADC1��SPI0_RX��TIM1_TRIG
#define DMA1_DEST_PER_HANDSHAKING_SIGNAL1       ((uint32_t)(1<<11))		//handshake signal 1��DCMI��UART6_RX��TIM1_CH1��TIM8_UP
#define DMA1_DEST_PER_HANDSHAKING_SIGNAL2       ((uint32_t)(2<<11))		//handshake signal 2��TIM8_CH1��TIM8_CH2��TIM8_CH3��ADC2��SPI0_RX��UART1_RX��UART6_RX��TIM1_CH2
#define DMA1_DEST_PER_HANDSHAKING_SIGNAL3       ((uint32_t)(3<<11))		//handshake signal 3��ADC2��SPI0_TX��TIM1_CH1��TIM8_CH2
#define DMA1_DEST_PER_HANDSHAKING_SIGNAL4       ((uint32_t)(4<<11))		//handshake signal 4��ADC1��TIM1_CH4��TIM1_TRIG��TIM1_COM��TIM8_CH3
#define DMA1_DEST_PER_HANDSHAKING_SIGNAL5       ((uint32_t)(5<<11))		//handshake signal 5��QSPI_TX��AES_OUT��SPI0_TX��UART1_RX��TIM1_UP
#define DMA1_DEST_PER_HANDSHAKING_SIGNAL6       ((uint32_t)(6<<11))		//handshake signal 6��TIM1_CH1��TIM1_CH2��TIM1_CH3��QSPI_RX��AES_IN��UART6_TX
#define DMA1_DEST_PER_HANDSHAKING_SIGNAL7       ((uint32_t)(7<<11))		//handshake signal 7��DCMI��SHA_IN��UART1_TX��UART6_TX��TIM8_CH4��TIM8_TRIG��TIM8_COM
#define DMA1_DEST_PER_HANDSHAKING_SIGNAL8       ((uint32_t)(8<<11))		//handshake signal 8��SPI3_RX��TIM9_UP��TIM10_CH4��TIM11_CH3
#define DMA1_DEST_PER_HANDSHAKING_SIGNAL9       ((uint32_t)(9<<11))		//handshake signal 9��SPI3_TX��TIM9_TRIG��TIM10_UP��TIM11_CH4
#define DMA1_DEST_PER_HANDSHAKING_SIGNAL10      ((uint32_t)(10<<11))	//handshake signal 10��TIM9_CH1��TIM10_TRIG��TIM11_UP��USART8_RX
#define DMA1_DEST_PER_HANDSHAKING_SIGNAL11      ((uint32_t)(11<<11))	//handshake signal 11��TIM9_CH2��TIM10_CH1��TIM11_TRIG��USART8_TX
#define DMA1_DEST_PER_HANDSHAKING_SIGNAL12      ((uint32_t)(12<<11))	//handshake signal 12��CORDIC_IN��TIM9_CH3��TIM10_CH2��TIM11_CH1
#define DMA1_DEST_PER_HANDSHAKING_SIGNAL13      ((uint32_t)(13<<11))	//handshake signal 13��CORDIC_OUT��TIM9_CH4��TIM10_CH3��TIM11_CH2


/*************DMA1 SRC_PER**************/       //Source Handshaking number
#define DMA1_SRC_PER_HANDSHAKING_NULL           0x00000000U
#define DMA1_SRC_PER_HANDSHAKING_SIGNAL0        ((uint32_t)(0<<7))		//handshake signal 0��ADC1��SPI0_RX��TIM1_TRIG
#define DMA1_SRC_PER_HANDSHAKING_SIGNAL1        ((uint32_t)(1<<7))		//handshake signal 1��DCMI��UART6_RX��TIM1_CH1��TIM8_UP
#define DMA1_SRC_PER_HANDSHAKING_SIGNAL2        ((uint32_t)(2<<7))		//handshake signal 2��TIM8_CH1��TIM8_CH2��TIM8_CH3��ADC2��SPI0_RX��UART1_RX��UART6_RX��TIM1_CH2
#define DMA1_SRC_PER_HANDSHAKING_SIGNAL3        ((uint32_t)(3<<7))		//handshake signal 3��ADC2��SPI0_TX��TIM1_CH1��TIM8_CH2
#define DMA1_SRC_PER_HANDSHAKING_SIGNAL4        ((uint32_t)(4<<7))		//handshake signal 4��ADC1��TIM1_CH4��TIM1_TRIG��TIM1_COM��TIM8_CH3
#define DMA1_SRC_PER_HANDSHAKING_SIGNAL5        ((uint32_t)(5<<7))		//handshake signal 5��QSPI_TX��AES_OUT��SPI0_TX��UART1_RX��TIM1_UP
#define DMA1_SRC_PER_HANDSHAKING_SIGNAL6        ((uint32_t)(6<<7))		//handshake signal 6��TIM1_CH1��TIM1_CH2��TIM1_CH3��QSPI_RX��AES_IN��UART6_TX
#define DMA1_SRC_PER_HANDSHAKING_SIGNAL7        ((uint32_t)(7<<7))		//handshake signal 7��DCMI��SHA_IN��UART1_TX��UART6_TX��TIM8_CH4��TIM8_TRIG��TIM8_COM
#define DMA1_SRC_PER_HANDSHAKING_SIGNAL8        ((uint32_t)(8<<7))		//handshake signal 8��SPI3_RX��TIM9_UP��TIM10_CH4��TIM11_CH3
#define DMA1_SRC_PER_HANDSHAKING_SIGNAL9        ((uint32_t)(9<<7))		//handshake signal 9��SPI3_TX��TIM9_TRIG��TIM10_UP��TIM11_CH4
#define DMA1_SRC_PER_HANDSHAKING_SIGNAL10       ((uint32_t)(10<<7))		//handshake signal 10��TIM9_CH1��TIM10_TRIG��TIM11_UP��USART8_RX
#define DMA1_SRC_PER_HANDSHAKING_SIGNAL11       ((uint32_t)(11<<7))		//handshake signal 11��TIM9_CH2��TIM10_CH1��TIM11_TRIG��USART8_TX
#define DMA1_SRC_PER_HANDSHAKING_SIGNAL12       ((uint32_t)(12<<7))		//handshake signal 12��CORDIC_IN��TIM9_CH3��TIM10_CH2��TIM11_CH1
#define DMA1_SRC_PER_HANDSHAKING_SIGNAL13       ((uint32_t)(13<<7))		//handshake signal 13��CORDIC_OUT��TIM9_CH4��TIM10_CH3��TIM11_CH2

/**
  * @}
  */
#endif  
  
#if defined(UM324xH)
/** @defgroup DMA_DEST_PER_SIGNAL
  * @{
   */
#define DMA_DST_PER_SIGNAL0                0x00000000U
#define DMA_DST_PER_SIGNAL1                (DMA_CFGH0_DEST_PER_0 )
#define DMA_DST_PER_SIGNAL2                (DMA_CFGH0_DEST_PER_1 )
#define DMA_DST_PER_SIGNAL3                (DMA_CFGH0_DEST_PER_0 | DMA_CFGH0_DEST_PER_1)
#define DMA_DST_PER_SIGNAL4                (DMA_CFGH0_DEST_PER_2 )
#define DMA_DST_PER_SIGNAL5                (DMA_CFGH0_DEST_PER_0 | DMA_CFGH0_DEST_PER_2)
#define DMA_DST_PER_SIGNAL6                (DMA_CFGH0_DEST_PER_1 | DMA_CFGH0_DEST_PER_2)
#define DMA_DST_PER_SIGNAL7                (DMA_CFGH0_DEST_PER_0 |DMA_CFGH0_DEST_PER_1 | DMA_CFGH0_DEST_PER_2)
/**
  * @}
  */


/** @defgroup DMA_SRC_PER_SIGNAL
  * @{
   */

#define DMA_SRC_PER_SIGNAL0                 0x00000000U
#define DMA_SRC_PER_SIGNAL1                 (DMA_CFGH0_SRC_PER_0 )
#define DMA_SRC_PER_SIGNAL2                 (DMA_CFGH0_SRC_PER_1 )
#define DMA_SRC_PER_SIGNAL3                 (DMA_CFGH0_SRC_PER_0 | DMA_CFGH0_SRC_PER_1)
#define DMA_SRC_PER_SIGNAL4                 (DMA_CFGH0_SRC_PER_2 )
#define DMA_SRC_PER_SIGNAL5                 (DMA_CFGH0_SRC_PER_0 | DMA_CFGH0_SRC_PER_2)
#define DMA_SRC_PER_SIGNAL6                 (DMA_CFGH0_SRC_PER_1 | DMA_CFGH0_SRC_PER_2)
#define DMA_SRC_PER_SIGNAL7                 (DMA_CFGH0_SRC_PER_0 |DMA_CFGH0_SRC_PER_1 | DMA_CFGH0_SRC_PER_2)

/**
  * @}
  */
#endif  
  
/** @defgroup DMA_Priority_level DMA Priority level
  * @{
  */
#define DMA_PRIORITY_LOW              0x00000000U              /*!< Priority level : Low       */
#define DMA_PRIORITY_MEDIUM           DMA_CCR_PL_0             /*!< Priority level : Medium    */
#define DMA_PRIORITY_HIGH             DMA_CCR_PL_1             /*!< Priority level : High      */
#define DMA_PRIORITY_VERY_HIGH        DMA_CCR_PL               /*!< Priority level : Very_High */

/**
  * @}
  */

/** @defgroup DMA_interrupt_enable_definitions DMA interrupt enable definitions
  * @{
  */
#define DMA_IT_TFR_CH0                    DMA_MASKTFR_MASKTFR_0
#define DMA_IT_BLOCK_CH0                  DMA_MASKBLOCK_MASKBLOCK_0
#define DMA_IT_SRCTRAN_CH0                DMA_MASKSRCTRAN_MASKSRCTRAN_0
#define DMA_IT_DSTTRAN_CH0                DMA_MASKDSTTRAN_MASKDSTTRAN_0
#define DMA_IT_ERROR_CH0                  DMA_MASKERR_MASKERR_0

#define DMA_IT_TFR_CH1                    DMA_MASKTFR_MASKTFR_1
#define DMA_IT_BLOCK_CH1                  DMA_MASKBLOCK_MASKBLOCK_1
#define DMA_IT_SRCTRAN_CH1                DMA_MASKSRCTRAN_MASKSRCTRAN_1
#define DMA_IT_DSTTRAN_CH1                DMA_MASKDSTTRAN_MASKDSTTRAN_1
#define DMA_IT_ERROR_CH1                  DMA_MASKERR_MASKERR_1

#define DMA_IT_TFR_CH2                    DMA_MASKTFR_MASKTFR_2
#define DMA_IT_BLOCK_CH2                  DMA_MASKBLOCK_MASKBLOCK_2
#define DMA_IT_SRCTRAN_CH2                DMA_MASKSRCTRAN_MASKSRCTRAN_2
#define DMA_IT_DSTTRAN_CH2                DMA_MASKDSTTRAN_MASKDSTTRAN_2
#define DMA_IT_ERROR_CH2                  DMA_MASKERR_MASKERR_2

#define DMA_IT_TFR_CH3                    DMA_MASKTFR_MASKTFR_3
#define DMA_IT_BLOCK_CH3                  DMA_MASKBLOCK_MASKBLOCK_3
#define DMA_IT_SRCTRAN_CH3                DMA_MASKSRCTRAN_MASKSRCTRAN_3
#define DMA_IT_DSTTRAN_CH3                DMA_MASKDSTTRAN_MASKDSTTRAN_3
#define DMA_IT_ERROR_CH3                  DMA_MASKERR_MASKERR_3

#define DMA_IT_TFR_CH4                    DMA_MASKTFR_MASKTFR_4
#define DMA_IT_BLOCK_CH4                  DMA_MASKBLOCK_MASKBLOCK_4
#define DMA_IT_SRCTRAN_CH4                DMA_MASKSRCTRAN_MASKSRCTRAN_4
#define DMA_IT_DSTTRAN_CH4                DMA_MASKDSTTRAN_MASKDSTTRAN_4
#define DMA_IT_ERROR_CH4                  DMA_MASKERR_MASKERR_4

#define DMA_IT_TFR_CH5                    DMA_MASKTFR_MASKTFR_5
#define DMA_IT_BLOCK_CH5                  DMA_MASKBLOCK_MASKBLOCK_5
#define DMA_IT_SRCTRAN_CH5                DMA_MASKSRCTRAN_MASKSRCTRAN_5
#define DMA_IT_DSTTRAN_CH5                DMA_MASKDSTTRAN_MASKDSTTRAN_5
#define DMA_IT_ERROR_CH5                  DMA_MASKERR_MASKERR_5

#define DMA_IT_TFR_CH6                    DMA_MASKTFR_MASKTFR_6
#define DMA_IT_BLOCK_CH6                  DMA_MASKBLOCK_MASKBLOCK_6
#define DMA_IT_SRCTRAN_CH6                DMA_MASKSRCTRAN_MASKSRCTRAN_6
#define DMA_IT_DSTTRAN_CH6                DMA_MASKDSTTRAN_MASKDSTTRAN_6
#define DMA_IT_ERROR_CH6                  DMA_MASKERR_MASKERR_6

#define DMA_IT_TFR_CH7                    DMA_MASKTFR_MASKTFR_7
#define DMA_IT_BLOCK_CH7                  DMA_MASKBLOCK_MASKBLOCK_7
#define DMA_IT_SRCTRAN_CH7                DMA_MASKSRCTRAN_MASKSRCTRAN_7
#define DMA_IT_DSTTRAN_CH7                DMA_MASKDSTTRAN_MASKDSTTRAN_7
#define DMA_IT_ERROR_CH7                  DMA_MASKERR_MASKERR_7

/**
  * @}
  */

/** @defgroup DMA_flag_definitions DMA flag definitions
  * @{
  */
#define DMA_FLAG_TFR_CH0                  DMA_STATUSTFR_STATUSTFR_0
#define DMA_FLAG_BLOCK_CH0                DMA_STATUSBLOCK_STATUSBLOCK_0
#define DMA_FLAG_SRCTRAN_CH0              DMA_STATUSSRCTRAN_STATUSSRCTRAN_0
#define DMA_FLAG_DSTTRAN_CH0              DMA_STATUSDSTTRAN_STATUSDSTTRAN_0
#define DMA_FLAG_ERROR_CH0                DMA_STATUSERR_STATUSERR_0

#define DMA_FLAG_TFR_CH1                  DMA_STATUSTFR_STATUSTFR_1
#define DMA_FLAG_BLOCK_CH1                DMA_STATUSBLOCK_STATUSBLOCK_1
#define DMA_FLAG_SRCTRAN_CH1              DMA_STATUSSRCTRAN_STATUSSRCTRAN_1
#define DMA_FLAG_DSTTRAN_CH1              DMA_STATUSDSTTRAN_STATUSDSTTRAN_1
#define DMA_FLAG_ERROR_CH1                DMA_STATUSERR_STATUSERR_1

#define DMA_FLAG_TFR_CH2                  DMA_STATUSTFR_STATUSTFR_2
#define DMA_FLAG_BLOCK_CH2                DMA_STATUSBLOCK_STATUSBLOCK_2
#define DMA_FLAG_SRCTRAN_CH2              DMA_STATUSSRCTRAN_STATUSSRCTRAN_2
#define DMA_FLAG_DSTTRAN_CH2              DMA_STATUSDSTTRAN_STATUSDSTTRAN_2
#define DMA_FLAG_ERROR_CH2                DMA_STATUSERR_STATUSERR_2

#define DMA_FLAG_TFR_CH3                  DMA_STATUSTFR_STATUSTFR_3
#define DMA_FLAG_BLOCK_CH3                DMA_STATUSBLOCK_STATUSBLOCK_3
#define DMA_FLAG_SRCTRAN_CH3              DMA_STATUSSRCTRAN_STATUSSRCTRAN_3
#define DMA_FLAG_DSTTRAN_CH3              DMA_STATUSDSTTRAN_STATUSDSTTRAN_3
#define DMA_FLAG_ERROR_CH3                DMA_STATUSERR_STATUSERR_3

#define DMA_FLAG_TFR_CH4                  DMA_STATUSTFR_STATUSTFR_4
#define DMA_FLAG_BLOCK_CH4                DMA_STATUSBLOCK_STATUSBLOCK_4
#define DMA_FLAG_SRCTRAN_CH4              DMA_STATUSSRCTRAN_STATUSSRCTRAN_4
#define DMA_FLAG_DSTTRAN_CH4              DMA_STATUSDSTTRAN_STATUSDSTTRAN_4
#define DMA_FLAG_ERROR_CH4                DMA_STATUSERR_STATUSERR_4

#define DMA_FLAG_TFR_CH5                  DMA_STATUSTFR_STATUSTFR_5
#define DMA_FLAG_BLOCK_CH5                DMA_STATUSBLOCK_STATUSBLOCK_5
#define DMA_FLAG_SRCTRAN_CH5              DMA_STATUSSRCTRAN_STATUSSRCTRAN_5
#define DMA_FLAG_DSTTRAN_CH5              DMA_STATUSDSTTRAN_STATUSDSTTRAN_5
#define DMA_FLAG_ERROR_CH5                DMA_STATUSERR_STATUSERR_5

#define DMA_FLAG_TFR_CH6                  DMA_STATUSTFR_STATUSTFR_6
#define DMA_FLAG_BLOCK_CH6                DMA_STATUSBLOCK_STATUSBLOCK_6
#define DMA_FLAG_SRCTRAN_CH6              DMA_STATUSSRCTRAN_STATUSSRCTRAN_6
#define DMA_FLAG_DSTTRAN_CH6              DMA_STATUSDSTTRAN_STATUSDSTTRAN_6
#define DMA_FLAG_ERROR_CH6                DMA_STATUSERR_STATUSERR_6

#define DMA_FLAG_TFR_CH7                  DMA_STATUSTFR_STATUSTFR_7
#define DMA_FLAG_BLOCK_CH7                DMA_STATUSBLOCK_STATUSBLOCK_7
#define DMA_FLAG_SRCTRAN_CH7              DMA_STATUSSRCTRAN_STATUSSRCTRAN_7
#define DMA_FLAG_DSTTRAN_CH7              DMA_STATUSDSTTRAN_STATUSDSTTRAN_7
#define DMA_FLAG_ERROR_CH7                DMA_STATUSERR_STATUSERR_7

/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup DMA_Exported_Macros DMA Exported Macros
  * @{
  */

/** @brief  Reset DMA handle state.
  * @param  __HANDLE__ DMA handle
  * @retval None
  */
#define __HAL_DMA_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_DMA_STATE_RESET)

/**
  * @brief  Enable the specified DMA Channel.
  * @param  __HANDLE__ DMA handle
  * @retval None
  */
#define __HAL_DMA_ENABLE(__HANDLE__)        ((__HANDLE__)->Instance->CFGREG |=  DMA_CFGREG_DMA_EN)

/**
  * @brief  Disable the specified DMA Channel.
  * @param  __HANDLE__ DMA handle
  * @retval None
  */
#define __HAL_DMA_DISABLE(__HANDLE__)       ((__HANDLE__)->Instance->CFGREG &=  ~DMA_CFGREG_DMA_EN)

/* Interrupt & Flag management */

/**
  * @brief  Enable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param __INTERRUPT__ specifies the DMA interrupt sources to be enabled or disabled.
  * @retval None
  */
#define __HAL_DMA_ENABLE_IT(__HANDLE__, __CHANNEL__)   (&((__HANDLE__)->Instance->CTL0) |= (__INTERRUPT__))

/**
  * @brief  Disable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param __INTERRUPT__ specifies the DMA interrupt sources to be enabled or disabled.
  * @retval None
  */
#define __HAL_DMA_DISABLE_IT(__HANDLE__, __CHANNEL__)  ((__HANDLE__)->Instance->CTL0 +  &= ~(__INTERRUPT__))

/**
  * @brief  Check whether the specified DMA Channel interrupt is enabled or not.
  * @param  __HANDLE__ DMA handle
  * @param  __INTERRUPT__ specifies the DMA interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg DMA_IT_TC  Transfer complete interrupt mask
  *            @arg DMA_IT_HT  Half transfer complete interrupt mask
  *            @arg DMA_IT_TE  Transfer error interrupt mask
  * @retval None
  */
#define __HAL_DMA_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)  (((__HANDLE__)->Instance->CCR & (__INTERRUPT__)))

/**
  * @brief  Return the number of remaining data units in the current DMA Channel transfer.
  * @param  __HANDLE__ DMA handle
  * @retval None
  */
//#define __HAL_DMA_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->CNDTR)
#define __HAL_DMA_GET_COUNTER(__HANDLE__) (uint32_t)(*((&((__HANDLE__)->Instance->CTLH0)) + (((__HANDLE__)->DmaChannelSel)*0x58)))


/**
  * @brief  Enable DMA channel
  * @param  __HANDLE__ DMA handle
            __CHANNEL__ DMA channel
  * @retval None
  */
#define __HAL_DMA_CHANNEL_ENABLE(__HANDLE__,__CHANNEL__)   WRITE_REG(((__HANDLE__)->Instance->CHENREG),(1U<<__CHANNEL__)|(1U<<(__CHANNEL__+8)))

/**
  * @brief  Disable DMA channel
  * @param  __HANDLE__ DMA handle
            __CHANNEL__ DMA channel
  * @retval None
  */
#define __HAL_DMA_CHANNEL_DISABLE(__HANDLE__,__CHANNEL__)   WRITE_REG(((__HANDLE__)->Instance->CHENREG),((1U<<(__CHANNEL__+8))))


/**
  * @brief  Enable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param  __CHANNEL__ DMA channel
  * @retval None
  */
#define __HAL_DMA_ENABLE_TFR_IT(__HANDLE__, __CHANNEL__)   WRITE_REG(((__HANDLE__)->Instance->MASKTFR),(1U<<__CHANNEL__) | (1U<<(__CHANNEL__ + 8)))

/**
  * @brief  Disable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param  __CHANNEL__ DMA channel
  * @retval None
  */
#define __HAL_DMA_DISABLE_TFR_IT(__HANDLE__, __CHANNEL__)  WRITE_REG(((__HANDLE__)->Instance->MASKTFR),(1U<<(__CHANNEL__ + 8)))

/**
  * @brief  Enable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param  __CHANNEL__ DMA channel
  * @retval None
  */
#define __HAL_DMA_ENABLE_BLOCK_IT(__HANDLE__, __CHANNEL__)   WRITE_REG(((__HANDLE__)->Instance->MASKBLOCK),(1U<<__CHANNEL__) | (1U<<(__CHANNEL__ + 8)))

/**
  * @brief  Disable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param  __CHANNEL__ DMA channel
  * @retval None
  */
#define __HAL_DMA_DISABLE_BLOCK_IT(__HANDLE__, __CHANNEL__)  WRITE_REG(((__HANDLE__)->Instance->MASKBLOCK),(1U<<(__CHANNEL__ + 8)))

/**
  * @brief  Enable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param  __CHANNEL__ DMA channel
  * @retval None
  */
#define __HAL_DMA_ENABLE_SRCTRAN_IT(__HANDLE__, __CHANNEL__)   WRITE_REG(((__HANDLE__)->Instance->MASKSRCTRAN),(1U<<__CHANNEL__) | (1U<<(__CHANNEL__ + 8)))

/**
  * @brief  Disable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param  __CHANNEL__ DMA channel
  * @retval None
  */
#define __HAL_DMA_DISABLE_SRCTRAN_IT(__HANDLE__, __CHANNEL__)  WRITE_REG(((__HANDLE__)->Instance->MASKSRCTRAN),(1U<<(__CHANNEL__ + 8)))

/**
  * @brief  Enable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param  __CHANNEL__ DMA channel
  * @retval None
  */
#define __HAL_DMA_ENABLE_DSTTRAN_IT(__HANDLE__, __CHANNEL__)   WRITE_REG(((__HANDLE__)->Instance->MASKDSTTRAN),(1U<<__CHANNEL__) | (1U<<(__CHANNEL__ + 8)))

/**
  * @brief  Disable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param  __CHANNEL__ DMA channel
  * @retval None
  */
#define __HAL_DMA_DISABLE_DSTTRAN_IT(__HANDLE__, __CHANNEL__)  WRITE_REG(((__HANDLE__)->Instance->MASKDSTTRAN),(1U<<(__CHANNEL__ + 8)))

/**
  * @brief  Enable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param  __CHANNEL__ DMA channel
  * @retval None
  */
#define __HAL_DMA_ENABLE_ERROR_IT(__HANDLE__, __CHANNEL__)   WRITE_REG(((__HANDLE__)->Instance->MASKERR),(1U<<__CHANNEL__) | (1U<<(__CHANNEL__ + 8)))

/**
  * @brief  Disable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param  __CHANNEL__ DMA channel
  * @retval None
  */
#define __HAL_DMA_DISABLE_ERROR_IT(__HANDLE__, __CHANNEL__)  WRITE_REG(((__HANDLE__)->Instance->MASKERR),(1U<<(__CHANNEL__ + 8)))

/**
  * @brief  Clear DMA Transmission interruption Flag
  * @param  __HANDLE__ DMA handle
            __CHANNEL__ DMA channel
  * @retval None
  */

#define __HAL_DMA_CLEAR_TFR_FLAG(__HANDLE__,__CHANNEL__)  WRITE_REG(((__HANDLE__)->Instance->CLEARTFR),(1U<<__CHANNEL__))

/**
  * @brief  Clear DMA Block Transmission interruption Flag
  * @param  __HANDLE__ DMA handle
            __CHANNEL__ DMA channel
  * @retval None
  */

#define __HAL_DMA_CLEAR_BLOCK_FLAG(__HANDLE__,__CHANNEL__)  WRITE_REG(((__HANDLE__)->Instance->CLEARBLOCK),(1U<<__CHANNEL__))

/**
  * @brief  Clear DMA Source Transmission interruption Flag
  * @param  __HANDLE__ DMA handle
            __CHANNEL__ DMA channel
  * @retval None
  */

#define __HAL_DMA_CLEAR_SRCTRAN_FLAG(__HANDLE__,__CHANNEL__)  WRITE_REG(((__HANDLE__)->Instance->CLEARSRCTRAN),(1U<<__CHANNEL__))

/**
  * @brief  Clear DMA destination Transmission interruption Flag
  * @param  __HANDLE__ DMA handle
            __CHANNEL__ DMA channel
  * @retval None
  */

#define __HAL_DMA_CLEAR_DSTTRAN_FLAG(__HANDLE__,__CHANNEL__)  WRITE_REG(((__HANDLE__)->Instance->CLEARDSTTRAN),(1U<<__CHANNEL__))

/**
  * @brief  Clear DMA error interruption Flag
  * @param  __HANDLE__ DMA handle
            __CHANNEL__ DMA channel
  * @retval None
  */

#define __HAL_DMA_CLEAR_ERROR_FLAG(__HANDLE__,__CHANNEL__)  WRITE_REG(((__HANDLE__)->Instance->CLEARERR),(1U<<__CHANNEL__))

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/

/** @addtogroup DMA_Exported_Functions
  * @{
  */

/** @addtogroup DMA_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions *****************************/
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma);

HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, pDMA_CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID);

/**
  * @}
  */

/** @addtogroup DMA_Exported_Functions_Group2
  * @{
  */
/* IO operation functions *****************************************************/
HAL_StatusTypeDef HAL_DMA_Start(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress,
                                   uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, uint32_t Timeout);

void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);

/**
  * @}
  */

/** @addtogroup DMA_Exported_Functions_Group3
  * @{
  */
/* Peripheral State and Error functions ***************************************/
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
uint32_t             HAL_DMA_GetError(DMA_HandleTypeDef *hdma);
/**
  * @}
  */

/** @addtogroup DMA_Exported_Functions_Group4
  * @{
  */
/* ************************* DMAMUX functions ***************************************/
/* ------------------------- REQUEST -----------------------------------------*/
HAL_StatusTypeDef HAL_DMAEx_ConfigMuxRequestGenerator(DMA_HandleTypeDef *hdma,
        HAL_DMA_MuxRequestGeneratorConfigTypeDef *pRequestGeneratorConfig);
HAL_StatusTypeDef HAL_DMAEx_EnableMuxRequestGenerator(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMAEx_DisableMuxRequestGenerator(DMA_HandleTypeDef *hdma);
/* -------------------------------------------------------------------------- */

/* ------------------------- SYNCHRO -----------------------------------------*/
HAL_StatusTypeDef HAL_DMAEx_ConfigMuxSync(DMA_HandleTypeDef *hdma, HAL_DMA_MuxSyncConfigTypeDef *pSyncConfig);
/* -------------------------------------------------------------------------- */

void HAL_DMA_TfrCallback(DMA_HandleTypeDef *hdma);
void HAL_DMA_rxCallback(DMA_HandleTypeDef *hdma);
/**
  * @}
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup DMA_Private_Macros DMA Private Macros
  * @{
  */

#define IS_DMA_DIRECTION(DIRECTION) (((DIRECTION) == DMA_PERIPH_TO_MEMORY ) || \
                                     ((DIRECTION) == DMA_MEMORY_TO_PERIPH)  || \
                                     ((DIRECTION) == DMA_MEMORY_TO_MEMORY))

#define IS_DMA_BUFFER_SIZE(SIZE) (((SIZE) >= 0x1U) && ((SIZE) < 0x40000U))

#define IS_DMA_PERIPHERAL_INC_STATE(STATE) (((STATE) == DMA_PINC_ENABLE) || \
                                            ((STATE) == DMA_PINC_DISABLE))

#define IS_DMA_MEMORY_INC_STATE(STATE) (((STATE) == DMA_MINC_ENABLE)  || \
                                        ((STATE) == DMA_MINC_DISABLE))

#define IS_DMA_ALL_REQUEST(REQUEST)    ((REQUEST) <= DMA_REQUEST_UCPD1_TX)

#define IS_DMA_PERIPHERAL_DATA_SIZE(SIZE) (((SIZE) == DMA_PDATAALIGN_BYTE)     || \
                                           ((SIZE) == DMA_PDATAALIGN_HALFWORD) || \
                                           ((SIZE) == DMA_PDATAALIGN_WORD))

#define IS_DMA_MEMORY_DATA_SIZE(SIZE) (((SIZE) == DMA_MDATAALIGN_BYTE)     || \
                                       ((SIZE) == DMA_MDATAALIGN_HALFWORD) || \
                                       ((SIZE) == DMA_MDATAALIGN_WORD ))

#define IS_DMA_MODE(MODE) (((MODE) == DMA_NORMAL )  || \
                           ((MODE) == DMA_CIRCULAR))

#define IS_DMA_PRIORITY(PRIORITY) (((PRIORITY) == DMA_PRIORITY_LOW )   || \
                                   ((PRIORITY) == DMA_PRIORITY_MEDIUM) || \
                                   ((PRIORITY) == DMA_PRIORITY_HIGH)   || \
                                   ((PRIORITY) == DMA_PRIORITY_VERY_HIGH))

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __UM324xx_HAL_DMA_H */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
