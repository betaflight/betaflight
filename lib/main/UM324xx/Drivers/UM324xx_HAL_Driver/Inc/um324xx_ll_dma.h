/**
  ******************************************************************************
  * @file    stm32f4xx_ll_dma.h
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    14-April-2017
  * @brief   Header file of DMA LL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324xx_LL_DMA_H__
#define __UM324xx_LL_DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx.h"

/** @addtogroup STM32F4xx_LL_Driver
  * @{
  */

#if defined (DMA1) || defined (DMA2)

/** @defgroup DMA_LL DMA
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup DMA_LL_Private_Variables DMA Private Variables
  * @{
  */
/* Array used to get the DMA stream register offset versus stream index LL_DMA_STREAM_x */
static const uint32_t STREAM_OFFSET_TAB[] =
{
  (uint32_t)(DMA1_Stream0_BASE - DMA1_BASE),
  (uint32_t)(DMA1_Stream1_BASE - DMA1_BASE),
  (uint32_t)(DMA1_Stream2_BASE - DMA1_BASE),
  (uint32_t)(DMA1_Stream3_BASE - DMA1_BASE),
  (uint32_t)(DMA1_Stream4_BASE - DMA1_BASE),
  (uint32_t)(DMA1_Stream5_BASE - DMA1_BASE),
  (uint32_t)(DMA1_Stream6_BASE - DMA1_BASE),
  (uint32_t)(DMA1_Stream7_BASE - DMA1_BASE)
};

/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup DMA_LL_Private_Constants DMA Private Constants
  * @{
  */
/**
  * @}
  */


/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/** @defgroup DMA_LL_EC_STREAM STREAM
  * @{
  */
#define LL_DMA_STREAM_0                   DMA_Channel_0
#define LL_DMA_STREAM_1                   DMA_Channel_1
#define LL_DMA_STREAM_2                   DMA_Channel_2
#define LL_DMA_STREAM_3                   DMA_Channel_3
#define LL_DMA_STREAM_4                   DMA_Channel_4
#define LL_DMA_STREAM_5                   DMA_Channel_5
#define LL_DMA_STREAM_6                   DMA_Channel_6
#define LL_DMA_STREAM_7                   DMA_Channel_7
#define LL_DMA_STREAM_ALL                 0xFFFF0000U
/**
  * @}
  */

/** @defgroup DMA_Data_transfer_direction DMA Data transfer direction
  * @{
  */
#define LL_DMA_MEMORY_TO_MEMORY                0U                                                  /*!< memory to memory transfer   */
#define LL_DMA_MEMORY_TO_PERIPH                DMA_CTL0_TT_FC_0                                   /*!< Peripheral to memory direction */
#define LL_DMA_PERIPH_TO_MEMORY                DMA_CTL0_TT_FC_1                                   /*!< Memory to peripheral direction */
#define LL_DMA_PERIPH_TO_PERIPH                (DMA_CTL0_TT_FC_0 | DMA_CTL0_TT_FC_1)              /*!< Memory to memory direction     */

/**
  * @}
  */

/** @defgroup DMA_destination_incremented_mode DMA Peripheral incremented mode
  * @{
  */
#define LL_DMA_DSTINC_INC              0x00000000U                     /*!< destination address increment */
#define LL_DMA_DSTINC_DEC              DMA_CTL0_DINC_0                 /*!< destination address decrement */
#define LL_DMA_DSTINC_NOC              DMA_CTL0_DINC_1                 /*!< destination address remains unchanged*/
/**
  * @}
  */

/** @defgroup DMA_source_incremented_mode DMA Memory incremented mode
  * @{
  */
#define LL_DMA_SRCINC_INC              0x00000000U                   /*!< Source address increment  */
#define LL_DMA_SRCINC_DEC              DMA_CTL0_SINC_0               /*!< Source address decrement */
#define LL_DMA_SRCINC_NOC              DMA_CTL0_SINC_1               /*!< Source address remains unchanged*/
/**
  * @}
  */

/** @defgroup DMA_source_data_size DMA Memory data size
  * @{
  */
#define LL_DMA_SRCDATAALIGN_BYTE          0x00000000U                  /*!< source data alignment : Byte     */
#define LL_DMA_SRCDATAALIGN_HALFWORD      DMA_CTL0_SRC_TR_WIDTH_0              /*!< source data alignment : HalfWord */
#define LL_DMA_SRCDATAALIGN_WORD          DMA_CTL0_SRC_TR_WIDTH_1              /*!< source data alignment : Word     */
/**
  * @}
  */

/** @defgroup DMA_destination_data_size DMA Memory data size
  * @{
  */
#define LL_DMA_DSTDATAALIGN_BYTE          0x00000000U                          /*!< destination data alignment : Byte     */
#define LL_DMA_DSTDATAALIGN_HALFWORD      DMA_CTL0_DST_TR_WIDTH_0              /*!< destination data alignment : HalfWord */
#define LL_DMA_DSTDATAALIGN_WORD          DMA_CTL0_DST_TR_WIDTH_1              /*!< destination data alignment : Word     */
/**
  * @}
  */

/** @defgroup DMA_destination transmission Burst length
  * @{
  */
#define LL_DMA_BURST_DST_NUM_1         0x00000000U               /*!< DMA_destination transmission Burst length is 1BYTE */
#define LL_DMA_BURST_DST_NUM_4         DMA_CTL0_DST_MSIZE_0      /*!< DMA_destination transmission Burst length is 4BYTE */
#define LL_DMA_BURST_DST_NUM_8         DMA_CTL0_DST_MSIZE_1      /*!< DMA_destination transmission Burst length is 8BYTE */
#define LL_DMA_BURST_DST_NUM_16        (DMA_CTL0_DST_MSIZE_0 \
                                     | DMA_CTL0_DST_MSIZE_1)  /*!< DMA_destination transmission Burst length is 16BYTE */

/**
  * @}
  */

/** @defgroup DMA_Source transmission Burst length
  * @{
  */
#define LL_DMA_BURST_SRC_NUM_1         0x00000000U                 /*!< DMA_Source transmission Burst length is 1BYTE */
#define LL_DMA_BURST_SRC_NUM_4         DMA_CTL0_SRC_MSIZE_0        /*!< DMA_Source transmission Burst length is 4BYTE */
#define LL_DMA_BURST_SRC_NUM_8         DMA_CTL0_SRC_MSIZE_1        /*!< DMA_Source transmission Burst length is 8BYTE */
#define LL_DMA_BURST_SRC_NUM_16        (DMA_CTL0_SRC_MSIZE_0 \
                                    |   DMA_CTL0_SRC_MSIZE_1)     /*!< DMA_Source transmission Burst length is 16BYTE */

/**
  * @}
  */

/*************DMA0 DEST_PER**************/      //Destination handshaking number
#define LL_DMA_DEST_PER_HANDSHAKING_NULL          0x00000000U
#define LL_DMA_DEST_PER_HANDSHAKING_SIGNAL0       ((uint32_t)(0<<11))		
#define LL_DMA_DEST_PER_HANDSHAKING_SIGNAL1       ((uint32_t)(1<<11))		
#define LL_DMA_DEST_PER_HANDSHAKING_SIGNAL2       ((uint32_t)(2<<11))		
#define LL_DMA_DEST_PER_HANDSHAKING_SIGNAL3       ((uint32_t)(3<<11))		
#define LL_DMA_DEST_PER_HANDSHAKING_SIGNAL4       ((uint32_t)(4<<11))		
#define LL_DMA_DEST_PER_HANDSHAKING_SIGNAL5       ((uint32_t)(5<<11))		
#define LL_DMA_DEST_PER_HANDSHAKING_SIGNAL6       ((uint32_t)(6<<11))		
#define LL_DMA_DEST_PER_HANDSHAKING_SIGNAL7       ((uint32_t)(7<<11))		
#define LL_DMA_DEST_PER_HANDSHAKING_SIGNAL8       ((uint32_t)(8<<11))		
#define LL_DMA_DEST_PER_HANDSHAKING_SIGNAL9       ((uint32_t)(9<<11))		
#define LL_DMA_DEST_PER_HANDSHAKING_SIGNAL10      ((uint32_t)(10<<11))	
#define LL_DMA_DEST_PER_HANDSHAKING_SIGNAL11      ((uint32_t)(11<<11))	
#define LL_DMA_DEST_PER_HANDSHAKING_SIGNAL12      ((uint32_t)(12<<11))	
#define LL_DMA_DEST_PER_HANDSHAKING_SIGNAL13      ((uint32_t)(13<<11))	


/*************DMA0 SRC_PER**************/       //Source Handshaking number
#define LL_DMA_SRC_PER_HANDSHAKING_NULL           0x00000000U
#define LL_DMA_SRC_PER_HANDSHAKING_SIGNAL0        ((uint32_t)(0<<7))		
#define LL_DMA_SRC_PER_HANDSHAKING_SIGNAL1        ((uint32_t)(1<<7))		
#define LL_DMA_SRC_PER_HANDSHAKING_SIGNAL2        ((uint32_t)(2<<7))		
#define LL_DMA_SRC_PER_HANDSHAKING_SIGNAL3        ((uint32_t)(3<<7))		
#define LL_DMA_SRC_PER_HANDSHAKING_SIGNAL4        ((uint32_t)(4<<7))		
#define LL_DMA_SRC_PER_HANDSHAKING_SIGNAL5        ((uint32_t)(5<<7))		
#define LL_DMA_SRC_PER_HANDSHAKING_SIGNAL6        ((uint32_t)(6<<7))		
#define LL_DMA_SRC_PER_HANDSHAKING_SIGNAL7        ((uint32_t)(7<<7))		
#define LL_DMA_SRC_PER_HANDSHAKING_SIGNAL8        ((uint32_t)(8<<7))		
#define LL_DMA_SRC_PER_HANDSHAKING_SIGNAL9        ((uint32_t)(9<<7))		
#define LL_DMA_SRC_PER_HANDSHAKING_SIGNAL10       ((uint32_t)(10<<7))		
#define LL_DMA_SRC_PER_HANDSHAKING_SIGNAL11       ((uint32_t)(11<<7))		
#define LL_DMA_SRC_PER_HANDSHAKING_SIGNAL12       ((uint32_t)(12<<7))		
#define LL_DMA_SRC_PER_HANDSHAKING_SIGNAL13       ((uint32_t)(13<<7))		

/** @defgroup DMA Source/destination reload enable
  * @{
  */
#define LL_DMA_SRC_RELOAD_DISABLE           0x00000000U     /*!< Source transmission software handshake*/
#define LL_DMA_SRC_RELOAD_ENABLE            DMA_CFG0_RELOAD_SRC             /*!< Source transmission hardware handshake */

#define LL_DMA_DST_RELOAD_DISABLE           0x00000000U    /*!< destination transmission software handshake*/
#define LL_DMA_DST_RELOAD_ENABLE            DMA_CFG0_RELOAD_DST            /*!< destination transmission hardware handshake */
/**
  * @}
  */

  /** @defgroup DMA hardware/software handshaking
  * @{
  */
#define LL_DMA_SRC_HS_SW           DMA_CFG0_HS_SEL_SRC     /*!< Source transmission software handshake*/
#define LL_DMA_SRC_HS_HW           0x00000000U             /*!< Source transmission hardware handshake */

#define LL_DMA_DST_HS_SW           DMA_CFG0_HS_SEL_DST    /*!< destination transmission software handshake*/
#define LL_DMA_DST_HS_HW           0x00000000U            /*!< destination transmission hardware handshake */
/**
  * @}
  */

/** @defgroup DMA FIFO mode
  * @{
  */
#define LL_DMA_FIFOMODE_DISABLE         0x00000000U               /*!< FIFO Mode disable */
#define LL_DMA_FIFOMODE_ENABLE         DMA_CFGH0_FIFO_MODE       /*!< FIFO Mode disable */

/**
  * @}
  */

/** @defgroup DMA FC mode
  * @{
  */

#define LL_DMA_FCMODE_DISABLE         DMA_CFGH0_FCMODE                 /*!< FC Mode disable */
#define LL_DMA_FCMODE_ENABLE         0x00000000U                       /*!< FC Mode disable */
/**
  * @}
  */

#define LL_DMA_PRIORITY_0              (0 << 5)
#define LL_DMA_PRIORITY_1              (1 << 5)
#define LL_DMA_PRIORITY_2              (2 << 5)
#define LL_DMA_PRIORITY_3              (3 << 5)
#define LL_DMA_PRIORITY_4              (4 << 5)
#define LL_DMA_PRIORITY_5              (5 << 5)
#define LL_DMA_PRIORITY_6              (6 << 5)
#define LL_DMA_PRIORITY_7              (7 << 5)

/** @defgroup DMA_LL_ES_INIT DMA Exported Init structure
  * @{
  */
typedef struct
{
    uint32_t SrcAddress;                /*!< Specifies the peripheral base address for DMA transfer
                                        or as Source base address in case of memory to memory transfer direction.

                                        This parameter must be a value between Min_Data = 0 and Max_Data = 0xFFFFFFFF. */

    uint32_t DstAddress;                /*!< Specifies the memory base address for DMA transfer
                                        or as Destination base address in case of memory to memory transfer direction.

                                        This parameter must be a value between Min_Data = 0 and Max_Data = 0xFFFFFFFF. */

    uint32_t Direction;                   /*!< Specifies if the data will be transferred from memory to peripheral,
                                            from memory to memory or from peripheral to memory.
                                            This parameter can be a value of @ref DMA_Data_transfer_direction */

    uint32_t SrcInc;                      /*!< Specifies whether the Peripheral address register should be incremented or not.
                                           This parameter can be a value of @ref DMA_Peripheral_incremented_mode */

    uint32_t DstInc;                      /*!< Specifies whether the memory address register should be incremented or not.
                                           This parameter can be a value of @ref DMA_Memory_incremented_mode */

    uint32_t SrcDataAlignment;            /*!< Specifies the Peripheral data width.
                                           This parameter can be a value of @ref DMA_Peripheral_data_size */

    uint32_t DstDataAlignment;            /*!< The Burst length of the target transmission. The number of data
                                        written each time the target peripheral Req handshake signal is available*/

    uint32_t SrcMSize;                    /*!<  Burst length of source transmission. The number of data read each time
                                        the source peripheral Req handshake signal is available*/

    uint32_t DstMSize;                    /*!<  Burst length of source transmission. The number of data read each time
                                        the source peripheral Req handshake signal is available*/
    uint32_t NbData;                      /*!< Specifies the number of data to transfer, in data unit.
                                        The data unit is equal to the source buffer configuration set in PeripheralSize
                                        or MemorySize parameters depending in the transfer direction.
                                        This parameter must be a value between Min_Data = 0 and Max_Data = 0x0000FFFF

                                        This feature can be modified afterwards using unitary function @ref LL_DMA_SetDataLength(). */

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
    uint32_t Channel;
    
} LL_DMA_InitTypeDef;
/** @defgroup DMA_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

uint32_t LL_DMA_Init(DMA_TypeDef *DMAx, uint32_t Stream, LL_DMA_InitTypeDef *DMA_InitStruct);
uint32_t LL_DMA_DeInit(DMA_TypeDef *DMAx, uint32_t Stream);
void LL_DMA_StructInit(LL_DMA_InitTypeDef *DMA_InitStruct);

/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */

#endif /* DMA1 || DMA2 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_LL_DMA_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
