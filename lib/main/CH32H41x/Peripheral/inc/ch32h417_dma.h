/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_dma.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the 
*                      DMA firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/ 
#ifndef __CH32H417_DMA_H
#define __CH32H417_DMA_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32h417.h"
 
typedef enum
{
  DMAMUX_DMAREQ_ID_TIM1_CH1                = 1, /*!< dmamux channel dma request inputs resources: generator channel1 */
  DMAMUX_DMAREQ_ID_TIM1_CH2                = 2, /*!< dmamux channel dma request inputs resources: generator channel2 */
  DMAMUX_DMAREQ_ID_TIM1_CH3                = 3, /*!< dmamux channel dma request inputs resources: generator channel3 */
  DMAMUX_DMAREQ_ID_TIM1_CH4                = 4, /*!< dmamux channel dma request inputs resources: generator channel4 */
  DMAMUX_DMAREQ_ID_TIM1_UP                 = 5, /*!< dmamux channel dma request inputs resources: adc1 */
  DMAMUX_DMAREQ_ID_TIM1_COM                = 6, /*!< dmamux channel dma request inputs resources: adc2 */
  DMAMUX_DMAREQ_ID_TIM1_TRIG               = 7, /*!< dmamux channel dma request inputs resources: adc3 */
  DMAMUX_DMAREQ_ID_TIM2_CH1                = 8, /*!< dmamux channel dma request inputs resources: dac1 */
  DMAMUX_DMAREQ_ID_TIM2_CH2                = 9, /*!< dmamux channel dma request inputs resources: dac2 */
  DMAMUX_DMAREQ_ID_TIM2_CH3                = 10, /*!< dmamux channel dma request inputs resources: timer6 overflow */
  DMAMUX_DMAREQ_ID_TIM2_CH4                = 11, /*!< dmamux channel dma request inputs resources: timer7 overflow */
  DMAMUX_DMAREQ_ID_TIM2_UP                 = 12, /*!< dmamux channel dma request inputs resources: spi1 rx */
  DMAMUX_DMAREQ_ID_TIM2_TRIG               = 13, /*!< dmamux channel dma request inputs resources: spi1 tx */
  DMAMUX_DMAREQ_ID_TIM3_CH1                = 14, /*!< dmamux channel dma request inputs resources: dac1 */
  DMAMUX_DMAREQ_ID_TIM3_CH2                = 15, /*!< dmamux channel dma request inputs resources: dac2 */
  DMAMUX_DMAREQ_ID_TIM3_CH3                = 16, /*!< dmamux channel dma request inputs resources: timer6 overflow */
  DMAMUX_DMAREQ_ID_TIM3_CH4                = 17, /*!< dmamux channel dma request inputs resources: timer7 overflow */
  DMAMUX_DMAREQ_ID_TIM3_UP                 = 18, /*!< dmamux channel dma request inputs resources: spi1 rx */
  DMAMUX_DMAREQ_ID_TIM3_TRIG               = 19, /*!< dmamux channel dma request inputs resources: spi1 tx */
 
  DMAMUX_DMAREQ_ID_TIM4_CH1                = 20, /*!< dmamux channel dma request inputs resources: dac1 */
  DMAMUX_DMAREQ_ID_TIM4_CH2                = 21, /*!< dmamux channel dma request inputs resources: dac2 */
  DMAMUX_DMAREQ_ID_TIM4_CH3                = 22, /*!< dmamux channel dma request inputs resources: timer6 overflow */
  DMAMUX_DMAREQ_ID_TIM4_CH4                = 23, /*!< dmamux channel dma request inputs resources: timer7 overflow */
  DMAMUX_DMAREQ_ID_TIM4_UP                 = 24, /*!< dmamux channel dma request inputs resources: spi1 rx */
  DMAMUX_DMAREQ_ID_TIM4_TRIG               = 25, /*!< dmamux channel dma request inputs resources: spi1 tx */
 
  DMAMUX_DMAREQ_ID_TIM5_CH1                = 26, /*!< dmamux channel dma request inputs resources: dac1 */
  DMAMUX_DMAREQ_ID_TIM5_CH2                = 27, /*!< dmamux channel dma request inputs resources: dac2 */
  DMAMUX_DMAREQ_ID_TIM5_CH3                = 28, /*!< dmamux channel dma request inputs resources: timer6 overflow */
  DMAMUX_DMAREQ_ID_TIM5_CH4                = 29, /*!< dmamux channel dma request inputs resources: timer7 overflow */
  DMAMUX_DMAREQ_ID_TIM5_UP                 = 30, /*!< dmamux channel dma request inputs resources: spi1 rx */
  DMAMUX_DMAREQ_ID_TIM5_TRIG               = 31, /*!< dmamux channel dma request inputs resources: spi1 tx */
  
  DMAMUX_DMAREQ_ID_TIM8_CH1                = 32, /*!< dmamux channel dma request inputs resources: generator channel1 */
  DMAMUX_DMAREQ_ID_TIM8_CH2                = 33, /*!< dmamux channel dma request inputs resources: generator channel2 */
  DMAMUX_DMAREQ_ID_TIM8_CH3                = 34, /*!< dmamux channel dma request inputs resources: generator channel3 */
  DMAMUX_DMAREQ_ID_TIM8_CH4                = 35, /*!< dmamux channel dma request inputs resources: generator channel4 */
  DMAMUX_DMAREQ_ID_TIM8_UP                 = 36, /*!< dmamux channel dma request inputs resources: adc1 */
  DMAMUX_DMAREQ_ID_TIM8_COM                = 37, /*!< dmamux channel dma request inputs resources: adc2 */
  DMAMUX_DMAREQ_ID_TIM8_TRIG               = 38, /*!< dmamux channel dma request inputs resources: adc3 */

  DMAMUX_DMAREQ_ID_TIM9_CH1                = 39, /*!< dmamux channel dma request inputs resources: dac1 */
  DMAMUX_DMAREQ_ID_TIM9_CH2                = 40, /*!< dmamux channel dma request inputs resources: dac2 */
  DMAMUX_DMAREQ_ID_TIM9_CH3                = 41, /*!< dmamux channel dma request inputs resources: timer6 overflow */
  DMAMUX_DMAREQ_ID_TIM9_CH4                = 42, /*!< dmamux channel dma request inputs resources: timer7 overflow */
  DMAMUX_DMAREQ_ID_TIM9_UP                 = 43, /*!< dmamux channel dma request inputs resources: spi1 rx */
  DMAMUX_DMAREQ_ID_TIM9_TRIG               = 44, /*!< dmamux channel dma request inputs resources: spi1 tx */
 
  DMAMUX_DMAREQ_ID_TIM10_CH1               = 45, /*!< dmamux channel dma request inputs resources: dac1 */
  DMAMUX_DMAREQ_ID_TIM10_CH2               = 46, /*!< dmamux channel dma request inputs resources: dac2 */
  DMAMUX_DMAREQ_ID_TIM10_CH3               = 47, /*!< dmamux channel dma request inputs resources: timer6 overflow */
  DMAMUX_DMAREQ_ID_TIM10_CH4               = 48, /*!< dmamux channel dma request inputs resources: timer7 overflow */
  DMAMUX_DMAREQ_ID_TIM10_UP                = 49, /*!< dmamux channel dma request inputs resources: spi1 rx */
  DMAMUX_DMAREQ_ID_TIM10_TRIG              = 50, /*!< dmamux channel dma request inputs resources: spi1 tx */

  DMAMUX_DMAREQ_ID_TIM11_CH1               = 51, /*!< dmamux channel dma request inputs resources: dac1 */
  DMAMUX_DMAREQ_ID_TIM11_CH2               = 52, /*!< dmamux channel dma request inputs resources: dac2 */
  DMAMUX_DMAREQ_ID_TIM11_CH3               = 53, /*!< dmamux channel dma request inputs resources: timer6 overflow */
  DMAMUX_DMAREQ_ID_TIM11_CH4               = 54, /*!< dmamux channel dma request inputs resources: timer7 overflow */
  DMAMUX_DMAREQ_ID_TIM11_UP                = 55, /*!< dmamux channel dma request inputs resources: spi1 rx */
  DMAMUX_DMAREQ_ID_TIM11_TRIG              = 56, /*!< dmamux channel dma request inputs resources: spi1 tx */

  DMAMUX_DMAREQ_ID_TIM12_CH1               = 57, /*!< dmamux channel dma request inputs resources: dac1 */
  DMAMUX_DMAREQ_ID_TIM12_CH2               = 58, /*!< dmamux channel dma request inputs resources: dac2 */
  DMAMUX_DMAREQ_ID_TIM12_CH3               = 59, /*!< dmamux channel dma request inputs resources: timer6 overflow */
  DMAMUX_DMAREQ_ID_TIM12_CH4               = 60, /*!< dmamux channel dma request inputs resources: timer7 overflow */
  DMAMUX_DMAREQ_ID_TIM12_UP                = 61, /*!< dmamux channel dma request inputs resources: spi1 rx */
  DMAMUX_DMAREQ_ID_TIM12_TRIG              = 62, /*!< dmamux channel dma request inputs resources: spi1 tx */
  
  DMAMUX_DMAREQ_ID_SPI1_TX                 = 63,
  DMAMUX_DMAREQ_ID_SPI1_RX                 = 64,
  DMAMUX_DMAREQ_ID_SPI2_TX                 = 65,
  DMAMUX_DMAREQ_ID_SPI2_RX                 = 66,
  DMAMUX_DMAREQ_ID_SPI3_TX                 = 67,
  DMAMUX_DMAREQ_ID_SPI3_RX                 = 68,
  DMAMUX_DMAREQ_ID_SPI4_TX                 = 69,
  DMAMUX_DMAREQ_ID_SPI4_RX                 = 70,
  DMAMUX_DMAREQ_ID_QSPI1                   = 71,
  DMAMUX_DMAREQ_ID_QSPI2                   = 72,

  DMAMUX_DMAREQ_ID_I2C1_TX                 = 73,
  DMAMUX_DMAREQ_ID_I2C1_RX                 = 74,
  DMAMUX_DMAREQ_ID_I2C2_TX                 = 75,
  DMAMUX_DMAREQ_ID_I2C2_RX                 = 76,  
  DMAMUX_DMAREQ_ID_I2C3_TX                 = 77,
  DMAMUX_DMAREQ_ID_I2C3_RX                 = 78,
  DMAMUX_DMAREQ_ID_I2C4_TX                 = 79,
  DMAMUX_DMAREQ_ID_I2C4_RX                 = 80,

  DMAMUX_DMAREQ_ID_I3C_RS                  = 81,
  DMAMUX_DMAREQ_ID_I3C_TC                  = 82,
  DMAMUX_DMAREQ_ID_I3C_TX                  = 83,
  DMAMUX_DMAREQ_ID_I3C_RX                  = 84,
  
  DMAMUX_DMAREQ_ID_USART1_TX               = 85,
  DMAMUX_DMAREQ_ID_USART1_RX               = 86,  
  DMAMUX_DMAREQ_ID_USART2_TX               = 87,
  DMAMUX_DMAREQ_ID_USART2_RX               = 88,
  DMAMUX_DMAREQ_ID_USART3_TX               = 89,
  DMAMUX_DMAREQ_ID_USART3_RX               = 90,
  DMAMUX_DMAREQ_ID_USART4_TX               = 91,
  DMAMUX_DMAREQ_ID_USART4_RX               = 92,
  DMAMUX_DMAREQ_ID_USART5_TX               = 93,
  DMAMUX_DMAREQ_ID_USART5_RX               = 94,
  DMAMUX_DMAREQ_ID_USART6_TX               = 95,
  DMAMUX_DMAREQ_ID_USART6_RX               = 96,
  DMAMUX_DMAREQ_ID_USART7_TX               = 97,
  DMAMUX_DMAREQ_ID_USART7_RX               = 98,
  DMAMUX_DMAREQ_ID_USART8_TX               = 99,
  DMAMUX_DMAREQ_ID_USART8_RX               = 100,  

  DMAMUX_DMAREQ_ID_SPWMI_TX                = 101,
  DMAMUX_DMAREQ_ID_SPWMI_RX                = 102, 
  
  DMAMUX_DMAREQ_ID_DAC1                    = 103,
  DMAMUX_DMAREQ_ID_DAC2                    = 104, 

  DMAMUX_DMAREQ_ID_REV1                    = 105,
  DMAMUX_DMAREQ_ID_REV2                    = 106, 

  DMAMUX_DMAREQ_ID_DFSDM_DMA0              = 107,
  DMAMUX_DMAREQ_ID_DFSDM_DMA1              = 108, 

  DMAMUX_DMAREQ_ID_REV3                    = 109,
  DMAMUX_DMAREQ_ID_REV4                    = 110,  
  
  DMAMUX_DMAREQ_ID_SDIO                    = 111,    
  
  DMAMUX_DMAREQ_ID_SAIA_TX                 = 112,
  DMAMUX_DMAREQ_ID_SAIA_RX                 = 113, 
  DMAMUX_DMAREQ_ID_SAIB_TX                 = 114,
  DMAMUX_DMAREQ_ID_SAIB_RX                 = 115, 

  DMAMUX_DMAREQ_ID_REV5                    = 116,
  DMAMUX_DMAREQ_ID_REV6                    = 117,  
  DMAMUX_DMAREQ_ID_REV7                    = 118,
  DMAMUX_DMAREQ_ID_REV8                    = 119,   
  
  DMAMUX_DMAREQ_ID_ADC1                    = 120,
  DMAMUX_DMAREQ_ID_ADC2                    = 121,   

  DMAMUX_DMAREQ_ID_TIM6_UP                 = 122,
  DMAMUX_DMAREQ_ID_TIM7_UP                 = 123 
} dmamux_requst_id_sel_type;



/* DMA Init structure definition */
typedef struct
{
  uint32_t DMA_PeripheralBaseAddr; /* Specifies the peripheral base address for DMAy Channelx. */

  uint32_t DMA_Memory0BaseAddr;     /* Specifies the memory0 base address for DMAy Channelx. */

  uint32_t DMA_DIR;                /* Specifies if the peripheral is the source or destination.
                                      This parameter can be a value of @ref DMA_data_transfer_direction */

  uint32_t DMA_BufferSize;         /* Specifies the buffer size, in data unit, of the specified Channel. 
                                      The data unit is equal to the configuration set in DMA_PeripheralDataSize
                                      or DMA_MemoryDataSize members depending in the transfer direction. */

  uint32_t DMA_PeripheralInc;      /* Specifies whether the Peripheral address register is incremented or not.
                                      This parameter can be a value of @ref DMA_peripheral_incremented_mode */

  uint32_t DMA_MemoryInc;          /* Specifies whether the memory address register is incremented or not.
                                      This parameter can be a value of @ref DMA_memory_incremented_mode */

  uint32_t DMA_PeripheralDataSize; /* Specifies the Peripheral data width.
                                      This parameter can be a value of @ref DMA_peripheral_data_size */

  uint32_t DMA_MemoryDataSize;     /* Specifies the Memory data width.
                                      This parameter can be a value of @ref DMA_memory_data_size */

  uint32_t DMA_Mode;               /* Specifies the operation mode of the DMAy Channelx.
                                      This parameter can be a value of @ref DMA_circular_normal_mode.
                                      @note: The circular buffer mode cannot be used if the memory-to-memory
                                            data transfer is configured on the selected Channel */

  uint32_t DMA_Priority;           /* Specifies the software priority for the DMAy Channelx.
                                      This parameter can be a value of @ref DMA_priority_level */

  uint32_t DMA_M2M;                /* Specifies if the DMAy Channelx will be used in memory-to-memory transfer.
                                      This parameter can be a value of @ref DMA_memory_to_memory */

  uint32_t DMA_BufferMode;         /* Specifies if the DMAy Channelx will be used in buffer mode.
                                      This parameter can be a value of @ref DMA_BufferMode_Memory */

  uint32_t DMA_Memory1BaseAddr;     /* Specifies the memory1 base address for DMAy Channelx. */

  uint32_t DMA_DoubleBuffer_StartMemory;  /* Specifies if the DMAy Channelx will be used in double buffer start memory.
                                      This parameter can be a value of @ref DMA_DoubleBufferMode_Memory */

}DMA_InitTypeDef;




/* DMA_data_transfer_direction */
#define DMA_DIR_PeripheralDST              ((uint32_t)0x00000010)
#define DMA_DIR_PeripheralSRC              ((uint32_t)0x00000000)

/* DMA_peripheral_incremented_mode */
#define DMA_PeripheralInc_Enable           ((uint32_t)0x00000040)
#define DMA_PeripheralInc_Disable          ((uint32_t)0x00000000)
											
/* DMA_memory_incremented_mode */
#define DMA_MemoryInc_Enable               ((uint32_t)0x00000080)
#define DMA_MemoryInc_Disable              ((uint32_t)0x00000000)
										
/* DMA_peripheral_data_size */
#define DMA_PeripheralDataSize_Byte        ((uint32_t)0x00000000)
#define DMA_PeripheralDataSize_HalfWord    ((uint32_t)0x00000100)
#define DMA_PeripheralDataSize_Word        ((uint32_t)0x00000200)

/* DMA_memory_data_size */
#define DMA_MemoryDataSize_Byte            ((uint32_t)0x00000000)
#define DMA_MemoryDataSize_HalfWord        ((uint32_t)0x00000400)
#define DMA_MemoryDataSize_Word            ((uint32_t)0x00000800)

/* DMA_circular_normal_mode */
#define DMA_Mode_Circular                  ((uint32_t)0x00000020)
#define DMA_Mode_Normal                    ((uint32_t)0x00000000)

/* DMA_priority_level */
#define DMA_Priority_VeryHigh              ((uint32_t)0x00003000)
#define DMA_Priority_High                  ((uint32_t)0x00002000)
#define DMA_Priority_Medium                ((uint32_t)0x00001000)
#define DMA_Priority_Low                   ((uint32_t)0x00000000)

/* DMA_memory_to_memory */
#define DMA_M2M_Enable                     ((uint32_t)0x00004000)
#define DMA_M2M_Disable                    ((uint32_t)0x00000000)

/* DMA_interrupts_definition */
#define DMA_IT_TC                          ((uint32_t)0x00000002)
#define DMA_IT_HT                          ((uint32_t)0x00000004)
#define DMA_IT_TE                          ((uint32_t)0x00000008)

#define DMA1_IT_GL1                        ((uint32_t)0x00000001)
#define DMA1_IT_TC1                        ((uint32_t)0x00000002)
#define DMA1_IT_HT1                        ((uint32_t)0x00000004)
#define DMA1_IT_TE1                        ((uint32_t)0x00000008)
#define DMA1_IT_GL2                        ((uint32_t)0x00000010)
#define DMA1_IT_TC2                        ((uint32_t)0x00000020)
#define DMA1_IT_HT2                        ((uint32_t)0x00000040)
#define DMA1_IT_TE2                        ((uint32_t)0x00000080)
#define DMA1_IT_GL3                        ((uint32_t)0x00000100)
#define DMA1_IT_TC3                        ((uint32_t)0x00000200)
#define DMA1_IT_HT3                        ((uint32_t)0x00000400)
#define DMA1_IT_TE3                        ((uint32_t)0x00000800)
#define DMA1_IT_GL4                        ((uint32_t)0x00001000)
#define DMA1_IT_TC4                        ((uint32_t)0x00002000)
#define DMA1_IT_HT4                        ((uint32_t)0x00004000)
#define DMA1_IT_TE4                        ((uint32_t)0x00008000)
#define DMA1_IT_GL5                        ((uint32_t)0x00010000)
#define DMA1_IT_TC5                        ((uint32_t)0x00020000)
#define DMA1_IT_HT5                        ((uint32_t)0x00040000)
#define DMA1_IT_TE5                        ((uint32_t)0x00080000)
#define DMA1_IT_GL6                        ((uint32_t)0x00100000)
#define DMA1_IT_TC6                        ((uint32_t)0x00200000)
#define DMA1_IT_HT6                        ((uint32_t)0x00400000)
#define DMA1_IT_TE6                        ((uint32_t)0x00800000)
#define DMA1_IT_GL7                        ((uint32_t)0x01000000)
#define DMA1_IT_TC7                        ((uint32_t)0x02000000)
#define DMA1_IT_HT7                        ((uint32_t)0x04000000)
#define DMA1_IT_TE7                        ((uint32_t)0x08000000)
#define DMA1_IT_GL8                        ((uint32_t)0x10000000)
#define DMA1_IT_TC8                        ((uint32_t)0x20000000)
#define DMA1_IT_HT8                        ((uint32_t)0x40000000)
#define DMA1_IT_TE8                        ((uint32_t)0x80000000)

#define DMA2_IT_GL1                        ((uint32_t)0x00000001)
#define DMA2_IT_TC1                        ((uint32_t)0x00000002)
#define DMA2_IT_HT1                        ((uint32_t)0x00000004)
#define DMA2_IT_TE1                        ((uint32_t)0x00000008)
#define DMA2_IT_GL2                        ((uint32_t)0x00000010)
#define DMA2_IT_TC2                        ((uint32_t)0x00000020)
#define DMA2_IT_HT2                        ((uint32_t)0x00000040)
#define DMA2_IT_TE2                        ((uint32_t)0x00000080)
#define DMA2_IT_GL3                        ((uint32_t)0x00000100)
#define DMA2_IT_TC3                        ((uint32_t)0x00000200)
#define DMA2_IT_HT3                        ((uint32_t)0x00000400)
#define DMA2_IT_TE3                        ((uint32_t)0x00000800)
#define DMA2_IT_GL4                        ((uint32_t)0x00001000)
#define DMA2_IT_TC4                        ((uint32_t)0x00002000)
#define DMA2_IT_HT4                        ((uint32_t)0x00004000)
#define DMA2_IT_TE4                        ((uint32_t)0x00008000)
#define DMA2_IT_GL5                        ((uint32_t)0x00010000)
#define DMA2_IT_TC5                        ((uint32_t)0x00020000)
#define DMA2_IT_HT5                        ((uint32_t)0x00040000)
#define DMA2_IT_TE5                        ((uint32_t)0x00080000)
#define DMA2_IT_GL6                        ((uint32_t)0x00100000)
#define DMA2_IT_TC6                        ((uint32_t)0x00200000)
#define DMA2_IT_HT6                        ((uint32_t)0x00400000)
#define DMA2_IT_TE6                        ((uint32_t)0x00800000)
#define DMA2_IT_GL7                        ((uint32_t)0x01000000)
#define DMA2_IT_TC7                        ((uint32_t)0x02000000)
#define DMA2_IT_HT7                        ((uint32_t)0x04000000)
#define DMA2_IT_TE7                        ((uint32_t)0x08000000)
#define DMA2_IT_GL8                        ((uint32_t)0x10000000)
#define DMA2_IT_TC8                        ((uint32_t)0x20000000)
#define DMA2_IT_HT8                        ((uint32_t)0x40000000)
#define DMA2_IT_TE8                        ((uint32_t)0x80000000)

/* DMA_flags_definition */
#define DMA1_FLAG_GL1                      ((uint32_t)0x00000001)
#define DMA1_FLAG_TC1                      ((uint32_t)0x00000002)
#define DMA1_FLAG_HT1                      ((uint32_t)0x00000004)
#define DMA1_FLAG_TE1                      ((uint32_t)0x00000008)
#define DMA1_FLAG_GL2                      ((uint32_t)0x00000010)
#define DMA1_FLAG_TC2                      ((uint32_t)0x00000020)
#define DMA1_FLAG_HT2                      ((uint32_t)0x00000040)
#define DMA1_FLAG_TE2                      ((uint32_t)0x00000080)
#define DMA1_FLAG_GL3                      ((uint32_t)0x00000100)
#define DMA1_FLAG_TC3                      ((uint32_t)0x00000200)
#define DMA1_FLAG_HT3                      ((uint32_t)0x00000400)
#define DMA1_FLAG_TE3                      ((uint32_t)0x00000800)
#define DMA1_FLAG_GL4                      ((uint32_t)0x00001000)
#define DMA1_FLAG_TC4                      ((uint32_t)0x00002000)
#define DMA1_FLAG_HT4                      ((uint32_t)0x00004000)
#define DMA1_FLAG_TE4                      ((uint32_t)0x00008000)
#define DMA1_FLAG_GL5                      ((uint32_t)0x00010000)
#define DMA1_FLAG_TC5                      ((uint32_t)0x00020000)
#define DMA1_FLAG_HT5                      ((uint32_t)0x00040000)
#define DMA1_FLAG_TE5                      ((uint32_t)0x00080000)
#define DMA1_FLAG_GL6                      ((uint32_t)0x00100000)
#define DMA1_FLAG_TC6                      ((uint32_t)0x00200000)
#define DMA1_FLAG_HT6                      ((uint32_t)0x00400000)
#define DMA1_FLAG_TE6                      ((uint32_t)0x00800000)
#define DMA1_FLAG_GL7                      ((uint32_t)0x01000000)
#define DMA1_FLAG_TC7                      ((uint32_t)0x02000000)
#define DMA1_FLAG_HT7                      ((uint32_t)0x04000000)
#define DMA1_FLAG_TE7                      ((uint32_t)0x08000000)
#define DMA1_FLAG_GL8                      ((uint32_t)0x10000000)
#define DMA1_FLAG_TC8                      ((uint32_t)0x20000000)
#define DMA1_FLAG_HT8                      ((uint32_t)0x40000000)
#define DMA1_FLAG_TE8                      ((uint32_t)0x80000000)

#define DMA2_FLAG_GL1                      ((uint32_t)0x00000001)
#define DMA2_FLAG_TC1                      ((uint32_t)0x00000002)
#define DMA2_FLAG_HT1                      ((uint32_t)0x00000004)
#define DMA2_FLAG_TE1                      ((uint32_t)0x00000008)
#define DMA2_FLAG_GL2                      ((uint32_t)0x00000010)
#define DMA2_FLAG_TC2                      ((uint32_t)0x00000020)
#define DMA2_FLAG_HT2                      ((uint32_t)0x00000040)
#define DMA2_FLAG_TE2                      ((uint32_t)0x00000080)
#define DMA2_FLAG_GL3                      ((uint32_t)0x00000100)
#define DMA2_FLAG_TC3                      ((uint32_t)0x00000200)
#define DMA2_FLAG_HT3                      ((uint32_t)0x00000400)
#define DMA2_FLAG_TE3                      ((uint32_t)0x00000800)
#define DMA2_FLAG_GL4                      ((uint32_t)0x00001000)
#define DMA2_FLAG_TC4                      ((uint32_t)0x00002000)
#define DMA2_FLAG_HT4                      ((uint32_t)0x00004000)
#define DMA2_FLAG_TE4                      ((uint32_t)0x00008000)
#define DMA2_FLAG_GL5                      ((uint32_t)0x00010000)
#define DMA2_FLAG_TC5                      ((uint32_t)0x00020000)
#define DMA2_FLAG_HT5                      ((uint32_t)0x00040000)
#define DMA2_FLAG_TE5                      ((uint32_t)0x00080000)
#define DMA2_FLAG_GL6                      ((uint32_t)0x00100000)
#define DMA2_FLAG_TC6                      ((uint32_t)0x00200000)
#define DMA2_FLAG_HT6                      ((uint32_t)0x00400000)
#define DMA2_FLAG_TE6                      ((uint32_t)0x00800000)
#define DMA2_FLAG_GL7                      ((uint32_t)0x01000000)
#define DMA2_FLAG_TC7                      ((uint32_t)0x02000000)
#define DMA2_FLAG_HT7                      ((uint32_t)0x04000000)
#define DMA2_FLAG_TE7                      ((uint32_t)0x08000000)
#define DMA2_FLAG_GL8                      ((uint32_t)0x10000000)
#define DMA2_FLAG_TC8                      ((uint32_t)0x20000000)
#define DMA2_FLAG_HT8                      ((uint32_t)0x40000000)
#define DMA2_FLAG_TE8                      ((uint32_t)0x80000000)

/* DMA_MuxChannel_definition */
#define DMA_MuxChannel1                    ((uint8_t)0x00)
#define DMA_MuxChannel2                    ((uint8_t)0x01)
#define DMA_MuxChannel3                    ((uint8_t)0x02)
#define DMA_MuxChannel4                    ((uint8_t)0x03)
#define DMA_MuxChannel5                    ((uint8_t)0x04)
#define DMA_MuxChannel6                    ((uint8_t)0x05)
#define DMA_MuxChannel7                    ((uint8_t)0x06)
#define DMA_MuxChannel8                    ((uint8_t)0x07)
#define DMA_MuxChannel9                    ((uint8_t)0x08)
#define DMA_MuxChannel10                   ((uint8_t)0x09)
#define DMA_MuxChannel11                   ((uint8_t)0x0A)
#define DMA_MuxChannel12                   ((uint8_t)0x0B)
#define DMA_MuxChannel13                   ((uint8_t)0x0C)
#define DMA_MuxChannel14                   ((uint8_t)0x0D)
#define DMA_MuxChannel15                   ((uint8_t)0x0E)
#define DMA_MuxChannel16                   ((uint8_t)0x0F)



/* DMA_BufferMode_Memory */
#define DMA_SingleBufferMode               ((uint32_t)0x00000000)
#define DMA_DoubleBufferMode               ((uint32_t)0x00008000)

/* DMA_DoubleBufferMode_Memory */
#define DMA_DoubleBufferMode_Memory_0      ((uint32_t)0x00000000)
#define DMA_DoubleBufferMode_Memory_1      ((uint32_t)0x00010000)

void DMA_DeInit(DMA_Channel_TypeDef* DMAy_Channelx);
void DMA_Init(DMA_Channel_TypeDef* DMAy_Channelx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct);
void DMA_Cmd(DMA_Channel_TypeDef* DMAy_Channelx, FunctionalState NewState);
void DMA_ITConfig(DMA_Channel_TypeDef* DMAy_Channelx, uint32_t DMA_IT, FunctionalState NewState);
void DMA_SetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx, uint16_t DataNumber); 
uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx);
FlagStatus DMA_GetFlagStatus(DMA_TypeDef* DMAx, uint32_t DMAy_FLAG);
void DMA_ClearFlag(DMA_TypeDef* DMAx, uint32_t DMAy_FLAG);
ITStatus DMA_GetITStatus(DMA_TypeDef* DMAx, uint32_t DMAy_IT);
void DMA_ClearITPendingBit(DMA_TypeDef* DMAx, uint32_t DMAy_IT);
void DMA_MuxChannelConfig(uint8_t DMA_MuxChannelx, uint32_t DMA_Requestx);


#ifdef __cplusplus
}
#endif

#endif 

