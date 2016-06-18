/**
  ******************************************************************************
  * @file    stm32f4xx_qspi.h
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    21-October-2015
  * @brief   This file contains all the functions prototypes for the QSPI 
  *          firmware library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4XX_QUADSPI_H
#define __STM32F4XX_QUADSPI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @addtogroup QSPI
  * @{
  */
#if defined(STM32F446xx) || defined(STM32F469_479xx)
/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  QSPI Communication Configuration Init structure definition  
  */

typedef struct
{
  
  uint32_t QSPI_ComConfig_FMode;            /* Specifies the Functional Mode
                                          This parameter can be a value of @ref QSPI_ComConfig_Functional_Mode*/  
	
  uint32_t QSPI_ComConfig_DDRMode;          /* Specifies the Double Data Rate Mode
                                          This parameter can be a value of @ref QSPI_ComConfig_DoubleDataRateMode*/
	
  uint32_t QSPI_ComConfig_DHHC;            /* Specifies the Delay Half Hclk Cycle
                                          This parameter can be a value of @ref QSPI_ComConfig_DelayHalfHclkCycle*/
  
  uint32_t QSPI_ComConfig_SIOOMode;         /* Specifies the Send Instruction Only Once Mode
                                          This parameter can be a value of @ref QSPI_ComConfig_SendInstructionOnlyOnceMode*/
  
  uint32_t QSPI_ComConfig_DMode;            /* Specifies the Data Mode
                                          This parameter can be a value of @ref QSPI_ComConfig_DataMode*/
  
  uint32_t QSPI_ComConfig_DummyCycles;      /* Specifies the Number of Dummy Cycles.
                                           This parameter can be a number between 0x00 and 0x1F */             

  uint32_t QSPI_ComConfig_ABSize;           /* Specifies the Alternate Bytes Size
                                          This parameter can be a value of @ref QSPI_ComConfig_AlternateBytesSize*/ 

  uint32_t QSPI_ComConfig_ABMode;           /* Specifies the Alternate Bytes Mode
                                          This parameter can be a value of @ref QSPI_ComConfig_AlternateBytesMode*/     
 
  uint32_t QSPI_ComConfig_ADSize;           /* Specifies the Address Size
                                          This parameter can be a value of @ref QSPI_ComConfig_AddressSize*/

  uint32_t QSPI_ComConfig_ADMode;           /* Specifies the Address Mode
                                          This parameter can be a value of @ref QSPI_ComConfig_AddressMode*/
  
  uint32_t QSPI_ComConfig_IMode;            /* Specifies the Instruction Mode
                                          This parameter can be a value of @ref QSPI_ComConfig_InstructionMode*/

  uint32_t QSPI_ComConfig_Ins;      /* Specifies the Instruction Mode
                                          This parameter can be a value of @ref QSPI_ComConfig_Instruction*/
  
}QSPI_ComConfig_InitTypeDef;

/** 
  * @brief  QSPI Init structure definition  
  */

typedef struct
{
  uint32_t QSPI_SShift;    /* Specifies the Sample Shift
                              This parameter can be a value of @ref QSPI_Sample_Shift*/

  uint32_t QSPI_Prescaler; /* Specifies the prescaler value used to divide the QSPI clock.
                              This parameter can be a number between 0x00 and 0xFF */ 

  uint32_t QSPI_CKMode;    /* Specifies the Clock Mode
                              This parameter can be a value of @ref QSPI_Clock_Mode*/

  uint32_t QSPI_CSHTime;   /* Specifies the Chip Select High Time
                              This parameter can be a value of @ref QSPI_ChipSelectHighTime*/   
 
  uint32_t QSPI_FSize;     /* Specifies the Flash Size.
                               QSPI_FSize+1 is effectively the number of address bits required to address the flash memory.
                               The flash capacity can be up to 4GB (addressed using 32 bits) in indirect mode, but the
                               addressable space in memory-mapped mode is limited to 512MB
                               This parameter can be a number between 0x00 and 0x1F */
  uint32_t QSPI_FSelect;   /* Specifies the Flash which will be used,
                                 This parameter can be a value of @ref QSPI_Fash_Select*/
  uint32_t QSPI_DFlash;    /* Specifies the Dual Flash Mode State
                                 This parameter can be a value of @ref QSPI_Dual_Flash*/
}QSPI_InitTypeDef;

/* Exported constants --------------------------------------------------------*/

/** @defgroup QSPI_Exported_Constants
  * @{
  */

/** @defgroup QSPI_Sample_Shift 
  * @{
  */
#define QSPI_SShift_NoShift                        ((uint32_t)0x00000000)
#define QSPI_SShift_HalfCycleShift                 ((uint32_t)QUADSPI_CR_SSHIFT_0)
#define QSPI_SShift_OneCycleShift                  ((uint32_t)QUADSPI_CR_SSHIFT_1)
#define QSPI_SShift_OneAndHalfCycleShift           ((uint32_t)QUADSPI_CR_SSHIFT)
#define IS_QSPI_SSHIFT(SSHIFT)      (((SSHIFT) == QSPI_SShift_NoShift) ||  ((SSHIFT) == QSPI_SShift_HalfCycleShift) || \
                                     ((SSHIFT) == QSPI_SShift_OneCycleShift) || ((SSHIFT) == QSPI_SShift_OneAndHalfCycleShift))
/**
  * @}
  */
  
/** @defgroup QSPI_Prescaler 
  * @{
  */
#define IS_QSPI_PRESCALER(PRESCALER) (((PRESCALER) <= 0xFF))
/**
  * @}
  */

/** @defgroup QSPI_Clock_Mode 
  * @{
  */
#define QSPI_CKMode_Mode0           ((uint32_t)0x00000000)
#define QSPI_CKMode_Mode3           ((uint32_t)QUADSPI_DCR_CKMODE)
#define IS_QSPI_CKMODE(CKMode)      (((CKMode) == QSPI_CKMode_Mode0) || ((CKMode) == QSPI_CKMode_Mode3))
/**
  * @}
  */
  
/** @defgroup QSPI_ChipSelectHighTime 
  * @{
  */
#define QSPI_CSHTime_1Cycle           ((uint32_t)0x00000000)
#define QSPI_CSHTime_2Cycle           ((uint32_t)QUADSPI_DCR_CSHT_0)
#define QSPI_CSHTime_3Cycle           ((uint32_t)QUADSPI_DCR_CSHT_1)
#define QSPI_CSHTime_4Cycle           ((uint32_t)QUADSPI_DCR_CSHT_0 | QUADSPI_DCR_CSHT_1)
#define QSPI_CSHTime_5Cycle           ((uint32_t)QUADSPI_DCR_CSHT_2)
#define QSPI_CSHTime_6Cycle           ((uint32_t)QUADSPI_DCR_CSHT_2 | QUADSPI_DCR_CSHT_0)
#define QSPI_CSHTime_7Cycle           ((uint32_t)QUADSPI_DCR_CSHT_2 | QUADSPI_DCR_CSHT_1)
#define QSPI_CSHTime_8Cycle           ((uint32_t)QUADSPI_DCR_CSHT)
#define IS_QSPI_CSHTIME(CSHTIME)    (((CSHTIME) == QSPI_CSHTime_1Cycle) || \
                                     ((CSHTIME) == QSPI_CSHTime_2Cycle) || \
                                     ((CSHTIME) == QSPI_CSHTime_3Cycle) || \
                                     ((CSHTIME) == QSPI_CSHTime_4Cycle) || \
                                     ((CSHTIME) == QSPI_CSHTime_5Cycle) || \
                                     ((CSHTIME) == QSPI_CSHTime_6Cycle) || \
                                     ((CSHTIME) == QSPI_CSHTime_7Cycle) || \
                                     ((CSHTIME) == QSPI_CSHTime_8Cycle))
/**
  * @}
  */
  
/** @defgroup QSPI_Flash_Size 
  * @{
  */
#define IS_QSPI_FSIZE(FSIZE)         (((FSIZE) <= 0x1F))
/**
  * @}
  */
	
/** @defgroup QSPI_Fash_Select 
  * @{
  */
#define QSPI_FSelect_1           ((uint32_t)0x00000000)
#define QSPI_FSelect_2           ((uint32_t)QUADSPI_CR_FSEL)
#define IS_QSPI_FSEL(FLA)    (((FLA) == QSPI_FSelect_1) || ((FLA) == QSPI_FSelect_2))
/**
  * @}
  */
	
/** @defgroup QSPI_Dual_Flash 
  * @{
  */
#define QSPI_DFlash_Disable           ((uint32_t)0x00000000)
#define QSPI_DFlash_Enable            ((uint32_t)QUADSPI_CR_DFM)
#define IS_QSPI_DFM(FLA)    (((FLA) == QSPI_DFlash_Enable) || ((FLA) == QSPI_DFlash_Disable))
/**
  * @}
  */

/** @defgroup QSPI_ComConfig_Functional_Mode 
  * @{
  */  
#define QSPI_ComConfig_FMode_Indirect_Write     ((uint32_t)0x00000000)
#define QSPI_ComConfig_FMode_Indirect_Read      ((uint32_t)QUADSPI_CCR_FMODE_0)
#define QSPI_ComConfig_FMode_Auto_Polling       ((uint32_t)QUADSPI_CCR_FMODE_1)
#define QSPI_ComConfig_FMode_Memory_Mapped      ((uint32_t)QUADSPI_CCR_FMODE)
#define IS_QSPI_FMODE(FMODE)                  (((FMODE) == QSPI_ComConfig_FMode_Indirect_Write) || \
                                               ((FMODE) == QSPI_ComConfig_FMode_Indirect_Read) || \
                                               ((FMODE) == QSPI_ComConfig_FMode_Auto_Polling) || \
                                               ((FMODE) == QSPI_ComConfig_FMode_Memory_Mapped))
/**
  * @}
  */
	
/** @defgroup QSPI_ComConfig_DoubleDataRateMode 
  * @{
  */
#define QSPI_ComConfig_DDRMode_Disable           ((uint32_t)0x00000000)
#define QSPI_ComConfig_DDRMode_Enable            ((uint32_t)QUADSPI_CCR_DDRM)
#define IS_QSPI_DDRMODE(DDRMODE)    (((DDRMODE) == QSPI_ComConfig_DDRMode_Disable) || \
                                     ((DDRMODE) == QSPI_ComConfig_DDRMode_Enable))
/**
  * @}
  */
	
/** @defgroup QSPI_ComConfig_DelayHalfHclkCycle 
  * @{
  */
#define QSPI_ComConfig_DHHC_Disable           ((uint32_t)0x00000000)
#define QSPI_ComConfig_DHHC_Enable            ((uint32_t)QUADSPI_CCR_DHHC)
#define IS_QSPI_DHHC(DHHC)          (((DHHC) == QSPI_ComConfig_DHHC_Disable) || \
                                     ((DHHC) == QSPI_ComConfig_DHHC_Enable))
/**
  * @}
  */
  
/** @defgroup QSPI_ComConfig_SendInstructionOnlyOnceMode 
  * @{
  */
#define QSPI_ComConfig_SIOOMode_Disable           ((uint32_t)0x00000000)
#define QSPI_ComConfig_SIOOMode_Enable            ((uint32_t)QUADSPI_CCR_SIOO)
#define IS_QSPI_SIOOMODE(SIOOMODE)    (((SIOOMODE) == QSPI_ComConfig_SIOOMode_Disable) || \
                                       ((SIOOMODE) == QSPI_ComConfig_SIOOMode_Enable))
/**
  * @}
  */  

/** @defgroup QSPI_ComConfig_DataMode 
  * @{
  */
#define QSPI_ComConfig_DMode_NoData          ((uint32_t)0x00000000)
#define QSPI_ComConfig_DMode_1Line           ((uint32_t)QUADSPI_CCR_DMODE_0)
#define QSPI_ComConfig_DMode_2Line           ((uint32_t)QUADSPI_CCR_DMODE_1)
#define QSPI_ComConfig_DMode_4Line           ((uint32_t)QUADSPI_CCR_DMODE)
#define IS_QSPI_DMODE(DMODE)        (((DMODE) == QSPI_ComConfig_DMode_NoData) || \
                                     ((DMODE) == QSPI_ComConfig_DMode_1Line) || \
                                     ((DMODE) == QSPI_ComConfig_DMode_2Line) || \
                                     ((DMODE) == QSPI_ComConfig_DMode_4Line))
/**
  * @}
  */
  
/** @defgroup QSPI_ComConfig_AlternateBytesSize 
  * @{
  */
#define QSPI_ComConfig_ABSize_8bit            ((uint32_t)0x00000000)
#define QSPI_ComConfig_ABSize_16bit           ((uint32_t)QUADSPI_CCR_ABSIZE_0)
#define QSPI_ComConfig_ABSize_24bit           ((uint32_t)QUADSPI_CCR_ABSIZE_1)
#define QSPI_ComConfig_ABSize_32bit           ((uint32_t)QUADSPI_CCR_ABSIZE)
#define IS_QSPI_ABSIZE(ABSIZE)      (((ABSIZE) == QSPI_ComConfig_ABSize_8bit) || \
                                     ((ABSIZE) == QSPI_ComConfig_ABSize_16bit) || \
                                     ((ABSIZE) == QSPI_ComConfig_ABSize_24bit) || \
                                     ((ABSIZE) == QSPI_ComConfig_ABSize_32bit))
/**
  * @}
  */
  
/** @defgroup QSPI_ComConfig_AlternateBytesMode 
  * @{
  */
#define QSPI_ComConfig_ABMode_NoAlternateByte          ((uint32_t)0x00000000)
#define QSPI_ComConfig_ABMode_1Line                    ((uint32_t)QUADSPI_CCR_ABMODE_0)
#define QSPI_ComConfig_ABMode_2Line                    ((uint32_t)QUADSPI_CCR_ABMODE_1)
#define QSPI_ComConfig_ABMode_4Line                    ((uint32_t)QUADSPI_CCR_ABMODE)
#define IS_QSPI_ABMODE(ABMODE)      (((ABMODE) == QSPI_ComConfig_ABMode_NoAlternateByte) || \
                                     ((ABMODE) == QSPI_ComConfig_ABMode_1Line) || \
                                     ((ABMODE) == QSPI_ComConfig_ABMode_2Line) || \
                                     ((ABMODE) == QSPI_ComConfig_ABMode_4Line))
/**
  * @}
  */
  
/** @defgroup QSPI_ComConfig_AddressSize 
  * @{
  */
#define QSPI_ComConfig_ADSize_8bit            ((uint32_t)0x00000000)
#define QSPI_ComConfig_ADSize_16bit           ((uint32_t)QUADSPI_CCR_ADSIZE_0)
#define QSPI_ComConfig_ADSize_24bit           ((uint32_t)QUADSPI_CCR_ADSIZE_1)
#define QSPI_ComConfig_ADSize_32bit           ((uint32_t)QUADSPI_CCR_ADSIZE)
#define IS_QSPI_ADSIZE(ADSIZE)      (((ADSIZE) == QSPI_ComConfig_ADSize_8bit) || \
                                     ((ADSIZE) == QSPI_ComConfig_ADSize_16bit) || \
                                     ((ADSIZE) == QSPI_ComConfig_ADSize_24bit) || \
                                     ((ADSIZE) == QSPI_ComConfig_ADSize_32bit))
/**
  * @}
  */
  
/** @defgroup QSPI_ComConfig_AddressMode 
  * @{
  */
#define QSPI_ComConfig_ADMode_NoAddress          ((uint32_t)0x00000000)
#define QSPI_ComConfig_ADMode_1Line              ((uint32_t)QUADSPI_CCR_ADMODE_0)
#define QSPI_ComConfig_ADMode_2Line              ((uint32_t)QUADSPI_CCR_ADMODE_1)
#define QSPI_ComConfig_ADMode_4Line              ((uint32_t)QUADSPI_CCR_ADMODE)
#define IS_QSPI_ADMODE(ADMODE)        (((ADMODE) == QSPI_ComConfig_ADMode_NoAddress) || \
                                       ((ADMODE) == QSPI_ComConfig_ADMode_1Line) || \
                                       ((ADMODE) == QSPI_ComConfig_ADMode_2Line) || \
                                       ((ADMODE) == QSPI_ComConfig_ADMode_4Line))
/**
  * @}
  */  
  
/** @defgroup QSPI_ComConfig_InstructionMode 
  * @{
  */
#define QSPI_ComConfig_IMode_NoInstruction          ((uint32_t)0x00000000)
#define QSPI_ComConfig_IMode_1Line                  ((uint32_t)QUADSPI_CCR_IMODE_0)
#define QSPI_ComConfig_IMode_2Line                  ((uint32_t)QUADSPI_CCR_IMODE_1)
#define QSPI_ComConfig_IMode_4Line                  ((uint32_t)QUADSPI_CCR_IMODE)
#define IS_QSPI_IMODE(IMODE)        (((IMODE) == QSPI_ComConfig_IMode_NoInstruction) || \
                                     ((IMODE) == QSPI_ComConfig_IMode_1Line) || \
                                     ((IMODE) == QSPI_ComConfig_IMode_2Line) || \
                                     ((IMODE) == QSPI_ComConfig_IMode_4Line))
/**
  * @}
  */

/** @defgroup QSPI_ComConfig_Instruction 
  * @{
  */
#define IS_QSPI_INSTRUCTION(INSTRUCTION)           ((INSTRUCTION) <= 0xFF) 
/**
  * @}
  */ 

/** @defgroup QSPI_InterruptsDefinition 
  * @{
  */
#define QSPI_IT_TO                (uint32_t)(QUADSPI_CR_TOIE | QUADSPI_SR_TOF)
#define QSPI_IT_SM                (uint32_t)(QUADSPI_CR_SMIE | QUADSPI_SR_SMF)
#define QSPI_IT_FT                (uint32_t)(QUADSPI_CR_FTIE | QUADSPI_SR_FTF)
#define QSPI_IT_TC                (uint32_t)(QUADSPI_CR_TCIE | QUADSPI_SR_TCF)
#define QSPI_IT_TE                (uint32_t)(QUADSPI_CR_TEIE | QUADSPI_SR_TEF)
#define IS_QSPI_IT(IT)   ((((IT) & 0xFFE0FFE0) == 0) && ((IT) != 0))
#define IS_QSPI_CLEAR_IT(IT)   ((((IT) & 0xFFE4FFE4) == 0) && ((IT) != 0))  
/**
  * @}
  */
  
/** @defgroup QSPI_FlagsDefinition 
  * @{
  */
#define QSPI_FLAG_TO                  QUADSPI_SR_TOF
#define QSPI_FLAG_SM                  QUADSPI_SR_SMF
#define QSPI_FLAG_FT                  QUADSPI_SR_FTF
#define QSPI_FLAG_TC                  QUADSPI_SR_TCF
#define QSPI_FLAG_TE                  QUADSPI_SR_TEF
#define QSPI_FLAG_BUSY                QUADSPI_SR_BUSY
#define IS_QSPI_GET_FLAG(FLAG)   (((FLAG) == QSPI_FLAG_TO) || ((FLAG) == QSPI_FLAG_SM) || \
                                  ((FLAG) == QSPI_FLAG_FT) || ((FLAG) == QSPI_FLAG_TC) || \
                                  ((FLAG) == QSPI_FLAG_TE) || ((FLAG) == QSPI_FLAG_BUSY))
#define IS_QSPI_CLEAR_FLAG(FLAG)  (((FLAG) == QSPI_FLAG_TO) || ((FLAG) == QSPI_FLAG_SM) || \
                                  ((FLAG) == QSPI_FLAG_TC) ||  ((FLAG) == QSPI_FLAG_TE))

/**
  * @}
  */
  
/** @defgroup QSPI_Polling_Match_Mode 
  * @{
  */
#define QSPI_PMM_AND                 ((uint32_t)0x00000000)
#define QSPI_PMM_OR                  ((uint32_t)QUADSPI_CR_PMM)
#define IS_QSPI_PMM(PMM)      (((PMM) == QSPI_PMM_AND) || ((PMM) == QSPI_PMM_OR)) 
/**
  * @}
  */
  
/** @defgroup QSPI_Polling_Interval 
  * @{
  */
#define IS_QSPI_PIR(PIR)                 ((PIR) <= QUADSPI_PIR_INTERVAL) 
/**
  * @}
  */
  
/** @defgroup QSPI_Timeout 
  * @{
  */
#define IS_QSPI_TIMEOUT(TIMEOUT)         ((TIMEOUT) <= QUADSPI_LPTR_TIMEOUT) 
/**
  * @}
  */
  
/** @defgroup QSPI_DummyCycle 
  * @{
  */
#define IS_QSPI_DCY(DCY)                 ((DCY) <= 0x1F) 
/**
  * @}
  */
  
/** @defgroup QSPI_FIFOThreshold 
  * @{
  */
#define IS_QSPI_FIFOTHRESHOLD(FIFOTHRESHOLD)        ((FIFOTHRESHOLD) <= 0x0F) 
/**
  * @}
  */  

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Initialization and Configuration functions *********************************/
void QSPI_DeInit(void);
void QSPI_Init(QSPI_InitTypeDef* QSPI_InitStruct);
void QSPI_StructInit(QSPI_InitTypeDef* QSPI_InitStruct);
void QSPI_ComConfig_Init(QSPI_ComConfig_InitTypeDef* QSPI_ComConfig_InitStruct);
void QSPI_ComConfig_StructInit(QSPI_ComConfig_InitTypeDef* QSPI_ComConfig_InitStruct);
void QSPI_Cmd(FunctionalState NewState);
void QSPI_AutoPollingMode_Config(uint32_t QSPI_Match, uint32_t QSPI_Mask , uint32_t QSPI_Match_Mode);
void QSPI_AutoPollingMode_SetInterval(uint32_t QSPI_Interval);
void QSPI_MemoryMappedMode_SetTimeout(uint32_t QSPI_Timeout);
void QSPI_SetAddress(uint32_t QSPI_Address);
void QSPI_SetAlternateByte(uint32_t QSPI_AlternateByte);
void QSPI_SetFIFOThreshold(uint32_t QSPI_FIFOThreshold);
void QSPI_SetDataLength(uint32_t QSPI_DataLength);
void QSPI_TimeoutCounterCmd(FunctionalState NewState);
void QSPI_AutoPollingModeStopCmd(FunctionalState NewState);
void QSPI_AbortRequest(void);
void QSPI_DualFlashMode_Cmd(FunctionalState NewState);

/* Data transfers functions ***************************************************/
void     QSPI_SendData8(uint8_t Data);
void     QSPI_SendData16(uint16_t Data);
void     QSPI_SendData32(uint32_t Data);
uint8_t  QSPI_ReceiveData8(void);
uint16_t QSPI_ReceiveData16(void);
uint32_t QSPI_ReceiveData32(void);

/* DMA transfers management functions *****************************************/
void QSPI_DMACmd(FunctionalState NewState);

/* Interrupts and flags management functions **********************************/
void       QSPI_ITConfig(uint32_t QSPI_IT, FunctionalState NewState);
uint32_t   QSPI_GetFIFOLevel(void);
FlagStatus QSPI_GetFlagStatus(uint32_t QSPI_FLAG);
void       QSPI_ClearFlag(uint32_t QSPI_FLAG);
ITStatus   QSPI_GetITStatus(uint32_t QSPI_IT);
void       QSPI_ClearITPendingBit(uint32_t QSPI_IT);
uint32_t   QSPI_GetFMode(void);

#endif /* STM32F446xx || STM32F469_479xx */
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__STM32F4XX_QUADSPI_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
