/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_qspi.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the
*                      QSPI firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_QSPI_H
#define __CH32H417_QSPI_H

#ifdef __cplusplus
 extern "C" {
#endif
#include "ch32h417.h"

/* QSPI Communication Configuration Init structure definition   */
typedef struct
{
  uint32_t QSPI_ComConfig_FMode; /* Specifies the Functional Mode
                              This parameter can be a value of @ref QSPI_ComConfig_Functional_Mode*/


  uint32_t QSPI_ComConfig_SIOOMode; /* Specifies the Send Instruction Only Once Mode
                              This parameter can be a value of @ref QSPI_ComConfig_SendInstructionOnlyOnceMode*/

  uint32_t QSPI_ComConfig_DMode; /* Specifies the Data Mode
                              This parameter can be a value of @ref QSPI_ComConfig_DataMode*/

  uint32_t QSPI_ComConfig_DummyCycles; /* Specifies the Number of Dummy Cycles.
                                This parameter can be a number between 0x00 and 0x1F */

  uint32_t QSPI_ComConfig_ABSize; /* Specifies the Alternate Bytes Size
                              This parameter can be a value of @ref QSPI_ComConfig_AlternateBytesSize*/

  uint32_t QSPI_ComConfig_ABMode; /* Specifies the Alternate Bytes Mode
                              This parameter can be a value of @ref QSPI_ComConfig_AlternateBytesMode*/

  uint32_t QSPI_ComConfig_ADSize; /* Specifies the Address Size
                              This parameter can be a value of @ref QSPI_ComConfig_AddressSize*/

  uint32_t QSPI_ComConfig_ADMode; /* Specifies the Address Mode
                              This parameter can be a value of @ref QSPI_ComConfig_AddressMode*/

  uint32_t QSPI_ComConfig_IMode; /* Specifies the Instruction Mode
                              This parameter can be a value of @ref QSPI_ComConfig_InstructionMode*/

  uint32_t QSPI_ComConfig_Ins; /* Specifies the Instruction Mode
                              This parameter can be a value of @ref QSPI_ComConfig_Instruction*/

} QSPI_ComConfig_InitTypeDef;

/* QSPI Init structure definition   */
typedef struct
{

  uint32_t QSPI_Prescaler; /* Specifies the prescaler value used to divide the QSPI clock.
                    This parameter can be a number between 0x00 and 0xFF */

  uint32_t QSPI_CKMode; /* Specifies the Clock Mode
                    This parameter can be a value of @ref QSPI_Clock_Mode*/

  uint32_t QSPI_CSHTime; /* Specifies the Chip Select High Time
                    This parameter can be a value of @ref QSPI_ChipSelectHighTime*/

  uint32_t QSPI_FSize;   /* Specifies the Flash Size.
                      QSPI_FSize+1 is effectively the number of address bits required to address the flash memory.
                      The flash capacity can be up to 4GB (addressed using 32 bits) in indirect mode, but the
                      addressable space in memory-mapped mode is limited to 512MB
                      This parameter can be a number between 0x00 and 0x1F */
  uint32_t QSPI_FSelect; /* Specifies the Flash which will be used,
                        This parameter can be a value of @ref QSPI_Fash_Select*/
  uint32_t QSPI_DFlash;  /* Specifies the Dual Flash Mode State
                        This parameter can be a value of @ref QSPI_Dual_Flash*/
} QSPI_InitTypeDef;


/* QSPI_Clock_Mode */
#define QSPI_CKMode_Mode0                     ((uint32_t)0x00000000)
#define QSPI_CKMode_Mode3                     ((uint32_t)QSPI_DCR_CKMODE)

/* QSPI_ChipSelectHighTime */
#define QSPI_CSHTime_1Cycle                   ((uint32_t)0x00000000)
#define QSPI_CSHTime_2Cycle                   ((uint32_t)QSPI_DCR_CSHT_0)
#define QSPI_CSHTime_3Cycle                   ((uint32_t)QSPI_DCR_CSHT_1)
#define QSPI_CSHTime_4Cycle                   ((uint32_t)QSPI_DCR_CSHT_0 | QSPI_DCR_CSHT_1)
#define QSPI_CSHTime_5Cycle                   ((uint32_t)QSPI_DCR_CSHT_2)
#define QSPI_CSHTime_6Cycle                   ((uint32_t)QSPI_DCR_CSHT_2 | QSPI_DCR_CSHT_0)
#define QSPI_CSHTime_7Cycle                   ((uint32_t)QSPI_DCR_CSHT_2 | QSPI_DCR_CSHT_1)
#define QSPI_CSHTime_8Cycle                   ((uint32_t)QSPI_DCR_CSHT)

/* QSPI_Fash_Select */
#define QSPI_FSelect_1                        ((uint32_t)0x00000000)
#define QSPI_FSelect_2                        ((uint32_t)QSPI_CR_FSEL)

/* QSPI_Dual_Flash */
#define QSPI_DFlash_Disable                   ((uint32_t)0x00000000)
#define QSPI_DFlash_Enable                    ((uint32_t)QSPI_CR_DFM)

/* QSPI_ComConfig_Functional_Mode */
#define QSPI_ComConfig_FMode_Indirect_Write   ((uint32_t)0x00000000)
#define QSPI_ComConfig_FMode_Indirect_Read    ((uint32_t)QSPI_CCR_FMODE_0)
#define QSPI_ComConfig_FMode_Auto_Polling     ((uint32_t)QSPI_CCR_FMODE_1)
#define QSPI_ComConfig_FMode_Memory_Mapped    ((uint32_t)QSPI_CCR_FMODE)


/* QSPI_ComConfig_SendInstructionOnlyOnceMode */
#define QSPI_ComConfig_SIOOMode_Disable       ((uint32_t)0x00000000)
#define QSPI_ComConfig_SIOOMode_Enable        ((uint32_t)QSPI_CCR_SIOO)

/* QSPI_ComConfig_DataMode */
#define QSPI_ComConfig_DMode_NoData           ((uint32_t)0x00000000)
#define QSPI_ComConfig_DMode_1Line            ((uint32_t)QSPI_CCR_DMODE_0)
#define QSPI_ComConfig_DMode_2Line            ((uint32_t)QSPI_CCR_DMODE_1)
#define QSPI_ComConfig_DMode_4Line            ((uint32_t)QSPI_CCR_DMODE)

/* QSPI_ComConfig_AlternateBytesSize */
#define QSPI_ComConfig_ABSize_8bit            ((uint32_t)0x00000000)
#define QSPI_ComConfig_ABSize_16bit           ((uint32_t)QSPI_CCR_ABSIZE_0)
#define QSPI_ComConfig_ABSize_24bit           ((uint32_t)QSPI_CCR_ABSIZE_1)
#define QSPI_ComConfig_ABSize_32bit           ((uint32_t)QSPI_CCR_ABSIZE)

/* QSPI_ComConfig_AlternateBytesMode */
#define QSPI_ComConfig_ABMode_NoAlternateByte ((uint32_t)0x00000000)
#define QSPI_ComConfig_ABMode_1Line           ((uint32_t)QSPI_CCR_ABMODE_0)
#define QSPI_ComConfig_ABMode_2Line           ((uint32_t)QSPI_CCR_ABMODE_1)
#define QSPI_ComConfig_ABMode_4Line           ((uint32_t)QSPI_CCR_ABMODE)

/* QSPI_ComConfig_AddressSize */
#define QSPI_ComConfig_ADSize_8bit            ((uint32_t)0x00000000)
#define QSPI_ComConfig_ADSize_16bit           ((uint32_t)QSPI_CCR_ADSIZE_0)
#define QSPI_ComConfig_ADSize_24bit           ((uint32_t)QSPI_CCR_ADSIZE_1)
#define QSPI_ComConfig_ADSize_32bit           ((uint32_t)QSPI_CCR_ADSIZE)

/* QSPI_ComConfig_AddressMode */
#define QSPI_ComConfig_ADMode_NoAddress       ((uint32_t)0x00000000)
#define QSPI_ComConfig_ADMode_1Line           ((uint32_t)QSPI_CCR_ADMODE_0)
#define QSPI_ComConfig_ADMode_2Line           ((uint32_t)QSPI_CCR_ADMODE_1)
#define QSPI_ComConfig_ADMode_4Line           ((uint32_t)QSPI_CCR_ADMODE)

/* QSPI_ComConfig_InstructionMode */
#define QSPI_ComConfig_IMode_NoInstruction    ((uint32_t)0x00000000)
#define QSPI_ComConfig_IMode_1Line            ((uint32_t)QSPI_CCR_IMODE_0)
#define QSPI_ComConfig_IMode_2Line            ((uint32_t)QSPI_CCR_IMODE_1)
#define QSPI_ComConfig_IMode_4Line            ((uint32_t)QSPI_CCR_IMODE)


/* QSPI_Interrupts_definition */
#define QSPI_IT_TO                            (uint32_t)(QSPI_CR_TOIE | QSPI_SR_TOF)
#define QSPI_IT_SM                            (uint32_t)(QSPI_CR_SMIE | QSPI_SR_SMF)
#define QSPI_IT_FT                            (uint32_t)(QSPI_CR_FTIE | QSPI_SR_FTF)
#define QSPI_IT_TC                            (uint32_t)(QSPI_CR_TCIE | QSPI_SR_TCF)
#define QSPI_IT_TE                            (uint32_t)(QSPI_CR_TEIE | QSPI_SR_TEF)

/* QSPI_Flags_definition */
#define QSPI_FLAG_TO                          QSPI_SR_TOF
#define QSPI_FLAG_SM                          QSPI_SR_SMF
#define QSPI_FLAG_FT                          QSPI_SR_FTF
#define QSPI_FLAG_TC                          QSPI_SR_TCF
#define QSPI_FLAG_TE                          QSPI_SR_TEF
#define QSPI_FLAG_BUSY                        QSPI_SR_BUSY

/* QSPI_Polling_Match_Mode */
#define QSPI_PMM_AND                          ((uint32_t)0x00000000)
#define QSPI_PMM_OR                           ((uint32_t)QSPI_CR_PMM)

/* QSPI_SIOXEN  */
#define QSPI_SIOXEN                           ((uint32_t)0x00002000)

void QSPI_DeInit(QSPI_TypeDef *QUADSPIx);
void QSPI_Init(QSPI_TypeDef *QUADSPIx, QSPI_InitTypeDef *QSPI_InitStruct);
void QSPI_StructInit(QSPI_InitTypeDef *QSPI_InitStruct);
void QSPI_ComConfig_Init(QSPI_TypeDef *QUADSPIx, QSPI_ComConfig_InitTypeDef *QSPI_ComConfig_InitStruct);
void QSPI_ComConfig_StructInit(QSPI_ComConfig_InitTypeDef *QSPI_ComConfig_InitStruct);
void QSPI_Cmd(QSPI_TypeDef *QUADSPIx, FunctionalState NewState);
void QSPI_AutoPollingMode_Config(QSPI_TypeDef *QUADSPIx, uint32_t QSPI_Match, uint32_t QSPI_Mask, uint32_t QSPI_Match_Mode);
void QSPI_AutoPollingMode_SetInterval(QSPI_TypeDef *QUADSPIx, uint32_t QSPI_Interval);
void QSPI_MemoryMappedMode_SetTimeout(QSPI_TypeDef *QUADSPIx, uint32_t QSPI_Timeout);
void QSPI_SetAddress(QSPI_TypeDef *QSPIx, uint32_t QSPI_Address);
void QSPI_SetAlternateByte(QSPI_TypeDef *QSPIx, uint32_t QSPI_AlternateByte);
void QSPI_SetFIFOThreshold(QSPI_TypeDef *QSPIx, uint32_t QSPI_FIFOThreshold);
void QSPI_SetDataLength(QSPI_TypeDef *QSPIx, uint32_t QSPI_DataLength);
void QSPI_TimeoutCounterCmd(QSPI_TypeDef *QSPIx, FunctionalState NewState);
void QSPI_AutoPollingModeStopCmd(QSPI_TypeDef *QSPIx, FunctionalState NewState);
void QSPI_AbortRequest(QSPI_TypeDef *QUADSPIx);
void QSPI_DualFlashMode_Cmd(QSPI_TypeDef *QUADSPIx, FunctionalState NewState);
void QSPI_SendData8(QSPI_TypeDef *QUADSPIx, uint8_t Data);
void QSPI_SendData16(QSPI_TypeDef *QUADSPIx, uint16_t Data);
void QSPI_SendData32(QSPI_TypeDef *QUADSPIx, uint32_t Data);
uint8_t QSPI_ReceiveData8(QSPI_TypeDef *QUADSPIx);
uint16_t QSPI_ReceiveData16(QSPI_TypeDef *QUADSPIx);
uint32_t QSPI_ReceiveData32(QSPI_TypeDef *QUADSPIx);
void QSPI_DMACmd(QSPI_TypeDef *QUADSPIx, FunctionalState NewState);
void QSPI_Start(QSPI_TypeDef *QSPIx);
void QSPI_ITConfig(QSPI_TypeDef *QUADSPIx, uint32_t QSPI_IT, FunctionalState NewState);
uint32_t QSPI_GetFIFOLevel(QSPI_TypeDef *QUADSPIx);
FlagStatus QSPI_GetFlagStatus(QSPI_TypeDef *QUADSPIx, uint32_t QSPI_FLAG);
void QSPI_ClearFlag(QSPI_TypeDef *QUADSPIx, uint32_t QSPI_FLAG);
ITStatus QSPI_GetITStatus(QSPI_TypeDef *QUADSPIx, uint32_t QSPI_IT);
void QSPI_ClearITPendingBit(QSPI_TypeDef *QUADSPIx, uint32_t QSPI_IT);
uint32_t QSPI_GetFMode(QSPI_TypeDef *QUADSPIx);
void QSPI_EnableQuad(QSPI_TypeDef *QSPIx, FunctionalState NewState);
#ifdef __cplusplus
}
#endif

#endif
