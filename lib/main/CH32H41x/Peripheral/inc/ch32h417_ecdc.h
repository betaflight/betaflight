/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32h417_ecdc.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2025/03/01
 * Description        : This file contains all the functions prototypes for the
 *                      ECDC firmware library.
 *********************************************************************************
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH32H417_ECDC_H
#define __CH32H417_ECDC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ch32h417.h"

/* ECDCAlgorithm definition */
typedef enum
{
    ECDCAlgorithm_SM4 = 0,
    ECDCAlgorithm_AES
} ECDC_Algorithm_TypeDef;

/* ECDCBlockCipherMode definition */
typedef enum
{
    ECDCBlockCipherMode_CTR = 0,
    ECDCBlockCipherMode_ECB
} ECDC_BlockCipherMode_TypeDef;

/* ECDCKeyLen definition */
typedef enum
{
    ECDCKeyLen_128b = 0,
    ECDCKeyLen_192b,
    ECDCKeyLen_256b
} ECDC_KeyLen_TypeDef;

/* ECDCExcuteEndian definition */
typedef enum
{
    ECDCExcuteEndian_Little = 0,
    ECDCExcuteEndian_Big
} ECDC_ExcuteEndian_TypeDef;

/* ECDCExcuteMode definition */
typedef enum
{
    ECDC_RAM_Encrypt = 0x82,
    ECDC_RAM_Decrypt = 0x8a,
    ECDC_SingleTime_Encrypt = 0x2,
    ECDC_SingleTime_Decrypt = 0xa
} ECDC_ExcuteMode_TypeDef;

/* ECDC_IV definition */
typedef struct
{
    uint32_t IV_31T0;
    uint32_t IV_63T32;
    uint32_t IV_95T64;
    uint32_t IV_127T96;
} ECDC_IV_TypeDef;

/* ECDC_KEY definition */
typedef struct
{
    uint32_t KEY_31T0;
    uint32_t KEY_63T32;
    uint32_t KEY_95T64;
    uint32_t KEY_127T96;
    uint32_t KEY_159T128;
    uint32_t KEY_191T160;
    uint32_t KEY_223T192;
    uint32_t KEY_255T224;
} ECDC_KEY_TypeDef;

/* ECDC Init structure definition */
typedef struct
{
    ECDC_ExcuteMode_TypeDef ExcuteMode;
    ECDC_Algorithm_TypeDef Algorithm;
    ECDC_BlockCipherMode_TypeDef BlockCipherMode;
    ECDC_KeyLen_TypeDef KeyLen;
    ECDC_ExcuteEndian_TypeDef ExcuteEndian;
    ECDC_KEY_TypeDef *Key;
    ECDC_IV_TypeDef *IV;
} ECDC_InitTypeDef;

/* ECDC_Clock_Div */
#define ECDC_ClockSource_PLLCLK_Div1 ((uint8_t)0x01)
#define ECDC_ClockSource_PLLCLK_Div2 ((uint8_t)0x20)
#define ECDC_ClockSource_PLLCLK_Div3 ((uint8_t)0x30)
#define ECDC_ClockSource_PLLCLK_Div4 ((uint8_t)0x40)
#define ECDC_ClockSource_PLLCLK_Div5 ((uint8_t)0x50)
#define ECDC_ClockSource_PLLCLK_Div6 ((uint8_t)0x60)
#define ECDC_ClockSource_PLLCLK_Div7 ((uint8_t)0x70)

/* ECDC_Flag_Definition */
#define ECDC_FLAG_KeyEx_END ((uint32_t)0x00010000)
#define ECDC_FLAG_Single_END ((uint32_t)0x00020000)
#define ECDC_FLAG_RAM2RAM_END ((uint32_t)0x00040000)

/* ECDC_Interrupts_Definition */
#define ECDC_IT_KeyEx_END ((uint8_t)0x00010000)
#define ECDC_IT_Single_END ((uint8_t)0x00020000)
#define ECDC_IT_RAM2RAM_END ((uint8_t)0x00040000)

void ECDC_DeInit(void);
void ECDC_Init(ECDC_InitTypeDef *ECDC_InitStruct);
void ECDC_StructInit(ECDC_InitTypeDef *ECDC_InitStruct);
void ECDC_CTR_SetCounter(ECDC_IV_TypeDef *ECDC_IvStruct);
void ECDC_SetKey(ECDC_KEY_TypeDef *ECDC_KeyStruct);
void ECDC_SingleWR_RawData(uint32_t *WriteDataPointer);
void ECDC_SingleRD_EcdcData(uint32_t *ReadDataPointer);
void ECDC_SetSRC_BaseAddr(uint32_t Address, uint32_t Len);
void ECDC_SetDST_BaseAddr(uint32_t Address);
void ECDC_HardwareClockCmd(FunctionalState NewState);
void ECDC_ClockConfig(uint8_t ECDC_PLLDiv);
void ECDC_KeyExCmd(FunctionalState NewState);
void ECDC_ITConfig(uint32_t ECDC_IT, FunctionalState NewState);
FlagStatus ECDC_GetFlagStatus(uint32_t ECDC_FLAG);
void ECDC_ClearFlag(uint32_t ECDC_FLAG);
ITStatus ECDC_GetITStatus(uint32_t ECDC_IT);
void ECDC_ClearITPendingBit(uint32_t ECDC_IT);

#ifdef __cplusplus
}
#endif

#endif
