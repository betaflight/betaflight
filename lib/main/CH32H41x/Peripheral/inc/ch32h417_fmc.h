/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_fmc.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the FMC
*                      firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_FMC_H
#define __CH32H417_FMC_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32h417.h"

/* FMC Init structure definition */
typedef struct
{
  uint32_t FMC_AddressSetupTime;       /* Defines the number of HCLK cycles to configure
                                           the duration of the address setup time.
                                           This parameter can be a value between 0 and 0xF.
                                           @note: It is not used with synchronous NOR Flash memories. */

  uint32_t FMC_AddressHoldTime;        /* Defines the number of HCLK cycles to configure
                                           the duration of the address hold time.
                                           This parameter can be a value between 0 and 0xF.
                                           @note: It is not used with synchronous NOR Flash memories.*/

  uint32_t FMC_DataSetupTime;          /* Defines the number of HCLK cycles to configure
                                           the duration of the data setup time.
                                           This parameter can be a value between 0 and 0xFF.
                                           @note: It is used for SRAMs, ROMs and asynchronous multiplexed NOR Flash memories. */

  uint32_t FMC_BusTurnAroundDuration;  /* Defines the number of HCLK cycles to configure
                                           the duration of the bus turnaround.
                                           This parameter can be a value between 0 and 0xF.
                                           @note: It is only used for multiplexed NOR Flash memories. */

  uint32_t FMC_CLKDivision;            /* Defines the period of CLK clock output signal, expressed in number of HCLK cycles.
                                           This parameter can be a value between 1 and 0xF.
                                           @note: This parameter is not used for asynchronous NOR Flash, SRAM or ROM accesses. */

  uint32_t FMC_DataLatency;            /* Defines the number of memory clock cycles to issue
                                           to the memory before getting the first data.
                                           The value of this parameter depends on the memory type as shown below:
                                              - It must be set to 0 in case of a CRAM
                                              - It is don't care in asynchronous NOR, SRAM or ROM accesses
                                              - It may assume a value between 0 and 0xF in NOR Flash memories
                                                with synchronous burst mode enable */

  uint32_t FMC_AccessMode;             /* Specifies the asynchronous access mode.
                                            This parameter can be a value of @ref FMC_Access_Mode */
}FMC_NORSRAMTimingInitTypeDef;

typedef struct
{
  uint32_t FMC_Bank;                /* Specifies the NOR/SRAM memory bank that will be used.
                                        This parameter can be a value of @ref FMC_NORSRAM_Bank */

  uint32_t FMC_DataAddressMux;      /* Specifies whether the address and data values are
                                        multiplexed on the databus or not.
                                        This parameter can be a value of @ref FMC_Data_Address_Bus_Multiplexing */

  uint32_t FMC_MemoryType;          /* Specifies the type of external memory attached to
                                        the corresponding memory bank.
                                        This parameter can be a value of @ref FMC_Memory_Type */

  uint32_t FMC_MemoryDataWidth;     /* Specifies the external memory device width.
                                        This parameter can be a value of @ref FMC_Data_Width */

  uint32_t FMC_BurstAccessMode;     /* Enables or disables the burst access mode for Flash memory,
                                        valid only with synchronous burst Flash memories.
                                        This parameter can be a value of @ref FMC_Burst_Access_Mode */
                                       
  uint32_t FMC_AsynchronousWait;    /* Enables or disables wait signal during asynchronous transfers,
                                        valid only with asynchronous Flash memories.
                                        This parameter can be a value of @ref FMC_AsynchronousWait */

  uint32_t FMC_WaitSignalPolarity;  /* Specifies the wait signal polarity, valid only when accessing
                                        the Flash memory in burst mode.
                                        This parameter can be a value of @ref FMC_Wait_Signal_Polarity */

  uint32_t FMC_WaitSignalActive;    /* Specifies if the wait signal is asserted by the memory one
                                        clock cycle before the wait state or during the wait state,
                                        valid only when accessing memories in burst mode.
                                        This parameter can be a value of @ref FMC_Wait_Timing */

  uint32_t FMC_WriteOperation;      /* Enables or disables the write operation in the selected bank by the FMC.
                                        This parameter can be a value of @ref FMC_Write_Operation */

  uint32_t FMC_WaitSignal;          /* Enables or disables the wait-state insertion via wait
                                        signal, valid for Flash memory access in burst mode.
                                        This parameter can be a value of @ref FMC_Wait_Signal */

  uint32_t FMC_ExtendedMode;        /* Enables or disables the extended mode.
                                        This parameter can be a value of @ref FMC_Extended_Mode */

  uint32_t FMC_WriteBurst;          /* Enables or disables the write burst operation.
                                        This parameter can be a value of @ref FMC_Write_Burst */

  uint32_t FMC_CPSIZE;              /* Specifies the external CRAM page size.
                                        This parameter can be a value of @ref FMC_CPSIZE */

  uint32_t FMC_BMP;                 /* Specifies the address mapping.
                                        This parameter can be a value of @ref FMC_BMP */                                          

  FMC_NORSRAMTimingInitTypeDef* FMC_ReadWriteTimingStruct; /* Timing Parameters for write and read access if the  ExtendedMode is not used*/

  FMC_NORSRAMTimingInitTypeDef* FMC_WriteTimingStruct;     /* Timing Parameters for write access if the  ExtendedMode is used*/
}FMC_NORSRAMInitTypeDef;

typedef struct
{
  uint32_t FMC_SetupTime;      /* Defines the number of HCLK cycles to setup address before
                                   the command assertion for NAND-Flash read or write access
                                   to common/Attribute or I/O memory space (depending on
                                   the memory space timing to be configured).
                                   This parameter can be a value between 0 and 0xFF.*/

  uint32_t FMC_WaitSetupTime;  /* Defines the minimum number of HCLK cycles to assert the
                                   command for NAND-Flash read or write access to
                                   common/Attribute or I/O memory space (depending on the
                                   memory space timing to be configured).
                                   This parameter can be a number between 0x00 and 0xFF */

  uint32_t FMC_HoldSetupTime;  /* Defines the number of HCLK clock cycles to hold address
                                   (and data for write access) after the command deassertion
                                   for NAND-Flash read or write access to common/Attribute
                                   or I/O memory space (depending on the memory space timing
                                   to be configured).
                                   This parameter can be a number between 0x00 and 0xFF */

  uint32_t FMC_HiZSetupTime;   /* Defines the number of HCLK clock cycles during which the
                                   databus is kept in HiZ after the start of a NAND-Flash
                                   write access to common/Attribute or I/O memory space (depending
                                   on the memory space timing to be configured).
                                   This parameter can be a number between 0x00 and 0xFF */
}FMC_NAND_PCCARDTimingInitTypeDef;


typedef struct
{
  uint32_t FMC_Bank;             /* Specifies the NAND memory bank that will be used.
                                     This parameter can be a value of @ref FMC_NAND_Bank */

  uint32_t FMC_Waitfeature;      /* Enables or disables the Wait feature for the NAND Memory Bank.
                                     This parameter can be any value of @ref FMC_Wait_feature */

  uint32_t FMC_MemoryDataWidth;  /* Specifies the external memory device width.
                                     This parameter can be any value of @ref FMC_NAND_Data_Width */

  uint32_t FMC_ECC;              /* Enables or disables the ECC computation.
                                     This parameter can be any value of @ref FMC_ECC */

  uint32_t FMC_ECCPageSize;      /* Defines the page size for the extended ECC.
                                     This parameter can be any value of @ref FMC_ECC_Page_Size */

  uint32_t FMC_TCLRSetupTime;    /* Defines the number of HCLK cycles to configure the
                                     delay between CLE low and RE low.
                                     This parameter can be a value between 0 and 0xFF. */

  uint32_t FMC_TARSetupTime;     /* Defines the number of HCLK cycles to configure the
                                     delay between ALE low and RE low.
                                     This parameter can be a number between 0x0 and 0xFF */

  FMC_NAND_PCCARDTimingInitTypeDef*  FMC_CommonSpaceTimingStruct;   /* FMC Common Space Timing */

  FMC_NAND_PCCARDTimingInitTypeDef*  FMC_AttributeSpaceTimingStruct; /* FMC Attribute Space Timing */
}FMC_NANDInitTypeDef;

/* FMC SDRAM Init structure definition */
typedef struct
{
  uint32_t FMC_LoadToActiveDelay;     /* Defines load to active delay.
                                        This parameter can be a number between 0x0 and 0xF */

  uint32_t FMC_ExitSelfRefreshDelay;  /* Defines exit self refresh delay.
                                        This parameter can be a number between 0x0 and 0xF */

  uint32_t FMC_SelfRefreshTime;       /* Defines self refresh time.
                                        This parameter can be a number between 0x0 and 0xF */

  uint32_t FMC_RowCycleDelay;         /* Defines row cycle delay.
                                        This parameter can be a number between 0x0 and 0xF */

  uint32_t FMC_WriteRecoveryTime;     /* Defines write recover time.
                                        This parameter can be a number between 0x0 and 0xF */

  uint32_t FMC_RPDelay;               /* Defines row precharge delay.
                                        This parameter can be a number between 0x0 and 0xF */   

  uint32_t FMC_RCDDelay;              /* Defines row to column delay.
                                        This parameter can be a number between 0x0 and 0xF */  

} FMC_SDRAM_TimingTypeDef;

typedef struct
{
  uint32_t FMC_Bank;               /* Specifies the SDRAM memory bank that will be used.
                                      This parameter can be a value of @ref FMC_SDRAM_Bank */

  uint32_t FMC_ColumnBitsNumber;   /* Defines the number of column bits.
                                      This parameter can be a value of @ref FMC_ColumnBitsNumber.*/

  uint32_t FMC_RowBitsNumber;      /* Defines the number of row bits.
                                      This parameter can be a value of @ref FMC_RowBitsNumber.*/

  uint32_t FMC_MemoryDataWidth;    /* Defines memory data width.
                                      This parameter can be a value of @ref FMC_MemoryDataWidth.*/  

  uint32_t FMC_InternalBankNumber; /* Defines Internal Bank Number.
                                      This parameter can be a value of @ref FMC_InternalBankNumber.*/  

  uint32_t FMC_CASLatency;         /* Defines CAS delay.
                                      This parameter can be a value of @ref FMC_CASLatency.*/  
 
  uint32_t FMC_WriteProtection;    /* Defines write protect.
                                      This parameter can be a value of @ref FMC_WriteProtection.*/   

  uint32_t FMC_SDClockPeriod;      /* Defines the SDRAM clock period.
                                      This parameter can be a value of @ref FMC_SDClockPeriod.*/   

  uint32_t FMC_ReadBurst;          /* Defines the SDRAM burst read.
                                      This parameter can be a value of @ref FMC_ReadBurst.*/

  uint32_t FMC_ReadPipeDelay;      /* Defines the SDRAM pipe delay read .
                                      This parameter can be a value of @ref FMC_ReadPipeDelay.*/

  uint32_t FMC_NRFS_CNT;           /* Defines the count of NRFS.
                                      This parameter can be a number between 0x0 and 0xF */

  uint32_t FMC_PHASE_SEL;          /* Defines Phase shift.
                                      This parameter can be a number between 0x0 and 0xF */

  uint32_t FMC_ENHANCE_READ_MODE;  /* Defines enhance read mode.
                                      This parameter can be a value of @ref FMC_ENHANCE_READ_MODE.*/ 

  FMC_SDRAM_TimingTypeDef* FMC_SDRAM_Timing;     
} FMC_SDRAM_InitTypeDef;

/* FMC SDRAM BANK Status enumeration */
typedef enum
{
   FMC_SDRAM_Normal = 0,
   FMC_SDRAM_SelfRefresh,
   FMC_SDRAM_PowerOff 
}FMC_SDRAM_BANK_Sta_TypeDef;

/* FMC_NORSRAM_Bank */
#define FMC_Bank1_NORSRAM1                             ((uint32_t)0x00000000)
#define FMC_Bank1_NORSRAM2                             ((uint32_t)0x00000002)
#define FMC_Bank1_NORSRAM3                             ((uint32_t)0x00000004)
#define FMC_Bank1_NORSRAM4                             ((uint32_t)0x00000006)

/* FMC_NAND_Bank */
#define FMC_Bank3_NAND                                 ((uint32_t)0x00000000)

/* FMC_SDRAM_Bank */
#define FMC_Bank5_SDRAM                                ((uint32_t)0x00000000)
#define FMC_Bank6_SDRAM                                ((uint32_t)0x00000002)

/* FMC_Data_Address_Bus_Multiplexing */
#define FMC_DataAddressMux_Disable                     ((uint32_t)0x00000000)
#define FMC_DataAddressMux_Enable                      ((uint32_t)0x00000002)

/* FMC_Memory_Type */
#define FMC_MemoryType_SRAM                            ((uint32_t)0x00000000)
#define FMC_MemoryType_PSRAM                           ((uint32_t)0x00000004)
#define FMC_MemoryType_NOR                             ((uint32_t)0x00000008)

/* FMC_Data_Width */
#define FMC_MemoryDataWidth_8b                         ((uint32_t)0x00000000)
#define FMC_MemoryDataWidth_16b                        ((uint32_t)0x00000010)
#define FMC_MemoryDataWidth_32b                        ((uint32_t)0x00000020)

/* FMC_NAND_Data_Width */
#define FMC_NAND_MemDataWidth_8b                       ((uint32_t)0x00000000)
#define FMC_NAND_MemDataWidth_16b                      ((uint32_t)0x00000010)

/* FMC_CPSIZE */
#define FMC_CPSIZE_None                                ((uint32_t)0x00000000)
#define FMC_CPSIZE_128Bytes                            ((uint32_t)0x00010000)
#define FMC_CPSIZE_256Bytes                            ((uint32_t)0x00020000)
#define FMC_CPSIZE_512Bytes                            ((uint32_t)0x00030000)
#define FMC_CPSIZE_1024Bytes                           ((uint32_t)0x00040000)

/* FMC_BMP */
#define FMC_BMP_Mode0                                  ((uint32_t)0x00000000)
#define FMC_BMP_Mode1                                  ((uint32_t)0x00010000)

/* FMC_Burst_Access_Mode */
#define FMC_BurstAccessMode_Disable                    ((uint32_t)0x00000000)
#define FMC_BurstAccessMode_Enable                     ((uint32_t)0x00000100)

/* FMC_AsynchronousWait */
#define FMC_AsynchronousWait_Disable                   ((uint32_t)0x00000000)
#define FMC_AsynchronousWait_Enable                    ((uint32_t)0x00008000)

/* FMC_Wait_Signal_Polarity */
#define FMC_WaitSignalPolarity_Low                     ((uint32_t)0x00000000)
#define FMC_WaitSignalPolarity_High                    ((uint32_t)0x00000200)

/* FMC_Wait_Timing */
#define FMC_WaitSignalActive_BeforeWaitState           ((uint32_t)0x00000000)
#define FMC_WaitSignalActive_DuringWaitState           ((uint32_t)0x00000800)

/* FMC_Write_Operation */
#define FMC_WriteOperation_Disable                     ((uint32_t)0x00000000)
#define FMC_WriteOperation_Enable                      ((uint32_t)0x00001000)

/* FMC_Wait_Signal */
#define FMC_WaitSignal_Disable                         ((uint32_t)0x00000000)
#define FMC_WaitSignal_Enable                          ((uint32_t)0x00002000)

/* FMC_Extended_Mode */
#define FMC_ExtendedMode_Disable                       ((uint32_t)0x00000000)
#define FMC_ExtendedMode_Enable                        ((uint32_t)0x00004000)

/* FMC_Write_Burst */
#define FMC_WriteBurst_Disable                         ((uint32_t)0x00000000)
#define FMC_WriteBurst_Enable                          ((uint32_t)0x00080000)

/* FMC_Access_Mode */
#define FMC_AccessMode_A                               ((uint32_t)0x00000000)
#define FMC_AccessMode_B                               ((uint32_t)0x10000000)
#define FMC_AccessMode_C                               ((uint32_t)0x20000000)
#define FMC_AccessMode_D                               ((uint32_t)0x30000000)

/* FMC_Wait_feature */
#define FMC_Waitfeature_Disable                        ((uint32_t)0x00000000)
#define FMC_Waitfeature_Enable                         ((uint32_t)0x00000002)

/* FMC_ECC */
#define FMC_ECC_Disable                                ((uint32_t)0x00000000)
#define FMC_ECC_Enable                                 ((uint32_t)0x00000040)

/* FMC_ECC_Page_Size */
#define FMC_ECCPageSize_256Bytes                       ((uint32_t)0x00000000)
#define FMC_ECCPageSize_512Bytes                       ((uint32_t)0x00020000)
#define FMC_ECCPageSize_1024Bytes                      ((uint32_t)0x00040000)
#define FMC_ECCPageSize_2048Bytes                      ((uint32_t)0x00060000)
#define FMC_ECCPageSize_4096Bytes                      ((uint32_t)0x00080000)
#define FMC_ECCPageSize_8192Bytes                      ((uint32_t)0x000A0000)

/* FMC_ColumnBitsNumber */
#define FMC_ColumnBitsNumber_8                         ((uint32_t)0x00000000)
#define FMC_ColumnBitsNumber_9                         ((uint32_t)0x00000001)
#define FMC_ColumnBitsNumber_10                        ((uint32_t)0x00000002)
#define FMC_ColumnBitsNumber_11                        ((uint32_t)0x00000003)

/* FMC_RowBitsNumber */
#define FMC_ROWBitsNumber_11                           ((uint32_t)0x00000000)
#define FMC_ROWBitsNumber_12                           ((uint32_t)0x00000001)
#define FMC_ROWBitsNumber_13                           ((uint32_t)0x00000002)

/* FMC_MemoryDataWidth */
#define FMC_MemoryDataWidth_8                          ((uint32_t)0x00000000)
#define FMC_MemoryDataWidth_16                         ((uint32_t)0x00000001)
#define FMC_MemoryDataWidth_32                         ((uint32_t)0x00000002)

/* FMC_InternalBankNumber */
#define FMC_InternalBankNumber_2                       ((uint32_t)0x00000000)
#define FMC_InternalBankNumber_4                       ((uint32_t)0x00000001)

/* FMC_CASLatency */
#define FMC_CASLatency_1CLk                            ((uint32_t)0x00000001)
#define FMC_CASLatency_2CLk                            ((uint32_t)0x00000002)
#define FMC_CASLatency_3CLk                            ((uint32_t)0x00000003)

/* FMC_WriteProtection */
#define FMC_WriteProtection_Enable                     ((uint32_t)0x00000001)
#define FMC_WriteProtection_Disable                    ((uint32_t)0x00000000)

/* FMC_SDClockPeriod */
#define FMC_SDClockPeriod_Disable                      ((uint32_t)0x00000000)
#define FMC_SDClockPeriod_2HCLK                        ((uint32_t)0x00000002)
#define FMC_SDClockPeriod_3HCLK                        ((uint32_t)0x00000003)

/* FMC_ReadBurst */
#define FMC_ReadBurst_Disable                          ((uint32_t)0x00000000)
#define FMC_ReadBurst_Enable                           ((uint32_t)0x00000001)

/* FMC_ReadPipeDelay */
#define FMC_ReadPipeDelay_none                         ((uint32_t)0x00000000)
#define FMC_ReadPipeDelay_1HCLK                        ((uint32_t)0x00000001)
#define FMC_ReadPipeDelay_2HCLK                        ((uint32_t)0x00000002)

/* FMC_ENHANCE_READ_MODE */
#define FMC_ENHANCE_READ_MODE_Enable                   ((uint32_t)0x00000000)
#define FMC_ENHANCE_READ_MODE_Disable                  ((uint32_t)0x00008000)

/* FMC_SDRAM_SEL */
#define FMC_SDRAM_SEL_None                             ((uint32_t)0x00000000)
#define FMC_SDRAM_SEL_Bank5                            ((uint32_t)0x00000010)
#define FMC_SDRAM_SEL_Bank6                            ((uint32_t)0x00000008)
#define FMC_SDRAM_SEL_Bank5_6                          ((uint32_t)0x00000018)

/* FMC_SDRAM_CMD_Mode */
#define FMC_SDRAM_CMD_Mode0                            ((uint32_t)0x00000000)
#define FMC_SDRAM_CMD_Mode1                            ((uint32_t)0x00000001)
#define FMC_SDRAM_CMD_Mode2                            ((uint32_t)0x00000002)
#define FMC_SDRAM_CMD_Mode3                            ((uint32_t)0x00000003)
#define FMC_SDRAM_CMD_Mode4                            ((uint32_t)0x00000004)
#define FMC_SDRAM_CMD_Mode5                            ((uint32_t)0x00000005)
#define FMC_SDRAM_CMD_Mode6                            ((uint32_t)0x00000006)

/* FMC_interrupts_definition */
#define FMC_IT_RE                                      ((uint32_t)0x00004000))

/* FMC_flags_definition */
#define FMC_FLAG_FEMPT                                 ((uint32_t)0x00000040)
#define FMC_FLAG_BUSY                                  ((uint32_t)0x10000020))
#define FMC_FLAG_RE                                    ((uint32_t)0x10000001))


void FMC_DeInit(void);
void FMC_NORSRAMDeInit(uint32_t FMC_Bank);
void FMC_NANDDeInit(uint32_t FMC_Bank);
void FMC_NORSRAMInit(FMC_NORSRAMInitTypeDef *FMC_NORSRAMInitStruct);
void FMC_NANDInit(FMC_NANDInitTypeDef *FMC_NANDInitStruct);
void FMC_SDRAM_Init(FMC_SDRAM_InitTypeDef *FMC_SDRAMInitStruct);
void FMC_NORSRAMStructInit(FMC_NORSRAMInitTypeDef *FMC_NORSRAMInitStruct);
void FMC_NANDStructInit(FMC_NANDInitTypeDef *FMC_NANDInitStruct);
void FMC_SDRAM_SendCMDConfig(uint32_t SDRAM_Sel, uint32_t CMD_Mode, uint32_t CMD_Refresh_cnt, uint32_t SDRAM_ModeREG);
void FMC_NORSRAM_NANDCmd(FunctionalState NewState);
void FMC_NORSRAMCmd(uint32_t FMC_Bank, FunctionalState NewState);
void FMC_SDRAMCmd(uint32_t FMC_SDRAMBank, FunctionalState NewState);
void FMC_NANDCmd(uint32_t FMC_Bank, FunctionalState NewState);
void FMC_NANDECCCmd(uint32_t FMC_Bank, FunctionalState NewState);
uint32_t FMC_GetECC(uint32_t FMC_Bank);
void FMC_SDRAM_SetRefreshCnt(uint16_t Refresh_Cnt);
FMC_SDRAM_BANK_Sta_TypeDef FMC_SDRAM_GetBankSta(uint32_t SDRAM_Bank);
void FMC_ITConfig(uint32_t FMC_IT, FunctionalState NewState);
ITStatus FMC_GetITStatus(uint32_t FMC_IT);
void FMC_ClearITPendingBit(uint32_t FMC_IT);
FlagStatus FMC_GetFlagStatus(uint32_t FMC_FLAG);
void FMC_ClearFlag(uint32_t FMC_FLAG);



#ifdef __cplusplus
}
#endif

#endif 
