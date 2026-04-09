/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_fmc.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the FMC firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_fmc.h"
#include "ch32h417_rcc.h"

/* FMC BCRx Mask */
#define BCR_MBKEN_Set          ((uint32_t)0x00000001)
#define BCR_MBKEN_Reset        ((uint32_t)0x000FFFFE)
#define BCR_FACCEN_Set         ((uint32_t)0x00000040)

/* FMC PCRx Mask */
#define PCR_PBKEN_Set          ((uint32_t)0x00000004)
#define PCR_PBKEN_Reset        ((uint32_t)0x000FFFFB)
#define PCR_ECCEN_Set          ((uint32_t)0x00000040)
#define PCR_ECCEN_Reset        ((uint32_t)0x000FFFBF)
#define PCR_MemoryType_NAND    ((uint32_t)0x00000008)


/*********************************************************************
 * @fn      FMC_DeInit
 *
 * @brief   Deinitializes the FMC peripheral registers to their default
 *        reset values.
 *
 * @param   none
 *
 * @return  none
 */
void FMC_DeInit(void)
{
    RCC_HBPeriphResetCmd(RCC_HBPeriph_FMC, ENABLE);
    RCC_HBPeriphResetCmd(RCC_HBPeriph_FMC, DISABLE);
}

/*********************************************************************
 * @fn      FMC_NORSRAMDeInit
 *
 * @brief   Deinitializes the FMC NOR/SRAM Banks registers to their default
 *        reset values.
 *
 * @param   FMC_Bank-
 *            FMC_Bank1_NORSRAM1 - FMC Bank1 NOR/SRAM1.
 *
 * @return  none
 */
void FMC_NORSRAMDeInit(uint32_t FMC_Bank)
{
    if(FMC_Bank == FMC_Bank1_NORSRAM1)
    {
        FMC_Bank1->BTCR[FMC_Bank] = 0x000030DB;
    }
    else
    {
        FMC_Bank1->BTCR[FMC_Bank] = 0x000030D2;
    }
    FMC_Bank1->BTCR[FMC_Bank + 1] = 0x0FFFFFFF;
    FMC_Bank1E->BWTR[FMC_Bank] = 0x0FFFFFFF;
}

/*********************************************************************
 * @fn      FMC_NANDDeInit
 *
 * @brief   Deinitializes the FMC NAND Banks registers to their default
 *        reset values.
 *
 * @param   FMC_Bank -
 *            FMC_Bank2_NAND - FMC Bank2 NAND.
 *
 * @return  none
 */
void FMC_NANDDeInit(uint32_t FMC_Bank)
{
    if(FMC_Bank == FMC_Bank3_NAND)
    {
        FMC_Bank3->PCR = 0x00000018;
        FMC_Bank3->SR = 0x00000040;
        FMC_Bank3->PMEM = 0xFCFCFCFC;
        FMC_Bank3->PATT = 0xFCFCFCFC;
    }
}

/*********************************************************************
 * @fn      FMC_NORSRAMInit
 *
 * @brief   Initializes the FMC NOR/SRAM Banks according to the specified
 *        parameters in the FMC_NORSRAMInitStruct.
 *
 * @param   SMC_NORSRAMInitStruct - pointer to a FMC_NORSRAMInitTypeDef
 *        structure that contains the configuration information for the FMC NOR/SRAM
 *        specified Banks.
 *
 * @return  none
 */
void FMC_NORSRAMInit(FMC_NORSRAMInitTypeDef *FMC_NORSRAMInitStruct)
{
    FMC_Bank1->BTCR[FMC_NORSRAMInitStruct->FMC_Bank] =
        (uint32_t)FMC_NORSRAMInitStruct->FMC_DataAddressMux |
        FMC_NORSRAMInitStruct->FMC_MemoryType |
        FMC_NORSRAMInitStruct->FMC_MemoryDataWidth |
        FMC_NORSRAMInitStruct->FMC_BurstAccessMode |
        FMC_NORSRAMInitStruct->FMC_AsynchronousWait |
        FMC_NORSRAMInitStruct->FMC_WaitSignalPolarity |
        FMC_NORSRAMInitStruct->FMC_WaitSignalActive |
        FMC_NORSRAMInitStruct->FMC_WriteOperation |
        FMC_NORSRAMInitStruct->FMC_WaitSignal |
        FMC_NORSRAMInitStruct->FMC_ExtendedMode |
        FMC_NORSRAMInitStruct->FMC_WriteBurst  |
        FMC_NORSRAMInitStruct->FMC_CPSIZE|
        FMC_NORSRAMInitStruct->FMC_BMP;

    if(FMC_NORSRAMInitStruct->FMC_MemoryType == FMC_MemoryType_NOR)
    {
        FMC_Bank1->BTCR[FMC_NORSRAMInitStruct->FMC_Bank] |= (uint32_t)BCR_FACCEN_Set;
    }

    FMC_Bank1->BTCR[FMC_NORSRAMInitStruct->FMC_Bank + 1] =
        (uint32_t)FMC_NORSRAMInitStruct->FMC_ReadWriteTimingStruct->FMC_AddressSetupTime |
        (FMC_NORSRAMInitStruct->FMC_ReadWriteTimingStruct->FMC_AddressHoldTime << 4) |
        (FMC_NORSRAMInitStruct->FMC_ReadWriteTimingStruct->FMC_DataSetupTime << 8) |
        (FMC_NORSRAMInitStruct->FMC_ReadWriteTimingStruct->FMC_BusTurnAroundDuration << 16) |
        (FMC_NORSRAMInitStruct->FMC_ReadWriteTimingStruct->FMC_CLKDivision << 20) |
        (FMC_NORSRAMInitStruct->FMC_ReadWriteTimingStruct->FMC_DataLatency << 24) |
        FMC_NORSRAMInitStruct->FMC_ReadWriteTimingStruct->FMC_AccessMode;

    if(FMC_NORSRAMInitStruct->FMC_ExtendedMode == FMC_ExtendedMode_Enable)
    {
        FMC_Bank1E->BWTR[FMC_NORSRAMInitStruct->FMC_Bank] =
            (uint32_t)FMC_NORSRAMInitStruct->FMC_WriteTimingStruct->FMC_AddressSetupTime |
            (FMC_NORSRAMInitStruct->FMC_WriteTimingStruct->FMC_AddressHoldTime << 4) |
            (FMC_NORSRAMInitStruct->FMC_WriteTimingStruct->FMC_DataSetupTime << 8) |
            (FMC_NORSRAMInitStruct->FMC_WriteTimingStruct->FMC_BusTurnAroundDuration << 16) |
            FMC_NORSRAMInitStruct->FMC_WriteTimingStruct->FMC_AccessMode;
    }
    else
    {
        FMC_Bank1E->BWTR[FMC_NORSRAMInitStruct->FMC_Bank] = 0x0FFFFFFF;
    }
        FMC_Bank1->BTCR[0]|=(1<<31); 
}

/*********************************************************************
 * @fn      FMC_NANDInit
 *
 * @brief   Initializes the FMC NAND Banks according to the specified
 *        parameters in the FMC_NANDInitStruct.
 *
 * @param   FMC_NANDInitStruct - pointer to a FMC_NANDInitTypeDef
 *        structure that contains the configuration information for the FMC
 *        NAND specified Banks.
 *
 * @return  none
 */
void FMC_NANDInit(FMC_NANDInitTypeDef *FMC_NANDInitStruct)
{
    uint32_t tmppcr = 0, tmppmem = 0, tmppatt = 0;

    tmppcr = (uint32_t)FMC_NANDInitStruct->FMC_Waitfeature |
             PCR_MemoryType_NAND |
             FMC_NANDInitStruct->FMC_MemoryDataWidth |
             FMC_NANDInitStruct->FMC_ECC |
             FMC_NANDInitStruct->FMC_ECCPageSize |
             (FMC_NANDInitStruct->FMC_TCLRSetupTime << 9) |
             (FMC_NANDInitStruct->FMC_TARSetupTime << 13);

    tmppmem = (uint32_t)FMC_NANDInitStruct->FMC_CommonSpaceTimingStruct->FMC_SetupTime |
              (FMC_NANDInitStruct->FMC_CommonSpaceTimingStruct->FMC_WaitSetupTime << 8) |
              (FMC_NANDInitStruct->FMC_CommonSpaceTimingStruct->FMC_HoldSetupTime << 16) |
              (FMC_NANDInitStruct->FMC_CommonSpaceTimingStruct->FMC_HiZSetupTime << 24);

    tmppatt = (uint32_t)FMC_NANDInitStruct->FMC_AttributeSpaceTimingStruct->FMC_SetupTime |
              (FMC_NANDInitStruct->FMC_AttributeSpaceTimingStruct->FMC_WaitSetupTime << 8) |
              (FMC_NANDInitStruct->FMC_AttributeSpaceTimingStruct->FMC_HoldSetupTime << 16) |
              (FMC_NANDInitStruct->FMC_AttributeSpaceTimingStruct->FMC_HiZSetupTime << 24);

    if(FMC_NANDInitStruct->FMC_Bank == FMC_Bank3_NAND)
    {
        FMC_Bank3->PCR = tmppcr;
        FMC_Bank3->PMEM = tmppmem;
        FMC_Bank3->PATT = tmppatt;
    }
}

/*********************************************************************
* @fn      FMC_SDRAM_Init
*
* @brief   Initializes the FMC SDRAM Banks according to the specified
*        parameters in the FMC_SDRAM_InitTypeDef.
*
* @param   FMC_SDRAMInitStruct - pointer to a FMC_SDRAM_InitTypeDef
*        structure that contains the configuration information for the FMC
*        SDRAM specified Banks.
*
* @return  none
*/ 
void FMC_SDRAM_Init(FMC_SDRAM_InitTypeDef *FMC_SDRAMInitStruct)
{
    FMC_Bank5_6->SDCR[FMC_SDRAMInitStruct->FMC_Bank] = 0;
    FMC_Bank5_6->SDTR[FMC_SDRAMInitStruct->FMC_Bank] = 0;
    FMC_Bank5_6->MISC &= ~(FMC_MISC_NRFS_CNT | FMC_MISC_Phase_Sel | FMC_MISC_Enhance_read_mode);

    FMC_Bank5_6->SDCR[FMC_SDRAMInitStruct->FMC_Bank] =
      (FMC_SDRAMInitStruct->FMC_ColumnBitsNumber) |
      (FMC_SDRAMInitStruct->FMC_RowBitsNumber      << 2) |
      (FMC_SDRAMInitStruct->FMC_MemoryDataWidth    << 4) |
      (FMC_SDRAMInitStruct->FMC_InternalBankNumber << 6) |
      (FMC_SDRAMInitStruct->FMC_CASLatency         << 7) |
      (FMC_SDRAMInitStruct->FMC_WriteProtection    << 9) |
      (FMC_SDRAMInitStruct->FMC_SDClockPeriod      << 10) |
      (FMC_SDRAMInitStruct->FMC_ReadBurst          << 12) |
      (FMC_SDRAMInitStruct->FMC_ReadPipeDelay      << 13);

    FMC_Bank5_6->SDTR[FMC_SDRAMInitStruct->FMC_Bank] = 
      (FMC_SDRAMInitStruct->FMC_SDRAM_Timing->FMC_LoadToActiveDelay) |
      (FMC_SDRAMInitStruct->FMC_SDRAM_Timing->FMC_ExitSelfRefreshDelay << 4) |
      (FMC_SDRAMInitStruct->FMC_SDRAM_Timing->FMC_SelfRefreshTime      << 8) |
      (FMC_SDRAMInitStruct->FMC_SDRAM_Timing->FMC_RowCycleDelay        << 12) |
      (FMC_SDRAMInitStruct->FMC_SDRAM_Timing->FMC_WriteRecoveryTime    << 16) |
      (FMC_SDRAMInitStruct->FMC_SDRAM_Timing->FMC_RPDelay              << 20) |
      (FMC_SDRAMInitStruct->FMC_SDRAM_Timing->FMC_RCDDelay             << 24); 

    FMC_Bank5_6->MISC |= (uint32_t)(FMC_SDRAMInitStruct->FMC_NRFS_CNT | 
                    (FMC_SDRAMInitStruct->FMC_PHASE_SEL << 4) |
                    FMC_SDRAMInitStruct->FMC_ENHANCE_READ_MODE);
}

/*********************************************************************
 * @fn      FMC_NORSRAMStructInit
 *
 * @brief   Fills each FMC_NORSRAMInitStruct member with its default value.
 *
 * @param   FMC_NORSRAMInitStruct - pointer to a FMC_NORSRAMInitTypeDef
 *        structure which will be initialized.
 *
 * @return  none
 */
void FMC_NORSRAMStructInit(FMC_NORSRAMInitTypeDef *FMC_NORSRAMInitStruct)
{
    FMC_NORSRAMInitStruct->FMC_Bank = FMC_Bank1_NORSRAM1;
    FMC_NORSRAMInitStruct->FMC_DataAddressMux = FMC_DataAddressMux_Enable;
    FMC_NORSRAMInitStruct->FMC_MemoryType = FMC_MemoryType_SRAM;
    FMC_NORSRAMInitStruct->FMC_MemoryDataWidth = FMC_MemoryDataWidth_8b;
    FMC_NORSRAMInitStruct->FMC_BurstAccessMode = FMC_BurstAccessMode_Disable;
    FMC_NORSRAMInitStruct->FMC_AsynchronousWait = FMC_AsynchronousWait_Disable;
    FMC_NORSRAMInitStruct->FMC_WaitSignalPolarity = FMC_WaitSignalPolarity_Low;
    FMC_NORSRAMInitStruct->FMC_WaitSignalActive = FMC_WaitSignalActive_BeforeWaitState;
    FMC_NORSRAMInitStruct->FMC_WriteOperation = FMC_WriteOperation_Enable;
    FMC_NORSRAMInitStruct->FMC_WaitSignal = FMC_WaitSignal_Enable;
    FMC_NORSRAMInitStruct->FMC_ExtendedMode = FMC_ExtendedMode_Disable;
    FMC_NORSRAMInitStruct->FMC_WriteBurst = FMC_WriteBurst_Disable;
    FMC_NORSRAMInitStruct->FMC_ReadWriteTimingStruct->FMC_AddressSetupTime = 0xF;
    FMC_NORSRAMInitStruct->FMC_ReadWriteTimingStruct->FMC_AddressHoldTime = 0xF;
    FMC_NORSRAMInitStruct->FMC_ReadWriteTimingStruct->FMC_DataSetupTime = 0xFF;
    FMC_NORSRAMInitStruct->FMC_ReadWriteTimingStruct->FMC_BusTurnAroundDuration = 0xF;
    FMC_NORSRAMInitStruct->FMC_ReadWriteTimingStruct->FMC_CLKDivision = 0xF;
    FMC_NORSRAMInitStruct->FMC_ReadWriteTimingStruct->FMC_DataLatency = 0xF;
    FMC_NORSRAMInitStruct->FMC_ReadWriteTimingStruct->FMC_AccessMode = FMC_AccessMode_A;
    FMC_NORSRAMInitStruct->FMC_WriteTimingStruct->FMC_AddressSetupTime = 0xF;
    FMC_NORSRAMInitStruct->FMC_WriteTimingStruct->FMC_AddressHoldTime = 0xF;
    FMC_NORSRAMInitStruct->FMC_WriteTimingStruct->FMC_DataSetupTime = 0xFF;
    FMC_NORSRAMInitStruct->FMC_WriteTimingStruct->FMC_BusTurnAroundDuration = 0xF;
    FMC_NORSRAMInitStruct->FMC_WriteTimingStruct->FMC_AccessMode = FMC_AccessMode_A;
}

/*********************************************************************
 * @fn      FMC_NANDStructInit
 *
 * @brief   Fills each FMC_NANDInitStruct member with its default value.
 *
 * @param   FMC_NANDInitStruct - pointer to a FMC_NANDInitTypeDef
 *        structure which will be initialized.
 *
 * @return  none
 */
void FMC_NANDStructInit(FMC_NANDInitTypeDef *FMC_NANDInitStruct)
{
    FMC_NANDInitStruct->FMC_Bank = FMC_Bank3_NAND;
    FMC_NANDInitStruct->FMC_Waitfeature = FMC_Waitfeature_Disable;
    FMC_NANDInitStruct->FMC_MemoryDataWidth = FMC_MemoryDataWidth_8b;
    FMC_NANDInitStruct->FMC_ECC = FMC_ECC_Disable;
    FMC_NANDInitStruct->FMC_ECCPageSize = FMC_ECCPageSize_256Bytes;
    FMC_NANDInitStruct->FMC_TCLRSetupTime = 0x0;
    FMC_NANDInitStruct->FMC_TARSetupTime = 0x0;
    FMC_NANDInitStruct->FMC_CommonSpaceTimingStruct->FMC_SetupTime = 0xFC;
    FMC_NANDInitStruct->FMC_CommonSpaceTimingStruct->FMC_WaitSetupTime = 0xFC;
    FMC_NANDInitStruct->FMC_CommonSpaceTimingStruct->FMC_HoldSetupTime = 0xFC;
    FMC_NANDInitStruct->FMC_CommonSpaceTimingStruct->FMC_HiZSetupTime = 0xFC;
    FMC_NANDInitStruct->FMC_AttributeSpaceTimingStruct->FMC_SetupTime = 0xFC;
    FMC_NANDInitStruct->FMC_AttributeSpaceTimingStruct->FMC_WaitSetupTime = 0xFC;
    FMC_NANDInitStruct->FMC_AttributeSpaceTimingStruct->FMC_HoldSetupTime = 0xFC;
    FMC_NANDInitStruct->FMC_AttributeSpaceTimingStruct->FMC_HiZSetupTime = 0xFC;
}

/*********************************************************************
 * @fn      FMC_SDRAM_SendCMDConfig
 *
 * @brief   Configures the send commed for the SDRAM regular.
 *
 * @param   SDRAM_Sel - SDRAM send cmd to target area.
 *            FMC_SDRAM_SEL_None - Do not send commed to Bank5 and Bank6. 
 *            FMC_SDRAM_SEL_Bank5 - Send commed to Bank5. 
 *            FMC_SDRAM_SEL_Bank6 - Send commed to Bank6. 
 *            FMC_SDRAM_SEL_Bank5_6 - Send commed to Bank5 and Bank6.
 *          CMD_Mode - commed mode.
 *            FMC_SDRAM_CMD_Mode0 - commed mode 0.
 *            FMC_SDRAM_CMD_Mode1 - commed mode 1.
 *            FMC_SDRAM_CMD_Mode2 - commed mode 2.
 *            FMC_SDRAM_CMD_Mode3 - commed mode 3.
 *            FMC_SDRAM_CMD_Mode4 - commed mode 4.
 *            FMC_SDRAM_CMD_Mode5 - commed mode 5.
 *            FMC_SDRAM_CMD_Mode6 - commed mode 6.
 *          CMD_Refresh_cnt - commed refresh count. 
 *            This parameter can be a number between 0x0 and 0xF
 *          SDRAM_ModeREG - mode rigester. 
 *            This parameter can be a number between 0x0 and 0x1FFF
 *
 * @return  none
 */
void FMC_SDRAM_SendCMDConfig(uint32_t SDRAM_Sel, uint32_t CMD_Mode, uint32_t CMD_Refresh_cnt, uint32_t SDRAM_ModeREG)
{
    FMC_Bank5_6->SDCMR = 0;
    FMC_Bank5_6->SDCMR |= SDRAM_Sel | CMD_Mode | (CMD_Refresh_cnt << 5) | (SDRAM_ModeREG << 9);
}

/*********************************************************************
 * @fn      FMC_NORSRAM_NANDCmd
 *
 * @brief   Enables or disables the specified NOR/SRAM/NAND Memory Bank.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void FMC_NORSRAM_NANDCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        FMC_Bank1->BTCR[0] |= FMC_BCR1_FMCEN;
    }
    else
    {
        FMC_Bank1->BTCR[0] &= FMC_BCR1_FMCEN;
    }
}

/*********************************************************************
 * @fn      FMC_NORSRAMCmd
 *
 * @brief   Enables or disables the specified NOR/SRAM Memory Bank.
 *
 * @param   FMC_Bank - specifies the FMC Bank to be used
 *            FMC_Bank1_NORSRAM1 - FMC Bank1 NOR/SRAM1
 *            FMC_Bank1_NORSRAM2 - FMC Bank1 NOR/SRAM2
 *            FMC_Bank1_NORSRAM3 - FMC Bank1 NOR/SRAM3
 *            FMC_Bank1_NORSRAM4 - FMC Bank1 NOR/SRAM4
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void FMC_NORSRAMCmd(uint32_t FMC_Bank, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        FMC_Bank1->BTCR[FMC_Bank] |= BCR_MBKEN_Set;
    }
    else
    {
        FMC_Bank1->BTCR[FMC_Bank] &= BCR_MBKEN_Reset;
    }
}

/*********************************************************************
 * @fn      FMC_NORSRAMCmd
 *
 * @brief   Enables or disables the specified NOR/SRAM Memory Bank.
 *
 * @param   FMC_SDRAMBank - specifies the FMC SDRAM Bank to be used
 *            FMC_Bank5_SDRAM - FMC Bank5 SDRAM
 *            FMC_Bank6_SDRAM - FMC Bank6 SDRAM
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void FMC_SDRAMCmd(uint32_t FMC_SDRAMBank, FunctionalState NewState)
{
    if(FMC_SDRAMBank == FMC_Bank5_SDRAM)
    {
        if(NewState != DISABLE)
        {
            FMC_Bank5_6->MISC |= FMC_MISC_En_Bank1;
        }
        else
        {
            FMC_Bank5_6->MISC &= ~FMC_MISC_En_Bank1;
        }
    }
    else if(FMC_SDRAMBank == FMC_Bank6_SDRAM)
    {
        if(NewState != DISABLE)
        {
            FMC_Bank5_6->MISC |= FMC_MISC_En_Bank2;
        }
        else
        {
            FMC_Bank5_6->MISC &= ~FMC_MISC_En_Bank2;
        }
    }   
}

/*********************************************************************
 * @fn      FMC_NANDCmd
 *
 * @brief   Enables or disables the specified NAND Memory Bank.
 *
 * @param   FMC_Bank - specifies the FMC Bank to be used
 *            FMC_Bank3_NAND - FMC Bank3 NAND
 *          NewStat - ENABLE or DISABLE.
 *
 * @return  none
 */
void FMC_NANDCmd(uint32_t FMC_Bank, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        if(FMC_Bank == FMC_Bank3_NAND)
        {
            FMC_Bank3->PCR |= PCR_PBKEN_Set;
        }
    }
    else
    {
        if(FMC_Bank == FMC_Bank3_NAND)
        {
            FMC_Bank3->PCR &= PCR_PBKEN_Reset;
        }
    }
}

/*********************************************************************
 * @fn      FMC_NANDECCCmd
 *
 * @brief   Enables or disables the FMC NAND ECC feature.
 *
 * @param   FMC_Bank - specifies the FMC Bank to be used
 *            FMC_Bank3_NAND - FMC Bank3 NAND
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void FMC_NANDECCCmd(uint32_t FMC_Bank, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        if(FMC_Bank == FMC_Bank3_NAND)
        {
            FMC_Bank3->PCR |= PCR_ECCEN_Set;
        }
    }
    else
    {
        if(FMC_Bank == FMC_Bank3_NAND)
        {
            FMC_Bank3->PCR &= PCR_ECCEN_Reset;
        }
    }
}

/*********************************************************************
 * @fn      FMC_GetECC
 *
 * @brief   Returns the error correction code register value.
 *
 * @param   FMC_Bank - specifies the FMC Bank to be used
 *            FMC_Bank3_NAND - FMC Bank3 NAND
 *          NewState - ENABLE or DISABLE.
 *
 * @return  eccval - The Error Correction Code (ECC) value.
 */
uint32_t FMC_GetECC(uint32_t FMC_Bank)
{
    uint32_t eccval = 0x00000000;

    if(FMC_Bank == FMC_Bank3_NAND)
    {
        eccval = FMC_Bank3->ECCR;
    }

    return (eccval);
}

/*********************************************************************
 * @fn      FMC_SDRAM_SetRefreshCnt
 *
 * @brief   Set refresh count for the SDRAM.
 *
 * @param   Refresh_Cnt -  refresh count
 *            This parameter can be a number between 0x0 and 0x1FFFF
 *
 * @return  none
 */
void FMC_SDRAM_SetRefreshCnt(uint16_t Refresh_Cnt)
{
    FMC_Bank5_6->SDRTR &= ~FMC_SDRTR_COUNT;
    FMC_Bank5_6->SDRTR |= (uint32_t)Refresh_Cnt;
}

/*********************************************************************
 * @fn      FMC_SDRAM_GetBankSta
 *
 * @brief   Returns the SDRAM bank status.
 *
 * @param   SDRAM_Bank - specifies the SDRAM Bank to be used
 *            FMC_Bank5_SDRAM - FMC Bank5 SDRAM.
 *            FMC_Bank6_SDRAM - FMC Bank6 SDRAM.
 *
 * @return  bank_sta - pointer to a FMC_NORSRAMInitTypeDef structure.
 */
FMC_SDRAM_BANK_Sta_TypeDef FMC_SDRAM_GetBankSta(uint32_t SDRAM_Bank)
{
    FMC_SDRAM_BANK_Sta_TypeDef bank_sta = FMC_SDRAM_Normal;

    if(SDRAM_Bank == FMC_Bank5_SDRAM)
    {
        bank_sta = (FMC_Bank5_6->SDSR >> 1);
    }
    else if(SDRAM_Bank == FMC_Bank6_SDRAM)
    {
        bank_sta = (FMC_Bank5_6->SDSR >> 3);
    }

    return (bank_sta);
}


/*********************************************************************
 * @fn      FMC_ITConfig
 *
 * @brief   Enables or disables the specified FMC interrupts.
 *
 * @param   FMC_IT - specifies the ADC interrupt sources to be enabled or disabled.
 *            FMC_IT_RE - refresh interrupt mask.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void FMC_ITConfig(uint32_t FMC_IT, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        FMC_Bank5_6->SDRTR |= FMC_IT;
    }
    else
    {
        FMC_Bank5_6->SDRTR &= (~(uint32_t)FMC_IT);
    }
}

/*********************************************************************
 * @fn      FMC_GetITStatus
 *
 * @brief   Checks whether the specified FMC interrupt has occurred or not.
 *
 * @param   FMC_IT - specifies the FMC interrupt sources to be enabled or disabled.
 *            FMC_IT_RE - refresh interrupt mask.
 *
 * @return  FlagStatus - SET or RESET.
 */
ITStatus FMC_GetITStatus(uint32_t FMC_IT)
{
    ITStatus bitstatus = RESET;
    uint32_t itmask = 0, enablestatus = 0;
  
    itmask = FMC_IT >> 14;
    enablestatus = FMC_Bank5_6->SDRTR & FMC_IT;

    if(((FMC_Bank5_6->SDSR & itmask) != (uint32_t)RESET) && enablestatus)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      FMC_ClearITPendingBit
 *
 * @brief   Clears the FMC's interrupt pending bits.
 *
 * @param   FMC_IT - specifies the FMC interrupt sources to be enabled or disabled.
 *            FMC_IT_RE - refresh interrupt mask.
 *
 * @return  none
 */
void FMC_ClearITPendingBit(uint32_t FMC_IT)
{
    uint32_t itmask = 0;

    itmask = (uint32_t)(FMC_IT >> 14);
    FMC_Bank5_6->SDRTR |= (uint32_t)itmask;
}

/*********************************************************************
 * @fn      FMC_GetFlagStatus
 *
 * @brief   Checks whether the specified FMC flag is set or not.
 *
 * @param   FMC_FLAG - specifies the flag to check.
 *            FMC_FLAG_FEMPT - NAND FIFO empty flag.
 *            FMC_FLAG_BUSY - SDRAM busy flag.
 *            FMC_FLAG_RE - SDRAM busy refresh error flag.
 *
 * @return  FlagStatus -  SET or RESET.
 */
FlagStatus FMC_GetFlagStatus(uint32_t FMC_FLAG)
{
    FlagStatus bitstatus = RESET;

    if(FMC_FLAG & (1 << 31))
    {
        FMC_FLAG &= 0x7FFFFFFF;
        if((FMC_Bank5_6->SDSR & FMC_FLAG) != (uint32_t)RESET)
        {
            bitstatus = SET;
        }
        else
        {
            bitstatus = RESET;
        }
    }
    else 
    {
        if((FMC_Bank3->SR & FMC_FLAG) != (uint32_t)RESET)
        {
            bitstatus = SET;
        }
        else
        {
            bitstatus = RESET;
        }    
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      FMC_ClearFlag
 *
 * @brief   Clears the FMC's pending flags.
 *
 * @param   FMC_FLAG - specifies the flag to check.
 *            FMC_FLAG_RE - SDRAM busy refresh error flag.
 *
 * @return  none
 */
void FMC_ClearFlag(uint32_t FMC_FLAG)
{
    FMC_FLAG &= 0x7FFFFFFF;
    FMC_Bank5_6->SDRTR |= FMC_FLAG;
}