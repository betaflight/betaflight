/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32h417_ecdc.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2025/03/01
 * Description        : This file provides all the ECDC firmware functions.
 *********************************************************************************
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch32h417_ecdc.h"
#include "ch32h417_rcc.h"
#include <stddef.h>
/*********************************************************************
 * @fn      ECDC_DeInit
 *
 * @brief   Deinitializes the ECDC peripheral registers to their default
 *        reset values.
 *
 * @param   none
 *
 * @return  none
 */
void ECDC_DeInit(void)
{
    RCC_HB2PeriphResetCmd(RCC_HB2Periph_ECDC, ENABLE);
    RCC_HB2PeriphResetCmd(RCC_HB2Periph_ECDC, DISABLE);
}

/*********************************************************************
 * @fn      ECDC_Init
 *
 * @brief   Function to initialize the ECDC (Encryption Controller) with the given parameters.
 *
 * @param   ECDC_InitStruct - Parameters to initialize the ECDC.
 *
 * @return  none
 */
void ECDC_Init(ECDC_InitTypeDef *ECDC_InitStruct)
{
    ECDC->INT_FG = 0xff;

    ECDC->CTRL &= ~(ECDC_WRSRAM_EN | ECDC_ALGRM_MOD | ECDC_CIPHER_MOD | ECDC_KLEN_MASK | ECDC_DAT_MOD | ECDC_KEYEX_EN |
                    ECDC_MODE_SEL | ECDC_NORMAL_EN);

    ECDC->CTRL |= ((uint32_t)ECDC_InitStruct->Algorithm << 8) | ((uint32_t)ECDC_InitStruct->BlockCipherMode << 9) |
                  ((uint32_t)ECDC_InitStruct->KeyLen << 10) | ((uint32_t)ECDC_InitStruct->ExcuteEndian << 13);
    ECDC_SetKey(ECDC_InitStruct->Key);

    if (ECDC_InitStruct->BlockCipherMode == ECDCBlockCipherMode_CTR && ECDC_InitStruct->IV != NULL)
    {
        ECDC_CTR_SetCounter(ECDC_InitStruct->IV);
    }

    ECDC_KeyExCmd(ENABLE);
    for(int i = 0; i < (HCLKClock / SystemClock + 1) * 64 * (((ECDC->CTRL >> 4) & 0x7) == 0 ? (1) : (((ECDC->CTRL >> 4) & 0x7))); i++)
    {
        __asm volatile ("\tnop");
    }
    ECDC_KeyExCmd(DISABLE);

    ECDC->CTRL |= ECDC_InitStruct->ExcuteMode | ((uint32_t)ECDC_InitStruct->ExcuteEndian << 13);
}

/*********************************************************************
 * @fn      ECDC_StructInit
 *
 * @brief   Fills each ECDC_InitStruct member with its default value.
 *
 * @param   ECDC_InitStruct - pointer to an ECDC_InitTypeDef structure that
 *        contains the configuration information for the specified ECDC
 *        peripheral.
 *
 * @return  none
 */
void ECDC_StructInit(ECDC_InitTypeDef *ECDC_InitStruct)
{
    ECDC_InitStruct->ExcuteMode = 0;
    ECDC_InitStruct->Algorithm = ECDCAlgorithm_SM4;
    ECDC_InitStruct->BlockCipherMode = ECDCBlockCipherMode_CTR;
    ECDC_InitStruct->KeyLen = ECDCKeyLen_128b;
    ECDC_InitStruct->ExcuteEndian = ECDCExcuteEndian_Little;
}

/*********************************************************************
 * @fn      ECDC_CTR_SetCounter
 *
 * @brief   This function sets the counter of the ECDC module for CTR mode.
 *
 * @param   ECDC_IvStruct - pointer to an ECDC_IV_TypeDef structure.
 *
 * @return  none
 */
void ECDC_CTR_SetCounter(ECDC_IV_TypeDef *ECDC_IvStruct)
{
    ECDC->IV_31T0 = ECDC_IvStruct->IV_31T0;
    ECDC->IV_63T32 = ECDC_IvStruct->IV_63T32;
    ECDC->IV_95T64 = ECDC_IvStruct->IV_95T64;
    ECDC->IV_127T96 = ECDC_IvStruct->IV_127T96;
}

/*********************************************************************
 * @fn      ECDC_SetKey
 *
 * @brief   This function sets the key of the ECDC module.
 *
 * @param   ECDC_IV - pointer to an ECDC_IV_TypeDef structure.
 *
 * @return  none
 */
void ECDC_SetKey(ECDC_KEY_TypeDef *ECDC_KeyStruct)
{
    ECDC->KEY_31T0 = ECDC_KeyStruct->KEY_31T0;
    ECDC->KEY_63T32 = ECDC_KeyStruct->KEY_63T32;
    ECDC->KEY_95T64 = ECDC_KeyStruct->KEY_95T64;
    ECDC->KEY_127T96 = ECDC_KeyStruct->KEY_127T96;
    if (((ECDC->CTRL >> 10) & 0x3) >= ECDCKeyLen_192b)
    {
        ECDC->KEY_159T128 = ECDC_KeyStruct->KEY_159T128;
        ECDC->KEY_191T160 = ECDC_KeyStruct->KEY_191T160;
    }
    if (((ECDC->CTRL >> 10) & 0x3) >= ECDCKeyLen_256b)
    {
        ECDC->KEY_223T192 = ECDC_KeyStruct->KEY_223T192;
        ECDC->KEY_255T224 = ECDC_KeyStruct->KEY_255T224;
    }
}

/*********************************************************************
 * @fn      ECDC_SingleWR_RawData
 *
 * @brief   Function to write raw data for a single encrypt or decrypt
 *
 * @param   WriteDataPointer - Pointer to the data to be written.
 *
 * @return  none
 */
void ECDC_SingleWR_RawData(uint32_t *WriteDataPointer)
{
    ECDC->SGSD_127T96 = WriteDataPointer[3];
    ECDC->SGSD_95T64 = WriteDataPointer[2];
    ECDC->SGSD_63T32 = WriteDataPointer[1];
    ECDC->SGSD_31T0 = WriteDataPointer[0];
}

/*********************************************************************
 * @fn      ECDC_SingleRD_EcdcData
 *
 * @brief   Function to read  data after encrypt or decrypt for a single encrypt or decrypt
 *
 * @param   ReadDataPointer - Pointer to the data to be read.
 *
 * @return  none
 */
void ECDC_SingleRD_EcdcData(uint32_t *ReadDataPointer)
{
    *ReadDataPointer++ = ECDC->SGRT_31T0;
    *ReadDataPointer++ = ECDC->SGRT_63T32;
    *ReadDataPointer++ = ECDC->SGRT_95T64;
    *ReadDataPointer++ = ECDC->SGRT_127T96;
}

/*********************************************************************
 * @fn      ECDC_SetSRC_BaseAddr
 *
 * @brief   Set source base address in RAM
 *
 * @param   Address - source base address in RAM.(Address%32 == 0)
 *          Len - Encrypt or decrypt data length.(Len%32 == 0)
 *
 * @return  none
 */
void ECDC_SetSRC_BaseAddr(uint32_t Address, uint32_t Len)
{
    ECDC->SRC_ADDR = Address;
    ECDC->SRAM_LEN = Len;
}

/*********************************************************************
 * @fn      ECDC_SetDST_BaseAddr
 *
 * @brief   Set destination  base address in RAM
 *
 * @param   Address - source base address in RAM.(Address%32 == 0)
 *
 * @return  none
 */
void ECDC_SetDST_BaseAddr(uint32_t Address)
{
    ECDC->DST_ADDR = Address;
}

/*********************************************************************
 * @fn      ECDC_HardwareClockCmd
 *
 * @brief   Enables or disables hardware clock for the ECDC .
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void ECDC_HardwareClockCmd(FunctionalState NewState)
{
    if (NewState)
    {
        ECDC->CTRL |= ECDC_ECDC_SM4_CLOCK_EN;
    }
    else
    {
        ECDC->CTRL &= ~ECDC_ECDC_SM4_CLOCK_EN;
    }
}

/*********************************************************************
 * @fn      ECDC_ClockConfig
 *
 * @brief   Configures the ECDC clock source and divider factor.
 *
 * @param   RCC_PLLDiv - specifies the PLL Division factor.
 *
 *
 * @return  none
 */
void ECDC_ClockConfig(uint8_t ECDC_PLLDiv)
{
    ECDC->CTRL &= ~ECDC_CLKDIV_MASK;
    ECDC->CTRL &= ~ECDC_CLOCK_SELECT;
    if (ECDC_PLLDiv == ECDC_ClockSource_PLLCLK_Div1)
    {
        ECDC->CTRL |= ECDC_CLOCK_SELECT;
    }
    else
    {
        ECDC->CTRL |= (uint16_t)ECDC_PLLDiv;
    }
}

/*********************************************************************
 * @fn      ECDC_KeyExCmd
 *
 * @brief   Enables or disables key extend for the ECDC .
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void ECDC_KeyExCmd(FunctionalState NewState)
{
    if (NewState)
    {
        ECDC->CTRL |= ECDC_KEYEX_EN;
    }
    else
    {
        ECDC->CTRL &= ~ECDC_KEYEX_EN;
    }
}

/*********************************************************************
 * @fn      ECDC_ITConfig
 *
 * @brief   Enables or disables the specified ECDC interrupts.
 *
 * @param   ECDC_IT - specifies the ECDC interrupts sources to be enabled or disabled.
 *            ECDC_IT_KeyEx_END - key extend end Interrupt source.
 *            ECDC_IT_Single_END - Single end Interrupt source.
 *            ECDC_IT_RAM2RAM_END - RAM to RAM end Interrupt source.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void ECDC_ITConfig(uint32_t ECDC_IT, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        ECDC->CTRL |= ECDC_IT;
    }
    else
    {
        ECDC->CTRL &= (uint8_t)~ECDC_IT;
    }
}

/*********************************************************************
 * @fn      ECDC_GetFlagStatus
 *
 * @brief   Checks whether the specified ECDC flag is set or not.
 *
 * @param   ECDC_FLAG - specifies the ECDC interrupt source to check..
 *            ECDC_FLAG_KeyEx_END - key extend end flag.
 *            ECDC_FLAG_Single_END - Single end flag.
 *            ECDC_FLAG_RAM2RAM_END - RAM to RAM end flag.
 *
 * @return  none
 */
FlagStatus ECDC_GetFlagStatus(uint32_t ECDC_FLAG)
{
    FlagStatus bitstatus = RESET;

    if ((ECDC->INT_FG & ECDC_FLAG) != (uint8_t)RESET)
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
 * @fn      ECDC_ClearFlag
 *
 * @brief   Clears the ECDC's pending flags.
 *
 * @param   ECDC_FLAG - clear pending flags..
 *            ECDC_FLAG_KeyEx_END - key extend end flag.
 *            ECDC_FLAG_Single_END - Single end flag.
 *            ECDC_FLAG_RAM2RAM_END - RAM to RAM end flag.
 *
 * @return  none
 */
void ECDC_ClearFlag(uint32_t ECDC_FLAG)
{
    ECDC->INT_FG |= (uint32_t)ECDC_FLAG;
}

/*********************************************************************
 * @fn      ECDC_GetITStatus
 *
 * @brief   Checks whether the ECDC interrupt has occurred or not.
 *
 * @param   ECDC_IT - specifies the ECDC interrupts sources to be enabled or disabled.
 *            ECDC_IT_KeyEx_END - key extend end Interrupt source.
 *            ECDC_IT_Single_END - Single end Interrupt source.
 *            ECDC_IT_RAM2RAM_END - RAM to RAM end Interrupt source.
 *
 * @return  none
 */
ITStatus ECDC_GetITStatus(uint32_t ECDC_IT)
{
    ITStatus bitstatus = RESET;

    if (((ECDC->INT_FG & ECDC_IT) != (uint32_t)RESET))
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
 * @fn      ECDC_ClearITPendingBit
 *
 * @brief   Clears the ECDC's interrupt pending bits.
 *
 * @param   ECDC_IT - specifies the ECDC interrupts sources to be enabled or disabled.
 *            ECDC_IT_KeyEx_END - key extend end Interrupt source.
 *            ECDC_IT_Single_END - Single end Interrupt source.
 *            ECDC_IT_RAM2RAM_END - RAM to RAM end Interrupt source.
 *
 * @return  none
 */
void ECDC_ClearITPendingBit(uint32_t ECDC_IT)
{
    ECDC->INT_FG |= (uint32_t)ECDC_IT;
}
