/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32h417_serdes.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2025/03/01
 * Description        : This file provides all the GPIO firmware functions.
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch32h417_serdes.h"
#include "ch32h417.h"
#include <stddef.h>
/*********************************************************************
 * @fn      SDS_Config
 * 
 * @brief   The function `SDS_Config` configures a SDS device based on the provided configuration and waits for PLL lock before
 *        returning a status.
 * 
 * @param   sds The `sds` parameter is a pointer to a structure of type `SDS_TypeDef`.
 *          sds_cfg The `sds_cfg` parameter is a pointer to a structure of type `SDS_CFG_TypeDef`.
 * 
 * @return  The function `SDS_Config` returns either `NoREADY` or `READY` based on certain conditions. If the condition
 *        inside the `while` loop is not met within the specified timeout, it returns `NoREADY`. Otherwise, it returns `READY`.
 */
ErrorStatus SDS_Config(SDS_TypeDef *sds, SDS_CFG_TypeDef *sds_cfg)
{
    uint32_t time_out = 0;
    sds->CTRL = *(uint32_t *)sds_cfg;
    while (!(SDS_ReadCOMMAFlagBit(sds) & SDSIT_ST_PLLLock))
    {
        if (time_out++ > 0x10000)
            return NoREADY;
    }
    return READY;
}

/*********************************************************************
 * @fn      SDS_DMA_Tx_CFG
 * 
 * @brief   The function `SDS_DMA_Tx_CFG` sets custom word, DMA address, and data length in a SDS structure.
 * 
 * @param   sds The `sds` parameter is a pointer to a structure of type `SDS_TypeDef`.
 *          DMAaddr DMAaddr is the memory address where the data to be transmitted is located.
 *          DataLen The `DataLen` parameter in the `SDS_DMA_Tx_CFG` function represents the length of the data to be
 *        transmitted via DMA (Direct Memory Access). It specifies the number of data elements to be transferred.
 *          CustomWord CustomWord is a 32-bit unsigned integer that is written to the DATA0 register of the SDS_TypeDef
 *        structure.
 * 
 * @return  none
 */
void SDS_DMA_Tx_CFG(SDS_TypeDef *sds, uint32_t DMAaddr, uint32_t DataLen, uint32_t CustomWord)
{
    sds->DATA0 = CustomWord;
    sds->DMA_0 = DMAaddr;
    *(uint16_t *)(((void *)&(sds->RTX_CTRL))) = DataLen;
}

/*********************************************************************
 * @fn      SDS_DMA_Rx_CFG
 * 
 * @brief   The function `SDS_DMA_Rx_CFG` configures DMA receive addresses and data lengths for an SDS device, returning an error
 * status if the addresses are not aligned.
 * 
 * @param   sds SDS_TypeDef pointer representing the SDS peripheral
 *          DMAaddr0 DMAaddr0 is the memory address for the first DMA transfer.
 *          DataLen0 It seems like you forgot to provide the definition for the parameter `DataLen0`. Could you please
 *        provide more information about what `DataLen0` represents in the context of the `SDS_DMA_Rx_CFG` function?
 *          DMAaddr1 DMAaddr1 is the memory address for the second block of data to be transferred via DMA.
 *          DataLen1 It seems like you missed providing the definition for the parameter `DataLen1`. Could you please provide
 *        more information or clarify what `DataLen1` represents in the context of the `SDS_DMA_Rx_CFG` function?
 * 
 * @return  The function `SDS_DMA_Rx_CFG` will return either `NoREADY` or `READY` based on the conditions specified in the
 *        code snippet.
 */
ErrorStatus SDS_DMA_Rx_CFG(SDS_TypeDef *sds, uint32_t DMAaddr0, uint32_t DataLen0, uint32_t DMAaddr1, uint32_t DataLen1)
{
    if ((DMAaddr0 & 0xf) || (DMAaddr1 & 0xf))
    {
        return NoREADY;
    }
    sds->DMA_1 = DMAaddr1;
    sds->DMA_0 = DMAaddr0;

    return READY;
}

/*********************************************************************
 * @fn      SDS_GetFirstWord
 * 
 * @brief   The function `SDS_GetFirstWord` returns the first word from a specified buffer in an SDS_TypeDef structure.
 * 
 * @param   sds SDS_TypeDef structure pointer containing two data fields DATA0 and DATA1
 *          buffnum `buffnum` is an input parameter of type `uint8_t` that determines which buffer's data to retrieve. If
 *        `buffnum` is 0, the function returns the value stored in `sds->DATA0`. If `buffnum` is not 0, the function returns
 * 
 * @return  The function `SDS_GetFirstWord` returns the first word from the specified buffer number in the `SDS_TypeDef`
 *        structure. If `buffnum` is 0, it returns `sds->DATA0`, otherwise it returns `sds->DATA1`.
 */
uint32_t SDS_GetFirstWord(SDS_TypeDef *sds, uint8_t buffnum)
{
    if (buffnum == 0)
    {
        return sds->DATA0;
    }
    else
    {
        return sds->DATA1;
    }
}

/*********************************************************************
 * @fn      SDS_RTX_Ctrl
 * 
 * @brief   The function SDS_RTX_Ctrl sets a specific field in the RTX_CTRL register of an SDS_TypeDef struct based on the value in
 *        a SDSRTXCtrl_TypeDef struct.
 * 
 * @param   sds The `sds` parameter is a pointer to a structure of type `SDS_TypeDef`.
 *          ctrl The `ctrl` parameter is a pointer to a structure of type `SDSRTXCtrl_TypeDef`.
 * 
 * @return  none
 */
void SDS_RTX_Ctrl(SDS_TypeDef *sds, SDSRTXCtrl_TypeDef *ctrl)
{
    *(uint16_t *)(((void *)&(sds->RTX_CTRL)) + 2) = (*(uint32_t *)ctrl) << 16;
}

/*********************************************************************
 * @fn      SDS_ClearIT
 * 
 * @brief   The function `SDS_ClearIT` clears specific interrupt flags in the `STATUS` register of a `SDS_TypeDef` structure.
 * 
 * @param   sds The `sds` parameter is a pointer to a structure of type `SDS_TypeDef`.
 *          sdsit_fl The parameter `sdsit_fl` is of type `SDSIT_EN_TypeDef`, which is likely an enumeration type representing
 *        different flags or options related to an SDS (Synchronous Data Stream) operation.
 * 
 * @return  none
 */
void SDS_ClearIT(SDS_TypeDef *sds, SDSIT_EN_TypeDef sdsit_fl)
{
    sds->STATUS |= (sdsit_fl << 16);
}

/*********************************************************************
 * @fn      SDS_ReadIT
 * 
 * @brief   function `SDS_ReadIT` reads the interrupt status from a specified SDS device.
 * 
 * @param   sds The parameter `sds` is a pointer to a structure of type `SDS_TypeDef`.
 * 
 * @return  The function `SDS_ReadIT` is returning the value of the STATUS field of the `sds` structure shifted right by 16
 *        bits.
 */
SDSIT_EN_TypeDef SDS_ReadIT(SDS_TypeDef *sds)
{
    return (sds->STATUS >> 16);
}

/*********************************************************************
 * 
 * @brief   function SDS_ConfigIT configures the interrupt enable settings for a SDS peripheral.
 * 
 * @param   sds The `sds` parameter is a pointer to a structure of type `SDS_TypeDef`.
 *          sdsit_en The parameter `sdsit_en` is of type `SDSIT_EN_TypeDef`, which is likely an enumeration type representing
 *        different interrupt enable options for the SDS (Synchronous Data Stream) module.
 * 
 * @return  none
 */
void SDS_ConfigIT(SDS_TypeDef *sds, SDSIT_EN_TypeDef sdsit_en)
{
    sds->INT_EN |= (sdsit_en);
}

/*********************************************************************
 * @fn      SDS_ReadCOMMAFlagBit
 * 
 * @brief   The function `SDS_ReadCOMMAFlagBit` reads a specific flag bit from the STATUS field of an SDS_TypeDef structure.
 * 
 * @param   sds The parameter `sds` is a pointer to a structure of type `SDS_TypeDef`.
 * 
 * @return  The function `SDS_ReadCOMMAFlagBit` is returning a specific bit field from the `STATUS` member of the
 *        `SDS_TypeDef` structure pointed to by the `sds` parameter. It shifts the `STATUS` value right by 8 bits and then applies
 *        a bitwise AND operation with `0xf` to extract a 4-bit field. This extracted field is then returned as
 */
SDSIT_ST_TypeDef SDS_ReadCOMMAFlagBit(SDS_TypeDef *sds)
{
    return (sds->STATUS >> 8) & 0xf;
}

/*********************************************************************
 * @fn      SDS_GetCurrentDMA
 * 
 * @brief   The function SDS_GetCurrentDMA retrieves the current DMA addresses from a given SDS_TypeDef structure.
 * 
 * @param   sds SDS_TypeDef structure pointer containing DMA information.
 *          Addr0 Addr0 is a pointer to a uint32_t variable where the function will store the value of sds->DMA_0 if Addr0 is
 *        not NULL.
 *          Addr1 Addr1 is a pointer to a uint32_t variable where the function will store the value of sds->DMA_1 if Addr1 is
 *        not NULL.
 * 
 * @return  none
 */
void SDS_GetCurrentDMA(SDS_TypeDef *sds,uint32_t *Addr0, uint32_t *Addr1)
{
    if (Addr0 != NULL)
    {
        *Addr0 = sds->DMA_0;
    }
    if (Addr1 != NULL)
    {
        *Addr1 = sds->DMA_1;
    }
}
