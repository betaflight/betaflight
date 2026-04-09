/********************************** (C) COPYRIGHT  ******************************
* File Name          : ch32h417_sdmmc.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the SDMMC firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_sdmmc.h"
#include "ch32h417.h"

#define SDMMC_BusWidth_MASK   ((uint8_t)EMMC_LW_MASK)

/*********************************************************************
 * @fn      SDMMC_DeInit
 *
 * @brief   Deinitializes the SDMMC peripheral registers to their default
 *            reset values.
 *
 * @return  none
 */
void SDMMC_DeInit(void)
{
    RCC_HBPeriphResetCmd(RCC_HBPeriph_SDMMC, ENABLE);
    RCC_HBPeriphResetCmd(RCC_HBPeriph_SDMMC, DISABLE);
}

/*********************************************************************
 * @fn      SDMMC_Init
 *
 * @brief   Initializes the SDMMC peripheral according to the specified
 *            parameters in the SDMMC_InitStruct.
 *
 * @param   SDMMC_InitStruct - pointer to a SDMMC_InitTypeDef structure
 *            that contains the configuration information for the SDMMC
 *            peripheral.
 *
 * @return  none
 */
void SDMMC_Init(SDMMC_InitTypeDef *SDMMC_InitStruct)
{
    uint16_t tmpreg;

    SDMMC->CONTROL = 0;
    tmpreg = (SDMMC_InitStruct->SDMMC_Mode & SDMMC_Mode_Slave) |
            (uint16_t)(SDMMC_InitStruct->SDMMC_BusWidth & SDMMC_BusWidth_MASK) |
            (uint16_t)(SDMMC_InitStruct->SDMMC_ClockEdge & SDMMC_SampleClock_Falling) |
            ((uint16_t)SDMMC_InitStruct->SDMMC_DMA_EN << 3) | ((uint16_t)SDMMC_InitStruct->SDMMC_SlaveForceCrc_ERR << 9);
    SDMMC->CONTROL = tmpreg;

    SDMMC->CLK_DIV = 0;
    tmpreg = (SDMMC_InitStruct->SDMMC_PhaseInv & SDMMC_Phase_Inverse) |
            (SDMMC_InitStruct->SDMMC_ClockSpeed & SDMMC_ClockSpeed_High) |
            (uint16_t)(SDMMC_InitStruct->SDMMC_ClockDiv & EMMC_DIV_MASK) |
            ((uint16_t)SDMMC_InitStruct->SDMMC_Clock_OE << 8);
    SDMMC->CLK_DIV = tmpreg;

    SDMMC->TIMEOUT = SDMMC_InitStruct->SDMMC_TimeOut;
}

/*********************************************************************
 * @fn      SDMMC_InternalLogicReset
 *
 * @brief   Reset internal logic of the SDMMC controller.
 *
 * @param   none
 *
 * @return  none
 */
void SDMMC_InternalLogicReset(void)
{
    SDMMC->CONTROL = (uint16_t)(EMMC_RST_LGC | EMMC_ALL_CLR);
}

/*********************************************************************
 * @fn      SDMMC_SetBusWidth
 *
 * @brief   Sets the Bus Width of the SDMMC controller.
 *
 * @param   BusWidth - SDMMC Bus Width.
 *            SDMMC_BusWidth_1 - one data line
 *            SDMMC_BusWidth_4 - 4 data line
 *            SDMMC_BusWidth_8 - 8data line
 *
 * @return  none
 */
void SDMMC_SetBusWidth(uint8_t BusWidth)
{
    SDMMC->CONTROL &= (uint16_t)~SDMMC_BusWidth_MASK;
    SDMMC->CONTROL |= (uint16_t)(BusWidth & SDMMC_BusWidth_MASK);
}

/*********************************************************************
 * @fn      SDMMC_SetClockSpeed
 *
 * @brief   Sets the SDCK clock speed of the SDMMC controller.
 *
 * @param   ClockMode - SDMMC_ClockSpeed.
 *            SDMMC_ClockSpeed_Low - low speed mode.
 *            SDMMC_ClockSpeed_High - High speed mode.
 *          ClockDIV - SDMMC_DIV_MASK.
 *            range form 0x00 to 0x1F.
 *
 * @return  none
 */
void SDMMC_SetClockSpeed(uint16_t ClockMode, uint16_t ClockDIV)
{
    SDMMC->CLK_DIV &= ~(EMMC_CLKOE | EMMC_DIV_MASK);
    SDMMC->CLK_DIV |= ClockMode | ClockDIV | EMMC_CLKOE;
    // printf("SDMMC->CLK_DIV=%X\r\n",SDMMC->CLK_DIV);
}

/*********************************************************************
 * @fn      SDMMC_ClockCmd
 *
 * @brief   Set the SDCK clock output enable or disable. Use PB1 for
 *        SDCK, if other remap of SDMMC was selected, the code should be modified.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void SDMMC_ClockCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        SDMMC->CLK_DIV |= (uint16_t)EMMC_CLKOE;
    }
    else
    {
        SDMMC->CLK_DIV &= (uint16_t)~EMMC_CLKOE;
    }
}

/*********************************************************************
 * @fn      SDMMC_SetCommand
 *
 * @brief   Set SDIO command parameter and send command.
 *
 * @param   SDMMC_CMDInitStruct - pointer to a SDMMC_CMDInitTypeDef structure
 *        that contains the configuration information for the SDMMC peripheral.
 *
 * @return  none
 */
void SDMMC_CommandConfig(SDMMC_CMDInitTypeDef *SDMMC_CMDInitStruct)
{
    // SDMMC->ARGUMENT = 0;
    //  SDMMC->CMD_SET=0;
    SDMMC->ARGUMENT = SDMMC_CMDInitStruct->SDMMC_Argument;
    SDMMC->CMD_SET = ((uint16_t)SDMMC_CMDInitStruct->SDMMC_CheckCRC << 10) | 
                     ((uint16_t)SDMMC_CMDInitStruct->SDMMC_CheckIdx << 11)  | 
                     (uint16_t)(SDMMC_CMDInitStruct->SDMMC_CMDIdx & EMMC_CMDIDX_MASK) |
                     (SDMMC_CMDInitStruct->SDMMC_RespExpect & EMMC_RPTY_MASK);
}

/*********************************************************************
 * @fn      SDMMC_GetResponse
 *
 * @brief   Obtain the value in the RESPONSE0~RESPONSE3 registers.
 *
 * @param   SDMMC_Resp - specifys the RESPONSEn register.
 *            Response0 - RESPONSE0 register.
 *            Response1 - RESPONSE1 register.
 *            Response2 - RESPONSE2 register.
 *            Response3 - RESPONSE3 register.
 *
 * @return  uint32_t type value in specified register.
 */
uint32_t SDMMC_GetResponse(SDMMC_RespEnumTypeDef SDMMC_Resp)
{
    return *((__IO uint32_t *)(((uint32_t *)&(SDMMC->RESPONSE0)) + (uint32_t)SDMMC_Resp));
}

/*********************************************************************
 * @fn      SDMMC_DMAContinueWrite
 *
 * @brief   Continue the SDMMC DMA write process.
 *
 * @return  none
 */
void SDMMC_DMAContinueWrite(void)
{
    SDMMC->WRITE_CONT = 0;
}

/*********************************************************************
 * @fn      SDMMC_GetStatus_LineData0
 *
 * @brief   Get DATA0 line voltage level.
 *
 * @return  FlagStatus - SET or RESET.
 */
FlagStatus SDMMC_GetStatus_LineData0(void)
{
    FlagStatus bitstatus = RESET;

    if((SDMMC->STATUS & EMMC_DAT0STA) != (uint32_t)RESET)
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
 * @fn      SDMMC_GetStatus_LineCMD
 *
 * @brief   Get CMD line voltage level.
 *
 * @return  FlagStatus - SET or RESET.
 */
FlagStatus SDMMC_GetStatus_LineCMD(void)
{
    FlagStatus bitstatus = RESET;

    if((SDMMC->STATUS & EMMC_CMDSTA) != (uint32_t)RESET)
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
 * @fn      SDMMC_GetBlockNumSuccess
 *
 * @brief   Get block number that have been transmit successfully.
 *
 * @return  uint16_t type value specified success block number.
 */
uint16_t SDMMC_GetBlockNumSuccess(void)
{
    return (uint16_t)((uint16_t)SDMMC->STATUS & EMMC_MASK_BLOCK_NUM);
}

/*********************************************************************
 * @fn      SDMMC_GetFlagStatus
 *
 * @brief   Get the flag that indicates the SDMMC controller status.
 *
 * @param   SDMMC_Flag - flag selection bit.
 *            SDMMC_FLAG_SLV_BUF_RELEASE - SLV_BUF_RELEASE flag
 *            SDMMC_FLAG_SDIOINT - SDIOINT flag
 *            SDMMC_FLAG_FIFO_OV - FIFO_OV flag
 *            SDMMC_FLAG_BKGAP - BKGAP flag
 *            SDMMC_FLAG_TRANDONE - TRANDONE flag
 *            SDMMC_FLAG_TRANERR - TRANERR flag
 *            SDMMC_FLAG_DATTMO - DATTMO flag
 *            SDMMC_FLAG_CMDDONE - CMDDONE  flag
 *            SDMMC_FLAG_REIDX_ER - REIDX_ER flag
 *            SDMMC_FLAG_RECRC_WR - RECRC_WR flag
 *            SDMMC_FLAG_RE_TMOUT - RE_TMOUT flag
 *
 * @return  FlagStatus - SET or RESET.
 */
FlagStatus SDMMC_GetFlagStatus(uint16_t SDMMC_Flag)
{
    if((SDMMC->INT_FG & SDMMC_Flag) != (uint16_t)RESET)
    {
        return SET;
    }
    else
    {
        return RESET;
    }
}

/*********************************************************************
 * @fn      SDMMC_ClearFlags
 *
 * @brief   Clear the flags of SDMMC controller.
 *
 * @param   SDMMC_Flag - flag selection bit.
 *            SDMMC_FLAG_SLV_BUF_RELEASE - SLV_BUF_RELEASE flag
 *            SDMMC_FLAG_SDIOINT - SDIOINT flag
 *            SDMMC_FLAG_FIFO_OV - FIFO_OV flag
 *            SDMMC_FLAG_BKGAP - BKGAP flag
 *            SDMMC_FLAG_TRANDONE - TRANDONE flag
 *            SDMMC_FLAG_TRANERR - TRANERR flag
 *            SDMMC_FLAG_DATTMO - DATTMO flag
 *            SDMMC_FLAG_CMDDONE - CMDDONE  flag
 *            SDMMC_FLAG_REIDX_ER - REIDX_ER flag
 *            SDMMC_FLAG_RECRC_WR - RECRC_WR flag
 *            SDMMC_FLAG_RE_TMOUT - RE_TMOUT flag
 *
 * @return  none.
 */
void SDMMC_ClearFlags(uint16_t SDMMC_Flag)
{
    SDMMC->INT_FG = SDMMC_Flag;
}

/*********************************************************************
 * @fn      SDMMC_ITConfig
 *
 * @brief   Enable or disable the SDMMC controller flag to generate interrupt.
 *
 * @param   SDMMC_IT - flag interrupt selection bit.
 *            SDMMC_IT_SDIOINT - SDIOINT interrupt enable 
 *            SDMMC_IT_FIFO_OV - FIFO_OV interrupt enable 
 *            SDMMC_IT_BKGAP - BKGAP interrupt enable 
 *            SDMMC_IT_TRANDONE - TRANDONE interrupt enable 
 *            SDMMC_IT_TRANERR - TRANERR interrupt enable 
 *            SDMMC_IT_DATTMO - DATTMO interrupt enable 
 *            SDMMC_IT_CMDDONE - CMDDONE interrupt enable 
 *            SDMMC_IT_REIDX_ER - REIDX_ER interrupt enable 
 *            SDMMC_IT_RECRC_WR - RECRC_WR interrupt enable 
 *            SDMMC_IT_RE_TMOUT - RE_TMOUT interrupt enable 
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none.
 */
void SDMMC_ITConfig(uint16_t SDMMC_IT, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        SDMMC->INT_EN |= SDMMC_IT;
    }
    else
    {
        SDMMC->INT_EN &= ~SDMMC_IT;
    }
}

/*********************************************************************
 * @fn      SDMMC_GetITStatus
 *
 * @brief   Get the interrupt state of SDMMC controller flag.
 *
 * @param   SDMMC_IT - flag interrupt selection bit.
 *            SDMMC_IT_SDIOINT - SDIOINT interrupt flag 
 *            SDMMC_IT_FIFO_OV - FIFO_OV interrupt flag 
 *            SDMMC_IT_BKGAP - BKGAP interrupt flag 
 *            SDMMC_IT_TRANDONE - TRANDONE interrupt flag 
 *            SDMMC_IT_TRANERR - TRANERR interrupt flag 
 *            SDMMC_IT_DATTMO - DATTMO interrupt flag 
 *            SDMMC_IT_CMDDONE - CMDDONE interrupt flag 
 *            SDMMC_IT_REIDX_ER - REIDX_ER interrupt flag 
 *            SDMMC_IT_RECRC_WR - RECRC_WR interrupt flag 
 *            SDMMC_IT_RE_TMOUT - RE_TMOUT interrupt flag 
 *
 * @return  FlagStatus - SET or RESET.
 */
ITStatus SDMMC_GetITStatus(uint16_t SDMMC_IT)
{
    ITStatus bitstatus = RESET;
    uint32_t enablestatus = 0;

    enablestatus = (SDMMC->INT_EN & (uint16_t)SDMMC_IT);

    if(((SDMMC->INT_FG & SDMMC_IT) != (uint32_t)RESET) && enablestatus)
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
 * @fn      SDMMC_ClearITPendingBits
 *
 * @brief   Clear the interrupt pending bits of SDMMC controller flag.
 *
 * @param   SDMMC_IT - flag interrupt selection bit.
 *            SDMMC_IT_SDIOINT - SDIOINT interrupt flag 
 *            SDMMC_IT_FIFO_OV - FIFO_OV interrupt flag 
 *            SDMMC_IT_BKGAP - BKGAP interrupt flag 
 *            SDMMC_IT_TRANDONE - TRANDONE interrupt flag 
 *            SDMMC_IT_TRANERR - TRANERR interrupt flag 
 *            SDMMC_IT_DATTMO - DATTMO interrupt flag 
 *            SDMMC_IT_CMDDONE - CMDDONE interrupt flag 
 *            SDMMC_IT_REIDX_ER - REIDX_ER interrupt flag 
 *            SDMMC_IT_RECRC_WR - RECRC_WR interrupt flag 
 *            SDMMC_IT_RE_TMOUT - RE_TMOUT interrupt flag 
 *
 * @return  none.
 */
void SDMMC_ClearITPendingBits(uint16_t SDMMC_IT)
{
    SDMMC->INT_FG = SDMMC_IT;
}

/*********************************************************************
 * @fn      SDMMC_BlockConfig
 *
 * @brief   Set the block size and block number in byte of data of
 *          SDMMC controller.
 *
 * @param   BlockSize - block size of target SDIO device.
 *            range form 1 to 2048.
 *          BlockNum - block number of data.
 *            range form 1 to 65535.
 *
 * @return  none.
 */
void SDMMC_BlockConfig(uint32_t BlockSize, uint32_t BlockNum)
{
    SDMMC->BLOCK_CFG = (uint32_t)((BlockSize << 16) | BlockNum);
    // __NOP( );

}

/*********************************************************************
 * @fn      SDMMC_TranMode_Init
 *
 * @brief   Set the transmission mode of SDMMC controller.
 *
 * @param   SDMMC_TranModeStruct - pointer to a SDMMC_TranModeTypeDef structure
 *        that contains the configuration information for the SDMMC peripheral.
 *
 * @return  none.
 */
void SDMMC_TranMode_Init(SDMMC_TranModeTypeDef *SDMMC_TranModeStruct)
{
    SDMMC->TRAN_MODE = 0;
    SDMMC->TRAN_MODE = ((uint32_t)(SDMMC_TranModeStruct->TranMode_Direction)) | 
          (SDMMC_TranModeStruct->TranMode_DDR_ClockSW_Mode) |
          (((uint32_t)(SDMMC_TranModeStruct->TranMode_DualDMA_TNCnt) << 8)) |
          ((uint32_t)SDMMC_TranModeStruct->TranMode_DualDMA) | ((uint32_t)SDMMC_TranModeStruct->TranMode_AutoGapStop) | 
          ((uint32_t)SDMMC_TranModeStruct->TranMode_GapStop) | ((uint32_t)SDMMC_TranModeStruct->TranMode_Boot) | 
          ((uint32_t)SDMMC_TranModeStruct->TranMode_DDR_EN) | ((uint32_t)SDMMC_TranModeStruct->TranMode_DDR_ClockFall_Check);
         
}

/*********************************************************************
 * @fn      SDMMC_SetAUTOGAPStop
 *
 * @brief   Get the EMMC_AUTOGAPSTOP bit status.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none.
 */
void SDMMC_SetAUTOGAPStop(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        SDMMC->TRAN_MODE |= (uint32_t)EMMC_AUTOGAPSTOP | EMMC_GAP_STOP;
    }
    else
    {
        SDMMC->TRAN_MODE &= (uint32_t)~EMMC_AUTOGAPSTOP;
    }
}

/*********************************************************************
 * @fn      SDMMC_GetGAPStop_Status
 *
 * @brief   Get the EMMC_GAP_STOP bit status.
 *
 * @param   none.
 *
 * @return  FlagStatus - SET or RESET.
 */
FlagStatus SDMMC_GetStatus_GAPStop(void)
{
    FlagStatus bitstatus = RESET;

    if((SDMMC->TRAN_MODE & EMMC_GAP_STOP) != (uint32_t)RESET)
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
 * @fn      SDMMC_SetGAPStop
 *
 * @brief   Set the RB_EMMC_GAP_STOP bit and stop SDCK output after
 *          1 data block transfered.
 *
 * @param   none.
 *
 * @return  none.
 */
void SDMMC_SetGAPStop(void)
{
    SDMMC->TRAN_MODE |= EMMC_GAP_STOP;
}

/*********************************************************************
 * @fn      SDMMC_ClearGAPStop
 *
 * @brief   Clear the RB_EMMC_GAP_STOP bit and re-start SDCK output.
 *
 * @param   none.
 *
 * @return  none.
 */
void SDMMC_ClearGAPStop(void)
{
    SDMMC->TRAN_MODE &= ~EMMC_GAP_STOP;
}

/*********************************************************************
 * @fn      SDMMC_SetDMAAddr1
 *
 * @brief   Set the DMA start address 1 of SDMMC controller.
 *
 * @param   Address - start address 1 of DMA.
 *
 * @return  none.
 */
void SDMMC_SetDMAAddr1(uint32_t Address)
{
    SDMMC->DMA_BEG1 = Address;
}

/*********************************************************************
 * @fn      SDMMC_SetDMAAddr2
 *
 * @brief   Set the DMA start address 2 of SDMMC controller.
 *
 * @param   Address - start address 2 of DMA.
 *            uint32_t value of address.
 *
 * @return  none.
 */
void SDMMC_SetDMAAddr2(uint32_t Address)
{
    SDMMC->DMA_BEG2 = Address;
}

/*********************************************************************
 * @fn      SDMMC_IOInputDelayDDRInit
 *
 * @brief   Initializes the SDMMC peripheral DDR mode GPIO input delay according to the specified
 *        parameters in the SDMMC_IOInputDelayDDRTypeDef.
 *
 * @param   SSDMMC_IOInputDelayDDRStruct - pointer to a SSDMMC_IOInputDelayDDRStruct structure
 *        that contains the configuration information for the SDMMC peripheral.
 *
 * @return  none
 */
void SDMMC_IOInputDelayDDRInit(SDMMC_IOInputDelayDDRTypeDef *SSDMMC_IOInputDelayDDRStruct)
{
    SDMMC->TUNE_CLK_CMD &= ~(EMMC_TUNNE_CMD_I | EMMC_TUNNE_CLK_I);
    SDMMC->TUNE_DATI = 0;

    SDMMC->TUNE_CLK_CMD |= ((uint32_t)SSDMMC_IOInputDelayDDRStruct->SDMMC_CLK_IN_Delay << 4) |
                        ((uint32_t)SSDMMC_IOInputDelayDDRStruct->SDMMC_CMD_IN_Delay << 20);
    SDMMC->TUNE_DATI |= ((uint32_t)SSDMMC_IOInputDelayDDRStruct->SDMMC_DATA0_IN_Delay) |
                     ((uint32_t)SSDMMC_IOInputDelayDDRStruct->SDMMC_DATA1_IN_Delay << 4) |
                     ((uint32_t)SSDMMC_IOInputDelayDDRStruct->SDMMC_DATA2_IN_Delay << 8) |
                     ((uint32_t)SSDMMC_IOInputDelayDDRStruct->SDMMC_DATA3_IN_Delay << 12) |
                     ((uint32_t)SSDMMC_IOInputDelayDDRStruct->SDMMC_DATA4_IN_Delay << 16) |
                     ((uint32_t)SSDMMC_IOInputDelayDDRStruct->SDMMC_DATA5_IN_Delay << 20) |
                     ((uint32_t)SSDMMC_IOInputDelayDDRStruct->SDMMC_DATA6_IN_Delay << 24) |
                     ((uint32_t)SSDMMC_IOInputDelayDDRStruct->SDMMC_DATA7_IN_Delay << 28);
}

/*********************************************************************
 * @fn      SDMMC_IOOutputDelayDDRInit
 *
 * @brief   Initializes the SDMMC peripheral DDR mode GPIO output delay according to the specified
 *        parameters in the SDMMC_IOInputDelayDDRTypeDef.
 *
 * @param   SSDMMC_IOOutputDelayDDRStruct - pointer to a SSDMMC_IOOutputDelayDDRStruct structure
 *        that contains the configuration information for the SDMMC peripheral.
 *
 * @return  none
 */
void SDMMC_IOOutputDelayDDRInit(SDMMC_IOOutputDelayDDRTypeDef *SSDMMC_IOOutputDelayDDRStruct)
{
    SDMMC->TUNE_CLK_CMD &= ~(EMMC_TUNNE_CMD_O | EMMC_TUNNE_CLK_O);
    SDMMC->TUNE_DATO = 0;

    SDMMC->TUNE_CLK_CMD |= ((uint32_t)SSDMMC_IOOutputDelayDDRStruct->SDMMC_CLK_OUT_Delay) |
                        ((uint32_t)SSDMMC_IOOutputDelayDDRStruct->SDMMC_CMD_OUT_Delay << 16);
    SDMMC->TUNE_DATO |= ((uint32_t)SSDMMC_IOOutputDelayDDRStruct->SDMMC_DATA0_OUT_Delay) |
                     ((uint32_t)SSDMMC_IOOutputDelayDDRStruct->SDMMC_DATA1_OUT_Delay << 4) |
                     ((uint32_t)SSDMMC_IOOutputDelayDDRStruct->SDMMC_DATA2_OUT_Delay << 8) |
                     ((uint32_t)SSDMMC_IOOutputDelayDDRStruct->SDMMC_DATA3_OUT_Delay << 12) |
                     ((uint32_t)SSDMMC_IOOutputDelayDDRStruct->SDMMC_DATA4_OUT_Delay << 16) |
                     ((uint32_t)SSDMMC_IOOutputDelayDDRStruct->SDMMC_DATA5_OUT_Delay << 20) |
                     ((uint32_t)SSDMMC_IOOutputDelayDDRStruct->SDMMC_DATA6_OUT_Delay << 24) |
                     ((uint32_t)SSDMMC_IOOutputDelayDDRStruct->SDMMC_DATA7_OUT_Delay << 28);
}
