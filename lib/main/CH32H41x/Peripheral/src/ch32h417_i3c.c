/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32h417_i3c.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the RNG firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_i3c.h"
#include "ch32h417_rcc.h"

/*********************************************************************
 * @fn      I3C_DeInit
 *
 * @brief De-initializes the I3C peripheral registers to their default
 * 
 * @return None
 */
void I3C_DeInit(void)
{
    RCC_HB2PeriphResetCmd(RCC_HB2Periph_I3C, ENABLE);
    RCC_HB2PeriphResetCmd(RCC_HB2Periph_I3C, DISABLE);
}

/*********************************************************************
 * @fn      I3C_Ctrl_Init
 *
 * @brief   Initializes the I3C control bus registers according to the 
 *        specified parameters in the I3C_Ctrl_BusTypeDef structure.
 * 
 * @param   I3C_InitStruct - pointer to a I3C_Ctrl_BusTypeDef structure that
 *        contains the configuration information for the I3C control bus.
 *
 * @return  none
 */
void I3C_Ctrl_Init(I3C_Ctrl_BusTypeDef *I3C_InitStruct)
{
    uint32_t waveform_value = 0, timing_value = 0;

    I3C->CFGR = 0;

    I3C->CFGR |= I3C_CFGR_CRINIT;
    I3C->RESET &= ~I3C_RESET_HST_SIE_RST;

    waveform_value = ((uint32_t)I3C_InitStruct->SCLPPLowDuration |
                      ((uint32_t)I3C_InitStruct->SCLI3CHighDuration << 8) |
                      ((uint32_t)I3C_InitStruct->SCLODLowDuration << 16) |
                      ((uint32_t)I3C_InitStruct->SCLI2CHighDuration << 24));

    I3C->TIMINGR0 = waveform_value;

    timing_value = ((uint32_t)I3C_InitStruct->SDAHoldTime |
                    (uint32_t)I3C_InitStruct->WaitTime |
                    ((uint32_t)I3C_InitStruct->BusFreeDuration << 16) |
                    (uint32_t)I3C_InitStruct->BusIdleDuration);

    I3C->TIMINGR1 = timing_value;
}

/*********************************************************************
 * @fn      I3C_Tgt_Init
 *
 * @brief   Initializes the I3C target registers according to the specified 
 * 
 * @param   BusAvailableDuration - specifies the bus available duration 
 *        in number of kernel clock cycles.
 *
 * @return  none
 */
void I3C_Tgt_Init(uint8_t BusAvailableDuration)
{
    I3C->CFGR = 0;

    I3C->RESET &= ~I3C_RESET_TGT_SIE_RST;
    I3C->TIMINGR1 &= I3C_TIMINGR1_AVAL;
    I3C->TIMINGR1 |= (uint32_t)BusAvailableDuration;
}

/*********************************************************************
 * @fn      I3C_Ctrl_Config
 *
 * @brief   Sets the I3C control bus configuration according to the specified 
 *        parameters in the I3C_CtrlConfTypeDef structure.
 * 
 * @param   pConfig - pointer to a I3C_CtrlConfTypeDef structure that contains
 *        the configuration information for the I3C control bus.
 *
 * @return  none
 */
void I3C_Ctrl_Config(I3C_CtrlConfTypeDef *pConfig)
{
    uint32_t timing2_value = 0, cfgr_value = 0;

    timing2_value = (((uint32_t)pConfig->StallTime << 8) |
                     ((uint32_t)pConfig->ACKStallState << 3) |
                     ((uint32_t)pConfig->CCCStallState << 2) |
                     ((uint32_t)pConfig->TxStallState << 1) |
                     ((uint32_t)pConfig->RxStallState << 0));

    I3C->TIMINGR2 = timing2_value;

    cfgr_value = ((uint32_t)pConfig->HotJoinAllowed_EN << 7);

    I3C->CFGR |= cfgr_value;

    I3C->DEVR0 &= ~I3C_DEVR0_DA;

    I3C->DEVR0 |= (uint32_t)(pConfig->DynamicAddr << 1);

    I3C->DEVR0 |= I3C_DEVR0_DAVAL;
}

/*********************************************************************
 * @fn      I3C_Tgt_Config
 *
 * @brief   Sets the I3C target configuration according to the specified 
 *        parameters in the I3C_TgtConfTypeDef structure.
 * 
 * @param   pConfig - pointer to a I3C_TgtConfTypeDef structure that contains
 *        the configuration information for the I3C target.
 *
 * @return  none
 */
void I3C_Tgt_Config(I3C_TgtConfTypeDef *pConfig)
{
    uint32_t value1, value2, value3, value4, value5;

    value1 = (pConfig->HandOffActivityState | pConfig->MaxDataSpeed | pConfig->DataTurnAroundDuration |
              ((uint32_t)pConfig->MaxReadTurnAround << 16));

    I3C->GETMDSR = value1;

    value2 = (pConfig->IBIPayloadSize | (pConfig->MaxReadDataSize & I3C_MAXRLR_MRL));

    I3C->MAXRLR = value2;

    I3C->MAXWLR = pConfig->MaxWriteDataSize;

    I3C->EPIDR = (uint32_t)pConfig->MIPIIdentifier << 12;

    I3C->DCR = (uint32_t)pConfig->Identifier;

    value3 = (((uint32_t)pConfig->HandOffDelay << 3) |
              ((uint32_t)pConfig->GroupAddrCapability << 9));

    I3C->CRCAPR = value3;

    I3C->GETCAPR |= ((uint32_t)pConfig->PendingReadMDB) << 14;

    value4 = (((uint32_t)pConfig->MaxSpeedLimitation << 0) |
              ((uint32_t)pConfig->IBIPayload_EN << 2) |
              ((uint32_t)pConfig->CtrlCapability << 6));

    I3C->BCR = value4;

    value5 = (((uint32_t)pConfig->IBIRequest << 16) |
              ((uint32_t)pConfig->CtrlRoleRequest << 17) |
              ((uint32_t)pConfig->HotJoinRequest << 19));

    I3C->DEVR0 |= value5;
}

/*********************************************************************
 * @fn      I3C_Cmd
 *
 * @brief   Enables or disables the I3C interface.
 *
 * @param   NewState - new state of the DFSDM interface(ENABLE or DISABLE).
 *
 * @return  none
 */
void I3C_Cmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        I3C->CFGR |= I3C_CFGR_EN;
    }
    else
    {
        I3C->CFGR &= ~I3C_CFGR_EN;
    }
}

/*********************************************************************
 * @fn      I3C_ArbitrationHeaderCmd
 *
 * @brief   Enables or disables An arbitration header (7'h7E) is sent after 
 *        Start in case of legacy I2C or I3C private transfers.
 *
 * @param   NewState - new state of the DFSDM interface(ENABLE or DISABLE).
 *
 * @return  none
 */
void I3C_ArbitrationHeaderCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        I3C->CFGR |= I3C_CFGR_NOARBH;
    }
    else
    {
        I3C->CFGR &= ~I3C_CFGR_NOARBH;
    }
}

/*********************************************************************
 * @fn      I3C_HJAckCmd
 *
 * @brief   Enables or disables Hot Join Request is Acked. Current frame 
 *        on the bus is continued An Hot Join interrupt is sent through HJF flag..       
 *
 * @param   NewState - new state of the DFSDM interface(ENABLE or DISABLE).
 *
 * @return  none
 */
void I3C_HJAckCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        I3C->CFGR |= I3C_CFGR_HJACK;
    }
    else
    {
        I3C->CFGR &= ~I3C_CFGR_HJACK;
    }
}

/*********************************************************************
 * @fn      I3C_DMAReq_RXCmd
 *
 * @brief   Enables or disables DMA FIFO reception requests.      
 *
 * @param   NewState - new state of the DFSDM interface(ENABLE or DISABLE).
 *
 * @return  none
 */
void I3C_DMAReq_RXCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        I3C->CFGR |= I3C_CFGR_RXDMAEN;
    }
    else
    {
        I3C->CFGR &= ~I3C_CFGR_RXDMAEN;
    }
}

/*********************************************************************
 * @fn      I3C_DMAReq_TXCmd
 *
 * @brief   Enables or disables DMA FIFO transmission requests.      
 *
 * @param   NewState - new state of the DFSDM interface(ENABLE or DISABLE).
 *
 * @return  none
 */
void I3C_DMAReq_TXCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        I3C->CFGR |= I3C_CFGR_TXMAEN;
    }
    else
    {
        I3C->CFGR &= ~I3C_CFGR_TXMAEN;
    }
}

/*********************************************************************
 * @fn      I3C_DMAReq_StatusCmd
 *
 * @brief   Enables or disables DMA FIFO Status requests.      
 *
 * @param   NewState - new state of the DFSDM interface(ENABLE or DISABLE).
 *
 * @return  none
 */
void I3C_DMAReq_StatusCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        I3C->CFGR |= I3C_CFGR_SDMAEN;
    }
    else
    {
        I3C->CFGR &= ~I3C_CFGR_SDMAEN;
    }
}

/*********************************************************************
 * @fn      I3C_DMAReq_ControlCmd
 *
 * @brief   Enables or disables DMA FIFO Control word transfer requests.     
 *
 * @param   NewState - new state of the DFSDM interface(ENABLE or DISABLE).
 *
 * @return  none
 */
void I3C_DMAReq_ControlCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        I3C->CFGR |= I3C_CFGR_CDMAEN;
    }
    else
    {
        I3C->CFGR &= ~I3C_CFGR_CDMAEN;
    }
}

/*********************************************************************
 * @fn      I3C_ExitPatternCmd
 *
 * @brief   Enables or disables An Exit Pattern is sent after header
 *        (MTYPE = header) to program an escalation fault.
 *
 * @param   NewState - new state of the DFSDM interface(ENABLE or DISABLE).
 *
 * @return  none
 */
void I3C_ExitPatternCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        I3C->CFGR |= I3C_CFGR_EXITPTRN;
    }
    else
    {
        I3C->CFGR &= ~I3C_CFGR_EXITPTRN;
    }
}

/*********************************************************************
 * @fn      I3C_RequestTransfer
 *
 * @brief   Request a Transfer start.
 *
 * @param   none
 *
 * @return  none
 */
void I3C_RequestTransfer(void)
{
    I3C->CFGR |= I3C_CFGR_TSFSET;
}

/*********************************************************************
 * @fn      I3C_TxPreloadConfig
 *
 * @brief   Set TX FIFO preload (target mode).
 * 
 * @param   TxDataCount - Target mode Tx FIFO prdload.
 *            This parameter must be a number  from 0 to 0xFFFF.
 *
 * @return  none
 */
void I3C_TxPreloadConfig(uint16_t TxDataCount)
{
    I3C->TGTTDR &= ~(I3C_TGTTDR_TGTTDCNT | I3C_TGTTDR_PRELOAD);
    I3C->TGTTDR |= ((uint32_t)TxDataCount) | I3C_TGTTDR_PRELOAD;
}

/*********************************************************************
 * @fn      I3C_TARGET_ResetCmd
 *
 * @brief   Enables or disables Target mode reset.
 *
 * @param   NewState - new state of the DFSDM interface(ENABLE or DISABLE).
 *
 * @return  none
 */
void I3C_TARGET_ResetCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        I3C->RESET |= I3C_RESET_TGT_SIE_RST;
    }
    else
    {
        I3C->RESET &= ~I3C_RESET_TGT_SIE_RST;
    }
}

/*********************************************************************
 * @fn      I3C_CONTROLLER_ResetCmd
 *
 * @brief   Enables or disables Controller mode reset.
 *
 * @param   NewState - new state of the DFSDM interface(ENABLE or DISABLE).
 *
 * @return  none
 */
void I3C_CONTROLLER_ResetCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        I3C->RESET |= I3C_RESET_HST_SIE_RST;
    }
    else
    {
        I3C->RESET &= ~I3C_RESET_HST_SIE_RST;
    }
}

/*********************************************************************
 * @fn      I3C_SetModeConfig
 *
 * @brief   Configure peripheral mode.
 * 
 * @param   PeripheralMode - peropheral mode.
 *            PeripheralMode_CONTROLLER - Controller mode.
 *            PeripheralMode_TARGET - Target mode.
 *
 * @return  none
 */
void I3C_SetModeConfig(uint32_t PeripheralMode)
{
    I3C->CFGR &= ~I3C_CFGR_CRINIT;
    I3C->CFGR |= PeripheralMode;
}

/*********************************************************************
 * @fn      I3C_Ctrl_ConfigBusDevices
 *
 * @brief   Sets the I3C control bus configuration for the specified device.
 * 
 * @param   pDesc - pointer to a I3C_DeviceConfTypeDef structure that contains
 *        the configuration information for the I3C device.
 * 
 * @return none
 */
void I3C_Ctrl_ConfigBusDevices(I3C_DeviceConfTypeDef *pDesc)
{
    uint32_t value = 0;
    value = (((uint32_t)pDesc->TargetDynamicAddr << 1) |
             ((uint32_t)pDesc->IBIAck << 16) |
             ((uint32_t)pDesc->CtrlRoleReqAck << 17) |
             ((uint32_t)pDesc->CtrlStopTransfer << 19) |
             ((uint32_t)pDesc->IBIPayload << 18));

    if (pDesc->DeviceIndex == I3C_DeviceIndex_1)
    {
        I3C->DEVR1 = value;
    }
    else if (pDesc->DeviceIndex == I3C_DeviceIndex_2)
    {
        I3C->DEVR2 = value;
    }
    else if (pDesc->DeviceIndex == I3C_DeviceIndex_3)
    {
        I3C->DEVR3 = value;
    }
    else if (pDesc->DeviceIndex == I3C_DeviceIndex_4)
    {
        I3C->DEVR4 = value;
    }
}

/*********************************************************************
 * @fn      I3C_Ctrl_SetConfigResetPattern
 *
 * @brief   Sets the I3C control bus configuration reset pattern according to the specified 
 * 
 * @param   resetPattern - specifies the reset pattern.
 * 
 * @return  none
 */
void I3C_Ctrl_SetConfigResetPattern(FunctionalState resetPattern)
{
    if (resetPattern == ENABLE)
    {
        I3C->CFGR |= I3C_CFGR_RSTPTRN;
    }
    else
    {
        I3C->CFGR &= ~I3C_CFGR_RSTPTRN;
    }
}

/*********************************************************************
 * @fn      I3C_Ctrl_GetConfigResetPattern
 *
 * @brief   Gets the I3C control bus configuration reset pattern.
 * 
 * @return  The current reset pattern configuration. 
 */
FunctionalState I3C_Ctrl_GetConfigResetPattern(void)
{
    FunctionalState status = ENABLE;
    if ((I3C->CFGR & I3C_CFGR_RSTPTRN) == I3C_CFGR_RSTPTRN)
    {
        status = ENABLE;
    }
    else
    {
        status = DISABLE;
    }

    return status;
}

/*********************************************************************
 * @fn      I3C_FlushAllFifo
 *
 * @brief   Flushes all the I3C FIFOs.
 * 
 * @return  none
 */
void I3C_FlushAllFifo(void)
{
    I3C->CFGR |= (I3C_CFGR_TXFLUSH | I3C_CFGR_RXFLUSH);

    if (I3C->CFGR & I3C_CFGR_CRINIT)
    {
        I3C->CFGR |= (I3C_CFGR_SFLUSH | I3C_CFGR_CFLUSH);
    }
}

/*********************************************************************
 * @fn      I3C_FlushControlFifo
 *
 * @brief   Flushes the I3C control FIFO.
 * 
 * @return  none
 */
void I3C_FlushControlFifo(void)
{
    I3C->CFGR |= I3C_CFGR_CFLUSH;
}

/*********************************************************************
 * @fn      I3C_FlushStatusFifo
 *
 * @brief   Flushes the I3C status FIFO.
 * 
 * @return  none
 */
void I3C_FlushStatusFifo(void)
{
    I3C->CFGR |= I3C_CFGR_SFLUSH;
}

/*********************************************************************
 * @fn      I3C_SetConfigFifo
 *
 * @brief   Set I3C FIFOs configuration.
 * 
 * @param   pConfig The pointer to the I3C_FifoConfTypeDef structure 
 *        that contains the configuration information for the I3C FIFOs.
 * 
 * @return  none
 */
void I3C_SetConfigFifo(I3C_FifoConfTypeDef *pConfig)
{
    uint32_t value, mask;

    value = (pConfig->TxFifoThreshold | pConfig->RxFifoThreshold);

    mask = (I3C_CFGR_TXTHRES | I3C_CFGR_RXTHRES);

    if (I3C->CFGR & I3C_CFGR_CRINIT)
    {
        value |= (pConfig->StatusFifo | pConfig->ControlFifo);
        mask |= (I3C_CFGR_TMODE | I3C_CFGR_SMODE);
    }

    I3C->CFGR &= ~mask;
    I3C->CFGR |= value;
}

/*********************************************************************
 * @fn      I3C_ClearConfigFifo
 *
 * @brief   Clear I3C FIFOs configuration.
 * 
 * @return  none
 */
void I3C_ClearConfigFifo(void)
{
    uint32_t mask;

    mask = (I3C_CFGR_TXTHRES | I3C_CFGR_RXTHRES);

    if (I3C->CFGR & I3C_CFGR_CRINIT)
    {
        mask |= (I3C_CFGR_TMODE | I3C_CFGR_SMODE);
    }

    I3C->CFGR &= ~mask;
}

/*********************************************************************
 * @fn      I3C_ReadByte
 *
 * @brief   Reads a byte from the I3C bus.
 * 
 * @return  The read byte.
 */
uint8_t I3C_ReadByte(void)
{
    return I3C->RDBR;
}

/*********************************************************************
 * @fn      I3C_ReadWord
 *
 * @brief   Reads a word from the I3C bus.
 * 
 * @return  The read word.
 */
uint32_t I3C_ReadWord(void)
{
    return I3C->RDWR;
}

/*********************************************************************
 * @fn      I3C_WriteByte
 *
 * @brief   Writes a byte to the I3C bus.
 * 
 * @param   Byte - The byte to be written.
 * 
 * @return  none
 */
void I3C_WriteByte(uint8_t Byte)
{
    I3C->TDBR = (uint32_t)Byte;
}

/*********************************************************************
 * @fn      I3C_WriteWord
 *
 * @brief   Writes a word to the I3C bus.
 * 
 * @param   Word - The word to be written.
 * 
 * @return  none
 */
void I3C_WriteWord(uint32_t Word)
{
    I3C->TDWR = Word;
}

/*********************************************************************
 * @fn      I3C_IBIDataConfig
 *
 * @brief   Configure the I3C IBI data to be transmitted.
 * 
 * @param   Data - The data to be transmitted.
 * 
 * @return  none
 */
void I3C_IBIDataConfig(uint32_t Data)
{
    I3C->IBIDR = Data;
}

/*********************************************************************
 * @fn       I3C_GetCCCInfo
 *
 * @brief   Get CCC information for the I3C.
 * 
 * @param   notifyId - pointer to a I3C_Notify_ID_TypeDef enumeration.
 *          pCCCInfo - pointer to a I3C_CCCInfoTypeDef enumeration.
 * 
 * @return  none
 */
void  I3C_GetCCCInfo(I3C_Notify_ID_TypeDef notifyId, I3C_CCCInfoTypeDef *pCCCInfo)
{
    if((notifyId & EVENT_ID_DAU) == EVENT_ID_DAU)
    {
        pCCCInfo->DynamicAddrValid = I3C->DEVR0 & I3C_DEVR0_DAVAL;
        pCCCInfo->DynamicAddress = (I3C->DEVR0 & I3C_DEVR0_DA) >> 1;       
    }
    
    if((notifyId & EVENT_ID_SETMWL) == EVENT_ID_SETMWL)
    {
        pCCCInfo->MaxWriteLength = I3C->MAXWLR & I3C_MAXWLR_MWL;   
    }

    if((notifyId & EVENT_ID_SETMRL) == EVENT_ID_SETMRL)
    {
        pCCCInfo->MaxReadLength = I3C->MAXRLR & I3C_MAXRLR_MRL;     
    }

    if((notifyId & EVENT_ID_RSTACT) == EVENT_ID_RSTACT)
    {
        pCCCInfo->ResetAction = I3C->DEVR0 & I3C_DEVR0_RSTACT; 
    }

    if((notifyId & EVENT_ID_ENTASx) == EVENT_ID_ENTASx)
    {
        pCCCInfo->ActivityState = I3C->DEVR0 & I3C_DEVR0_AS;  
    }

    if((notifyId & EVENT_ID_ENEC_DISEC) == EVENT_ID_ENEC_DISEC)
    {
        pCCCInfo->HotJoinAllowed = (I3C->DEVR0 & I3C_DEVR0_HJEN) >> 19; 
        pCCCInfo->InBandAllowed = (I3C->DEVR0 & I3C_DEVR0_IBIEN) >> 16; 
        pCCCInfo->CtrlRoleAllowed = (I3C->DEVR0 & I3C_DEVR0_CREN) >> 17;   
    }

    if((notifyId & EVENT_ID_IBI) == EVENT_ID_IBI)
    {
        pCCCInfo->IBICRTgtAddr = (I3C->RMR & I3C_RMR_RADD) >> 17; 
        pCCCInfo->IBITgtNbPayload = I3C->RMR & I3C_RMR_IBIRDCNT;
        pCCCInfo->IBITgtPayload = I3C->IBIDR;   
    }

    if((notifyId & EVENT_ID_CR) == EVENT_ID_CR)
    {
        pCCCInfo->IBICRTgtAddr = (I3C->RMR & I3C_RMR_RADD) >> 17;   
    }
}

/*********************************************************************
 * @fn      I3C_ControllerHandleMessage
 *
 * @brief   I3C Message content on the I3C Bus as Controller.
 * 
 * @param   TargetAddr - Specifies the target address to be programmed.
 *            This parameter must be a number from 0 to 0xFF.
 *          TransferSize Specifies the number of bytes to be programmed.
 *            This parameter must be a number from 0 to 0xFFFF. 
 *          Direction - Write or read message.
 *            I3C_Direction_RD - Read message.
 *            I3C_Direction_WR - Write message.
 *          MessageType - Message type.
 *            I3C_CONTROLLER_MessageType0 - Host message type 0.
 *            I3C_CONTROLLER_MessageType1 - Host message type 1.
 *            I3C_CONTROLLER_MessageType2 - Host message type 2.
 *            I3C_CONTROLLER_MessageType3 - Host message type 3.   
 *            I3C_CONTROLLER_MessageType4 - Host message type 4.  
 *          EndMode - Message end mode.
 *            I3C_EndMode_0 - Message end mode 0.
 *            I3C_EndMode_1 - Message end mode 1.
 *
 * @return  none
 */
void I3C_ControllerHandleMessage(uint32_t TargetAddr, uint32_t TransferSize,
                                 uint32_t Direction, uint32_t MessageType, uint32_t EndMode)
{
    I3C->CTLR = ((uint32_t)(TargetAddr << 17) | (uint32_t)(TransferSize) | Direction | MessageType | EndMode);
}

/*********************************************************************
 * @fn      I3C_ControllerHandleCCC
 *
 * @brief   I3C Common Command Code content on the I3C Bus as Controller.
 * 
 * @param   CCCValue - Specifies the Command Code to be programmed.
 *            This parameter must be a number from 0 to 0xFF.
 *          AddByteSize - Specifies the number of CCC additional bytes to be programmed.
 *            This parameter must be a number from 0 to 0xFFFF. 
 *          EndMode - Message end mode.
 *            I3C_EndMode_0 - Message end mode 0.
 *            I3C_EndMode_1 - Message end mode 1.
 *
 * @return  none
 */
void I3C_ControllerHandleCCC(uint32_t CCCValue, uint32_t AddByteSize, uint32_t EndMode)
{
    I3C->CTLR = CCCValue | AddByteSize | EndMode;
}

/*********************************************************************
 * @fn      I3C_TargetHandleMessage
 *
 * @brief   I3C Message content on the I3C Bus as Target.
 * 
 * @param   IBISize - Specifies the number of IBI bytes.
 *            This parameter must be a number from 0 to 0xFFFF. 
 *          MessageType - Message end mode.
 *            I3C_TARGET_MessageType0 - Target message type 0.
 *            I3C_TARGET_MessageType1 - Target message type 1.
 *            I3C_TARGET_MessageType2 - Target message type 2.
 *
 * @return  none
 */
void I3C_TargetHandleMessage(uint32_t MessageType, uint32_t IBISize)
{
    I3C->CTLR = MessageType | IBISize;
}

/*********************************************************************
 * @fn      I3C_GetMessageDirection
 *
 * @brief   Get Message identifier.
 * 
 * @return  0 - Write. 
 *          1 - Read.
 */
uint8_t I3C_GetMessageDirection(void)
{
    return ((uint8_t)((I3C->STATR & I3C_STATR_DIR) >> 18));
}

/*********************************************************************
 * @fn      I3C_GetTargetAbortPrivateRead
 *
 * @brief   Indicates if a Target abort a private read command.
 * 
 * @return  0 - Target no end reading message early. 
 *          1 - Target end reading message early. .
 */
uint8_t I3C_GetTargetAbortPrivateRead(void)
{
    return ((uint8_t)((I3C->STATR & I3C_STATR_ABT) >> 17));
}

/*********************************************************************
 * @fn      I3C_GetGetXferDataCount
 *
 * @brief   Get the number of data during a Transfer.
 * 
 * @return  The number of data during a Transfer.
 */
uint16_t I3C_GetGetXferDataCount(void)
{
    return ((uint16_t)(I3C->STATR));
}

/*********************************************************************
 * @fn      I3C_GetMessageIdentifier
 *
 * @brief   Get Message identifier.
 * 
 * @return  Message identifier.
 */
uint8_t I3C_GetMessageIdentifier(void)
{
    return ((uint8_t)((I3C->STATR & I3C_STATR_MID) >> 24));
}

/*********************************************************************
 * @fn      I3C_GetGetIBITargetAddr
 *
 * @brief   Get the target address received during accepted IBI or Controller-role request.
 * 
 * @return  The target address received during accepted IBI or Controller-role request.
 */
uint8_t I3C_GetGetIBITargetAddr(void)
{
    return ((uint8_t)((I3C->RMR & I3C_RMR_RADD) >> 17));
}

/*********************************************************************
 * @fn      I3C_GetReceiveCommandCode
 *
 * @brief   Get CCC code of received command.
 * 
 * @return  CCC code of received command.
 */
uint8_t I3C_GetReceiveCommandCode(void)
{
    return ((uint8_t)((I3C->RMR & I3C_RMR_RCODE) >> 8));
}

/*********************************************************************
 * @fn      I3C_GetGetNbIBIAddData
 *
 * @brief   Get the number of data bytes received when reading IBI data (controller mode).
 * 
 * @return  The number of data bytes received when reading IBI data
 */
uint8_t I3C_GetGetNbIBIAddData(void)
{
    return ((uint8_t)(I3C->RMR & I3C_RMR_IBIRDCNT));
}

/*********************************************************************
 * @fn      I3C_GetGetResetAction
 *
 * @brief   Get Reset Action (Target only).
 * 
 * @return  0 - reset action none.
 *          1 - reset action partial.
 *          2 - reset action full.
 */
uint8_t I3C_GetGetResetAction(void)
{
    return ((uint8_t)((I3C->DEVR0 & 0x00C00000) >> 22));
}

/*********************************************************************
 * @fn      I3C_GetAllowedPayloadUpdate
 *
 * @brief   Indicates if update of the Device Characteristics Register 
 *        is Allowed or Not Allowed.
 * @param   DeviceIndex - Specifies the index value of the device in the DEVRx register.
 *            I3C_DeviceIndex_1 - Device Index 1.
 *            I3C_DeviceIndex_2 - Device Index 2.
 *            I3C_DeviceIndex_3 - Device Index 3.
 *            I3C_DeviceIndex_4 - Device Index 4.
 * 
 * @return  NewState - ENABLE or DISABLE
 */
FunctionalState I3C_GetAllowedPayloadUpdate(uint32_t DeviceIndex)
{
    FunctionalState NewState = DISABLE;

    if(DeviceIndex == I3C_DeviceIndex_1)
    {
        if(I3C->DEVR1 & I3C_DEVR1_DIS)
        {
            NewState = ENABLE;
        } 
    }
    else if(DeviceIndex == I3C_DeviceIndex_2)
    {
        if(I3C->DEVR2 & I3C_DEVR2_DIS)
        {
            NewState = ENABLE;
        } 
    }
    else if(DeviceIndex == I3C_DeviceIndex_3)
    {
        if(I3C->DEVR3 & I3C_DEVR3_DIS)
        {
            NewState = ENABLE;
        } 
    }
    else if(DeviceIndex == I3C_DeviceIndex_4)
    {
        if(I3C->DEVR4 & I3C_DEVR4_DIS)
        {
            NewState = ENABLE;
        } 
    }

    return NewState;
}

/*********************************************************************
 * @fn      I3C_GetMIPIInstanceID
 *
 * @brief   Get the MIPI Instance ID.
 * 
 * @return  The MIPI Instance ID.
 */
uint8_t I3C_GetMIPIInstanceID(void)
{
    return ((uint8_t)((I3C->EPIDR & I3C_EPIDR_MIPIID) >> 12));
}

/*********************************************************************
 * @fn      I3C_GetIDTypeSelector
 *
 * @brief   Get the ID type selector.
 * 
 * @return  The ID type selector.
 */
uint8_t I3C_GetIDTypeSelector(void)
{
    return ((uint8_t)((I3C->EPIDR & I3C_EPIDR_IDTSEL) >> 16));
}

/*********************************************************************
 * @fn      I3C_GetMIPIManufacturerID
 *
 * @brief   Get the MIPI Manufacturer ID.
 * 
 * @return  The MIPI Manufacturer ID.
 */
uint16_t I3C_GetMIPIManufacturerID(void)
{
    return ((uint16_t)((I3C->EPIDR & I3C_EPIDR_MIPIMID) >> 17));
}

/*********************************************************************
 * @fn      I3C_ITConfig
 *
 * @brief   Enables or disables the specified I3C interrupt.
 * 
 * @param   I3C_IT - The interrupt to be enabled or disabled.
 *            I3C_IT_GRPIE - DEFGRPA CCC and RX FIFO interrupt.
 *            I3C_IT_DEFIE - DEFTGTS CCC and RX FIFO interrupt.
 *            I3C_IT_INTUPDIE - ENEC/DISEC CCC interrupt.
 *            I3C_IT_ASUPDIE - ENTASx CCC interrupt.
 *            I3C_IT_RSTIE - reset pattern interrupt.
 *            I3C_IT_MRLUPDIE - SETMRL CCC interrupt.
 *            I3C_IT_MWLUPDIE - SETMWL CCC interrupt.
 *            I3C_IT_DAUPDIE - ENTDAA/RSTDAA/SETNEWDA CCC interrupt.
 *            I3C_IT_STAIE - GETSTATUS CCC interrupt.
 *            I3C_IT_GETIE - GETxxx CCC interrupt.
 *            I3C_IT_WKPIE - wakeup interrupt.
 *            I3C_IT_HJIE - Hot-join interrupt.
 *            I3C_IT_CRUPDIE - controller-role update interrupt.
 *            I3C_IT_CRIE - Controller-role request interrupt.
 *            I3C_IT_IBIENDIE - IBI end interrupt.
 *            I3C_IT_IBIIE - IBI request interrupt.
 *            I3C_IT_ERRIE - error interrupt.
 *            I3C_IT_RXTGTENDIE - Target-initiated read end interrupt.
 *            I3C_IT_FCIE - Frame end interrupt.
 *            I3C_IT_RXFNEIE - RX-FIFO not empty interrupt.
 *            I3C_IT_TXFNEIE - TX-FIFO not full interrupt.
 *            I3C_IT_CFNFIE - Control FIFO not full interrupt.   
 *            I3C_IT_SFNEIE - Status FIFO not empty interrupt.        
 *          NewState - The new state of the interrupt.
 * 
 * @return  none
 */
void I3C_ITConfig(uint32_t I3C_IT, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        I3C->INTENR |= I3C_IT;
    }
    else
    {
        I3C->INTENR &= (uint32_t)~I3C_IT;
    }
}

/*********************************************************************
 * @fn      I3C_GetITStatus
 *
 * @brief   Checks whether the specified I3C interrupt flag is set or not.
 * 
 * @param   I3C_IT - specifies the interrupt flag to check.
 *            I3C_IT_GRPIE - DEFGRPA CCC and RX FIFO interrupt.
 *            I3C_IT_DEFIE - DEFTGTS CCC and RX FIFO interrupt.
 *            I3C_IT_INTUPDIE - ENEC/DISEC CCC interrupt.
 *            I3C_IT_ASUPDIE - ENTASx CCC interrupt.
 *            I3C_IT_RSTIE - reset pattern interrupt.
 *            I3C_IT_MRLUPDIE - SETMRL CCC interrupt.
 *            I3C_IT_MWLUPDIE - SETMWL CCC interrupt.
 *            I3C_IT_DAUPDIE - ENTDAA/RSTDAA/SETNEWDA CCC interrupt.
 *            I3C_IT_STAIE - GETSTATUS CCC interrupt.
 *            I3C_IT_GETIE - GETxxx CCC interrupt.
 *            I3C_IT_WKPIE - wakeup interrupt.
 *            I3C_IT_HJIE - Hot-join interrupt.
 *            I3C_IT_CRUPDIE - controller-role update interrupt.
 *            I3C_IT_CRIE - Controller-role request interrupt.
 *            I3C_IT_IBIENDIE - IBI end interrupt.
 *            I3C_IT_IBIIE - IBI request interrupt.
 *            I3C_IT_ERRIE - error interrupt.
 *            I3C_IT_RXTGTENDIE - Target-initiated read end interrupt.
 *            I3C_IT_FCIE - Frame end interrupt.
 *            I3C_IT_RXFNEIE - RX-FIFO not empty interrupt.
 *            I3C_IT_TXFNEIE - TX-FIFO not full interrupt.
 *            I3C_IT_CFNFIE - Control FIFO not full interrupt.   
 *            I3C_IT_SFNEIE - Status FIFO not empty interrupt.    
 * 
 * @return  ITStatus - SET or RESET.
 */
ITStatus I3C_GetITStatus(uint32_t I3C_IT)
{
    ITStatus bitstatus    = RESET;
    uint32_t enablestatus = I3C->INTENR;

    if (((I3C->EVR & I3C_IT) != (uint32_t)RESET) && enablestatus)
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
 * @fn      I3C_ClearITPendingBit
 *
 * @brief   Clears the specified I3C interrupt flag.
 * 
 * @param   I3C_IT - specifies the interrupt flag to clear.
 *            I3C_IT_GRPIE - DEFGRPA CCC and RX FIFO interrupt.
 *            I3C_IT_DEFIE - DEFTGTS CCC and RX FIFO interrupt.
 *            I3C_IT_INTUPDIE - ENEC/DISEC CCC interrupt.
 *            I3C_IT_ASUPDIE - ENTASx CCC interrupt.
 *            I3C_IT_RSTIE - reset pattern interrupt.
 *            I3C_IT_MRLUPDIE - SETMRL CCC interrupt.
 *            I3C_IT_MWLUPDIE - SETMWL CCC interrupt.
 *            I3C_IT_DAUPDIE - ENTDAA/RSTDAA/SETNEWDA CCC interrupt.
 *            I3C_IT_STAIE - GETSTATUS CCC interrupt.
 *            I3C_IT_GETIE - GETxxx CCC interrupt.
 *            I3C_IT_WKPIE - wakeup interrupt.
 *            I3C_IT_HJIE - Hot-join interrupt.
 *            I3C_IT_CRUPDIE - controller-role update interrupt.
 *            I3C_IT_CRIE - Controller-role request interrupt.
 *            I3C_IT_IBIENDIE - IBI end interrupt.
 *            I3C_IT_IBIIE - IBI request interrupt.
 *            I3C_IT_ERRIE - error interrupt.
 *            I3C_IT_RXTGTENDIE - Target-initiated read end interrupt.
 *            I3C_IT_FCIE - Frame end interrupt.  
 * 
 * @return  none
 */
void I3C_ClearITPendingBit(uint32_t I3C_IT)
{
    I3C->CEVR = I3C_IT;
}

/*********************************************************************
 * @fn      I3C_GetFlagStatus
 *
 * @brief   Checks whether the specified I3C flag is set or not.
 * 
 * @param   I3C_FLAG - specifies the flag to check.
 *            I3C_FLAG_CFEF - Control FIFO not empty flag 
 *            I3C_FLAG_TXFEF - Tx FIFO empty flag               
 *            I3C_FLAG_CFNFF - Control FIFO not full flag      
 *            I3C_FLAG_SFNEF - Status FIFO not empty flag      
 *            I3C_FLAG_TXFNFF - Tx FIFO not full flag            
 *            I3C_FLAG_RXFNEF - Rx FIFO not empty flag           
 *            I3C_FLAG_TXLASTF - Last written data byte/word flag 
 *            I3C_FLAG_RXLASTF - Last read data byte/word flag    
 *            I3C_FLAG_FCF - Frame complete flag              
 *            I3C_FLAG_RXTGTENDF - Target-initiated read end flag   
 *            I3C_FLAG_ERRF - Error flag                       
 *            I3C_FLAG_IBIF - IBI request flag                 
 *            I3C_FLAG_IBIENDF - IBI end flag                     
 *            I3C_FLAG_CRF - Controller-role request flag     
 *            I3C_FLAG_CRUPDF - Controller-role update flag      
 *            I3C_FLAG_HJF - Hot-join flag                    
 *            I3C_FLAG_WKPF - Wakeup flag                      
 *            I3C_FLAG_GETF - GETxxx CCC flag                  
 *            I3C_FLAG_STAF - Format 1 GETSTATUS CCC flag      
 *            I3C_FLAG_DAUPDF - ENTDAA/RSTDAA/SETNEWDA CCC flag  
 *            I3C_FLAG_MWLUPDF - SETMWL CCC flag                  
 *            I3C_FLAG_MRLUPDF - SETMRL CCC flag                  
 *            I3C_FLAG_RSTF - Reset pattern flag               
 *            I3C_FLAG_ASUPDF - ENTASx CCC flag                  
 *            I3C_FLAG_INTUPDF - ENEC/DISEC CCC flag              
 *            I3C_FLAG_DEFF - DEFTGTS CCC flag                 
 *            I3C_FLAG_GRPF - DEFGRPA CCC flag                  
 * 
 * @return  FlagStatus - SET or RESET.
 */
FlagStatus I3C_GetFlagStatus(uint32_t I3C_FLAG)
{
    FlagStatus bitstatus = RESET;

    if ((I3C->EVR & I3C_FLAG) != (uint32_t)RESET)
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
 * @fn      I3C_GetErrorStatus
 *
 * @brief   Checks whether the specified I3C error flag is set or not.
 * 
 * @param   I3C_Error - specifies the error flag to check.
 *            I3C_ERROR_CE0 - CE0 error
 *            I3C_ERROR_CE1 - CE1 error
 *            I3C_ERROR_CE2 - CE2 error
 *            I3C_ERROR_CE3 - CE3 error
 *            I3C_ERROR_TE0 - TE0 error
 *            I3C_ERROR_TE1 - TE1 error
 *            I3C_ERROR_TE2 - TE2 error
 *            I3C_ERROR_TE3 - TE3 error
 *            I3C_ERROR_TE4 - TE4 error
 *            I3C_ERROR_TE5 - TE5 error
 *            I3C_ERROR_TE6 - TE6 error
 *            I3C_ERROR_PERR - protocol error
 *            I3C_ERROR_STALL - SCL stall error
 *            I3C_ERROR_DOVR - Rx FIFO Over-Run or Tx FIFO Under-Run error
 *            I3C_ERROR_COVR - S FIFO Over-Run or C FIFO Under-Run error 
 *            I3C_ERROR_ADDRESS_NACK - address not acknowledged error 
 *            I3C_ERROR_DATA_NACK - data not acknowledged error
 *            I3C_ERROR_DATA_HAND_OFF - data error during controller-role hand-off process
 * @return   FlagStatus - SET or RESET.
 */
FlagStatus I3C_GetErrorStatus(uint32_t I3C_Error)
{
    FlagStatus bitstatus = RESET;

    if ((I3C->STATER & I3C_Error) != (uint32_t)RESET)
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
 * @fn      I3C_ClearFlag
 *
 * @brief   Clears the specified I3C flag.
 * 
 * @param   I3C_FLAG - specifies the flag to clear.  
 *            I3C_FLAG_FCF - Frame complete flag              
 *            I3C_FLAG_RXTGTENDF - Target-initiated read end flag   
 *            I3C_FLAG_ERRF - Error flag                       
 *            I3C_FLAG_IBIF - IBI request flag                 
 *            I3C_FLAG_IBIENDF - IBI end flag                     
 *            I3C_FLAG_CRF - Controller-role request flag     
 *            I3C_FLAG_CRUPDF - Controller-role update flag      
 *            I3C_FLAG_HJF - Hot-join flag                    
 *            I3C_FLAG_WKPF - Wakeup flag                      
 *            I3C_FLAG_GETF - GETxxx CCC flag                  
 *            I3C_FLAG_STAF - Format 1 GETSTATUS CCC flag      
 *            I3C_FLAG_DAUPDF - ENTDAA/RSTDAA/SETNEWDA CCC flag  
 *            I3C_FLAG_MWLUPDF - SETMWL CCC flag                  
 *            I3C_FLAG_MRLUPDF - SETMRL CCC flag                  
 *            I3C_FLAG_RSTF - Reset pattern flag               
 *            I3C_FLAG_ASUPDF - ENTASx CCC flag                  
 *            I3C_FLAG_INTUPDF - ENEC/DISEC CCC flag              
 *            I3C_FLAG_DEFF - DEFTGTS CCC flag                 
 *            I3C_FLAG_GRPF - DEFGRPA CCC flag   
 * 
 * @return  none
 */
void I3C_ClearFlag(uint32_t I3C_FLAG)
{
    I3C->CEVR = I3C_FLAG;
}
