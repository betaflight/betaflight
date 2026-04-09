/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32h417_qspi.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the QSPI firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_qspi.h"
#include "ch32h417_rcc.h"

#define QSPI_CR_CLEAR_MASK               0x00FFFFCF
#define QSPI_DCR_CLEAR_MASK              0xFFE0F7FE
#define QSPI_CCR_CLEAR_MASK              0x90800000
#define QSPI_PIR_CLEAR_MASK              0xFFFF0000
#define QSPI_LPTR_CLEAR_MASK             0xFFFF0000
#define QSPI_CCR_CLEAR_INSTRUCTION_MASK  0xFFFFFF00
#define QSPI_CCR_CLEAR_DCY_MASK          0xFFC3FFFF
#define QSPI_CR_CLEAR_FIFOTHRESHOLD_MASK 0xFFFFF0FF
#define QSPI_CR_INTERRUPT_MASK           0x001F0000
#define QSPI_SR_INTERRUPT_MASK           0x0000001F
#define QSPI_FSR_INTERRUPT_MASK          0x0000001B

/*********************************************************************
 * @fn      QSPI_DeInit
 *
 * @brief   Deinitializes the QSPI peripheral registers to their default 
 *          reset values.
 *
 * @param   QSPIx - where x can be  - 1, 2. 
 * 
 * @return  None
 */
void QSPI_DeInit(QSPI_TypeDef *QSPIx)
{
    if (QSPIx == QSPI1)
    {
        RCC_HB2PeriphResetCmd(RCC_HB1Periph_QSPI1, ENABLE);
        RCC_HB2PeriphResetCmd(RCC_HB1Periph_QSPI1, DISABLE);
    }
    else if (QSPIx == QSPI2)
    {
        RCC_HB2PeriphResetCmd(RCC_HB1Periph_QSPI2, ENABLE);
        RCC_HB2PeriphResetCmd(RCC_HB1Periph_QSPI2, DISABLE);
    }
}

/*********************************************************************
 * @fn      QSPI_ComConfig_StructInit
 *
 * @brief   Fills each QSPI_ComConfig_InitStruct member with its default value.
 *
 * @param   QSPIx - where x can be  - 1, 2.
 *          QSPI_ComConfig_InitStruct - pointer to a QSPI_ComConfig_InitTypeDef 
 *        structure which will be initialized.
 *
 * @return  none
 */
void QSPI_ComConfig_StructInit(QSPI_ComConfig_InitTypeDef *QSPI_ComConfig_InitStruct)
{

    QSPI_ComConfig_InitStruct->QSPI_ComConfig_SIOOMode = QSPI_ComConfig_SIOOMode_Disable;
    QSPI_ComConfig_InitStruct->QSPI_ComConfig_FMode = QSPI_ComConfig_FMode_Indirect_Write;
    QSPI_ComConfig_InitStruct->QSPI_ComConfig_DMode = QSPI_ComConfig_DMode_NoData;
    QSPI_ComConfig_InitStruct->QSPI_ComConfig_DummyCycles = 0;
    QSPI_ComConfig_InitStruct->QSPI_ComConfig_ABSize = QSPI_ComConfig_ABSize_8bit;
    QSPI_ComConfig_InitStruct->QSPI_ComConfig_ABMode = QSPI_ComConfig_ABMode_NoAlternateByte;
    QSPI_ComConfig_InitStruct->QSPI_ComConfig_ADSize = QSPI_ComConfig_ADSize_8bit;
    QSPI_ComConfig_InitStruct->QSPI_ComConfig_ADMode = QSPI_ComConfig_ADMode_NoAddress;
    QSPI_ComConfig_InitStruct->QSPI_ComConfig_IMode = QSPI_ComConfig_IMode_NoInstruction;
    QSPI_ComConfig_InitStruct->QSPI_ComConfig_Ins = 0;
}

/*********************************************************************
 * @fn      QSPI_Init
 *
 * @brief   Initializes the QSPI peripheral according to the specified 
 *        parameters in the QSPI_InitStruct.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          QSPI_InitStruct - pointer to a QSPI_InitTypeDef structure that
 *        contains the configuration information for the specified QSPI peripheral.
 * 
 * @return  none
 */
void QSPI_Init(QSPI_TypeDef *QSPIx, QSPI_InitTypeDef *QSPI_InitStruct)
{
    uint32_t tmpreg = 0;

    tmpreg = QSPIx->CR;

    tmpreg &= QSPI_CR_CLEAR_MASK;

    tmpreg |= (uint32_t)(((QSPI_InitStruct->QSPI_Prescaler) << 24) | (QSPI_InitStruct->QSPI_FSelect) | (QSPI_InitStruct->QSPI_DFlash));

    QSPIx->CR = tmpreg;

    tmpreg = QSPIx->DCR;

    tmpreg &= QSPI_DCR_CLEAR_MASK;

    tmpreg |= (uint32_t)(((QSPI_InitStruct->QSPI_FSize) << 16) | (QSPI_InitStruct->QSPI_CSHTime) | (QSPI_InitStruct->QSPI_CKMode));

    QSPIx->DCR = tmpreg;
}

/*********************************************************************
 * @fn      QSPI_ComConfig_Init
 * 
 * @brief   Initializes the QSPI CCR according to the specified 
 *        parameters in the QSPI_ComConfig_InitStruct.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          QSPI_ComConfig_InitStruct - pointer to a QSPI_ComConfig_InitTypeDef structure that
 *        contains the communication configuration informations about QSPI peripheral.
 * 
 * @return  none
 */
void QSPI_ComConfig_Init(QSPI_TypeDef *QSPIx, QSPI_ComConfig_InitTypeDef *QSPI_ComConfig_InitStruct)
{
    uint32_t tmpreg = 0;

    tmpreg = QSPIx->CCR;

    tmpreg &= QSPI_CCR_CLEAR_MASK;

    tmpreg |= (uint32_t)((QSPI_ComConfig_InitStruct->QSPI_ComConfig_FMode) | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_SIOOMode) | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_DMode) | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_ABSize) | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_ABMode) | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_ADSize) | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_ADMode) | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_IMode) | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_Ins) | ((QSPI_ComConfig_InitStruct->QSPI_ComConfig_DummyCycles) << 18));

    QSPIx->CCR = tmpreg;
}

/*********************************************************************
 * @fn      QSPI_Cmd
 * 
 * @brief   Enables or disables QSPI peripheral.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          NewState - ENABLE or DISABLE.
 * 
 * @return  none
 */
void QSPI_Cmd(QSPI_TypeDef *QSPIx, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        QSPIx->CR |= QSPI_CR_EN;
    }
    else
    {
        QSPIx->CR &= ~QSPI_CR_EN;
    }
}

/*********************************************************************
 * @fn      QSPI_AutoPollingMode_Config
 * 
 * @brief   Configure the QSPI Automatic Polling Mode.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          QSPI_Match - Value to be compared with the masked status register to get a match. 
 *        This parameter can be any value between 0x00000000 and 0xFFFFFFFF.
 *          QSPI_Mask - Mask to be applied to the status bytes received in polling mode.. 
 *        This parameter can be any value between 0x00000000 and 0xFFFFFFFF.
 *          QSPI_Match_Mode - indicates which method should be used for determining a match during
 *        automatic polling mode. 
 *          This parameter can be any value of :
 *            QSPI_PMM_AND - AND match mode- SMF is set if all the unmasked bits received from the flash match
 *                 the corresponding bits in the match register
 *            QSPI_PMM_OR - OR match mode- SMF is set if any one of the unmasked bits received from the flash
 *              matches its corresponding bit in the match register.
 * 
 * @return  none
 */
void QSPI_AutoPollingMode_Config(QSPI_TypeDef *QSPIx, uint32_t QSPI_Match, uint32_t QSPI_Mask, uint32_t QSPI_Match_Mode)
{

    QSPIx->PSMAR = QSPI_Match;

    QSPIx->PSMKR = QSPI_Mask;

    if (QSPI_Match_Mode)
    {
        QSPIx->CR |= QSPI_CR_PMM;
    }
    else
    {
        QSPIx->CR &= ~QSPI_CR_PMM;
    }
    
}

/*********************************************************************
 * @fn      QSPI_AutoPollingMode_SetInterval
 * 
 * @brief   Sets the number of CLK cycle between two read during automatic polling phases.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          QSPI_Interval - The number of CLK cycle between two read during automatic polling phases. 
 *        This parameter can be any value of between 0x0000 and 0xFFFF
 * 
 * @return  none
 */
void QSPI_AutoPollingMode_SetInterval(QSPI_TypeDef *QSPIx, uint32_t QSPI_Interval)
{
    uint32_t tmpreg = 0;

    tmpreg = QSPIx->PIR;

    tmpreg &= QSPI_PIR_CLEAR_MASK;

    tmpreg |= QSPI_Interval;

    QSPIx->PIR = tmpreg;
    
}

/*********************************************************************
 * @fn      QSPI_MemoryMappedMode_SetTimeout
 * 
 * @brief   Sets the value of the Timeout in Memory Mapped mode
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          QSPI_Timeout - This field indicates how many CLK cycles QSPI waits after the 
 *        FIFO becomes full until it raises nCS, putting the flash memory 
 *        in a lowerconsumption state. 
 *        This parameter can be any value of between 0x0000 and 0xFFFF
 * 
 * @return  none
 */
void QSPI_MemoryMappedMode_SetTimeout(QSPI_TypeDef *QSPIx, uint32_t QSPI_Timeout)
{
    uint32_t tmpreg = 0;

    tmpreg = QSPIx->LPTR;

    tmpreg &= QSPI_LPTR_CLEAR_MASK;

    tmpreg |= QSPI_Timeout;

    QSPIx->LPTR = tmpreg;
    
}

/*********************************************************************
 * @fn      QSPI_SetAddress
 * 
 * @brief   Sets the value of the Address
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          QSPI_Address - Address to be send to the external flash memory.  
 *        This parameter can be any value of between 0x00000000 and 0xFFFFFFFF
 * 
 * @return  none
 */
void QSPI_SetAddress(QSPI_TypeDef *QSPIx, uint32_t QSPI_Address)
{

    QSPIx->AR = QSPI_Address;
    
}

/*********************************************************************
 * @fn      QSPI_SetAlternateByte
 * 
 * @brief   Sets the value of the Alternate Bytes
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          QSPI_AlternateByte - Optional data to be send to the external QSPI device right after the address. 
 *        This parameter can be any value of between 0x00000000 and 0xFFFFFFFF
 * 
 * @return  none
 */
void QSPI_SetAlternateByte(QSPI_TypeDef *QSPIx, uint32_t QSPI_AlternateByte)
{

    QSPIx->ABR = QSPI_AlternateByte;
    
}

/*********************************************************************
 * @fn      QSPI_SetFIFOThreshold
 * 
 * @brief   Sets the FIFO Threshold
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          QSPI_FIFOThres - Defines, in indirect mode, the threshold number 
 *        of bytes in the FIFO which will cause the FIFO Threshold Flag 
 *           FTF to be set.
 *        This parameter can be any value of between 0x00 and 0x0F
 * 
 * @return  none
 */
void QSPI_SetFIFOThreshold(QSPI_TypeDef *QSPIx, uint32_t QSPI_FIFOThreshold)
{
    uint32_t tmpreg = 0;

    tmpreg = QSPIx->CR;

    tmpreg &= QSPI_CR_CLEAR_FIFOTHRESHOLD_MASK;

    tmpreg |= (QSPI_FIFOThreshold << 8);

    QSPIx->CR = tmpreg;
}

/*********************************************************************
 * @fn      QSPI_SetDataLength
 * 
 * @brief   Sets number of Bytes to be transferred 
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          QSPI_DataLength - Number of data to be retrieved (value) 
 *        in indirect and status-polling modes.
 * 
 * @return  none
 */
void QSPI_SetDataLength(QSPI_TypeDef *QSPIx, uint32_t QSPI_DataLength)
{

    QSPIx->DLR = QSPI_DataLength -1;
    
}

/*********************************************************************
 * @fn      QSPI_TimeoutCounterCmd
 * 
 * @brief   Enables or disables The Timeout Counter.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          NewState - ENABLE or DISABLE.
 * 
 * @return  none
 */
void QSPI_TimeoutCounterCmd(QSPI_TypeDef *QSPIx, FunctionalState NewState)
{

    if (NewState != DISABLE)
    {
        QSPIx->CR |= QSPI_CR_TCEN;
    }
    else
    {
        QSPIx->CR &= ~QSPI_CR_TCEN;
    }
    
}

/*********************************************************************
 * @fn      QSPI_AutoPollingModeStopCmd
 * 
 * @brief   Enables or disables Automatic Polling Mode Stop when a match occurs.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          NewState - ENABLE or DISABLE.
 * 
 * @return  none
 */
void QSPI_AutoPollingModeStopCmd(QSPI_TypeDef *QSPIx, FunctionalState NewState)
{

    if (NewState != DISABLE)
    {
        QSPIx->CR |= QSPI_CR_APMS;
    }
    else
    {
        QSPIx->CR &= ~QSPI_CR_APMS;
    }
    
}

/*********************************************************************
 * @fn      QSPI_AbortRequest
 * 
 * @brief   Abort the on-going command sequence.
 *
 * @param   none
 *
 * @return  none
 */
void QSPI_AbortRequest(QSPI_TypeDef *QSPIx)
{
    QSPIx->CR |= QSPI_CR_ABORT;
}

/*********************************************************************
 * @fn      QSPI_SendData8
 * 
 * @brief   Transmits a 8bit Data through the QSPI peripheral.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          Data - Data to be transmitted.
 * 
 * @return  none
 */
void QSPI_SendData8(QSPI_TypeDef *QSPIx, uint8_t Data)
{
    uint32_t qspibase = 0;

    qspibase = (uint32_t)QSPIx;
    qspibase += 0x20;

    *(__IO uint8_t *)qspibase = Data;
}

/*********************************************************************
 * @fn      QSPI_SendData16
 * 
 * @brief   Transmits a 16bit Data through the QSPI peripheral.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          Data - Data to be transmitted.
 * 
 * @return  none
 */
void QSPI_SendData16(QSPI_TypeDef *QSPIx, uint16_t Data)
{
    uint32_t qspibase = 0;

    qspibase = (uint32_t)QSPIx;
    qspibase += 0x20;

    *(__IO uint16_t *)qspibase = Data;
}

/*********************************************************************
 * @fn      QSPI_SendData32
 * 
 * @brief   Transmits a 32bit Data through the QSPI peripheral.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          Data - Data to be transmitted.
 * 
 * @return  none
 */
void QSPI_SendData32(QSPI_TypeDef *QSPIx, uint32_t Data)
{
    QSPIx->DR = Data;
}

/*********************************************************************
 * @fn      QSPI_ReceiveData8
 * 
 * @brief   Returns the most recent received 8bit data by the QSPI peripheral. 
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 * 
 * @return  The value of the received data.
 */
uint8_t QSPI_ReceiveData8(QSPI_TypeDef *QSPIx)
{
    uint32_t qspibase = 0;

    qspibase = (uint32_t)QSPIx;
    qspibase += 0x20;

    return *(__IO uint8_t *)qspibase;
}

/*********************************************************************
 * @fn      QSPI_ReceiveData16
 * 
 * @brief   Returns the most recent received 16bit data by the QSPI peripheral.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 * 
 * @return  The value of the received data.
 */
uint16_t QSPI_ReceiveData16(QSPI_TypeDef *QSPIx)
{
    uint32_t qspibase = 0;

    qspibase = (uint32_t)QSPIx;
    qspibase += 0x20;

    return *(__IO uint16_t *)qspibase;
}

/*********************************************************************
 * @fn      QSPI_ReceiveData32
 * 
 * @brief   Returns the most recent received 32bit data by the QSPI peripheral. 
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 * 
 * @return  The value of the received data.
 */
uint32_t QSPI_ReceiveData32(QSPI_TypeDef *QSPIx)
{
    return QSPIx->DR;
}

/*********************************************************************
 * @fn      QSPI_DMACmd
 * 
 * @brief   Enables or disables DMA for Indirect Mode.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          NewState - ENABLE or DISABLE.
 * 
 * @return  none
 */
void QSPI_DMACmd(QSPI_TypeDef *QSPIx, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        QSPIx->CR |= QSPI_CR_DMAEN;
    }
    else
    {
        QSPIx->CR &= ~QSPI_CR_DMAEN;
    }
}

/*********************************************************************
 * @fn      QSPI_Start
 * 
 * @brief   Start a single QSPI transfer.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 * 
 * @return  none
 */
void QSPI_Start(QSPI_TypeDef *QSPIx)
{
    QSPIx->CR |= QSPI_CR_START;
}

/*********************************************************************
 * @fn      QSPI_ITConfig
 * 
 * @brief   Enables or disables the specified QSPI interrupts.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          QSPI_IT - specifies the QSPI interrupt source to be enabled or disabled. 
 *        This parameter can be one of the following values:
 *              QSPI_IT_TO - Timeout interrupt
 *              QSPI_IT_SM - Status Match interrupt
 *              QSPI_IT_FT - FIFO Threshold
 *              QSPI_IT_TC - Transfer Complete
 *              QSPI_IT_TE - Transfer Error      
 *          NewState - ENABLE or DISABLE.
 * 
 * @return  none
 */
void QSPI_ITConfig(QSPI_TypeDef *QSPIx, uint32_t QSPI_IT, FunctionalState NewState)
{
    uint32_t tmpreg = 0;

    tmpreg = QSPIx->CR;

    if (NewState != DISABLE)
    {
        tmpreg |= (uint32_t)(QSPI_IT & QSPI_CR_INTERRUPT_MASK);
    }
    else
    {
        tmpreg &= ~(uint32_t)(QSPI_IT & QSPI_CR_INTERRUPT_MASK);
    }

    QSPIx->CR = tmpreg;
}

/*********************************************************************
 * @fn      QSPI_GetFIFOLevel
 * 
 * @brief   Returns the current QSPI FIFO filled level.
 * 
 * @return  Number of valid bytes which are being held in the FIFO.
 *        0x00 - FIFO is empty
 *        0x1F - FIFO is full    
 */
uint32_t QSPI_GetFIFOLevel(QSPI_TypeDef *QSPIx)
{
    return ((QSPIx->SR & QSPI_SR_FLEVEL) >> 8);
}

/*********************************************************************
 * @fn      QSPI_GetFMode
 * 
 * @brief   Returns the QSPI functional mode.  
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 * 
 * @return  QSPI Functional Mode .The returned value can be one of the following:
 *              - 0x00000000: QSPI_FMode_Indirect_Write
 *              - 0x04000000: QSPI_FMode_Indirect_Read
 *              - 0x08000000: QSPI_FMode_AutoPolling
 *              - 0x0C000000: QSPI_FMode_MemoryMapped
 */
uint32_t QSPI_GetFMode(QSPI_TypeDef *QSPIx)
{
    return (QSPIx->CCR & QSPI_CCR_FMODE);
}

/*********************************************************************
 * @fn      QSPI_GetFlagStatus
 * 
 * @brief   Checks whether the specified QSPI flag is set or not.  
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          QSPI_FLAG - specifies the QSPI flag to check. 
 *        This parameter can be one of the following values:
 *              QSPI_FLAG_TO - Timeout interrupt flag
 *              QSPI_FLAG_SM - Status Match interrupt flag
 *              QSPI_FLAG_FT - FIFO Threshold flag
 *              QSPI_FLAG_TC - Transfer Complete flag
 *              QSPI_FLAG_TE - Transfer Error flag
 *              QSPI_FLAG_BUSY - Busy flag      
 * 
 * @return  The new state of QSPI_FLAG (SET or RESET).
 */
FlagStatus QSPI_GetFlagStatus(QSPI_TypeDef *QSPIx, uint32_t QSPI_FLAG)
{
    FlagStatus bitstatus = RESET;

    if ((QSPIx->SR & QSPI_FLAG) != RESET)
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
 * @fn      QSPI_ClearFlag
 * 
 * @brief   Clears the QSPI flag.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          QSPI_FLAG - specifies the QSPI flag to clear. 
 *        This parameter can be one of the following values:
 *              QSPI_FLAG_TO - Timeout interrupt flag
 *              QSPI_FLAG_SM - Status Match interrupt flag
 *              QSPI_FLAG_TC - Transfer Complete flag
 *              QSPI_FLAG_TE - Transfer Error flag
 * 
 * @return  none
 */
void QSPI_ClearFlag(QSPI_TypeDef *QSPIx, uint32_t QSPI_FLAG)
{
    QSPIx->FCR = QSPI_FLAG;
}

/*********************************************************************
 * @fn      QSPI_GetITStatus
 * 
 * @brief   Checks whether the specified QSPI interrupt has occurred or not.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          QSPI_IT - specifies the QSPI interrupt source to check. 
 *        This parameter can be one of the following values:
 *              QSPI_IT_TO - Timeout interrupt 
 *              QSPI_IT_SM - Status Match interrupt
 *              QSPI_IT_FT - FIFO Threshold
 *              QSPI_IT_TC - Transfer Complete
 *              QSPI_IT_TE - Transfer Error    
 * 
 * @return  The new state of QSPI_IT (SET or RESET).
 */
ITStatus QSPI_GetITStatus(QSPI_TypeDef *QSPIx, uint32_t QSPI_IT)
{
    ITStatus bitstatus = RESET;
    uint32_t tmpcreg = 0, tmpsreg = 0;

    tmpcreg = QSPIx->CR;
    tmpcreg &= (uint32_t)(QSPI_IT & QSPI_CR_INTERRUPT_MASK);

    tmpsreg = QSPIx->SR;
    tmpsreg &= (uint32_t)(QSPI_IT & QSPI_SR_INTERRUPT_MASK);

    if ((tmpcreg != RESET) && (tmpsreg != RESET))
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
 * @fn      QSPI_ClearITPendingBit
 * 
 * @brief   Clears the QSPI's interrupt pending bits.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          QSPI_IT - specifies the QSPI pending bit to clear. 
 *        This parameter can be one of the following values:
 *              QSPI_IT_TO - Timeout interrupt 
 *              QSPI_IT_SM - Status Match interrupt
 *              QSPI_IT_TC - Transfer Complete
 *              QSPI_IT_TE - Transfer Error 
 * 
 * @return  none
 */
void QSPI_ClearITPendingBit(QSPI_TypeDef *QSPIx, uint32_t QSPI_IT)
{
    QSPIx->FCR = (uint32_t)(QSPI_IT & QSPI_FSR_INTERRUPT_MASK);
}

/*********************************************************************
 * @fn      QSPI_DualFlashMode_Cmd
 * 
 * @brief   Enables or disables QSPI Dual Flash Mode.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          NewState - ENABLE or DISABLE.
 * 
 * @return  none
 */
void QSPI_DualFlashMode_Cmd(QSPI_TypeDef *QSPIx, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        QSPIx->CR |= QSPI_CR_DFM;
    }
    else
    {
        QSPIx->CR &= ~QSPI_CR_DFM;
    }
}

/*********************************************************************
 * @fn      QSPI_EnableQuad
 * 
 * @brief   Enables or disables QSPI 4-line Mode.
 * 
 * @param   QSPIx - where x can be  - 1, 2.
 *          NewState - ENABLE or DISABLE.
 * 
 * @return  none
 */
void QSPI_EnableQuad(QSPI_TypeDef *QSPIx, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        QSPIx->CR |= QSPI_SIOXEN;
    }
    else
    {
        QSPIx->CR &= ~QSPI_SIOXEN;
    }
}
