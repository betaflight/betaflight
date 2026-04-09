/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32h417_hsem.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the HSEM firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_hsem.h"

/* HSEM Key */
#define HSEM_KEY   (0x5AA50000)

/*********************************************************************
 * @fn      HSEM_Take
 *
 * @brief   Take a semaphore in 2 Step mode.
 *
 * @param   HSEM_ID - pointer to a HSEM_ID_TypeDef structure.
 *          ProcessID - Process ID from 0 to 255.
 *
 * @return  READY - Take success.
 *          NoREADY - Take error.
 */
ErrorStatus HSEM_Take(HSEM_ID_TypeDef HSEM_ID, uint32_t ProcessID)
{
    ErrorStatus status = NoREADY;

    HSEM->RX[HSEM_ID] = ((ProcessID & 0xFF) | ((NVIC_GetCurrentCoreID() << 8) | (1 << 31)));

    if( HSEM->RX[HSEM_ID] == ((ProcessID & 0xFF) | ((NVIC_GetCurrentCoreID() << 8) | (1 << 31))))
    {
        status = READY;
    }
    else
    {
        status = NoREADY;
    }

    return (status);
}

/*********************************************************************
 * @fn      HSEM_FastTake
 *
 * @brief   Fast Take a semaphore with 1 Step mode.
 *
 * @param   HSEM_ID - pointer to a HSEM_ID_TypeDef structure.
 *
 * @return  READY - Take success.
 *          NoREADY - Take error.
 */
ErrorStatus HSEM_FastTake(HSEM_ID_TypeDef HSEM_ID)
{
    ErrorStatus status = NoREADY;

    if( HSEM->RLRX[HSEM_ID] == ((NVIC_GetCurrentCoreID() << 8) | (1 << 31)))
    {
        status = READY;
    }
    else
    {
        status = NoREADY;
    }

    return (status);
}

/*********************************************************************
 * @fn      HSEM_GetOneSemTakenState
 *
 * @brief   Get one semaphore state Taken or not.
 *
 * @param   HSEM_ID - pointer to a HSEM_ID_TypeDef structure.
 *
 * @return  ENABLE - Take.
 *          DISABLE - No Take.
 */
FunctionalState HSEM_GetOneSemTakenState(HSEM_ID_TypeDef HSEM_ID)
{
    FunctionalState status = DISABLE;

    if( (HSEM->RX[HSEM_ID] & (1 << 31)) == (1 << 31))
    {
        status = ENABLE;
    }
    else
    {
        status = DISABLE;
    }

    return (status);
}

/*********************************************************************
 * @fn      HSEM_GetAllSemTakenState
 *
 * @brief   Get all semaphore state Taken or not.
 *
 * @return  All semaphore state Taken or not.
 */
uint32_t HSEM_GetAllSemTakenState(void)
{
    return (HSEM->LSE);
}

/*********************************************************************
 * @fn      HSEM_OwnCoreGetAllSemTakenState
 *
 * @brief   Its own core(V3F or V5F) get all semaphore state Taken or not.
 *
 * @return  All semaphore state Taken or not for own core(V3F or V5F) .
 */
uint32_t HSEM_OwnCoreGetAllSemTakenState(void)
{
    return (HSEM->LSM);
}

/*********************************************************************
 * @fn      HSEM_ReleaseOneSem
 *
 * @brief   Release one semaphore.
 *
 * @param   HSEM_ID - pointer to a HSEM_ID_TypeDef structure.
 *          ProcessID - Process ID from 0 to 255.
 *
 * @return  none
 */
void HSEM_ReleaseOneSem(HSEM_ID_TypeDef HSEM_ID, uint32_t ProcessID)
{
    HSEM->RX[HSEM_ID] = (ProcessID & 0xFF) | ((NVIC_GetCurrentCoreID() << 8));
}

/*********************************************************************
 * @fn      HSEM_ReleaseAllSem
 *
 * @brief   Release all semaphore.
 *
 * @return  none
 */
void HSEM_ReleaseAllSem(void)
{
    HSEM->CLR = HSEM_KEY | (1<<13) | (1<<12);
}

/*********************************************************************
 * @fn      HSEM_ReleaseSem_MatchCID_PID
 *
 * @brief   Release semaphore that matches the ProcessID and CoreID.
 *
 * @param   CoreID - core ID.
 *            HSEM_Core_ID_V3F - Riscv V3F
 *            HSEM_Core_ID_V5F - Riscv V5F
 *          ProcessID - Process ID from 0 to 255.
 *
 * @return  none
 */
void HSEM_ReleaseSem_MatchCID_PID(uint32_t CoreID, uint32_t ProcessID)
{
    HSEM->CLR = HSEM_KEY | CoreID | (ProcessID & 0xFF);
}

/*********************************************************************
 * @fn      HSEM_ReleaseSem_MatchCID
 *
 * @brief   Release semaphore that matches the CoreID.
 *
 * @param   CoreID - core ID.
 *            HSEM_Core_ID_V3F - Riscv V3F
 *            HSEM_Core_ID_V5F - Riscv V5F
 *
 * @return  none
 */
void HSEM_ReleaseSem_MatchCID(uint32_t CoreID )
{
    HSEM->CLR = HSEM_KEY | CoreID | (1<<12);
}

/*********************************************************************
 * @fn      HSEM_ReleaseSem_MatchPID
 *
 * @brief   Release semaphore that matches the ProcessID and CoreID.
 *
 * @param   ProcessID - Process ID from 0 to 255.
 *
 * @return  none
 */
void HSEM_ReleaseSem_MatchPID(uint32_t ProcessID)
{
    HSEM->CLR = HSEM_KEY | (ProcessID & 0xFF) | (1<<13);
}

/*********************************************************************
 * @fn      HSEM_SetClearKey
 *
 * @brief   Set semaphore Key.
 *
 * @param   Key - semaphore key value from 0 to 0xFFFF.      
 *
 * @return  none
 */
void HSEM_SetClearKey(uint32_t Key)
{
    HSEM->CLR = (Key << 16);
}

/*********************************************************************
 * @fn      HSEM_GetClearKey
 *
 * @brief   Get semaphore Key.
 *
 * @return  semaphore key value from 0 to 0xFFFF.
 */
uint32_t HSEM_GetClearKey(void)
{
    return ((HSEM->KEY >> 16));
}

/*********************************************************************
 * @fn      HSEM_ITConfig
 *
 * @brief   Enables or disables the HSEM interrupts.
 *
 * @param   HSEM_ID - pointer to a HSEM_ID_TypeDef structure.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void HSEM_ITConfig(HSEM_ID_TypeDef HSEM_ID, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        HSEM->IER |= (1 << HSEM_ID);
    }
    else
    {
        HSEM->IER &= (~(uint32_t)(1 << HSEM_ID));
    }
}

/*********************************************************************
 * @fn      HSEM_GetFlagStatus
 *
 * @brief   Checks whether the specified HSEM flag is set or not.
 *
 * @param   HSEM_ID - pointer to a HSEM_ID_TypeDef structure.
 *
 * @return  FlagStatus: SET or RESET.
 */
FlagStatus HSEM_GetFlagStatus(HSEM_ID_TypeDef HSEM_ID)
{
    FlagStatus bitstatus = RESET;

    if((HSEM->ISR & (1<<HSEM_ID)) != (uint32_t)RESET)
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
 * @fn      HSEM_ClearFlag
 *
 * @brief   Clears the HSEM's pending flags.
 *
 * @param   HSEM_ID - pointer to a HSEM_ID_TypeDef structure.
 *
 * @return  none
 */
void HSEM_ClearFlag(HSEM_ID_TypeDef HSEM_ID)
{
    HSEM->ISR = (1<<HSEM_ID);
}

/*********************************************************************
 * @fn      HSEM_GetITStatus
 *
 * @brief   Checks whether the specified HSEM interrupt has occurred or not.
 *
 * @param   HSEM_ID - pointer to a HSEM_ID_TypeDef structure.
 *
 * @return  FlagStatus: SET or RESET.
 */
ITStatus HSEM_GetITStatus(HSEM_ID_TypeDef HSEM_ID)
{
    ITStatus bitstatus = RESET;

    if(((HSEM->ISM & (1<<HSEM_ID)) != (uint32_t)RESET))
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
 * @fn      HSEM_ClearITPendingBit
 *
 * @brief   Clears the HSEM's interrupt pending bits.
 *
 * @param   HSEM_ID - pointer to a HSEM_ID_TypeDef structure.
 *
 * @return  none
 */
void HSEM_ClearITPendingBit(HSEM_ID_TypeDef HSEM_ID)
{
    HSEM->ISR = (1<<HSEM_ID);
}
