/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_dbgmcu.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the DBGMCU firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_dbgmcu.h"

#define IDCODE_DEVID_MASK    ((uint32_t)0x0000FFFF)

/*********************************************************************
 * @fn      DBGMCU_GetREVID
 *
 * @brief   Returns the device revision identifier.
 *
 * @return  Revision identifier.
 */
uint32_t DBGMCU_GetREVID(void)
{
    return ((*(uint32_t *)0x1FFFF704) & IDCODE_DEVID_MASK);
}

/*********************************************************************
 * @fn      DBGMCU_GetDEVID
 *
 * @brief   Returns the device identifier.
 *
 * @return  Device identifier.
 */
uint32_t DBGMCU_GetDEVID(void)
{
    return ((*(uint32_t *)0x1FFFF704) >> 16);
}

/*********************************************************************
 * @fn      __get_DEBUG_CR
 *
 * @brief   Return the DEBUGE Control Register
 *
 * @return  DEBUGE Control value
 */
uint32_t __get_DEBUG_CR(void)
{
    uint32_t result;

    __asm volatile("csrr %0,""0x7C0" : "=r"(result));
    return (result);
}

/*********************************************************************
 * @fn      __set_DEBUG_CR
 *
 * @brief   Set the DEBUGE Control Register
 *
 * @param   value  - set DEBUGE Control value
 *
 * @return  none
 */
void __set_DEBUG_CR(uint32_t value)
{
    __asm volatile("csrw 0x7C0, %0" : : "r"(value));
}

/*********************************************************************
 * @fn      DBGMCU_Config
 *
 * @brief   Configures the specified peripheral and low power mode behavior
 *        when the MCU under Debug mode.
 *
 * @param   DBGMCU_Periph - specifies the peripheral and low power mode.
 *            DBGMCU_IWDG_STOP - Debug IWDG stopped when Core is halted
 *            DBGMCU_WWDG_STOP - Debug WWDG stopped when Core is halted
 *            DBGMCU_I2C1_SMBUS_TIMEOUT - Debug I2C1 SMBUS time out when Core is halted
 *            DBGMCU_I2C2_SMBUS_TIMEOUT - Debug I2C2 SMBUS time out when Core is halted
 *            DBGMCU_TIM1_STOP - TIM1 counter stopped when Core is halted
 *            DBGMCU_TIM2_STOP - TIM2 counter stopped when Core is halted
 *            DBGMCU_TIM3_STOP - TIM3 counter stopped when Core is halted
 *            DBGMCU_TIM4_STOP - TIM4 counter stopped when Core is halted
 *            DBGMCU_TIM5_STOP - TIM5 counter stopped when Core is halted
 *            DBGMCU_TIM6_STOP - TIM6 counter stopped when Core is halted
 *            DBGMCU_TIM7_STOP - TIM7 counter stopped when Core is halted
 *            DBGMCU_TIM8_STOP - TIM8 counter stopped when Core is halted
 *            DBGMCU_TIM9_STOP - TIM9 counter stopped when Core is halted
 *            DBGMCU_TIM10_STOP - TIM10 counter stopped when Core is halted
 *            DBGMCU_TIM11_STOP - TIM11 counter stopped when Core is halted
 *            DBGMCU_TIM12_STOP - TIM12 counter stopped when Core is halted
 *            DBGMCU_LPTIM1_STOP - LPTIM1 counter stopped when Core is halted
 *            DBGMCU_LPTIM2_STOP - LPTIM2 counter stopped when Core is halted
 *            DBGMCU_I2C3_SMBUS_TIMEOUT - Debug I2C3 SMBUS time out when Core is halted
 *            DBGMCU_I2C4_SMBUS_TIMEOUT - Debug I2C4 SMBUS time out when Core is halted
 *            DBGMCU_CAN1_STOP - Debug CAN1 stopped when Core is halted
 *            DBGMCU_CAN2_STOP - Debug CAN2 stopped when Core is halted
 *            DBGMCU_CAN3_STOP - Debug CAN3 stopped when Core is halted
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState)
{
    uint32_t val;

    if(NewState != DISABLE)
    {
        __set_DEBUG_CR(DBGMCU_Periph);
    }
    else
    {
        val = __get_DEBUG_CR();
        val &= ~(uint32_t)DBGMCU_Periph;
        __set_DEBUG_CR(val);
    }
}

/*********************************************************************
 * @fn      DBGMCU_GetCHIPID
 *
 * @brief   Returns the CHIP identifier.
 *
 * @return Device identifier.
 *          ChipID List-
 *  CH32H417QEU6 - 0x417005xD 
 *  CH32H417QEU6 - 0x417105xD
 *  CH32H417QEU6 - 0x417205xD
 */
uint32_t DBGMCU_GetCHIPID( void )
{
    return( *( uint32_t * )0x1FFFF704 );
}

