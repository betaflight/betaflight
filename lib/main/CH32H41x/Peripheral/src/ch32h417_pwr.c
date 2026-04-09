/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32h417_pwr.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the PWR firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_pwr.h"
#include "ch32h417_rcc.h"

/* PWR registers bit mask */
/* CTLR register bit mask */
#define CTLR_DS_MASK     ((uint32_t)0xFFFFFFFE)
#define CTLR_PLS_MASK    ((uint32_t)0xFFFFFF1F)

/*********************************************************************
 * @fn      PWR_DeInit
 *
 * @brief   Deinitializes the PWR peripheral registers to their default
 *        reset values.
 *
 * @return  none
 */
void PWR_DeInit(void)
{
    RCC_HB1PeriphResetCmd(RCC_HB1Periph_PWR, ENABLE);
    RCC_HB1PeriphResetCmd(RCC_HB1Periph_PWR, DISABLE);
}

/*********************************************************************
 * @fn      PWR_BackupAccessCmd
 *
 * @brief   Enables or disables access to the RTC and backup registers.
 *
 * @param   NewState - new state of the access to the RTC and backup registers,
 *            This parameter can be: ENABLE or DISABLE.
 *
 * @return  none
 */
void PWR_BackupAccessCmd(FunctionalState NewState)
{
    if(NewState)
    {
        PWR->CTLR |= PWR_CTLR_DBP;
    }
    else
    {
        PWR->CTLR &= ~(PWR_CTLR_DBP);
    }
}

/*********************************************************************
 * @fn      PWR_PVDCmd
 *
 * @brief   Enables or disables the Power Voltage Detector(PVD).
 *
 * @param   NewState - new state of the PVD(ENABLE or DISABLE).
 *
 * @return  none
 */
void PWR_PVDCmd(FunctionalState NewState)
{
    if(NewState)
    {
        PWR->CTLR |= PWR_CTLR_PVDE;
    }
    else
    {
        PWR->CTLR &= ~(PWR_CTLR_PVDE);
    }
}

/*********************************************************************
 * @fn      PWR_PVDLevelConfig
 *
 * @brief   Configures the voltage threshold detected by the Power Voltage
 *        Detector(PVD).
 *
 * @param   PWR_PVDLevel - specifies the PVD detection level
 *            PWR_PVDLevel_MODE0 - PVD detection level set to mode 0.
 *            PWR_PVDLevel_MODE1 - PVD detection level set to mode 1.
 *            PWR_PVDLevel_MODE2 - PVD detection level set to mode 2.
 *            PWR_PVDLevel_MODE3 - PVD detection level set to mode 3.
 *            PWR_PVDLevel_MODE4 - PVD detection level set to mode 4.
 *            PWR_PVDLevel_MODE5 - PVD detection level set to mode 5.
 *            PWR_PVDLevel_MODE6 - PVD detection level set to mode 6.
 *            PWR_PVDLevel_MODE7 - PVD detection level set to mode 7.
 *
 * @return  none
 */
void PWR_PVDLevelConfig(uint32_t PWR_PVDLevel)
{
    uint32_t tmpreg = 0;
    tmpreg = PWR->CTLR;
    tmpreg &= CTLR_PLS_MASK;
    tmpreg |= PWR_PVDLevel;
    PWR->CTLR = tmpreg;
}

/*********************************************************************
 * @fn      PWR_EnterSTOPMode
 *
 * @brief   Enters STOP mode.
 *
 * @param   PWR_Regulator - specifies the regulator state in STOP mode.
 *            PWR_Regulator_ON - STOP mode with regulator ON
 *            PWR_Regulator_LowPower - STOP mode with regulator in low power mode
 *          PWR_STOPEntry - specifies if STOP mode in entered with WFI or WFE instruction.
 *            PWR_STOPEntry_WFI - enter STOP mode with WFI instruction
 *            PWR_STOPEntry_WFE - enter STOP mode with WFE instruction
 *
 * @return  none
 */
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry)
{
    uint32_t tmpreg = 0;
    tmpreg = PWR->CTLR;
    tmpreg &= CTLR_DS_MASK;
    tmpreg |= PWR_Regulator;
    PWR->CTLR = tmpreg;

    NVIC->SCTLR |= (1 << 2);

    if(PWR_STOPEntry == PWR_STOPEntry_WFI)
    {
        __WFI();
    }
    else
    {
        __WFE();
    }

    NVIC->SCTLR &= ~(1 << 2);
}

/*********************************************************************
 * @fn      PWR_GetFlagStatus
 *
 * @brief   Checks whether the specified PWR flag is set or not.
 *
 * @param   PWR_FLAG - specifies the flag to check.
 *            PWR_FLAG_PVDO - PVD Output
 *
 * @return  The new state of PWR_FLAG (SET or RESET).
 */
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG)
{
    FlagStatus bitstatus = RESET;

    if((PWR->CSR & PWR_FLAG) != (uint32_t)RESET)
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
 * @fn      PWR_VIO18ModeCfg
 *
 * @brief   specified the VIO18 configuration mode .
 *
 * @param   PWR_VIO18CfgMode - specifies the VIO18 configuration mode.
 *            PWR_VIO18CFGMODE_SW - software Configuration
 *            PWR_VIO18CFGMODE_HW - hardware Configuration
 *
 * @return  none.
 */
void PWR_VIO18ModeCfg(uint32_t PWR_VIO18CfgMode)
{
    PWR->CTLR &= ~PWR_CTLR_VIO_SWCR;
    PWR->CTLR |= PWR_VIO18CfgMode;
 }

/*********************************************************************
 * @fn      PWR_GetFlagStatus
 *
 * @brief   specified the VIO18 level.
 *
 * @param   VIO18Level - specifies the VIO18 level.
 *            PWR_VIO18Level_MODE0 - VIO18 level 0
 *            PWR_VIO18Level_MODE1 - VIO18 level 1
 *            PWR_VIO18Level_MODE2 - VIO18 level 2
 *            PWR_VIO18Level_MODE3 - VIO18 level 3
 *            PWR_VIO18Level_MODE4 - VIO18 level 4
 *            PWR_VIO18Level_MODE5 - VIO18 level 5
 *
 * @return  none.
 */
void PWR_VIO18LevelCfg(uint16_t VIO18Level)
{
    PWR->CTLR &= ~PWR_CTLR_VSEL_VIO18;
    PWR->CTLR |= VIO18Level;
}

/*********************************************************************
 * @fn      PWR_GetVIO18InitialStatus
 *
 * @brief   Get the VIO18 initial status.
 *
 * @return  PWR_VIO18InitialStatus - VIO18 initial status
 *            PWR_VIO18InitialStatus_0 - VIO18 initial status 0
 *            PWR_VIO18InitialStatus_1 - VIO18 initial status 1
 *            PWR_VIO18InitialStatus_2 - VIO18 initial status 2
 *            PWR_VIO18InitialStatus_3 - VIO18 initial status 3
 */
PWR_VIO18InitialStatus PWR_GetVIO18InitialStatus(void)
{
    return ((PWR->CSR & PWR_CSR_VIO18_SR) >> 8);
}
