/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32f20x_it.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/08/08
* Description        : This file contains the headers of the interrupt handlers.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32F20xIT_H
#define __CH32F20xIT_H

#include "ch32f20x.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);


#endif /* __CH32F20xIT_H */
