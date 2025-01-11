/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_it.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main Interrupt Service Routines.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "ch32v30x_it.h"

void NMI_Handler(void) __attribute__((naked));
void HardFault_Handler(void) __attribute__((naked));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handle(void){
      __asm volatile ("call NMI_Handler_impl; mret");
}

__attribute__((used)) void NMI_Handler_impl(void)
{

}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void){
      __asm volatile ("call HardFault_Handler_impl; mret");
}

__attribute__((used)) void HardFault_Handler_impl(void)
{
  while (1)
  {
  }
}
