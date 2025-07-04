/*!
    \file    gd32f4xx_it.h
    \brief   the header file of the ISR

    \version 2024-12-20, V3.3.1, firmware for GD32F4xx
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#ifndef GD32F4XX_IT_H
#define GD32F4XX_IT_H

#include "gd32f4xx.h"

/* function declarations */
/* this function handles NMI exception */
void NMI_Handler(void);

void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);

/* this function handles SVC exception */
void SVC_Handler(void);
/* this function handles DebugMon exception */
void DebugMon_Handler(void);
/* this function handles PendSV exception */
void PendSV_Handler(void);

void SysTick_Handler(void);

#ifdef USE_USB_FS
/* this function handles USBFS wakeup interrupt request */
void USBFS_WKUP_IRQHandler(void);
/* this function handles USBFS global interrupt request */
void USBFS_IRQHandler(void);

#endif /* USE_USB_FS */
#ifdef USE_USB_HS
/* this function handles USBHS wakeup interrupt request */
void USBHS_WKUP_IRQHandler(void);
/* this function handles USBHS global interrupt request */
void USBHS_IRQHandler(void);
#endif /* USE_USB_HS */
#ifdef USB_HS_DEDICATED_EP1_ENABLED
/* this function handles EP1_IN IRQ interrupt request */
void USBHS_EP1_In_IRQHandler(void);
/* this function handles EP1_OUT IRQ interrupt request */
void USBHS_EP1_Out_IRQHandler(void);
#endif /* USB_HS_DEDICATED_EP1_ENABLED */

#endif /* GD32F4XX_IT_H */
