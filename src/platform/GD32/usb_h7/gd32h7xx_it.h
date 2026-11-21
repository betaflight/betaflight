/*!
    \file    gd32f4xx_it.h
    \brief   the header file of the ISR

    \version 2025-01-24, V1.4.0, firmware for GD32H7xx
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

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

#ifndef GD32H7XX_IT_H
#define GD32H7XX_IT_H

#include "gd32h7xx.h"

/* function declarations */
/* this function handles NMI exception */
void NMI_Handler(void);
/* this function handles HardFault exception */
void HardFault_Handler(void);
/* this function handles MemManage exception */
void MemManage_Handler(void);
/* this function handles BusFault exception */
void BusFault_Handler(void);
/* this function handles UsageFault exception */
void UsageFault_Handler(void);
/* this function handles SVC exception */
void SVC_Handler(void);
/* this function handles DebugMon exception */
void DebugMon_Handler(void);
/* this function handles PendSV exception */
void PendSV_Handler(void);
/* this function handles FPU exception */
void FPU_IRQHandler(void);
/* this function handles TIMER2 IRQ Handler */
void TIMER2_IRQHandler(void);

#ifdef USE_USBHS0
/* this function handles USBHS wakeup interrupt handler */
void USBHS0_WKUP_IRQHandler(void);
/* this function handles USBHS IRQ Handler */
void USBHS0_IRQHandler(void);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
/* this function handles USBHS wakeup interrupt handler */
void USBHS1_WKUP_IRQHandler(void);
/* this function handles USBHS IRQ Handler */
void USBHS1_IRQHandler(void);
#endif /* USE_USBHS1 */

#ifdef USB_DEDICATED_EP1_ENABLED

#ifdef USE_USBHS0
/* this function handles USBHS0 dedicated endpoint 1 OUT interrupt request */
void USBHS0_EP1_OUT_IRQHandler(void);
/* this function handles USBHS0 dedicated endpoint 1 IN interrupt request */
void USBHS0_EP1_IN_IRQHandler(void);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
/* this function handles USBHS1 dedicated endpoint 1 OUT interrupt request */
void USBHS1_EP1_OUT_IRQHandler(void);
/* this function handles USBHS1 dedicated endpoint 1 IN interrupt request */
void USBHS1_EP1_IN_IRQHandler(void);
#endif /* USE_USBHS1 */

#endif /* USB_DEDICATED_EP1_ENABLED */

#endif /* GD32H7XX_IT_H */
