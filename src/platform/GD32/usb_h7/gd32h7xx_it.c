/*!
    \file    gd32h7xx_it.c
    \brief   main interrupt service routines

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

#include "gd32h7xx_it.h"
#include "drv_usbd_int.h"

extern usb_core_driver USB_OTG_dev;

#if 0
extern void usb_timer_irq(void);
#endif

/* local function prototypes ('static') */
static void resume_mcu_clk(void);

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
    /* if NMI exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
    /* if SVC exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
    /* if DebugMon exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
    /* if PendSV exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles FPU exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void FPU_IRQHandler(void)
{
    /* if FPU exception occurs, go to infinite loop */
    while(1) {
    }
}

#if 0
/*!
    \brief      this function handles Timer2 update interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER2_IRQHandler(void)
{
    usb_timer_irq();
}
#endif

#ifdef USE_USBHS0
/*!
    \brief      this function handles USBHS0 interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBHS0_IRQHandler(void)
{
    usbd_isr(&USB_OTG_dev);
}

#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
/*!
    \brief      this function handles USBHS1 interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBHS1_IRQHandler(void)
{
    usbd_isr(&USB_OTG_dev);
}

#endif /* USE_USBHS1 */

#ifdef USE_USBHS0
/*!
    \brief      this function handles USBHS0 wakeup interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBHS0_WKUP_IRQHandler(void)
{
    if(USB_OTG_dev.bp.low_power) {
        resume_mcu_clk();

         #ifndef USE_IRC48M

         #else
             /* enable IRC48M clock */
             rcu_osci_on(RCU_IRC48M);

             /* wait till IRC48M is ready */
             while (SUCCESS != rcu_osci_stab_wait(RCU_IRC48M)) {
             }

             rcu_ck48m_clock_config(RCU_CK48MSRC_IRC48M);
         #endif /* USE_IRC48M */

         rcu_periph_clock_enable(RCU_USBHS0);

         usb_clock_active(&USB_OTG_dev);
    }

    exti_interrupt_flag_clear(EXTI_31);
}

#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
/*!
    \brief      this function handles USBHS1 wakeup interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBHS1_WKUP_IRQHandler(void)
{
    if(USB_OTG_dev.bp.low_power) {
        resume_mcu_clk();

        #ifndef USE_IRC48M

        #else
            /* enable IRC48M clock */
            rcu_osci_on(RCU_IRC48M);

            /* wait till IRC48M is ready */
            while (SUCCESS != rcu_osci_stab_wait(RCU_IRC48M)) {
            }

            rcu_ck48m_clock_config(RCU_CK48MSRC_IRC48M);
        #endif /* USE_IRC48M */

        rcu_periph_clock_enable(RCU_USBHS1);

        usb_clock_active(&USB_OTG_dev);
    }

    exti_interrupt_flag_clear(EXTI_32);
}

#endif /* USE_USBHS1 */

#ifdef USB_DEDICATED_EP1_ENABLED

#ifdef USE_USBHS0
/*!
    \brief      this function handles USBHS0 dedicated endpoint 1 OUT interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBHS0_EP1_OUT_IRQHandler(void)
{
    usbd_int_dedicated_ep1out(&USB_OTG_dev);
}

#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
/*!
    \brief      this function handles USBHS1 dedicated endpoint 1 OUT interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBHS1_EP1_OUT_IRQHandler(void)
{
    usbd_int_dedicated_ep1out(&USB_OTG_dev);
}

#endif /* USE_USBHS1 */

#ifdef USE_USBHS0
/*!
    \brief      this function handles USBHS0 dedicated endpoint 1 IN interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBHS0_EP1_IN_IRQHandler(void)
{
    usbd_int_dedicated_ep1in(&USB_OTG_dev);
}

#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
/*!
    \brief      this function handles USBHS1 dedicated endpoint 1 IN interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBHS1_EP1_IN_IRQHandler(void)
{
    usbd_int_dedicated_ep1in(&USB_OTG_dev);
}

#endif /* USE_USBHS1 */

#endif /* USB_DEDICATED_EP1_ENABLED */

/*!
    \brief      resume MCU clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void resume_mcu_clk(void)
{
    /* enable HXTAL */
    rcu_osci_on(RCU_HXTAL);

    /* wait till HXTAL is ready */
    while(RESET == rcu_flag_get(RCU_FLAG_HXTALSTB)) {
    }

    /* enable PLL1 */
    rcu_osci_on(RCU_PLL1_CK);

    /* wait till PLL1 is ready */
    while(RESET == rcu_flag_get(RCU_FLAG_PLL1STB)) {
    }

    /* enable PLL */
    rcu_osci_on(RCU_PLL0_CK);

    /* wait till PLL is ready */
    while(RESET == rcu_flag_get(RCU_FLAG_PLL0STB)) {
    }

    /* select PLL as system clock source */
    rcu_system_clock_source_config(RCU_CKSYSSRC_PLL0P);

    /* wait till PLL is used as system clock source */
    while(RCU_SCSS_PLL0P != rcu_system_clock_source_get()) {
    }
}
