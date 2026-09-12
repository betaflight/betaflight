/*!
    \file    usb_bsp.c
    \brief   USB hardware configuration for GD32H7xx

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

#include "drv_usb_hw.h"
#include "usbd_cdc_vcp.h"

#define TIM_MSEC_DELAY                          0x01U
#define TIM_USEC_DELAY                          0x02U

#if (HSE_VALUE==25000000)
#define RCU_PLLUSBHSPRE_VAL    RCU_PLLUSBHSPRE_DIV5
#define RCU_PLLUSBHS_MUL_VAL   RCU_PLLUSBHS_MUL96
#elif (HSE_VALUE==24000000)
#define RCU_PLLUSBHSPRE_VAL    RCU_PLLUSBHSPRE_DIV4
#define RCU_PLLUSBHS_MUL_VAL   RCU_PLLUSBHS_MUL80
#elif (HSE_VALUE==12000000)
#define RCU_PLLUSBHSPRE_VAL    RCU_PLLUSBHSPRE_DIV3
#define RCU_PLLUSBHS_MUL_VAL   RCU_PLLUSBHS_MUL120
#elif (HSE_VALUE==8000000)
#define RCU_PLLUSBHSPRE_VAL    RCU_PLLUSBHSPRE_DIV2
#define RCU_PLLUSBHS_MUL_VAL   RCU_PLLUSBHS_MUL120
#else
#define RCU_PLLUSBHSPRE_VAL    RCU_PLLUSBHSPRE_DIV2
#define RCU_PLLUSBHS_MUL_VAL   RCU_PLLUSBHS_MUL120
#endif

__IO uint32_t delay_time = 0U;
__IO uint16_t timer_prescaler = 5U;

/*!
    \brief      configure USB clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_rcu_config(void)
{
    pmu_usb_regulator_enable();
    pmu_usb_voltage_detector_enable();
    while(SET != pmu_flag_get(PMU_FLAG_USB33RF)) {
    }

#ifndef USE_IRC48M

#ifdef USE_USBHS0
    rcu_usb48m_clock_config(IDX_USBHS0, RCU_USB48MSRC_PLL0R);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    rcu_usb48m_clock_config(IDX_USBHS1, RCU_USB48MSRC_PLL0R);
#endif /* USE_USBHS1 */

#else
    /* enable IRC48M clock */
    rcu_osci_on(RCU_IRC48M);

    /* wait till IRC48M is ready */
    while(SUCCESS != rcu_osci_stab_wait(RCU_IRC48M)) {
    }

#ifdef USE_USBHS0
    rcu_usb48m_clock_config(IDX_USBHS0, RCU_USB48MSRC_IRC48M);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    rcu_usb48m_clock_config(IDX_USBHS1, RCU_USB48MSRC_IRC48M);
#endif /* USE_USBHS1 */

#endif /* USE_IRC48M */

#ifdef USE_USBHS0
    rcu_periph_clock_enable(RCU_USBHS0);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    rcu_periph_clock_enable(RCU_USBHS1);
#endif /* USE_USBHS1 */

#ifdef USE_ULPI_PHY
#ifdef USE_USBHS0
    rcu_periph_clock_enable(RCU_USBHS0ULPI);
#endif

#ifdef USE_USBHS1
    rcu_periph_clock_enable(RCU_USBHS1ULPI);
#endif
#endif /* USE_ULPI_PHY */
}

/*!
    \brief      configure USB data line GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_gpio_config(void)
{
#ifdef USE_ULPI_PHY

#ifdef USE_USBHS0
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOF);

    /* ULPI_STP(PC0) GPIO pin configuration */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_0);

    /* ULPI_CK(PA5) GPIO pin configuration */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_5);

    /* ULPI_NXT(PC3) GPIO pin configuration */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_3);

    /* ULPI_DIR(PI11) GPIO pin configuration */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_2);

    /* ULPI_D1(PB0), ULPI_D2(PB1), ULPI_D3(PB10), ULPI_D4(PB11) \
       ULPI_D5(PB12), ULPI_D6(PB13) and ULPI_D7(PB5) GPIO pin configuration */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, \
                    GPIO_PIN_5 |\
                      GPIO_PIN_10 | GPIO_PIN_1 | GPIO_PIN_0);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, \
                    GPIO_PIN_5   |\
                     GPIO_PIN_10 | GPIO_PIN_1 | GPIO_PIN_0);


    gpio_mode_set(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, \
                    GPIO_PIN_2 |\
                    GPIO_PIN_1 | GPIO_PIN_0);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, \
                    GPIO_PIN_2   |\
                    GPIO_PIN_1 | GPIO_PIN_0);

    /* ULPI_D0(PA3) GPIO pin configuration */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_3);

    gpio_af_set(GPIOF, GPIO_AF_5, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_1);
    gpio_af_set(GPIOC, GPIO_AF_10, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);
    gpio_af_set(GPIOA, GPIO_AF_10, GPIO_PIN_5 | GPIO_PIN_3);
    gpio_af_set(GPIOB, GPIO_AF_10, GPIO_PIN_5  |\
                                    GPIO_PIN_10 | GPIO_PIN_1 | GPIO_PIN_0);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    rcu_periph_clock_enable(RCU_GPIOH);
    rcu_periph_clock_enable(RCU_GPIOG);

    /* ULPI_STP(PH2) GPIO pin configuration */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_2);


    /* ULPI_CK(PH5) GPIO pin configuration */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_5);

    /* ULPI_NXT(PH4) GPIO pin configuration */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_4);

    /* ULPI_DIR(PH3) GPIO pin configuration */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_3);

    /* ULPI_D0(PH6) GPIO pin configuration */
    /* ULPI_D1(PH7), ULPI_D2(PH8), ULPI_D3(PH9), ULPI_D4(PH10) \
       ULPI_D5(PH11), ULPI_D6(PH12) and ULPI_D7(PG5) GPIO pin configuration */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, \
                    GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 |\
                    GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, \
                    GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 |\
                    GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);

    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_5);

    gpio_af_set(GPIOG, GPIO_AF_8, GPIO_PIN_5);
    gpio_af_set(GPIOH, GPIO_AF_8, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |\
                                   GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8| GPIO_PIN_9| GPIO_PIN_10| GPIO_PIN_11| GPIO_PIN_12);
#endif /* USE_USBHS1 */

#endif /* USE_ULPI_PHY */
}

/*!
    \brief      USB bsp initialize
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_bsp_para_init(void)
{
#ifdef USE_USBHS0

#ifdef USE_USB_FS
    usb_para_init(&USB_OTG_dev, USBHS0, USB_SPEED_FULL);
#endif
#ifdef USE_USB_HS
    usb_para_init(&USB_OTG_dev, USBHS0, USB_SPEED_HIGH);
#endif

#endif /* USE_USBHS0 */

#ifdef USE_USBHS1

#ifdef USE_USB_FS
    usb_para_init(&USB_OTG_dev, USBHS1, USB_SPEED_FULL);
#endif
#ifdef USE_USB_HS
    usb_para_init(&USB_OTG_dev, USBHS1, USB_SPEED_HIGH);
#endif

#endif /* USE_USBHS1 */

#ifdef USE_USB_HS

#ifndef USE_ULPI_PHY
#ifdef USE_USBHS1
    pllusb_rcu_config(USBHS1);
#else
    pllusb_rcu_config(USBHS0);
#endif
#endif /* !USE_ULPI_PHY */

#endif /* USE_USB_HS */
}

/*!
    \brief      configure USB interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_intr_config(void)
{
     nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);

#ifdef USE_USBHS0
    nvic_irq_enable(USBHS0_IRQn, 3U, 0U);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    nvic_irq_enable(USBHS1_IRQn, 3U, 0U);
#endif /* USE_USBHS0 */

    /* enable the power module clock */
    rcu_periph_clock_enable(RCU_PMU);

#ifdef USE_USBHS0
    /* USB wakeup EXTI line configuration */
    exti_interrupt_flag_clear(EXTI_31);
    exti_init(EXTI_31, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_enable(EXTI_31);

    nvic_irq_enable(USBHS0_WKUP_IRQn, 1U, 0U);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    /* USB wakeup EXTI line configuration */
    exti_interrupt_flag_clear(EXTI_32);
    exti_init(EXTI_32, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_enable(EXTI_32);

    nvic_irq_enable(USBHS1_WKUP_IRQn, 1U, 0U);
#endif /* USE_USBHS1 */

#ifdef USB_DEDICATED_EP1_ENABLED

#ifdef USE_USBHS0
    nvic_irq_enable(USBHS0_EP1_OUT_IRQn, 1U, 0U);
    nvic_irq_enable(USBHS0_EP1_IN_IRQn, 1U, 0U);
#endif /* USE_USBHS0 */

#ifdef USE_USBHS1
    nvic_irq_enable(USBHS1_EP1_OUT_IRQn, 1U, 0U);
    nvic_irq_enable(USBHS1_EP1_IN_IRQn, 1U, 0U);
#endif /* USE_USBHS1 */

#endif /* USB_DEDICATED_EP1_ENABLED */
}

/*!
    \brief      delay in microseconds
    \param[in]  usec: value of delay required in microseconds
    \param[out] none
    \retval     none
*/
void usb_udelay(const uint32_t usec)
{
    uint32_t count = 0;

    const uint32_t ticks_per_us = SystemCoreClock / 1000000U;
    const uint32_t utime = (ticks_per_us * usec / 7);
    do {
        if ( ++count > utime )
        {
        return ;
        }
    } while (1);
}

/*!
    \brief      delay in milliseconds
    \param[in]  msec: value of delay required in milliseconds
    \param[out] none
    \retval     none
*/
void usb_mdelay(const uint32_t msec)
{
    usb_udelay(msec*1000);
}

/*!
    \brief      configure the PLL of USB
    \param[in]  usb_periph: USBHS0 or USBHS1
    \param[out] none
    \retval     none
*/
void pllusb_rcu_config(uint32_t usb_periph)
{
    if(USBHS0 == usb_periph) {
        rcu_pllusb0_config(RCU_PLLUSBHSPRE_HXTAL, RCU_PLLUSBHSPRE_VAL, RCU_PLLUSBHS_MUL_VAL, RCU_USBHS_DIV8);
        RCU_ADDCTL1 |= RCU_ADDCTL1_PLLUSBHS0EN;
        while(0U == (RCU_ADDCTL1 & RCU_ADDCTL1_PLLUSBHS0STB)) {
        }

        rcu_usbhs_clock_selection_enable(IDX_USBHS0);
        rcu_usb48m_clock_config(IDX_USBHS0, RCU_USB48MSRC_PLLUSBHS);
        rcu_usbhs_clock_config(IDX_USBHS0, RCU_USBHSSEL_60M);
    } else {
        rcu_pllusb1_config(RCU_PLLUSBHSPRE_HXTAL, RCU_PLLUSBHSPRE_VAL, RCU_PLLUSBHS_MUL_VAL, RCU_USBHS_DIV8);
        RCU_ADDCTL1 |= RCU_ADDCTL1_PLLUSBHS1EN;
        while(0U == (RCU_ADDCTL1 & RCU_ADDCTL1_PLLUSBHS1STB)) {
        }

        rcu_usbhs_clock_selection_enable(IDX_USBHS1);
        rcu_usb48m_clock_config(IDX_USBHS1, RCU_USB48MSRC_PLLUSBHS);
        rcu_usbhs_clock_config(IDX_USBHS1, RCU_USBHSSEL_60M);
    }
}

#if 0
/*!
    \brief      timer base IRQ
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_timer_irq(void)
{
    if(RESET != timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_UP)) {
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);

        if(delay_time > 0x00U) {
            delay_time--;
        } else {
            timer_disable(TIMER2);
        }
    }
}

/*!
   \brief      delay routine based on TIMER2
   \param[in]  nTime: delay Time
   \param[in]  unit: delay Time unit = milliseconds / microseconds
   \param[out] none
   \retval     none
*/
static void hw_delay(uint32_t ntime, uint8_t unit)
{
   delay_time = ntime;

   hw_time_set(unit);

   while(0U != delay_time) {
   }

   timer_disable(TIMER2);
}

/*!
   \brief      configures TIMER for delay routine based on Timer2
   \param[in]  unit: msec /usec
   \param[out] none
   \retval     none
*/
static void hw_time_set(uint8_t unit)
{
   timer_parameter_struct  timer_basestructure;

   timer_prescaler = ((rcu_clock_freq_get(CK_APB1) / 1000000U * 2U) / 12U) - 1U;

   timer_disable(TIMER2);
   timer_interrupt_disable(TIMER2, TIMER_INT_UP);

   if(TIM_USEC_DELAY == unit) {
       timer_basestructure.period = 11U;
   } else if(TIM_MSEC_DELAY == unit) {
       timer_basestructure.period = 11999U;
   } else {
       /* no operation */
   }

   timer_basestructure.prescaler         = timer_prescaler;
   timer_basestructure.alignedmode       = TIMER_COUNTER_EDGE;
   timer_basestructure.counterdirection  = TIMER_COUNTER_UP;
   timer_basestructure.clockdivision     = TIMER_CKDIV_DIV1;
   timer_basestructure.repetitioncounter = 0U;

   timer_init(TIMER2, &timer_basestructure);

   timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);

   timer_auto_reload_shadow_enable(TIMER2);

   /* TIMER2 interrupt enable */
   timer_interrupt_enable(TIMER2, TIMER_INT_UP);

   /* TIMER2 enable counter */
   timer_enable(TIMER2);
}
#endif
