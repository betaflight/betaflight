/*!
    \file    gd32f4xx_hw.c
    \brief   USB hardware configuration for GD32F4xx

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

#include "drv_usb_hw.h"

#define TIM_MSEC_DELAY                          0x01U
#define TIM_USEC_DELAY                          0x02U

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
#ifdef USE_USB_FS

    /* configure the PLLSAIP = 48MHz, PLLSAI_N = 288, PLLSAI_P = 6, PLLSAI_R = 2 */
    rcu_pllsai_config(288U, 6U, 2U);
    /* enable PLLSAI */
    RCU_CTL |= RCU_CTL_PLLSAIEN;
    /* wait until PLLSAI is stable */
    while(0U == (RCU_CTL & RCU_CTL_PLLSAISTB)){
    }

    rcu_pll48m_clock_config(RCU_PLL48MSRC_PLLSAIP);
    rcu_ck48m_clock_config(RCU_CK48MSRC_PLL48M);

    rcu_periph_clock_enable(RCU_USBFS);

#elif defined(USE_USB_HS)

#ifdef USE_EMBEDDED_PHY
    rcu_pll48m_clock_config(RCU_PLL48MSRC_PLLQ);
    rcu_ck48m_clock_config(RCU_CK48MSRC_PLL48M);
#elif defined(USE_ULPI_PHY)
    rcu_periph_clock_enable(RCU_USBHSULPI);
#endif /* USE_EMBEDDED_PHY */

    rcu_periph_clock_enable(RCU_USBHS);

#endif /* USB_USBFS */
}

/*!
    \brief      configure USB data line GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_gpio_config(void)
{
    rcu_periph_clock_enable(RCU_SYSCFG);

#ifdef USE_USB_FS
    rcu_periph_clock_enable(RCU_GPIOA);

    /* USBFS_DM(PA11) and USBFS_DP(PA12) GPIO pin configuration */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11 | GPIO_PIN_12);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_11 | GPIO_PIN_12);

    gpio_af_set(GPIOA, GPIO_AF_10, GPIO_PIN_11 | GPIO_PIN_12);

#elif defined(USE_USB_HS)

#ifdef USE_ULPI_PHY
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOH);
    rcu_periph_clock_enable(RCU_GPIOI);

    /* ULPI_STP(PC0) GPIO pin configuration */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_0);

    /* ULPI_CK(PA5) GPIO pin configuration */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_5);

    /* ULPI_NXT(PH4) GPIO pin configuration */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_4);

    /* ULPI_DIR(PI11) GPIO pin configuration */
    gpio_mode_set(GPIOI, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOI, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_11);

    /* ULPI_D1(PB0), ULPI_D2(PB1), ULPI_D3(PB10), ULPI_D4(PB11) \
       ULPI_D5(PB12), ULPI_D6(PB13) and ULPI_D7(PB5) GPIO pin configuration */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, \
                  GPIO_PIN_5 | GPIO_PIN_13 | GPIO_PIN_12 | \
                  GPIO_PIN_11 | GPIO_PIN_10 | GPIO_PIN_1 | GPIO_PIN_0);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, \
                            GPIO_PIN_5 | GPIO_PIN_13 | GPIO_PIN_12 | \
                            GPIO_PIN_11 | GPIO_PIN_10 | GPIO_PIN_1 | GPIO_PIN_0);

    /* ULPI_D0(PA3) GPIO pin configuration */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_3);

    gpio_af_set(GPIOC, GPIO_AF_10, GPIO_PIN_0);
    gpio_af_set(GPIOH, GPIO_AF_10, GPIO_PIN_4);
    gpio_af_set(GPIOI, GPIO_AF_10, GPIO_PIN_11);
    gpio_af_set(GPIOA, GPIO_AF_10, GPIO_PIN_5 | GPIO_PIN_3);
    gpio_af_set(GPIOB, GPIO_AF_10, GPIO_PIN_5 | GPIO_PIN_13 | GPIO_PIN_12 | \
                GPIO_PIN_11 | GPIO_PIN_10 | GPIO_PIN_1 | GPIO_PIN_0);
#elif defined(USE_EMBEDDED_PHY)
    rcu_periph_clock_enable(RCU_GPIOB);

    /* USBHS_DM(PB14) and USBHS_DP(PB15) GPIO pin configuration */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14 | GPIO_PIN_15);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_14 | GPIO_PIN_15);
    gpio_af_set(GPIOB, GPIO_AF_12, GPIO_PIN_14 | GPIO_PIN_15);
#endif /* USE_ULPI_PHY */

#endif /* USE_USBFS */
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

#ifdef USE_USB_FS
    nvic_irq_enable(USBFS_IRQn, 2U, 0U);

#if USBFS_LOW_POWER
    /* enable the power module clock */
    rcu_periph_clock_enable(RCU_PMU);

    /* USB wakeup EXTI line configuration */
    exti_interrupt_flag_clear(EXTI_18);
    exti_init(EXTI_18, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_enable(EXTI_18);

    nvic_irq_enable(USBFS_WKUP_IRQn, 0U, 0U);
#endif /* USBFS_LOW_POWER */
#elif defined(USE_USB_HS)
    nvic_irq_enable(USBHS_IRQn, 2U, 0U);

#if USBHS_LOW_POWER
    /* enable the power module clock */
    rcu_periph_clock_enable(RCU_PMU);

    /* USB wakeup EXTI line configuration */
    exti_interrupt_flag_clear(EXTI_20);
    exti_init(EXTI_20, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_enable(EXTI_20);

    nvic_irq_enable(USBHS_WKUP_IRQn, 0U, 0U);
#endif /* USBHS_LOW_POWER */
#endif /* USE_USB_FS */

#ifdef USB_HS_DEDICATED_EP1_ENABLED
    nvic_irq_enable(USBHS_EP1_Out_IRQn, 1U, 0U);
    nvic_irq_enable(USBHS_EP1_In_IRQn, 1U, 0U);
#endif /* USB_HS_DEDICATED_EP1_ENABLED */
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

    const uint32_t utime = (120 * usec / 7);
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
