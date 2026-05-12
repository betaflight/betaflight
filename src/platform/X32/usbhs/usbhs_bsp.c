/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */


#include "platform.h"

#include "usbhs_bsp.h"
#include "x32m7xx_pwr.h"

#include "drivers/nvic.h"


#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

/**
*\*\name   USB_BSP_Init.
*\*\fun    Initializes BSP configurations.
*\*\param  none
*\*\return none
*/
void USB_BSP_Init(void)
{
    GPIO_InitType GPIO_InitStructure;

    RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_AFIO, ENABLE);

#if defined(USE_USBHS1)
    RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOA, ENABLE);
    GPIO_InitStruct(&GPIO_InitStructure);

    // DM   PA11
    GPIO_InitStructure.Pin              = GPIO_PIN_11;
    GPIO_InitStructure.GPIO_Mode        = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate   = GPIO_AF10;  
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure); 

    //  DP   PA12
    GPIO_InitStructure.Pin              = GPIO_PIN_12;
    GPIO_InitStructure.GPIO_Mode        = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate   = GPIO_AF10; 
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure); 
#endif /* defined(USE_USBHS1) */

#if defined(USE_USBHS2)
    RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOB, ENABLE);
    GPIO_InitStruct(&GPIO_InitStructure);
    
    // DM   PB14
    GPIO_InitStructure.Pin              = GPIO_PIN_14;
    GPIO_InitStructure.GPIO_Mode        = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate   = GPIO_AF12;  
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure); 

    //  DP   PA12
    GPIO_InitStructure.Pin              = GPIO_PIN_15;
    GPIO_InitStructure.GPIO_Mode        = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate   = GPIO_AF11; 
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure); 
#endif /* defined(USE_USBHS2) */
}

/**
*\*\name   USB_BSP_EnableInterrupt.
*\*\fun    Enable USB Global interrupt.
*\*\param  USBx: USB device
*\*\param  coreID
*\*\        - USB1_HS_CORE_ID
*\*\        - USB2_HS_CORE_ID
*\*\return none
*/
void USB_BSP_EnableInterrupt(USB_CORE_MODULE *USBx, USB_CORE_ID_TypeDef coreID)
{
    NVIC_InitType NVIC_InitStructure;
    EXTI_InitType EXTI_InitStructure;
    
    /* Enable EXTI clocks */
    RCC_EnableAPB5PeriphClk2(RCC_APB5_PERIPHEN_EXTI, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUPING);

    if(coreID == USBHS1_CORE_ID)
    {
#ifndef USE_HOST_MODE
#ifdef USB_DEDICATED_EP_ENABLED
        NVIC_InitStructure.NVIC_IRQChannel                   = USB1_HS_EPx_OUT_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStructure);   

        NVIC_InitStructure.NVIC_IRQChannel                   = USB1_HS_EPx_IN_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStructure); 
#endif /* USB_DEDICATED_EP_ENABLED */
#endif /* USE_HOST_MODE */

        NVIC_InitStructure.NVIC_IRQChannel                   = USB1_HS_WKUP_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_USB_WUP);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = NVIC_PRIORITY_SUB(NVIC_PRIO_USB_WUP);
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStructure);     

        NVIC_InitStructure.NVIC_IRQChannel                   = USB1_HS_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_USB);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = NVIC_PRIORITY_SUB(NVIC_PRIO_USB);
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        /* Configure the EXTI line 62 connected internally to the USBHS1 IP */
        EXTI_ClrITPendBit(EXTI_LINE62);
        EXTI_InitStruct(&EXTI_InitStructure);
        EXTI_InitStructure.EXTI_Line    = EXTI_LINE62; 
        EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_InitPeripheral(&EXTI_InitStructure);
    }
    else if(coreID == USBHS2_CORE_ID)
    {
#ifndef USE_HOST_MODE
#ifdef USB_DEDICATED_EP_ENABLED
        NVIC_InitStructure.NVIC_IRQChannel                   = USB2_HS_EPx_OUT_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStructure);   

        NVIC_InitStructure.NVIC_IRQChannel                   = USB2_HS_EPx_IN_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStructure); 
#endif /* USB_DEDICATED_EP_ENABLED */
#endif /* USE_HOST_MODE */

        NVIC_InitStructure.NVIC_IRQChannel                   = USB2_HS_WKUP_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_USB_WUP);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = NVIC_PRIORITY_SUB(NVIC_PRIO_USB_WUP);
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStructure);     

        NVIC_InitStructure.NVIC_IRQChannel                   = USB2_HS_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_USB);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = NVIC_PRIORITY_SUB(NVIC_PRIO_USB);
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        /* Configure the EXTI line 63 connected internally to the USBHS2 IP */
        EXTI_ClrITPendBit(EXTI_LINE63);
        EXTI_InitStruct(&EXTI_InitStructure);
        EXTI_InitStructure.EXTI_Line    = EXTI_LINE63;
        EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_InitPeripheral(&EXTI_InitStructure);
    }
}

/**
*\*\name   USB_BSP_uDelay.
*\*\fun    This function provides delay time in micro sec.
*\*\param  usec : Value of delay required in micro sec.
*\*\return none
*/
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunsafe-loop-optimizations"
void USB_BSP_uDelay (const uint32_t usec)
{
    uint32_t count = 0;

    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreqValue(&RCC_Clocks);

    const uint32_t ticks_per_us = RCC_Clocks.SysClkFreq / 1000000U;
    const uint32_t utime = (ticks_per_us * usec / 7);
    do {
        if ( ++count > utime )
        {
        return ;
        }
    } while (1);
}
#pragma GCC diagnostic pop


/**
*\*\name   USB_BSP_mDelay.
*\*\fun    This function provides delay time in milli sec.
*\*\param  msec : Value of delay required in milli sec.
*\*\return none
*/
void USB_BSP_mDelay (const uint32_t msec)
{
    USB_BSP_uDelay(msec * 1000);
}

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif



