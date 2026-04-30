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

#include "usbhs_dcd_int.h"

 extern USB_CORE_MODULE USB_dev;


 #ifdef USE_USBHS1
/**
*\*\name    USB1_HS_WKUP_IRQHandler.
*\*\fun     This function handles USB_HS_WKUP_IRQHandler Handler.
*\*\param   none
*\*\return  none 
**/
void USB1_HS_WKUP_IRQHandler(void)
{
    EXTI_ClrITPendBit(EXTI_LINE62);
    USBD_ISTR_WKUP_handler(&USB_dev);
}

/**
*\*\name    USB1_HS_IRQHandler.
*\*\fun     This function handles USB_HS_IRQHandler Handler.
*\*\param   none
*\*\return  none 
**/
void USB1_HS_IRQHandler(void)
{
    USBD_ISTR_Handler(&USB_dev);
}

#ifdef USB_DEDICATED_EP_ENABLED
/**
*\*\name    USB1_HS_EPx_OUT_IRQHandler.
*\*\fun     This function handles USB_HS_EPx_OUT_IRQHandler Handler.
*\*\param   none
*\*\return  none 
**/
void USB1_HS_EPx_OUT_IRQHandler()
{
    USBD_EP_OUT_ISTR_Handler(&USB_dev);
}

/**
*\*\name    USB1_HS_EPx_IN_IRQHandler.
*\*\fun     This function handles USB_HS_EPx_IN_IRQHandler Handler.
*\*\param   none
*\*\return  none 
**/
void USB1_HS_EPx_IN_IRQHandler()
{
    USBD_EP_IN_ISTR_Handler(&USB_dev);
}
#endif /* USB_DEDICATED_EP_ENABLED */
#endif /* USE_USBHS1 */

#ifdef USE_USBHS2
/**
*\*\name    USB2_HS_WKUP_IRQHandler.
*\*\fun     This function handles USB_HS_WKUP_IRQHandler Handler.
*\*\param   none
*\*\return  none 
**/
void USB2_HS_WKUP_IRQHandler(void)
{
    EXTI_ClrITPendBit(EXTI_LINE63);
    USBD_ISTR_WKUP_handler(&USB_dev);
}

/**
*\*\name    USB2_HS_IRQHandler.
*\*\fun     This function handles USB_HS_IRQHandler Handler.
*\*\param   none
*\*\return  none 
**/
void USB2_HS_IRQHandler(void)
{
    USBD_ISTR_Handler(&USB_dev);
}

#ifdef USB_DEDICATED_EP_ENABLED
/**
*\*\name    USB2_HS_EPx_OUT_IRQHandler.
*\*\fun     This function handles USB_HS_EPx_OUT_IRQHandler Handler.
*\*\param   none
*\*\return  none 
**/
void USB2_HS_EPx_OUT_IRQHandler()
{
    USBD_EP_OUT_ISTR_Handler(&USB_dev);
}

/**
*\*\name    USB2_HS_EPx_IN_IRQHandler.
*\*\fun     This function handles USB_HS_EPx_IN_IRQHandler Handler.
*\*\param   none
*\*\return  none 
**/
void USB2_HS_EPx_IN_IRQHandler()
{
    USBD_EP_IN_ISTR_Handler(&USB_dev);
}
#endif /* USB_DEDICATED_EP_ENABLED */
#endif /* USE_USBHS2 */