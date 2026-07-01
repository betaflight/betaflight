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

#include "usbd_user.h"


USBD_User_cb_TypeDef USER_cb = 
{
    USBD_USER_Init,
    USBD_USER_DeviceReset,
    USBD_USER_DeviceConfigured,
    USBD_USER_DeviceSuspended,
    USBD_USER_DeviceResumed,
    USBD_USER_DeviceConnected,
    USBD_USER_DeviceDisconnected,
};

/**
*\*\name   USBD_USER_Init.
*\*\fun    host lib initialization.
*\*\param  none.
*\*\param  none.
*\*\return none.
*/
void USBD_USER_Init(void)
{

}

/**
*\*\name   USBD_USER_DeviceReset.
*\*\fun    device Reset Event.
*\*\param  speed : device speed.
*\*\param  none.
*\*\return none.
*/
void USBD_USER_DeviceReset(uint8_t speed)
{
    switch (speed)
    {
        case USB_SPEED_HIGH:
        break;

        case USB_SPEED_FULL:
        break;
        default:
        break;
    }
}

/**
*\*\name   USBD_USER_DeviceConfigured.
*\*\fun    device configuration Event.
*\*\param  none.
*\*\param  none.
*\*\return none.
*/
void USBD_USER_DeviceConfigured(void)
{

}

/**
*\*\name   USBD_USER_DeviceConnected.
*\*\fun    device connection Event.
*\*\param  none.
*\*\param  none.
*\*\return none.
*/
void USBD_USER_DeviceConnected(void)
{
}

/**
*\*\name   USBD_USER_DeviceDisconnected.
*\*\fun    device disconnection Event.
*\*\param  none.
*\*\param  none.
*\*\return none.
*/
void USBD_USER_DeviceDisconnected(void)
{
}

/**
*\*\name   USBD_USER_DeviceSuspended.
*\*\fun    device suspend Event.
*\*\param  none.
*\*\param  none.
*\*\return none.
*/
void USBD_USER_DeviceSuspended(void)
{
}

/**
*\*\name   USBD_USER_DeviceResumed.
*\*\fun    device resume Event.
*\*\param  none.
*\*\param  none.
*\*\return none.
*/
void USBD_USER_DeviceResumed(void)
{
}


