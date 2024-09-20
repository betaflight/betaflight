/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */ 
/** USBX Component                                                        */ 
/**                                                                       */
/**   HID Mouse Client Class                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_mouse.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_mouse_callback                   PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the callback mechanism for a report registration.  */ 
/*    For the mouse, we filter the mouse coordinate changes and the       */
/*    state of the buttons.                                               */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    callback                              Pointer to callback           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HID Class                                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_hid_mouse_callback(UX_HOST_CLASS_HID_REPORT_CALLBACK *callback)
{

UX_HOST_CLASS_HID_CLIENT    *hid_client;
UX_HOST_CLASS_HID_MOUSE     *mouse_instance;

    /* Get the HID client instance that issued the callback.  */
    hid_client =  callback -> ux_host_class_hid_report_callback_client;

    /* Get the mouse local instance */
    mouse_instance =  (UX_HOST_CLASS_HID_MOUSE *) hid_client -> ux_host_class_hid_client_local_instance;

    /* Analyze the usage we have received.  */
    switch (callback -> ux_host_class_hid_report_callback_usage)
    {


        /* X/Y Axis movement.  */
        case    UX_HOST_CLASS_HID_MOUSE_AXIS_X      :
            
            /* Add the deplacement to the position.  */
            mouse_instance -> ux_host_class_hid_mouse_x_position += (SCHAR) callback -> ux_host_class_hid_report_callback_value;

            break;

        case    UX_HOST_CLASS_HID_MOUSE_AXIS_Y      :

            /* Add the deplacement to the position.  */
            mouse_instance  -> ux_host_class_hid_mouse_y_position += (SCHAR) callback -> ux_host_class_hid_report_callback_value;
            break;

        /* Buttons.  */
        case    UX_HOST_CLASS_HID_MOUSE_BUTTON_1    :
    
            /* Check the state of button 1.  */
            if (callback -> ux_host_class_hid_report_callback_value == UX_TRUE)
                mouse_instance  -> ux_host_class_hid_mouse_buttons |= UX_HOST_CLASS_HID_MOUSE_BUTTON_1_PRESSED;
            else                
                mouse_instance  -> ux_host_class_hid_mouse_buttons &= (ULONG)~UX_HOST_CLASS_HID_MOUSE_BUTTON_1_PRESSED;
            break;
            
        case    UX_HOST_CLASS_HID_MOUSE_BUTTON_2    :

            /* Check the state of button 2.  */
            if (callback -> ux_host_class_hid_report_callback_value == UX_TRUE)
                mouse_instance  -> ux_host_class_hid_mouse_buttons |= UX_HOST_CLASS_HID_MOUSE_BUTTON_2_PRESSED;
            else                
                mouse_instance  -> ux_host_class_hid_mouse_buttons &= (ULONG)~UX_HOST_CLASS_HID_MOUSE_BUTTON_2_PRESSED;
            break;

        case    UX_HOST_CLASS_HID_MOUSE_BUTTON_3    :

            /* Check the state of button 3.  */
            if (callback -> ux_host_class_hid_report_callback_value == UX_TRUE)
                mouse_instance  -> ux_host_class_hid_mouse_buttons |= UX_HOST_CLASS_HID_MOUSE_BUTTON_3_PRESSED;
            else                
                mouse_instance  -> ux_host_class_hid_mouse_buttons &= (ULONG)~UX_HOST_CLASS_HID_MOUSE_BUTTON_3_PRESSED;
            break;


        /* Wheel movement.  */
        case    UX_HOST_CLASS_HID_MOUSE_WHEEL      :

            mouse_instance -> ux_host_class_hid_mouse_wheel += (SCHAR) callback -> ux_host_class_hid_report_callback_value;

            break;
        
        default :

            /* We have received a Usage we don't know about. Ignore it.  */
            break;
    }        

    /* Return to caller.  */
    return;    
}

