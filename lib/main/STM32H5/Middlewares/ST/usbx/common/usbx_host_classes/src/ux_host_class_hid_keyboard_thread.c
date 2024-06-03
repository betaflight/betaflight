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
/**   HID Keyboard Client                                                 */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_keyboard.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_keyboard_thread                  PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains the keyboard thread used to process the changes  */
/*    in the keyboard LEDs.                                               */ 
/*                                                                        */
/*    It's for RTOS mode.                                                 */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    keyboard_instance                     The keyboard instance for     */ 
/*                                          there was a change in LEDs    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_semaphore_get                Get signal semaphore          */ 
/*    _ux_host_class_hid_report_id_get      Get report ID                 */
/*    _ux_host_class_hid_report_set         Do SET_REPORT                 */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    ThreadX                                                             */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_hid_keyboard_thread(ULONG thread_input)
{
UX_HOST_CLASS_HID                       *hid;
UX_HOST_CLASS_HID_CLIENT_REPORT         client_report;
UX_HOST_CLASS_HID_REPORT_GET_ID         report_id;
UINT                                    status;
UX_HOST_CLASS_HID_KEYBOARD              *keyboard_instance;

    /* Cast the thread_input variable into the proper format.  */
    UX_THREAD_EXTENSION_PTR_GET(keyboard_instance, UX_HOST_CLASS_HID_KEYBOARD, thread_input)

    /* Loop forever waiting for changes signaled through the semaphore. */     
    while (1)
    {   

        /* Wait for the semaphore to be put by the root hub or a regular hub.  */
        status =  _ux_host_semaphore_get(&keyboard_instance -> ux_host_class_hid_keyboard_semaphore, UX_WAIT_FOREVER);
        
        /* The semaphore could be awaken because the semaphore was destroyed by the HID client
           when the keyboard is removed.  */
        if (status == UX_SUCCESS)           
        {

            /* We got awaken by the keyboard callback which indicates a change on the LEDs has to happen.  
               We need to build the field for the leds. */
            keyboard_instance -> ux_host_class_hid_keyboard_led_mask =  keyboard_instance ->  ux_host_class_hid_keyboard_alternate_key_state &
                                                                        UX_HID_KEYBOARD_STATE_MASK_LOCK;
    
            /* Recall the HID instance for this client.  */
            hid =  keyboard_instance -> ux_host_class_hid_keyboard_hid;
    
            /* We need to find the OUTPUT report for the keyboard LEDs.  */
            report_id.ux_host_class_hid_report_get_report = UX_NULL;
            report_id.ux_host_class_hid_report_get_type = UX_HOST_CLASS_HID_REPORT_TYPE_OUTPUT;
            status =  _ux_host_class_hid_report_id_get(hid, &report_id);
    
            /* The report ID should exist.  If there is an error, we do not proceed.  */
            if (status == UX_SUCCESS)
            {
    
                /* Memorize the report pointer.  */
                client_report.ux_host_class_hid_client_report =  report_id.ux_host_class_hid_report_get_report;
                
                /* The report set is raw since the LEDs mask is already in the right format.  */
                client_report.ux_host_class_hid_client_report_flags = UX_HOST_CLASS_HID_REPORT_RAW;
    
                /* The length of this report is 1 byte.  */
                client_report.ux_host_class_hid_client_report_length = 1;
            
                /* The output report buffer is the LED mask field.  */
                client_report.ux_host_class_hid_client_report_buffer = &keyboard_instance -> ux_host_class_hid_keyboard_led_mask;
    
                /* The HID class will perform the SET_REPORT command.  */
                _ux_host_class_hid_report_set(hid, &client_report);
            }
        }
        else

            /* The thread should terminate upon a semaphore error.  */
            return;
    }
}

