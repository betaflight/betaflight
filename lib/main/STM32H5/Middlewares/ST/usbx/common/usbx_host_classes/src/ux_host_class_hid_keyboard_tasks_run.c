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


#if defined(UX_HOST_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_hid_keyboard_tasks_run               PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This file contains the keyboard tasks used to process the changes   */
/*    in the keyboard LEDs.                                               */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    client                                The keyboard client           */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_class_hid_report_id_get      Get report ID                 */
/*    _ux_host_class_hid_report_set         Do SET_REPORT                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Host HID                                                       */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved HID OUTPUT report  */
/*                                            handling in standalone mode,*/
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_hid_keyboard_tasks_run(UX_HOST_CLASS_HID_CLIENT *client)
{

UX_HOST_CLASS_HID_KEYBOARD              *keyboard;
UX_HOST_CLASS_HID                       *hid;
UX_HOST_CLASS_HID_CLIENT_REPORT         client_report;
UX_HOST_CLASS_HID_REPORT_GET_ID         report_id;
UINT                                    status;


    /* Sanity check.  */
    if (client == UX_NULL)
        return;

    /* Get keyboard instance.  */
    keyboard = (UX_HOST_CLASS_HID_KEYBOARD *)client -> ux_host_class_hid_client_local_instance;
    if (keyboard == UX_NULL)
        return;

    /* Get HID instance.  */
    hid = keyboard -> ux_host_class_hid_keyboard_hid;
    if (hid == UX_NULL)
        return;

    /* Check if there is pending chnages.  */
    if (keyboard -> ux_host_class_hid_keyboard_out_state != UX_STATE_WAIT)
        return;

    /* We got awaken by the keyboard callback which indicates a change on
        the LEDs has to happen. We need to build the field for the LEDs. */
    keyboard -> ux_host_class_hid_keyboard_led_mask =
                keyboard ->  ux_host_class_hid_keyboard_alternate_key_state &
                                            UX_HID_KEYBOARD_STATE_MASK_LOCK;

    /* We need to find the OUTPUT report for the keyboard LEDs.  */
    if (keyboard -> ux_host_class_hid_keyboard_out_report == UX_NULL)
    {

        report_id.ux_host_class_hid_report_get_report = UX_NULL;
        report_id.ux_host_class_hid_report_get_type = UX_HOST_CLASS_HID_REPORT_TYPE_OUTPUT;
        status = _ux_host_class_hid_report_id_get(hid, &report_id);
        if (status != UX_SUCCESS)
        {
            keyboard -> ux_host_class_hid_keyboard_status = status;
            return;
        }
        keyboard -> ux_host_class_hid_keyboard_out_report = report_id.ux_host_class_hid_report_get_report;
    }

    /* Build a RAW client report.  */
    client_report.ux_host_class_hid_client_report = keyboard -> ux_host_class_hid_keyboard_out_report;
    client_report.ux_host_class_hid_client_report_flags = UX_HOST_CLASS_HID_REPORT_RAW;
    client_report.ux_host_class_hid_client_report_length = 1;
    client_report.ux_host_class_hid_client_report_buffer = &keyboard -> ux_host_class_hid_keyboard_led_mask;

    /* The HID class will perform the SET_REPORT command.  */
    status = _ux_host_class_hid_report_set_run(hid, &client_report);
    if (status < UX_STATE_WAIT)
    {

        /* Update status.  */
        keyboard -> ux_host_class_hid_keyboard_status = hid -> ux_host_class_hid_status;

        /* The change is processed.  */
        keyboard -> ux_host_class_hid_keyboard_out_state = UX_STATE_IDLE;
    }
}
#endif
