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
/**   Device HID Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_hid.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_hid_receiver_event_free             PORTABLE C     */
/*                                                            6.1.11      */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function free event buffer for new incoming interrupt OUT      */
/*    and advance the current reading position.                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hid                                   Pointer of the HID instance   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    status                                UX_SUCCESS if freed           */
/*                                          UX_ERROR if nothing to free   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_hid_receiver_event_free(UX_SLAVE_CLASS_HID *hid)
{
#if !defined(UX_DEVICE_CLASS_HID_INTERRUPT_OUT_SUPPORT)
    UX_PARAMETER_NOT_USED(hid);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_DEVICE_CLASS_HID_RECEIVER            *receiver;
UX_DEVICE_CLASS_HID_RECEIVED_EVENT      *pos;
UCHAR                                   *next_pos;

    /* Get receiver.  */
    receiver = hid -> ux_device_class_hid_receiver;

    /* Get current event.  */
    pos = receiver -> ux_device_class_hid_receiver_event_read_pos;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_HID_RECEIVER_EVENT_FREE, hid, pos, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* If event is not valid, return error.  */
    if (pos -> ux_device_class_hid_received_event_length == 0)
        return(UX_ERROR);

    /* Invalidate the event and advance position.  */
    next_pos = (UCHAR *)pos + receiver -> ux_device_class_hid_receiver_event_buffer_size + sizeof(ULONG);
    if (next_pos >= (UCHAR *)receiver -> ux_device_class_hid_receiver_events_end)
        next_pos = (UCHAR *)receiver -> ux_device_class_hid_receiver_events;
    receiver -> ux_device_class_hid_receiver_event_read_pos = (UX_DEVICE_CLASS_HID_RECEIVED_EVENT *)next_pos;
    pos -> ux_device_class_hid_received_event_length = 0;

    /* Inform receiver thread to (re)start.  */
    _ux_device_event_flags_set(&hid -> ux_device_class_hid_event_flags_group,
                                UX_DEVICE_CLASS_HID_RECEIVER_RESTART, UX_OR);

    /* Return event status to the user.  */
    return(UX_SUCCESS);
#endif
}
