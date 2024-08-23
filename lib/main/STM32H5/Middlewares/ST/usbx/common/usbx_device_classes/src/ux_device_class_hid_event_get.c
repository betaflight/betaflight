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
/*    _ux_device_class_hid_event_get                      PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function checks if there is an event from the application      */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid                                      Address of hid class       */ 
/*    event                                    Pointer of the event       */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                   UX_SUCCESS if there is an  */ 
/*                                             event                      */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_copy                  Copy memory                */
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
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_hid_event_get(UX_SLAVE_CLASS_HID *hid, 
                                      UX_SLAVE_CLASS_HID_EVENT *hid_event)
{

UX_SLAVE_CLASS_HID_EVENT        *current_hid_event;
UX_SLAVE_DEVICE                 *device;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_HID_EVENT_GET, hid, hid_event, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Get the pointer to the device.  */                   
    device =  &_ux_system_slave -> ux_system_slave_device;
    
    /* Check the device state.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
        return(UX_DEVICE_HANDLE_UNKNOWN);

    /* Check if the head and the tail of the event array is the same.  */
    if (hid -> ux_device_class_hid_event_array_head == 
        hid -> ux_device_class_hid_event_array_tail)

        /* No event to report.  */
        return(UX_ERROR);        

    /* There is an event to report, get the current pointer to the event.  */
    current_hid_event =  hid -> ux_device_class_hid_event_array_tail;

    /* Keep the event data length inside buffer area.  */
    if (current_hid_event -> ux_device_class_hid_event_length > UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH)
        current_hid_event -> ux_device_class_hid_event_length = UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH;

    /* fill in the event structure from the user.  */
    hid_event -> ux_device_class_hid_event_length =  current_hid_event -> ux_device_class_hid_event_length;
    _ux_utility_memory_copy(hid_event -> ux_device_class_hid_event_buffer, current_hid_event -> ux_device_class_hid_event_buffer,
                                current_hid_event -> ux_device_class_hid_event_length); /* Use case of memcpy is verified. */

    /* Adjust the tail pointer.  Check if we are at the end.  */
    if ((current_hid_event + 1) == hid -> ux_device_class_hid_event_array_end)

        /* We are at the end, go back to the beginning.  */
        hid -> ux_device_class_hid_event_array_tail =  hid -> ux_device_class_hid_event_array;
        
    else        
        /* We are not at the end, increment the tail position.  */
        hid -> ux_device_class_hid_event_array_tail++;

    /* Return event status to the user.  */
    return(UX_SUCCESS);
}

