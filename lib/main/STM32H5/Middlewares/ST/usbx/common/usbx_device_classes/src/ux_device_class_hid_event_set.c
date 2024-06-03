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
/*    _ux_device_class_hid_event_set                      PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function sends an event to the hid class. It is processed      */ 
/*    asynchronously by the interrupt thread.                             */ 
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
/*    _ux_device_event_flags_set               Set event flags            */
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
/*                                            added standalone support,   */
/*                                            verified memset and memcpy  */
/*                                            cases, used UX prefix to    */
/*                                            refer to TX symbols instead */
/*                                            of using them directly,     */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_hid_event_set(UX_SLAVE_CLASS_HID *hid, 
                                      UX_SLAVE_CLASS_HID_EVENT *hid_event)
{

UX_SLAVE_CLASS_HID_EVENT   *current_hid_event;
UX_SLAVE_CLASS_HID_EVENT   *next_hid_event;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_HID_EVENT_SET, hid, hid_event, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Current position of the head.  */
    current_hid_event =  hid -> ux_device_class_hid_event_array_head;
    
    /* If the pointer is NULL, the round robin buffer has not been activated.  */
    if (current_hid_event == UX_NULL)
        return (UX_ERROR);
    
    /* Calculate the next position.  */
    if ((current_hid_event + 1) == hid -> ux_device_class_hid_event_array_end)

        /* We are at the end, go back to the beginning.  */
        next_hid_event =  hid -> ux_device_class_hid_event_array;
        
    else        

        /* We are not at the end, increment the head position.  */
        next_hid_event = current_hid_event + 1;
    

    /* Any place left for this event ? */
    if (next_hid_event == hid -> ux_device_class_hid_event_array_tail)
        return (UX_ERROR);

    /* There is an event to report, get the current pointer to the event.  */
    current_hid_event =  hid -> ux_device_class_hid_event_array_head;

    /* Update the head.  */
    hid -> ux_device_class_hid_event_array_head = next_hid_event;

    /* Check if this event has a report ID.  */
    if (hid -> ux_device_class_hid_report_id == UX_TRUE)
    {

        /* Yes, there's a report ID. Check to see if our event buffer can also
           fit the extra byte.  */
        if (hid_event -> ux_device_class_hid_event_length + 1 > UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_MEMORY_INSUFFICIENT, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

            /* Return overflow error.  */
            return(UX_MEMORY_INSUFFICIENT);
        }

        /* Store the report ID.  */
        *current_hid_event -> ux_device_class_hid_event_buffer =  (UCHAR)(hid_event -> ux_device_class_hid_event_report_id);  
                
        /* Store the data itself.  */
        _ux_utility_memory_copy(current_hid_event -> ux_device_class_hid_event_buffer + 1, hid_event -> ux_device_class_hid_event_buffer,
                                hid_event -> ux_device_class_hid_event_length); /* Use case of memcpy is verified. */
    
        /* fill in the event structure from the user.  */
        current_hid_event -> ux_device_class_hid_event_length =  hid_event -> ux_device_class_hid_event_length + 1;    
    }
    else
    {
    
        /* No report ID to consider.  */
        _ux_utility_memory_copy(current_hid_event -> ux_device_class_hid_event_buffer, hid_event -> ux_device_class_hid_event_buffer,
                                hid_event -> ux_device_class_hid_event_length); /* Use case of memcpy is verified. */

        /* fill in the event structure from the user.  */
        current_hid_event -> ux_device_class_hid_event_length = hid_event -> ux_device_class_hid_event_length;    
    }

#if defined(UX_DEVICE_STANDALONE)

    /* Set state machine to start sending if no transfer on going.  */
    if (hid -> ux_device_class_hid_event_state != UX_STATE_WAIT &&
        hid -> ux_device_class_hid_event_state != UX_STATE_EXIT)
        hid -> ux_device_class_hid_event_state = UX_STATE_RESET;
#else

    /* Set an event to wake up the interrupt thread.  */
    _ux_device_event_flags_set(&hid -> ux_device_class_hid_event_flags_group, UX_DEVICE_CLASS_HID_NEW_EVENT, UX_OR);                
#endif

    /* Return event status to the user.  */
    return(UX_SUCCESS);
}

