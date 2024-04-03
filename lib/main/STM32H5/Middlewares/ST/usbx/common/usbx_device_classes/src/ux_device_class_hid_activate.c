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
/*    _ux_device_class_hid_activate                       PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function activates an instance of a HID device.                */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                              Pointer to hid command         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_thread_resume              Resume thread                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Source Code                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  12-31-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added Get/Set Protocol      */
/*                                            request support,            */
/*                                            resulting in version 6.1.3  */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added packet size assert,   */
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            added interrupt OUT support,*/
/*                                            added packet size assert,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone receiver,  */
/*                                            fixed standalone tx length, */
/*                                            fixed standalone EP IN init,*/
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_hid_activate(UX_SLAVE_CLASS_COMMAND *command)
{

UX_SLAVE_INTERFACE                      *interface_ptr;
UX_SLAVE_CLASS                          *class_ptr;
UX_SLAVE_CLASS_HID                      *hid;
UX_SLAVE_ENDPOINT                       *endpoint_interrupt;
UX_SLAVE_ENDPOINT                       *endpoint_in = UX_NULL;
#if defined(UX_DEVICE_CLASS_HID_INTERRUPT_OUT_SUPPORT)
UX_SLAVE_ENDPOINT                       *endpoint_out = UX_NULL;
#endif

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    hid =  (UX_SLAVE_CLASS_HID *) class_ptr -> ux_slave_class_instance;

    /* Get the interface that owns this instance.  */
    interface_ptr =  (UX_SLAVE_INTERFACE  *) command -> ux_slave_class_command_interface;

    /* Store the class instance into the interface.  */
    interface_ptr -> ux_slave_interface_class_instance =  (VOID *)hid;

    /* Now the opposite, store the interface in the class instance.  */
    hid -> ux_slave_class_hid_interface =  interface_ptr;

    /* Locate the endpoints.  */
    endpoint_interrupt =  interface_ptr -> ux_slave_interface_first_endpoint;

    /* Check if interrupt IN endpoint exists.  */
    while (endpoint_interrupt != UX_NULL)
    {
        if ((endpoint_interrupt -> ux_slave_endpoint_descriptor.bmAttributes &
             UX_MASK_ENDPOINT_TYPE) == UX_INTERRUPT_ENDPOINT)
        {
            if ((endpoint_interrupt -> ux_slave_endpoint_descriptor.bEndpointAddress &
                 UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN)
            {

                /* It's interrupt IN endpoint we need.  */
                endpoint_in = endpoint_interrupt;
#if defined(UX_DEVICE_CLASS_HID_INTERRUPT_OUT_SUPPORT)
                if (endpoint_out != UX_NULL)
#endif
                    break;
            }
#if defined(UX_DEVICE_CLASS_HID_INTERRUPT_OUT_SUPPORT)
            else
            {

                /* It's optional interrupt OUT endpoint.  */
                endpoint_out = endpoint_interrupt;
                if (endpoint_in != UX_NULL)
                    break;
            }
#endif
        }

        /* Try next endpoint.  */
        endpoint_interrupt =  endpoint_interrupt -> ux_slave_endpoint_next_endpoint;
    }

    /* Check if we found right endpoint.  */
    if (endpoint_in == UX_NULL)
        return (UX_ERROR);

    /* Validate event buffer size (fits largest transfer payload size).  */
    UX_ASSERT(UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH >=
              endpoint_in -> ux_slave_endpoint_transfer_request.
                            ux_slave_transfer_request_transfer_length);

    /* Default HID protocol is report protocol.  */
    hid -> ux_device_class_hid_protocol = UX_DEVICE_CLASS_HID_PROTOCOL_REPORT;

    /* Save the endpoints in the hid instance.  */
    hid -> ux_device_class_hid_interrupt_endpoint         = endpoint_in;

#if defined(UX_DEVICE_CLASS_HID_INTERRUPT_OUT_SUPPORT)

    /* Save endpoint OUT.  */
    hid -> ux_device_class_hid_read_endpoint              = endpoint_out;

    /* Resume receiver thread/task (if present).  */
    if (hid -> ux_device_class_hid_receiver && endpoint_out)
    {

        /* Reset events.  */
        hid -> ux_device_class_hid_receiver -> ux_device_class_hid_receiver_event_save_pos =
            hid -> ux_device_class_hid_receiver -> ux_device_class_hid_receiver_events;
        hid -> ux_device_class_hid_receiver -> ux_device_class_hid_receiver_event_read_pos =
            hid -> ux_device_class_hid_receiver -> ux_device_class_hid_receiver_events;
        _ux_utility_memory_set(
            hid -> ux_device_class_hid_receiver -> ux_device_class_hid_receiver_events, 0x00,
            (ALIGN_TYPE)hid -> ux_device_class_hid_receiver -> ux_device_class_hid_receiver_events_end -
            (ALIGN_TYPE)hid -> ux_device_class_hid_receiver -> ux_device_class_hid_receiver_events); /* Use case of memset is verified. */

#if !defined(UX_DEVICE_STANDALONE)

        /* Resume thread.  */
        _ux_utility_thread_resume(&hid -> ux_device_class_hid_receiver -> ux_device_class_hid_receiver_thread);
#else

        /* Setup read state for receiver.  */
        hid -> ux_device_class_hid_read_state = UX_DEVICE_CLASS_HID_RECEIVER_START;
#endif
    }
#endif

#if !defined(UX_DEVICE_STANDALONE)

    /* Resume thread.  */
    _ux_device_thread_resume(&class_ptr -> ux_slave_class_thread);
#else

    /* Reset event buffered for background transfer.  */
    _ux_utility_memory_set((VOID *)&hid -> ux_device_class_hid_event, 0,
                                            sizeof(UX_SLAVE_CLASS_HID_EVENT)); /* Use case of memset is verified. */
    hid -> ux_device_class_hid_event.ux_device_class_hid_event_length =
                    endpoint_in -> ux_slave_endpoint_transfer_request.
                                    ux_slave_transfer_request_transfer_length;

    /* Reset event sending state.  */
    hid -> ux_device_class_hid_event_state = UX_STATE_RESET;
#endif


    /* If there is a activate function call it.  */
    if (hid -> ux_slave_class_hid_instance_activate != UX_NULL)
    {

        /* Invoke the application.  */
        hid -> ux_slave_class_hid_instance_activate(hid);
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_HID_ACTIVATE, hid, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_DEVICE_OBJECT_TYPE_INTERFACE, hid, 0, 0, 0)

    /* Return completion status.  */
    return(UX_SUCCESS);
}
