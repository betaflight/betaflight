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
/*    _ux_device_class_hid_control_request                PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function manages the based sent by the host on the control     */ 
/*    endpoints with a CLASS or VENDOR SPECIFIC type.                     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid                                 Pointer to hid class            */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_request     Transfer request              */ 
/*    _ux_device_class_hid_report_get       Process Get_Report request    */
/*    _ux_device_class_hid_report_set       Process Set_Report request    */
/*    _ux_device_class_hid_descriptor_send  Send requested descriptor     */
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
/*                                            prefixed UX to MS_TO_TICK,  */
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile warnings 64b, */
/*                                            resulting in version 6.1.2  */
/*  12-31-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added Get/Set Protocol      */
/*                                            request support,            */
/*                                            resulting in version 6.1.3  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_hid_control_request(UX_SLAVE_CLASS_COMMAND *command)
{

UX_SLAVE_TRANSFER           *transfer_request;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_CLASS              *class_ptr;
ULONG                       request;
ULONG                       request_value;
ULONG                       request_index;
ULONG                       request_length;
ULONG                       descriptor_type;
UCHAR                       duration;
UX_SLAVE_CLASS_HID          *hid;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get the pointer to the transfer request associated with the control endpoint.  */
    transfer_request =  &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

    /* Extract all necessary fields of the request.  */
    request =  *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);
    request_value  =   _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_VALUE);
    request_index  =   _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_INDEX);
    request_length =   _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_LENGTH);

    /* Duration - upper byte of wValue.  */
    duration       =   *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_VALUE + 1);
    
     /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;
    
    /* Get the storage instance from this class container.  */
    hid =  (UX_SLAVE_CLASS_HID *) class_ptr -> ux_slave_class_instance;

    /* Here we proceed only the standard request we know of at the device level.  */
    switch (request)
    {

        case UX_DEVICE_CLASS_HID_COMMAND_GET_REPORT:

            /* Send the requested report to the host.  */
            _ux_device_class_hid_report_get(hid, request_value, request_index, request_length);
            break;

        case UX_DEVICE_CLASS_HID_COMMAND_SET_REPORT:

            /* Extract the descriptor type.  */
            descriptor_type =  (request_value & 0xff00) >> 8;

            /* Get the requested report from the host.  */
            _ux_device_class_hid_report_set(hid, descriptor_type, request_index, request_length);
            break;

        case UX_GET_DESCRIPTOR:

            /* Send the requested descriptor to the host.  */
            _ux_device_class_hid_descriptor_send(hid, request_value, request_index, request_length);
            break;            
            
        case UX_DEVICE_CLASS_HID_COMMAND_GET_IDLE:
        case UX_DEVICE_CLASS_HID_COMMAND_SET_IDLE:

            /* Ignore Report ID for now.  */

            if (request == UX_DEVICE_CLASS_HID_COMMAND_GET_IDLE)
            {

                /* Send the idle rate.  */
                *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR)hid -> ux_device_class_hid_event_idle_rate;
                _ux_device_stack_transfer_request(transfer_request, 1, request_length);
            }
            else
            {

                /* Accept the idle rate if it changes.  */
                if ((UCHAR)hid -> ux_device_class_hid_event_idle_rate != duration)
                {

                    hid -> ux_device_class_hid_event_idle_rate = duration;
                    if (duration == 0)
                    {

                        /* No need to repeat last report, no timeout.  */
                        hid -> ux_device_class_hid_event_wait_timeout = UX_WAIT_FOREVER;
                    }
                    else
                    {

                        /* Calculate the timeout value.  Weighted as 4ms.  */
                        hid -> ux_device_class_hid_event_wait_timeout = (ULONG)UX_MS_TO_TICK((ULONG)duration << 2u);

                        /* Be sure to have a timeout that is not zero.  */
                        if (hid -> ux_device_class_hid_event_wait_timeout == 0)
                            hid -> ux_device_class_hid_event_wait_timeout ++;

#if defined(UX_DEVICE_STANDALONE)

                        /* Restart event checking if no transfer in progress.  */
                        if (hid -> ux_device_class_hid_event_state != UX_STATE_WAIT)
                            hid -> ux_device_class_hid_event_state = UX_STATE_RESET;
#else

                        /* Set an event to wake up the interrupt thread.  */
                        _ux_device_event_flags_set(&hid -> ux_device_class_hid_event_flags_group, UX_DEVICE_CLASS_HID_NEW_IDLE_RATE, UX_OR);
#endif
                    }
                }
            }
            break;

        case UX_DEVICE_CLASS_HID_COMMAND_GET_PROTOCOL:

            /* Send the protocol.  */
            *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR)hid -> ux_device_class_hid_protocol;
            _ux_device_stack_transfer_request(transfer_request, 1, request_length);
            break;

        case UX_DEVICE_CLASS_HID_COMMAND_SET_PROTOCOL:

            /* Accept the protocol.  */
            hid -> ux_device_class_hid_protocol = request_value;
            break;

        default:

            /* Unknown function. It's not handled.  */
            return(UX_ERROR);
    }

    /* It's handled.  */
    return(UX_SUCCESS);
}

