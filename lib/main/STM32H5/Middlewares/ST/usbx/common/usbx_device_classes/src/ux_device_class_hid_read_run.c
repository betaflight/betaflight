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


#if defined(UX_DEVICE_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_hid_read_run                       PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function reads from the HID class.                             */
/*    This function must not be used with receiver related functions.     */
/*    This function is for standalone mode.                               */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hid                                   Address of hid class          */
/*                                              instance                  */
/*    buffer                                Pointer to buffer to save     */
/*                                              received data             */
/*    requested_length                      Length of bytes to read       */
/*    actual_length                         Pointer to save number of     */
/*                                              bytes read                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_transfer_request     Transfer request              */
/*    _ux_utility_memory_copy               Copy memory                   */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Initial Version 6.1.12        */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_hid_read_run(UX_SLAVE_CLASS_HID *hid, UCHAR *buffer,
                                   ULONG requested_length, ULONG *actual_length)
{
#if !defined(UX_DEVICE_CLASS_HID_INTERRUPT_OUT_SUPPORT)
    UX_PARAMETER_NOT_USED(hid);
    UX_PARAMETER_NOT_USED(buffer);
    UX_PARAMETER_NOT_USED(requested_length);
    UX_PARAMETER_NOT_USED(actual_length);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_SLAVE_ENDPOINT           *endpoint;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_TRANSFER           *transfer_request;
UINT                        read_state;
UINT                        status= UX_SUCCESS;


    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_HID_READ, hid, buffer, requested_length, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* As long as the device is in the CONFIGURED state.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CONFIGURATION_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONFIGURATION_HANDLE_UNKNOWN, device, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Cannot proceed with command, the interface is down.  */
        hid -> ux_device_class_hid_read_state = UX_STATE_RESET;
        hid -> ux_device_class_hid_read_status = UX_CONFIGURATION_HANDLE_UNKNOWN;
        return(UX_STATE_EXIT);
    }

    /* Locate the endpoint.  */
    endpoint =  hid -> ux_device_class_hid_read_endpoint;

    /* Check endpoint. If NULL, we have not yet received the proper SET_INTERFACE command.  */
    if (endpoint == UX_NULL)
    {
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ENDPOINT_HANDLE_UNKNOWN);

        hid -> ux_device_class_hid_read_state = UX_STATE_RESET;
        hid -> ux_device_class_hid_read_status = UX_CONFIGURATION_HANDLE_UNKNOWN;
        return(UX_STATE_EXIT);
    }

    /* All HID reading  are on the endpoint OUT, from the host.  */
    transfer_request = &endpoint -> ux_slave_endpoint_transfer_request;

    /* Handle state cases.  */
    read_state = hid -> ux_device_class_hid_read_state;
    switch(read_state)
    {
    case UX_STATE_RESET:
        hid -> ux_device_class_hid_read_state = UX_DEVICE_CLASS_HID_READ_START;
        hid -> ux_device_class_hid_read_status = UX_TRANSFER_NO_ANSWER;
        hid -> ux_device_class_hid_read_buffer = buffer;
        hid -> ux_device_class_hid_read_requested_length = requested_length;
        hid -> ux_device_class_hid_read_actual_length = 0;

        /* Fall through.  */
    case UX_DEVICE_CLASS_HID_READ_START:

        /* Get remaining requested length.  */
        requested_length = hid -> ux_device_class_hid_read_requested_length -
                        hid -> ux_device_class_hid_read_actual_length;

        /* There is no remaining, we are done.  */
        if (requested_length == 0)
        {
            *actual_length = hid -> ux_device_class_hid_read_actual_length;
            hid -> ux_device_class_hid_read_state = UX_STATE_RESET;
            hid -> ux_device_class_hid_read_status = UX_SUCCESS;
            return(UX_STATE_NEXT);
        }

        /* Check if we have enough in the local buffer.  */
        if (requested_length > UX_SLAVE_REQUEST_DATA_MAX_LENGTH)

            /* We have too much to transfer.  */
            hid -> ux_device_class_hid_read_transfer_length = UX_SLAVE_REQUEST_DATA_MAX_LENGTH;

        else

            /* We can proceed with the demanded length.  */
            hid -> ux_device_class_hid_read_transfer_length = requested_length;

        /* Next state.  */
        hid -> ux_device_class_hid_read_state = UX_DEVICE_CLASS_HID_READ_WAIT;
        UX_SLAVE_TRANSFER_STATE_RESET(transfer_request);

        /* Fall through.  */
    case UX_DEVICE_CLASS_HID_READ_WAIT:

        /* Send the request to the device controller.  */
        status =  _ux_device_stack_transfer_run(transfer_request,
                            hid -> ux_device_class_hid_read_transfer_length,
                            hid -> ux_device_class_hid_read_transfer_length);

        /* Error case.  */
        if (status < UX_STATE_NEXT)
        {

            hid -> ux_device_class_hid_read_state = UX_STATE_RESET;
            hid -> ux_device_class_hid_read_status =
                transfer_request -> ux_slave_transfer_request_completion_code;
            return(UX_STATE_EXIT);
        }

        /* Success case.  */
        if (status == UX_STATE_NEXT)
        {

            /* We need to copy the buffer locally.  */
            _ux_utility_memory_copy(hid -> ux_device_class_hid_read_buffer,
                    transfer_request -> ux_slave_transfer_request_data_pointer,
                    transfer_request -> ux_slave_transfer_request_actual_length); /* Use case of memcpy is verified. */

            /* Next buffer address.  */
            hid -> ux_device_class_hid_read_buffer +=
                    transfer_request -> ux_slave_transfer_request_actual_length;

            /* Set the length actually received. */
            hid -> ux_device_class_hid_read_actual_length +=
                    transfer_request -> ux_slave_transfer_request_actual_length;

            /* Last transfer status.  */
            hid -> ux_device_class_hid_read_status =
                transfer_request -> ux_slave_transfer_request_completion_code;

            /* Update actual length.  */
            *actual_length = hid -> ux_device_class_hid_read_actual_length;

            /* Check short packet.  */
            if (transfer_request -> ux_slave_transfer_request_actual_length <
                transfer_request -> ux_slave_transfer_request_requested_length)
            {

                /* It's done.  */
                hid -> ux_device_class_hid_read_state = UX_STATE_RESET;
                return(UX_STATE_NEXT);
            }

            /* Next state.  */
            hid -> ux_device_class_hid_read_state = UX_DEVICE_CLASS_HID_READ_START;
        }

        /* Keep waiting.  */
        return(UX_STATE_WAIT);
    
    /* Receiver running states.  */
    case UX_DEVICE_CLASS_HID_RECEIVER_START:    /* Fall through.  */
    case UX_DEVICE_CLASS_HID_RECEIVER_WAIT:     /* Fall through.  */
    case UX_DEVICE_CLASS_HID_RECEIVER_ERROR:

        /* Receiver running.  */
        return(UX_STATE_ERROR);

    default: /* Error.  */
        hid -> ux_device_class_hid_read_status = UX_INVALID_STATE;
        hid -> ux_device_class_hid_read_state = UX_STATE_RESET;
        break;
    }

    /* Error case.  */
    return(UX_STATE_EXIT);
#endif
}
#endif
