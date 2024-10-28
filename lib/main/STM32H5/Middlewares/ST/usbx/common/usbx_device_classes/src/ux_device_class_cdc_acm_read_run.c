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
/**   Device CDC Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_device_stack.h"


#if defined(UX_DEVICE_STANDALONE)


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_cdc_acm_read_run                   PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function reads from the CDC class.                             */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    cdc_acm                                   Address of cdc_acm class  */
/*                                              instance                  */
/*    buffer                                    Pointer to buffer to save */
/*                                              received data             */
/*    requested_length                          Length of bytes to read   */
/*    actual_length                             Pointer to save number of */
/*                                              bytes read                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    State machine Status to check                                       */
/*    UX_STATE_NEXT                         Transfer done, to next state  */
/*    UX_STATE_EXIT                         Abnormal, to reset state      */
/*    (others)                              Keep running, waiting         */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_transfer_run         Run Transfer state machine    */
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
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*  10-31-2022     Yajun Xia                Modified comment(s),          */
/*                                            fixed return code,          */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_cdc_acm_read_run(UX_SLAVE_CLASS_CDC_ACM *cdc_acm,
                    UCHAR *buffer, ULONG requested_length, ULONG *actual_length)
{

UX_SLAVE_ENDPOINT           *endpoint;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_INTERFACE          *class_interface;
UX_SLAVE_TRANSFER           *transfer_request;
ULONG                       max_transfer_length;
UINT                        status = UX_SUCCESS;


#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE

    /* Check if current cdc-acm is using callback or not. We cannot use direct reads with callback on.  */
    if (cdc_acm -> ux_slave_class_cdc_acm_transmission_status == UX_TRUE)

        /* Not allowed. */
        return(UX_ERROR);
#endif

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
        cdc_acm -> ux_device_class_cdc_acm_read_state = UX_STATE_RESET;
        cdc_acm -> ux_device_class_cdc_acm_read_status = UX_CONFIGURATION_HANDLE_UNKNOWN;

        return(UX_STATE_EXIT);
    }

    /* This is the first time we are activated. We need the interface to the class.  */
    class_interface =  cdc_acm -> ux_slave_class_cdc_acm_interface;

    /* Locate the endpoints.  */
    endpoint =  class_interface -> ux_slave_interface_first_endpoint;

    /* Check the endpoint direction, if OUT we have the correct endpoint.  */
    if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_OUT)
    {

        /* So the next endpoint has to be the OUT endpoint.  */
        endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
    }

    /* All CDC reading  are on the endpoint OUT, from the host.  */
    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

    /* Handle state cases.  */
    switch(cdc_acm -> ux_device_class_cdc_acm_read_state)
    {
    case UX_STATE_RESET:

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_CDC_ACM_READ, cdc_acm, buffer, requested_length, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

        cdc_acm -> ux_device_class_cdc_acm_read_state = UX_DEVICE_CLASS_CDC_ACM_READ_START;
        cdc_acm -> ux_device_class_cdc_acm_read_status = UX_TRANSFER_NO_ANSWER;
        cdc_acm -> ux_device_class_cdc_acm_read_buffer = buffer;
        cdc_acm -> ux_device_class_cdc_acm_read_requested_length = requested_length;
        cdc_acm -> ux_device_class_cdc_acm_read_actual_length = 0;

        /* Fall through. */
    case UX_DEVICE_CLASS_CDC_ACM_READ_START:

        /* Get remaining transfer length.  */
        requested_length = cdc_acm -> ux_device_class_cdc_acm_read_requested_length -
                        cdc_acm -> ux_device_class_cdc_acm_read_actual_length;

        /* There is nothing remaining, it's done.  */
        if (requested_length == 0)
        {
            *actual_length = cdc_acm -> ux_device_class_cdc_acm_read_actual_length;
            cdc_acm -> ux_device_class_cdc_acm_read_state = UX_STATE_RESET;
            cdc_acm -> ux_device_class_cdc_acm_read_status = UX_SUCCESS;
            return(UX_STATE_NEXT);
        }

        /* Check if we have enough in the local buffer.  */
        /* Use wMaxPacketSize for faster action, UX_SLAVE_REQUEST_DATA_MAX_LENGTH for better performance.  */
        max_transfer_length = endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize;
        if (requested_length > max_transfer_length)
        {
            cdc_acm -> ux_device_class_cdc_acm_read_transfer_length = max_transfer_length;
        }
        else
        {
            cdc_acm -> ux_device_class_cdc_acm_read_transfer_length = requested_length;
        }

        /* Next state.  */
        cdc_acm -> ux_device_class_cdc_acm_read_state = UX_DEVICE_CLASS_CDC_ACM_READ_WAIT;
        UX_SLAVE_TRANSFER_STATE_RESET(transfer_request);

        /* Fall through.  */
    case UX_DEVICE_CLASS_CDC_ACM_READ_WAIT:

        /* Run the transfer state machine.  */
        status = _ux_device_stack_transfer_run(transfer_request,
                    cdc_acm -> ux_device_class_cdc_acm_read_transfer_length,
                    cdc_acm -> ux_device_class_cdc_acm_read_transfer_length);

        /* Error case.  */
        if (status < UX_STATE_NEXT)
        {
            cdc_acm -> ux_device_class_cdc_acm_read_state = UX_STATE_RESET;
            cdc_acm -> ux_device_class_cdc_acm_read_status =
                transfer_request -> ux_slave_transfer_request_completion_code;
            return(UX_STATE_ERROR);
        }

        /* Success case.  */
        if (status == UX_STATE_NEXT)
        {

            /* We need to copy the buffer locally.  */
            _ux_utility_memory_copy(cdc_acm -> ux_device_class_cdc_acm_read_buffer,
                    transfer_request -> ux_slave_transfer_request_data_pointer,
                    cdc_acm -> ux_device_class_cdc_acm_read_transfer_length); /* Use case of memcpy is verified. */

            /* Next buffer address.  */
            cdc_acm -> ux_device_class_cdc_acm_read_buffer +=
                    transfer_request -> ux_slave_transfer_request_actual_length;

            /* Set the length actually received. */
            cdc_acm -> ux_device_class_cdc_acm_read_actual_length +=
                    transfer_request -> ux_slave_transfer_request_actual_length;

            /* Last transfer status.  */
            cdc_acm -> ux_device_class_cdc_acm_read_status =
                transfer_request -> ux_slave_transfer_request_completion_code;

            /* Update actual length.  */
            *actual_length = cdc_acm -> ux_device_class_cdc_acm_read_actual_length;

            /* Check short packet.  */
            if (transfer_request -> ux_slave_transfer_request_actual_length <
                transfer_request -> ux_slave_transfer_request_requested_length)
            {

                /* It's done.  */
                cdc_acm -> ux_device_class_cdc_acm_read_state = UX_STATE_RESET;
                return(UX_STATE_NEXT);
            }

            /* Next state.  */
            cdc_acm -> ux_device_class_cdc_acm_read_state = UX_DEVICE_CLASS_CDC_ACM_READ_START;
        }

        /* Keep waiting.  */
        return(UX_STATE_WAIT);

    default: /* Error.  */
        cdc_acm -> ux_device_class_cdc_acm_read_state = UX_STATE_RESET;
        cdc_acm -> ux_device_class_cdc_acm_read_status = UX_INVALID_STATE;
        break;
    }

    /* Error cases.  */
    return(UX_STATE_EXIT);
}
#endif
