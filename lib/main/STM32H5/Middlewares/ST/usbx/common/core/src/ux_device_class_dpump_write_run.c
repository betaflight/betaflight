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
/**   Device DPUMP Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_dpump.h"
#include "ux_device_stack.h"


#if defined(UX_DEVICE_STANDALONE)

#define UX_DEVICE_CLASS_DPUMP_WRITE_START    (UX_STATE_STEP + 1)
#define UX_DEVICE_CLASS_DPUMP_WRITE_WAIT     (UX_STATE_STEP + 2)

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_dpump_write_run                    PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function writes to the DPUMP class.                            */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dpump                                   DPUMP class instance        */
/*    buffer                                  Buffer data to write        */
/*    requested_length                        Bytes to write              */
/*    actual_length                           Bytes written               */
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
/*    _ux_device_stack_transfer_request     Request transfer              */
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
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_dpump_write_run(UX_SLAVE_CLASS_DPUMP *dpump, UCHAR *buffer,
                                ULONG requested_length, ULONG *actual_length)
{

UX_SLAVE_ENDPOINT           *endpoint;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_TRANSFER           *transfer_request;
UINT                        status;
UINT                        zlp = UX_FALSE;
UINT                        read_state;


    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_DPUMP_WRITE, dpump, buffer, requested_length, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

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
        dpump -> ux_device_class_dpump_write_state = UX_STATE_RESET;
        dpump -> ux_device_class_dpump_write_status = UX_CONFIGURATION_HANDLE_UNKNOWN;
        return(UX_STATE_EXIT);
    }

    /* Locate the OUT endpoint.  */
    endpoint =  dpump -> ux_slave_class_dpump_bulkin_endpoint;

    /* Check endpoint. If NULL, we have not yet received the proper SET_INTERFACE command.  */
    if (endpoint == UX_NULL)
    {
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ENDPOINT_HANDLE_UNKNOWN);

        dpump -> ux_device_class_dpump_write_state = UX_STATE_RESET;
        dpump -> ux_device_class_dpump_write_status = UX_ENDPOINT_HANDLE_UNKNOWN;
        return(UX_STATE_EXIT);
    }

    /* All DPUMP reading  are on the endpoint OUT, from the host.  */
    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

    /* Handle state cases.  */
    read_state = dpump -> ux_device_class_dpump_write_state;
    switch(read_state)
    {
    case UX_STATE_RESET:
        dpump -> ux_device_class_dpump_write_state = UX_DEVICE_CLASS_DPUMP_WRITE_START;
        dpump -> ux_device_class_dpump_write_status = UX_TRANSFER_NO_ANSWER;
        dpump -> ux_device_class_dpump_write_buffer = buffer;
        dpump -> ux_device_class_dpump_write_requested_length = requested_length;
        dpump -> ux_device_class_dpump_write_actual_length = 0;
        if (requested_length == 0)
            zlp = UX_TRUE;

        /* Fall through.  */
    case UX_DEVICE_CLASS_DPUMP_WRITE_START:

        /* Get remaining requested length.  */
        requested_length = dpump -> ux_device_class_dpump_write_requested_length -
                        dpump -> ux_device_class_dpump_write_actual_length;

        /* There is no remaining, we are done.  */
        if (requested_length == 0 && !zlp)
        {
            *actual_length = dpump -> ux_device_class_dpump_write_actual_length;
            dpump -> ux_device_class_dpump_write_state = UX_STATE_RESET;
            dpump -> ux_device_class_dpump_write_status = UX_SUCCESS;
            return(UX_STATE_NEXT);
        }

        /* Check if we have enough in the local buffer.  */
        if (requested_length > UX_SLAVE_REQUEST_DATA_MAX_LENGTH)

            /* We have too much to transfer.  */
            dpump -> ux_device_class_dpump_write_transfer_length = UX_SLAVE_REQUEST_DATA_MAX_LENGTH;

        else

            /* We can proceed with the demanded length.  */
            dpump -> ux_device_class_dpump_write_transfer_length = requested_length;

        /* On a out, we copy the buffer to the caller. Not very efficient but it makes the API
           easier.  */
        _ux_utility_memory_copy(transfer_request -> ux_slave_transfer_request_data_pointer, 
                            dpump -> ux_device_class_dpump_write_buffer,
                            dpump -> ux_device_class_dpump_write_transfer_length); /* Use case of memcpy is verified. */

        /* Next state.  */
        dpump -> ux_device_class_dpump_write_state = UX_DEVICE_CLASS_DPUMP_WRITE_WAIT;
        UX_SLAVE_TRANSFER_STATE_RESET(transfer_request);

        /* Fall through.  */
    case UX_DEVICE_CLASS_DPUMP_WRITE_WAIT:

        /* Send the request to the device controller.  */
        status =  _ux_device_stack_transfer_run(transfer_request,
                            dpump -> ux_device_class_dpump_write_transfer_length,
                            dpump -> ux_device_class_dpump_write_transfer_length);

        /* Error case.  */
        if (status < UX_STATE_NEXT)
        {

            dpump -> ux_device_class_dpump_write_state = UX_STATE_RESET;
            dpump -> ux_device_class_dpump_write_status =
                transfer_request -> ux_slave_transfer_request_completion_code;
            return(UX_STATE_EXIT);
        }

        /* Success case.  */
        if (status == UX_STATE_NEXT)
        {

            /* Next buffer address.  */
            dpump -> ux_device_class_dpump_write_buffer +=
                    transfer_request -> ux_slave_transfer_request_actual_length;

            /* Set the length actually received. */
            dpump -> ux_device_class_dpump_write_actual_length +=
                    transfer_request -> ux_slave_transfer_request_actual_length;

            /* Last transfer status.  */
            dpump -> ux_device_class_dpump_write_status =
                transfer_request -> ux_slave_transfer_request_completion_code;

            /* Update actual length.  */
            *actual_length = dpump -> ux_device_class_dpump_write_actual_length;

            /* Check ZLP case.  */
            if (dpump -> ux_device_class_dpump_write_requested_length == 0)
            {
                dpump -> ux_device_class_dpump_write_state = UX_STATE_RESET;
                return(UX_STATE_NEXT);
            }

            /* Next state.  */
            dpump -> ux_device_class_dpump_write_state = UX_DEVICE_CLASS_DPUMP_WRITE_START;
        }

        /* Keep waiting.  */
        return(UX_STATE_WAIT);

    default: /* Error.  */
        dpump -> ux_device_class_dpump_write_state = UX_STATE_RESET;
        break;
    }

    /* Error case.  */
    return(UX_STATE_EXIT);
}
#endif
