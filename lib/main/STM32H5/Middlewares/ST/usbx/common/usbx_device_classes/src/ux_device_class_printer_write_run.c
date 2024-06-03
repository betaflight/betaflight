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
/**   Device Printer Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_printer.h"
#include "ux_device_stack.h"


#if defined(UX_DEVICE_STANDALONE)

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_printer_write_run                   PORTABLE C     */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yajun Xia, Microsoft Corporation                                    */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function writes to the Printer class.                          */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    printer                               Address of printer class      */
/*                                            instance                    */
/*    buffer                                Pointer to data to write      */
/*    requested_length                      Length of bytes to write,     */
/*                                            set to 0 to issue ZLP       */
/*    actual_length                         Pointer to save number of     */
/*                                            bytes written               */
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
/*  10-31-2022         Yajun Xia            Initial Version 6.2.0         */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_printer_write_run(UX_DEVICE_CLASS_PRINTER *printer, UCHAR *buffer,
                                ULONG requested_length, ULONG *actual_length)
{

UX_SLAVE_ENDPOINT           *endpoint;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_TRANSFER           *transfer_request;
UINT                        zlp = UX_FALSE;
UINT                        status = 0;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PRINTER_WRITE, printer, buffer, requested_length, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

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
        printer -> ux_device_class_printer_write_state = UX_STATE_RESET;
        printer -> ux_device_class_printer_write_status = UX_CONFIGURATION_HANDLE_UNKNOWN;

        return(UX_STATE_EXIT);
    }

    /* Locate the endpoints.  */
    endpoint = printer -> ux_device_class_printer_endpoint_in;

    /* Check if it's available.  */
    if (endpoint == UX_NULL)
    {
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ENDPOINT_HANDLE_UNKNOWN);

        printer -> ux_device_class_printer_write_state = UX_STATE_RESET;
        printer -> ux_device_class_printer_write_status = UX_ENDPOINT_HANDLE_UNKNOWN;

        return(UX_STATE_EXIT);
    }

    /* We are writing to the IN endpoint.  */
    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

    /* Handle state cases.  */
    switch(printer -> ux_device_class_printer_write_state)
    {
    case UX_STATE_RESET:
        printer -> ux_device_class_printer_write_state = UX_DEVICE_CLASS_PRINTER_WRITE_START;
        printer -> ux_device_class_printer_write_status = UX_TRANSFER_NO_ANSWER;
        printer -> ux_device_class_printer_write_buffer = buffer;
        printer -> ux_device_class_printer_write_requested_length = requested_length;
        printer -> ux_device_class_printer_write_actual_length = 0;
        printer -> ux_device_class_printer_write_host_length = UX_SLAVE_REQUEST_DATA_MAX_LENGTH;
        if (requested_length == 0)
            zlp = UX_TRUE;

        /* Fall through.  */
    case UX_DEVICE_CLASS_PRINTER_WRITE_START:

        /* Get remaining requested length.  */
        requested_length = printer -> ux_device_class_printer_write_requested_length -
                        printer -> ux_device_class_printer_write_actual_length;

        /* There is no remaining, we are done.  */
        if (requested_length == 0 && !zlp)
        {
            *actual_length = printer -> ux_device_class_printer_write_actual_length;
            printer -> ux_device_class_printer_write_state = UX_STATE_RESET;
            printer -> ux_device_class_printer_write_status = UX_SUCCESS;
            return(UX_STATE_NEXT);
        }

        /* Check if we have enough in the local buffer.  */
        if (requested_length > UX_SLAVE_REQUEST_DATA_MAX_LENGTH)

            /* We have too much to transfer.  */
            printer -> ux_device_class_printer_write_transfer_length =
                                            UX_SLAVE_REQUEST_DATA_MAX_LENGTH;

        else
        {

            /* We can proceed with the demanded length.  */
            printer -> ux_device_class_printer_write_transfer_length = requested_length;

#if !defined(UX_DEVICE_CLASS_PRINTER_WRITE_AUTO_ZLP)

            /* Assume expected length and transfer length match.  */
            printer -> ux_device_class_printer_write_host_length = requested_length;
#else

            /* Assume expected more than transfer to let stack append ZLP if needed.  */
            printer -> ux_device_class_printer_write_host_length = UX_SLAVE_REQUEST_DATA_MAX_LENGTH + 1;
#endif
        }


        /* On a out, we copy the buffer to the caller. Not very efficient but it makes the API
           easier.  */
        _ux_utility_memory_copy(transfer_request -> ux_slave_transfer_request_data_pointer, 
                            printer -> ux_device_class_printer_write_buffer,
                            printer -> ux_device_class_printer_write_transfer_length); /* Use case of memcpy is verified. */

        /* Next state.  */
        printer -> ux_device_class_printer_write_state = UX_DEVICE_CLASS_PRINTER_WRITE_WAIT;
        UX_SLAVE_TRANSFER_STATE_RESET(transfer_request);

        /* Fall through.  */
    case UX_DEVICE_CLASS_PRINTER_WRITE_WAIT:

        /* Send the request to the device controller.  */
        status =  _ux_device_stack_transfer_run(transfer_request,
                            printer -> ux_device_class_printer_write_transfer_length,
                            printer -> ux_device_class_printer_write_host_length);

        /* Error case.  */
        if (status < UX_STATE_NEXT)
        {

            printer -> ux_device_class_printer_write_state = UX_STATE_RESET;
            printer -> ux_device_class_printer_write_status =
                transfer_request -> ux_slave_transfer_request_completion_code;
            return(UX_STATE_ERROR);
        }

        /* Success case.  */
        if (status == UX_STATE_NEXT)
        {

            /* Next buffer address.  */
            printer -> ux_device_class_printer_write_buffer +=
                    transfer_request -> ux_slave_transfer_request_actual_length;

            /* Set the length actually received. */
            printer -> ux_device_class_printer_write_actual_length +=
                    transfer_request -> ux_slave_transfer_request_actual_length;

            /* Last transfer status.  */
            printer -> ux_device_class_printer_write_status =
                transfer_request -> ux_slave_transfer_request_completion_code;

            /* Update actual done length.  */
            *actual_length = printer -> ux_device_class_printer_write_actual_length;

            /* Check ZLP case.  */
            if (printer -> ux_device_class_printer_write_requested_length == 0)
            {
                printer -> ux_device_class_printer_write_state = UX_STATE_RESET;
                return(UX_STATE_NEXT);
            }

            /* Next state.  */
            printer -> ux_device_class_printer_write_state = UX_DEVICE_CLASS_PRINTER_WRITE_START;
        }

        /* Keep waiting.  */
        return(UX_STATE_WAIT);

    default: /* Error.  */
        printer -> ux_device_class_printer_write_state = UX_STATE_RESET;
        break;
    }

    /* Error case.  */
    return(UX_STATE_EXIT);
}

#endif
