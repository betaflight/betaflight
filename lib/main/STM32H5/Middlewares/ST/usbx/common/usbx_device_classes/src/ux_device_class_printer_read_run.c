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
/*    _ux_device_class_printer_read_run                   PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yajun Xia, Microsoft Corporation                                    */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function reads from the Printer class.                         */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    printer                               Address of printer class      */
/*                                            instance                    */
/*    buffer                                Pointer to buffer to save     */
/*                                            received data               */
/*    requested_length                      Length of bytes to read       */
/*    actual_length                         Pointer to save number of     */
/*                                            bytes read                  */
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
/*    _ux_device_stack_transfer_run         Transfer request              */
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
/*  10-31-2022        Yajun xia             Initial Version 6.2.0         */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_printer_read_run(UX_DEVICE_CLASS_PRINTER *printer, UCHAR *buffer,
                                ULONG requested_length, ULONG *actual_length)
{

UX_SLAVE_ENDPOINT           *endpoint;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_TRANSFER           *transfer_request;
ULONG                       max_transfer_length;
UINT                        status = UX_SUCCESS;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PRINTER_READ, printer, buffer, requested_length, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

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
        printer -> ux_device_class_printer_read_state = UX_STATE_RESET;
        printer -> ux_device_class_printer_read_status = UX_CONFIGURATION_HANDLE_UNKNOWN;

        return(UX_STATE_EXIT);
    }

    /* Locate the endpoint.  */
    endpoint = printer -> ux_device_class_printer_endpoint_out;

    /* All Printer reading  are on the endpoint OUT, from the host.  */
    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

    /* Handle state cases.  */
    switch(printer -> ux_device_class_printer_read_state)
    {
    case UX_STATE_RESET:

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PRINTER_READ, printer, buffer, requested_length, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

        printer -> ux_device_class_printer_read_state = UX_DEVICE_CLASS_PRINTER_READ_START;
        printer -> ux_device_class_printer_read_status = UX_TRANSFER_NO_ANSWER;
        printer -> ux_device_class_printer_read_buffer = buffer;
        printer -> ux_device_class_printer_read_requested_length = requested_length;
        printer -> ux_device_class_printer_read_actual_length = 0;

        /* Fall through. */
    case UX_DEVICE_CLASS_PRINTER_READ_START:

        /* Get remaining transfer length.  */
        requested_length = printer -> ux_device_class_printer_read_requested_length -
                        printer -> ux_device_class_printer_read_actual_length;

        /* There is nothing remaining, it's done.  */
        if (requested_length == 0)
        {
            *actual_length = printer -> ux_device_class_printer_read_actual_length;
            printer -> ux_device_class_printer_read_state = UX_STATE_RESET;
            printer -> ux_device_class_printer_read_status = UX_SUCCESS;
            return(UX_STATE_NEXT);
        }

        printer -> ux_device_class_printer_read_transfer_length = requested_length;

        /* Next state.  */
        printer -> ux_device_class_printer_read_state = UX_DEVICE_CLASS_PRINTER_READ_WAIT;
        UX_SLAVE_TRANSFER_STATE_RESET(transfer_request);

        /* Fall through.  */
    case UX_DEVICE_CLASS_PRINTER_READ_WAIT:

        /* Get a full packet each time */
        max_transfer_length = endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize;

        /* Run the transfer state machine.  */
        status = _ux_device_stack_transfer_run(transfer_request,
                                                max_transfer_length,
                                                max_transfer_length);

        /* Error case.  */
        if (status < UX_STATE_NEXT)
        {
            printer -> ux_device_class_printer_read_state = UX_STATE_RESET;
            printer -> ux_device_class_printer_read_status =
                transfer_request -> ux_slave_transfer_request_completion_code;
            return(UX_STATE_ERROR);
        }

        /* Success case.  */
        if (status == UX_STATE_NEXT)
        {
            /* Check overflow */
            if (printer -> ux_device_class_printer_read_transfer_length <
                transfer_request -> ux_slave_transfer_request_actual_length)
            {
                printer -> ux_device_class_printer_read_state = UX_STATE_ERROR;
                printer -> ux_device_class_printer_read_status = UX_TRANSFER_BUFFER_OVERFLOW;
                return(UX_STATE_ERROR);
            }

            /* We need to copy the buffer locally.  */
            _ux_utility_memory_copy(printer -> ux_device_class_printer_read_buffer,
                    transfer_request -> ux_slave_transfer_request_data_pointer,
                    transfer_request -> ux_slave_transfer_request_actual_length); /* Use case of memcpy is verified. */

            /* Next buffer address.  */
            printer -> ux_device_class_printer_read_buffer +=
                    transfer_request -> ux_slave_transfer_request_actual_length;

            /* Set the length actually received. */
            printer -> ux_device_class_printer_read_actual_length +=
                    transfer_request -> ux_slave_transfer_request_actual_length;

            /* Last transfer status.  */
            printer -> ux_device_class_printer_read_status =
                transfer_request -> ux_slave_transfer_request_completion_code;

            /* Update actual length.  */
            *actual_length = printer -> ux_device_class_printer_read_actual_length;

            /* Check short packet.  */
            if (transfer_request -> ux_slave_transfer_request_actual_length <
                transfer_request -> ux_slave_transfer_request_requested_length)
            {

                /* It's done.  */
                printer -> ux_device_class_printer_read_state = UX_STATE_RESET;
                return(UX_STATE_NEXT);
            }

            /* Next state.  */
            printer -> ux_device_class_printer_read_state = UX_DEVICE_CLASS_PRINTER_READ_START;
        }

        /* Keep waiting.  */
        return(UX_STATE_WAIT);

    default: /* Error.  */
        printer -> ux_device_class_printer_read_state = UX_STATE_RESET;
        printer -> ux_device_class_printer_read_status = UX_INVALID_STATE;
        break;
    }

    /* Error cases.  */
    return(UX_STATE_EXIT);
}

#endif
