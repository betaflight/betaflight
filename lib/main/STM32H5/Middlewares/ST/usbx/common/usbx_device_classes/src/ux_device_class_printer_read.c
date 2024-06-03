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


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_printer_read                       PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function reads from the Printer class.                         */
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
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_transfer_request     Transfer request              */
/*    _ux_utility_memory_copy               Copy memory                   */
/*    _ux_device_mutex_off                  Release mutex                 */
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
UINT _ux_device_class_printer_read(UX_DEVICE_CLASS_PRINTER *printer, UCHAR *buffer,
                                ULONG requested_length, ULONG *actual_length)
{

UX_SLAVE_ENDPOINT           *endpoint;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_TRANSFER           *transfer_request;
UINT                        status= UX_SUCCESS;
ULONG                       local_requested_length;

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
        return(UX_CONFIGURATION_HANDLE_UNKNOWN);
    }

    /* Locate the endpoint.  */
    endpoint = printer -> ux_device_class_printer_endpoint_out;

    /* Protect this thread.  */
    _ux_device_mutex_on(&printer -> ux_device_class_printer_endpoint_out_mutex);

    /* All Printer reading  are on the endpoint OUT, from the host.  */
    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

    /* Reset the actual length.  */
    *actual_length =  0;

    /* Check if we need more transactions.  */
    while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED && requested_length != 0)
    {

        /* Check if we have enough in the local buffer.  */
        if (requested_length > endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize)

            /* We have too much to transfer.  */
            local_requested_length = endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize;

        else

            /* We can proceed with the demanded length.  */
            local_requested_length = requested_length;

        /* Send the request to the device controller.  */
        status =  _ux_device_stack_transfer_request(transfer_request,
                                local_requested_length, local_requested_length);

        /* Check the status */
        if (status == UX_SUCCESS)
        {

            /* We need to copy the buffer locally.  */
            _ux_utility_memory_copy(buffer,
                    transfer_request -> ux_slave_transfer_request_data_pointer,
                    transfer_request -> ux_slave_transfer_request_actual_length); /* Use case of memcpy is verified. */

            /* Next buffer address.  */
            buffer += transfer_request -> ux_slave_transfer_request_actual_length;

            /* Set the length actually received. */
            *actual_length += transfer_request -> ux_slave_transfer_request_actual_length;

            /* Decrement what left has to be done.  */
            requested_length -= transfer_request -> ux_slave_transfer_request_actual_length;

            /* Is this a short packet or a ZLP indicating we are done with this transfer ?  */
            if (transfer_request -> ux_slave_transfer_request_actual_length <
                endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize)
            {

                /* We are done.  */
                /* Free Mutex resource.  */
                _ux_device_mutex_off(&printer -> ux_device_class_printer_endpoint_out_mutex);

                /* Return with success.  */
                return(UX_SUCCESS);
            }
        }
        else
        {

            /* Free Mutex resource.  */
            _ux_device_mutex_off(&printer -> ux_device_class_printer_endpoint_out_mutex);

            /* We got an error.  */
            return(status);
        }
    }

    /* Free Mutex resource.  */
    _ux_device_mutex_off(&printer -> ux_device_class_printer_endpoint_out_mutex);

    /* Check why we got here, either completion or device was extracted.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_NO_ANSWER);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_NO_ANSWER, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Device must have been extracted.  */
        return (UX_TRANSFER_NO_ANSWER);
    }
    else

        /* Simply return the last transaction result.  */
        return(status);
}
