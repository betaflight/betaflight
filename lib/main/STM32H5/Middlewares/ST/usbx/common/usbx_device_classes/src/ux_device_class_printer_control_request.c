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
/*    _ux_device_class_printer_control_request            PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function manages the requests sent by the host on the control  */
/*    endpoint with a CLASS or VENDOR SPECIFIC type.                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                               Pointer to class command      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    (ux_device_class_printer_soft_reset)  Notify a software reset       */
/*    _ux_device_class_printer_soft_reset   Performs software reset       */
/*    _ux_device_stack_transfer_request     Transfer request              */
/*    _ux_utility_short_get                 Get short value from buffer   */
/*    _ux_utility_short_get_big_endian      Get big endian short value    */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Device Printer Class                                                */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_printer_control_request(UX_SLAVE_CLASS_COMMAND *command)
{
UX_DEVICE_CLASS_PRINTER                 *printer;
UX_SLAVE_CLASS                          *printer_class;
UX_SLAVE_INTERFACE                      *printer_interface;
UX_SLAVE_TRANSFER                       *transfer_request;
UX_SLAVE_DEVICE                         *device;
ULONG                                   request;
ULONG                                   value;
ULONG                                   request_length;
ULONG                                   length;
UCHAR                                   *buffer;
UCHAR                                   index_low;
UCHAR                                   *descriptor;
ULONG                                   descriptor_length;
ULONG                                   index;
UCHAR                                   found;

    /* Get the class container.  */
    printer_class =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    printer = (UX_DEVICE_CLASS_PRINTER *) printer_class -> ux_slave_class_instance;

    /* Get the interface.  */
    printer_interface = printer_class -> ux_slave_class_interface;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get the pointer to the transfer request associated with the control endpoint.  */
    transfer_request =  &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

    /* Extract all necessary fields of the request.  */
    request =  *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);

    /* Extract all necessary fields of the value.  */
    value =  _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_VALUE);
    index_low = transfer_request -> ux_slave_transfer_request_setup[UX_SETUP_INDEX];

    /* Pickup the request length.  */
    request_length =   _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_LENGTH);

    /* Get buffer to fill.  */
    buffer = transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Here we proceed only the standard request we know of at the device level.  */
    switch (request)
    {
    case UX_DEVICE_CLASS_PRINTER_GET_DEVICE_ID:

        /* Sanity check
           - wValue == config index
           - wIndex_high == interface (already checked before entry, index of
                                       interface should equal to interface number)
           - wIndex_low == alternate  */
        if (index_low != (UCHAR)printer_interface -> ux_slave_interface_descriptor.bAlternateSetting)
            return(UX_ERROR);

        /* Check config index.  */
        descriptor = _ux_system_slave -> ux_system_slave_device_framework;
        length = _ux_system_slave->ux_system_slave_device_framework_length;
        index = 0;
        found = 0;
        while(length)
        {

            /* By default use bLength @ 0.  */
            descriptor_length = descriptor[0];

            /* Check bDescriptorType @ 1.  */
            switch(descriptor[1])
            {
            case UX_CONFIGURATION_DESCRIPTOR_ITEM:

                /* It's configuration requested?  */
                if (index == value)
                {

                    /* Check if bConfigurationValue @ 5 is as expected.  */
                    if ((UCHAR)printer_class -> ux_slave_class_configuration_number != descriptor[5])
                        return(UX_ERROR);

                    /* Find the configuration.  */
                    found = 1;
                    break;
                }

                /* Target to next configuration index.  */
                index ++;

                /* Use wTotalLength @ 2.  */
                descriptor_length = _ux_utility_short_get(descriptor + 2);
                break;

            default:
                break;
            }

            /* Break if found.  */
            if (found)
                break;

            /* Descriptor broken check.  */
            if (descriptor_length < 2 ||
                descriptor_length > length)
            {

                /* Error trap!  */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);
                return(UX_DESCRIPTOR_CORRUPTED);
            }

            /* Next descriptor.  */
            descriptor += descriptor_length;
            length -= descriptor_length;
        }

        /* Request parameter not expected if not found.  */
        if (!found)
            return(UX_ERROR);

        /* Length of data (first two bytes in big endian).  */
        length = _ux_utility_short_get_big_endian(printer ->
                        ux_device_class_printer_parameter.ux_device_class_printer_device_id);

        /* Prepare device ID to send.  */
        if (length)
        {

            /* Sanity check.  */
            if (length > UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH)
            {

                /* Error trap!  */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_BUFFER_OVERFLOW);
                return(UX_BUFFER_OVERFLOW);
            }

            /* Copy data.  */
            _ux_utility_memory_copy(buffer, printer ->
                        ux_device_class_printer_parameter.ux_device_class_printer_device_id,
                        length); /* Use case of memcpy is verified. */
        }

        /* Set the phase of the transfer to data out.  */
        transfer_request -> ux_slave_transfer_request_phase =  UX_TRANSFER_PHASE_DATA_OUT;

        /* Transfer.  */
        _ux_device_stack_transfer_request(transfer_request, length, request_length);
        break ;

    case UX_DEVICE_CLASS_PRINTER_GET_PORT_STATUS:

        /* Sanity check.  */
        if (request_length < 1)
            return(UX_ERROR);

        /* Fill status byte.  */
        *buffer = (UCHAR)printer -> ux_device_class_printer_port_status;

        /* Set the phase of the transfer to data out.  */
        transfer_request -> ux_slave_transfer_request_phase = UX_TRANSFER_PHASE_DATA_OUT;

        /* Perform the data transfer.  */
        _ux_device_stack_transfer_request(transfer_request, 1, request_length);
        break;

    case UX_DEVICE_CLASS_PRINTER_SOFT_RESET:
        _ux_device_class_printer_soft_reset(printer);

        /* If there is a soft reset function call it.  */
        if (printer -> ux_device_class_printer_parameter.
                                ux_device_class_printer_soft_reset != UX_NULL)
        {

            /* Invoke the application callback.  */
            printer -> ux_device_class_printer_parameter.
                                ux_device_class_printer_soft_reset(printer);
        }
        break;

    default:

        /* Unknown function. It's not handled.  */
        return(UX_ERROR);
    }

    /* It's handled.  */
    return(UX_SUCCESS);
}
