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
/*    _ux_device_class_printer_activate                   PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function initializes the USB Printer device.                   */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                               Pointer to printer command    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    (ux_device_class_printer_instance_deactivate)                       */
/*                                          Notify activation             */
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
UINT  _ux_device_class_printer_activate(UX_SLAVE_CLASS_COMMAND *command)
{

UX_SLAVE_INTERFACE                      *printer_interface;
UX_SLAVE_CLASS                          *printer_class;
UX_DEVICE_CLASS_PRINTER                 *printer;
UX_SLAVE_ENDPOINT                       *endpoint;

    /* Get the class container.  */
    printer_class = command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    printer = (UX_DEVICE_CLASS_PRINTER *) printer_class -> ux_slave_class_instance;

    /* Get the interface that owns this instance.  */
    printer_interface =  (UX_SLAVE_INTERFACE  *) command -> ux_slave_class_command_interface;

    /* Store the class instance into the interface.  */
    printer_interface -> ux_slave_interface_class_instance =  (VOID *)printer;

    /* Now the opposite, store the interface in the class instance.  */
    printer -> ux_device_class_printer_interface =  printer_interface;

    /* Save endpoints for future use.  */
    printer -> ux_device_class_printer_endpoint_in = UX_NULL;
    printer -> ux_device_class_printer_endpoint_out = UX_NULL;
    endpoint = printer_interface -> ux_slave_interface_first_endpoint;
    while(endpoint)
    {
        if (endpoint -> ux_slave_endpoint_descriptor.bmAttributes == UX_BULK_ENDPOINT)
        {
            if (endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_IN)
            {
                printer -> ux_device_class_printer_endpoint_in = endpoint;
                if (printer -> ux_device_class_printer_endpoint_out)
                    break;
            }
            else
            {
                printer -> ux_device_class_printer_endpoint_out = endpoint;
                if (printer -> ux_device_class_printer_endpoint_in)
                    break;
            }
        }

        /* Next endpoint.  */
        endpoint = endpoint -> ux_slave_endpoint_next_endpoint;
    }

    /* Check error, there must be Bulk OUT.  */
    if (printer -> ux_device_class_printer_endpoint_out == UX_NULL)
        return(UX_DESCRIPTOR_CORRUPTED);

    /* Initialize port status.
        Benign status of "Paper Not Empty", "Selected", and "No Error".  */
    printer -> ux_device_class_printer_port_status = UX_DEVICE_CLASS_PRINTER_SELECT |
                                                     UX_DEVICE_CLASS_PRINTER_NOT_ERROR;

    /* If there is a activate function call it.  */
    if (printer -> ux_device_class_printer_parameter.ux_device_class_printer_instance_activate != UX_NULL)
    {
        /* Invoke the application.  */
        printer -> ux_device_class_printer_parameter.ux_device_class_printer_instance_activate(printer);
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PRINTER_ACTIVATE, printer, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_DEVICE_OBJECT_TYPE_INTERFACE, printer, 0, 0, 0)

    /* Return completion status.  */
    return(UX_SUCCESS);
}

