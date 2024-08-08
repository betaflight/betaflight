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
/*    _ux_device_class_printer_deactivate                 PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function deactivate an instance of the printer class.          */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                               Pointer to a class command    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    (ux_device_class_printer_instance_deactivate)                       */
/*                                          Notify deactivation           */
/*    _ux_device_class_printer_soft_reset   Performs software reset       */
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_printer_deactivate(UX_SLAVE_CLASS_COMMAND *command)
{

UX_DEVICE_CLASS_PRINTER     *printer;
UX_SLAVE_CLASS              *printer_class;

    /* Get the class container.  */
    printer_class =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    printer = (UX_DEVICE_CLASS_PRINTER *) printer_class -> ux_slave_class_instance;

    /* Terminate the transactions pending on the endpoints.  */
    _ux_device_class_printer_soft_reset(printer);

    /* Endpoints not ready.  */
    printer -> ux_device_class_printer_endpoint_in = UX_NULL;
    printer -> ux_device_class_printer_endpoint_out = UX_NULL;

    /* Reset port status.  */
    printer -> ux_device_class_printer_port_status =  0;

    /* If there is a deactivate function call it.  */
    if (printer -> ux_device_class_printer_parameter.ux_device_class_printer_instance_deactivate != UX_NULL)
    {

        /* Invoke the application.  */
        printer -> ux_device_class_printer_parameter.ux_device_class_printer_instance_deactivate(printer);
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PRINTER_DEACTIVATE, printer, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_UNREGISTER(printer);

    /* Return completion status.  */
    return(UX_SUCCESS);
}
