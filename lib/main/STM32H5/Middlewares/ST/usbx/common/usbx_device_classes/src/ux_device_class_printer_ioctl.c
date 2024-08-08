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
/*    _ux_device_class_printer_ioctl                      PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function performs certain functions on the printer instance    */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    printer                               Address of printer class      */
/*                                            instance                    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Status                                                              */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    None                                                                */
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
UINT _ux_device_class_printer_ioctl(UX_DEVICE_CLASS_PRINTER *printer, ULONG ioctl_function,
                                    VOID *parameter)
{

UINT                                status;
UX_SLAVE_ENDPOINT                   *endpoint;
UX_SLAVE_TRANSFER                   *transfer;


    /* Default to success.  */
    status = UX_SUCCESS;

    /* The command request will tell us what we need to do here.  */
    switch (ioctl_function)
    {
    case UX_DEVICE_CLASS_PRINTER_IOCTL_PORT_STATUS_SET:

        /* Set port status.  */
        printer -> ux_device_class_printer_port_status = (ULONG)(UCHAR)(ALIGN_TYPE)parameter;
        break;

    case UX_DEVICE_CLASS_PRINTER_IOCTL_READ_TIMEOUT_SET:

        /* Set endpoint transfer timeout.  */
        endpoint = printer -> ux_device_class_printer_endpoint_out;
        transfer = &endpoint -> ux_slave_endpoint_transfer_request;
        transfer -> ux_slave_transfer_request_timeout = (ULONG)(ALIGN_TYPE)parameter;
        break;

    case UX_DEVICE_CLASS_PRINTER_IOCTL_WRITE_TIMEOUT_SET:

        /* Set endpoint transfer timeout (if exist).  */
        endpoint = printer -> ux_device_class_printer_endpoint_in;
        if (endpoint != UX_NULL)
        {
            transfer = &endpoint -> ux_slave_endpoint_transfer_request;
            transfer -> ux_slave_transfer_request_timeout = (ULONG)(ALIGN_TYPE)parameter;
        }
        break;

    default:

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Function not supported. Return an error.  */
        status =  UX_FUNCTION_NOT_SUPPORTED;
        break;
    }

    /* Return status to caller.  */
    return(status);

}
