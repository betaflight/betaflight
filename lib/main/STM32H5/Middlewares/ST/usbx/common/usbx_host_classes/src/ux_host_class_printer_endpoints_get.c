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
/**   Printer Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_printer.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_printer_endpoints_get                PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function search for the handle of the bulk out endpoint and    */
/*    optionally the bulk in endpoint of the printer is bidirectional.    */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    printer                               Pointer to printer class      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_interface_endpoint_get Get interface endpoint        */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_printer_activate       Activate printer class        */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            initialized timeout values, */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_printer_endpoints_get(UX_HOST_CLASS_PRINTER *printer)
{

UINT            status;
UINT            endpoint_index;
UX_ENDPOINT     *endpoint;


    /* Search the bulk OUT endpoint. It is attached to the interface container.  */
    for (endpoint_index = 0; endpoint_index < printer -> ux_host_class_printer_interface -> ux_interface_descriptor.bNumEndpoints;
                        endpoint_index++)
    {                        

        /* Get interface endpoint.  */
        status =  _ux_host_stack_interface_endpoint_get(printer -> ux_host_class_printer_interface, endpoint_index, &endpoint);

        /* Check the completion status.  */
        if (status == UX_SUCCESS)
        {

            /* Check if endpoint is bulk and OUT.  */
            if (((endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_OUT) &&
                ((endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT))
            {

                /* This transfer_request always have the OUT direction.  */
                endpoint -> ux_endpoint_transfer_request.ux_transfer_request_type =  UX_REQUEST_OUT;

                /* By default wait UX_HOST_CLASS_PRINTER_CLASS_TRANSFER_TIMEOUT.  */
                endpoint -> ux_endpoint_transfer_request.ux_transfer_request_timeout_value =
                                UX_MS_TO_TICK(UX_HOST_CLASS_PRINTER_CLASS_TRANSFER_TIMEOUT);

                /* We have found the bulk endpoint, save it.  */
                printer -> ux_host_class_printer_bulk_out_endpoint =  endpoint;
                break;
            }
        }                
    }            

    /* The bulk out endpoint is mandatory.  */
    if (printer -> ux_host_class_printer_bulk_out_endpoint == UX_NULL)
    {

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_ENDPOINT_HANDLE_UNKNOWN, printer, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_ENDPOINT_HANDLE_UNKNOWN);
    }
            
    /* Search the bulk IN endpoint. This endpoint is optional and only valid for
       bidirectional printers. It is attached to the interface container.  */
    if ((printer -> ux_host_class_printer_interface -> ux_interface_descriptor.bInterfaceProtocol == 
                                                UX_HOST_CLASS_PRINTER_PROTOCOL_BI_DIRECTIONAL) ||
        (printer -> ux_host_class_printer_interface -> ux_interface_descriptor.bInterfaceProtocol == 
                                                UX_HOST_CLASS_PRINTER_PROTOCOL_IEEE_1284_4_BI_DIR))
    {                                                        

        for (endpoint_index = 0; endpoint_index < printer -> ux_host_class_printer_interface -> ux_interface_descriptor.bNumEndpoints;
                            endpoint_index++)
        {                        

            /* Get the endpoint handle.  */
            status =  _ux_host_stack_interface_endpoint_get(printer -> ux_host_class_printer_interface, endpoint_index, &endpoint);

            /* Check the completion status.  */
            if (status == UX_SUCCESS)
            {

                /* Check if endpoint is bulk and IN.  */
                if (((endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN) &&
                    ((endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT))
                {

                    /* This transfer_request always have the IN direction.  */
                    endpoint -> ux_endpoint_transfer_request.ux_transfer_request_type =  UX_REQUEST_IN;

                    /* By default wait UX_HOST_CLASS_PRINTER_CLASS_TRANSFER_TIMEOUT.  */
                    endpoint -> ux_endpoint_transfer_request.ux_transfer_request_timeout_value =
                                    UX_MS_TO_TICK(UX_HOST_CLASS_PRINTER_CLASS_TRANSFER_TIMEOUT);

                    /* We have found the bulk endpoint, save it.  */
                    printer -> ux_host_class_printer_bulk_in_endpoint =  endpoint;
                    break;
                }
            }                
        }    

        /* The bulk in endpoint is mandatory for these protocol.  */
        if (printer -> ux_host_class_printer_bulk_in_endpoint == UX_NULL)
        {

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_ENDPOINT_HANDLE_UNKNOWN, printer, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_ENDPOINT_HANDLE_UNKNOWN);
        }
    }            

    /* All endpoints have been mounted.  */
    return(UX_SUCCESS);
}

