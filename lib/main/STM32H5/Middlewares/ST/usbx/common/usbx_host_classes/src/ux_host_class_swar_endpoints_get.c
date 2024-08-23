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
/**   Host Sierra Wireless AR module class                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_swar.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_swar_endpoints_get                   PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function search for the handle of the bulk out and bulk in     */
/*    endpoints. The Sierra Wireless USB device has multiple interfaces.  */ 
/*    We first need to find the interface which uses Bulk endpoints to    */ 
/*    carry data.                                                         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    swar                                     Pointer to swar class      */ 
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
/*    _ux_host_class_swar_activate             Activate swar class        */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_swar_endpoints_get(UX_HOST_CLASS_SWAR *swar)
{

UINT            status;
UX_ENDPOINT     *endpoint;
UINT            endpoint_index;

    /* Search the bulk OUT endpoint. It is attached to the interface container.  */
    for (endpoint_index = 0; endpoint_index < swar -> ux_host_class_swar_interface -> ux_interface_descriptor.bNumEndpoints;
                        endpoint_index++)
    {                        

        /* Get interface endpoint.  */
        status =  _ux_host_stack_interface_endpoint_get(swar -> ux_host_class_swar_interface, endpoint_index, &endpoint);

        /* Check the completion status.  */
        if (status == UX_SUCCESS)
        {

            /* Check if endpoint is bulk and OUT.  */
            if (((endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_OUT) &&
                ((endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT))
            {

                /* This transfer_request always have the OUT direction.  */
                endpoint -> ux_endpoint_transfer_request.ux_transfer_request_type =  UX_REQUEST_OUT;

                /* We have found the bulk endpoint, save it.  */
                swar -> ux_host_class_swar_bulk_out_endpoint =  endpoint;
                break;
            }
        }                
    }            

    /* The bulk out endpoint is mandatory.  */
    if (swar -> ux_host_class_swar_bulk_out_endpoint == UX_NULL)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ENDPOINT_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_ENDPOINT_HANDLE_UNKNOWN, swar, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_ENDPOINT_HANDLE_UNKNOWN);
    }
            
    /* Search the bulk IN endpoint. It is attached to the interface container.  */

    for (endpoint_index = 0; endpoint_index < swar -> ux_host_class_swar_interface -> ux_interface_descriptor.bNumEndpoints;
                        endpoint_index++)
    {                        

        /* Get the endpoint handle.  */
        status =  _ux_host_stack_interface_endpoint_get(swar -> ux_host_class_swar_interface, endpoint_index, &endpoint);

        /* Check the completion status.  */
        if (status == UX_SUCCESS)
        {

            /* Check if endpoint is bulk and IN.  */
            if (((endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN) &&
                ((endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT))
            {

                /* This transfer_request always have the IN direction.  */
                endpoint -> ux_endpoint_transfer_request.ux_transfer_request_type =  UX_REQUEST_IN;

                /* We have found the bulk endpoint, save it.  */
                swar -> ux_host_class_swar_bulk_in_endpoint =  endpoint;
                break;
            }
        }                
    }    

    /* The bulk in endpoint is mandatory.  */
    if (swar -> ux_host_class_swar_bulk_in_endpoint == UX_NULL)
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ENDPOINT_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_ENDPOINT_HANDLE_UNKNOWN, swar, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_ENDPOINT_HANDLE_UNKNOWN);
    }
    
    /* All endpoints have been mounted.  */
    return(UX_SUCCESS);
}

