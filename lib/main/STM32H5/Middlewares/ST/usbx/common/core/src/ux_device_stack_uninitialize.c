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
/**   Device Stack                                                        */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_stack.h"

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_stack_uninitialize                       PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function uninitializes the generic portion of the device side  */
/*    of USBX.                                                            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_free               Free                          */ 
/*    _ux_utility_semaphore_delete          Delete semaphore              */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed uninitialize in case  */
/*                                            there is no EP except EP0,  */
/*                                            resulting in version 6.1.6  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_stack_uninitialize(VOID)
{
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_ENDPOINT               *endpoints_pool;
UX_SLAVE_TRANSFER               *transfer_request;
ULONG                           endpoints_found;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_STACK_INITIALIZE, 0, 0, 0, 0, UX_TRACE_DEVICE_STACK_EVENTS, 0, 0)

    /* Get the pointer to the device. */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Free class memory. */
    _ux_utility_memory_free(_ux_system_slave -> ux_system_slave_class_array);

    /* Allocate some memory for the Control Endpoint.  First get the address of the transfer request for the 
       control endpoint. */
    transfer_request =  &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

    /* Free memory for the control endpoint buffer.  */
    _ux_utility_memory_free(transfer_request -> ux_slave_transfer_request_data_pointer);

    /* Get the number of endpoints found in the device framework.  */
    endpoints_found = device -> ux_slave_device_endpoints_pool_number;
    
    /* Get the endpoint pool address in the device container.  */
    endpoints_pool =  device -> ux_slave_device_endpoints_pool;

    /* Parse all endpoints and fee memory and semaphore. */
    while (endpoints_found-- != 0)
    {
        /* Free the memory for endpoint data pointer.  */
        _ux_utility_memory_free(endpoints_pool -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer);
    
        /* Remove the TX semaphore for the endpoint.  */
        _ux_device_semaphore_delete(&endpoints_pool -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_semaphore);
    
        /* Next endpoint.  */
        endpoints_pool++;
    }

    /* Free the endpoint pool address in the device container.  */
    if (device -> ux_slave_device_endpoints_pool)
        _ux_utility_memory_free(device -> ux_slave_device_endpoints_pool);

    /* Free memory for interface pool.  */
    _ux_utility_memory_free(device -> ux_slave_device_interfaces_pool);

    /* Return successful completion.  */
    return(UX_SUCCESS);
}




