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
/*    _ux_device_stack_interface_delete                   PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function deletes an interface. Semaphore and memory are        */
/*    released and the controller driver is invoked to disable the        */
/*    hardware endpoint.  The interface is then removed from the          */ 
/*    configuration.                                                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    interface                             Pointer to interface          */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    (ux_slave_dcd_function)               DCD dispatch function         */ 
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
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_stack_interface_delete(UX_SLAVE_INTERFACE *interface_ptr)
{

UX_SLAVE_DCD            *dcd;
UX_SLAVE_DEVICE         *device;
UX_SLAVE_ENDPOINT       *endpoint;
UX_SLAVE_ENDPOINT       *next_endpoint;

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_UNREGISTER(interface_ptr);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_STACK_INTERFACE_DELETE, interface_ptr, 0, 0, 0, UX_TRACE_DEVICE_STACK_EVENTS, 0, 0)

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Find the first endpoints associated with this interface.  */    
    next_endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;        
    
    /* Parse all the endpoints.  */    
    while (next_endpoint != UX_NULL)
    {

        /* Save this endpoint.  */
        endpoint =  next_endpoint;
        
        /* Find the next endpoint.  */
        next_endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
        
        /* Get the pointer to the DCD.  */
        dcd =  &_ux_system_slave->ux_system_slave_dcd;

        /* The endpoint must be destroyed.  */
        dcd -> ux_slave_dcd_function(dcd, UX_DCD_DESTROY_ENDPOINT, endpoint);

        /* Free the endpoint.  */
        endpoint -> ux_slave_endpoint_status =  UX_UNUSED;

        /* Make sure the endpoint instance is now cleaned up.  */
        endpoint -> ux_slave_endpoint_state =  0;
        endpoint -> ux_slave_endpoint_next_endpoint =  UX_NULL;
        endpoint -> ux_slave_endpoint_interface =  UX_NULL;
        endpoint -> ux_slave_endpoint_device =  UX_NULL;
    }        

    /* It's always from first one (to delete).  */
    /* Rebuild the first link.  */
    device -> ux_slave_device_first_interface =  interface_ptr -> ux_slave_interface_next_interface;

    /* The interface is removed from the link, its memory must be cleaned and returned to the pool.  */
    interface_ptr -> ux_slave_interface_class          =  UX_NULL;
    interface_ptr -> ux_slave_interface_class_instance =  UX_NULL;
    interface_ptr -> ux_slave_interface_next_interface =  UX_NULL;
    interface_ptr -> ux_slave_interface_first_endpoint =  UX_NULL;
    interface_ptr -> ux_slave_interface_status         =  UX_UNUSED;

    /* Return successful completion.  */    
    return(UX_SUCCESS);       
}

