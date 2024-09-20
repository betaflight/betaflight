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
/**   Device RNDIS Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_rndis.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_rndis_activate                     PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function activates the USB RNDIS device.                       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                             Pointer to rndis command        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_network_driver_activate         Activate NetX USB interface     */
/*    _ux_utility_memory_set              Set memory                      */
/*    _ux_device_thread_resume            Resume thread                   */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX Source Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_rndis_activate(UX_SLAVE_CLASS_COMMAND *command)
{
#if defined(UX_DEVICE_STANDALONE)
    UX_PARAMETER_NOT_USED(command);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_SLAVE_INTERFACE          *interface_ptr;            
UX_SLAVE_CLASS              *class_ptr;
UX_SLAVE_CLASS_RNDIS        *rndis;
UX_SLAVE_ENDPOINT           *endpoint;
ULONG                       physical_address_msw;
ULONG                       physical_address_lsw;

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    rndis = (UX_SLAVE_CLASS_RNDIS *) class_ptr -> ux_slave_class_instance;

    /* Get the interface that owns this instance.  */
    interface_ptr =  (UX_SLAVE_INTERFACE  *) command -> ux_slave_class_command_interface;
    
    /* Check if this is the Control or Data interface.  */
    if (command -> ux_slave_class_command_class == UX_DEVICE_CLASS_RNDIS_CLASS_COMMUNICATION_CONTROL)
    {

        /* Store the class instance into the interface.  */
        interface_ptr -> ux_slave_interface_class_instance =  (VOID *)rndis;
         
        /* Now the opposite, store the interface in the class instance.  */
        rndis -> ux_slave_class_rndis_interface =  interface_ptr;

        /* If there is a activate function call it.  */
        if (rndis -> ux_slave_class_rndis_parameter.ux_slave_class_rndis_instance_activate != UX_NULL)
        
            /* Invoke the application.  */
            rndis -> ux_slave_class_rndis_parameter.ux_slave_class_rndis_instance_activate(rndis);
    }
    else
    
        /* This is the DATA Class, only store the rndis instance in the interface.  */
        interface_ptr -> ux_slave_interface_class_instance =  (VOID *)rndis;

    /* Locate the endpoints.  Interrupt for Control and Bulk in/out for Data.  */
    endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;
    
    /* Parse all endpoints.  */
    while (endpoint != UX_NULL)
    {
    
        /* Check the endpoint direction, and type.  */
        if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN)
        {

            /* Look at type.  */
            if ((endpoint -> ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_INTERRUPT_ENDPOINT)
        
                /* We have found the interrupt endpoint, save it.  */
                rndis -> ux_slave_class_rndis_interrupt_endpoint =  endpoint;

            else
                            
                /* We have found the bulk in endpoint, save it.  */
                rndis -> ux_slave_class_rndis_bulkin_endpoint =  endpoint;

        }
        else
        {
            /* Look at type for out endpoint.  */
            if ((endpoint -> ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT)
        
                /* We have found the bulk out endpoint, save it.  */
                rndis -> ux_slave_class_rndis_bulkout_endpoint =  endpoint;
        }                

        /* Next endpoint.  */
        endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
    }

    /* Check if this is the Control or Data interface.  */
    if (command -> ux_slave_class_command_class == UX_DEVICE_CLASS_RNDIS_CLASS_COMMUNICATION_DATA)
    {

        /* Now check if all endpoints have been found.  */
        if (rndis -> ux_slave_class_rndis_bulkout_endpoint == UX_NULL || rndis -> ux_slave_class_rndis_bulkin_endpoint == UX_NULL ||
            rndis -> ux_slave_class_rndis_interrupt_endpoint == UX_NULL)

            /* Not all endpoints have been found. Major error, do not proceed.  */
            return(UX_ERROR);

        /* Declare the link to be up. That may need to change later to make it dependent on the
           WAN/Wireless modem.  */
        rndis -> ux_slave_class_rndis_link_state = UX_DEVICE_CLASS_RNDIS_LINK_STATE_UP;

        /* Setup the physical address of this IP instance.  */
        physical_address_msw =  (ULONG)((rndis -> ux_slave_class_rndis_local_node_id[0] << 8) | (rndis -> ux_slave_class_rndis_local_node_id[1]));
        physical_address_lsw =  (ULONG)((rndis -> ux_slave_class_rndis_local_node_id[2] << 24) | (rndis -> ux_slave_class_rndis_local_node_id[3] << 16) | 
                                                       (rndis -> ux_slave_class_rndis_local_node_id[4] << 8) | (rndis -> ux_slave_class_rndis_local_node_id[5]));
            
        /* Register this interface to the NetX USB interface broker.  */
        _ux_network_driver_activate((VOID *) rndis, _ux_device_class_rndis_write,
                                        &rndis -> ux_slave_class_rndis_network_handle,
                                        physical_address_msw,
                                        physical_address_lsw);
                
        /* Reset the endpoint buffers.  */
        _ux_utility_memory_set(rndis -> ux_slave_class_rndis_bulkout_endpoint -> ux_slave_endpoint_transfer_request. 
                                        ux_slave_transfer_request_data_pointer, 0, UX_SLAVE_REQUEST_DATA_MAX_LENGTH); /* Use case of memset is verified. */
        _ux_utility_memory_set(rndis -> ux_slave_class_rndis_bulkin_endpoint -> ux_slave_endpoint_transfer_request. 
                                        ux_slave_transfer_request_data_pointer, 0, UX_SLAVE_REQUEST_DATA_MAX_LENGTH); /* Use case of memset is verified. */
        _ux_utility_memory_set(rndis -> ux_slave_class_rndis_interrupt_endpoint -> ux_slave_endpoint_transfer_request. 
                                        ux_slave_transfer_request_data_pointer, 0, UX_SLAVE_REQUEST_DATA_MAX_LENGTH); /* Use case of memset is verified. */
    
        /* Resume the endpoint threads.  */
        _ux_device_thread_resume(&rndis -> ux_slave_class_rndis_interrupt_thread); 
        _ux_device_thread_resume(&rndis -> ux_slave_class_rndis_bulkout_thread); 
        _ux_device_thread_resume(&rndis -> ux_slave_class_rndis_bulkin_thread); 

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_RNDIS_ACTIVATE, rndis, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)
        
        /* If trace is enabled, register this object.  */
        UX_TRACE_OBJECT_REGISTER(UX_TRACE_DEVICE_OBJECT_TYPE_INTERFACE, rndis, 0, 0, 0)
        
        /* Return completion status.  */
        return(UX_SUCCESS);
         
    }                
    else
    {
    
        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_RNDIS_ACTIVATE, rndis, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)
    
        /* If trace is enabled, register this object.  */
        UX_TRACE_OBJECT_REGISTER(UX_TRACE_DEVICE_OBJECT_TYPE_INTERFACE, rndis, 0, 0, 0)
        
        /* Return completion status.  */
        return(UX_SUCCESS);
    }
#endif
}
