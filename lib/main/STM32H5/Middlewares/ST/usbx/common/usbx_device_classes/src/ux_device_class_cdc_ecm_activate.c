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
/**   Device CDC_ECM Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_cdc_ecm.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_cdc_ecm_activate                   PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function activates the USB CDC_ECM device.                     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                           Pointer to cdc_ecm command        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
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
/*                                            cases, used UX prefix to    */
/*                                            refer to TX symbols instead */
/*                                            of using them directly,     */
/*                                            resulting in version 6.1    */
/*  08-02-2021     Wen Wang                 Modified comment(s),          */
/*                                            fixed spelling error,       */
/*                                            resulting in version 6.1.8  */
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
UINT  _ux_device_class_cdc_ecm_activate(UX_SLAVE_CLASS_COMMAND *command)
{
#if defined(UX_DEVICE_STANDALONE)
    UX_PARAMETER_NOT_USED(command);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_SLAVE_INTERFACE          *interface_ptr;            
UX_SLAVE_CLASS              *class_ptr;
UX_SLAVE_CLASS_CDC_ECM      *cdc_ecm;
UX_SLAVE_ENDPOINT           *endpoint;
ULONG                       physical_address_msw;
ULONG                       physical_address_lsw;

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    cdc_ecm = (UX_SLAVE_CLASS_CDC_ECM *) class_ptr -> ux_slave_class_instance;

    /* Get the interface that owns this instance.  */
    interface_ptr =  (UX_SLAVE_INTERFACE  *) command -> ux_slave_class_command_interface;
    
    /* Check if this is the Control or Data interface.  */
    if (command -> ux_slave_class_command_class == UX_DEVICE_CLASS_CDC_ECM_CLASS_COMMUNICATION_CONTROL)
    {

        /* Store the class instance into the interface.  */
        interface_ptr -> ux_slave_interface_class_instance =  (VOID *)cdc_ecm;
         
        /* Now the opposite, store the interface in the class instance.  */
        cdc_ecm -> ux_slave_class_cdc_ecm_interface =  interface_ptr;
        
        /* Locate the interrupt endpoint. */
        endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;
    
        /* Parse all endpoints.  */
        while (endpoint != UX_NULL)
        {
    
            /* Check the endpoint direction, and type.  */
            if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN)
            {

                /* Look at type.  */
                if ((endpoint -> ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_INTERRUPT_ENDPOINT)
                {                    
        
                    /* We have found the interrupt endpoint, save it.  */
                    cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_endpoint =  endpoint;

                    /* Reset the endpoint buffers.  */
                    _ux_utility_memory_set(cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_endpoint -> ux_slave_endpoint_transfer_request. 
                                        ux_slave_transfer_request_data_pointer, 0, UX_SLAVE_REQUEST_DATA_MAX_LENGTH); /* Use case of memset is verified. */

                    /* Resume the interrupt endpoint threads.  */
                    _ux_device_thread_resume(&cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_thread); 

                }
                
            }        

            /* Next endpoint.  */
            endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
        }
        
    }
    else

        /* This is the DATA Class, only store the cdc_ecm instance in the interface.  */
        interface_ptr -> ux_slave_interface_class_instance =  (VOID *)cdc_ecm;

    /* Reset the CDC ECM alternate setting to 0.  */
    cdc_ecm -> ux_slave_class_cdc_ecm_current_alternate_setting =  0;

    /* Check if this is the Control or Data interface.  */
    if (command -> ux_slave_class_command_class == UX_DEVICE_CLASS_CDC_ECM_CLASS_COMMUNICATION_DATA)
    {

        /* Reset endpoint instance pointers.  */
        cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_endpoint = UX_NULL;
        cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_endpoint = UX_NULL;

        /* Does the data class have bulk endpoint declared ? If yes we need to start link.
           If not, the host will change the alternate setting at a later stage.  */
        if (interface_ptr -> ux_slave_interface_descriptor.bNumEndpoints != 0)
        {   

            /* Locate the endpoints.  Control and Bulk in/out for Data Interface.  */
            endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;
        
            /* Parse all endpoints.  */
            while (endpoint != UX_NULL)
            {
            
                /* Check the endpoint direction, and type.  */
                if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN)
                {
        
                    /* Look at type.  */
                    if ((endpoint -> ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT)
                
                        /* We have found the bulk in endpoint, save it.  */
                        cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_endpoint =  endpoint;
                        
                }
                else
                {
                    /* Look at type for out endpoint.  */
                    if ((endpoint -> ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT)
                
                        /* We have found the bulk out endpoint, save it.  */
                        cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_endpoint =  endpoint;
                }                
        
                /* Next endpoint.  */
                endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
            }
    
    
            /* Now check if all endpoints have been found.  */
            if (cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_endpoint == UX_NULL || cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_endpoint == UX_NULL)
    
                /* Not all endpoints have been found. Major error, do not proceed.  */
                return(UX_ERROR);

            /* Declare the link to be up. That may need to change later to make it dependent on the
               WAN/Wireless modem.  */
            cdc_ecm -> ux_slave_class_cdc_ecm_link_state = UX_DEVICE_CLASS_CDC_ECM_LINK_STATE_UP;
            
            /* Wake up the Interrupt thread and send a network notification to the host.  */
            _ux_device_event_flags_set(&cdc_ecm -> ux_slave_class_cdc_ecm_event_flags_group, UX_DEVICE_CLASS_CDC_ECM_NETWORK_NOTIFICATION_EVENT, UX_OR);                

            /* Reset the endpoint buffers.  */
            _ux_utility_memory_set(cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_endpoint -> ux_slave_endpoint_transfer_request. 
                                            ux_slave_transfer_request_data_pointer, 0, UX_SLAVE_REQUEST_DATA_MAX_LENGTH); /* Use case of memset is verified. */
            _ux_utility_memory_set(cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_endpoint -> ux_slave_endpoint_transfer_request. 
                                            ux_slave_transfer_request_data_pointer, 0, UX_SLAVE_REQUEST_DATA_MAX_LENGTH); /* Use case of memset is verified. */

            /* Resume the endpoint threads.  */
            _ux_device_thread_resume(&cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_thread); 
            _ux_device_thread_resume(&cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_thread); 

        }
        
        /* Setup the physical address of this IP instance.  */
        physical_address_msw =  (ULONG)((cdc_ecm -> ux_slave_class_cdc_ecm_local_node_id[0] << 8) | (cdc_ecm -> ux_slave_class_cdc_ecm_local_node_id[1]));
        physical_address_lsw =  (ULONG)((cdc_ecm -> ux_slave_class_cdc_ecm_local_node_id[2] << 24) | (cdc_ecm -> ux_slave_class_cdc_ecm_local_node_id[3] << 16) | 
                                                       (cdc_ecm -> ux_slave_class_cdc_ecm_local_node_id[4] << 8) | (cdc_ecm -> ux_slave_class_cdc_ecm_local_node_id[5]));
            
        /* Register this interface to the NetX USB interface broker.  */
        _ux_network_driver_activate((VOID *) cdc_ecm, _ux_device_class_cdc_ecm_write,
                                        &cdc_ecm -> ux_slave_class_cdc_ecm_network_handle,
                                        physical_address_msw,
                                        physical_address_lsw);
                                        
        /* Check Link.  */
        if (cdc_ecm -> ux_slave_class_cdc_ecm_link_state == UX_DEVICE_CLASS_CDC_ECM_LINK_STATE_UP)
        {

            /* Communicate the state with the network driver.  */
            _ux_network_driver_link_up(cdc_ecm -> ux_slave_class_cdc_ecm_network_handle);

            /* If there is an activate function call it.  */
            if (cdc_ecm -> ux_slave_class_cdc_ecm_parameter.ux_slave_class_cdc_ecm_instance_activate != UX_NULL)

                /* Invoke the application.  */
                cdc_ecm -> ux_slave_class_cdc_ecm_parameter.ux_slave_class_cdc_ecm_instance_activate(cdc_ecm);
        }
    }        

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_CDC_ECM_ACTIVATE, cdc_ecm, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_DEVICE_OBJECT_TYPE_INTERFACE, cdc_ecm, 0, 0, 0)

    /* Return completion status.  */
    return(UX_SUCCESS);
#endif
}
