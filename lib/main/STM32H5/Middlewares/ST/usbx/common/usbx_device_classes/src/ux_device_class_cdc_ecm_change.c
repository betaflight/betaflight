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
/*    _ux_device_class_cdc_ecm_change                     PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function changes the interface of the CDC_ECM device           */ 
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
/*    _ux_network_driver_link_up            Link status up                */
/*    _ux_network_driver_link_down          Link status down              */
/*    _ux_utility_memory_set                Set memory                    */
/*    _ux_device_thread_resume              Resume thread                 */
/*    _ux_device_event_flags_set            Set event flags               */
/*    _ux_device_stack_transfer_all_request_abort                         */
/*                                          Abort transfer                */
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
UINT  _ux_device_class_cdc_ecm_change(UX_SLAVE_CLASS_COMMAND *command)
{

UX_SLAVE_INTERFACE                      *interface_ptr;            
UX_SLAVE_CLASS                          *class_ptr;
UX_SLAVE_CLASS_CDC_ECM                  *cdc_ecm;
UX_SLAVE_ENDPOINT                       *endpoint;

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    cdc_ecm = (UX_SLAVE_CLASS_CDC_ECM *) class_ptr -> ux_slave_class_instance;

    /* Get the interface that owns this instance.  */
    interface_ptr =  (UX_SLAVE_INTERFACE  *) command -> ux_slave_class_command_interface;
    
    /* Locate the endpoints.  Control and Bulk in/out for Data Interface.  */
    endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;
    
    /* If the interface to mount has a non zero alternate setting, the class is really active with
       the endpoints active.  If the interface reverts to alternate setting 0, it needs to have
       the pending transactions terminated.  */
    if (interface_ptr -> ux_slave_interface_descriptor.bAlternateSetting != 0)       
    {
    
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

        /* Declare the link to be up. */
        cdc_ecm -> ux_slave_class_cdc_ecm_link_state = UX_DEVICE_CLASS_CDC_ECM_LINK_STATE_UP;
        
        /* Communicate the state with the network driver.  */
        _ux_network_driver_link_up(cdc_ecm -> ux_slave_class_cdc_ecm_network_handle);

        /* Reset the endpoint buffers.  */
        _ux_utility_memory_set(cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_endpoint -> ux_slave_endpoint_transfer_request. 
                                        ux_slave_transfer_request_data_pointer, 0, UX_SLAVE_REQUEST_DATA_MAX_LENGTH); /* Use case of memset is verified. */
        _ux_utility_memory_set(cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_endpoint -> ux_slave_endpoint_transfer_request. 
                                        ux_slave_transfer_request_data_pointer, 0, UX_SLAVE_REQUEST_DATA_MAX_LENGTH); /* Use case of memset is verified. */

        /* Resume the endpoint threads.  */
        _ux_device_thread_resume(&cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_thread); 
        _ux_device_thread_resume(&cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_thread); 
        
        /* Wake up the Interrupt thread and send a network notification to the host.  */
        _ux_device_event_flags_set(&cdc_ecm -> ux_slave_class_cdc_ecm_event_flags_group, UX_DEVICE_CLASS_CDC_ECM_NETWORK_NOTIFICATION_EVENT, UX_OR);                

        /* If there is an activate function call it.  */
        if (cdc_ecm -> ux_slave_class_cdc_ecm_parameter.ux_slave_class_cdc_ecm_instance_activate != UX_NULL)

            /* Invoke the application.  */
            cdc_ecm -> ux_slave_class_cdc_ecm_parameter.ux_slave_class_cdc_ecm_instance_activate(cdc_ecm);
    }                
    else
    {

        /* In this case, we are reverting to the Alternate Setting 0.  */

        /* Declare the link to be down.  */
        cdc_ecm -> ux_slave_class_cdc_ecm_link_state = UX_DEVICE_CLASS_CDC_ECM_LINK_STATE_DOWN;
        
        /* Communicate the state with the network driver.  */
        _ux_network_driver_link_down(cdc_ecm -> ux_slave_class_cdc_ecm_network_handle);

        /* Terminate the transactions pending on the bulk in endpoint.  If there is a transfer on the
           bulk out endpoint, we simply let it finish and let NetX throw it away.  */
        _ux_device_stack_transfer_all_request_abort(cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_endpoint, UX_TRANSFER_APPLICATION_RESET);

        /* Notify the thread waiting for network notification events. In this case,
           the event is that the link state has been switched to down.  */
        _ux_device_event_flags_set(&cdc_ecm -> ux_slave_class_cdc_ecm_event_flags_group, UX_DEVICE_CLASS_CDC_ECM_NETWORK_NOTIFICATION_EVENT, UX_OR);                

        /* Wake up the bulk in thread so that it can clean up the xmit queue.  */
        _ux_device_event_flags_set(&cdc_ecm -> ux_slave_class_cdc_ecm_event_flags_group, UX_DEVICE_CLASS_CDC_ECM_NEW_DEVICE_STATE_CHANGE_EVENT, UX_OR);                

        /* If there is a deactivate function call it.  */
        if (cdc_ecm -> ux_slave_class_cdc_ecm_parameter.ux_slave_class_cdc_ecm_instance_deactivate != UX_NULL)

            /* Invoke the application.  */
            cdc_ecm -> ux_slave_class_cdc_ecm_parameter.ux_slave_class_cdc_ecm_instance_deactivate(cdc_ecm);
    }

    /* Set the CDC ECM alternate setting to the new one.  */
    cdc_ecm -> ux_slave_class_cdc_ecm_current_alternate_setting = interface_ptr -> ux_slave_interface_descriptor.bAlternateSetting;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_CDC_ECM_CHANGE, cdc_ecm, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_DEVICE_OBJECT_TYPE_INTERFACE, cdc_ecm, 0, 0, 0)

    /* Return completion status.  */
    return(UX_SUCCESS);
}

