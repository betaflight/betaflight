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
/**   CDC ECM Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_cdc_ecm.h"
#include "ux_host_stack.h"

UX_HOST_CLASS_CDC_ECM_NX_ETHERNET_POOL_ALLOCSIZE_ASSERT

#if !defined(UX_HOST_STANDALONE)
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_cdc_ecm_activate                     PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates the cdc_ecm instance, configure the device.   */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                             CDC ECM class command pointer   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request          Transfer request           */
/*    _ux_host_class_cdc_ecm_endpoints_get     Get endpoints of cdc_ecm   */ 
/*    _ux_host_class_cdc_ecm_mac_address_get   Get MAC address */
/*    _ux_host_stack_class_instance_create     Create class instance      */ 
/*    _ux_host_stack_class_instance_destroy    Destroy the class instance */ 
/*    _ux_utility_memory_allocate              Allocate memory block      */ 
/*    _ux_utility_memory_free                  Free memory block          */ 
/*    _ux_host_semaphore_create                Create semaphore           */
/*    _ux_host_semaphore_delete                Delete semaphore           */
/*    _ux_utility_thread_create                Create thread              */
/*    _ux_utility_thread_delete                Delete thread              */
/*    _ux_utility_thread_resume                Resume thread              */
/*    _ux_network_driver_activate              Activate NetX USB interface*/
/*    nx_packet_pool_create                    Create NetX packet pool    */
/*    nx_packet_pool_delete                    Delete NetX packet pool    */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_cdc_ecm_entry             Entry of cdc_ecm class     */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  02-02-2021     Xiuwen Cai               Modified comment(s), added    */
/*                                            compile option for using    */
/*                                            packet pool from NetX,      */
/*                                            resulting in version 6.1.4  */
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
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            deprecated ECM pool option, */
/*                                            supported NX packet chain,  */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_cdc_ecm_activate(UX_HOST_CLASS_COMMAND *command)
{

UX_INTERFACE                        *interface_ptr;
UX_HOST_CLASS_CDC_ECM               *cdc_ecm;
UINT                                status;
UX_TRANSFER                         *transfer_request;
ULONG                               physical_address_msw = 0;
ULONG                               physical_address_lsw = 0;
UX_INTERFACE                        *control_interface;
UX_INTERFACE                        *cur_interface;

    /* The CDC ECM class is always activated by the interface descriptor and not the
       device descriptor.  */
    interface_ptr =  (UX_INTERFACE *) command -> ux_host_class_command_container;

    /* Is this the control interface?  */
    if (interface_ptr -> ux_interface_descriptor.bInterfaceClass == UX_HOST_CLASS_CDC_CONTROL_CLASS)
    {

        /* We ignore the control interface. All activation is performed when
           we receive the data interface.  */
        return(UX_SUCCESS);
    }

    /* Obtain memory for this class instance.  */
    cdc_ecm =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, sizeof(UX_HOST_CLASS_CDC_ECM));
    if (cdc_ecm == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Store the class container into this instance.  */
    cdc_ecm -> ux_host_class_cdc_ecm_class =  command -> ux_host_class_command_class_ptr;

    /* Store the device container into the cdc_ecm class instance.  */
    cdc_ecm -> ux_host_class_cdc_ecm_device =  interface_ptr -> ux_interface_configuration -> ux_configuration_device;

    /* Store the interface container into the cdc_acm class instance.  */
    cdc_ecm -> ux_host_class_cdc_ecm_interface_data =  interface_ptr;

    /* We need to link the data and control interfaces together. In order
       to do this, we first need to find the control interface. Per the spec, 
       it should be behind this one.  */

    /* Set the current interface to the second interface. */
    cur_interface =  interface_ptr -> ux_interface_configuration -> ux_configuration_first_interface;

    /* Initialize to null. */
    control_interface =  UX_NULL;

    /* Loop through all the interfaces until we find the current data interface.  */
    while (cur_interface != interface_ptr)
    {

        /* Is this a control interface?  */
        if (cur_interface -> ux_interface_descriptor.bInterfaceClass == UX_HOST_CLASS_CDC_CONTROL_CLASS)
        {

            /* Save it.  */
            control_interface =  cur_interface;
        }

        /* Advance current interface.  */
        cur_interface =  cur_interface -> ux_interface_next_interface;
    }

    /* Did we not find the control interface?  */
    if (control_interface == UX_NULL)
    {

        /* This in an invalid descriptor.  */

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Return error.  */
        status =  UX_DESCRIPTOR_CORRUPTED;
    }
    else
    {

        /* We found the control interface.  */
        status =  UX_SUCCESS;
    }

    if (status == UX_SUCCESS)
    {

        /* Save the control interface.  */
        cdc_ecm -> ux_host_class_cdc_ecm_interface_control =  (UX_INTERFACE *) control_interface;

        /* Get the cdc_ecm endpoint(s) on the interface. */
        status =  _ux_host_class_cdc_ecm_endpoints_get(cdc_ecm);
    }

    if (status == UX_SUCCESS)
    {

        /* Allocate a Thread stack.  */
        cdc_ecm -> ux_host_class_cdc_ecm_thread_stack =  
                    _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_THREAD_STACK_SIZE);
        if (cdc_ecm -> ux_host_class_cdc_ecm_thread_stack == UX_NULL)
            status =  UX_MEMORY_INSUFFICIENT;
    }

    if (status == UX_SUCCESS)
    {

        /* Create the semaphore for aborting bulk in transfers.  */
        status =  _ux_host_semaphore_create(&cdc_ecm -> ux_host_class_cdc_ecm_bulk_in_transfer_waiting_for_check_and_arm_to_finish_semaphore, 
                                               "host CDC-ECM bulk in wait semaphore", 0);
        if (status == UX_SUCCESS)
        {

            /* Create the semaphore for aborting bulk out transfers.  */
            status =  _ux_host_semaphore_create(&cdc_ecm -> ux_host_class_cdc_ecm_bulk_out_transfer_waiting_for_check_and_arm_to_finish_semaphore, 
                                                   "host CDC-ECM bulk out wait semaphore", 0);
            if (status == UX_SUCCESS)
            {

                /* Create the semaphore to wake up the CDC ECM thread.  */
                status =  _ux_host_semaphore_create(&cdc_ecm -> ux_host_class_cdc_ecm_interrupt_notification_semaphore, "host CDC-ECM interrupt notification semaphore", 0);
                if (status == UX_SUCCESS)
                {

                    /* Create the cdc_ecm class thread. We do not start it yet.  */
                    status =  _ux_utility_thread_create(&cdc_ecm -> ux_host_class_cdc_ecm_thread,
                                            "ux_host_cdc_ecm_thread", _ux_host_class_cdc_ecm_thread,
                                            (ULONG) (ALIGN_TYPE) cdc_ecm, 
                                            cdc_ecm -> ux_host_class_cdc_ecm_thread_stack,
                                            UX_THREAD_STACK_SIZE, 
                                            UX_THREAD_PRIORITY_CLASS,
                                            UX_THREAD_PRIORITY_CLASS,
                                            UX_NO_TIME_SLICE, UX_DONT_START);
                    if (status == UX_SUCCESS)
                    {

                        UX_THREAD_EXTENSION_PTR_SET(&(cdc_ecm -> ux_host_class_cdc_ecm_thread), cdc_ecm)

                        /* We now need to retrieve the MAC address of the node which is embedded in the ECM descriptor.
                            We will parse the entire configuration descriptor of the device and look for the ECM Ethernet Networking Functional Descriptor.  */ 
                        status =  _ux_host_class_cdc_ecm_mac_address_get(cdc_ecm);

                        if (status == UX_SUCCESS)
                        {

                            /* Setup the physical address of this IP instance.  */
                            physical_address_msw =  (ULONG)((cdc_ecm -> ux_host_class_cdc_ecm_node_id[0] << 8) | (cdc_ecm -> ux_host_class_cdc_ecm_node_id[1]));
                            physical_address_lsw =  (ULONG)((cdc_ecm -> ux_host_class_cdc_ecm_node_id[2] << 24) | (cdc_ecm -> ux_host_class_cdc_ecm_node_id[3] << 16) | 
                                                                                (cdc_ecm -> ux_host_class_cdc_ecm_node_id[4] << 8) | (cdc_ecm -> ux_host_class_cdc_ecm_node_id[5]));

                            /* The ethernet link is down by default.  */
                            cdc_ecm -> ux_host_class_cdc_ecm_link_state =  UX_HOST_CLASS_CDC_ECM_LINK_STATE_DOWN;
                        }

                        if (status == UX_SUCCESS)
                        {

                            /* Register this interface to the NetX USB interface broker.  */
                            status =  _ux_network_driver_activate((VOID *) cdc_ecm, _ux_host_class_cdc_ecm_write, 
                                                                    &cdc_ecm -> ux_host_class_cdc_ecm_network_handle, 
                                                                    physical_address_msw, physical_address_lsw);
                        }

                        if (status == UX_SUCCESS)
                        {

                            /* Mark the cdc_ecm data instance as live now.  */
                            cdc_ecm -> ux_host_class_cdc_ecm_state =  UX_HOST_CLASS_INSTANCE_LIVE;

                            /* This instance of the device must also be stored in the interface container.  */
                            interface_ptr -> ux_interface_class_instance =  (VOID *) cdc_ecm;

                            /* Create this class instance.  */
                            _ux_host_stack_class_instance_create(cdc_ecm -> ux_host_class_cdc_ecm_class, (VOID *) cdc_ecm);

                            /* Start the interrupt pipe now if it exists.  */
                            if (cdc_ecm -> ux_host_class_cdc_ecm_interrupt_endpoint != UX_NULL)
                            {

                                /* Obtain the transfer request from the interrupt endpoint.  */
                                transfer_request =  &cdc_ecm -> ux_host_class_cdc_ecm_interrupt_endpoint -> ux_endpoint_transfer_request;
                                status =  _ux_host_stack_transfer_request(transfer_request);
                            }

                            if (status == UX_SUCCESS)
                            {

                                /* Activation is complete.  */

                                /* Now we can start the CDC-ECM thread.  */
                                _ux_utility_thread_resume(&cdc_ecm -> ux_host_class_cdc_ecm_thread);

                                /* We need to inform the application if a function has been programmed 
                                    in the system structure. */
                                if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
                                {
                                    
                                    /* Call system change function. Note that the application should
                                        wait until the link state is up until using this instance. The
                                        link state is changed to up by the CDC-ECM thread, which isn't
                                        started until after the data interface has been processed.  */
                                    _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, cdc_ecm -> ux_host_class_cdc_ecm_class, (VOID *) cdc_ecm);
                                }
                            
                                /* If trace is enabled, insert this event into the trace buffer.  */
                                UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_CDC_ECM_ACTIVATE, cdc_ecm, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

                                /* If trace is enabled, register this object.  */
                                UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, cdc_ecm, 0, 0, 0)

                                /* Activation was successful.  */
                                return(UX_SUCCESS);
                            }

                            /* Error starting interrupt endpoint.  */

                            /* Destroy this class instance.  */
                            _ux_host_stack_class_instance_destroy(cdc_ecm -> ux_host_class_cdc_ecm_class, (VOID *) cdc_ecm);

                            /* Unmount instance.  */
                            interface_ptr -> ux_interface_class_instance =  UX_NULL;
                        }

                        /* Delete CDC-ECM thread.  */
                        _ux_utility_thread_delete(&cdc_ecm -> ux_host_class_cdc_ecm_thread);
                    }

                    /* Delete interrupt notification semaphore.  */
                    _ux_host_semaphore_delete(&cdc_ecm -> ux_host_class_cdc_ecm_interrupt_notification_semaphore);
                }

                /* Delete class-level bulk out semaphore.  */
                _ux_host_semaphore_delete(&cdc_ecm -> ux_host_class_cdc_ecm_bulk_out_transfer_waiting_for_check_and_arm_to_finish_semaphore);
            }

            /* Delete class-level bulk in semaphore.  */
            _ux_host_semaphore_delete(&cdc_ecm -> ux_host_class_cdc_ecm_bulk_in_transfer_waiting_for_check_and_arm_to_finish_semaphore);
        }
    }

    /* An error occurred. We must clean up resources.  */

    if (cdc_ecm -> ux_host_class_cdc_ecm_interrupt_endpoint != UX_NULL &&
        cdc_ecm -> ux_host_class_cdc_ecm_interrupt_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer != UX_NULL)
        _ux_utility_memory_free(cdc_ecm -> ux_host_class_cdc_ecm_interrupt_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer);

    if (cdc_ecm -> ux_host_class_cdc_ecm_thread_stack != UX_NULL)
        _ux_utility_memory_free(cdc_ecm -> ux_host_class_cdc_ecm_thread_stack);

    _ux_utility_memory_free(cdc_ecm);
    
    /* Return completion status.  */
    return(status);    
}
#endif
