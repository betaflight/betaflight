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
/**   HUB Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hub.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hub_activate                         PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs the enumeration of the HUB. The HUB          */ 
/*    descriptor is read, the interrupt endpoint activated, power is set  */ 
/*    to the downstream ports and the HUB instance will be awaken when    */ 
/*    there is a status change on the HUB or one of the ports.            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Pointer to command            */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_hub_configure          Configure HUB                 */ 
/*    _ux_host_class_hub_descriptor_get     Get descriptor                */ 
/*    _ux_host_class_hub_interrupt_endpoint_start                         */
/*                                          Start interrupt endpoint      */ 
/*    _ux_host_class_hub_ports_power        Power ports                   */ 
/*    _ux_host_stack_class_instance_create  Create class instance         */ 
/*    _ux_host_stack_class_instance_destroy Destroy class instance        */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Free memory block             */
/*    _ux_host_semaphore_create             Create semaphore              */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HUB Class                                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            optimized based on compile  */
/*                                            definitions,                */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hub_activate(UX_HOST_CLASS_COMMAND *command)
{

UX_DEVICE           *device;
UX_HOST_CLASS_HUB   *hub;
UINT                status;


#if UX_MAX_DEVICES > 1
    /* We need to make sure that the enumeration thread knows about at least
       one active HUB instance and the function to call when the thread
       is awaken.  */
    _ux_system_host -> ux_system_host_enum_hub_function =  _ux_host_class_hub_change_detect;
#endif

    /* The HUB is always activated by the device descriptor and not the
       instance descriptor.  */
    device =  (UX_DEVICE *) command -> ux_host_class_command_container;

    /* Instantiate this HUB class.  */
    hub =  (UX_HOST_CLASS_HUB *) _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS_HUB));
    if (hub == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);
        
    /* Store the class container into this instance.  */
    hub -> ux_host_class_hub_class =  command -> ux_host_class_command_class_ptr;

    /* Store the device container instance in the HUB instance, this is for 
       the class instance when it needs to talk to the USBX stack.  */
    hub -> ux_host_class_hub_device =  device;

#if defined(UX_HOST_STANDALONE)

    /* Store the instance in the device container, this is for the USBX stack
        when it needs to invoke the class.  */        
    device -> ux_device_class_instance =  (VOID *) hub;

    /* Store the hub interface.  */
    hub -> ux_host_class_hub_interface = device ->
            ux_device_first_configuration -> ux_configuration_first_interface;

    /* Store the class task function.  */
    hub -> ux_host_class_hub_class -> ux_host_class_task_function = _ux_host_class_hub_tasks_run;

    /* During activation and tasks, control transfer is used for requests.  */
    hub -> ux_host_class_hub_transfer = &device -> ux_device_control_endpoint.ux_endpoint_transfer_request;

    /* The HUB is configured and activated in ACTIVATE_WAIT.  */
    hub -> ux_host_class_hub_run_status = UX_SUCCESS;
    hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_GET_STATUS;
    status = UX_SUCCESS;
#else

    /* Configure the HUB.  */
    status =  _ux_host_class_hub_configure(hub);     
    if (status == UX_SUCCESS)
    {

        /* Get the HUB descriptor.  */
        status =  _ux_host_class_hub_descriptor_get(hub);        
        if (status == UX_SUCCESS)
        {

            /* Power up the HUB downstream ports. This function always returns
               success since we may be dealing with multiple ports.  */            
            _ux_host_class_hub_ports_power(hub);

            /* Search the HUB interrupt endpoint and start it.  */
            status =  _ux_host_class_hub_interrupt_endpoint_start(hub);      
            if (status == UX_SUCCESS)
            {

                /* Create this class instance.  */
                _ux_host_stack_class_instance_create(hub -> ux_host_class_hub_class, (VOID *) hub);
                
                /* Store the instance in the device container, this is for the USBX stack
                   when it needs to invoke the class.  */        
                device -> ux_device_class_instance =  (VOID *) hub;

                /* Mark the HUB as live now.  */
                hub -> ux_host_class_hub_state =  UX_HOST_CLASS_INSTANCE_LIVE;

                /* If all is fine and the device is mounted, we may need to inform the application
                   if a function has been programmed in the system structure.  */
                if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
                {
                    
                    /* Call system change function.  */
                    _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, hub -> ux_host_class_hub_class, (VOID *) hub);
                }

                /* Return success.  */
                return(UX_SUCCESS);
            }
        }
    }

    /* We get here when an error occurred.  */

    /* Free the hub instance.  */
    _ux_utility_memory_free(hub);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HUB_ACTIVATE, hub, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, hub, 0, 0, 0)

#endif

    /* Return completion status.  */
    return(status);    
}
