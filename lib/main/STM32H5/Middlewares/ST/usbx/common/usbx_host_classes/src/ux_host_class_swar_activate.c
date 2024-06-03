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
/*    _ux_host_class_swar_activate                        PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function calls the USBX stack to activate the class.           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Dpump class command pointer   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_swar_configure           Configure swar class        */ 
/*    _ux_host_class_swar_endpoints_get       Get endpoints of swar       */ 
/*    _ux_host_stack_class_instance_create    Create class instance       */ 
/*    _ux_host_stack_class_instance_destroy   Destroy the class instance  */ 
/*    _ux_utility_memory_allocate             Allocate memory block       */ 
/*    _ux_utility_memory_free                 Free memory block           */ 
/*    _ux_host_semaphore_create               Create swar semaphore       */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_swar_entry            Entry of swar class            */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_swar_activate(UX_HOST_CLASS_COMMAND *command)
{

UX_DEVICE                           *device;
UX_HOST_CLASS_SWAR                  *swar;
UINT                                status;
    

    /* The Sierra Wireless class is always activated by the device descriptor. */
    device =  (UX_DEVICE *) command -> ux_host_class_command_container;

    /* Obtain memory for this class instance.  */
    swar =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS_SWAR));
    if (swar == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Store the class container into this instance.  */
    swar -> ux_host_class_swar_class =  command -> ux_host_class_command_class_ptr;

    /* Store the device container into the swar class instance.  */
    swar -> ux_host_class_swar_device =  device;

    /* Store the instance in the device container, this is for the USBX stack
       when it needs to invoke the class for deactivation.  */        
    device -> ux_device_class_instance =  (VOID *) swar;

    /* Create this class instance.  */
    _ux_host_stack_class_instance_create(swar -> ux_host_class_swar_class, (VOID *) swar);

    /* Configure the swar.  */
    status =  _ux_host_class_swar_configure(swar);

    /* Get the swar endpoint(s). We will need to search for Bulk Out and Bulk In endpoints on interface .  */
    if (status == UX_SUCCESS)
        status =  _ux_host_class_swar_endpoints_get(swar);

    /* Create the semaphore to protect 2 threads from accessing the same swar instance.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_host_semaphore_create(&swar -> ux_host_class_swar_semaphore, "ux_host_class_swar_semaphore", 1);
        if (status != UX_SUCCESS)
            status = UX_SEMAPHORE_ERROR;
    }

    /* Success things.  */
    if (status == UX_SUCCESS)
    {

        /* Mark the swar as live now.  */
        swar -> ux_host_class_swar_state =  UX_HOST_CLASS_INSTANCE_LIVE;

        /* If all is fine and the device is mounted, we may need to inform the application
        if a function has been programmed in the system structure.  */
        if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
        {

            /* Call system change function.  */
            _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, swar -> ux_host_class_swar_class, (VOID *) swar);
        }

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_SWAR_ACTIVATE, swar, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* If trace is enabled, register this object.  */
        UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, swar, 0, 0, 0)

        /* Return success.  */
        return(UX_SUCCESS);
    }

    /* There was a problem during the configuration, so free the resources.  */
    /* The last resource, semaphore is not created or created error, no need to free.  */
    _ux_host_stack_class_instance_destroy(swar -> ux_host_class_swar_class, (VOID *) swar);
    device -> ux_device_class_instance = UX_NULL;
    _ux_utility_memory_free(swar);

    /* Return completion status.  */
    return(status);    
}

