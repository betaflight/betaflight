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
/**   Generic Serial Host module class                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_gser.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_gser_deactivate                      PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called when this instance of the gser has been     */
/*    removed from the bus either directly or indirectly. The bulk in\out */ 
/*    pipes will be destroyed and the instanced removed.                  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                                  Swar class command pointer */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_class_instance_destroy Destroy the class instance    */ 
/*    _ux_host_stack_endpoint_transfer_abort                              */
/*                                          Abort endpoint transfer       */ 
/*    _ux_utility_memory_free               Free memory block             */ 
/*    _ux_host_semaphore_delete             Delete protection semaphore   */ 
/*    _ux_utility_thread_schedule_other     Schedule other threads        */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_gser_entry                Entry of gser class        */ 
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
UINT  _ux_host_class_gser_deactivate(UX_HOST_CLASS_COMMAND *command)
{

UX_HOST_CLASS_GSER          *gser;
ULONG                       interface_index;

    /* Get the instance for this class.  */
    gser =  (UX_HOST_CLASS_GSER *) command -> ux_host_class_command_instance;

    /* The gser class is being shut down.  */
    gser -> ux_host_class_gser_state =  UX_HOST_CLASS_INSTANCE_SHUTDOWN;

    for (interface_index = 0; interface_index < UX_HOST_CLASS_GSER_INTERFACE_NUMBER; interface_index++)
    {

        /* We need to abort transactions on the bulk Out pipes.  */
        _ux_host_stack_endpoint_transfer_abort(gser -> ux_host_class_gser_interface_array[interface_index].ux_host_class_gser_bulk_out_endpoint);

        /* We need to abort transactions on the bulk In pipes.  */
        _ux_host_stack_endpoint_transfer_abort(gser -> ux_host_class_gser_interface_array[interface_index].ux_host_class_gser_bulk_in_endpoint);

        /* The enumeration thread needs to sleep a while to allow the application or the class that may be using
           endpoints to exit properly.  */
        _ux_host_thread_schedule_other(UX_THREAD_PRIORITY_ENUM); 

        /* Destroy the semaphore.  */
        _ux_host_semaphore_delete(&gser -> ux_host_class_gser_interface_array[interface_index].ux_host_class_gser_semaphore);

    }
    
    /* Destroy the instance.  */
    _ux_host_stack_class_instance_destroy(gser -> ux_host_class_gser_class, (VOID *) gser);

    /* Before we free the device resources, we need to inform the application
        that the device is removed.  */
    if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
    {
        
        /* Inform the application the device is removed.  */
        _ux_system_host -> ux_system_host_change_function(UX_DEVICE_REMOVAL, gser -> ux_host_class_gser_class, (VOID *) gser);
    }
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_GSER_DEACTIVATE, gser, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_UNREGISTER(gser);

    /* Free the gser instance memory.  */
    _ux_utility_memory_free(gser);

    /* Return successful status.  */
    return(UX_SUCCESS);         
}

