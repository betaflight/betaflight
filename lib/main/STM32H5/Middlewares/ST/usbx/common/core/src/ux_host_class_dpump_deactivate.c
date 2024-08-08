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
/**   Host Data Pump Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_dpump.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_dpump_deactivate                     PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called when this instance of the dpump has been    */
/*    removed from the bus either directly or indirectly. The bulk in\out */ 
/*    pipes will be destroyed and the instanced removed.                  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                             Data Pump class command pointer */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_class_instance_destroy Destroy the class instance    */ 
/*    _ux_host_stack_endpoint_transfer_abort Abort endpoint transfer      */ 
/*    _ux_utility_memory_free               Free memory block             */ 
/*    _ux_utility_semaphore_get             Get protection semaphore      */ 
/*    _ux_utility_semaphore_delete          Delete protection semaphore   */ 
/*    _ux_utility_thread_sleep              Sleep thread                  */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_dpump_entry              Entry of dpump class        */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_dpump_deactivate(UX_HOST_CLASS_COMMAND *command)
{

UX_HOST_CLASS_DPUMP         *dpump;
UINT                        status;


    /* Get the instance for this class.  */
    dpump =  (UX_HOST_CLASS_DPUMP *) command -> ux_host_class_command_instance;

    /* The dpump is being shut down.  */
    dpump -> ux_host_class_dpump_state =  UX_HOST_CLASS_INSTANCE_SHUTDOWN;

    /* Protect thread reentry to this instance.  */
    status =  _ux_host_semaphore_get(&dpump -> ux_host_class_dpump_semaphore, UX_WAIT_FOREVER);
    if (status != UX_SUCCESS)
    {        

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, dpump, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }
    
    /* We need to abort transactions on the bulk Out pipe.  */
    _ux_host_stack_endpoint_transfer_abort(dpump -> ux_host_class_dpump_bulk_out_endpoint);

    /* We need to abort transactions on the bulk In pipe.  */
    _ux_host_stack_endpoint_transfer_abort(dpump -> ux_host_class_dpump_bulk_in_endpoint);

    /* If the class instance was busy, let it finish properly and not return.  */
    _ux_host_thread_sleep(UX_ENUMERATION_THREAD_WAIT); 

    /* Destroy the instance.  */
    _ux_host_stack_class_instance_destroy(dpump -> ux_host_class_dpump_class, (VOID *) dpump);

    /* Destroy the semaphore.  */
    _ux_host_semaphore_delete(&dpump -> ux_host_class_dpump_semaphore);

    /* Before we free the device resources, we need to inform the application
        that the device is removed.  */
    if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
    {
        
        /* Inform the application the device is removed.  */
        _ux_system_host -> ux_system_host_change_function(UX_DEVICE_REMOVAL, dpump -> ux_host_class_dpump_class, (VOID *) dpump);
    }
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_DPUMP_DEACTIVATE, dpump, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_UNREGISTER(dpump);

    /* Free the dpump instance memory.  */
    _ux_utility_memory_free(dpump);

    /* Return successful status.  */
    return(UX_SUCCESS);         
}

