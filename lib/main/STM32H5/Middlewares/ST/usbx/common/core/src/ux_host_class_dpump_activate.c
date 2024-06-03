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
/*    _ux_host_class_dpump_activate                       PORTABLE C      */ 
/*                                                           6.1.12       */
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
/*    _ux_host_class_dpump_configure          Configure dpump class       */ 
/*    _ux_host_class_dpump_endpoints_get      Get endpoints of dpump      */ 
/*    _ux_host_stack_class_instance_create    Create class instance       */ 
/*    _ux_host_stack_class_instance_destroy   Destroy the class instance  */ 
/*    _ux_utility_memory_allocate             Allocate memory block       */ 
/*    _ux_utility_semaphore_create            Create dpump semaphore      */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_dpump_entry          Entry of dpump class            */ 
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
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_dpump_activate(UX_HOST_CLASS_COMMAND *command)
{

UX_INTERFACE                *interface_ptr;
UX_HOST_CLASS_DPUMP         *dpump;
UINT                        status;
    

    /* The data pump is always activated by the interface descriptor and not the
       device descriptor.  */
    interface_ptr =  (UX_INTERFACE *) command -> ux_host_class_command_container;

    /* Obtain memory for this class instance.  */
    dpump =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS_DPUMP));
    if (dpump == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Store the class container into this instance.  */
    dpump -> ux_host_class_dpump_class =  command -> ux_host_class_command_class_ptr;

    /* Store the interface container into the dpump class instance.  */
    dpump -> ux_host_class_dpump_interface =  interface_ptr;

    /* Store the device container into the dpump class instance.  */
    dpump -> ux_host_class_dpump_device =  interface_ptr -> ux_interface_configuration -> ux_configuration_device;

    /* This instance of the device must also be stored in the interface container.  */
    interface_ptr -> ux_interface_class_instance =  (VOID *) dpump;

    /* Create this class instance.  */
    _ux_host_stack_class_instance_create(dpump -> ux_host_class_dpump_class, (VOID *) dpump);

    /* Configure the dpump.  */
    status =  _ux_host_class_dpump_configure(dpump);     
    if (status != UX_SUCCESS)
    {

        _ux_host_stack_class_instance_destroy(dpump -> ux_host_class_dpump_class, (VOID *) dpump);
        return(status);
    }

    /* Get the dpump endpoint(s). We will need to search for Bulk Out and Bulk In endpoints.  Do not check for errors
       here as the alternate setting for this interface may be 0 which has no endpoints.  */
    _ux_host_class_dpump_endpoints_get(dpump);

    /* Create the semaphore to protect 2 threads from accessing the same dpump instance.  */
    status =  _ux_host_semaphore_create(&dpump -> ux_host_class_dpump_semaphore, "ux_dpump_semaphore", 1);
    if (status != UX_SUCCESS)
        return(UX_SEMAPHORE_ERROR);

    /* Mark the dpump as live now.  */
    dpump -> ux_host_class_dpump_state =  UX_HOST_CLASS_INSTANCE_LIVE;

    /* If all is fine and the device is mounted, we may need to inform the application
       if a function has been programmed in the system structure.  */
    if ((status == UX_SUCCESS) && (_ux_system_host -> ux_system_host_change_function != UX_NULL))
    {
        
        /* Call system change function.  */
        _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, dpump -> ux_host_class_dpump_class, (VOID *) dpump);
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_DPUMP_ACTIVATE, dpump, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)
  
    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, dpump, 0, 0, 0)

    /* Return completion status.  */
    return(status);    
}

