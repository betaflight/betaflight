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
/*    _ux_host_class_gser_activate                        PORTABLE C      */ 
/*                                                           6.1.11       */
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
/*    _ux_host_class_gser_configure           Configure gser class        */ 
/*    _ux_host_class_gser_endpoints_get       Get endpoints of gser       */ 
/*    _ux_host_stack_class_instance_create    Create class instance       */ 
/*    _ux_host_stack_class_instance_destroy   Destroy the class instance  */ 
/*    _ux_utility_memory_allocate             Allocate memory block       */ 
/*    _ux_utility_memory_free                 Free memory block           */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_gser_entry            Entry of gser class            */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_gser_activate(UX_HOST_CLASS_COMMAND *command)
{

UX_DEVICE                           *device;
UX_HOST_CLASS_GSER                  *gser;
UINT                                status;


    /* The Generic Modem class is always activated by the device descriptor. */
    device =  (UX_DEVICE *) command -> ux_host_class_command_container;

    /* Obtain memory for this class instance.  */
    gser =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS_GSER));
    if (gser == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Store the class container into this instance.  */
    gser -> ux_host_class_gser_class =  command -> ux_host_class_command_class_ptr;

    /* Store the device container into the gser class instance.  */
    gser -> ux_host_class_gser_device =  device;
  
    /* Store the instance in the device container, this is for the USBX stack
       when it needs to invoke the class for deactivation.  */        
    device -> ux_device_class_instance =  (VOID *) gser;

    /* Create this class instance.  */
    _ux_host_stack_class_instance_create(gser -> ux_host_class_gser_class, (VOID *) gser);

    /* Configure the gser class.  */
    status =  _ux_host_class_gser_configure(gser);

    /* Get the gser endpoint(s). We will need to search for Bulk Out and Bulk In endpoints on each interface .  */
    if (status == UX_SUCCESS)
        status =  _ux_host_class_gser_endpoints_get(gser);

    /* Success things.  */
    if (status == UX_SUCCESS)
    {

        /* Mark the gser as live now.  */
        gser -> ux_host_class_gser_state =  UX_HOST_CLASS_INSTANCE_LIVE;

        /* If all is fine and the device is mounted, we may need to inform the application
        if a function has been programmed in the system structure.  */
        if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
        {
            
            /* Call system change function.  */
            _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, gser -> ux_host_class_gser_class, (VOID *) gser);
        }

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_GSER_ACTIVATE, gser, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)
    
        /* If trace is enabled, register this object.  */
        UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, gser, 0, 0, 0)

        /* Return success.  */
        return(UX_SUCCESS);
    }

    /* Destroy class instance.  */
    _ux_host_stack_class_instance_destroy(gser -> ux_host_class_gser_class, (VOID *) gser);

    /* Clear the instance in the device container.  */
    device -> ux_device_class_instance = UX_NULL;

    /* Free instance memory.  */
    _ux_utility_memory_free(gser);

    /* Return completion status.  */
    return(status);
}

