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
/*    _ux_host_class_gser_configure                       PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function calls the USBX stack to do a SET_CONFIGURATION to the */
/*    gser. Once the gser is configured, its interface will be            */ 
/*    activated. The bulk endpoints enumerated(1 IN, 1 OUT ).             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    gser                                   Pointer to gser class        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_configuration_interface_get  Get interface           */ 
/*    _ux_host_stack_device_configuration_get     Get configuration       */ 
/*    _ux_host_stack_device_configuration_select  Select configuration    */ 
/*    _ux_host_semaphore_create                   Create semaphore        */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_gser_activate                gser class activate     */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_gser_configure(UX_HOST_CLASS_GSER *gser)
{

UINT                    status;
UX_CONFIGURATION        *configuration;
ULONG                   interface_index;
#if UX_MAX_DEVICES > 1
UX_DEVICE               *parent_device;
#endif


    /* If the device has been configured already, we don't need to do it
       again. */
    if (gser -> ux_host_class_gser_device -> ux_device_state == UX_DEVICE_CONFIGURED)
        return(UX_SUCCESS);

    /* A gser normally has one configuration. So retrieve the 1st configuration
       only.  */
    status =  _ux_host_stack_device_configuration_get(gser -> ux_host_class_gser_device, 0, &configuration);
    if (status != UX_SUCCESS)
    {
    
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CONFIGURATION_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONFIGURATION_HANDLE_UNKNOWN, gser -> ux_host_class_gser_device, 0, 0, UX_TRACE_ERRORS, 0, 0)
    
        return(UX_CONFIGURATION_HANDLE_UNKNOWN);
    }

#if UX_MAX_DEVICES > 1
    /* Check the gser power source and check the parent power source for 
       incompatible connections.  */
    if (gser -> ux_host_class_gser_device -> ux_device_power_source == UX_DEVICE_BUS_POWERED)
    {

        /* Get parent device pointer.  */
        parent_device =  gser -> ux_host_class_gser_device -> ux_device_parent;
        
        /* If the device is NULL, the parent is the root gser and we don't have to worry 
           if the parent is not the root gser, check for its power source.  */
        if ((parent_device != UX_NULL) && (parent_device -> ux_device_power_source == UX_DEVICE_BUS_POWERED))
        {                       

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CONFIGURATION_HANDLE_UNKNOWN);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONNECTION_INCOMPATIBLE, gser, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_CONNECTION_INCOMPATIBLE);
        }            
    }
#endif

    /* We have the valid configuration. Ask the USBX stack to set this configuration.  */        
    status =  _ux_host_stack_device_configuration_select(configuration);
    if (status != UX_SUCCESS)
        return(status);

    /* If the operation went well, the gser default alternate setting for the gser interface is 
       active. We have to scan all interfaces and attach them. Each interface has an semaphore as well for protection. */
    for (interface_index = 0; interface_index < UX_HOST_CLASS_GSER_INTERFACE_NUMBER; interface_index++)
    {

        /* Get that interface.  */
        status =  _ux_host_stack_configuration_interface_get(configuration, interface_index, 0, &gser -> ux_host_class_gser_interface_array[interface_index].ux_host_class_gser_interface);
    
        /* Should not fail. But do a sanity check.  */
        if (status == UX_SUCCESS)
        {

            /* Store the instance in the interface container, this is for the USB stack
               when it needs to invoke the class.  */        
            gser -> ux_host_class_gser_interface_array[interface_index].ux_host_class_gser_interface -> ux_interface_class_instance =  (VOID *) gser;

            /* Store the class container in the interface.  The device has the correct class, duplicate it to the 
               interface.  */
            gser -> ux_host_class_gser_interface_array[interface_index].ux_host_class_gser_interface -> ux_interface_class =  gser -> ux_host_class_gser_device -> ux_device_class ;

            /* Create the semaphore to protect 2 threads from accessing the same gser instance.  */
            status =  _ux_host_semaphore_create(&gser -> ux_host_class_gser_interface_array[interface_index].ux_host_class_gser_semaphore, "ux_host_class_gser_semaphore", 1);

            /* Check status.  */
            if (status != UX_SUCCESS)
                return(UX_SEMAPHORE_ERROR);

        }
    }
    
    /* Return completion status.  */
    return(UX_SUCCESS);
}

