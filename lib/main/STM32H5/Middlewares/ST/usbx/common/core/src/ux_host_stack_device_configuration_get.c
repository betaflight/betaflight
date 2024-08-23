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
/**   Host Stack                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_stack_device_configuration_get             PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function returns a configuration container based on a device   */
/*    handle and a configuration index.                                   */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    device                                Pointer to device             */ 
/*    configuration_index                   Index of configuration        */ 
/*    configuration                         Pointer to configuration      */ 
/*                                            destination                 */ 
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
/*    Application                                                         */ 
/*    USBX Components                                                     */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_device_configuration_get(UX_DEVICE *device, UINT configuration_index,
                                                        UX_CONFIGURATION **configuration)
{

UINT                    current_configuration_index;
UX_CONFIGURATION        *current_configuration;

    /* Do a sanity check on the device handle.  */
    if (device -> ux_device_handle != (ULONG) (ALIGN_TYPE) device)
    {
        
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_DEVICE_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DEVICE_HANDLE_UNKNOWN, device, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_DEVICE_HANDLE_UNKNOWN);
    }
        
    /* Start with the configuration attached to the device.  */
    current_configuration =  device -> ux_device_first_configuration;

    /* The first configuration has the index 0.  */    
    current_configuration_index =  0;
    
    /* Traverse the list of the configurations until we found the right one.  */        
    while (current_configuration != UX_NULL)
    {

        /* Check if the configuration index matches the current one.  */
        if (configuration_index == current_configuration_index)
        {

            /* Return the configuration pointer.  */
            *configuration =  current_configuration;

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_DEVICE_CONFIGURATION_GET, device, current_configuration, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

            /* Return successful completion.  */
            return(UX_SUCCESS);
        }
        
        /* Move to the next configuration.  */
        current_configuration =  current_configuration -> ux_configuration_next_configuration;
        
        /* Move to the next index.  */
        current_configuration_index++;
    }
    
    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_CONFIGURATION_HANDLE_UNKNOWN);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONFIGURATION_HANDLE_UNKNOWN, configuration, 0, 0, UX_TRACE_ERRORS, 0, 0)
    
    /* Return an error.  */
    return(UX_CONFIGURATION_HANDLE_UNKNOWN);
}

