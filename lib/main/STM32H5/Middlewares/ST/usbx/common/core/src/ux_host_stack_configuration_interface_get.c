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
/*    _ux_host_stack_configuration_interface_get          PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function returns an interface container based on a             */ 
/*    configuration handle, an interface index and an alternate setting   */ 
/*    index.                                                              */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    configuration                         Pointer to configuration      */ 
/*    interface_index                       Index of interface            */ 
/*    alternate_setting_index               Index of alternate setting    */ 
/*    interface                             Destination of interface      */ 
/*                                            pointer                     */ 
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
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_configuration_interface_get(UX_CONFIGURATION *configuration, 
                                                UINT interface_index, UINT alternate_setting_index,
                                                UX_INTERFACE **ux_interface)
{
    
UINT                current_interface_number;
UINT                container_index;
UX_INTERFACE        *current_interface;


    /* Do a sanity check on the configuration handle.  */
    if (configuration -> ux_configuration_handle != (ULONG) (ALIGN_TYPE) configuration)
    {

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONFIGURATION_HANDLE_UNKNOWN, configuration, 0, 0, UX_TRACE_ERRORS, 0, 0)
    
        return(UX_CONFIGURATION_HANDLE_UNKNOWN);
    }
            
    /* Start with the interface attached to the configuration.  */
    current_interface =  configuration -> ux_configuration_first_interface;

    /* The first interface has the index 0 */    
    container_index =  0;
    
    /* Reset the interface number */
    current_interface_number =  0;
    
    /* Traverse the list of the interfaces until we found the right one */        
    while (current_interface != UX_NULL)
    {

        /* Check if the interface index matches the current one.  */
        if (interface_index == container_index)
        {

            /* We have found the correct interface, now search for the alternate setting.  */
            current_interface_number =  current_interface -> ux_interface_descriptor.bInterfaceNumber;

            /* The first alternate setting has the index 0.  */    
            container_index =  0;

            /* Loop on all the alternate settings for this interface.  */
            while (current_interface != UX_NULL)
            {

                /* Check if the index is matched */
                if (alternate_setting_index == container_index)
                {

                    /* We have found the right interface/alternate setting combination. Set the
                       interface return pointer.  */
                    *ux_interface =  current_interface;

                    /* Return success to caller.  */
                    return(UX_SUCCESS);
                }

                /* Move to next alternate setting index.  */
                container_index++;

                /* Move to the next alternate setting.   */
                current_interface =  current_interface -> ux_interface_next_interface;

    
                /* Check new interface pointer, might be the end.  */
                if (current_interface != UX_NULL)
                {
    
                    /* And verify that we are still in the same interface.  */
                    if (current_interface -> ux_interface_descriptor.bInterfaceNumber != current_interface_number)
                    {

                        /* Error trap. */
                        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_INTERFACE_HANDLE_UNKNOWN);

                        /* If trace is enabled, insert this event into the trace buffer.  */
                        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_INTERFACE_HANDLE_UNKNOWN, ux_interface, 0, 0, UX_TRACE_ERRORS, 0, 0)

                        return(UX_INTERFACE_HANDLE_UNKNOWN);
                    }                    
                }
            }       
        }
        
        /* Check the current interface, we may already be at the end ... */
        if (current_interface != UX_NULL)
        {
        
            /* Move to the next interface.  */
            current_interface =  current_interface -> ux_interface_next_interface;
        
            /* Move to the next interface index. */
            container_index++;
        }            
    }
    
    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_INTERFACE_HANDLE_UNKNOWN);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_INTERFACE_HANDLE_UNKNOWN, ux_interface, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* Didn't find the right interface/alternate setting, return an error!  */
    return(UX_INTERFACE_HANDLE_UNKNOWN);
}

