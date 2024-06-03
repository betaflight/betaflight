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
/*    _ux_host_stack_new_configuration_create             PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates a new configuration for the current device    */
/*    a device can have multiple configurations.                          */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    device                                Pointer to the descriptor     */ 
/*                                            for the device              */
/*    configuration_descriptor              Configuration descriptor      */ 
/*                                            previously parsed           */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
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
VOID  _ux_host_stack_new_configuration_create(UX_DEVICE *device, UX_CONFIGURATION *configuration)
{

UX_CONFIGURATION    *list_configuration;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_NEW_CONFIGURATION_CREATE, device, configuration, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* The device that owns this configuration is memorized in the 
       configuration container itself, easier for back chaining.  */
    configuration -> ux_configuration_device =  device;

    /* Save the configuration handle in the container, this is for ensuring the
       configuration container is not corrupted.  */
    configuration -> ux_configuration_handle =  (ULONG) (ALIGN_TYPE) configuration;

    /* There is 2 cases for the creation of the configuration descriptor 
       if this is the first one, the configuration descriptor is hooked
       to the device. If it is not the first one, the configuration is 
       hooked to the end of the chain of configurations.  */
    if (device -> ux_device_first_configuration == UX_NULL)
    {
        device -> ux_device_first_configuration =  configuration;
    }
    else
    {

        /* Get the pointer to the first configuration.  */
        list_configuration =  device -> ux_device_first_configuration;

        /* And traverse until we have reached the end of the configuration list.  */
        while (list_configuration -> ux_configuration_next_configuration != UX_NULL)
        {
            list_configuration =  list_configuration -> ux_configuration_next_configuration;
        }

        /* Hook the new configuration.  */
        list_configuration -> ux_configuration_next_configuration =  configuration;
    }

    /* Return to caller.  */
    return;
}

