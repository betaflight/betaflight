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
/*    _ux_host_stack_device_configuration_select          PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function selects a specific configuration for a device.        */
/*    When this configuration is set to the device, by default all the    */
/*    device interface and their associated alternate setting 0 is        */ 
/*    activated on the device. If the device/interface class driver       */ 
/*    wishes to change the setting of a particular interface, it needs    */ 
/*    to issue a select interface setting function.                       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    configuration                          Pointer to configuration     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_configuration_instance_create Create configuration   */ 
/*                                                   instance             */  
/*    _ux_host_stack_configuration_instance_delete Delete configuration   */ 
/*                                                   instance             */ 
/*    _ux_host_stack_configuration_set             Set configuration      */ 
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
/*                                            optimized based on compile  */
/*                                            definitions,                */
/*                                            resulting in version 6.1    */
/*  02-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used pointer for current    */
/*                                            selected configuration,     */
/*                                            resulting in version 6.1.4  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refine power usage check,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_device_configuration_select(UX_CONFIGURATION *configuration)
{

UX_DEVICE               *device;
UX_CONFIGURATION        *current_configuration;
UINT                    status;
    
    /* Check for validity of the configuration handle.  */
    if (configuration -> ux_configuration_handle != (ULONG) (ALIGN_TYPE) configuration)
    {
    
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_CONFIGURATION_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONFIGURATION_HANDLE_UNKNOWN, configuration, 0, 0, UX_TRACE_ERRORS, 0, 0)
    
        return(UX_CONFIGURATION_HANDLE_UNKNOWN);
    }

    /* Get the device container for this configuration.  */       
    device =  configuration -> ux_configuration_device;
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_DEVICE_CONFIGURATION_SELECT, device, configuration, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* We need to check the amount of power the bus powered device is consuming
       before switch configuration. Otherwise we may run the risk of
       an over current fault. */
    if (((configuration -> ux_configuration_descriptor.bmAttributes & UX_CONFIGURATION_DEVICE_SELF_POWERED) == 0) &&
         (configuration -> ux_configuration_descriptor.MaxPower > UX_DEVICE_MAX_POWER_GET(device)))
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_OVER_CURRENT_CONDITION);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_OVER_CURRENT_CONDITION, configuration, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_OVER_CURRENT_CONDITION);
    }

    /* Check for the state of the device . If the device is already configured, 
       we need to cancel the existing configuration before enabling this one.   */
    if (device -> ux_device_state == UX_DEVICE_CONFIGURED)
    {

        /* The device is configured. Get the first configuration pointer.  */
        current_configuration =  device -> ux_device_current_configuration;

        /* Deselect this instance */
        _ux_host_stack_configuration_instance_delete(current_configuration);
    }

    /* The device can now be configured.  */
    status =  _ux_host_stack_configuration_set(configuration);
    if (status != UX_SUCCESS)
        return(status);

    /* Create the configuration instance.  */
    status =  _ux_host_stack_configuration_instance_create(configuration);

    /* Return completion status.  */
    return(status);
}

