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
/**   HID Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_configure                        PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function calls the USBX stack to do a SET_CONFIGURATION to the */ 
/*    HID. Once the HID is configured, its interface will be activated    */ 
/*    and all the endpoints enumerated (1 interrupt endpoint in the case  */ 
/*    of the HID).                                                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid                                   Pointer to HID class          */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_configuration_interface_get Get interface            */ 
/*    _ux_host_stack_device_configuration_get    Get configuration        */ 
/*    _ux_host_stack_device_configuration_select Select configuration     */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HID Class                                                           */ 
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
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_configure(UX_HOST_CLASS_HID *hid)
{

UINT                    status;
UX_CONFIGURATION        *configuration;
#if UX_MAX_DEVICES > 1
UX_DEVICE               *device;
UX_DEVICE               *parent_device;
#endif


    /* If the device has been configured already, we don't need to do it again. */
    if (hid -> ux_host_class_hid_device -> ux_device_state == UX_DEVICE_CONFIGURED)
        return(UX_SUCCESS);

    /* A HID normally has one configuration. So retrieve the 1st configuration only.  */
    status =  _ux_host_stack_device_configuration_get(hid -> ux_host_class_hid_device, 0, &configuration);
    if (status != UX_SUCCESS)
    {
    
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CONFIGURATION_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONFIGURATION_HANDLE_UNKNOWN, hid -> ux_host_class_hid_device, 0, 0, UX_TRACE_ERRORS, 0, 0)
    
        return(UX_CONFIGURATION_HANDLE_UNKNOWN);
    }
        
#if UX_MAX_DEVICES > 1
    /* Get the device container for this configuration.  */
    device =  configuration -> ux_configuration_device;

    /* Get the parent container for this device.  */
    parent_device =  device -> ux_device_parent;
    
    /* Check the HID power source and check the parent power source for 
       incompatible connections.  */
    if (hid -> ux_host_class_hid_device -> ux_device_power_source == UX_DEVICE_BUS_POWERED)
    {
        
        /* If the device is NULL, the parent is the root hid and we don't have to worry. 
           If the parent is not the root HID, check for its power source.  */
        if ((parent_device != UX_NULL) && (parent_device -> ux_device_power_source == UX_DEVICE_BUS_POWERED))
        {                        

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CONNECTION_INCOMPATIBLE);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONNECTION_INCOMPATIBLE, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_CONNECTION_INCOMPATIBLE);
        }            
    }
#endif

    /* We have the valid configuration. Ask the USBX stack to set this configuration.  */        
    _ux_host_stack_device_configuration_select(configuration);

    /* If the operation went well, the hid default alternate setting for the HID interface is active 
       and the interrupt endpoint is now enabled. We have to memorize the first interface since the 
       interrupt endpoint is hooked to it. */
    status =  _ux_host_stack_configuration_interface_get(configuration, 0, 0, &hid -> ux_host_class_hid_interface);

    /* Return completion status.  */
    return(status);
}

