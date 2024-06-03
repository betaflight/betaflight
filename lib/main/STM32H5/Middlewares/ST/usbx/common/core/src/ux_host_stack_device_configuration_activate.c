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
/*    _ux_host_stack_device_configuration_activate        PORTABLE C      */
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
/*    activated on the device.                                            */
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
/*    _ux_utility_semaphore_get                    Get semaphore          */
/*    _ux_utility_semaphore_put                    Put semaphore          */
/*    _ux_host_stack_configuration_interface_scan  Scan and activate      */
/*                                                 interfaces             */
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
/*  02-02-2021     Chaoqiong Xiao           Initial Version 6.1.4         */
/*  06-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed trace enabled error,  */
/*                                            resulting in version 6.1.7  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_device_configuration_activate(UX_CONFIGURATION *configuration)
{
#if defined(UX_HOST_STANDALONE)
UX_INTERRUPT_SAVE_AREA
#endif
UX_DEVICE               *device;
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
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_DEVICE_CONFIGURATION_ACTIVATE, device, configuration, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

#if defined(UX_HOST_STANDALONE)

    /* Check device lock.  */
    UX_DISABLE
    if (device -> ux_device_flags & UX_DEVICE_FLAG_LOCK)
    {
        UX_RESTORE
        return(UX_BUSY);
    }
    device -> ux_device_flags |= UX_DEVICE_FLAG_LOCK;
    UX_RESTORE
#else

    /* Protect the control endpoint semaphore here.  It will be unprotected in the
       transfer request function.  */
    status =  _ux_host_semaphore_get(&device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);

    /* Check for status.  */
    if (status != UX_SUCCESS)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_SEMAPHORE_ERROR);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_SEMAPHORE_ERROR, configuration, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_SEMAPHORE_ERROR);
    }
#endif

    /* Check for the state of the device . If the device is already configured,
       we need to cancel the existing configuration before enabling this one.   */
    if (device -> ux_device_state == UX_DEVICE_CONFIGURED)
    {

        /* If this configuration is already activated, we are good,
           otherwise report error.  */
        status = (device -> ux_device_current_configuration == configuration) ?
                    UX_SUCCESS : UX_ALREADY_ACTIVATED;
#if defined(UX_HOST_STANDALONE)
        device -> ux_device_flags &= ~UX_DEVICE_FLAG_LOCK;
#else
        _ux_host_semaphore_put(&device -> ux_device_protection_semaphore);
#endif
        return(status);
    }

    /* Scan and activate the interfaces.  */
    status =  _ux_host_stack_configuration_interface_scan(configuration);

#if defined(UX_HOST_STANDALONE)

    if (status == UX_SUCCESS)
    {

        /* Place device enum state: LOCK -> SET_CONFIGURE.  */
        device -> ux_device_enum_trans =
            &device -> ux_device_control_endpoint.ux_endpoint_transfer_request;
        device -> ux_device_enum_state = UX_HOST_STACK_ENUM_TRANS_LOCK_WAIT;
        device -> ux_device_enum_inst.configuration = configuration;
        device -> ux_device_enum_next_state = UX_HOST_STACK_ENUM_CONFIG_SET;

        /* Set enumeration flag to process enumeration sequence.  */
        device -> ux_device_flags |= UX_DEVICE_FLAG_ENUM;

        /* Wait until enumeration done and device removed.  */
        while(device -> ux_device_enum_state != UX_STATE_IDLE)
        {
            _ux_system_host_tasks_run();
        }
    }
#endif

    /* Return completion status.  */
    return(status);
}
