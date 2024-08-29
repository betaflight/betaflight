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
/*    _ux_host_stack_device_get                           PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function returns a device container based on its index. The    */
/*    device index start with device 0. Note that the index is a ULONG    */
/*    because we could have several controllers and a byte index might    */
/*    not be enough.                                                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    device_index                          Index of device               */
/*    device                                Destination for device pointer*/
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
/*                                            optimized based on compile  */
/*                                            definitions,                */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_device_get(ULONG device_index, UX_DEVICE **device)
{

UX_DEVICE   *current_device;
#if UX_MAX_DEVICES > 1
ULONG       current_device_index;
#endif

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_DEVICE_GET, device_index, 0, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* Check if the device index is still within the limits.  */
    if (device_index >= UX_SYSTEM_HOST_MAX_DEVICES_GET())
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_DEVICE_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DEVICE_HANDLE_UNKNOWN, device, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_DEVICE_HANDLE_UNKNOWN);
    }

#if UX_MAX_DEVICES > 1

    /* Start with the first device.  */
    current_device =        _ux_system_host -> ux_system_host_device_array;
    current_device_index =  0;

    /* Search the list until the end.  */
    while (current_device_index < _ux_system_host -> ux_system_host_max_devices)
    {

        /* Check to see if this device is existing.  */
        if (current_device -> ux_device_handle != UX_UNUSED)
        {

            /* Have we reached the index we are looking for?  */
            if (device_index == current_device_index)
            {

                /* Yes, return the device pointer.  */
                *device =  current_device;

                /* Return successful completion.  */
                return(UX_SUCCESS);
            }
        }

        /* Move to next device index.  */
        current_device_index++;

        /* Move to next device.  */
        current_device++;
    }
#else

    /* Only one device, just check if it's used.  */
    current_device = _ux_system_host -> ux_system_host_device_array;
    if (current_device -> ux_device_handle != UX_UNUSED)
    {
        *device = current_device;
        return(UX_SUCCESS);
    }
#endif

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DEVICE_HANDLE_UNKNOWN, device, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* Return error.  */
    return(UX_DEVICE_HANDLE_UNKNOWN);
}
