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
/*    _ux_host_stack_hcd_unregister                       PORTABLE C      */
/*                                                           6.1.2        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function unregisters a USB host controller driver (HCD) with   */
/*    the USBX stack, removes all devices rooted from that controller and */
/*    disable/stop the controller.                                        */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_name                              Name of HCD to unregister     */
/*    hcd_param1                            Parameter 1 of HCD            */
/*    hcd_param2                            Parameter 2 of HCD            */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*    UX_SUCCESS                            HCD unregistered, related     */
/*                                          devices removed and hardware  */
/*                                          disabled/stopped success      */
/*    UX_ERROR                              HCD not found                 */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_string_length_check       Check and return C string     */
/*                                          length if no error            */
/*    _ux_utility_memory_compare            Memory compare                */
/*    (ux_hcd_entry_function)               HCD dispatch entry function   */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  09-30-2020     Chaoqiong Xiao           Initial Version 6.1           */
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed HCD devices scan,     */
/*                                            used HCD uninit command,    */
/*                                            fixed HCD status scan,      */
/*                                            resulting in version 6.1.2  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_hcd_unregister(UCHAR *hcd_name,
                                    ULONG hcd_param1, ULONG hcd_param2)
{

UINT        status;
UX_DEVICE   *device;
UX_HCD      *hcd;
#if UX_MAX_CLASS_DRIVER > 1 || UX_MAX_DEVICES > 1
ULONG       scan_index;
#endif
#if !defined(UX_NAME_REFERENCED_BY_POINTER)
UINT        hcd_name_length =  0;
#endif


    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_HCD_UNREGISTER, hcd_name, hcd_param1, hcd_param2, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

#if !defined(UX_NAME_REFERENCED_BY_POINTER)
    /* Get the length of the class name (exclude null-terminator).  */
    status =  _ux_utility_string_length_check(hcd_name,
                                    &hcd_name_length, UX_MAX_HCD_NAME_LENGTH);
    if (status)
        return(status);
#endif

    /* Get first HCD.  */
    hcd =  _ux_system_host -> ux_system_host_hcd_array;

    /* Default to not found status.  */
    status = UX_FALSE;

#if UX_MAX_CLASS_DRIVER > 1
    /* We need to parse the controller driver table to find an empty spot.  */
    for (scan_index = 0;
         scan_index < UX_SYSTEM_HOST_MAX_HCD_GET();
         scan_index++)
    {
#endif

        /* Is this slot available and saved hcd_parameters match?  */
        if (hcd -> ux_hcd_status != UX_UNUSED &&
            hcd -> ux_hcd_io == hcd_param1 &&
            hcd -> ux_hcd_irq == hcd_param2)
        {


            /* Yes, compare the name (with null terminator).  */
            status = ux_utility_name_match(hcd -> ux_hcd_name,
                                            hcd_name, hcd_name_length + 1);
#if UX_MAX_CLASS_DRIVER > 1

            /* Break if name match (found HCD).  */
            if (status == UX_TRUE)
                break;
#endif
            }

#if UX_MAX_CLASS_DRIVER > 1
        /* Try the next HCD structure */
        hcd ++;
    }
#endif

    /* No valid HCD found.  */
    if (status != UX_TRUE)
        return(UX_ERROR);

    /* Now disable controller.  */
    hcd -> ux_hcd_entry_function(hcd, UX_HCD_UNINITIALIZE, UX_NULL);

    /* Get first device.  */
    device = _ux_system_host -> ux_system_host_device_array;

#if UX_MAX_DEVICES > 1
    /* Remove all devices connected to the controller.  */
    for (scan_index = 0;
         scan_index < _ux_system_host -> ux_system_host_max_devices;
         scan_index ++)
    {
#else

        /* No device.  */
        if (device -> ux_device_handle == UX_UNUSED)
            return(UX_SUCCESS);
#endif

        /* Is this device on the HCD root hub?  */
        if (UX_DEVICE_HCD_MATCH(device, hcd) &&
            UX_DEVICE_PARENT_IS_ROOTHUB(device))
        {

            /* Remove the device (and downstream things connected to it).  */
            _ux_host_stack_device_remove(hcd, UX_DEVICE_PARENT_GET(device),
                                         UX_DEVICE_PORT_LOCATION_GET(device));

            /* The device has been removed, so the port is free again.  */
            hcd -> ux_hcd_rh_device_connection &= (ULONG)
                                    ~(1 << device -> ux_device_port_location);
        }

#if UX_MAX_DEVICES > 1

        /* Try the next device.  */
        device ++;
    }
#endif

    /* And we have one new controller unregistered.  */
    _ux_system_host -> ux_system_host_registered_hcd --;

    return(UX_SUCCESS);
}
