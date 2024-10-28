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
/*    _ux_host_stack_rh_device_insertion                  PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function handles a device insertion on a downstream port of    */
/*    the root hub pointed by HCD.                                        */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    HCD                                   Pointer to HCD structure      */
/*    port_index                            Port index of insertion       */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_delay_ms                  Thread sleep                  */
/*    _ux_host_stack_new_device_create      New device create             */
/*    _ux_host_stack_device_remove          Device remove                 */
/*    (hcd_entry_function)                  Entry function of HCD driver  */
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
/*                                            checked HCD status before   */
/*                                            retrying enumeration,       */
/*                                            resulting in version 6.1    */
/*  02-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            handled more fail cases,    */
/*                                            updated internal call,      */
/*                                            added notification for      */
/*                                            device connection,          */
/*                                            added disconnection check   */
/*                                            in enumeration retries,     */
/*                                            resulting in version 6.1.4  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_rh_device_insertion(UX_HCD *hcd, UINT port_index)
{
#if defined(UX_HOST_STANDALONE)
UX_DEVICE   *device;
UINT        status;


    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_RH_DEVICE_INSERTION, hcd, port_index, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* Anyway there is device connected.  */
    hcd -> ux_hcd_rh_device_connection |= (ULONG)(1 << port_index);

    /* Create a new device slot for enumeration.  */
    status =  _ux_host_stack_new_device_create(hcd, UX_NULL, port_index,
                                        UX_FULL_SPEED_DEVICE, UX_MAX_SELF_POWER,
                                        &device);

    /* Link the device in enumeration list.  */
    if (status == UX_SUCCESS)
    {

        /* Set enumeration flag to process enumeration sequence.  */
        device -> ux_device_flags |= UX_DEVICE_FLAG_ENUM;

        /* Done success.  */
        return(UX_SUCCESS);
    }

#else
UX_DEVICE   *device = UX_NULL;
UINT        index_loop;
UINT        device_speed;
ULONG       port_status;
UINT        status;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_RH_DEVICE_INSERTION, hcd, port_index, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* Perform a PORT_ENABLE command.  */
    port_status =  hcd -> ux_hcd_entry_function(hcd, UX_HCD_ENABLE_PORT, (VOID *)((ALIGN_TYPE)port_index));

    /* Check return status.  */
    if (port_status == UX_PORT_INDEX_UNKNOWN)
        return(port_status);

    /* A debounce interval with a minimum duration of 100 ms on attach.  */
    _ux_utility_delay_ms(100);

    /* The first attempts to do a device enumeration may fail.
       Typically, after the port is reset and the first command is sent to
       the device, there is no answer. In this case, we reset the port again
       and retry. Usually that does the trick!  */
    for (index_loop = 0; index_loop < UX_RH_ENUMERATION_RETRY; index_loop++)
    {

        /* Now we have to do a PORT_RESET command.  */
        port_status =  hcd -> ux_hcd_entry_function(hcd, UX_HCD_RESET_PORT, (VOID *)((ALIGN_TYPE)port_index));
        if (port_status == UX_SUCCESS)
        {

            /* The port reset phase was successful. Before we invoke the device enumeration function,
               we need to know the speed of the device.  */
            port_status =  hcd -> ux_hcd_entry_function(hcd, UX_HCD_GET_PORT_STATUS, (VOID *)((ALIGN_TYPE)port_index));

            /* Check return status.  */
            if (port_status == UX_PORT_INDEX_UNKNOWN)
            {

                /* Error trap. */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ROOT_HUB, UX_DEVICE_ENUMERATION_FAILURE);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DEVICE_ENUMERATION_FAILURE, port_index, 0, 0, UX_TRACE_ERRORS, 0, 0)

                return(UX_DEVICE_ENUMERATION_FAILURE);
            }

            /* Check if device is still connected.  */
            if ((port_status & UX_PS_CCS) == 0)
            {

                /* Device disconnected during enumeration retries.  */
                return(UX_DEVICE_ENUMERATION_FAILURE);
            }

            /* Set the device speed.  */
            device_speed =  port_status >> UX_PS_DS;

            /* Ask the USB stack to enumerate this device. A root hub is considered self
               powered. */
            status =  _ux_host_stack_new_device_create(hcd, UX_NULL, port_index, device_speed, UX_MAX_SELF_POWER, &device);

            /* Check return status.  */
            if (status == UX_SUCCESS)
            {

                /* Successful device create.  */

                /* The device has been mounted properly, we have to remember this
                   so when the device is removed, we have to invoke the enumeration
                   function again */
                hcd -> ux_hcd_rh_device_connection |= (ULONG)(1 << port_index);

                /* If the device instance is ready, notify application for connection.  */
                if (_ux_system_host -> ux_system_host_change_function)
                {
                    _ux_system_host -> ux_system_host_change_function(UX_DEVICE_CONNECTION, UX_NULL, (VOID*)device);
                }

                /* Return success to the caller.  */
                return(UX_SUCCESS);
            }
            else
            {

                /* Return error if HCD is dead.  */
                if (hcd -> ux_hcd_status != UX_HCD_STATUS_OPERATIONAL)
                    return(UX_CONTROLLER_DEAD);

                /* No retry if there are too many devices.  */
                if (status == UX_TOO_MANY_DEVICES)
                    break;

                /* No retry if there is no class found.  */
                if (status == UX_NO_CLASS_MATCH)
                    break;

                /* Simulate remove to free allocated resources if retry.  */
                if (index_loop < UX_RH_ENUMERATION_RETRY - 1)
                    _ux_host_stack_device_remove(hcd, UX_NULL, port_index);
            }
        }

        /* We get here if something did not go well. Either the port did not respond
           well to the ENABLE\RESET phases or the device did not enumerate well
           so we try again ! */
        _ux_utility_delay_ms(UX_RH_ENUMERATION_RETRY_DELAY);
    }
#endif /* defined(UX_HOST_STANDALONE)  */

    /* If we get here, the device did not enumerate completely.
       The device is still attached to the root hub and therefore
       there could be a physical connection with a unconfigured device.  */
    hcd -> ux_hcd_rh_device_connection |= (ULONG)(1 << port_index);

    /* Notify application for a physical connection failed to be enumed.
       Device instance NULL indicates too many devices.
       Device state unconfigured indicates enumeration fail.  */
    if (_ux_system_host -> ux_system_host_change_function)
    {
        _ux_system_host -> ux_system_host_change_function(UX_DEVICE_CONNECTION, UX_NULL, (VOID*)device);
    }

    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ROOT_HUB, UX_DEVICE_ENUMERATION_FAILURE);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DEVICE_ENUMERATION_FAILURE, port_index, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* Return a failed enumeration.  */
    return(UX_DEVICE_ENUMERATION_FAILURE);
}
