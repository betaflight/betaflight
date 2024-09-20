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
/**   HUB Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hub.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hub_port_change_connection_process   PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will process a connection change on the port. This    */
/*    indicates that a device has been attached to the port.              */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hub                                   Pointer to HUB class          */ 
/*    port                                  Port number                   */ 
/*    port_status                           Port status                   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_hub_feature            Set HUB feature               */ 
/*    _ux_host_class_hub_port_reset         Reset port                    */ 
/*    _ux_host_class_hub_status_get         Get status                    */ 
/*    _ux_host_stack_new_device_create      Create new device             */ 
/*    _ux_host_stack_device_remove          Remove device                 */ 
/*    _ux_utility_delay_ms                  Thread sleep                  */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HUB Class                                                           */ 
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
/*                                            handled more fail cases,    */
/*                                            updated internal call,      */
/*                                            added notification for      */
/*                                            device connection,          */
/*                                            added disconnection check   */
/*                                            in enumeration retries,     */
/*                                            resulting in version 6.1.4  */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_hub_port_change_connection_process(UX_HOST_CLASS_HUB *hub, UINT port, UINT port_status)
{

UX_HCD      *hcd;
#if !defined(UX_HOST_STANDALONE)
UX_DEVICE   *device = UX_NULL;
UINT        device_speed;
UINT        device_enumeration_retry;
USHORT      port_power;
UINT        status;
USHORT      local_port_status;
USHORT      local_port_change;
#endif
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HUB_PORT_CHANGE_CONNECTION_PROCESS, hub, port, port_status, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Get the HCD used by this instance.  */
    hcd = UX_DEVICE_HCD_GET(hub -> ux_host_class_hub_device);

    /* If there is a device attached on this HUB, there is a new device and it should 
       be enumerated.  */
    if (port_status & UX_HOST_CLASS_HUB_PORT_STATUS_CONNECTION)
    {

        /* Check if there was a previous device already attached on this port. This may happen when a device
           disconnects and reconnect very quickly before the hub has a chance to poll the port state. In this
           case we do a device removal before doing a device connection.  */
        if (hub -> ux_host_class_hub_port_state & (UINT)(1 << port))
        {

            /* There was a device attached previously. Perform a removal.  */
            _ux_host_stack_device_remove(hcd, hub -> ux_host_class_hub_device, port);
            
        }
        else
            
            /* Mark device connection.  */
            hub -> ux_host_class_hub_port_state |= (UINT)(1 << port);

#if defined(UX_HOST_STANDALONE)
        /* Port operations are done outside.  */
#else

        /* Tell the hub to clear the change bit for this port so that we do 
           not process the same change event again.  */
        _ux_host_class_hub_feature(hub, port, UX_CLEAR_FEATURE, UX_HOST_CLASS_HUB_C_PORT_CONNECTION);

        /* Some devices are known to fail on the first try.  */
        for (device_enumeration_retry = 0; device_enumeration_retry < UX_HOST_CLASS_HUB_ENUMERATION_RETRY; device_enumeration_retry++)
        {

            /* Wait for debounce.  */
            _ux_utility_delay_ms(UX_HOST_CLASS_HUB_ENUMERATION_DEBOUNCE_DELAY);

            /* The port must be reset.  */
            status =  _ux_host_class_hub_port_reset(hub, port);
            if (status != UX_SUCCESS)
                return;
                
            /* Reset succeeded, so perform a new port status after reset to force speed reevaluation.  */
            status =  _ux_host_class_hub_status_get(hub, port, &local_port_status, &local_port_change);
            if (status != UX_SUCCESS)
                return;

            /* Check if device is still connected.  */
            if ((local_port_status & UX_HOST_CLASS_HUB_PORT_STATUS_CONNECTION) == 0)
                return;

            /* Device connected. Get the device speed.  */
            if (local_port_status & UX_HOST_CLASS_HUB_PORT_STATUS_LOW_SPEED)
                device_speed =  UX_LOW_SPEED_DEVICE;
            else
            {

                if (local_port_status & UX_HOST_CLASS_HUB_PORT_STATUS_HIGH_SPEED)
                    device_speed =  UX_HIGH_SPEED_DEVICE;
                else    
                    device_speed =  UX_FULL_SPEED_DEVICE;
            }    
    
            /* Decide how much power this device can draw. This depends if the HUB is self 
               powered or bus powered.  */
            if (hub -> ux_host_class_hub_device -> ux_device_power_source == UX_DEVICE_BUS_POWERED)

                /* Hub is bus powered.  */
                port_power =  UX_MAX_BUS_POWER;
            else            
            
                /* Hub is self powered.  */
                port_power =  UX_MAX_SELF_POWER;

            /* Wait for reset recovery.  */
            _ux_utility_delay_ms(UX_HOST_CLASS_HUB_ENUMERATION_RESET_RECOVERY_DELAY);

            /* Perform the device creation.  */
            status =  _ux_host_stack_new_device_create(UX_DEVICE_HCD_GET(hub -> ux_host_class_hub_device),
                                            hub -> ux_host_class_hub_device,
                                            port, device_speed, port_power,
                                            &device);

            /* Check return status.  */
            if (status == UX_SUCCESS)
            {

                /* Successful device creation.  */

                /* If the device instance is ready, notify application for unconfigured device.  */
                if (_ux_system_host -> ux_system_host_change_function)
                {
                    _ux_system_host -> ux_system_host_change_function(UX_DEVICE_CONNECTION, UX_NULL, (VOID*)device);
                }

                /* Just return.  */
                return;
            }
            else
            {

                /* No retry if there are too many devices.  */
                if (status == UX_TOO_MANY_DEVICES)
                    break;

                /* No retry if there is no class found.  */
                if (status == UX_NO_CLASS_MATCH)
                    break;

                if (device_enumeration_retry < UX_HOST_CLASS_HUB_ENUMERATION_RETRY - 1)
                {

                    /* Simulate remove to free allocated resources before retry.  */
                    _ux_host_stack_device_remove(hcd, hub -> ux_host_class_hub_device, port);

                    /* Wait for a while.  */
                    _ux_utility_delay_ms(UX_HOST_CLASS_HUB_ENUMERATION_RETRY_DELAY);
                }
            }
        }

        /* If we get here, the device did not enumerate completely.
           The device is still attached to the hub and therefore there is a
           physical connection with a unenumerated device. */

        /* If the device instance is ready, notify application for unconfigured device.  */
        if (_ux_system_host -> ux_system_host_change_function)
        {
            _ux_system_host -> ux_system_host_change_function(UX_DEVICE_CONNECTION, UX_NULL, (VOID*)device);
        }

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ROOT_HUB, UX_DEVICE_ENUMERATION_FAILURE);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DEVICE_ENUMERATION_FAILURE, port, 0, 0, UX_TRACE_ERRORS, 0, 0)
#endif
    }           
    else
    {

        /* Check if there was a no previous device attached on this port. */
        if ((hub -> ux_host_class_hub_port_state & (UINT)(1 << port)))
        {
            /* Mark device disconnection.  */
            hub -> ux_host_class_hub_port_state &= (UINT)~(1 << port);

            /* We get here when there is a device extraction.  */
            _ux_host_stack_device_remove(hcd, hub -> ux_host_class_hub_device, port);
        }

#if defined(UX_HOST_STANDALONE)
        /* Port operations are done outside.  */
#else

        /* The port should be disabled now. Power is still applied.  */
        status =  _ux_host_class_hub_feature(hub, port, UX_CLEAR_FEATURE, UX_HOST_CLASS_HUB_PORT_ENABLE);

        /* We must clear the enable change condition so that we don't get awaken again.  */
        _ux_host_class_hub_feature(hub, port, UX_CLEAR_FEATURE, UX_HOST_CLASS_HUB_C_PORT_ENABLE);

        /* We must clear the connection change condition so that we don't get awaken again.  */
        _ux_host_class_hub_feature(hub, port, UX_CLEAR_FEATURE, UX_HOST_CLASS_HUB_C_PORT_CONNECTION);
#endif
    }        

    /* Return to caller.  */
    return;
}

