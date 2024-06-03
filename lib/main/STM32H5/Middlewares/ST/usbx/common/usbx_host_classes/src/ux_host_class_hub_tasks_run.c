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


#if defined(UX_HOST_STANDALONE)


static inline VOID _ux_host_class_hub_inst_tasks_run(UX_HOST_CLASS_HUB *hub);

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_hub_tasks_run                        PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function runs HUB background tasks.                            */
/*                                                                        */
/*    This function is for standalone mode.                               */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hub                                       Pointer to HUB instance   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_class_hub_port_change_connection_process                   */
/*                                          Process connection            */
/*    _ux_host_class_hub_port_change_enable_process                       */
/*                                          Enable process                */
/*    _ux_host_class_hub_port_change_over_current_process                 */
/*                                          Change over current process   */
/*    _ux_host_class_hub_port_change_reset_process                        */
/*                                          Reset process                 */
/*    _ux_host_class_hub_port_change_suspend_process                      */
/*                                          Suspend process               */
/*    _ux_host_class_hub_feature            Prepare feature request       */
/*    _ux_host_class_hub_status_get         Prepare get status request    */
/*    _ux_utility_short_get                 Get 16-bit word               */
/*    _ux_utility_memory_free               Memory free                   */
/*    _ux_host_stack_new_device_create      Obtain a free device instance */
/*    _ux_host_stack_transfer_run           Process the transfer          */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Host Stack                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Initial Version 6.1.12        */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed reset speed handling, */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hub_tasks_run(UX_HOST_CLASS *hub_class)
{
UX_HOST_CLASS_HUB           *hub;

    /* Validate class entry.  */
    if (hub_class -> ux_host_class_status != UX_USED ||
        hub_class -> ux_host_class_entry_function != _ux_host_class_hub_entry)
        return(UX_STATE_IDLE);

    /* Run for class instances.  */
    hub = (UX_HOST_CLASS_HUB *)hub_class -> ux_host_class_first_instance;
    while(hub)
    {

        /* Run tasks for each HUB instance.  */
        _ux_host_class_hub_inst_tasks_run(hub);
        hub = hub -> ux_host_class_hub_next_instance;
    }
    return(UX_STATE_WAIT);
}

/* Return non-zero if there is change.  */
static inline UCHAR _ux_host_class_hub_change_check(UX_HOST_CLASS_HUB *hub)
{
UX_ENDPOINT *endpoint = hub -> ux_host_class_hub_interrupt_endpoint;
UX_TRANSFER *transfer = &endpoint -> ux_endpoint_transfer_request;
UCHAR       *buffer   = transfer -> ux_transfer_request_data_pointer;
UCHAR       byte_i    = (UCHAR)(hub -> ux_host_class_hub_run_port >> 3);
UCHAR       bit_i     = (UCHAR)(hub -> ux_host_class_hub_run_port & 0x7u);
UCHAR       res       = (UCHAR)(buffer[byte_i] & (1u << bit_i));
    buffer[byte_i] = (UCHAR)(buffer[byte_i] & ~(1u << bit_i));
    return(res);
}
static inline VOID _ux_host_class_hub_status_process(UX_HOST_CLASS_HUB *hub, UINT port)
{
USHORT      port_status = hub -> ux_host_class_hub_run_port_status;
USHORT      port_change = hub -> ux_host_class_hub_run_port_change;

    /* Next port, if all changes handled.  */
    if (port_change == 0)
    {
        hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_CHANGE_NEXT;
        return;
    }

    if (port_change & UX_HOST_CLASS_HUB_PORT_CHANGE_OVER_CURRENT)
    {
        hub -> ux_host_class_hub_run_port_change = (USHORT)
                            (hub -> ux_host_class_hub_run_port_change &
                                ~UX_HOST_CLASS_HUB_PORT_CHANGE_OVER_CURRENT);
        _ux_host_class_hub_port_change_over_current_process(hub, port, port_status);

        /* Next: wait transfer done, then check change again.  */
        hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_TRANS_WAIT;
        hub -> ux_host_class_hub_next_state = UX_HOST_CLASS_HUB_STATUS_PROCESS;
        return;
    }
    if (port_change & UX_HOST_CLASS_HUB_PORT_CHANGE_ENABLE)
    {
        hub -> ux_host_class_hub_run_port_change = (USHORT)
                            (hub -> ux_host_class_hub_run_port_change &
                                ~UX_HOST_CLASS_HUB_PORT_CHANGE_ENABLE);
        _ux_host_class_hub_port_change_enable_process(hub, port, port_status);

        /* Next: wait transfer done, then check change again.  */
        hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_TRANS_WAIT;
        hub -> ux_host_class_hub_next_state = UX_HOST_CLASS_HUB_STATUS_PROCESS;
        return;
    }
    if (port_change & UX_HOST_CLASS_HUB_PORT_CHANGE_SUSPEND)
    {
        hub -> ux_host_class_hub_run_port_change = (USHORT)
                            (hub -> ux_host_class_hub_run_port_change &
                                ~UX_HOST_CLASS_HUB_PORT_CHANGE_SUSPEND);
        _ux_host_class_hub_port_change_suspend_process(hub, port, port_status);

        /* Next: wait transfer done, then check change again.  */
        hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_TRANS_WAIT;
        hub -> ux_host_class_hub_next_state = UX_HOST_CLASS_HUB_STATUS_PROCESS;
        return;
    }
    if (port_change & UX_HOST_CLASS_HUB_PORT_CHANGE_RESET)
    {
        hub -> ux_host_class_hub_run_port_change = (USHORT)
                            (hub -> ux_host_class_hub_run_port_change &
                                ~UX_HOST_CLASS_HUB_PORT_CHANGE_RESET);
        _ux_host_class_hub_port_change_reset_process(hub, port, port_status);

        /* Next: wait transfer done, then process reset
                 (get status and update enum device).  */
        hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_TRANS_WAIT;
        hub -> ux_host_class_hub_next_state = UX_HOST_CLASS_HUB_RESET_PROCESS;
        return;
    }
    if (port_change & UX_HOST_CLASS_HUB_PORT_CHANGE_CONNECTION)
    {
        hub -> ux_host_class_hub_run_port_change = (USHORT)
                            (hub -> ux_host_class_hub_run_port_change &
                                ~UX_HOST_CLASS_HUB_PORT_CHANGE_CONNECTION);
        _ux_host_class_hub_port_change_connection_process(hub, port, port_status);

        if (hub -> ux_host_class_hub_port_state & (1u << port))
        {

            /* Next: clear c_connection, then process connect.  */
            _ux_host_class_hub_feature(hub, port, UX_CLEAR_FEATURE, UX_HOST_CLASS_HUB_C_PORT_CONNECTION);
            hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_TRANS_WAIT;
            hub -> ux_host_class_hub_next_state = UX_HOST_CLASS_HUB_CONNECT_PROCESS;
            return;
        }

        /* Disable port, then clear c_enable, c_connection.  */
        _ux_host_class_hub_feature(hub, port, UX_CLEAR_FEATURE, UX_HOST_CLASS_HUB_PORT_ENABLE);
        hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_TRANS_WAIT;
        hub -> ux_host_class_hub_next_state = UX_HOST_CLASS_HUB_DISC_DISABLED;
        return;
    }
}
static inline VOID _ux_host_class_hub_inst_tasks_run(UX_HOST_CLASS_HUB *hub)
{
UX_HCD      *hcd;
UX_DEVICE   *device;
UX_DEVICE   *hub_device = hub -> ux_host_class_hub_device;
UX_ENDPOINT *ep0 = &hub_device -> ux_device_control_endpoint;
UX_ENDPOINT *dev_ep0;
UX_TRANSFER *trans0 = &ep0 -> ux_endpoint_transfer_request;
UINT        status;

    /* Task runs when hub is live.  */
    if (hub -> ux_host_class_hub_state != UX_HOST_CLASS_INSTANCE_LIVE)
        return;

    while(1)
    {

        /* Immediate state change: continue.
           Wait/pending state    : return.  */
        switch(hub -> ux_host_class_hub_run_state)
        {

        case UX_HOST_CLASS_HUB_CHANGE_CHECK   :

            /* Check if current port has changes.  */
            if (_ux_host_class_hub_change_check(hub))
            {
                if (hub -> ux_host_class_hub_run_port == 0)
                {

                    /* Device state change.  */
                    /* There is nothing handled now.  */

                    /* Next state: try next, port 1.  */
                    hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_CHANGE_NEXT;
                    continue;
                }

                /* Port state change.  */
                /* Next: read status.  */
                hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_STATUS_GET;
                continue;
            }

            /* No change detected,
             * still need to check if there is enumeration reset request.  */
            device = _ux_system_host -> ux_system_host_enum_device;
            while(device != UX_NULL)
            {

                /* Check if a device is waiting on the port for reset.  */
                if ((device -> ux_device_flags & UX_DEVICE_FLAG_RESET) &&
                    (device -> ux_device_parent == hub_device) &&
                    (device -> ux_device_port_location == hub -> ux_host_class_hub_run_port))
                {
                    device -> ux_device_flags &= ~UX_DEVICE_FLAG_RESET;
                    break;
                }

                /* Next enumerating device.  */
                device = device -> ux_device_enum_next;
            }
            if (device)
            {

                /* Next: reset then check port status ...  */
                hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_RESET;
                continue;
            }

            /* No change, try next port.  */

            /* Fall through.  */
        case UX_HOST_CLASS_HUB_CHANGE_NEXT    :
            if (hub -> ux_host_class_hub_run_port <
                hub -> ux_host_class_hub_descriptor.bNbPorts)
            {

                /* Check next port.  */
                hub -> ux_host_class_hub_run_port ++;
                hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_CHANGE_CHECK;
                continue;
            }

            /* All ports checked, next round.  */
            hub -> ux_host_class_hub_run_port = 0;
            hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_CHANGE_CHECK;
            return;

        case UX_HOST_CLASS_HUB_RESET          :
            status = _ux_host_class_hub_feature(hub, hub -> ux_host_class_hub_run_port,
                                UX_SET_FEATURE, UX_HOST_CLASS_HUB_PORT_RESET);

            /* Next: check another port, actions are taken on C_RESET detection.  */
            hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_TRANS_WAIT;
            hub -> ux_host_class_hub_next_state = UX_HOST_CLASS_HUB_CHANGE_NEXT;
            continue;

        case UX_HOST_CLASS_HUB_STATUS_GET     :
            status = _ux_host_class_hub_status_get(hub, hub -> ux_host_class_hub_run_port, UX_NULL, UX_NULL);

            if (status != UX_SUCCESS)
            {

                /* Fail, retry.  */
                hub -> ux_host_class_hub_run_port = 0;
                hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_CHANGE_CHECK;
                return;
            }
            hub -> ux_host_class_hub_transfer = trans0;
            hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_TRANS_WAIT;
            hub -> ux_host_class_hub_next_state = UX_HOST_CLASS_HUB_STATUS_GET_DONE;
            continue;

        case UX_HOST_CLASS_HUB_STATUS_GET_DONE:

            /* Store status change for later usage.  */
            hub -> ux_host_class_hub_run_port_status = (USHORT)
                _ux_utility_short_get(hub -> ux_host_class_hub_allocated);
            hub -> ux_host_class_hub_run_port_change = (USHORT)
                _ux_utility_short_get(hub -> ux_host_class_hub_allocated + 2);

            /* Free allocated buffer.  */
            _ux_utility_memory_free(hub -> ux_host_class_hub_allocated);
            hub -> ux_host_class_hub_allocated = UX_NULL;

            /* Fall through.  */
        case UX_HOST_CLASS_HUB_STATUS_PROCESS :
            _ux_host_class_hub_status_process(hub, hub -> ux_host_class_hub_run_port);
            hub -> ux_host_class_hub_transfer = trans0;
            continue;

        case UX_HOST_CLASS_HUB_CONNECT_PROCESS:

            status = _ux_host_stack_new_device_create(UX_DEVICE_HCD_GET(hub_device),
                            hub_device, hub -> ux_host_class_hub_run_port,
                            UX_FULL_SPEED_DEVICE, UX_MAX_SELF_POWER,
                            &device);
            if (status == UX_SUCCESS)
            {

                /* Put device in enumeration list.  */
                device -> ux_device_flags |= UX_DEVICE_FLAG_ENUM;
            }

            /* Try next port.  */
            hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_CHANGE_NEXT;
            return;

        case UX_HOST_CLASS_HUB_DISC_DISABLED:

            /* Next: clear C_ENABLE, then clear C_CONNECTION.  */
            _ux_host_class_hub_feature(hub, hub -> ux_host_class_hub_run_port, UX_CLEAR_FEATURE, UX_HOST_CLASS_HUB_C_PORT_ENABLE);
            hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_TRANS_WAIT;
            hub -> ux_host_class_hub_next_state = UX_HOST_CLASS_HUB_DISC_CLEAR_1;
            continue;

        case UX_HOST_CLASS_HUB_DISC_CLEAR_1:

            /* Next: clear C_CONNECTION, then next port.  */
            _ux_host_class_hub_feature(hub, hub -> ux_host_class_hub_run_port, UX_CLEAR_FEATURE, UX_HOST_CLASS_HUB_C_PORT_CONNECTION);
            hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_TRANS_WAIT;
            hub -> ux_host_class_hub_next_state = UX_HOST_CLASS_HUB_CHANGE_NEXT;
            continue;

        case UX_HOST_CLASS_HUB_RESET_PROCESS  :

            /* Find enum device, update enum port status, state -> ADDR_SET. */
            device = _ux_system_host -> ux_system_host_enum_device;
            while(device != UX_NULL)
            {

                /* If there is a device connected to the port, put its state to ADDR_SET.  */
                if ((device -> ux_device_parent == hub -> ux_host_class_hub_device) &&
                    (device -> ux_device_port_location == hub -> ux_host_class_hub_run_port))
                {

                    /* Save status and speed.  */
                    device -> ux_device_enum_port_status = (UCHAR)hub -> ux_host_class_hub_run_port_status;

                    /* Append speed information.  */
                    switch(hub -> ux_host_class_hub_run_port_status &
                        (UX_HOST_CLASS_HUB_PORT_STATUS_LOW_SPEED | UX_HOST_CLASS_HUB_PORT_STATUS_HIGH_SPEED))
                    {
                    case 0: /* It's Full speed.  */
                        device -> ux_device_enum_port_status |= UX_PS_DS_FS;
                        device -> ux_device_speed = UX_FULL_SPEED_DEVICE;
                        break;

                    case UX_HOST_CLASS_HUB_PORT_STATUS_HIGH_SPEED: /* It's high speed.  */
                        device -> ux_device_enum_port_status |= UX_PS_DS_HS;
                        device -> ux_device_speed = UX_HIGH_SPEED_DEVICE;
                        break;

                    default: /* It's Low speed.  */
                        device -> ux_device_speed = UX_LOW_SPEED_DEVICE;
                        break;
                    }

                    /* Return device address to 0.  */
                    hcd = UX_DEVICE_HCD_GET(device);
                    if (device -> ux_device_address)
                    {

                        /* Free the address.  */
                        hcd -> ux_hcd_address[(device -> ux_device_address-1) >> 3] &=
                            (UCHAR)(1u << ((device -> ux_device_address-1) & 7u));

                    }

                    /* Assume speed change, re-create EP0 at the HCD level.  */
                    dev_ep0 = &device -> ux_device_control_endpoint;
                    hcd -> ux_hcd_entry_function(hcd, UX_HCD_DESTROY_ENDPOINT, (VOID *)dev_ep0);
                    hcd -> ux_hcd_entry_function(hcd, UX_HCD_CREATE_ENDPOINT, (VOID *)dev_ep0);

                    /* Wait a while and set address.  */
                    device -> ux_device_enum_next_state = UX_HOST_STACK_ENUM_DEVICE_ADDR_SET;
                    device -> ux_device_enum_state = UX_HOST_STACK_ENUM_WAIT;
                    device -> ux_device_enum_wait_start = _ux_utility_time_get();
                    device -> ux_device_enum_wait_ms = UX_MS_TO_TICK_NON_ZERO(2);
                    break;
                }

                /* Next device.  */
                device = device -> ux_device_enum_next;
            }

            /* Reset processed check other status.  */
            hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_STATUS_PROCESS;
            continue;

        case UX_HOST_CLASS_HUB_TRANS_WAIT     :

            /* Poll transfer task.  */
            status = _ux_host_stack_transfer_run(hub -> ux_host_class_hub_transfer);

            /* Transfer done - next state.  */
            if (status == UX_STATE_NEXT || status == UX_STATE_IDLE)
            {
                hub -> ux_host_class_hub_run_state = hub -> ux_host_class_hub_next_state;
                continue;
            }

            /* Check error.  */
            if (status < UX_STATE_NEXT)
            {

                /* Fail, just check next port.  */
                hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_CHANGE_NEXT;
                hub -> ux_host_class_hub_run_status =
                                        hub -> ux_host_class_hub_transfer ->
                                            ux_transfer_request_completion_code;
                continue;
            }

            /* Transfer in progress, wait.  */
            return;

        default:
            break;
        }
    }
}
#endif
