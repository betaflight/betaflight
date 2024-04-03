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
extern UINT _ux_host_class_hub_descriptor_parse(UX_HOST_CLASS_HUB *hub, UCHAR *descriptor);
static inline UINT _ux_host_class_hub_activate_wait(UX_HOST_CLASS_COMMAND *command);
#endif


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_hub_entry                            PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the entry point of the HUB class. It will be       */
/*    called by the USBX stack enumeration module when there is a new     */
/*    device on the bus or when there is a device extraction.             */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                               Pointer to command            */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_class_hub_activate           Activate HUB class            */
/*    _ux_host_class_hub_deactivate         Deactivate HUB class          */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Host Stack                                                          */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used query usage of device  */
/*                                            ClassSubclassProtocol,      */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.12 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed power on delay calc,  */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hub_entry(UX_HOST_CLASS_COMMAND *command)
{

UINT    status;


    /* The command request will tell us we need to do here, either a enumeration
       query, an activation or a deactivation.  */
    switch (command -> ux_host_class_command_request)
    {

    case UX_HOST_CLASS_COMMAND_QUERY:

        /* The query command is used to let the stack enumeration process know if we want to own
           this device or not.  */
        if ((command -> ux_host_class_command_usage == UX_HOST_CLASS_COMMAND_USAGE_DCSP) &&
            (command -> ux_host_class_command_class == UX_HOST_CLASS_HUB_CLASS))
            return(UX_SUCCESS);
        else
            return(UX_NO_CLASS_MATCH);


    case UX_HOST_CLASS_COMMAND_ACTIVATE:

        /* The activate command is used when the device inserted has found a parent and
           is ready to complete the enumeration.  */
        status =  _ux_host_class_hub_activate(command);

        /* Return completion status.  */
        return(status);

#if defined(UX_HOST_STANDALONE)
    case UX_HOST_CLASS_COMMAND_ACTIVATE_WAIT:
        status = _ux_host_class_hub_activate_wait(command);
        return(status);
#endif

    case UX_HOST_CLASS_COMMAND_DEACTIVATE:

        /* The deactivate command is used when the device has been extracted either
           directly or when its parents has been extracted */
        status =  _ux_host_class_hub_deactivate(command);

        /* Return completion status.  */
        return(status);

    default:

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Return error status.  */
        return(UX_FUNCTION_NOT_SUPPORTED);
    }
}

#if defined(UX_HOST_STANDALONE)
#define UX_HOST_STACK_ENUM_TRANS_ERROR(t)       (                               \
    (t)->ux_transfer_request_completion_code != UX_SUCCESS ||                   \
    (t)->ux_transfer_request_actual_length !=                                   \
        (t)->ux_transfer_request_requested_length)

static inline UINT _ux_host_class_hub_enum_get_status(UX_HOST_CLASS_HUB *hub, UX_TRANSFER *trans)
{
    hub -> ux_host_class_hub_allocated = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, 2);
    if (hub -> ux_host_class_hub_allocated == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    trans -> ux_transfer_request_requested_length =  2;
    trans -> ux_transfer_request_data_pointer =      hub -> ux_host_class_hub_allocated;
    trans -> ux_transfer_request_function =          UX_GET_STATUS;
    trans -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
    trans -> ux_transfer_request_value =             0;
    trans -> ux_transfer_request_index =             0;
    return(UX_SUCCESS);
}
static inline VOID _ux_host_class_hub_enum_set_config(UX_HOST_CLASS_HUB *hub, UX_TRANSFER *trans, UX_CONFIGURATION *configuration)
{
    UX_PARAMETER_NOT_USED(hub);
    trans -> ux_transfer_request_requested_length =  0;
    trans -> ux_transfer_request_function =          UX_SET_CONFIGURATION;
    trans -> ux_transfer_request_type =              UX_REQUEST_OUT | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
    trans -> ux_transfer_request_value =             (USHORT) configuration -> ux_configuration_descriptor.bConfigurationValue;
    trans -> ux_transfer_request_index =             0;
}
static inline UINT _ux_host_class_hub_activate_wait(UX_HOST_CLASS_COMMAND *command)
{
UX_DEVICE               *device;
UX_ENDPOINT             *ep0;
UX_TRANSFER             *trans0, *trans;
UX_HOST_CLASS_HUB       *hub;
UINT                    status;
ULONG                   current_ms, elapsed_ms;

    /* Get the instance for this class.  */
    device = (UX_DEVICE *)command -> ux_host_class_command_container;
    hub =  (UX_HOST_CLASS_HUB *) device -> ux_device_class_instance;

    /* Get endpoint 0 and transfer request.  */
    ep0 = &device -> ux_device_control_endpoint;
    trans0 = &ep0 -> ux_endpoint_transfer_request;

    /* Get current transfer request.  */
    trans = hub -> ux_host_class_hub_transfer;

    /* Immediate state change: continue.
        Wait/pending state   : return.  */
    while(1)
    {

        /* Run initialize state machine.  */
        switch(hub -> ux_host_class_hub_enum_state)
        {

        case UX_HOST_CLASS_HUB_ENUM_GET_STATUS       :
            status = _ux_host_class_hub_enum_get_status(hub, trans0);
            if (status != UX_SUCCESS)
            {
                hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_DONE;
                hub -> ux_host_class_hub_run_status = status;
                continue;
            }

            UX_TRANSFER_STATE_RESET(trans0);
            hub -> ux_host_class_hub_transfer = trans0;
            hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_TRANS_WAIT;
            hub -> ux_host_class_hub_next_state = UX_HOST_CLASS_HUB_ENUM_POWER_CHECK;
            continue;

        case UX_HOST_CLASS_HUB_ENUM_POWER_CHECK      :

            /* Transfer request error check.  */
            if (UX_HOST_STACK_ENUM_TRANS_ERROR(trans))
            {
                hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_DONE;
                hub -> ux_host_class_hub_run_status = UX_CONNECTION_INCOMPATIBLE;
                continue;
            }

            /* Save power source setting.  */
            if (hub -> ux_host_class_hub_allocated[0] & UX_STATUS_DEVICE_SELF_POWERED)
                device -> ux_device_power_source = UX_DEVICE_SELF_POWERED;
            else
                device -> ux_device_power_source = UX_DEVICE_BUS_POWERED;

            /* Free allocated buffer.  */
            _ux_utility_memory_free(hub -> ux_host_class_hub_allocated);
            hub -> ux_host_class_hub_allocated = UX_NULL;

#if UX_MAX_DEVICES > 1

            /* Check the HUB power source and check the parent power source.  */
            if (device -> ux_device_power_source == UX_DEVICE_BUS_POWERED)
            {

                /* Check parent power.  */
                if (device -> ux_device_parent != UX_NULL)
                {
                    if (device -> ux_device_parent -> ux_device_power_source == UX_DEVICE_BUS_POWERED)
                    {
                        hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_DONE;
                        hub -> ux_host_class_hub_run_status = UX_CONNECTION_INCOMPATIBLE;
                        continue;
                    }
                }
            }
#endif
            /* Fall through.  */
        case UX_HOST_CLASS_HUB_ENUM_SET_CONFIG       :
            _ux_host_class_hub_enum_set_config(hub, trans0,
                                    device -> ux_device_first_configuration);

            UX_TRANSFER_STATE_RESET(trans0);
            hub -> ux_host_class_hub_transfer = trans0;
            hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_TRANS_WAIT;
            hub -> ux_host_class_hub_next_state = UX_HOST_CLASS_HUB_ENUM_SET_CONFIG_DONE;
            continue;

        case UX_HOST_CLASS_HUB_ENUM_SET_CONFIG_DONE  :

            /* Transfer request error check.  */
            if (UX_HOST_STACK_ENUM_TRANS_ERROR(trans))
            {
                hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_DONE;
                hub -> ux_host_class_hub_run_status = UX_CONNECTION_INCOMPATIBLE;
                continue;
            }

            /* Create configuration instance.  */
            status = _ux_host_stack_configuration_instance_create(device -> ux_device_first_configuration);
            if (status != UX_SUCCESS)
            {
                hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_DONE;
                hub -> ux_host_class_hub_run_status = status;
                continue;
            }

            /* Change device state.  */
            device -> ux_device_state = UX_DEVICE_CONFIGURED;
            device -> ux_device_current_configuration = device -> ux_device_first_configuration;

            /* Fall through.  */
        case UX_HOST_CLASS_HUB_ENUM_GET_HUB_DESC     :
            status = _ux_host_class_hub_descriptor_get(hub);
            if (UX_SUCCESS != status)
            {
                hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_DONE;
                hub -> ux_host_class_hub_run_status = status;
                continue;
            }

            /* Request already updated, to transfer state.  */
            hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_TRANS_WAIT;
            hub -> ux_host_class_hub_next_state = UX_HOST_CLASS_HUB_ENUM_GET_HUB_DESC_DONE;
            continue;

        case UX_HOST_CLASS_HUB_ENUM_GET_HUB_DESC_DONE:

            /* Transfer request error check.  */
            if (UX_HOST_STACK_ENUM_TRANS_ERROR(trans))
            {
                hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_DONE;
                hub -> ux_host_class_hub_run_status = UX_DESCRIPTOR_CORRUPTED;
                continue;
            }

            /* Parse descriptor.  */
            status = _ux_host_class_hub_descriptor_parse(hub, hub -> ux_host_class_hub_allocated);
            if (status != UX_SUCCESS)
            {
                hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_DONE;
                hub -> ux_host_class_hub_run_status = status;
                continue;
            }

            /* Free the allocated descriptor.  */
            _ux_utility_memory_free(hub -> ux_host_class_hub_allocated);
            hub -> ux_host_class_hub_allocated = UX_NULL;

            /* Initialize for port power ON.  */
            hub -> ux_host_class_hub_run_port = 1;

            /* Fall through.  */
        case UX_HOST_CLASS_HUB_ENUM_PORT_POWER       :

            /* Prepare for SetPortFeature(POWER).  */
            status = _ux_host_class_hub_feature(hub,
                                hub -> ux_host_class_hub_run_port,
                                UX_SET_FEATURE, UX_HOST_CLASS_HUB_PORT_POWER);

            /* Request already updated, to transfer state.  */
            hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_TRANS_WAIT;
            hub -> ux_host_class_hub_next_state = UX_HOST_CLASS_HUB_ENUM_PORT_POWER_DELAY;
            continue;

        case UX_HOST_CLASS_HUB_ENUM_PORT_POWER_DELAY :

            /* Transfer request error check.  */
            if (UX_HOST_STACK_ENUM_TRANS_ERROR(trans))
            {

                /* Set the HUB status to not powered.  */
                hub -> ux_host_class_hub_port_power &=
                            (UINT)~(1u << hub -> ux_host_class_hub_run_port);
                hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_PORT_NEXT;
                continue;
            }

            /* Delay a while (as described by hub descriptor).  */
            hub -> ux_host_class_hub_wait_start = _ux_utility_time_get();
            hub -> ux_host_class_hub_wait_ms = UX_MS_TO_TICK_NON_ZERO(hub -> ux_host_class_hub_descriptor.bPwrOn2PwrGood << 1);

            hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_TRANS_WAIT;
            hub -> ux_host_class_hub_next_state = UX_HOST_CLASS_HUB_ENUM_PORT_POWER_ON;
            continue;

        case UX_HOST_CLASS_HUB_ENUM_PORT_POWER_ON:
            hub -> ux_host_class_hub_port_power |= (UINT)(1u << hub -> ux_host_class_hub_run_port);

            /* Fall through.  */
        case UX_HOST_CLASS_HUB_ENUM_PORT_NEXT        :

            /* Check if the last port is powered.  */
            if (hub -> ux_host_class_hub_run_port <
                hub -> ux_host_class_hub_descriptor.bNbPorts)
            {

                /* Start another port power.  */
                hub -> ux_host_class_hub_run_port ++;
                hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_PORT_POWER;
                continue;
            }

            /* All port is powered.  */

            /* Fall through.  */
        case UX_HOST_CLASS_HUB_ENUM_INTERRUPT_START  :
            status = _ux_host_class_hub_interrupt_endpoint_start(hub);
            if (status != UX_SUCCESS)
                hub -> ux_host_class_hub_run_status = status;

            /* Fall through.  */
        case UX_HOST_CLASS_HUB_ENUM_DONE             :

            /* Free buffer allocated while enumerating.  */
            if (hub -> ux_host_class_hub_allocated)
            {
                _ux_utility_memory_free(hub -> ux_host_class_hub_allocated);
                hub -> ux_host_class_hub_allocated = UX_NULL;
            }

            /* Error cases.  */
            if (hub -> ux_host_class_hub_run_status != UX_SUCCESS)
            {

                /* Unlink from device.  */
                device -> ux_device_class_instance = UX_NULL;

                /* Free hub.  */
                _ux_utility_memory_free(hub);

                /* Error trap. */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, hub -> ux_host_class_hub_run_status);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, hub -> ux_host_class_hub_run_status, hub, 0, 0, UX_TRACE_ERRORS, 0, 0)

                /* To error state.  */
                hub -> ux_host_class_hub_enum_state = UX_STATE_ERROR;
                return(UX_STATE_ERROR);
            }

            /* Done success.  */

            /* Create this class instance.  */
            _ux_host_stack_class_instance_create(hub -> ux_host_class_hub_class, (VOID *) hub);

            /* Store the instance in the device container, this is for the USBX stack
                when it needs to invoke the class.  */
            device -> ux_device_class_instance =  (VOID *) hub;

            /* Mark the HUB as live now.  */
            hub -> ux_host_class_hub_state =  UX_HOST_CLASS_INSTANCE_LIVE;

            /* If all is fine and the device is mounted, we may need to inform the application
                if a function has been programmed in the system structure.  */
            if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
            {

                /* Call system change function.  */
                _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, hub -> ux_host_class_hub_class, (VOID *) hub);
            }

            /* Next state: CHANGE_CHECK.  */
            hub -> ux_host_class_hub_enum_state = UX_STATE_IDLE;
            hub -> ux_host_class_hub_run_state = UX_HOST_CLASS_HUB_CHANGE_CHECK;
            hub -> ux_host_class_hub_run_port = 0;

            /* Done activate.  */
            return(UX_STATE_NEXT);

        case UX_HOST_CLASS_HUB_ENUM_TRANS_WAIT:

            /* Poll transfer task.  */
            status = _ux_host_stack_transfer_run(hub -> ux_host_class_hub_transfer);
            hub -> ux_host_class_hub_run_status = hub -> ux_host_class_hub_transfer ->
                                            ux_transfer_request_completion_code;

            /* Transfer done - next state.  */
            if (status == UX_STATE_NEXT || status == UX_STATE_IDLE)
            {
                hub -> ux_host_class_hub_enum_state = hub -> ux_host_class_hub_next_state;
                continue;
            }

            /* Check error.  */
            if (status < UX_STATE_NEXT)
            {

                /* Fail.  */
                hub -> ux_host_class_hub_enum_state = UX_HOST_CLASS_HUB_ENUM_DONE;
                continue;
            }

            /* Transfer in progress, wait.  */
            return(UX_STATE_WAIT);

        case UX_HOST_CLASS_HUB_ENUM_DELAY_WAIT:
            current_ms = _ux_utility_time_get();
            elapsed_ms = _ux_utility_time_elapsed(current_ms,
                                        hub -> ux_host_class_hub_wait_start);
            if (elapsed_ms < hub -> ux_host_class_hub_wait_ms)
            {

                /* Keep waiting.  */
                return(UX_STATE_WAIT);
            }

            /* Next state.  */
            hub -> ux_host_class_hub_enum_state = hub -> ux_host_class_hub_next_state;
            continue;

        default:
            return(UX_STATE_NEXT);
        }
    }
}
#endif
