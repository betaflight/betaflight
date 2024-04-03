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
/**   Device Stack                                                        */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE

/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_host_stack.h"

#if defined(UX_HOST_STANDALONE)


#define UX_HOST_STACK_ENUM_TRANS_ERROR(t)       (                               \
    (t)->ux_transfer_request_completion_code != UX_SUCCESS ||                   \
    (t)->ux_transfer_request_actual_length !=                                   \
        (t)->ux_transfer_request_requested_length)

static inline VOID _ux_host_stack_port_check_run(VOID);
static inline VOID _ux_host_stack_enum_run(VOID);
static inline VOID _ux_host_stack_pending_transfers_run(VOID);


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                                 RELEASE      */
/*                                                                        */
/*    _ux_host_stack_tasks_run                              PORTABLE C    */
/*                                                             6.2.0      */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function runs host stack and registered classes tasks.         */
/*                                                                        */
/*    It's valid only in standalone mode.                                 */
/*    It's non-blocking.                                                  */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion State Status                                             */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    (ux_hcd_entry_function)               HCD function                  */
/*    (ux_host_class_task_function)         Host class task function      */
/*    (ux_system_host_change_function)      Host change callback function */
/*    (ux_system_host_enum_hub_function)    Host hub enumeration function */
/*    _ux_host_stack_rh_change_process      Host Root Hub process         */
/*    _ux_host_stack_device_address_set     Start process to set address  */
/*    _ux_host_stack_configuration_set      Start process to set config   */
/*    _ux_host_stack_device_descriptor_read Start process to read device  */
/*                                          descriptor                    */
/*    _ux_host_stack_new_configuration_create                             */
/*                                          Link a new configuration to   */
/*                                          a device                      */
/*    _ux_host_stack_configuration_instance_create                        */
/*                                          Create configuration instance */
/*    _ux_host_stack_transfer_run           Runs transfer state machine   */
/*    _ux_host_stack_class_device_scan      Query device class driver     */
/*    _ux_host_stack_class_interface_scan   Query interface class driver  */
/*    _ux_host_stack_interfaces_scan        Parse interfaces in a         */
/*                                          configuration                 */
/*    _ux_host_stack_device_remove          Remove a device               */
/*    _ux_utility_descriptor_parse          Parse a descriptor            */
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_free               Free memory                   */
/*    _ux_utility_time_get                  Get current time tick         */
/*    _ux_system_error_handler              Error trap                    */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone hub,       */
/*                                            used shared descriptor in   */
/*                                            device instance for enum,   */
/*                                            resulting in version 6.1.12 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved internal logic,    */
/*                                            fixed activation issue on   */
/*                                            no class linked interfaces, */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT _ux_host_stack_tasks_run(VOID)
{

UX_HCD *hcd_inst;
ULONG hcd_index;
UX_HOST_CLASS *class_inst;
ULONG class_index;

    /* =========== Run all HCD tasks.  */
    for (hcd_index = 0; hcd_index < UX_SYSTEM_HOST_MAX_HCD_GET(); hcd_index++)
    {
        hcd_inst = &_ux_system_host->ux_system_host_hcd_array[hcd_index];
        if (hcd_inst -> ux_hcd_status != UX_HCD_STATUS_OPERATIONAL ||
            hcd_inst -> ux_hcd_entry_function == UX_NULL)
            continue;
        hcd_inst -> ux_hcd_entry_function(hcd_inst, UX_HCD_TASKS_RUN, UX_NULL);
    }

    /* =========== Run port check process.  */
    _ux_host_stack_port_check_run();

    /* =========== Run enumeration process.  */
    _ux_host_stack_enum_run();

    /* =========== Run classes tasks.  */
    for (class_index = 0; class_index < UX_SYSTEM_HOST_MAX_CLASS_GET(); class_index++)
    {
        class_inst = &_ux_system_host->ux_system_host_class_array[class_index];
        if ((class_inst -> ux_host_class_status == UX_UNUSED) ||
            (class_inst -> ux_host_class_task_function == UX_NULL))
            continue;
        class_inst -> ux_host_class_task_function(class_inst);
    }

    /* =========== Run pending transfer tasks.  */
    _ux_host_stack_pending_transfers_run();

    /* =========== Run background tasks.  */
    if (_ux_system_host -> ux_system_host_change_function)
    {
        _ux_system_host -> ux_system_host_change_function(
                        UX_STANDALONE_WAIT_BACKGROUND_TASK, UX_NULL, UX_NULL);
    }

    /* No idle report support now.  */
    return (UX_STATE_WAIT);
}

static inline VOID _ux_host_stack_port_check_run(VOID)
{
#if UX_MAX_DEVICES > 1

    /* We try the hub first. For this we look into the USBX project
        structure to see if there is at least one hub.  */
    if (_ux_system_host->ux_system_host_enum_hub_function != UX_NULL)
    {

        /* Yes, there is a HUB function, call it!  */
        _ux_system_host->ux_system_host_enum_hub_function();
    }
#endif

    /* Check root hub changes.  */
    _ux_host_stack_rh_change_process();
}

static inline UINT _ux_host_stack_rh_port_enable(UX_DEVICE *device)
{
UX_HCD          *hcd = UX_DEVICE_HCD_GET(device);
ALIGN_TYPE      port_index = (ALIGN_TYPE)UX_DEVICE_PORT_LOCATION_GET(device);
    return hcd -> ux_hcd_entry_function(hcd, UX_HCD_ENABLE_PORT, (VOID *)port_index);
}
static inline UINT _ux_host_stack_rh_port_reset(UX_DEVICE *device)
{
UX_HCD          *hcd = UX_DEVICE_HCD_GET(device);
ALIGN_TYPE      port_index = (ALIGN_TYPE)UX_DEVICE_PORT_LOCATION_GET(device);
    return hcd -> ux_hcd_entry_function(hcd, UX_HCD_RESET_PORT, (VOID *)port_index);
}
static inline UINT _ux_host_stack_rh_port_status_get(UX_DEVICE *device)
{
UX_HCD          *hcd = UX_DEVICE_HCD_GET(device);
ALIGN_TYPE      port_index = (ALIGN_TYPE)UX_DEVICE_PORT_LOCATION_GET(device);
    return hcd -> ux_hcd_entry_function(hcd, UX_HCD_GET_PORT_STATUS, (VOID *)port_index);
}
static inline VOID _ux_host_stack_enum_address_sent(UX_DEVICE *device)
{

    /* Address (request wValue) applied to device.  */
    device -> ux_device_address =
            device -> ux_device_enum_trans -> ux_transfer_request_value;
    device -> ux_device_state = UX_DEVICE_ADDRESSED;

    /* Unlock enumeration.  */
    _ux_system_host -> ux_system_host_enum_lock = UX_NULL;
}
static inline VOID _ux_host_stack_enum_device_descriptor_parse0(
    UX_DEVICE *device, UCHAR *descriptor)
{

    /* Endpoint descriptor updated.  */
    device -> ux_device_descriptor.bMaxPacketSize0 = descriptor[7];

    /* Update EP0 wMaxPacketSize.  */
    device -> ux_device_control_endpoint.
                ux_endpoint_descriptor.wMaxPacketSize = descriptor[7];
}
static inline VOID _ux_host_stack_enum_configuration_descriptor_parse0(
    UX_DEVICE *device, UX_CONFIGURATION *configuration, UCHAR *descriptor)
{
    /* This configuration must be linked to the device.  */
    _ux_host_stack_new_configuration_create(device, configuration);

    /* The descriptor is in a packed format, parse it locally.  */
    _ux_utility_descriptor_parse(descriptor,
        _ux_system_configuration_descriptor_structure,
        UX_CONFIGURATION_DESCRIPTOR_ENTRIES,
        (UCHAR *) &configuration -> ux_configuration_descriptor);
}
static inline UINT _ux_host_stack_enum_configuration_read(UX_DEVICE *device)
{

UX_CONFIGURATION    *configuration;
UCHAR               *descriptor;
UX_ENDPOINT         *control_endpoint;
UX_TRANSFER         *transfer_request;

    /* Allocate configuration containter for the descriptor.  */
    configuration = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY,
                                                    sizeof(UX_CONFIGURATION));
    if (configuration == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Need to allocate memory for the configuration descriptor the first time we read
       only the configuration descriptor when we have the configuration descriptor, we have
       the length of the entire configuration\interface\endpoint descriptors.  */
    descriptor =  _ux_utility_memory_allocate(UX_SAFE_ALIGN,
                    UX_CACHE_SAFE_MEMORY, UX_CONFIGURATION_DESCRIPTOR_LENGTH);
    if (descriptor == UX_NULL)
    {
        _ux_utility_memory_free(configuration);
        return(UX_MEMORY_INSUFFICIENT);
    }

    /* Save allocated configuration container.  */
    device -> ux_device_enum_inst.configuration = configuration;

    /* Retrieve the pointer to the control endpoint and its transfer_request.  */
    control_endpoint =  &device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Create a transfer_request for the GET_DESCRIPTOR request.  */
    transfer_request -> ux_transfer_request_data_pointer =      descriptor;
    transfer_request -> ux_transfer_request_requested_length =  UX_CONFIGURATION_DESCRIPTOR_LENGTH;
    transfer_request -> ux_transfer_request_function =          UX_GET_DESCRIPTOR;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN |
                                                                UX_REQUEST_TYPE_STANDARD |
                                                                UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value =             (UINT)device -> ux_device_enum_index |
                                                                (UINT)(UX_CONFIGURATION_DESCRIPTOR_ITEM << 8);
    transfer_request -> ux_transfer_request_index =             0;

    /* Transfer for wait.  */
    UX_TRANSFER_STATE_RESET(transfer_request);
    device -> ux_device_enum_trans = transfer_request;

    return(UX_SUCCESS);
}
static inline VOID _ux_host_stack_configuration_parsed(UX_DEVICE *device)
{
UINT            status;

    /* By default (error) configuration not activated.  */
    device -> ux_device_enum_state = UX_HOST_STACK_ENUM_FAIL;

    /* Query device class driver.  */
    status =  _ux_host_stack_class_device_scan(device);
    if (status == UX_SUCCESS)
    {

        /* Prepare device activation.  */
        device -> ux_device_enum_index = 0xFF;
        device -> ux_device_enum_state = UX_HOST_STACK_ENUM_ACTIVATE;
        device -> ux_device_enum_inst.device = device;
    }
    else if (status == UX_NO_CLASS_MATCH)
    {

        /* Query interface class driver.  */
        status =  _ux_host_stack_class_interface_scan(device);
        if (status == UX_SUCCESS)
        {

            /* Prepare interfaces activation.  */
            device -> ux_device_enum_index = 0;
            device -> ux_device_enum_state = UX_HOST_STACK_ENUM_CONFIG_SET;
            device -> ux_device_enum_inst.configuration =
                                        device -> ux_device_first_configuration;
        }
    }
}

static inline VOID _ux_host_stack_device_enumerated(UX_DEVICE *device)
{

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_DEVICE,
            UX_DEVICE_HCD_GET(device), UX_DEVICE_PARENT_GET(device),
            UX_DEVICE_PORT_LOCATION_GET(device), 0);

    /* Check if there is unnecessary resource to free.  */
    if (device -> ux_device_packed_configuration &&
        device -> ux_device_packed_configuration_keep_count == 0)
    {
        _ux_utility_memory_free(device -> ux_device_packed_configuration);
        device -> ux_device_packed_configuration = UX_NULL;
    }

    /* Reset enumeration state.  */
    device -> ux_device_enum_state = UX_STATE_IDLE;

    /* Unlock device EP0.  */
    device -> ux_device_control_endpoint.ux_endpoint_transfer_request.
                        ux_transfer_request_flags &= ~UX_TRANSFER_FLAG_LOCK;

    /* Disconnect from enumeration list.  */

    /* Clear enumeration flag to stop enumeration sequence.  */
    device -> ux_device_flags &= ~UX_DEVICE_FLAG_ENUM;

    /* If HCD is dead or device disconnected, free device.  */
    if (UX_DEVICE_HCD_GET(device) -> ux_hcd_status != UX_HCD_STATUS_OPERATIONAL ||
        (device -> ux_device_enum_port_status & UX_PS_CCS) == 0)
    {
        _ux_host_stack_device_remove(UX_DEVICE_HCD_GET(device),
                                     UX_DEVICE_PARENT_GET(device),
                                     UX_DEVICE_PORT_LOCATION_GET(device));

        /* Done without notification.  */
        return;
    }

    /* Device connected and enumerated success/fail.  */

    /* For root hub, set root hub connection status.  */
    if (UX_DEVICE_PARENT_IS_ROOTHUB(device))
    {
        UX_DEVICE_HCD_GET(device) -> ux_hcd_rh_device_connection |=
                    (ULONG)(1 << UX_DEVICE_PORT_LOCATION_GET(device));
    }

    /* If change function available, notify device connection.  */
    if (_ux_system_host -> ux_system_host_change_function)
    {
        _ux_system_host -> ux_system_host_change_function(
                        UX_DEVICE_CONNECTION, UX_NULL, (VOID*)device);
    }
}

static inline UINT _ux_host_stack_interface_activate_wait(UX_DEVICE *device,
    UX_HOST_CLASS_COMMAND *command)
{
UINT            status;
UX_INTERFACE    *interface_inst;

    command -> ux_host_class_command_container = device -> ux_device_enum_inst.ptr;
    command -> ux_host_class_command_class_ptr = (device -> ux_device_enum_index == 0xFF) ?
                device -> ux_device_class :
                device -> ux_device_enum_inst.interface -> ux_interface_class;
    command -> ux_host_class_command_request = UX_HOST_CLASS_COMMAND_ACTIVATE_WAIT;

    /* If there is no class linked, just next state.  */
    if (command -> ux_host_class_command_class_ptr != UX_NULL)
        status = command -> ux_host_class_command_class_ptr -> ux_host_class_entry_function(command);
    else
        status = UX_STATE_NEXT;

    /* No wait command or ready for next state.  */
    if (status == UX_FUNCTION_NOT_SUPPORTED ||
        status == UX_STATE_NEXT)
    {

        /* Activate for device class driver.  */
        if (device -> ux_device_enum_index == 0xFF)
        {

            /* Enumeration done (class driver controls device configuration).  */
            device -> ux_device_enum_state = UX_HOST_STACK_ENUM_DONE;
            return(UX_STATE_NEXT);
        }
        else
        {

            /* Activate next interface (default setting).  */
            interface_inst = device -> ux_device_enum_inst.interface -> ux_interface_next_interface;
            while(interface_inst != UX_NULL &&
                interface_inst -> ux_interface_descriptor.bAlternateSetting != 0)
            {
                interface_inst = interface_inst -> ux_interface_next_interface;
            }
            if (interface_inst != UX_NULL)
            {
                device -> ux_device_enum_inst.interface = interface_inst;
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_ACTIVATE;
                return(UX_STATE_NEXT);
            }

            /* Enumeration is done.  */
            device -> ux_device_enum_state = UX_HOST_STACK_ENUM_DONE;
            return(UX_STATE_NEXT);
        }
    }

    /* Error cases.  */
    if (status < UX_STATE_NEXT)
    {
        device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
        return(UX_STATE_NEXT);
    }

    /* Keep waiting.  */
    return(UX_STATE_WAIT);
}


static inline VOID _ux_host_stack_device_enum_run(UX_DEVICE *device)
{
UX_INTERRUPT_SAVE_AREA
UINT                    status;
ULONG                   current_ms, elapsed_ms;
UX_TRANSFER             *trans;
UX_CONFIGURATION        *configuration;
UX_HOST_CLASS_COMMAND   class_command;
UCHAR                   *buffer;
INT                     immediate_state = UX_TRUE;

    /* Check if the device enumeration should be processed.  */
    if ((device -> ux_device_flags & UX_DEVICE_FLAG_ENUM) == 0)
        return;

    /* Enumeration lock check, devices not addressed should wait.  */
    UX_DISABLE
    if (_ux_system_host -> ux_system_host_enum_lock &&
        device -> ux_device_address == 0)
    {

        /* Check if the device is the one locks, others should wait.  */
        if (device != _ux_system_host -> ux_system_host_enum_lock)
        {

            /* Wait, return.  */
            UX_RESTORE
            return;
        }
    }
    UX_RESTORE

    while (immediate_state)
    {
        switch (device -> ux_device_enum_state)
        {
        case UX_STATE_RESET:

            /* Reset retry counts.  */
            device -> ux_device_enum_retry = UX_RH_ENUMERATION_RETRY;

            device -> ux_device_enum_state = UX_HOST_STACK_ENUM_PORT_ENABLE;
            device -> ux_device_enum_next_state = UX_HOST_STACK_ENUM_PORT_ENABLE;
            device -> ux_device_enum_port_status = UX_PS_CCS;

            /* Fall through.  */
        case UX_HOST_STACK_ENUM_PORT_ENABLE:

            /* Lock enumeration any way.  */
            _ux_system_host -> ux_system_host_enum_lock = device;

#if UX_MAX_DEVICES > 1
            if (UX_DEVICE_PARENT_IS_HUB(device))
            {

                /* Issue a port reset on hub side.  */
                device -> ux_device_flags |= UX_DEVICE_FLAG_RESET;
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_HUB_OPERATION_WAIT;
                return;
            }
#endif

            /* For device connected to roohub, we may need port enable (OHCI).  */
            status = _ux_host_stack_rh_port_enable(device);
            if (status == UX_PORT_INDEX_UNKNOWN)
            {

                /* No retry, enumeration fail.  */
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_FAIL;
                continue;
            }

            /* Wait a while after port connection.  */
            device -> ux_device_enum_next_state = UX_HOST_STACK_ENUM_PORT_RESET;
            device -> ux_device_enum_state = UX_HOST_STACK_ENUM_WAIT;
            device -> ux_device_enum_wait_start = _ux_utility_time_get();
            device -> ux_device_enum_wait_ms =
                        UX_MS_TO_TICK_NON_ZERO(UX_RH_ENUMERATION_RETRY_DELAY);
            continue;

#if UX_MAX_DEVICES > 1
        case UX_HOST_STACK_ENUM_HUB_OPERATION_WAIT:

            /* Keep waiting, state is changed in hub tasks.  */
            return;
#endif

        case UX_HOST_STACK_ENUM_PORT_RESET:

            /* Reset may blocking, wait the reset done.  */
            /* Fall through.  */
        case UX_HOST_STACK_ENUM_PORT_RESET_WAIT:

            /* Run states for reset.  */
            status = _ux_host_stack_rh_port_reset(device);

            /* Check fatal error (exit enumeration).  */
            if (status < UX_STATE_ERROR)
            {

                /* Fail no retry.  */
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_FAIL;
                continue;
            }

            /* Check error.  */
            if (status == UX_STATE_ERROR)
            {

                /* Fail retry.  */
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
                continue;
            }

            /* Ready for next state.  */
            if (status == UX_STATE_NEXT)
            {

                /* Return device address to 0.  */
#if UX_MAX_DEVICES > 1
                if (device -> ux_device_address)
                {

                    /* Free the address.  */
                    UX_DEVICE_HCD_GET(device) ->
                    ux_hcd_address[(device -> ux_device_address-1) >> 3] &=
                        (UCHAR)(1u << ((device -> ux_device_address-1) & 7u));
                }
#endif

                /* Get port status.  */
                status = _ux_host_stack_rh_port_status_get(device);
                if (status == UX_PORT_INDEX_UNKNOWN)
                {

                    /* Fail no retry: No connection.  */
                    device -> ux_device_enum_port_status = 0;
                    device -> ux_device_enum_state = UX_HOST_STACK_ENUM_FAIL;
                    continue;
                }

                /* Save port status, next : SetAddress.  */
                device -> ux_device_enum_port_status = (UCHAR)status;
                device -> ux_device_enum_trans = &device ->
                        ux_device_control_endpoint.ux_endpoint_transfer_request;
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_TRANS_LOCK_WAIT;
                device -> ux_device_enum_next_state = UX_HOST_STACK_ENUM_DEVICE_ADDR_SET;
                continue;
            }

            /* Keep waiting.  */
            return;

        case UX_HOST_STACK_ENUM_DEVICE_ADDR_SET:

            /* Check port connection status.  */
            if ((device -> ux_device_enum_port_status & UX_PS_CCS) == 0)
            {

                /* Port disconnected, no retry, just fail.  */
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_FAIL;
                continue;
            }

            /* Set device speed.  */
            device -> ux_device_speed = device -> ux_device_enum_port_status >> UX_PS_DS;

            /* Reset bMaxPacketSize0.  */
            device -> ux_device_descriptor.bMaxPacketSize0 = 0;

            /* Start SetAddress().  */
            status = _ux_host_stack_device_address_set(device);
            if (status != UX_SUCCESS)
            {

                /* Request fail, retry.  */
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
                continue;
            }

            /* Wait request progress done.  */
            UX_TRANSFER_STATE_RESET(device -> ux_device_enum_trans);
            device -> ux_device_enum_state = UX_HOST_STACK_ENUM_TRANS_WAIT;
            device -> ux_device_enum_next_state = UX_HOST_STACK_ENUM_DEVICE_ADDR_SENT;
            continue;

        case UX_HOST_STACK_ENUM_DEVICE_ADDR_SENT:

            /* Transfer error check.  */
            trans = device -> ux_device_enum_trans;
            if (trans -> ux_transfer_request_completion_code != UX_SUCCESS)
            {
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
                continue;
            }

            /* Address is already sent to device.  */
            _ux_host_stack_enum_address_sent(device);

            /* Wait at least 2ms after address is sent.  */
            device -> ux_device_enum_next_state = UX_HOST_STACK_ENUM_DEVICE_DESCR_READ;
            device -> ux_device_enum_state = UX_HOST_STACK_ENUM_WAIT;
            device -> ux_device_enum_wait_start = _ux_utility_time_get();
            device -> ux_device_enum_wait_ms = UX_MS_TO_TICK_NON_ZERO(2);
            continue;

        case UX_HOST_STACK_ENUM_DEVICE_DESCR_READ:

            /* Start GetDescriptor(Device).  */
            status = _ux_host_stack_device_descriptor_read(device);
            if (status != UX_SUCCESS)
            {
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
                continue;
            }
            UX_TRANSFER_STATE_RESET(device -> ux_device_enum_trans);
            device -> ux_device_enum_state = UX_HOST_STACK_ENUM_TRANS_WAIT;
            device -> ux_device_enum_next_state = UX_HOST_STACK_ENUM_DEVICE_DESCR_PARSE;
            continue;

        case UX_HOST_STACK_ENUM_DEVICE_DESCR_PARSE:

            /* Transfer error check.  */
            trans = device -> ux_device_enum_trans;
            buffer = trans -> ux_transfer_request_data_pointer;
            if (UX_HOST_STACK_ENUM_TRANS_ERROR(trans))
            {
                _ux_utility_memory_free(buffer);
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
                continue;
            }

            /* Device bMaxPacketSize0 not available, update it.  */
            if(device -> ux_device_descriptor.bMaxPacketSize0 == 0)
            {
                /* Validate the bMaxPacketSize0.  */
                if (buffer[7] != 8 && buffer[7] != 16 && buffer[7] != 32 && buffer[7] != 64)
                {

                    _ux_utility_memory_free(buffer);
                    device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
                    continue;
                }

                /* Validate the USB-IF bDeviceClass class code.  */
#if defined(UX_HOST_DEVICE_CLASS_CODE_VALIDATION_ENABLE)
                switch(buffer[4])
                {
                case 0x00: /* Fall through.  */
                case 0x02: /* Fall through.  */
                case 0x09: /* Fall through.  */
                case 0x11: /* Fall through.  */
                case 0xDC: /* Fall through.  */
                case 0xEF: /* Fall through.  */
                case 0xFF:
                    break;
                default:

                    /* Invalid device class code.  */
                    _ux_utility_memory_free(buffer);
                    device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
                    continue;
                }
#endif

                _ux_host_stack_enum_device_descriptor_parse0(device, buffer);

                /* Update SETUP wLength.  */
                trans -> ux_transfer_request_requested_length = UX_DEVICE_DESCRIPTOR_LENGTH;

                /* Issue GetDescriptor(Device) again and parse it.  */
                UX_TRANSFER_STATE_RESET(trans);
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_TRANS_WAIT;
                device -> ux_device_enum_next_state = UX_HOST_STACK_ENUM_DEVICE_DESCR_PARSE;
                continue;
            }

            /* Parse the device descriptor.  */
            _ux_utility_descriptor_parse(buffer,
                _ux_system_device_descriptor_structure, UX_DEVICE_DESCRIPTOR_ENTRIES,
                (UCHAR *) &device -> ux_device_descriptor);
            _ux_utility_memory_free(buffer);
            trans -> ux_transfer_request_data_pointer = UX_NULL;

            /* Start configuration enumeration, from index 0.  */
            device -> ux_device_enum_index = 0;

            /* Fall through.  */
        case UX_HOST_STACK_ENUM_CONFIG_DESCR_READ:

            /* Start GetDescriptor(Configuration, 9).  */
            status = _ux_host_stack_enum_configuration_read(device);
            if (status != UX_SUCCESS)
            {
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
                continue;
            }
            device -> ux_device_enum_state = UX_HOST_STACK_ENUM_TRANS_WAIT;
            device -> ux_device_enum_next_state = UX_HOST_STACK_ENUM_CONFIG_DESCR_PARSE;

            /* In this case, there are two memory blocks allocated floating:
               - UX_CONFIGURATION
               - configuration descriptor RX buffer.  */
            continue;

        case UX_HOST_STACK_ENUM_CONFIG_DESCR_PARSE:

            /* Transfer error check.  */
            trans = device -> ux_device_enum_trans;
            buffer = trans -> ux_transfer_request_data_pointer;
            if (UX_HOST_STACK_ENUM_TRANS_ERROR(trans))
            {
                _ux_utility_memory_free(buffer);
                _ux_utility_memory_free(device -> ux_device_enum_inst.ptr);
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
                continue;
            }

            /* Load current parsing configuration.  */
            configuration = device -> ux_device_enum_inst.configuration;

            /* Get configuration descriptor only.  */
            if (trans -> ux_transfer_request_actual_length == UX_CONFIGURATION_DESCRIPTOR_LENGTH)
            {

                /* Parse the configuration descriptor and free descriptor buffer.  */
                _ux_host_stack_enum_configuration_descriptor_parse0(device, configuration, buffer);

                /* Release descriptor buffer.  */
                _ux_utility_memory_free(buffer);
                trans -> ux_transfer_request_data_pointer = UX_NULL;

                /* If there is no interface for the configuration, check next.  */
                if (configuration -> ux_configuration_descriptor.wTotalLength ==
                    UX_CONFIGURATION_DESCRIPTOR_LENGTH)
                {
                    device -> ux_device_enum_state = UX_HOST_STACK_ENUM_CONFIG_DESCR_NEXT;
                    continue;
                }

                /* Allocate new buffer for total configuration.  */
                buffer = _ux_utility_memory_allocate(UX_SAFE_ALIGN,
                    UX_CACHE_SAFE_MEMORY,
                    configuration -> ux_configuration_descriptor.wTotalLength);
                if (buffer == UX_NULL)
                {
                    device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
                    continue;
                }
                trans -> ux_transfer_request_data_pointer = buffer;

                /* Modify transfer length for total configuration  */
                trans -> ux_transfer_request_requested_length =
                    configuration -> ux_configuration_descriptor.wTotalLength;

                /* Start transfer again.  */
                UX_TRANSFER_STATE_RESET(device -> ux_device_enum_trans);
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_TRANS_WAIT;
                device -> ux_device_enum_next_state = UX_HOST_STACK_ENUM_CONFIG_DESCR_PARSE;

                /* In this case, there is one memory blocks allocated floating:
                    - configuration descriptor RX buffer.  */
                continue;
            }

            /* Parse configuration and it's interfaces.  */
            status = _ux_host_stack_interfaces_scan(configuration, buffer);

            /* Release descriptor memory.  */
            _ux_utility_memory_free(buffer);
            trans -> ux_transfer_request_data_pointer = UX_NULL;

            /* Check operation status.  */
            if (status != UX_SUCCESS)
            {
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
                continue;
            }

            /* Enumerate next configuration.  */
            /* Fall through.  */
        case UX_HOST_STACK_ENUM_CONFIG_DESCR_NEXT:
            device -> ux_device_enum_index ++;
            if (device -> ux_device_enum_index <
                device -> ux_device_descriptor.bNumConfigurations)
            {
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_CONFIG_DESCR_READ;
                continue;
            }

            /* All configurations are enumerated.  */
            _ux_host_stack_configuration_parsed(device);

            /* Roll back to next state.  */
            continue;

        case UX_HOST_STACK_ENUM_CONFIG_SET:

            /* The device is now in the unconfigured state. We need to deal
               with the amount of power the device is consuming before allowing
               it to be configured. Otherwise we may run the risk of an over
               current fault. */        
            configuration = device -> ux_device_enum_inst.configuration;
            if (((configuration -> ux_configuration_descriptor.bmAttributes &
                    UX_CONFIGURATION_DEVICE_SELF_POWERED) == 0) &&
                 (configuration -> ux_configuration_descriptor.MaxPower >
                    UX_DEVICE_MAX_POWER_GET(device)))
            {

                /* Error trap. */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_OVER_CURRENT_CONDITION);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_OVER_CURRENT_CONDITION, configuration, 0, 0, UX_TRACE_ERRORS, 0, 0)

                /* Enumeration fail without retry.  */
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_FAIL;
                continue;
            }

            /* Prepare transfer request SetConfiguration().  */
            _ux_host_stack_configuration_set(configuration);

            /* Roll back to start transfer wait.  */
            UX_TRANSFER_STATE_RESET(device -> ux_device_enum_trans);
            device -> ux_device_enum_next_state = UX_HOST_STACK_ENUM_CONFIG_ACTIVATE;
            device -> ux_device_enum_state = UX_HOST_STACK_ENUM_TRANS_WAIT;
            continue;

        case UX_HOST_STACK_ENUM_CONFIG_ACTIVATE:
            trans = device -> ux_device_enum_trans;
            if (UX_HOST_STACK_ENUM_TRANS_ERROR(trans))
            {
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
                continue;
            }

            /* Set device state to configured.  */
            configuration = device -> ux_device_enum_inst.configuration;
            device -> ux_device_state = UX_DEVICE_CONFIGURED;
            device -> ux_device_current_configuration = configuration;

            /* Create the configuration instance.  */
            status =  _ux_host_stack_configuration_instance_create(configuration);
            if (status != UX_SUCCESS)
            {
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
                continue;
            }

            if (configuration -> ux_configuration_first_interface)
            {

                /* Activate classes.  */
                device -> ux_device_enum_index = 0;
                device -> ux_device_enum_inst.interface =
                            configuration -> ux_configuration_first_interface;
            }
            else
            {

                /* No activation, done  */
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_DONE;
                continue;
            }

            /* Fall through.  */
        case UX_HOST_STACK_ENUM_ACTIVATE:

            /* Class activate.  */
            class_command.ux_host_class_command_request = UX_HOST_CLASS_COMMAND_ACTIVATE_START;
            if (device -> ux_device_enum_index == 0xFF)
            {

                /* Device class driver.  */
                class_command.ux_host_class_command_container = device;
                class_command.ux_host_class_command_class_ptr = device -> ux_device_class;
            }
            else
            {

                /* Interface class driver.  */
                class_command.ux_host_class_command_container = device -> ux_device_enum_inst.ptr;
                class_command.ux_host_class_command_class_ptr =
                                    device -> ux_device_enum_inst.interface -> ux_interface_class;
            }

            /* If there class linked, start activation.  */
            if (class_command.ux_host_class_command_class_ptr != UX_NULL)
            {
                status = class_command.ux_host_class_command_class_ptr ->
                                    ux_host_class_entry_function(&class_command);
                if (status != UX_SUCCESS)
                {
                    device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
                    continue;
                }
            }

            /* Class activate execute wait.  */
            device -> ux_device_enum_state = UX_HOST_STACK_ENUM_ACTIVATE_WAIT;

            /* Fall through.  */
        case UX_HOST_STACK_ENUM_ACTIVATE_WAIT:

            /* Class activate execution wait.  */
            status = _ux_host_stack_interface_activate_wait(device, &class_command);

            /* Next state.  */
            if (status != UX_STATE_WAIT)
                continue;

            /* Keep waiting.  */
            return;

        case UX_HOST_STACK_ENUM_TRANS_WAIT:

            /* Poll transfer task.  */
            trans = device -> ux_device_enum_trans;
            status = _ux_host_stack_transfer_run(trans);

            /* Transfer done - next state.  */
            if (status == UX_STATE_NEXT || status == UX_STATE_IDLE)
            {
                device -> ux_device_enum_state = device -> ux_device_enum_next_state;
                continue;
            }

            /* Check error.  */
            if (status < UX_STATE_ERROR)
            {

                /* No retry, fail.  */
                if (trans -> ux_transfer_request_data_pointer)
                {
                    _ux_utility_memory_free(trans -> ux_transfer_request_data_pointer);
                    trans -> ux_transfer_request_data_pointer = UX_NULL;
                }
                if (device -> ux_device_enum_next_state == UX_HOST_STACK_ENUM_CONFIG_DESCR_PARSE)
                {
                    _ux_utility_memory_free(device -> ux_device_enum_inst.ptr);
                    device -> ux_device_enum_inst.ptr = UX_NULL;
                }
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_FAIL;
                continue;
            }
            if (status < UX_STATE_NEXT)
            {

                /* Error, retry.  */
                if (trans -> ux_transfer_request_data_pointer)
                {
                    _ux_utility_memory_free(trans -> ux_transfer_request_data_pointer);
                    trans -> ux_transfer_request_data_pointer = UX_NULL;
                }
                if (device -> ux_device_enum_next_state == UX_HOST_STACK_ENUM_CONFIG_DESCR_PARSE)
                {
                    _ux_utility_memory_free(device -> ux_device_enum_inst.ptr);
                    device -> ux_device_enum_inst.ptr = UX_NULL;
                }
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_RETRY;
                continue;
            }

            /* Transfer in progress, wait.  */
            return;

        case UX_HOST_STACK_ENUM_TRANS_LOCK_WAIT:
            trans = device -> ux_device_enum_trans;
            UX_DISABLE
            if (trans -> ux_transfer_request_flags & UX_TRANSFER_FLAG_LOCK)
            {

                /* Keep waiting.  */
                UX_RESTORE;
                return;
            }

            /* Lock it.  */
            trans -> ux_transfer_request_flags |= UX_TRANSFER_FLAG_LOCK;
            UX_RESTORE

            /* Apply next expected state.  */
            device -> ux_device_enum_state = device -> ux_device_enum_next_state;
            continue;

        case UX_HOST_STACK_ENUM_WAIT:
            current_ms = _ux_utility_time_get();
            elapsed_ms = _ux_utility_time_elapsed(current_ms,
                                                  device -> ux_device_enum_wait_start);
            if (elapsed_ms < device -> ux_device_enum_wait_ms)

                /* Keep waiting.  */
                return;

            /* Next pre-defined state.  */
            device -> ux_device_enum_state = device -> ux_device_enum_next_state;
            continue;

        case UX_HOST_STACK_ENUM_RETRY:

            /* Check remaining retry count. */
            if (device -> ux_device_enum_retry > 0)
            {
                device -> ux_device_enum_retry --;

                /* Check if there is unnecessary resource to free.  */
                if (device -> ux_device_packed_configuration &&
                    device -> ux_device_packed_configuration_keep_count == 0)
                {
                    _ux_utility_memory_free(device -> ux_device_packed_configuration);
                    device -> ux_device_packed_configuration = UX_NULL;
                }

                /* Start from port enable delay.  */
                device -> ux_device_enum_next_state = UX_HOST_STACK_ENUM_PORT_RESET;
                device -> ux_device_enum_state = UX_HOST_STACK_ENUM_WAIT;
                device -> ux_device_enum_wait_start = _ux_utility_time_get();
                device -> ux_device_enum_wait_ms =
                        UX_MS_TO_TICK_NON_ZERO(UX_RH_ENUMERATION_RETRY_DELAY);
                continue;
            }

            /* Tried several times, fail case.  */
            /* Fall through.  */
        case UX_HOST_STACK_ENUM_FAIL:

            /* Clear lock anyway.  */
            if (device == _ux_system_host -> ux_system_host_enum_lock)
                _ux_system_host -> ux_system_host_enum_lock = UX_NULL;

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD,
                UX_DEVICE_PARENT_IS_HUB(device) ? UX_SYSTEM_CONTEXT_HUB :
                                                  UX_SYSTEM_CONTEXT_ROOT_HUB,
                UX_DEVICE_ENUMERATION_FAILURE);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DEVICE_ENUMERATION_FAILURE,
                UX_DEVICE_PORT_LOCATION_GET(device), 0, 0, UX_TRACE_ERRORS, 0, 0);

            /* Fall through.  */
        case UX_HOST_STACK_ENUM_DONE:
            _ux_host_stack_device_enumerated(device);

            /* We are done now.  */
            /* Fall through.  */
        case UX_HOST_STACK_ENUM_IDLE:

            /* Nothing to run.  */
            return;

        default:

            /* Invalid state, reset.  */
            device -> ux_device_enum_state = UX_STATE_RESET;
        }

        /* Invalid unhandled state.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HOST_STACK, UX_INVALID_STATE);

        /* Break the immediate state loop.  */
        immediate_state = UX_FALSE;
    }
}

static inline VOID _ux_host_stack_enum_run(VOID)
{

UX_DEVICE           *enum_device;

    /* Check if there is device pending enumeration.  */
    enum_device = _ux_system_host -> ux_system_host_enum_device;

    /* Run enumeration task for each device.  */
    while(enum_device != UX_NULL)
    {

        /* Run enumeration task on the device.  */
        if ((enum_device -> ux_device_flags & UX_DEVICE_FLAG_PROTECT) == 0)
        {
            enum_device -> ux_device_flags |= UX_DEVICE_FLAG_PROTECT;
            _ux_host_stack_device_enum_run(enum_device);
            enum_device -> ux_device_flags &= ~UX_DEVICE_FLAG_PROTECT;
        }

        /* Check device lock.  */
        if (enum_device -> ux_device_flags & UX_DEVICE_FLAG_LOCK)
        {
            break;
        }

        /* Check next enumerating device.  */
        enum_device = enum_device -> ux_device_enum_next;
    }
}

static inline VOID _ux_host_stack_pending_transfers_run(VOID)
{
UX_TRANSFER         *transfer, *next;
    transfer = _ux_system_host -> ux_system_host_pending_transfers;
    while(transfer)
    {
        next = transfer -> ux_transfer_request_next_pending;
        _ux_host_stack_transfer_run(transfer);
        if (transfer == next || _ux_system_host -> ux_system_host_pending_transfers == next)
            break;
        transfer = next;
    }
}
#endif
