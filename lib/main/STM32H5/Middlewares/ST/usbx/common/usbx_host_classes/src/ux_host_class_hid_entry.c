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


#if defined(UX_HOST_STANDALONE)
#define UX_HOST_CLASS_HID_ENUM_HID_DESC_READ            (UX_STATE_WAIT)
#define UX_HOST_CLASS_HID_ENUM_HID_DESC_PARSE           (UX_STATE_CLASS_STEP + 1)
#define UX_HOST_CLASS_HID_ENUM_REPORT_DESC_READ         (UX_STATE_CLASS_STEP + 2)
#define UX_HOST_CLASS_HID_ENUM_REPORT_DESC_PARSE        (UX_STATE_CLASS_STEP + 3)
#define UX_HOST_CLASS_HID_ENUM_CLIENT_SEARCH            (UX_STATE_CLASS_STEP + 4)
#define UX_HOST_CLASS_HID_ENUM_CLIENT_ACTIVATE_WAIT     (UX_STATE_CLASS_STEP + 6)
#define UX_HOST_CLASS_HID_ENUM_TRANSFER_WAIT            (UX_STATE_CLASS_STEP + 7)
#define UX_HOST_CLASS_HID_ENUM_ERROR                    (UX_STATE_CLASS_STEP + 8)
#define UX_HOST_CLASS_HID_ENUM_DONE                     (UX_STATE_CLASS_STEP + 9)


static inline UINT _ux_host_class_hid_activate_wait(UX_HOST_CLASS_COMMAND *command);
#endif


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_hid_entry                            PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the entry point of the HID class. It will be       */
/*    called by the USB stack enumeration module when there is a new      */
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
/*    _ux_host_class_hid_activate           Activate HID class            */
/*    _ux_host_class_hid_deactivate         Deactivate HID class          */
/*    _ux_utility_memory_free               Free memory                   */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*    HID Class                                                           */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added destroy command,      */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed uninited variable,    */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_entry(UX_HOST_CLASS_COMMAND *command)
{

UINT                                status;
INT                                 scan_index;
UX_HOST_CLASS_HID_CLIENT            *client;
UX_HOST_CLASS_HID_CLIENT_COMMAND    client_command;


    /* The command request will tell us we need to do here, either a enumeration
       query, an activation or a deactivation.  */
    switch (command -> ux_host_class_command_request)
    {

    case UX_HOST_CLASS_COMMAND_QUERY:

        /* The query command is used to let the stack enumeration process know if we want to own
           this device or not.  */
        if ((command -> ux_host_class_command_usage == UX_HOST_CLASS_COMMAND_USAGE_CSP) &&
            (command -> ux_host_class_command_class == UX_HOST_CLASS_HID_CLASS))
            return(UX_SUCCESS);
        else
            return(UX_NO_CLASS_MATCH);


    case UX_HOST_CLASS_COMMAND_ACTIVATE:

        /* The activate command is used when the device inserted has found a parent and
           is ready to complete the enumeration.   */

        status =  _ux_host_class_hid_activate(command);
        return(status);


#if defined(UX_HOST_STANDALONE)
    case UX_HOST_CLASS_COMMAND_ACTIVATE_WAIT:
        status = _ux_host_class_hid_activate_wait(command);
        return(status);
#endif


    case UX_HOST_CLASS_COMMAND_DEACTIVATE:

        /* The deactivate command is used when the device has been extracted either
           directly or when its parents has been extracted.  */

        status =  _ux_host_class_hid_deactivate(command);
        return(status);

    case UX_HOST_CLASS_COMMAND_DESTROY:

        /* The destroy command is used when the class is unregistered.  */

        /* Free allocated resources for clients.  */
        if (command -> ux_host_class_command_class_ptr -> ux_host_class_client != UX_NULL)
        {

            /* Get client.  */
            client = command -> ux_host_class_command_class_ptr -> ux_host_class_client;

            /* Inform clients for destroy.  */
            for (scan_index = 0; scan_index < UX_HOST_CLASS_HID_MAX_CLIENTS; scan_index ++)
            {

                /* Inform client for destroy.  */
                client_command.ux_host_class_hid_client_command_request = UX_HOST_CLASS_COMMAND_DESTROY;
                client_command.ux_host_class_hid_client_command_container = (VOID *)command -> ux_host_class_command_class_ptr;
                client -> ux_host_class_hid_client_handler(&client_command);
            }

            /* Free clients memory.  */
            _ux_utility_memory_free(command -> ux_host_class_command_class_ptr -> ux_host_class_client);
            command -> ux_host_class_command_class_ptr -> ux_host_class_client = UX_NULL;
        }
        return(UX_SUCCESS);

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
static inline VOID _ux_host_class_hid_descriptor_read(UX_HOST_CLASS_HID *hid)
{
UX_INTERFACE                    *interface_ptr;
UX_CONFIGURATION                *configuration;
UX_ENDPOINT                     *control_endpoint;
UX_TRANSFER                     *transfer_request;

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &hid -> ux_host_class_hid_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Need to allocate memory for the descriptor.  */
    interface_ptr = hid -> ux_host_class_hid_interface;
    configuration = interface_ptr -> ux_interface_configuration;
    hid -> ux_host_class_hid_allocated = _ux_utility_memory_allocate(
                    UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY,
                    configuration -> ux_configuration_descriptor.wTotalLength);
    if (hid -> ux_host_class_hid_allocated == UX_NULL)
    {

        /* Next: error.  */
        hid -> ux_host_class_hid_status = UX_MEMORY_INSUFFICIENT;
        hid -> ux_host_class_hid_enum_state = UX_HOST_CLASS_HID_ENUM_ERROR;
        return;
    }

    /* Create a transfer request for the GET_DESCRIPTOR request.  */
    transfer_request -> ux_transfer_request_data_pointer =      hid -> ux_host_class_hid_allocated;
    transfer_request -> ux_transfer_request_requested_length =  configuration -> ux_configuration_descriptor.wTotalLength;
    transfer_request -> ux_transfer_request_function =          UX_GET_DESCRIPTOR;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value =             UX_CONFIGURATION_DESCRIPTOR_ITEM << 8;
    transfer_request -> ux_transfer_request_index =             0;
    UX_TRANSFER_STATE_RESET(transfer_request);

    /* Next: transfer and parse.  */
    hid -> ux_host_class_hid_enum_state = UX_HOST_CLASS_HID_ENUM_TRANSFER_WAIT;
    hid -> ux_host_class_hid_next_state = UX_HOST_CLASS_HID_ENUM_HID_DESC_PARSE;
}
static inline VOID _ux_host_class_hid_report_descriptor_read(UX_HOST_CLASS_HID *hid)
{
UX_ENDPOINT                     *control_endpoint;
UX_TRANSFER                     *transfer_request;

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &hid -> ux_host_class_hid_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Need to allocate memory for the descriptor.  */
    hid -> ux_host_class_hid_allocated = _ux_utility_memory_allocate(
                    UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY,
                    hid -> ux_host_class_hid_descriptor.wItemLength);
    if (hid -> ux_host_class_hid_allocated == UX_NULL)
    {

        /* Next: error.  */
        hid -> ux_host_class_hid_status = UX_MEMORY_INSUFFICIENT;
        hid -> ux_host_class_hid_enum_state = UX_HOST_CLASS_HID_ENUM_ERROR;
        return;
    }

    /* Create a transfer request for the GET_DESCRIPTOR request.  */
    transfer_request -> ux_transfer_request_data_pointer =      hid -> ux_host_class_hid_allocated;
    transfer_request -> ux_transfer_request_requested_length =  hid -> ux_host_class_hid_descriptor.wItemLength;
    transfer_request -> ux_transfer_request_function =          UX_GET_DESCRIPTOR;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_INTERFACE;
    transfer_request -> ux_transfer_request_value =             UX_HOST_CLASS_HID_REPORT_DESCRIPTOR << 8;
    transfer_request -> ux_transfer_request_index =             hid -> ux_host_class_hid_interface -> ux_interface_descriptor.bInterfaceNumber;
    UX_TRANSFER_STATE_RESET(transfer_request);

    /* Next: transfer and parse.  */
    hid -> ux_host_class_hid_enum_state = UX_HOST_CLASS_HID_ENUM_TRANSFER_WAIT;
    hid -> ux_host_class_hid_next_state = UX_HOST_CLASS_HID_ENUM_REPORT_DESC_PARSE;
}
static inline VOID _ux_host_class_hid_hid_descriptor_parse(UX_HOST_CLASS_HID *hid)
{
UX_DEVICE                       *device;
UX_TRANSFER                     *transfer;
UCHAR                           *descriptor;
UINT                            descriptor_length;
UINT                            descriptor_type;
UINT                            descriptor_2;
ULONG                           total_length;
ULONG                           interface_number;
UINT                            interface_found = UX_FALSE;

    /* Get transfer.  */
    device = hid -> ux_host_class_hid_device;
    transfer = &device -> ux_device_control_endpoint.ux_endpoint_transfer_request;

    /* Get current interface number.  */
    interface_number = hid -> ux_host_class_hid_interface ->
                                    ux_interface_descriptor.bInterfaceNumber;

    /* Get received descriptor.  */
    descriptor = transfer -> ux_transfer_request_data_pointer;
    total_length = transfer -> ux_transfer_request_actual_length;

    /* The HID descriptor is embedded within the configuration descriptor.
        We parse the entire descriptor to locate the HID portion.  */
    while(total_length)
    {

        /* Get length and type of the descriptor.  */
        descriptor_length = *descriptor;
        descriptor_type   = *(descriptor + 1);
        descriptor_2      = *(descriptor + 2);

        /* Make sure this descriptor has at least the minimum length.  */
        if (descriptor_length < 3)
            break;

        switch(descriptor_type)
        {
        case UX_INTERFACE_DESCRIPTOR_ITEM:

            /* Check if interface is what we expected.  */
            interface_found = (interface_number == descriptor_2) ? UX_TRUE : UX_FALSE;
            break;

        case UX_HOST_CLASS_HID_DESCRIPTOR:

            /* Check if we are in expected interface.  */
            if (!interface_found)
                break;

            /* Save HID descriptor for later usage.
                Note only first 7 entries are saved, since we only support first
                9 bytes, no optional descriptors are supported for now. */
            _ux_utility_descriptor_parse(descriptor,
                _ux_system_hid_descriptor_structure, UX_HID_DESCRIPTOR_ENTRIES,
                (UCHAR *) &hid -> ux_host_class_hid_descriptor);

            /* Free allocated bytes.  */
            _ux_utility_memory_free(hid -> ux_host_class_hid_allocated);
            hid -> ux_host_class_hid_allocated = UX_NULL;

            /* Next: HID report descriptor read.  */
            hid -> ux_host_class_hid_enum_state = UX_HOST_CLASS_HID_ENUM_REPORT_DESC_READ;
            return;

        default:
            break;
        }

        /* Verify if the descriptor is still valid.  */
        if (descriptor_length > total_length)
            break;

        /* Next descriptor.  */
        descriptor += descriptor_length;
        total_length -= descriptor_length;
    }

    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* Return an error.  */
    hid -> ux_host_class_hid_status = UX_DESCRIPTOR_CORRUPTED;
    hid -> ux_host_class_hid_enum_state = UX_HOST_CLASS_HID_ENUM_ERROR;
}
static inline VOID _ux_host_class_hid_report_descriptor_parse(UX_HOST_CLASS_HID *hid)
{
UX_DEVICE               *device;
UX_TRANSFER             *transfer;
UCHAR                   *descriptor;
ULONG                   length;
UX_HOST_CLASS_HID_ITEM  item;
UINT                    status = UX_SUCCESS;

    /* Get transfer.  */
    device = hid -> ux_host_class_hid_device;
    transfer = &device -> ux_device_control_endpoint.ux_endpoint_transfer_request;

    /* Get buffer and length.  */
    descriptor = hid -> ux_host_class_hid_allocated;
    length = transfer -> ux_transfer_request_actual_length;

    /* Parse the report descriptor and build the report items.  */
    while (length)
    {

        /* Get one item from the report and analyze it.  */
        _ux_host_class_hid_report_item_analyse(descriptor, &item);

        /* Point the descriptor right after the item identifier.  */
        descriptor +=  item.ux_host_class_hid_item_report_format;

        /* Process relative to the item type.  */
        switch (item.ux_host_class_hid_item_report_type)
        {

        case UX_HOST_CLASS_HID_TYPE_GLOBAL:

            /* This is a global item.  */
            status =  _ux_host_class_hid_global_item_parse(hid, &item, descriptor);
            break;

        
        case UX_HOST_CLASS_HID_TYPE_MAIN:

            /* This is a main item.  */
            status =  _ux_host_class_hid_main_item_parse(hid, &item, descriptor);
            break;


        case UX_HOST_CLASS_HID_TYPE_LOCAL:

            /* This is a local item.  */
            status =  _ux_host_class_hid_local_item_parse(hid, &item, descriptor);
            break;          

        default:

            /* This is a reserved item, meaning it shouldn't be used!  */

            /* Set status to error. The check after this switch statement 
                will handle the rest.  */
            status =  UX_DESCRIPTOR_CORRUPTED;
            break;
        }

        /* Recheck the status code.  */
        if (status != UX_SUCCESS)
        {
            break;
        }

        /* Jump to the next item.  */
        descriptor +=  item.ux_host_class_hid_item_report_length;
    
        /* Verify that the report descriptor is not corrupted.  */
        if (length < item.ux_host_class_hid_item_report_length)
        {

            /* Return error status.  */
            status = (UX_DESCRIPTOR_CORRUPTED);
            break;
        }

        /* Adjust the length.  */
        length -=  (ULONG)(item.ux_host_class_hid_item_report_length + item.ux_host_class_hid_item_report_format);
    }

    if (status != UX_SUCCESS)
    {
        hid -> ux_host_class_hid_status = status;
        hid -> ux_host_class_hid_enum_state = UX_HOST_CLASS_HID_ENUM_ERROR;
        return;
    }

    /* Search the HID interrupt endpoint.  */
    status = _ux_host_class_hid_interrupt_endpoint_search(hid);
    if (status != UX_SUCCESS)
    {
        hid -> ux_host_class_hid_status = status;
        hid -> ux_host_class_hid_enum_state = UX_HOST_CLASS_HID_ENUM_ERROR;
        return;
    }

    /* Allocated memory is no use now, free.  */
    _ux_utility_memory_free(hid -> ux_host_class_hid_allocated);
    hid -> ux_host_class_hid_allocated = UX_NULL;

    /* Next: search & activate client.  */
    hid -> ux_host_class_hid_enum_state = UX_HOST_CLASS_HID_ENUM_CLIENT_SEARCH;
}
static inline VOID _ux_host_class_hid_client_activate_wait(UX_HOST_CLASS_HID *hid)
{
UX_HOST_CLASS_HID_CLIENT_COMMAND    hid_client_command;
UINT                                status;

    hid_client_command.ux_host_class_hid_client_command_instance =   hid;
    hid_client_command.ux_host_class_hid_client_command_request =    UX_HOST_CLASS_COMMAND_ACTIVATE_WAIT;

    /* Call the HID client with an activate command.  */
    status = hid -> ux_host_class_hid_client -> ux_host_class_hid_client_handler(&hid_client_command);

    /* Error.  */
    if (status < UX_STATE_NEXT)
    {
        hid -> ux_host_class_hid_client = UX_NULL;
        hid -> ux_host_class_hid_status = UX_DEVICE_ENUMERATION_FAILURE;
        hid -> ux_host_class_hid_enum_state = UX_HOST_CLASS_HID_ENUM_ERROR;
        return;
    }

    /* Success.  */
    if (status == UX_STATE_NEXT)
    {
        hid -> ux_host_class_hid_enum_state = UX_HOST_CLASS_HID_ENUM_DONE;
        return;
    }

    /* Wait.  */
    return;
}
static inline UINT _ux_host_class_hid_activate_wait(UX_HOST_CLASS_COMMAND *command)
{

UX_INTERFACE            *interface_ptr;
UX_ENDPOINT             *control_endpoint;
UX_TRANSFER             *transfer;
UX_HOST_CLASS_HID       *hid;
UINT                    status;

    /* Get the instance for this class.  */
    interface_ptr = (UX_INTERFACE *)command -> ux_host_class_command_container;
    hid =  (UX_HOST_CLASS_HID *) interface_ptr -> ux_interface_class_instance;

    /* Run initialize state machine.  */
    switch(hid -> ux_host_class_hid_enum_state)
    {
    case UX_HOST_CLASS_HID_ENUM_HID_DESC_READ            :
        _ux_host_class_hid_descriptor_read(hid);
        break;

    case UX_HOST_CLASS_HID_ENUM_HID_DESC_PARSE           :
        _ux_host_class_hid_hid_descriptor_parse(hid);
        break;

    case UX_HOST_CLASS_HID_ENUM_REPORT_DESC_READ         :
        _ux_host_class_hid_report_descriptor_read(hid);
        break;

    case UX_HOST_CLASS_HID_ENUM_REPORT_DESC_PARSE        :
        _ux_host_class_hid_report_descriptor_parse(hid);
        break;

    case UX_HOST_CLASS_HID_ENUM_CLIENT_SEARCH            :

        /* Search and activate client.  */
        status = _ux_host_class_hid_client_search(hid);
        if (status != UX_SUCCESS)
        {

            /* There is no client, but HID can still be used.  */
            hid -> ux_host_class_hid_enum_state = UX_HOST_CLASS_HID_ENUM_DONE;
            break;
        }

        /* Activate wait in case there is still steps for client.  */
        hid -> ux_host_class_hid_enum_state = UX_HOST_CLASS_HID_ENUM_CLIENT_ACTIVATE_WAIT;

        /* Fall through.  */
    case UX_HOST_CLASS_HID_ENUM_CLIENT_ACTIVATE_WAIT     :
        _ux_host_class_hid_client_activate_wait(hid);
        break;

    case UX_HOST_CLASS_HID_ENUM_TRANSFER_WAIT            :

        /* Get transfer.  */
        control_endpoint = &hid -> ux_host_class_hid_device -> ux_device_control_endpoint;
        transfer = &control_endpoint -> ux_endpoint_transfer_request;

        /* Transfer state machine.  */
        status = _ux_host_stack_transfer_run(transfer);

        /* Is it done?  */
        if (status <= UX_STATE_NEXT)
        {

            /* Is there error?  */
            if (transfer -> ux_transfer_request_completion_code != UX_SUCCESS)
            {
                hid -> ux_host_class_hid_status = transfer -> ux_transfer_request_completion_code;
                hid -> ux_host_class_hid_enum_state = UX_STATE_EXIT;
                break;
            }

            /* No error, next state.  */
            hid -> ux_host_class_hid_enum_state = hid -> ux_host_class_hid_next_state;
            break;
        }

        /* Keep waiting.  */
        break;

    case UX_HOST_CLASS_HID_ENUM_ERROR                    :

        /* Clean interrupt endpoint.  */
        if (hid -> ux_host_class_hid_interrupt_endpoint &&
            hid -> ux_host_class_hid_interrupt_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer)
            _ux_utility_memory_free(hid -> ux_host_class_hid_interrupt_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer);

        /* Clean instance. */
        _ux_host_class_hid_instance_clean(hid);

        /* Error, destroy the class instance and return error code. */
        _ux_host_stack_class_instance_destroy(hid -> ux_host_class_hid_class, (VOID *) hid);

        /* Unmount instance. */
        interface_ptr -> ux_interface_class_instance = UX_NULL;

        /* Free memory.  */
        if (hid -> ux_host_class_hid_allocated)
            _ux_utility_memory_free(hid -> ux_host_class_hid_allocated);

        /* Free instance. */
        _ux_utility_memory_free(hid);
        return(UX_STATE_NEXT);

    case UX_HOST_CLASS_HID_ENUM_DONE                     :

        /* Free temperary memory.  */
        if (hid -> ux_host_class_hid_allocated)
        {
            _ux_utility_memory_free(hid -> ux_host_class_hid_allocated);
            hid -> ux_host_class_hid_allocated = UX_NULL;
        }

        /* Mark the HID class as live now.  */
        hid -> ux_host_class_hid_state =  UX_HOST_CLASS_INSTANCE_LIVE;

        /* We may need to inform the application if a function has been programmed in the system structure.  */
        if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
        {

            /* Call system change function.  */
            _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, hid -> ux_host_class_hid_class, (VOID *) hid);
        }

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HID_ACTIVATE, hid, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* If trace is enabled, register this object.  */
        UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, hid, 0, 0, 0)

        hid -> ux_host_class_hid_enum_state = UX_STATE_IDLE;
        return(UX_STATE_NEXT);

    default: /* IDLE, Other states.  */
        return(UX_STATE_NEXT);
    }

    /* By default, keep waiting.  */
    return(UX_STATE_WAIT);
}
#endif
