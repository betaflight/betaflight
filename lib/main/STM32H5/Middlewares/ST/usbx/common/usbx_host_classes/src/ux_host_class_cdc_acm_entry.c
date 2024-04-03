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
/**   Acm Cdc Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_cdc_acm.h"
#include "ux_host_stack.h"


#if defined(UX_HOST_STANDALONE)
#define UX_HOST_CLASS_CDC_ACM_INIT_TRANSFER_WAIT        (UX_STATE_WAIT)
#define UX_HOST_CLASS_CDC_ACM_INIT_DESCRIPTORS_PARSE    (UX_STATE_NEXT)

#define UX_HOST_CLASS_CDC_ACM_INIT_DELAY_WAIT           (UX_STATE_CLASS_STEP + 0)
#define UX_HOST_CLASS_CDC_ACM_INIT_LINE_CODING_SET      (UX_STATE_CLASS_STEP + 1)
#define UX_HOST_CLASS_CDC_ACM_INIT_LINE_CODING_GET      (UX_STATE_CLASS_STEP + 2)
#define UX_HOST_CLASS_CDC_ACM_INIT_LINE_CODING_CHECK    (UX_STATE_CLASS_STEP + 3)
#define UX_HOST_CLASS_CDC_ACM_INIT_LINE_STATE_SET       (UX_STATE_CLASS_STEP + 4)
#define UX_HOST_CLASS_CDC_ACM_INIT_ERROR                (UX_STATE_CLASS_STEP + 5)
#define UX_HOST_CLASS_CDC_ACM_INIT_DONE                 (UX_STATE_CLASS_STEP + 6)


static inline UINT  _ux_host_class_cdc_acm_activate_wait(UX_HOST_CLASS_COMMAND *command);
#endif


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_cdc_acm_entry                        PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the entry point of the cdc_acm class. It will be   */ 
/*    called by the USBX stack enumeration module when there is a new     */ 
/*    cdc_acm on the bus or when the USB cdc_acm is removed.              */ 
/*                                                                        */ 
/*    A CDC class can have multiple interfaces, one for Control and one   */ 
/*    for Data. Here we filter for the Communication Class with ACM       */ 
/*    subclass and the Communication Data Class.                          */ 
/*                                                                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                                  Acm Cdc class command      */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_cdc_acm_activate       Activate cdc_acm class        */ 
/*    _ux_host_class_cdc_acm_deactivate     Deactivate cdc_acm class      */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Acm Cdc Class                                                       */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_cdc_acm_entry(UX_HOST_CLASS_COMMAND *command)
{

UINT    status;


    /* The command request will tell us we need to do here, either a enumeration
       query, an activation or a deactivation.  */
    switch (command -> ux_host_class_command_request)
    {

    case UX_HOST_CLASS_COMMAND_QUERY:

        /* The query command is used to let the stack enumeration process know if we want to own
           this device or not.  */
        if((command -> ux_host_class_command_usage == UX_HOST_CLASS_COMMAND_USAGE_CSP) &&
                             ((command -> ux_host_class_command_class == UX_HOST_CLASS_CDC_DATA_CLASS) ||
                             ((command -> ux_host_class_command_class == UX_HOST_CLASS_CDC_CONTROL_CLASS) &&
                             (command -> ux_host_class_command_subclass == UX_HOST_CLASS_CDC_ACM_SUBCLASS)) ||
                             ((command -> ux_host_class_command_class == UX_HOST_CLASS_CDC_CONTROL_CLASS) &&
                             (command -> ux_host_class_command_subclass == UX_HOST_CLASS_CDC_DLC_SUBCLASS))))
        {
            /* Check for IAD presence.  */
            if ((command -> ux_host_class_command_iad_class == 0) && (command -> ux_host_class_command_iad_subclass == 0))
            
                /* No IAD, we accept this class.  */
                return(UX_SUCCESS);            
            
            else
            {

                if ((command -> ux_host_class_command_iad_class == UX_HOST_CLASS_CDC_CONTROL_CLASS) &&
                        (command -> ux_host_class_command_iad_subclass == UX_HOST_CLASS_CDC_ACM_SUBCLASS))
            
                    /* There is an IAD and this is for CDC-ACM.  */
                    return(UX_SUCCESS);                        

                else
                
                    /* The IAD does not match with CDC-ACM.  */
                    return(UX_NO_CLASS_MATCH);                        
            }
        }

        else            

            /* No match.  */
            return(UX_NO_CLASS_MATCH);                        
                
    case UX_HOST_CLASS_COMMAND_ACTIVATE:

        /* The activate command is used when the device inserted has found a parent and
           is ready to complete the enumeration.  */
        status =  _ux_host_class_cdc_acm_activate(command);
        return(status);

#if defined(UX_HOST_STANDALONE)
    case UX_HOST_CLASS_COMMAND_ACTIVATE_WAIT:
        status = _ux_host_class_cdc_acm_activate_wait(command);
        return(status);
#endif

    case UX_HOST_CLASS_COMMAND_DEACTIVATE:

        /* The deactivate command is used when the device has been extracted either      
           directly or when its parents has been extracted.  */
        status =  _ux_host_class_cdc_acm_deactivate(command);
        return(status);

    default: 
            
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_FUNCTION_NOT_SUPPORTED);
    }   
}

#if defined(UX_HOST_STANDALONE)
static inline VOID  _ux_host_class_cdc_acm_descriptors_parse(UX_HOST_CLASS_CDC_ACM *cdc_acm)
{

UX_ENDPOINT                 *control_endpoint;
UX_TRANSFER                 *transfer_request;
UX_INTERFACE                *interface_ptr;
UCHAR                       *descriptor;
ULONG                       total_descriptor_length;
UCHAR                       descriptor_length;
UCHAR                       descriptor_type;
UCHAR                       descriptor_byte2;
ULONG                       interface_found;
ULONG                       interface_parsed;
UCHAR                       offset;

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &cdc_acm -> ux_host_class_cdc_acm_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Check if transfer is done success.  */
    if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS ||
        transfer_request -> ux_transfer_request_actual_length !=
            transfer_request -> ux_transfer_request_requested_length)
    {
        cdc_acm -> ux_host_class_cdc_acm_status = UX_DESCRIPTOR_CORRUPTED;
        cdc_acm -> ux_host_class_cdc_acm_cmd_state = UX_HOST_CLASS_CDC_ACM_INIT_ERROR;
        return;
    }

    /* Get the interface.  */
    interface_ptr = cdc_acm -> ux_host_class_cdc_acm_interface;

    /* Parse the descriptor.  */
    total_descriptor_length = transfer_request -> ux_transfer_request_actual_length;
    descriptor = (UCHAR *)cdc_acm -> ux_host_class_cdc_acm_allocated;
    interface_found = UX_FALSE;
    interface_parsed = UX_FALSE;

    while(total_descriptor_length)
    {

        /* Get descriptor length, type, subtype.  */
        descriptor_length  = *descriptor;
        descriptor_type    = *(descriptor + 1);
        descriptor_byte2   = *(descriptor + 2);

        /* Make sure this descriptor has at least the minimum length.  */
        if (descriptor_length < 3 || descriptor_length > total_descriptor_length)
        {
            /* Descriptor is corrupted.  */
            break;
        }

        /* Process related descriptors.  */
        switch(descriptor_type)
        {
        case UX_INTERFACE_DESCRIPTOR_ITEM:

            /* Check if interface is what we expected.  */
            if (interface_ptr -> ux_interface_descriptor.bInterfaceNumber == descriptor_byte2)
            {

                /* Mark found.  */
                interface_found = UX_TRUE;
            }
            else
            {

                /* Run out of expected interface.  */
                if (interface_found)
                {

                    interface_parsed = UX_TRUE;
                }
            }
            break;

        case UX_HOST_CLASS_CDC_ACM_CS_INTERFACE:

            /* Check if we are in correct interface.  */
            if (interface_found)
            {

                /* Check bDescriptorSubType.  */
                switch(descriptor_byte2)
                {
                case UX_HOST_CLASS_CDC_ACM_CALL_MANAGEMENT_DESCRIPTOR:

                    /* Store capabilities.  */
                    cdc_acm -> ux_host_class_cdc_acm_capabilities =
                        *(descriptor + UX_HOST_CLASS_CDC_ACM_CALL_MANAGEMENT_CAPABILITIES);
                    break;

                case UX_HOST_CLASS_CDC_ACM_UNION_DESCRIPTOR:

                    /* Check related interfaces.  */
                    for (offset = UX_HOST_CLASS_CDC_ACM_UNION_FUNCTIONAL_MASTER;
                        offset < descriptor_length;
                        offset ++)
                    {

                        /* Save the interface in interface bitmap.  */
                        cdc_acm -> ux_host_class_cdc_acm_interfaces_bitmap |=
                                                1u << *(descriptor + offset);
                    }
                    break;

                default:
                    break;
                }
            }
            break;

        default:
            break;
        }

        /* Next descriptor.  */
        descriptor += descriptor_length;
        total_descriptor_length -= descriptor_length;
    }

    /* We can free the resource now.  */
    _ux_utility_memory_free(cdc_acm -> ux_host_class_cdc_acm_allocated);
    cdc_acm -> ux_host_class_cdc_acm_allocated = UX_NULL;

    /* Descriptor fail.  */
    if (interface_parsed == UX_FALSE)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Descriptor is corrupted.  */
        cdc_acm -> ux_host_class_cdc_acm_status = UX_DESCRIPTOR_CORRUPTED;
        cdc_acm -> ux_host_class_cdc_acm_cmd_state = UX_HOST_CLASS_CDC_ACM_INIT_ERROR;
        return;
    }

    /* Descriptor is parsed.  */
    cdc_acm -> ux_host_class_cdc_acm_tick = _ux_utility_time_get();
    cdc_acm -> ux_host_class_cdc_acm_cmd_state = UX_HOST_CLASS_CDC_ACM_INIT_DELAY_WAIT;
}

static inline UINT  _ux_host_class_cdc_acm_activate_wait(UX_HOST_CLASS_COMMAND *command)
{

UX_HOST_CLASS               *cdc_acm_class;
UX_HOST_CLASS_CDC_ACM       *cdc_acm_inst;
UX_INTERFACE                *interface_ptr;
UX_HOST_CLASS_CDC_ACM       *cdc_acm;
UX_ENDPOINT                 *control_endpoint;
UX_TRANSFER                 *transfer;
UCHAR                       *buffer;
ULONG                       rate, stop_bit, parity, data_bit;
UINT                        status;
ULONG                       tick, diff;

    /* Get the instance for this class.  */
    interface_ptr = (UX_INTERFACE *)command -> ux_host_class_command_container;
    cdc_acm =  (UX_HOST_CLASS_CDC_ACM *) interface_ptr -> ux_interface_class_instance;

    /* Run initialize state machine.  */
    switch(cdc_acm -> ux_host_class_cdc_acm_cmd_state)
    {
    case UX_HOST_CLASS_CDC_ACM_INIT_DESCRIPTORS_PARSE:
        _ux_host_class_cdc_acm_descriptors_parse(cdc_acm);
        break;

    case UX_HOST_CLASS_CDC_ACM_INIT_DELAY_WAIT:
        tick = _ux_utility_time_get();
        diff = _ux_utility_time_elapsed(cdc_acm -> ux_host_class_cdc_acm_tick, tick);
        if (diff > UX_MS_TO_TICK(UX_HOST_CLASS_CDC_ACM_DEVICE_INIT_DELAY))
        {
            /* Allocate some buffer for commands.  */
            cdc_acm -> ux_host_class_cdc_acm_allocated =
                _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY,
                                                           8);

            if (cdc_acm -> ux_host_class_cdc_acm_allocated == UX_NULL)
            {
                cdc_acm -> ux_host_class_cdc_acm_status = UX_MEMORY_INSUFFICIENT;
                cdc_acm -> ux_host_class_cdc_acm_cmd_state =
                                            UX_HOST_CLASS_CDC_ACM_INIT_ERROR;
                break;
            }
            control_endpoint = &cdc_acm -> ux_host_class_cdc_acm_device ->
                                                    ux_device_control_endpoint;
            transfer = &control_endpoint -> ux_endpoint_transfer_request;

            /* Initialize transfer buffer.  */
            transfer -> ux_transfer_request_data_pointer = (UCHAR *)
                                    cdc_acm -> ux_host_class_cdc_acm_allocated;

            /* Initialize transfer flags.  */
            transfer -> ux_transfer_request_flags = 0;

            cdc_acm -> ux_host_class_cdc_acm_cmd_state =
                                    UX_HOST_CLASS_CDC_ACM_INIT_LINE_CODING_SET;
        }
        break;

    case UX_HOST_CLASS_CDC_ACM_INIT_LINE_CODING_SET:

        /* Put line coding things in buffer.  */
        buffer = (UCHAR *)cdc_acm -> ux_host_class_cdc_acm_allocated;
        rate     = UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_RATE;
        stop_bit = UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_STOP_BIT;
        parity   = UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_PARITY;
        data_bit = UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_DATA_BIT;
        _ux_utility_long_put(buffer + UX_HOST_CLASS_CDC_ACM_LINE_CODING_RATE, rate);
        *(buffer + UX_HOST_CLASS_CDC_ACM_LINE_CODING_STOP_BIT) = (UCHAR)stop_bit;
        *(buffer + UX_HOST_CLASS_CDC_ACM_LINE_CODING_PARITY)   = (UCHAR)parity;
        *(buffer + UX_HOST_CLASS_CDC_ACM_LINE_CODING_DATA_BIT) = (UCHAR)data_bit;

        status = _ux_host_class_cdc_acm_command(cdc_acm,
                            UX_HOST_CLASS_CDC_ACM_REQ_SET_LINE_CODING, 0,
                            buffer, UX_HOST_CLASS_CDC_ACM_LINE_CODING_LENGTH);
        if (status != UX_SUCCESS)
        {
            cdc_acm -> ux_host_class_cdc_acm_status = status;
            cdc_acm -> ux_host_class_cdc_acm_cmd_state =
                                        UX_HOST_CLASS_CDC_ACM_INIT_ERROR;
            break;
        }
        cdc_acm -> ux_host_class_cdc_acm_cmd_state =
                                    UX_HOST_CLASS_CDC_ACM_INIT_TRANSFER_WAIT;
        cdc_acm -> ux_host_class_cdc_acm_next_state =
                                    UX_HOST_CLASS_CDC_ACM_INIT_LINE_CODING_GET;
        break;

    case UX_HOST_CLASS_CDC_ACM_INIT_LINE_CODING_GET:

        /* Get line coding things from device.  */
        buffer = (UCHAR *)cdc_acm -> ux_host_class_cdc_acm_allocated;
        status = _ux_host_class_cdc_acm_command(cdc_acm,
                            UX_HOST_CLASS_CDC_ACM_REQ_GET_LINE_CODING, 0,
                            buffer, UX_HOST_CLASS_CDC_ACM_LINE_CODING_LENGTH);
        if (status != UX_SUCCESS)
        {
            cdc_acm -> ux_host_class_cdc_acm_status = status;
            cdc_acm -> ux_host_class_cdc_acm_cmd_state =
                                        UX_HOST_CLASS_CDC_ACM_INIT_ERROR;
            break;
        }
        cdc_acm -> ux_host_class_cdc_acm_cmd_state =
                                UX_HOST_CLASS_CDC_ACM_INIT_TRANSFER_WAIT;
        cdc_acm -> ux_host_class_cdc_acm_next_state =
                                UX_HOST_CLASS_CDC_ACM_INIT_LINE_CODING_CHECK;
        break;

    case UX_HOST_CLASS_CDC_ACM_INIT_LINE_CODING_CHECK:

        /* Check line coding things in buffer.  */
        buffer = (UCHAR *)cdc_acm -> ux_host_class_cdc_acm_allocated;
        rate     = _ux_utility_long_get(buffer + UX_HOST_CLASS_CDC_ACM_LINE_CODING_RATE);
        stop_bit = *(buffer + UX_HOST_CLASS_CDC_ACM_LINE_CODING_STOP_BIT);
        parity   = *(buffer + UX_HOST_CLASS_CDC_ACM_LINE_CODING_PARITY);
        data_bit = *(buffer + UX_HOST_CLASS_CDC_ACM_LINE_CODING_DATA_BIT);
        if (rate != UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_RATE ||
            stop_bit != UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_STOP_BIT ||
            parity != UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_PARITY ||
            data_bit != UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_DATA_BIT)
        {
            cdc_acm -> ux_host_class_cdc_acm_status = UX_DEVICE_ENUMERATION_FAILURE;
            cdc_acm -> ux_host_class_cdc_acm_cmd_state = UX_HOST_CLASS_CDC_ACM_INIT_ERROR;
            break;
        }

        /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_INIT_LINE_STATE_SET:

        /* Put line state things.  */
        buffer = (UCHAR *)cdc_acm -> ux_host_class_cdc_acm_allocated;
        *(buffer) = UX_HOST_CLASS_CDC_ACM_CTRL_DTR | UX_HOST_CLASS_CDC_ACM_CTRL_RTS;
        status = _ux_host_class_cdc_acm_command(cdc_acm,
                                    UX_HOST_CLASS_CDC_ACM_REQ_SET_LINE_STATE,
                                    *(buffer), UX_NULL, 0);
        if (status != UX_SUCCESS)
        {
            cdc_acm -> ux_host_class_cdc_acm_status = status;
            cdc_acm -> ux_host_class_cdc_acm_cmd_state =
                                        UX_HOST_CLASS_CDC_ACM_INIT_ERROR;
            break;
        }
        cdc_acm -> ux_host_class_cdc_acm_cmd_state =
                                    UX_HOST_CLASS_CDC_ACM_INIT_TRANSFER_WAIT;
        cdc_acm -> ux_host_class_cdc_acm_next_state =
                                    UX_HOST_CLASS_CDC_ACM_INIT_DONE;
        break;

    case UX_HOST_CLASS_CDC_ACM_INIT_ERROR:
        /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_INIT_DONE:

        /* Free allocated buffer.  */
        if (cdc_acm -> ux_host_class_cdc_acm_allocated)
        {
            _ux_utility_memory_free(cdc_acm -> ux_host_class_cdc_acm_allocated);
            cdc_acm -> ux_host_class_cdc_acm_allocated = UX_NULL;
        }

        /* Check status.  */
        if (cdc_acm -> ux_host_class_cdc_acm_status == UX_SUCCESS)
        {

            /* We scan CDC ACM instances to find the DATA instance.  */
            /* Get class.  */
            cdc_acm_class = cdc_acm -> ux_host_class_cdc_acm_class;

            /* Get first instance linked to the class.  */
            cdc_acm_inst = (UX_HOST_CLASS_CDC_ACM *)cdc_acm_class -> ux_host_class_first_instance;

            /* Scan all instances.  */
            while(cdc_acm_inst)
            {

                /* Get interface of the instance.  */
                interface_ptr = cdc_acm_inst -> ux_host_class_cdc_acm_interface;

                /* If this data interface is inside the associate list, link it.  */
                if (cdc_acm -> ux_host_class_cdc_acm_interfaces_bitmap &
                    (1ul << interface_ptr -> ux_interface_descriptor.bInterfaceNumber))
                {

                    /* Save control instance and we are done.  */
                    cdc_acm_inst -> ux_host_class_cdc_acm_control = cdc_acm;
                    break;
                }

                /* Next instance.  */
                cdc_acm_inst = cdc_acm_inst -> ux_host_class_cdc_acm_next_instance;
            }

            /* Mark the cdc_acm as live now.  Both interfaces need to be live. */
            cdc_acm -> ux_host_class_cdc_acm_state =  UX_HOST_CLASS_INSTANCE_LIVE;

            /* If all is fine and the device is mounted, we may need to inform the application
                if a function has been programmed in the system structure.  */
            if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
            {

                /* Call system change function.  */
                _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, cdc_acm -> ux_host_class_cdc_acm_class, (VOID *) cdc_acm);
            }

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_CDC_ACM_ACTIVATE, cdc_acm, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

            /* If trace is enabled, register this object.  */
            UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, cdc_acm, 0, 0, 0)

            /* Reset CMD state.  */
            cdc_acm -> ux_host_class_cdc_acm_cmd_state = UX_STATE_RESET;
        }
        else
        {

            /* On error case, it's possible data buffer allocated for interrupt endpoint and transfer started, stop and free it.  */
            if (cdc_acm -> ux_host_class_cdc_acm_interrupt_endpoint && 
                cdc_acm -> ux_host_class_cdc_acm_interrupt_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer)
            {

                /* The first transfer request has already been initiated. Abort it.  */
                _ux_host_stack_endpoint_transfer_abort(cdc_acm -> ux_host_class_cdc_acm_interrupt_endpoint);

                /* Free the memory for the data pointer.  */
                _ux_utility_memory_free(cdc_acm -> ux_host_class_cdc_acm_interrupt_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer);
            }

            /* Destroy the instance.  */
            _ux_host_stack_class_instance_destroy(cdc_acm -> ux_host_class_cdc_acm_class, (VOID *) cdc_acm);

            /* Unmount instance. */
            interface_ptr = cdc_acm -> ux_host_class_cdc_acm_interface;
            interface_ptr -> ux_interface_class_instance = UX_NULL;

            /* Free instance. */
            _ux_utility_memory_free(cdc_acm);
        }

        /* Done, OK to go on.  */
        return(UX_STATE_NEXT);

    case UX_HOST_CLASS_CDC_ACM_INIT_TRANSFER_WAIT:

        /* Get transfer.  */
        control_endpoint = &cdc_acm -> ux_host_class_cdc_acm_device -> ux_device_control_endpoint;
        transfer = &control_endpoint -> ux_endpoint_transfer_request;

        /* Transfer state machine.  */
        status = _ux_host_stack_transfer_run(transfer);

        /* Is it done?  */
        if (status <= UX_STATE_NEXT)
        {

            /* Is there error?  */
            if (transfer -> ux_transfer_request_completion_code != UX_SUCCESS)
            {
                cdc_acm -> ux_host_class_cdc_acm_cmd_state = UX_STATE_EXIT;
                break;
            }

            /* No error, next state.  */
            cdc_acm -> ux_host_class_cdc_acm_cmd_state =
                                    cdc_acm -> ux_host_class_cdc_acm_next_state;
            break;
        }

        /* Keep waiting.  */
        break;

    /* UX_STATE_RESET, UX_STATE_EXIT, UX_STATE_IDLE, ...  */
    default:

        /* Do nothing.  */
        return(UX_STATE_NEXT);
    }

    /* Keep waiting.  */
    return(UX_STATE_WAIT);
}
#endif
