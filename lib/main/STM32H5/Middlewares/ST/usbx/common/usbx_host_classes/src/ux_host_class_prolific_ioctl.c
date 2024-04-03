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
/**   Prolific Class                                                      */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_prolific.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_prolific_ioctl                       PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the ioctl entry point for the application to       */ 
/*    configure the Prolific device.                                      */ 
/*                                                                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    prolific                              Pointer to prolific class     */ 
/*    ioctl_function                        ioctl function                */ 
/*    parameter                             pointer to structure          */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Transfer request              */
/*    _ux_host_stack_endpoint_transfer_abort                              */
/*                                          Abort transfer                */
/*    _ux_host_class_prolific_command       Send command to device        */ 
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_free               Free memory                   */
/*    _ux_utility_long_put                  Put 32-bit value              */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Storage Class                                                       */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_prolific_ioctl(UX_HOST_CLASS_PROLIFIC *prolific, ULONG ioctl_function,
                                    VOID *parameter)
{

UINT                                status;
UCHAR                               *data_buffer;
UX_HOST_CLASS_PROLIFIC_LINE_CODING  *line_coding;
UX_HOST_CLASS_PROLIFIC_LINE_STATE   *line_state;
ULONG                               value;
UX_ENDPOINT                         *control_endpoint;
UX_TRANSFER                         *transfer_request;
VOID                                (*callback_function) (struct UX_HOST_CLASS_PROLIFIC_STRUCT *, ULONG );

    /* Ensure the instance is valid.  */
    if ((prolific -> ux_host_class_prolific_state !=  UX_HOST_CLASS_INSTANCE_LIVE) && 
        (prolific -> ux_host_class_prolific_state !=  UX_HOST_CLASS_INSTANCE_MOUNTING))
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ENDPOINT_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, prolific, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* The command request will tell us we need to do here.  */
    switch (ioctl_function)
    {

    case UX_HOST_CLASS_PROLIFIC_IOCTL_SET_LINE_CODING:

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PROLIFIC_IOCTL_SET_LINE_CODING, prolific, parameter, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)
    
        /* Allocate some cache safe memory for the control command.  */
        data_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_PROLIFIC_LINE_CODING_LENGTH);
        
        /* Check if error. Return with error if no memory could be allocated.  */
        if (data_buffer == UX_NULL)
            
            /* Do not proceed. Set error code.  */
            status = UX_MEMORY_INSUFFICIENT;
        else
        {
        
            /* Build the buffer from the calling parameter. Cast the calling parameter.  */
            line_coding = (UX_HOST_CLASS_PROLIFIC_LINE_CODING *) parameter;
            
            /* Put the data rate.  */
            _ux_utility_long_put(data_buffer + UX_HOST_CLASS_PROLIFIC_LINE_CODING_RATE, 
                                line_coding -> ux_host_class_prolific_line_coding_dter);
                                
            /* Then the stop bit.  */
            *(data_buffer + UX_HOST_CLASS_PROLIFIC_LINE_CODING_STOP_BIT) = 
            (UCHAR) line_coding -> ux_host_class_prolific_line_coding_stop_bit;
            
            /* Then the parity.  */
            *(data_buffer + UX_HOST_CLASS_PROLIFIC_LINE_CODING_PARITY) = 
            (UCHAR) line_coding -> ux_host_class_prolific_line_coding_parity;
            
            /* Finally the data bits.  */
            *(data_buffer + UX_HOST_CLASS_PROLIFIC_LINE_CODING_DATA_BIT) = 
            (UCHAR) line_coding -> ux_host_class_prolific_line_coding_data_bits;

            /* Send the command to the device.  */
            status = _ux_host_class_prolific_command(prolific, UX_HOST_CLASS_PROLIFIC_REQ_SET_LINE_CODING,
                                    0, data_buffer, UX_HOST_CLASS_PROLIFIC_LINE_CODING_LENGTH);
            
            /* We free the resources allocated no matter what.  */
            _ux_utility_memory_free(data_buffer);            
        }        
        break;
                    
    case UX_HOST_CLASS_PROLIFIC_IOCTL_GET_LINE_CODING:

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PROLIFIC_IOCTL_GET_LINE_CODING, prolific, parameter, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)
    
        /* Allocate some cache safe memory for the control command.  */
        data_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_PROLIFIC_LINE_CODING_LENGTH);
        
        /* Check if error. Return with error if no memory could be allocated.  */
        if (data_buffer == UX_NULL)
            
            /* Do not proceed. Set error code.  */
            status = UX_MEMORY_INSUFFICIENT;
        else
        {
        
            /* Send the command to the device.  */
            status = _ux_host_class_prolific_command(prolific, UX_HOST_CLASS_PROLIFIC_REQ_GET_LINE_CODING,
                                    0, data_buffer, UX_HOST_CLASS_PROLIFIC_LINE_CODING_LENGTH);
        
            /* Fill in the calling buffer if the result is successful. */
            if (status == UX_SUCCESS)
            {
            
                /* Build the buffer from the calling parameter. Cast the calling parameter.  */
                line_coding = (UX_HOST_CLASS_PROLIFIC_LINE_CODING *) parameter;
            
                /* Get the data rate.  */
                line_coding -> ux_host_class_prolific_line_coding_dter = _ux_utility_long_get(data_buffer + UX_HOST_CLASS_PROLIFIC_LINE_CODING_RATE);
                                
                /* Then the stop bit.  */
                line_coding -> ux_host_class_prolific_line_coding_stop_bit = 
                (ULONG) *(data_buffer + UX_HOST_CLASS_PROLIFIC_LINE_CODING_STOP_BIT);
            
                /* Then the parity.  */
                line_coding -> ux_host_class_prolific_line_coding_parity = 
                (ULONG) *(data_buffer + UX_HOST_CLASS_PROLIFIC_LINE_CODING_PARITY);
            
                /* Finally the data bits.  */
                line_coding -> ux_host_class_prolific_line_coding_data_bits =
                (ULONG) *(data_buffer + UX_HOST_CLASS_PROLIFIC_LINE_CODING_DATA_BIT); 
            }

            /* We free the resources allocated no matter what.  */
            _ux_utility_memory_free(data_buffer);            
        }        
        break;
            
    case UX_HOST_CLASS_PROLIFIC_IOCTL_SET_LINE_STATE:

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PROLIFIC_IOCTL_SET_LINE_STATE, prolific, parameter, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)
    
        /* Cast the calling parameter.  */
        line_state = (UX_HOST_CLASS_PROLIFIC_LINE_STATE *) parameter;
                   
        /* Build the value field.  */
        value = (line_state -> ux_host_class_prolific_line_state_rts | 
                (line_state -> ux_host_class_prolific_line_state_dtr << 1));
    
        /* Send the command to the device.  */
        status = _ux_host_class_prolific_command(prolific, UX_HOST_CLASS_PROLIFIC_REQ_SET_LINE_STATE,
                                    value, UX_NULL,0);
        break;    

    case UX_HOST_CLASS_PROLIFIC_IOCTL_SEND_BREAK :

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PROLIFIC_IOCTL_SEND_BREAK, prolific, parameter, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)
    
        /* Build the value field.  */
        value = *((ULONG *) parameter);
    
        /* Send the command to the device.  */
        status = _ux_host_class_prolific_command(prolific, UX_HOST_CLASS_PROLIFIC_REQ_SEND_BREAK,
                                    value, UX_NULL,0);
        break;    

    case UX_HOST_CLASS_PROLIFIC_IOCTL_PURGE :

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PROLIFIC_IOCTL_PURGE, prolific, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)
    
        /* We need to get the default control endpoint transfer request pointer.  */
        control_endpoint =  &prolific -> ux_host_class_prolific_device -> ux_device_control_endpoint;
        transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

        /* Reset upstream data pipes part 1.  */
        transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
        transfer_request -> ux_transfer_request_requested_length    =  0;
        transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_PROLIFIC_VENDOR_WRITE_REQUEST;
        transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
        transfer_request -> ux_transfer_request_value               =  UX_HOST_CLASS_PROLIFIC_COMMAND_PIPE1_RESET;
        transfer_request -> ux_transfer_request_index               =  0;
        
        /* Send request to HCD layer.  */
        _ux_host_stack_transfer_request(transfer_request);
    
        /* Reset upstream data pipes part 1.  */
        transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
        transfer_request -> ux_transfer_request_requested_length    =  0;
        transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_PROLIFIC_VENDOR_WRITE_REQUEST;
        transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
        transfer_request -> ux_transfer_request_value               =  UX_HOST_CLASS_PROLIFIC_COMMAND_PIPE2_RESET;
        transfer_request -> ux_transfer_request_index               =  0;
        
        /* Send request to HCD layer.  */
        status =  _ux_host_stack_transfer_request(transfer_request);

        break;    

    case UX_HOST_CLASS_PROLIFIC_IOCTL_ABORT_IN_PIPE :

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PROLIFIC_IOCTL_ABORT_IN_PIPE, prolific, prolific -> ux_host_class_prolific_bulk_in_endpoint, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)
    
        /* We need to abort transactions on the bulk In pipe.  */
        _ux_host_stack_endpoint_transfer_abort(prolific -> ux_host_class_prolific_bulk_in_endpoint);
        
        /* Status is successful.  */
        status = UX_SUCCESS;
        break;

    case UX_HOST_CLASS_PROLIFIC_IOCTL_ABORT_OUT_PIPE :

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PROLIFIC_IOCTL_ABORT_OUT_PIPE, prolific, prolific -> ux_host_class_prolific_bulk_out_endpoint, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)
    
        /* We need to abort transactions on the bulk Out pipe.  */
        _ux_host_stack_endpoint_transfer_abort(prolific -> ux_host_class_prolific_bulk_out_endpoint);

        /* Status is successful.  */
        status = UX_SUCCESS;
        break;

    case UX_HOST_CLASS_PROLIFIC_IOCTL_REPORT_DEVICE_STATUS_CHANGE :

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PROLIFIC_IOCTL_REPORT_DEVICE_STATUS_CHANGE, prolific, parameter, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)
    
        /* Register a callback when the line state has changed. */
        callback_function = ((VOID (*) (struct UX_HOST_CLASS_PROLIFIC_STRUCT *, ULONG )) (ALIGN_TYPE)parameter);
        prolific -> ux_host_class_prolific_device_status_change_callback = callback_function;

        /* Status is successful.  */
        status = UX_SUCCESS;
        break;

    case UX_HOST_CLASS_PROLIFIC_IOCTL_GET_DEVICE_STATUS :

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PROLIFIC_IOCTL_GET_DEVICE_STATUS, prolific, prolific -> ux_host_class_prolific_device_state, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)
    
        /* Return the device status.  */
        * ((ULONG *) parameter) = prolific -> ux_host_class_prolific_device_state;

        /* Status is successful.  */
        status = UX_SUCCESS;
        break;


    default: 

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Function not supported. Return an error.  */
        status =  UX_FUNCTION_NOT_SUPPORTED;
        break;
    }   

    /* Return status to caller.  */
    return(status);
}

