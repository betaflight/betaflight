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
/**   Generic Serial Host module class                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_gser.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_gser_ioctl                           PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the ioctl entry point for the application to       */ 
/*    configure the ACM device.                                           */ 
/*                                                                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    acm                                   Pointer to acm class          */ 
/*    ioctl_function                        ioctl function                */ 
/*    parameter                             pointer to structure          */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_gser_command           Send command to acm device    */ 
/*    _ux_host_stack_endpoint_transfer_abort                              */
/*                                          Abort transfer                */
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_gser_ioctl(UX_HOST_CLASS_GSER *gser, ULONG interface_index, ULONG ioctl_function,
                                    VOID *parameter)
{

UINT                                status;
UCHAR                               *data_buffer;
UX_HOST_CLASS_GSER_LINE_CODING      *line_coding;
UX_HOST_CLASS_GSER_LINE_STATE       *line_state;
VOID                                (*callback_function) (struct UX_HOST_CLASS_GSER_STRUCT *, ULONG, ULONG );
ULONG                               value;

    /* Ensure the instance is valid.  */
    if (gser -> ux_host_class_gser_state !=  UX_HOST_CLASS_INSTANCE_LIVE)
    {        

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, gser, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* The command request will tell us we need to do here.  */
    switch (ioctl_function)
    {

    case UX_HOST_CLASS_GSER_IOCTL_SET_LINE_CODING:
    
        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_GSER_IOCTL_SET_LINE_CODING, gser, parameter, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)
    
        /* Allocate some cache safe memory for the control command.  */
        data_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_GSER_LINE_CODING_LENGTH);
        
        /* Check if error. Return with error if no memory could be allocated.  */
        if (data_buffer == UX_NULL)
            
            /* Do not proceed. Set error code.  */
            status = UX_MEMORY_INSUFFICIENT;
        else
        {
        
            /* Build the buffer from the calling parameter. Cast the calling parameter.  */
            line_coding = (UX_HOST_CLASS_GSER_LINE_CODING *) parameter;
            
            /* Put the data rate.  */
            _ux_utility_long_put(data_buffer + UX_HOST_CLASS_GSER_LINE_CODING_RATE, 
                                line_coding -> ux_host_class_gser_line_coding_dter);
                                
            /* Then the stop bit.  */
            *(data_buffer + UX_HOST_CLASS_GSER_LINE_CODING_STOP_BIT) = 
            (UCHAR) line_coding -> ux_host_class_gser_line_coding_stop_bit;
            
            /* Then the parity.  */
            *(data_buffer + UX_HOST_CLASS_GSER_LINE_CODING_PARITY) = 
            (UCHAR) line_coding -> ux_host_class_gser_line_coding_parity;
            
            /* Finally the data bits.  */
            *(data_buffer + UX_HOST_CLASS_GSER_LINE_CODING_DATA_BIT) = 
            (UCHAR) line_coding -> ux_host_class_gser_line_coding_data_bits;

            /* Send the command to the device.  */
            status = _ux_host_class_gser_command(gser, interface_index, UX_HOST_CLASS_GSER_REQ_SET_LINE_CODING,
                                    0, data_buffer, UX_HOST_CLASS_GSER_LINE_CODING_LENGTH);
            
            /* We free the resources allocated no matter what.  */
            _ux_utility_memory_free(data_buffer);            
        }        
        break;
                    
    case UX_HOST_CLASS_GSER_IOCTL_GET_LINE_CODING:
    
        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_GSER_IOCTL_GET_LINE_CODING, gser, parameter, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)
    
        /* Allocate some cache safe memory for the control command.  */
        data_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_GSER_LINE_CODING_LENGTH);
        
        /* Check if error. Return with error if no memory could be allocated.  */
        if (data_buffer == UX_NULL)
            
            /* Do not proceed. Set error code.  */
            status = UX_MEMORY_INSUFFICIENT;
        else
        {
        
            /* Send the command to the device.  */
            status = _ux_host_class_gser_command(gser, interface_index, UX_HOST_CLASS_GSER_REQ_GET_LINE_CODING,
                                    0, data_buffer, UX_HOST_CLASS_GSER_LINE_CODING_LENGTH);
        
            /* Fill in the calling buffer if the result is successful. */
            if (status == UX_SUCCESS)
            {
            
                /* Build the buffer from the calling parameter. Cast the calling parameter.  */
                line_coding = (UX_HOST_CLASS_GSER_LINE_CODING *) parameter;
            
                /* Get the data rate.  */
                line_coding -> ux_host_class_gser_line_coding_dter = _ux_utility_long_get(data_buffer + UX_HOST_CLASS_GSER_LINE_CODING_RATE);
                                
                /* Then the stop bit.  */
                line_coding -> ux_host_class_gser_line_coding_stop_bit = 
                (ULONG) *(data_buffer + UX_HOST_CLASS_GSER_LINE_CODING_STOP_BIT);
            
                /* Then the parity.  */
                line_coding -> ux_host_class_gser_line_coding_parity = 
                (ULONG) *(data_buffer + UX_HOST_CLASS_GSER_LINE_CODING_PARITY);
            
                /* Finally the data bits.  */
                line_coding -> ux_host_class_gser_line_coding_data_bits =
                (ULONG) *(data_buffer + UX_HOST_CLASS_GSER_LINE_CODING_DATA_BIT); 
            }

            /* We free the resources allocated no matter what.  */
            _ux_utility_memory_free(data_buffer);            
        }        
        break;
            
    case UX_HOST_CLASS_GSER_IOCTL_SET_LINE_STATE:

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_GSER_IOCTL_SET_LINE_STATE, gser, parameter, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* Cast the calling parameter.  */
        line_state = (UX_HOST_CLASS_GSER_LINE_STATE *) parameter;
                    
        /* Build the value field.  */
        value = (line_state -> ux_host_class_gser_line_state_dtr | 
                (line_state -> ux_host_class_gser_line_state_rts << 1));
    
        /* Send the command to the device.  */
        status = _ux_host_class_gser_command(gser, interface_index, UX_HOST_CLASS_GSER_REQ_SET_LINE_STATE,
                                    value, UX_NULL,0);
        break;    

    case UX_HOST_CLASS_GSER_IOCTL_SEND_BREAK :

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_GSER_IOCTL_SEND_BREAK, gser, parameter, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* Build the value field.  */
        value = *((ULONG *) parameter);
    
        /* Send the command to the device.  */
        status = _ux_host_class_gser_command(gser, interface_index, UX_HOST_CLASS_GSER_REQ_SEND_BREAK,
                                    value, UX_NULL,0);
        break;    

    case UX_HOST_CLASS_GSER_IOCTL_ABORT_IN_PIPE :

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_GSER_IOCTL_ABORT_IN_PIPE, gser, gser -> ux_host_class_gser_interface_array[interface_index].ux_host_class_gser_bulk_in_endpoint, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* We need to abort transactions on the bulk In pipe.  */
        _ux_host_stack_endpoint_transfer_abort(gser -> ux_host_class_gser_interface_array[interface_index].ux_host_class_gser_bulk_in_endpoint);
        
        /* Status is successful.  */
        status = UX_SUCCESS;
        break;

    case UX_HOST_CLASS_GSER_IOCTL_ABORT_OUT_PIPE :

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_GSER_IOCTL_ABORT_OUT_PIPE, gser, gser -> ux_host_class_gser_interface_array[interface_index].ux_host_class_gser_bulk_out_endpoint, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* We need to abort transactions on the bulk Out pipe.  */
        _ux_host_stack_endpoint_transfer_abort(gser -> ux_host_class_gser_interface_array[interface_index].ux_host_class_gser_bulk_out_endpoint);

        /* Status is successful.  */
        status = UX_SUCCESS;
        break;

    case UX_HOST_CLASS_GSER_IOCTL_NOTIFICATION_CALLBACK :

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_GSER_IOCTL_NOTIFICATION_CALLBACK, gser, parameter, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* Register a callback when the line state has changed. */
        callback_function = ((VOID (*) (struct UX_HOST_CLASS_GSER_STRUCT *, ULONG, ULONG )) (ALIGN_TYPE)parameter);
        gser -> ux_host_class_gser_device_status_change_callback = callback_function;

        /* Status is successful.  */
        status = UX_SUCCESS;
        break;

    case UX_HOST_CLASS_GSER_IOCTL_GET_DEVICE_STATUS :

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_GSER_IOCTL_GET_DEVICE_STATUS, gser, gser -> ux_host_class_gser_device_state, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* Return the device status.  */
        * ((ULONG *) parameter) = gser -> ux_host_class_gser_device_state;

        /* Status is successful.  */
        status = UX_SUCCESS;
        break;

    default: 

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Function not supported. Return an error.  */
        status =  UX_FUNCTION_NOT_SUPPORTED;
    }   

    /* Return status to caller.  */
    return(status);
}

