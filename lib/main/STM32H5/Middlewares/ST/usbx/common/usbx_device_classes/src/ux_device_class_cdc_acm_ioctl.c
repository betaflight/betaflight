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
/**   Device CDC Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_cdc_acm_ioctl                      PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs certain functions on the cdc acm instance    */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    cdc_acm                               Address of cdc_acm class      */ 
/*                                                instance                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Status                                                              */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_abort           Abort transfer            */
/*    _ux_utility_memory_allocate               Allocate memory           */
/*    _ux_utility_memory_free                   Free memory               */
/*    _ux_utility_event_flags_create            Create event flags        */
/*    _ux_utility_event_flags_delete            Delete event flags        */
/*    _ux_device_thread_create                  Create thread             */
/*    _ux_device_thread_delete                  Delete thread             */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    ThreadX                                                             */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  03-02-2021     Xiuwen Cai               Modified comment(s), removed  */
/*                                            unreachable statement,      */
/*                                            resulting in version 6.1.5  */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added macro to disable      */
/*                                            transmission support,       */
/*                                            moved transmission resource */
/*                                            management to init/uninit,  */
/*                                            resulting in version 6.1.6  */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile issue,        */
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            fixed aborting return code, */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_cdc_acm_ioctl(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, ULONG ioctl_function,
                                    VOID *parameter)
{

UINT                                                status;
UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER        *line_coding;
UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_PARAMETER         *line_state;
#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE
UX_SLAVE_CLASS_CDC_ACM_CALLBACK_PARAMETER           *callback;
#endif
UX_SLAVE_ENDPOINT                                   *endpoint;
UX_SLAVE_INTERFACE                                  *interface_ptr;
UX_SLAVE_TRANSFER                                   *transfer_request;

    /* Let's be optimist ! */
    status = UX_SUCCESS;

    /* The command request will tell us what we need to do here.  */
    switch (ioctl_function)
    {

        case UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING:
    
            /* Properly cast the parameter pointer.  */
            line_coding = (UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER *) parameter;
    
            /* Save the parameters in the cdc_acm function.  */
            cdc_acm -> ux_slave_class_cdc_acm_baudrate  =  line_coding -> ux_slave_class_cdc_acm_parameter_baudrate;
            cdc_acm -> ux_slave_class_cdc_acm_stop_bit  =  line_coding -> ux_slave_class_cdc_acm_parameter_stop_bit;
            cdc_acm -> ux_slave_class_cdc_acm_parity    =  line_coding -> ux_slave_class_cdc_acm_parameter_parity;
            cdc_acm -> ux_slave_class_cdc_acm_data_bit  =  line_coding -> ux_slave_class_cdc_acm_parameter_data_bit;
            
            break;
            
        case UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING:
    
            /* Properly cast the parameter pointer.  */
            line_coding = (UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER *) parameter;
    
            /* Save the parameters in the cdc_acm function.  */
            line_coding -> ux_slave_class_cdc_acm_parameter_baudrate = cdc_acm -> ux_slave_class_cdc_acm_baudrate;
            line_coding -> ux_slave_class_cdc_acm_parameter_stop_bit = cdc_acm -> ux_slave_class_cdc_acm_stop_bit;
            line_coding -> ux_slave_class_cdc_acm_parameter_parity   = cdc_acm -> ux_slave_class_cdc_acm_parity;
            line_coding -> ux_slave_class_cdc_acm_parameter_data_bit = cdc_acm -> ux_slave_class_cdc_acm_data_bit;
            
            break;
            

        case UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_STATE:
        
            /* Properly cast the parameter pointer.  */
            line_state = (UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_PARAMETER *) parameter;
    
            /* Return the DTR/RTS signals.  */
            line_state -> ux_slave_class_cdc_acm_parameter_rts = cdc_acm -> ux_slave_class_cdc_acm_data_rts_state;
            line_state -> ux_slave_class_cdc_acm_parameter_dtr = cdc_acm -> ux_slave_class_cdc_acm_data_dtr_state;
            
            break;
            
        case UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_STATE:
        
            /* Properly cast the parameter pointer.  */
            line_state = (UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_PARAMETER *) parameter;
    
            /* Set the DTR/RTS signals.  */
            cdc_acm -> ux_slave_class_cdc_acm_data_rts_state = line_state -> ux_slave_class_cdc_acm_parameter_rts;
            cdc_acm -> ux_slave_class_cdc_acm_data_dtr_state = line_state -> ux_slave_class_cdc_acm_parameter_dtr;
            
            break;
            

        case UX_SLAVE_CLASS_CDC_ACM_IOCTL_ABORT_PIPE:

            /* Get the interface from the instance.  */
            interface_ptr =  cdc_acm -> ux_slave_class_cdc_acm_interface;
    
            /* Locate the endpoints.  */
            endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;
            
            /* What direction ?  */
            switch( (ULONG) (ALIGN_TYPE) parameter)
            {
                case UX_SLAVE_CLASS_CDC_ACM_ENDPOINT_XMIT : 
    
                /* Check the endpoint direction, if IN we have the correct endpoint.  */
                if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_IN)
                {

                    /* So the next endpoint has to be the XMIT endpoint.  */
                    endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
                }
                break;
                
                case UX_SLAVE_CLASS_CDC_ACM_ENDPOINT_RCV : 
    
                /* Check the endpoint direction, if OUT we have the correct endpoint.  */
                if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_OUT)
                {

                    /* So the next endpoint has to be the RCV endpoint.  */
                    endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
                }
                break;
                


                default :
                
                /* Parameter not supported. Return an error.  */
                status =  UX_ENDPOINT_HANDLE_UNKNOWN;
            }
        
            /* Get the transfer request associated with the endpoint.  */
            transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

#if defined(UX_DEVICE_STANDALONE)

            /* Abort the transfer.  */
            _ux_device_stack_transfer_abort(transfer_request, UX_TRANSFER_STATUS_ABORT);
            if ((ULONG) (ALIGN_TYPE) parameter == UX_SLAVE_CLASS_CDC_ACM_ENDPOINT_XMIT)
                cdc_acm -> ux_device_class_cdc_acm_write_state = UX_STATE_RESET;
            else
                cdc_acm -> ux_device_class_cdc_acm_read_state = UX_STATE_RESET;
#else

            /* Check the status of the transfer. */ 
            if (transfer_request -> ux_slave_transfer_request_status ==  UX_TRANSFER_STATUS_PENDING)
            {

                /* Abort the transfer.  */
            _ux_device_stack_transfer_abort(transfer_request, UX_ABORTED);

            }
#endif
            break;

        case UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_READ_TIMEOUT:
        case UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_WRITE_TIMEOUT:

            /* Get the interface from the instance.  */
            interface_ptr =  cdc_acm -> ux_slave_class_cdc_acm_interface;

            /* Locate the endpoints.  */
            endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;

            /* If it's reading timeout but endpoint is OUT, it should be the next one.  */
            if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) !=
                (ULONG)((ioctl_function == UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_READ_TIMEOUT) ? UX_ENDPOINT_OUT : UX_ENDPOINT_IN))
                endpoint = endpoint -> ux_slave_endpoint_next_endpoint;

            /* Get the transfer request associated with the endpoint.  */
            transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

            /* Check the status of the transfer.  */ 
            if (transfer_request -> ux_slave_transfer_request_status ==  UX_TRANSFER_STATUS_PENDING)
                status = UX_ERROR;
            else
                transfer_request -> ux_slave_transfer_request_timeout = (ULONG) (ALIGN_TYPE) parameter;

            break;

#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE

        case UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_START:

            /* Check if we are in callback transmission already.  */
            if (cdc_acm -> ux_slave_class_cdc_acm_transmission_status == UX_TRUE)
            {
                /* We should not to that ! */
                return(UX_ERROR);
            
            }
            
            /* Properly cast the parameter pointer.  */
            callback = (UX_SLAVE_CLASS_CDC_ACM_CALLBACK_PARAMETER *) parameter;

            /* Save the callback function for write.  */
            cdc_acm -> ux_device_class_cdc_acm_write_callback  = callback -> ux_device_class_cdc_acm_parameter_write_callback;

            /* Save the callback function for read.  */
            cdc_acm -> ux_device_class_cdc_acm_read_callback = callback -> ux_device_class_cdc_acm_parameter_read_callback;

#if !defined(UX_DEVICE_STANDALONE)

            /* Start transmission threads.  */
            _ux_utility_thread_resume(&cdc_acm -> ux_slave_class_cdc_acm_bulkin_thread);
            _ux_utility_thread_resume(&cdc_acm -> ux_slave_class_cdc_acm_bulkout_thread);
#endif

            /* Declare the transmission with callback on.  */
            cdc_acm -> ux_slave_class_cdc_acm_transmission_status = UX_TRUE;
            
            /* We are done here.  */
            return(UX_SUCCESS);

        case UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_STOP:
        
            /* Check if we are in callback transmission already.  */
            if (cdc_acm -> ux_slave_class_cdc_acm_transmission_status == UX_TRUE)
            {
        
                /* Get the interface from the instance.  */
                interface_ptr =  cdc_acm -> ux_slave_class_cdc_acm_interface;
    
                /* Locate the endpoints.  */
                endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;

                /* Get the transfer request associated with the endpoint.  */
                transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;
                
                /* Abort the transfer.  */
                _ux_device_stack_transfer_abort(transfer_request, UX_ABORTED);
        
                /* Next endpoint.  */
                endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;

                /* Get the transfer request associated with the endpoint.  */
                transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;
                
                /* Abort the transfer.  */
                _ux_device_stack_transfer_abort(transfer_request, UX_ABORTED);

#if !defined(UX_DEVICE_STANDALONE)

                /* Suspend threads.  */
                _ux_device_thread_suspend(&cdc_acm -> ux_slave_class_cdc_acm_bulkin_thread);
                _ux_device_thread_suspend(&cdc_acm -> ux_slave_class_cdc_acm_bulkout_thread);
#endif

                /* Clear scheduled write flag.  */
                cdc_acm -> ux_slave_class_cdc_acm_scheduled_write = UX_FALSE;

                /* Declare the transmission with callback off.  */
                cdc_acm -> ux_slave_class_cdc_acm_transmission_status = UX_FALSE;
            }
            else
                
                /* We should not try to stop an non existing transmission.  */
                return(UX_ERROR);                

            break;                
#endif

        default: 

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_FUNCTION_NOT_SUPPORTED);
    
            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)
    
            /* Function not supported. Return an error.  */
            status =  UX_FUNCTION_NOT_SUPPORTED;
    }   

    /* Return status to caller.  */
    return(status);
          
}

