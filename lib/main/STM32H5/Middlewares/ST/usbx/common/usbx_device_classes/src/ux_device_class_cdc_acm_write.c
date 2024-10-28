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


#if !defined(UX_DEVICE_STANDALONE)
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_cdc_acm_write                      PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function writes to  the CDC class.                             */ 
/*                                                                        */ 
/*    It's for RTOS mode.                                                 */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    cdc_acm                               Address of cdc_acm class      */ 
/*                                                instance                */ 
/*    buffer                                Pointer to data to write      */
/*    requested_length                      Length of bytes to write,     */
/*                                                set to 0 to issue ZLP   */
/*    actual_length                         Pointer to save number of     */
/*                                                bytes written           */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*   _ux_utility_memory_copy                Copy memory                   */ 
/*   _ux_device_stack_transfer_request      Transfer request              */ 
/*   _ux_device_mutex_on                    Take Mutex                    */ 
/*   _ux_device_mutex_off                   Release Mutex                 */ 
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
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile issue,        */
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            added auto ZLP support,     */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_cdc_acm_write(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UCHAR *buffer, 
                                ULONG requested_length, ULONG *actual_length)
{

UX_SLAVE_ENDPOINT           *endpoint;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_INTERFACE          *interface_ptr;
UX_SLAVE_TRANSFER           *transfer_request;
ULONG                       local_requested_length;
ULONG                       local_host_length;
UINT                        status = 0;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_CDC_ACM_WRITE, cdc_acm, buffer, requested_length, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE

    /* Check if current cdc-acm is using callback or not. We cannot use direct reads with callback on.  */
    if (cdc_acm -> ux_slave_class_cdc_acm_transmission_status == UX_TRUE)
    
        /* Not allowed. */
        return(UX_ERROR);
#endif

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;
    
    /* As long as the device is in the CONFIGURED state.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
    {
            
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CONFIGURATION_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONFIGURATION_HANDLE_UNKNOWN, device, 0, 0, UX_TRACE_ERRORS, 0, 0)
    
        /* Cannot proceed with command, the interface is down.  */
        return(UX_CONFIGURATION_HANDLE_UNKNOWN);
    }
        
    /* We need the interface to the class.  */
    interface_ptr =  cdc_acm -> ux_slave_class_cdc_acm_interface;
    
    /* Locate the endpoints.  */
    endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;
    
    /* Check the endpoint direction, if IN we have the correct endpoint.  */
    if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_IN)
    {

        /* So the next endpoint has to be the IN endpoint.  */
        endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
    }

    /* Protect this thread.  */
    _ux_device_mutex_on(&cdc_acm -> ux_slave_class_cdc_acm_endpoint_in_mutex);
        
    /* We are writing to the IN endpoint.  */
    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

    /* Reset the actual length.  */
    *actual_length =  0;

    /* Check if the application forces a 0 length packet.  */
    if (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED && requested_length == 0)
    {
        
        /* Send the request for 0 byte packet to the device controller.  */
        status =  _ux_device_stack_transfer_request(transfer_request, 0, 0);

        /* Free Mutex resource.  */
        _ux_device_mutex_off(&cdc_acm -> ux_slave_class_cdc_acm_endpoint_in_mutex);

        /* Return the status.  */
        return(status);
        

    }
    else
    {    
        /* Check if we need more transactions.  */
        local_host_length = UX_SLAVE_REQUEST_DATA_MAX_LENGTH;
        while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED && requested_length != 0)
        { 
    
            /* Check if we have enough in the local buffer.  */
            if (requested_length > UX_SLAVE_REQUEST_DATA_MAX_LENGTH)
    
                /* We have too much to transfer.  */
                local_requested_length = UX_SLAVE_REQUEST_DATA_MAX_LENGTH;
                
            else
            {

                /* We can proceed with the demanded length.  */
                local_requested_length = requested_length;

#if !defined(UX_DEVICE_CLASS_CDC_ACM_WRITE_AUTO_ZLP)

                /* Assume the length match expectation.  */
                local_host_length = requested_length;
#else

                /* Assume expecting more, so ZLP is appended in stack.  */
                local_host_length = UX_SLAVE_REQUEST_DATA_MAX_LENGTH + 1;
#endif
            }
                            
            /* On a out, we copy the buffer to the caller. Not very efficient but it makes the API
               easier.  */
            _ux_utility_memory_copy(transfer_request -> ux_slave_transfer_request_data_pointer,
                                buffer, local_requested_length); /* Use case of memcpy is verified. */
        
            /* Send the request to the device controller.  */
            status =  _ux_device_stack_transfer_request(transfer_request, local_requested_length, local_host_length);
        
            /* Check the status */    
            if (status == UX_SUCCESS)
            {
    
                /* Next buffer address.  */
                buffer += transfer_request -> ux_slave_transfer_request_actual_length;
    
                /* Set the length actually received. */
                *actual_length += transfer_request -> ux_slave_transfer_request_actual_length; 
    
                /* Decrement what left has to be done.  */
                requested_length -= transfer_request -> ux_slave_transfer_request_actual_length;
    
            }
            
            else
            {
             
                /* Free Mutex resource.  */
                _ux_device_mutex_off(&cdc_acm -> ux_slave_class_cdc_acm_endpoint_in_mutex);
                
                /* We had an error, abort.  */
                return(status);
            }
        }
    }

    
    /* Free Mutex resource.  */
    _ux_device_mutex_off(&cdc_acm -> ux_slave_class_cdc_acm_endpoint_in_mutex);

    /* Check why we got here, either completion or device was extracted.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
    {
            
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_NO_ANSWER);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_NO_ANSWER, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Device must have been extracted.  */
        return (UX_TRANSFER_NO_ANSWER);
    }
    else
    
        /* Simply return the last transaction result.  */
        return(status);        
          
}
#endif
