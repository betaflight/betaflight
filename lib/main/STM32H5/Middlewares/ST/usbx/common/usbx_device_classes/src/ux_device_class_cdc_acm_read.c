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
/*    _ux_device_class_cdc_acm_read                       PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function reads from the CDC class.                             */ 
/*                                                                        */ 
/*    It's for RTOS mode.                                                 */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    cdc_acm                                   Address of cdc_acm class  */ 
/*                                              instance                  */ 
/*    buffer                                    Pointer to buffer to save */
/*                                              received data             */
/*    requested_length                          Length of bytes to read   */
/*    actual_length                             Pointer to save number of */
/*                                              bytes read                */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_request     Transfer request              */ 
/*    _ux_utility_memory_copy               Copy memory                   */ 
/*    _ux_device_mutex_off                  Release mutex                 */ 
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
/*  01-31-2022x    Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_cdc_acm_read(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UCHAR *buffer, 
                                ULONG requested_length, ULONG *actual_length)
{

UX_SLAVE_ENDPOINT           *endpoint;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_INTERFACE          *interface_ptr;
UX_SLAVE_TRANSFER           *transfer_request;
UINT                        status= UX_SUCCESS;
ULONG                       local_requested_length;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_CDC_ACM_READ, cdc_acm, buffer, requested_length, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

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
    
    /* This is the first time we are activated. We need the interface to the class.  */
    interface_ptr =  cdc_acm -> ux_slave_class_cdc_acm_interface;
    
    /* Locate the endpoints.  */
    endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;
    
    /* Check the endpoint direction, if OUT we have the correct endpoint.  */
    if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_OUT)
    {

        /* So the next endpoint has to be the OUT endpoint.  */
        endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
    }

    /* Protect this thread.  */
    _ux_device_mutex_on(&cdc_acm -> ux_slave_class_cdc_acm_endpoint_out_mutex);
        
    /* All CDC reading  are on the endpoint OUT, from the host.  */
    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;
    
    /* Reset the actual length.  */
    *actual_length =  0;
    
    /* Check if we need more transactions.  */
    while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED && requested_length != 0)
    { 
        
        /* Check if we have enough in the local buffer.  */
        if (requested_length > endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize)
    
            /* We have too much to transfer.  */
            local_requested_length = endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize;
            
        else
        
            /* We can proceed with the demanded length.  */
            local_requested_length = requested_length;
        
        /* Send the request to the device controller.  */
        status =  _ux_device_stack_transfer_request(transfer_request, local_requested_length, local_requested_length);
        
        /* Check the status */    
        if (status == UX_SUCCESS)
        {

            /* We need to copy the buffer locally.  */
            _ux_utility_memory_copy(buffer, transfer_request -> ux_slave_transfer_request_data_pointer,
                            transfer_request -> ux_slave_transfer_request_actual_length); /* Use case of memcpy is verified. */
    
            /* Next buffer address.  */
            buffer += transfer_request -> ux_slave_transfer_request_actual_length;
    
            /* Set the length actually received. */
            *actual_length += transfer_request -> ux_slave_transfer_request_actual_length; 
    
            /* Decrement what left has to be done.  */
            requested_length -= transfer_request -> ux_slave_transfer_request_actual_length;


            /* Is this a short packet or a ZLP indicating we are done with this transfer ?  */
            if (transfer_request -> ux_slave_transfer_request_actual_length < endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize)
            {            

                /* We are done.  */
                /* Free Mutex resource.  */
                _ux_device_mutex_off(&cdc_acm -> ux_slave_class_cdc_acm_endpoint_out_mutex);
    
                /* Return with success.  */
                return(UX_SUCCESS);

            }
        }
        else
        {
            
            /* Free Mutex resource.  */
            _ux_device_mutex_off(&cdc_acm -> ux_slave_class_cdc_acm_endpoint_out_mutex);
    
            /* We got an error.  */
            return(status);
        }            
    }

    
    /* Free Mutex resource.  */
    _ux_device_mutex_off(&cdc_acm -> ux_slave_class_cdc_acm_endpoint_out_mutex);

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
