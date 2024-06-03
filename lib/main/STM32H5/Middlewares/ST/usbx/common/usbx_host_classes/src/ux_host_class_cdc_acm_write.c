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
/**   CDC ACM Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_cdc_acm.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_cdc_acm_write                        PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function writes to the cdc_acm interface. The call is blocking */ 
/*    and only returns when there is either an error or when the transfer */ 
/*    is complete.                                                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    cdc_acm                               Pointer to cdc_acm class      */ 
/*    data_pointer                          Pointer to data to write      */ 
/*    requested_length                      Length of data to write       */ 
/*    actual_length                         Actual length of data written */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_host_stack_transfer_request_abort Abort transfer request        */ 
/*    _ux_host_semaphore_get                Get protection semaphore      */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  12-31-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed ZLP sending,          */
/*                                            resulting in version 6.1.3  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_cdc_acm_write(UX_HOST_CLASS_CDC_ACM *cdc_acm, UCHAR *data_pointer, 
                                    ULONG requested_length, ULONG *actual_length)
{

UX_TRANSFER     *transfer_request;
UINT            status;
ULONG           transfer_request_length;
#if defined(UX_HOST_STANDALONE)
ULONG           transfer_flags;
#endif
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_CDC_ACM_WRITE, cdc_acm, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Ensure the instance is valid.  */
    if (cdc_acm -> ux_host_class_cdc_acm_state !=  UX_HOST_CLASS_INSTANCE_LIVE)
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, cdc_acm, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* As further protection, we must ensure this instance of the interface is the data interface and not
       the control interface !  */
    if (cdc_acm -> ux_host_class_cdc_acm_interface -> ux_interface_descriptor.bInterfaceClass != UX_HOST_CLASS_CDC_DATA_CLASS)
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, cdc_acm, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

#if defined(UX_HOST_STANDALONE)
    if (cdc_acm -> ux_host_class_cdc_acm_write_state == UX_STATE_WAIT)
        return(UX_BUSY);
    cdc_acm -> ux_host_class_cdc_acm_write_state = UX_STATE_WAIT;
#endif

    /* Start by resetting the actual length of the transfer.  */
    *actual_length =  0;

    /* Get the pointer to the bulk out endpoint transfer request.  */
    transfer_request =  &cdc_acm -> ux_host_class_cdc_acm_bulk_out_endpoint -> ux_endpoint_transfer_request;

#if defined(UX_HOST_STANDALONE)

    /* Enable auto wait.  */
    transfer_flags = transfer_request -> ux_transfer_request_flags;
    transfer_request -> ux_transfer_request_flags |= UX_TRANSFER_FLAG_AUTO_WAIT;
#endif

    /* Perform a transfer on the bulk out endpoint until either the transfer is
       completed or when there is an error.  */
    do
    {

        /* Program the maximum authorized length for this transfer_request.  */
        if (requested_length > transfer_request -> ux_transfer_request_maximum_length)
            transfer_request_length =  transfer_request -> ux_transfer_request_maximum_length;
        else
            transfer_request_length =  requested_length;
                    
        /* Initialize the transfer_request.  */
        transfer_request -> ux_transfer_request_data_pointer =  data_pointer;
        transfer_request -> ux_transfer_request_requested_length =  transfer_request_length;
        
        /* Perform the transfer.  */
        status =  _ux_host_stack_transfer_request(transfer_request);

        /* If the transfer is successful, we need to wait for the transfer request to be completed.  */
        if (status == UX_SUCCESS)
        {

#if !defined(UX_HOST_STANDALONE)
            /* Wait for the completion of the transfer request.  */
            status =  _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore,
                                             transfer_request -> ux_transfer_request_timeout_value);

            /* If the semaphore did not succeed we probably have a time out.  */
            if (status != UX_SUCCESS)
            {

                /* All transfers pending need to abort. There may have been a partial transfer.  */
                _ux_host_stack_transfer_request_abort(transfer_request);
                
                /* Update the length of the actual data transferred. We do this after the 
                   abort of the transfer_request in case some data actually went out.  */
                *actual_length +=  transfer_request -> ux_transfer_request_actual_length;
            
                /* Set the completion code.  */
                transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_TIMEOUT;
        
                /* Error trap. */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_TIMEOUT);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_TIMEOUT, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)
        
                /* There was an error, return to the caller.  */
                return(UX_TRANSFER_TIMEOUT);
            }
#endif
        }
        else
        {

#if defined(UX_HOST_STANDALONE)

            /* Update the length of the actual data transferred. We do this after the 
                abort of the transfer request in case some data was actually received.  */
            *actual_length +=  transfer_request -> ux_transfer_request_actual_length;

            /* Restore previous setting.  */
            transfer_request -> ux_transfer_request_flags = transfer_flags;

            /* Not busy any more.  */
            cdc_acm -> ux_host_class_cdc_acm_write_state = UX_STATE_RESET;
#endif

            /* There was a non transfer error, no partial transfer to be checked */
            return(status);
        }

        /* Update the length of the transfer. Normally all the data has to be sent.  */
        *actual_length +=  transfer_request -> ux_transfer_request_actual_length;
        
        /* Check for completion of transfer. If the transfer is partial, return to caller.
           The transfer is marked as successful but the caller will need to check the length
           actually sent and determine if a partial transfer is OK. */
        if (transfer_request_length !=  transfer_request -> ux_transfer_request_actual_length)
        {

            /* Return success.  */
            return(UX_SUCCESS);
        }

        /* Update the data pointer for next transfer.  */        
        data_pointer +=  transfer_request_length;
        
        /* Update what is left to send out.  */
        requested_length -=  transfer_request_length;          

    } while (requested_length);

#if defined(UX_HOST_STANDALONE)

    /* Restore previous setting.  */
    transfer_request -> ux_transfer_request_flags = transfer_flags;

    /* Not busy any more.  */
    cdc_acm -> ux_host_class_cdc_acm_write_state = UX_STATE_RESET;
#endif

    /* We get here when all the transfers went through without errors.  */
    return(UX_SUCCESS); 
}

