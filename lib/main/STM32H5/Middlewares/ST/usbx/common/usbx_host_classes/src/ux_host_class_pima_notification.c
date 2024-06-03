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
/**   Pima Class                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_pima.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_pima_notification                    PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called by the completion thread when a transfer    */ 
/*    request has been completed either because the transfer is           */ 
/*    successful or there was an error.                                   */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    transfer_request                      Pointer to transfer request   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Transfer request              */
/*    _ux_utility_memory_copy               Copy memory                   */
/*    _ux_utility_short_get                 Get 16-bit value              */
/*    _ux_utility_long_get                  Get 32-bit value              */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX stack                                                          */ 
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
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_pima_notification(UX_TRANSFER *transfer_request)
{

UX_HOST_CLASS_PIMA                       *pima;
UX_HOST_CLASS_PIMA_SESSION                 *pima_session;
UX_HOST_CLASS_PIMA_EVENT                pima_event;

    /* Get the class instance for this transfer request.  */
    pima =  (UX_HOST_CLASS_PIMA *) transfer_request -> ux_transfer_request_class_instance;
    
    /* Check the state of the transfer.  If there is an error, we do not proceed with this notification.  */
    if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* We have an error. We do not rehook another transfer if the device instance is shutting down or
           if the transfer was aborted by the class..  */
        if ((pima -> ux_host_class_pima_state ==  UX_HOST_CLASS_INSTANCE_SHUTDOWN) || 
            (transfer_request -> ux_transfer_request_completion_code == UX_TRANSFER_STATUS_ABORT))

            /* We do not proceed.  */
            return;
        else

        {            

            /* Reactivate the PIMA interrupt pipe.  */
            _ux_host_stack_transfer_request(transfer_request);
        
            /* We do not proceed.  */
            return;        
        }            
        
    }

    /* Check if this packet is the first, if so we have the total length expected in the event.  */
    if (pima -> ux_host_class_pima_event_buffer_current_length == 0)
    {

        /* First packet. Maybe the only one needed. It may happen that the notification event is split amongst
           several interrupt packets.  */
        pima -> ux_host_class_pima_event_buffer_expected_length  = _ux_utility_long_get(transfer_request -> ux_transfer_request_data_pointer 
                                                                                            + UX_HOST_CLASS_PIMA_AEI_DATA_LENGTH);

        /* Set the current offset to the beginning of the buffer.  */
        pima -> ux_host_class_pima_event_buffer_current_offset =  pima -> ux_host_class_pima_event_buffer;
            
    }
    
    /* Check the length of this payload and make sure we have enough space.  */
    if ((pima -> ux_host_class_pima_event_buffer_current_length + transfer_request -> ux_transfer_request_actual_length) <= 
            UX_HOST_CLASS_PIMA_AEI_MAX_LENGTH)
    {
    
        /* We copy the current payload into our event notification buffer.  */
        _ux_utility_memory_copy(pima -> ux_host_class_pima_event_buffer_current_offset, transfer_request -> ux_transfer_request_data_pointer, 
                            transfer_request -> ux_transfer_request_actual_length); /* Use case of memcpy is verified. */
                            
        /* Set the new offset address. */
        pima -> ux_host_class_pima_event_buffer_current_offset += transfer_request -> ux_transfer_request_actual_length;
    
        /* Adjust the length.  */
        pima -> ux_host_class_pima_event_buffer_current_length += transfer_request -> ux_transfer_request_actual_length;
    }
    else
    
        /* We come here when we have a buffer overflow. Do not proceed.  */
        return;            

    /* Check if we have a complete notification event.  */
    if (pima -> ux_host_class_pima_event_buffer_current_length == pima -> ux_host_class_pima_event_buffer_expected_length)
    {

        /* Save the current event in the pima instance.  First, unpack the event code.  */
        pima -> ux_host_class_pima_event_code = _ux_utility_short_get(pima -> ux_host_class_pima_event_buffer + UX_HOST_CLASS_PIMA_AEI_EVENT_CODE);
        
        /* Unpack the Transaction ID.  */
        pima -> ux_host_class_pima_event_transaction_id = _ux_utility_long_get(pima -> ux_host_class_pima_event_buffer + UX_HOST_CLASS_PIMA_AEI_TRANSACTION_ID);
    
        /* Unpack the parameter 1.  */
        pima -> ux_host_class_pima_event_parameter_1 = _ux_utility_long_get(pima -> ux_host_class_pima_event_buffer + UX_HOST_CLASS_PIMA_AEI_PARAMETER_1);
        
        /* Unpack the parameter 2.  */
        pima -> ux_host_class_pima_event_parameter_2 = _ux_utility_long_get(pima -> ux_host_class_pima_event_buffer + UX_HOST_CLASS_PIMA_AEI_PARAMETER_2);
        
        /* Unpack the parameter 3.  */
        pima -> ux_host_class_pima_event_parameter_3 = _ux_utility_long_get(pima -> ux_host_class_pima_event_buffer + UX_HOST_CLASS_PIMA_AEI_PARAMETER_3);
        
        /* Check if a session is valid.  */
        if (pima -> ux_host_class_pima_session != UX_NULL)
        {
    
            /* Get session pointer.  */
            pima_session =  pima -> ux_host_class_pima_session;
            
            /* Check if this session is valid or not.  */
            if ((pima_session -> ux_host_class_pima_session_magic == UX_HOST_CLASS_PIMA_MAGIC_NUMBER) &&
               (pima_session -> ux_host_class_pima_session_state == UX_HOST_CLASS_PIMA_SESSION_STATE_OPENED))
            {
    
                /* If the application demanded a callback when event occur, create a event notification
                   message.  */
                if (pima_session -> ux_host_class_pima_session_event_callback != UX_NULL)
                {
    
                    /* Fill in the pima event structure. First with pima instance.  */
                    pima_event.ux_host_class_pima_event_pima_instance = pima;
                    
                    /* Fill in the opened session pointer.  Since the pima class only supports
                       one session, this address is hardwired to the opened session and is not coming from
                       the event buffer.  */
                    pima_event.ux_host_class_pima_event_session = pima_session;
                    
                    /* Fill in all the event parameters in the event callback structure.  */
                    pima_event.ux_host_class_pima_event_code =  pima -> ux_host_class_pima_event_code;
                    pima_event.ux_host_class_pima_event_transaction_id =  pima -> ux_host_class_pima_event_transaction_id;
                    pima_event.ux_host_class_pima_event_parameter_1 =  pima -> ux_host_class_pima_event_parameter_1;
                    pima_event.ux_host_class_pima_event_parameter_2 =  pima -> ux_host_class_pima_event_parameter_2;
                    pima_event.ux_host_class_pima_event_parameter_3 =  pima -> ux_host_class_pima_event_parameter_3;
                    
                    /* Send this event to the application.  */
                    pima_session -> ux_host_class_pima_session_event_callback(&pima_event);

                    /* If trace is enabled, insert this event into the trace buffer.  */
                    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PIMA_NOTIFICATION, pima, 
                                            pima -> ux_host_class_pima_event_code, 
                                            pima -> ux_host_class_pima_event_transaction_id, 
                                            pima -> ux_host_class_pima_event_parameter_1, 
                                            UX_TRACE_HOST_CLASS_EVENTS, 0, 0) 
    
                }
            }
        }
        
        /* We will receive a complete new transaction.  */
        pima -> ux_host_class_pima_event_buffer_current_length =  0;
    }    
    /* Reactivate the PIMA interrupt pipe.  */
    _ux_host_stack_transfer_request(transfer_request);

    /* Return to caller.  */
    return;
}

