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
/**   OHCI Controller Driver                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_hcd_ohci.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_hcd_ohci_done_queue_process                     PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function process the done queue that was posted by the        */ 
/*     controller during the last interrupt. The bad news is that the     */ 
/*     list of the TDs in the queue is in the opposite order of their     */ 
/*     actual completion. This FIFO made the OHCI design easier but the   */ 
/*     software has to work harder!                                       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ohci                              Pointer to OHCI HCD           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    (ux_transfer_request_completion_function)                           */ 
/*                                          Transfer completion function  */ 
/*    _ux_hcd_ohci_endpoint_error_clear     Clear endpoint error          */ 
/*    _ux_hcd_ohci_endpoint_reset           Reset endpoint                */ 
/*    _ux_hcd_ohci_frame_number_get         Get frame number              */ 
/*    _ux_hcd_ohci_next_td_clean            Clean next TD                 */ 
/*    _ux_hcd_ohci_register_read            Read OHCI register            */ 
/*    _ux_hcd_ohci_register_write           Write OHCI register           */ 
/*    _ux_host_semaphore_put                Put producer semaphore        */ 
/*    _ux_utility_virtual_address           Get virtual address           */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    OHCI Controller Driver                                              */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed an addressing issue,  */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_hcd_ohci_done_queue_process(UX_HCD_OHCI *hcd_ohci)
{

UX_ENDPOINT         *endpoint;
UX_OHCI_TD          *td;
UX_OHCI_ISO_TD      *iso_td;
UX_OHCI_TD          *previous_td;
UX_OHCI_TD          *next_td;
UINT                td_error_code;
UX_TRANSFER         *transfer_request;
ULONG               ohci_register_interrupt;
ULONG               transaction_length;
ULONG               current_frame;


    /* Get the first entry of the done queue. It may be NULL which means there is nothing to do! 
       The LSB of the TD pointer may be set by the OHCI controller to indicate that the Interrupt 
       status of the controller should be read. The TDs in the list are using physical addresses. 
       They need to be translated into virtual addresses.  */
    next_td =  _ux_utility_virtual_address((VOID *) ((ULONG) hcd_ohci -> ux_hcd_ohci_done_head & 0xfffffff0));
    
    /* Reset the last TD in the chain.  */
    previous_td =  UX_NULL;
    td =           UX_NULL;

    /* The TD we have now is the last in the FIFO, re-traverse the chain to get the TDs in the 
       chronological order.  */
    while (next_td != UX_NULL)
    {

        td =                        next_td;
        next_td =                   _ux_utility_virtual_address(td -> ux_ohci_td_next_td);
        td -> ux_ohci_td_next_td =  _ux_utility_physical_address(previous_td);
        previous_td =               td;
    }

    /* Process each TD in their chronological order now. The TD pointer now has the first TD in the 
       list, all values are in virtual addresses.  */
    while (td != UX_NULL)
    {   

        /* Get the pointer to the transfer request attached with this TD.  */
        transfer_request =  td -> ux_ohci_td_transfer_request;

        /* Get the endpoint associated with the transfer request */
        endpoint =  transfer_request -> ux_transfer_request_endpoint;

        /* Retrieve the error code for this transaction. There are 3 types of errors: 
           transmission, sequence, system.  */
        td_error_code =  td -> ux_ohci_td_dw0 >> UX_OHCI_TD_CC;

        /* The handling of the isoch TD is slightly different.  */
        switch ((endpoint -> ux_endpoint_descriptor.bmAttributes) & UX_MASK_ENDPOINT_TYPE)
        {

        case UX_CONTROL_ENDPOINT:
        case UX_BULK_ENDPOINT:
        case UX_INTERRUPT_ENDPOINT:

            switch (td_error_code)
            {

            case UX_OHCI_NO_ERROR:

                /* No error on the transmission of this TD. Update the length of the transfer request.  */
                transfer_request -> ux_transfer_request_actual_length +=  td -> ux_ohci_td_length;

                /* Check at the phase of the transfer, if this is a SETUP or DATA phases for a control 
                   endpoint, we wait for the setup phase or any phase for other types of endpoints.  */
                if ((td -> ux_ohci_td_status & UX_OHCI_TD_SETUP_PHASE) || (td -> ux_ohci_td_status & UX_OHCI_TD_DATA_PHASE))
                    break;

                /* Either this is a non control endpoint or it is the status phase and we are done.  */
                if (transfer_request -> ux_transfer_request_actual_length == transfer_request -> ux_transfer_request_requested_length)
                {

                    transfer_request -> ux_transfer_request_completion_code =  UX_SUCCESS;
                    if (transfer_request -> ux_transfer_request_completion_function != UX_NULL)
                        transfer_request -> ux_transfer_request_completion_function(transfer_request);
                    _ux_host_semaphore_put(&transfer_request -> ux_transfer_request_semaphore);
                }
                break;


            case UX_OHCI_ERROR_DATA_UNDERRUN:

                /* No error on the transmission of this TD but all data is not accounted for. This is typically 
                   a short packet and OHCI report it as an error. This allows for the ED to be halted and further 
                   attached TDs to be stopped. In this case, compute the correct received\sent length and
                   process the transfer request. */
                transfer_request -> ux_transfer_request_actual_length +=  td -> ux_ohci_td_length - ((ULONG) td -> ux_ohci_td_be -
                                                                                    (ULONG) td -> ux_ohci_td_cbp) - 1;

                /* Check at the phase of the transfer, if this is a SETUP or DATA phases for a control endpoint, 
                   we wait for the setup phase or any phase for other types of endpoints.  */
                if ((td -> ux_ohci_td_status & UX_OHCI_TD_SETUP_PHASE) || (td -> ux_ohci_td_status & UX_OHCI_TD_DATA_PHASE))
                    break;

                /* We need to reset the error bit in the ED.  */
                _ux_hcd_ohci_endpoint_error_clear(hcd_ohci, endpoint);

                /* Either this is a non control endpoint or it is the status phase and we are done */
                transfer_request -> ux_transfer_request_completion_code =  UX_SUCCESS;
                _ux_hcd_ohci_next_td_clean(td);
                if (transfer_request -> ux_transfer_request_completion_function != UX_NULL)
                    transfer_request -> ux_transfer_request_completion_function(transfer_request);
                _ux_host_semaphore_put(&transfer_request -> ux_transfer_request_semaphore);

                break;

          
            case UX_OHCI_ERROR_STALL:

                /* A stall condition happens when the device refuses the requested command or when a 
                   parameter in the command is wrong. We retire the transfer_request and mark the error.  */
                transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_STALLED;
                _ux_hcd_ohci_next_td_clean(td);
                if (transfer_request -> ux_transfer_request_completion_function != UX_NULL)
                    transfer_request -> ux_transfer_request_completion_function(transfer_request);
                _ux_host_semaphore_put(&transfer_request -> ux_transfer_request_semaphore);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_STALLED, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

                /* We need to reset the error bit in the ED.  */
                _ux_hcd_ohci_endpoint_reset(hcd_ohci, endpoint);
                break;


            case UX_OHCI_ERROR_DEVICE_NOT_RESPONDING:

                /* A stall condition happens when the device does not respond to the request. This mostly 
                   happens at the first GET_DESCRIPTOR after the port is enabled. This error has to be 
                   picked up by the enumeration module to reset the port and retry the command.  */ 
                transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_NO_ANSWER;
                _ux_hcd_ohci_next_td_clean(td);
                if (transfer_request -> ux_transfer_request_completion_function != UX_NULL)
                    transfer_request -> ux_transfer_request_completion_function(transfer_request);
                _ux_host_semaphore_put(&transfer_request -> ux_transfer_request_semaphore);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_NO_ANSWER, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

                /* We need to reset the error bit in the ED */
                _ux_hcd_ohci_endpoint_reset(hcd_ohci, endpoint);
                break;

                
            default:
            
                /* Any other errors default to this section. The command has been repeated 3 times 
                   and there is still a problem. The endpoint probably should be reset.   */
                transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_ERROR;
                _ux_hcd_ohci_next_td_clean(td);
                if (transfer_request -> ux_transfer_request_completion_function != UX_NULL)
                    transfer_request -> ux_transfer_request_completion_function(transfer_request);
                _ux_host_semaphore_put(&transfer_request -> ux_transfer_request_semaphore);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_ERROR, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

                /* We need to reset the error bit in the ED.  */
                _ux_hcd_ohci_endpoint_reset(hcd_ohci, endpoint);
                break;
            }
            break;

        
        case UX_ISOCHRONOUS_ENDPOINT:

            /* The length of the transfer is in the PSW.  */
            iso_td =  (UX_OHCI_ISO_TD *) td;
                
            switch (td_error_code)
            {

            case UX_OHCI_NO_ERROR:
                
                /* No error on the transmission of this TD. All data is accounted for. Check for the 
                   last TD in the transfer request. If last, process the transfer request. The method 
                   to calculate the length of the transaction is different between a IN and OUT 
                   transactions. For a OUT, if the PSW is 0, then all data was transmitted. For an IN 
                   the PSW indicates the number of bytes received.  */
                transaction_length =  iso_td -> ux_ohci_iso_td_offset_psw[0] & UX_OHCI_ISO_TD_OFFSET;
                
                if ((transfer_request -> ux_transfer_request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN)
                {
                 
                    transfer_request -> ux_transfer_request_actual_length +=  transaction_length;
                }
                else
                {

                    if (transaction_length == 0)
                        transfer_request -> ux_transfer_request_actual_length +=  iso_td -> ux_ohci_iso_td_length;
                    else
                        transfer_request -> ux_transfer_request_actual_length +=  transaction_length;
                }

                /* Check if the transfer request is complete or if this is an IN transaction and the length received 
                   is less than the max packet size.  */
                if ((transfer_request -> ux_transfer_request_actual_length == transfer_request -> ux_transfer_request_requested_length) ||
                   (((transfer_request -> ux_transfer_request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN) &&
                                                    (transaction_length < transfer_request -> ux_transfer_request_packet_length)))
                {

                        transfer_request -> ux_transfer_request_completion_code =  UX_SUCCESS;
                        if (transfer_request -> ux_transfer_request_completion_function != UX_NULL)
                            transfer_request -> ux_transfer_request_completion_function(transfer_request);
                        _ux_host_semaphore_put(&transfer_request -> ux_transfer_request_semaphore);
                }
                break;


            case UX_OHCI_ERROR_DATA_OVERRRUN:
                        
                /* In this case, we have missed the frame for the isoch transfer.  */
                _ux_hcd_ohci_frame_number_get(hcd_ohci, &current_frame);
                transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_MISSED_FRAME;
                if (transfer_request -> ux_transfer_request_completion_function != UX_NULL)
                    transfer_request -> ux_transfer_request_completion_function(transfer_request);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_MISSED_FRAME, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

                _ux_host_semaphore_put(&transfer_request -> ux_transfer_request_semaphore);
                break;

                
            default:

                /* Some other error happened, in isoch transfer, there is not much we can do.  */
                transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_ERROR;
                if (transfer_request -> ux_transfer_request_completion_function != UX_NULL)
                    transfer_request -> ux_transfer_request_completion_function(transfer_request);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_ERROR, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

                _ux_host_semaphore_put(&transfer_request -> ux_transfer_request_semaphore);
                break;
            }
        }                

        /* Free the TD that was just treated.  */
        td -> ux_ohci_td_status =  UX_UNUSED;

        /* And continue the TD loop.  */
        td =  _ux_utility_virtual_address(td -> ux_ohci_td_next_td);
    }

    /* The OHCI controller is now ready to receive the next done queue. We need to 
       reawake the OHCI controller on the WDH signal.  */
    ohci_register_interrupt =   _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_INTERRUPT_ENABLE);
    ohci_register_interrupt |=  OHCI_HC_INT_WDH;
    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_INTERRUPT_ENABLE, ohci_register_interrupt);
}

