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
/**   EHCI Controller Driver                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_hcd_ehci.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_hcd_ehci_asynch_td_process                      PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function processes the isochronous, periodic and asynchronous  */ 
/*    lists in search for transfers that occurred in the past             */ 
/*    (micro-)frame.                                                      */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    ed                                    Pointer to ED                 */ 
/*    td                                    Pointer to TD                 */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    UX_EHCI_TD *                          Pointer to TD                 */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    (ux_transfer_request_completion_function) Completion function       */ 
/*    _ux_hcd_ehci_ed_clean                 Clean ED                      */ 
/*    _ux_host_semaphore_put                Put semaphore                 */ 
/*    _ux_utility_virtual_address           Get virtual address           */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    EHCI Controller Driver                                              */
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
/*                                                                        */
/**************************************************************************/
UX_EHCI_TD  *_ux_hcd_ehci_asynch_td_process(UX_EHCI_ED *ed, UX_EHCI_TD *td)
{

UX_TRANSFER     *transfer_request;
ULONG           td_residual_length;
UINT            td_error;
UX_EHCI_TD      *next_td;
ULONG           td_element;
ULONG           pid;


    /* If the TD is still active, we know that the transfer has not taken place yet. In this case, 
       the TDs following this one do not need to be process and we return a NULL TD.  */
    if (td -> ux_ehci_td_control & UX_EHCI_TD_ACTIVE)
        return(UX_NULL);
        
    /* We need the transfer request associated with this TD.  */
    transfer_request =  td -> ux_ehci_td_transfer_request;
            
    /* We have a non active TD, meaning the TD was processed. We should explore the error code and 
       translate it into a generic USBX code.  */
    td_error =  UX_SUCCESS;
    
    /* Check if the TD was halted due to a major error. Note that the error status may contain an error
       but unless the Halt bit is set, it is not fatal.  */
    if (td -> ux_ehci_td_control & UX_EHCI_TD_HALTED)
    {
    
        /* Default to STALL.  */
        td_error =  UX_TRANSFER_STALLED;

        /* What else could it be ? Buffer underrun/overrun ? */
        if (td -> ux_ehci_td_control & UX_EHCI_TD_DATA_BUFFER_ERROR)
            td_error =  UX_TRANSFER_ERROR;

        /* Babble ? */
        if (td -> ux_ehci_td_control & UX_EHCI_TD_BABBLE_DETECTED)
            td_error =  UX_TRANSFER_ERROR;
    
        /* Timeout, CRD, Bad PID ... ? */
        if (td -> ux_ehci_td_control & UX_EHCI_TD_TRANSACTION_ERROR)
            td_error =  UX_TRANSFER_NO_ANSWER;

    }
    
    /* If there is an error, we should terminate this transfer and clean things.  */
    if (td_error != UX_SUCCESS)
    {

        /* Update the transfer code.  */
        transfer_request -> ux_transfer_request_completion_code =  td_error;
        
        /* Clean the link.  */
        _ux_hcd_ehci_ed_clean(ed);

        /* Free the TD that was just treated.  */
        td -> ux_ehci_td_status =  UX_UNUSED;

        /* We may do a call back.  */
        if (transfer_request -> ux_transfer_request_completion_function != UX_NULL)
            transfer_request -> ux_transfer_request_completion_function(transfer_request);

        /* Notify the application for debugging purposes.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, td_error);
            
        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, td_error, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Wake up the semaphore for this request.  */
        _ux_host_semaphore_put(&transfer_request -> ux_transfer_request_semaphore);

        /* Nothing else to be processed in this queue.  */
        return(UX_NULL);
    }           

    /* Update the length of this transaction if the PID is IN or OUT.  */
    pid =  td -> ux_ehci_td_control & UX_EHCI_PID_MASK;
    td_residual_length =  0;
    if ((pid == UX_EHCI_PID_OUT) || (pid == UX_EHCI_PID_IN))
    {

        td_residual_length =  (td -> ux_ehci_td_control >> UX_EHCI_TD_LG_LOC) & UX_EHCI_TD_LG_MASK;
        transfer_request -> ux_transfer_request_actual_length +=  td -> ux_ehci_td_length - td_residual_length;        
    }

    /* We get here when there is no error on the transfer. It may be that this transfer is not the last 
       one in the link and therefore we process the length received but do not complete the transfer request.  */
    if (td -> ux_ehci_td_control & UX_EHCI_TD_IOC)
    {

        /* Check if this transaction is complete or a short packet occurred, in both case, complete the 
           transaction. In the case of a control endpoint in STATUS phase we complete regardless.  */
        if ((td_residual_length != 0) || 
            (transfer_request -> ux_transfer_request_actual_length == transfer_request -> ux_transfer_request_requested_length) || 
            (td -> ux_ehci_td_phase & UX_EHCI_TD_STATUS_PHASE))  
        {

            /* Update the transfer code.  */
            transfer_request -> ux_transfer_request_completion_code =  UX_SUCCESS;
        
            /* Clean the link.  */
            _ux_hcd_ehci_ed_clean(ed);

            /* Free the TD that was just treated.  */
            td -> ux_ehci_td_status =  UX_UNUSED;

            /* We may do a call back.  */
            if (transfer_request -> ux_transfer_request_completion_function != UX_NULL)
                transfer_request -> ux_transfer_request_completion_function(transfer_request);
    
            /* Wake up the semaphore for this request.  */
            _ux_host_semaphore_put(&transfer_request -> ux_transfer_request_semaphore);

            /* Nothing else to be processed in this queue */
            return(UX_NULL);
        }
    }

    /* Get the next TD attached to this TD.  */
    td_element =   (ULONG) td -> ux_ehci_td_link_pointer;
    td_element &=  ~UX_EHCI_TD_T;
    next_td =  _ux_utility_virtual_address((VOID *) td_element);
    
    /* Free the TD that was just treated.  */
    td -> ux_ehci_td_status =  UX_UNUSED;

    /* This TD is now the first TD.  */
    ed -> ux_ehci_ed_first_td = next_td;

    /* We get here when we have reached a non completion of a request transfer
       we need to return the next TD in the linked list.  */
    return(next_td);
}

