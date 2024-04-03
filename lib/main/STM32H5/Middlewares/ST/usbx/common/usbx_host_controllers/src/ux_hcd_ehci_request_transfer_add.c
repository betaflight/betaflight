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
/*    _ux_hcd_ehci_request_transfer_add                   PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function adds a component of a transfer to an existing ED.    */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ehci                              Pointer to EHCI controller    */ 
/*    ed                                    Pointer to the ED             */
/*    phase                                 Phase (for control transfers) */
/*    pid                                   PID to be used with this      */ 
/*                                            request (SETUP,IN,OUT)      */
/*    toggle                                Toggle value 0 or 1           */
/*    buffer_address                        Buffer address for transfer   */
/*    buffer_length                         Buffer length                 */
/*    transfer_request                      Pointer to transfer request   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ehci_regular_td_obtain        Obtain regular TD             */ 
/*    _ux_utility_physical_address          Get physical address          */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ehci_request_transfer_add(UX_HCD_EHCI *hcd_ehci, UX_EHCI_ED *ed, ULONG phase, ULONG pid,
                                    ULONG toggle, UCHAR * buffer_address, ULONG buffer_length, UX_TRANSFER *transfer_request)
{

UX_EHCI_TD              *last_td;
UX_EHCI_TD              *td;
UX_EHCI_LINK_POINTER    lp;
UX_EHCI_POINTER         bp;
    

    /* Obtain a TD for this transaction.  */
    td =  _ux_hcd_ehci_regular_td_obtain(hcd_ehci);
    if (td == UX_NULL)
        return(UX_NO_TD_AVAILABLE);

    /* Store the transfer request associated with this TD.  */
    td -> ux_ehci_td_transfer_request =  transfer_request;

    /* Store the ED associated with this TD.  */
    td -> ux_ehci_td_ed =  ed;

    /* Mark the TD with the phase.  */
    td -> ux_ehci_td_phase |=  phase;
    
    /* Set the PID in the control DWORD of the TD.  */
    td -> ux_ehci_td_control =  pid;
    
    /* Set the buffer address if there is a data payload.  */
    bp.void_ptr = _ux_utility_physical_address(buffer_address);
    td -> ux_ehci_td_bp0 = bp.void_ptr; /* with offset.  */

    /* Fill in the next pages addresses if required.  */
    bp.value &=  UX_EHCI_PAGE_ALIGN;
    td -> ux_ehci_td_bp1 =  bp.u8_ptr + UX_EHCI_PAGE_SIZE;
    td -> ux_ehci_td_bp2 =  bp.u8_ptr + UX_EHCI_PAGE_SIZE * 2;
    td -> ux_ehci_td_bp3 =  bp.u8_ptr + UX_EHCI_PAGE_SIZE * 3;
    td -> ux_ehci_td_bp4 =  bp.u8_ptr + UX_EHCI_PAGE_SIZE * 4;
    
    /* Set the length of the data transfer. We keep its original value.  */
    td -> ux_ehci_td_control |=  buffer_length << UX_EHCI_TD_LG_LOC;
    td -> ux_ehci_td_length =    buffer_length;
   
    /* Add the completion trigger, the default error count, the active bit.  */
    td -> ux_ehci_td_control |=  UX_EHCI_TD_CERR | UX_EHCI_TD_ACTIVE;

    /* Add the toggle value. This value is only used for control transfers.  */
    td -> ux_ehci_td_control |=  toggle;
   
    /* Recall the last TD hooked to the ED. If the value is NULL, this will be the 
       first TD and should be hooked to the ED itself..  */
    last_td =  ed -> ux_ehci_ed_last_td;

    /* Do we hook this TD to the ED?  */
    if (last_td == UX_NULL)
    {

        /* The TD is hooked to the ED. We set the T bit so that the controller will
           not transfer this TD on hook up but when all the TDs have been hooked.
           We memorize this TD as the first TD as well.  */
        ed -> ux_ehci_ed_first_td =  td;
        lp.void_ptr = _ux_utility_physical_address(td);
        lp.value |= UX_EHCI_TD_T;
        ed -> ux_ehci_ed_queue_element = lp.td_ptr;
    }
    else
    {

        /* The TD is hooked to the end of the linked TDs.  */
        last_td -> ux_ehci_td_link_pointer =  _ux_utility_physical_address(td);
    }

    /* Memorize the last TD hooked.  */
    ed -> ux_ehci_ed_last_td =  td;

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

