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
/*    _ux_hcd_ohci_transfer_abort                         PORTABLE C      */ 
/*                                                           6.1.2        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function will abort transactions attached to a transfer       */ 
/*     request.                                                           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ohci                              Pointer to OHCI controller    */ 
/*    transfer_request                      Pointer to transfer request   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_delay_ms                  Delay                         */ 
/*    _ux_utility_physical_address          Get physical address          */ 
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
/*                                            fixed physical and virtual  */
/*                                            address conversion,         */
/*                                            resulting in version 6.1    */
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile warnings,     */
/*                                            resulting in version 6.1.2  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ohci_transfer_abort(UX_HCD_OHCI *hcd_ohci, UX_TRANSFER *transfer_request)
{

UX_ENDPOINT     *endpoint;
UX_OHCI_ED      *ed;
UX_OHCI_TD      *head_td;
UX_OHCI_TD      *tail_td;
ULONG           value_td;
ULONG           value_carry;


    UX_PARAMETER_NOT_USED(hcd_ohci);

    /* Get the pointer to the endpoint associated with the transfer request.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;
    
    /* From the endpoint container, get the address of the physical endpoint.  */
    ed =  (UX_OHCI_ED *) endpoint -> ux_endpoint_ed;
    
    /* Check if this physical endpoint has been initialized properly!  */
    if (ed == UX_NULL)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_ENDPOINT_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_ENDPOINT_HANDLE_UNKNOWN, endpoint, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_ENDPOINT_HANDLE_UNKNOWN);
    }

    /* The endpoint may be active. If so, set the skip bit.  */
    ed -> ux_ohci_ed_dw0 |=  UX_OHCI_ED_SKIP;
    
    /* Wait for the controller to finish the current frame processing.  */
    _ux_utility_delay_ms(1);

    /* Ensure that the potential Carry bit is maintained in the head ED.  */
    value_carry =  (ULONG)(ed -> ux_ohci_ed_head_td) & UX_OHCI_ED_TOGGLE_CARRY;

    /* Ensure that the potential Halt bit is removed in the head ED.  */
    value_td =  (ULONG) _ux_utility_virtual_address(ed -> ux_ohci_ed_head_td) & UX_OHCI_ED_MASK_TD;
    head_td =   (UX_OHCI_TD *)value_td;

    /* Remove all the tds from this ED and leave the head and tail pointing
       to the dummy TD.  */
    tail_td =  _ux_utility_virtual_address(ed -> ux_ohci_ed_tail_td);

    /* Free all tds attached to the ED */
    while (head_td != tail_td)
    {

        /* Update the head TD with the next TD.  */
        ed -> ux_ohci_ed_head_td =  head_td -> ux_ohci_td_next_td;

        /* Mark the current head TD as free.  */
        head_td -> ux_ohci_td_status =  UX_UNUSED;

        /* Now the new head TD is the next TD in the chain.  */
        head_td =  _ux_utility_virtual_address(ed -> ux_ohci_ed_head_td);
    }

    /* Restore the value carry for next transfers.  */       
    value_td =   (ULONG) ed -> ux_ohci_ed_head_td;
    value_td |=  value_carry;
    ed -> ux_ohci_ed_head_td =  (UX_OHCI_TD *) value_td;

    /* Remove the reset bit in the ED.  */
    ed -> ux_ohci_ed_dw0 &=  ~UX_OHCI_ED_SKIP;

    /* Return successful completion.  */
    return(UX_SUCCESS);         
}

