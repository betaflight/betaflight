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
/*    _ux_hcd_ohci_request_interrupt_transfer             PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function performs an interrupt transfer request. An interrupt */ 
/*     transfer can only be as large as the MaxpacketField in the         */ 
/*     endpoint descriptor. This was verified at a higher layer and does  */
/*     not need to be reverified here.                                    */ 
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
/*    _ux_hcd_ohci_regular_td_obtain        Get regular TD                */ 
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
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ohci_request_interrupt_transfer(UX_HCD_OHCI *hcd_ohci, UX_TRANSFER *transfer_request)
{

UX_ENDPOINT     *endpoint;
UX_OHCI_ED      *ed;
UX_OHCI_TD      *data_td;
UX_OHCI_TD      *tail_td;
    

    /* Get the pointer to the Endpoint.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

    /* Now get the physical ED attached to this endpoint.  */
    ed =  endpoint -> ux_endpoint_ed;

    /* Use the TD pointer by ed -> tail for the first TD of this transfer
        and chain from this one on.  */
    data_td =  _ux_utility_virtual_address(ed -> ux_ohci_ed_tail_td);

    /* Set the direction of the transfer. In USB 1.0, the direction of the Interrupt pipe could 
       only be HOST to DEVICE. In 1.1 bidirectional interrupt endpoints can be allowed. The 
       direction was checked when the endpoint was initialized.  */
    if ((transfer_request -> ux_transfer_request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN)

        data_td -> ux_ohci_td_dw0 =  UX_OHCI_TD_IN | UX_OHCI_TD_DEFAULT_DW0;
    else

        data_td -> ux_ohci_td_dw0 =  UX_OHCI_TD_OUT | UX_OHCI_TD_DEFAULT_DW0;

    /* Store the beginning of the buffer address in the TD.  */
    data_td -> ux_ohci_td_cbp =  _ux_utility_physical_address(transfer_request -> ux_transfer_request_data_pointer);

    /* Store the end buffer address in the TD.  */
    data_td -> ux_ohci_td_be =  data_td -> ux_ohci_td_cbp + transfer_request -> ux_transfer_request_requested_length - 1;

    /* Update the length of the transfer for this TD.  */
    data_td -> ux_ohci_td_length =  transfer_request -> ux_transfer_request_requested_length;

    /* Attach the endpoint and transfer_request to the TD.  */
    data_td -> ux_ohci_td_transfer_request =  transfer_request;
    data_td -> ux_ohci_td_ed =  ed;

    /* At this stage, the Head and Tail in the ED are still the same and the OHCI controller 
       will skip this ED until we have hooked the new tail TD.  */
    tail_td =  _ux_hcd_ohci_regular_td_obtain(hcd_ohci);
    if (tail_td == UX_NULL)
        return(UX_NO_TD_AVAILABLE);

    /* Attach the tail TD to the last data TD.  */
    data_td -> ux_ohci_td_next_td =  _ux_utility_physical_address(tail_td);

    /* Store the new tail TD.  */
    ed -> ux_ohci_ed_tail_td =  _ux_utility_physical_address(tail_td);

    /* There is no need to wake up the ohci controller on this transfer
       since periodic transactions will be picked up when the interrupt
       tree is scanned.  */
    return(UX_SUCCESS);           
}

