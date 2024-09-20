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
/*    _ux_hcd_ehci_asynchronous_endpoint_destroy          PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will destroy an asynchronous endpoint. The control    */
/*    and bulk endpoints fall into this category.                         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ehci                              Pointer to EHCI controller    */ 
/*    endpoint                              Pointer to endpoint           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ehci_door_bell_wait           Wait for door bell            */ 
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
UINT  _ux_hcd_ehci_asynchronous_endpoint_destroy(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint)
{

UX_EHCI_ED              *ed;
UX_EHCI_ED              *previous_ed;
UX_EHCI_ED              *next_ed;
UX_EHCI_LINK_POINTER    queue_head;
    

    /* From the endpoint container fetch the EHCI ED descriptor.  */
    ed =  (UX_EHCI_ED *) endpoint -> ux_endpoint_ed;

    /* Get the previous ED in the list for this ED.  */
    previous_ed =  ed -> ux_ehci_ed_previous_ed;

    /* Get the next ED in the list for this ED.  */
    next_ed =  ed -> ux_ehci_ed_next_ed;
    
    /* Point the previous ED to the ED after the ED to be removed.  */
    previous_ed -> ux_ehci_ed_next_ed =  next_ed;
    
    /* Point the next ED previous pointer to the previous ED.  */
    next_ed -> ux_ehci_ed_previous_ed =  previous_ed;
    
    /* Now remove the physical link.  */
    queue_head.void_ptr = _ux_utility_physical_address(next_ed);
    queue_head.value |=  UX_EHCI_QH_TYP_QH;
    previous_ed -> ux_ehci_ed_queue_head = queue_head.ed_ptr;

    /* If this ED was the last ED, we need to update the HCD last ED value.  */
    if (hcd_ehci -> ux_hcd_ehci_asynch_last_list == ed)
        hcd_ehci -> ux_hcd_ehci_asynch_last_list =  previous_ed;

    /* Arm the doorbell and wait for its completion.  */
    _ux_hcd_ehci_door_bell_wait(hcd_ehci);
        
    /* Now we can safely make the ED free.  */
    ed -> ux_ehci_ed_status =  UX_UNUSED;

    /* Return successful completion.  */
    return(UX_SUCCESS);        
}

