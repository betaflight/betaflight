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
/*    _ux_hcd_ohci_next_td_clean                          PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function cleans all the tds attached to a ED. The end of the  */ 
/*     TD chain is pointed by the tail TD.                                */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    td                                    Pointer to OHCI TD            */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
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
/*                                                                        */
/**************************************************************************/
VOID  _ux_hcd_ohci_next_td_clean(UX_OHCI_TD *td)
{

UX_OHCI_ED      *ed;
UX_OHCI_TD      *head_td;
UX_OHCI_TD      *tail_td;
ULONG           value_td;
ULONG           value_carry;


    /* Obtain the pointer to the ED from the TD.  */
    ed =  td -> ux_ohci_td_ed;

    /* Ensure that the potential Carry bit is maintained in the head ED.  */
    value_carry =  (ULONG)(ed -> ux_ohci_ed_head_td) & UX_OHCI_ED_TOGGLE_CARRY;

    /* Ensure that the potential Halt bit is removed in the head ED.  */
    value_td =  (ULONG) _ux_utility_virtual_address(ed -> ux_ohci_ed_head_td) & UX_OHCI_ED_MASK_TD;
    head_td =   (UX_OHCI_TD *)value_td;

    /* Remove all the tds from this ED and leave the head and tail pointing
       to the dummy TD.  */
    tail_td =  _ux_utility_virtual_address(ed -> ux_ohci_ed_tail_td);

    /* Free all tds attached to the ED.  */
    while (head_td != tail_td)
    {

        /* Mark the current head_td as free.  */
        head_td -> ux_ohci_td_status =  UX_UNUSED;

        /* Update the head TD with the next TD.  */
        ed -> ux_ohci_ed_head_td =  head_td -> ux_ohci_td_next_td;

        /* Now the new head_td is the next TD in the chain.  */
        head_td =  _ux_utility_virtual_address(ed -> ux_ohci_ed_head_td);
    }

    /* Restore the value carry for next transfers.  */       
    value_td =   (ULONG) ed -> ux_ohci_ed_head_td;
    value_td |=  value_carry;
    ed -> ux_ohci_ed_head_td =  (UX_OHCI_TD *) value_td;

    /* Return to caller.  */
    return;
}

