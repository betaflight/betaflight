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
/*    _ux_hcd_ehci_periodic_descriptor_link               PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function link/unlink the iTD/siTD/QH descriptor in periodic    */
/*    lists for Host Controller (HC) to scan for transfers in the         */
/*    (micro-)frames.                                                     */
/*                                                                        */
/*    If link_desc is not NULL:                                           */
/*      prev_desc -> physical_next = link_lp                              */
/*      link_desc -> physical_next = next_desc                            */
/*      next_desc -> virtual_previous = link_desc                         */
/*    Resulting sequence prev - link_lp ... link_desc - next_desc.        */
/*                                                                        */
/*    If link_desc is not NULL:                                           */
/*      prev_desc -> physical_next = link_lp                              */
/*      next_desc -> virtual_previous = link_desc                         */
/*    Resulting unlink of things between prev_desc and next_desc.         */
/*                                                                        */
/*    Note previous LP for linking item is not updated by this function.  */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    prev                                  Link Pointer to previous item */
/*                                          (virtual memory address)      */
/*    prev_next                             Physical link pointer data    */
/*                                          including address, Typ and T. */
/*                                          If it's NULL the item between */
/*                                          previous and next is unlinked */
/*    next_prev                             Link Pointer to item to link  */
/*                                          (virtual memory address)      */
/*    next                                  Link pointer to next item     */
/*                                          (physical address, Typ & T)   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
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
void _ux_hcd_ehci_periodic_descriptor_link(
    VOID* prev,
    VOID* prev_next,      /* for prev -> next_lp.  */
    VOID* next_prev,      /* for next -> prev_lp (virtual).   */
    VOID* next_desc)
{

UX_EHCI_PERIODIC_LINK_POINTER     virtual_lp;
UX_EHCI_PERIODIC_LINK_POINTER     prev_virtual_lp;
UX_EHCI_PERIODIC_LINK_POINTER     link_virtual_lp;
UX_EHCI_PERIODIC_LINK_POINTER     link_physical_lp;
UX_EHCI_PERIODIC_LINK_POINTER     next_physical_lp;


    /* Pointers.  */
    prev_virtual_lp.void_ptr = prev;
    link_physical_lp.void_ptr = prev_next;
    link_virtual_lp.void_ptr = next_prev;
    next_physical_lp.void_ptr = next_desc;

    /* Check link/unlink.  */
    if (prev_next && next_prev)
    {

        /* Link, fill next LP for linked item first.  */
        link_virtual_lp.itd_ptr -> ux_ehci_hsiso_td_next_lp = next_physical_lp;
    }
    else
    {

        /* Unlink, use info from previous and next item.  */
        link_virtual_lp = prev_virtual_lp;
        link_physical_lp = next_physical_lp;
    }

    /* Update LP of previous item for HC to access.  */
    prev_virtual_lp.itd_ptr -> ux_ehci_hsiso_td_next_lp = link_physical_lp;

    /* If next item is valid, update its previous LP.  */
    if ((next_physical_lp.value & (UX_EHCI_LINK_ADDRESS_MASK | UX_EHCI_T)) != UX_EHCI_T)
    {

        /* Get virtual address of next item.  */
        virtual_lp.value = next_physical_lp.value & UX_EHCI_LINK_ADDRESS_MASK;
        virtual_lp.void_ptr = _ux_utility_virtual_address(virtual_lp.void_ptr);

        /* Update previous LP, except static anchor.  */
        switch(next_physical_lp.value & UX_EHCI_TYP_MASK)
        {
        case UX_EHCI_TYP_ITD:
            virtual_lp.itd_ptr -> ux_ehci_hsiso_td_previous_lp = link_virtual_lp;
            break;
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
        case UX_EHCI_TYP_SITD:
            virtual_lp.sitd_ptr -> ux_ehci_fsiso_td_previous_lp = link_virtual_lp;
            break;
#endif
        default: /* QH.  */
            if ((virtual_lp.ed_ptr -> ux_ehci_ed_status & UX_EHCI_QH_STATIC) == 0)
                virtual_lp.ed_ptr -> ux_ehci_ed_previous_ed = link_virtual_lp.ed_ptr;
            break;
        }
    }
}

/*
anchor, next + head, tail ==> anchor, head ... tail, next
    head -> prev = anchor  (Vir)
    anchor -> next = head  (Phy)
    tail -> next = next    (Phy)
    next -> prev = tail    (Vir)

anchor, head ... tail, next ==> anchor, next
    anchor -> next = next   (Phy)
    next -> prev = anchor   (Vir)

*/
