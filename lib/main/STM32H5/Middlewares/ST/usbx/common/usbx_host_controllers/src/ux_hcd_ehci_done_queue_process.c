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
/*    _ux_hcd_ehci_done_queue_process                     PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function process the isochronous, periodic and asynchronous    */
/*    lists in search for transfers that occurred in the past             */
/*    (micro-)frame.                                                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_ehci                              Pointer to EHCI controller    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_hcd_ehci_asynch_td_process        Process asynch TD             */
/*    _ux_hcd_ehci_hsisochronous_tds_process                              */
/*                                          Process high speed            */
/*                                          isochronous TDs               */
/*    _ux_hcd_ehci_fsisochronous_tds_process                              */
/*                                          Process full speed (split)    */
/*                                          isochronous TDs               */
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
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile warning,      */
/*                                            resulting in version 6.1.2  */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_hcd_ehci_done_queue_process(UX_HCD_EHCI *hcd_ehci)
{

UX_EHCI_TD                      *td;
UX_EHCI_PERIODIC_LINK_POINTER   ed;
UX_EHCI_ED                      *start_ed;


#if UX_MAX_ISO_TD
UX_EHCI_PERIODIC_LINK_POINTER   lp;

    /* We scan the active isochronous list first.  */
    _ux_host_mutex_on(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);
    lp.itd_ptr = hcd_ehci -> ux_hcd_ehci_hsiso_scan_list;
    while(lp.itd_ptr != UX_NULL)
    {

        /* Process the iTD, return next active TD.  */
        lp.itd_ptr = _ux_hcd_ehci_hsisochronous_tds_process(hcd_ehci, lp.itd_ptr);
    }
    _ux_host_mutex_off(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);

#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)

    /* We scan the split isochronous list then.  */
    _ux_host_mutex_on(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);
    lp.sitd_ptr = hcd_ehci -> ux_hcd_ehci_fsiso_scan_list;
    while(lp.sitd_ptr != UX_NULL)
    {

        /* Process the iTD, return next active TD.  */
        lp.sitd_ptr = _ux_hcd_ehci_fsisochronous_tds_process(hcd_ehci, lp.sitd_ptr);
    }
    _ux_host_mutex_off(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);

#endif
#endif

    /* We scan the linked interrupt list then.  */
    _ux_host_mutex_on(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);
    ed.ed_ptr = hcd_ehci -> ux_hcd_ehci_interrupt_ed_list;
    while(ed.ed_ptr != UX_NULL)
    {

        /* Retrieve the fist TD attached to this ED.  */
        td =  ed.ed_ptr -> ux_ehci_ed_first_td;

        /* Process TD until there is no next available.  */
        while (td != UX_NULL)
            td =  _ux_hcd_ehci_asynch_td_process(ed.ed_ptr, td);
        
        /* Next ED.  */
        ed.ed_ptr = ed.ed_ptr -> ux_ehci_ed_next_ed;
    }
    _ux_host_mutex_off(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);

    /* Now we can parse the asynchronous list. The head ED is always empty and
       used as an anchor only.  */
    start_ed =  hcd_ehci -> ux_hcd_ehci_asynch_head_list;

    /* Point to the next ED in the asynchronous tree.  */
    ed.ed_ptr = start_ed -> ux_ehci_ed_queue_head;
    ed.value &= UX_EHCI_LINK_ADDRESS_MASK;

    /* Obtain the virtual address from the element.  */
    ed.void_ptr = _ux_utility_virtual_address(ed.void_ptr);

    /* Now traverse this list until we have found a non anchor endpoint that
       has transfers done.  */
    while (ed.ed_ptr != start_ed)
    {

        /* Retrieve the fist TD attached to this ED.  */
        td =  ed.ed_ptr -> ux_ehci_ed_first_td;
        while (td != UX_NULL)
        {

            td =  _ux_hcd_ehci_asynch_td_process(ed.ed_ptr, td);
        }

        /* Point to the next ED in the asynchronous tree.  */
        ed.ed_ptr = ed.ed_ptr -> ux_ehci_ed_queue_head;
        ed.value &= UX_EHCI_LINK_ADDRESS_MASK;

        /* Obtain the virtual address from the element.  */
        ed.void_ptr = _ux_utility_virtual_address(ed.void_ptr);
    }
}

