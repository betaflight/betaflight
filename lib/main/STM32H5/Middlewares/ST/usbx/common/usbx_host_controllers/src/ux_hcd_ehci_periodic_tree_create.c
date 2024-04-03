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
/*    _ux_hcd_ehci_periodic_tree_create                   PORTABLE C      */ 
/*                                                           6.1.2        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates the periodic static tree for the interrupt    */
/*    and isochronous eds.                                                */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ehci                              Pointer to EHCI controller    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ehci_ed_obtain                Obtain an ED                  */ 
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
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile warnings,     */
/*                                            resulting in version 6.1.2  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ehci_periodic_tree_create(UX_HCD_EHCI *hcd_ehci)
{

UX_EHCI_ED                      *ed;
UX_EHCI_ED                      *previous_ed;
UINT                            list_index;
UINT                            list_entries;
UINT                            current_list_entry;
UX_EHCI_ED                      *ed_list[32];
UX_EHCI_ED                      *ed_start_list[32];
UX_EHCI_PERIODIC_LINK_POINTER   lp;
    
    /* Start with the 1st list - it has 32 entries.  */
    list_entries =  32;

    /* Create each list one by one starting from the 32ms list.  */
    for (list_index = 0; list_index < 6; list_index++)
    {

        for (current_list_entry = 0; current_list_entry < list_entries; current_list_entry++)
        {

            /* In each list, insert an static QH as the anchor. There should not
               be any errors when obtaining a new ED, still we do a sanity check.  */
            ed =  _ux_hcd_ehci_ed_obtain(hcd_ehci);
            if (ed == UX_NULL)
                return(UX_NO_ED_AVAILABLE);

            /* Mark the anchor as being a QH pointer and a terminator.  */
            ed -> ux_ehci_ed_queue_head =  (UX_EHCI_ED *) (UX_EHCI_QH_TYP_QH | UX_EHCI_QH_T);

            /* The Queue element has the terminator bit on.  */
            ed -> ux_ehci_ed_queue_element =  (UX_EHCI_TD *) UX_EHCI_TD_T;

            /* The Alternate TD has the terminator bit on.  */
            ed -> ux_ehci_ed_alternate_td =  (UX_EHCI_TD *) UX_EHCI_TD_T;

            /* This endpoint is an anchor.  */
            ed -> ux_ehci_ed_status |=  UX_EHCI_QH_STATIC;

            /* Either we hook this new ED to the start list for further processing
               or we hook it to the 2 successive entries in the previous list.  */
            if (list_index == 0)
            {

                ed_start_list[current_list_entry] =  ed;
            }
            else
            {
 
                /* We need to update the previous ED with the link to this new ED. Since
                   this is a tree structure, this operation is done twice to the 2 previous
                   eds in the previous list.  */
                lp.void_ptr =                           _ux_utility_physical_address(ed);
                lp.value |=                             UX_EHCI_QH_TYP_QH;
                previous_ed =                           ed_list[current_list_entry * 2];
                previous_ed -> ux_ehci_ed_queue_head =  lp.ed_ptr;
                previous_ed -> ux_ehci_ed_next_ed =     ed;
                previous_ed -> REF_AS.ANCHOR.ux_ehci_ed_next_anchor = ed;
                previous_ed =                           ed_list[(current_list_entry * 2) + 1];
                previous_ed -> ux_ehci_ed_queue_head =  lp.ed_ptr;
                previous_ed -> ux_ehci_ed_next_ed =     ed;
                previous_ed -> REF_AS.ANCHOR.ux_ehci_ed_next_anchor = ed;
            }

            /* Memorize this ED in the local list. We do this operation now, otherwise
               we would erase the previous list eds.  */
            ed_list[current_list_entry] =  ed;
        }

        /*  Shift the number of entries in the next list by 1 (i.e. divide by 2).  */
        list_entries =  list_entries>>1;
    }

    /* Check the value of the ehci frame list entries. If 0, it was not initialized by the controller init function.  */
    if (hcd_ehci -> ux_hcd_ehci_frame_list_size == 0)
    
        /* Value not initialized. Use default.  */
        hcd_ehci -> ux_hcd_ehci_frame_list_size = UX_EHCI_FRAME_LIST_ENTRIES;

    /* The tree has been completed but the entries in the EHCI frame list are in the wrong order.
       We need to swap each entry according to the EHCI specified entry order list so that we 
       have a fair interval frequency for each periodic ED. The primary eds are fetched from the 
       start list, translated into physical addresses and stored into the frame List.  */      
    for (current_list_entry = 0; current_list_entry < 32; current_list_entry++)
    {

        ed =  ed_start_list[_ux_system_host_hcd_periodic_tree_entries[current_list_entry]];
        *(hcd_ehci -> ux_hcd_ehci_frame_list+current_list_entry) =  (UX_EHCI_ED *) _ux_utility_physical_address(ed);
    }
 
    /* We still haven't set the type of each queue head in the list itself. Do that now. */
    for (current_list_entry = 0; current_list_entry < 32; current_list_entry++)
    {

        lp.ed_ptr = hcd_ehci -> ux_hcd_ehci_frame_list[current_list_entry];
        lp.value |= UX_EHCI_QH_TYP_QH;
        hcd_ehci -> ux_hcd_ehci_frame_list[current_list_entry] =  lp.ed_ptr;
    }

    /* Now the first 32 entries in the frame list have to be duplicated to fill the other entries in the frame list.  
       If the list is set to 32 entries, nothing is done here.  */
    for (current_list_entry = 32; current_list_entry < hcd_ehci -> ux_hcd_ehci_frame_list_size; current_list_entry++)
        hcd_ehci -> ux_hcd_ehci_frame_list[current_list_entry] =  hcd_ehci -> ux_hcd_ehci_frame_list[current_list_entry & 0x1f];

    /* Now each traffic list from entry consumes 1/32 periodic bandwidth.
     * The poll entry layers depth low to high:
     * 32ms, 16ms, 8ms, 4ms, 2ms, 1ms -- micro-frames
     */

    /* Return successful completion.  */       
    return(UX_SUCCESS);
}

