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
/*    _ux_hcd_ohci_periodic_tree_create                   PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function creates the periodic static tree for the interrupt   */ 
/*     and isochronous eds.                                               */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ohci                              Pointer to OHCI controller    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ohci_ed_obtain                Obtain an ED                  */ 
/*    _ux_utility_physical_address          Get physical address          */ 
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
UINT  _ux_hcd_ohci_periodic_tree_create(UX_HCD_OHCI *hcd_ohci)
{

UX_HCD_OHCI_HCCA        *ohci_hcca;
UX_OHCI_ED              *ed;
UINT                    list_index;
UINT                    list_entries;
UINT                    current_list_entry;
UX_OHCI_ED              *ed_list[32];
UX_OHCI_ED              *ed_start_list[32];
    
    
    /* We need the pointer to the HCCA. It contains the pointer to the first
       32 periodic entries.  */
    ohci_hcca =  hcd_ohci -> ux_hcd_ohci_hcca;
    
    /* Start with the 1st list - it has 32 entries.  */
    list_entries =  32;

    /* Create each list one by one starting from the 32ms list.  */
    for (list_index = 0; list_index < 6; list_index++)
    {

        for (current_list_entry = 0; current_list_entry < list_entries;current_list_entry++)
        {

            /* In each list, insert an static ED as the anchor. There should not
               be any errors when obtaining a new ED, still we do a sanity check.  */
            ed =  _ux_hcd_ohci_ed_obtain(hcd_ohci);
            if (ed == UX_NULL)
                return(UX_NO_ED_AVAILABLE);

            /* Mark this anchor ED as static by putting it as SKIPPED, the OHCI
               controller will not look into its tail and head list and will simply
               jump to the next ED.  */
            ed -> ux_ohci_ed_dw0 =  UX_OHCI_ED_SKIP;

            /* Either we hook this new ED to the start list for further processing
               or we hook it to the 2 successive entries in the previous list.  */
            if (list_index == 0)
            {

                ed_start_list[current_list_entry] =  ed;
            }
            else
            {

                ed_list[current_list_entry * 2] -> ux_ohci_ed_next_ed =        _ux_utility_physical_address(ed);
                ed_list[(current_list_entry * 2) + 1] -> ux_ohci_ed_next_ed =  _ux_utility_physical_address(ed);
            }

            /* Memorize this ED in the local list. We do this operation now, otherwise
               we would erase the previous list eds.  */
            ed_list[current_list_entry] =  ed;
        }

        /*  Shift the number of entries in the next list by 1 (i.e. divide by 2).  */
        list_entries =  list_entries >> 1;
    }

    /* The tree has been completed but the entries in the HCCA are in the wrong order.
       We need to swap each entry according to the OHCI specified entry order list 
       so that we have a fair interval frequency for each periodic ED. The primary eds 
       are fetched from the start list, translated into physical addresses and stored 
       into the HCCA.  */      
    for (current_list_entry = 0; current_list_entry < 32; current_list_entry++)
    {

        ed =  ed_start_list[_ux_system_host_hcd_periodic_tree_entries[current_list_entry]];
        ohci_hcca -> ux_hcd_ohci_hcca_ed[current_list_entry] =  _ux_utility_physical_address(ed);
    }

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

