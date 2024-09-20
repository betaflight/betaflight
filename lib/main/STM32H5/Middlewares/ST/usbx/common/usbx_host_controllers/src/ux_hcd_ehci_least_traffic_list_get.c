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
/*    _ux_hcd_ehci_least_traffic_list_get                 PORTABLE C      */
/*                                                           6.1.6        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function return a pointer to the first ED in the periodic tree */
/*    that has the least traffic registered.                              */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_ehci                              Pointer to EHCI controller    */
/*    microframe_load                       Pointer to an array for 8     */
/*                                          micro-frame loads             */
/*    microframe_ssplit_count               Pointer to an array for 8     */
/*                                          micro-frame start split count */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    UX_EHCI_ED *                          Pointer to ED                 */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
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
/*                                            fixed compile warnings,     */
/*                                            resulting in version 6.1.2  */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile issues with   */
/*                                            some macro options,         */
/*                                            resulting in version 6.1.6  */
/*                                                                        */
/**************************************************************************/
UX_EHCI_ED  *_ux_hcd_ehci_least_traffic_list_get(UX_HCD_EHCI *hcd_ehci,
    ULONG microframe_load[8], ULONG microframe_ssplit_count[8])
{

UX_EHCI_ED                      *min_bandwidth_ed;
UX_EHCI_ED                      *ed;
UX_EHCI_PERIODIC_LINK_POINTER   anchor;
UINT                            list_index;
UINT                            frindex;
ULONG                           min_bandwidth_used;
ULONG                           bandwidth_used;


#if !defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
    UX_PARAMETER_NOT_USED(microframe_ssplit_count);
#endif

    /* Set the min bandwidth used to a arbitrary maximum value.  */
    min_bandwidth_used =  0xffffffff;

    /* The first ED is the list candidate for now.  */
    min_bandwidth_ed =  *hcd_ehci -> ux_hcd_ehci_frame_list;

    /* All list will be scanned.  */
    for (list_index = 0; list_index < 32; list_index++)
    {

        /* Reset the bandwidth for this list.  */
        bandwidth_used =  0;

        /* Get the ED of the beginning of the list we parse now.  */
        /* Obtain the ED address only.  */
        /* Obtain the virtual address from the element.  */
        anchor.ed_ptr =  *(hcd_ehci -> ux_hcd_ehci_frame_list + list_index);
        anchor.value &= UX_EHCI_LINK_ADDRESS_MASK;
        anchor.void_ptr = _ux_utility_virtual_address(anchor.void_ptr);

        /* Summary micro-frames loads of anchors.  */
        /* Reset microframe load table.  */
        for (frindex = 0; frindex < 8; frindex ++)
        {
            microframe_load[frindex] = 0;
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
            microframe_ssplit_count[frindex] = 0;
#endif
        }

        /* Scan static anchors in the list.  */
        ed = anchor.ed_ptr;
        while(ed -> REF_AS.ANCHOR.ux_ehci_ed_next_anchor != UX_NULL)
        {
            for (frindex = 0; frindex < 8; frindex ++)
            {
                microframe_load[frindex] += ed -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[frindex];
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
                microframe_ssplit_count[frindex] += ed -> REF_AS.ANCHOR.ux_ehci_ed_microframe_ssplit_count[frindex];
#endif
            }

            /* Next static anchor.  */
            ed = ed -> REF_AS.ANCHOR.ux_ehci_ed_next_anchor;
        }

        /* Summarize bandwidth from micro-frames.  */
        for (frindex = 0; frindex < 8; frindex ++)
            bandwidth_used += microframe_load[frindex];

        /* We have processed a list, check the bandwidth used by this list. If this bandwidth is
           the minimum, we memorize the ED.  */
        if (bandwidth_used < min_bandwidth_used)
        {

            /* We have found a better list with a lower used bandwidth,
               memorize the bandwidth for this list.  */
            min_bandwidth_used =  bandwidth_used;

            /* Memorize the ED for this list.  */
            min_bandwidth_ed =  anchor.ed_ptr;

            /* To optimize time, if bandwidth is 0, we just break the loop.  */
            if (min_bandwidth_used == 0)
                break;
        }
    }

    /* Return the ED list with the lowest bandwidth.  */
    return(min_bandwidth_ed);
}
