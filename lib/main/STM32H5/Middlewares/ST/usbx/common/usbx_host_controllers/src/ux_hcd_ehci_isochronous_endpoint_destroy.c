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
/*    _ux_hcd_ehci_isochronous_endpoint_destroy           PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will destroy an isochronous endpoint.                 */
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
/*    None                                                                */
/*    _ux_host_mutex_on                     Get mutex                     */
/*    _ux_host_mutex_off                    Put mutex                     */
/*    _ux_hcd_ehci_periodic_descriptor_link Link/unlink descriptor        */
/*    _ux_utility_memory_free               Free memory                   */
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
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ehci_isochronous_endpoint_destroy(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint)
{
#if UX_MAX_ISO_TD == 0

    UX_PARAMETER_NOT_USED(hcd_ehci);
    UX_PARAMETER_NOT_USED(endpoint);

    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_FUNCTION_NOT_SUPPORTED);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* Not supported, return error.  */
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_EHCI_HSISO_ED                *ed;
UX_EHCI_PERIODIC_LINK_POINTER   ed_td;
UX_EHCI_PERIODIC_LINK_POINTER   lp;
ULONG                           max_packet_size;
UINT                            frindex;
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
ULONG                           split_count;
ULONG                           last_frindex;
ULONG                           last_size;
#endif


    /* Get ED iTD/siTD.  */
    ed_td.void_ptr = endpoint -> ux_endpoint_ed;

    /* Access to periodic list.  */
    _ux_host_mutex_on(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);

    /* Check active iTD/siTD and unlink it.  */
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
    if (endpoint -> ux_endpoint_device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
    {
        /* TBD.  */
    }
    else
#endif
    {

        /* Get ED.  */
        ed = ed_td.itd_ptr -> ux_ehci_hsiso_td_ed;

        /* Unlink from periodic list.  */
        if (ed_td.itd_ptr -> ux_ehci_hsiso_td_previous_lp.void_ptr != UX_NULL)
        {

            /* Get next LP of last iTD.  */
            lp = ed -> ux_ehci_hsiso_ed_fr_td[ed -> ux_ehci_hsiso_ed_nb_tds - 1]
                        -> ux_ehci_hsiso_td_next_lp;
            _ux_hcd_ehci_periodic_descriptor_link(
                    ed_td.itd_ptr -> ux_ehci_hsiso_td_previous_lp.void_ptr,
                    UX_NULL, UX_NULL,
                    lp.void_ptr);
        }
    }

    /* Unlink iTD/siTD from the scan list.  */
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
    if (endpoint -> ux_endpoint_device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
    {

        /* Get list head.  */
        lp.sitd_ptr = hcd_ehci -> ux_hcd_ehci_fsiso_scan_list;

        /* Check if unlink from head.  */
        if (lp.sitd_ptr == ed_td.sitd_ptr)

            /* Unlink from list head.  */
            hcd_ehci -> ux_hcd_ehci_fsiso_scan_list = ed_td.sitd_ptr -> ux_ehci_fsiso_td_next_scan_td;
        else
        {

            /* Scan items in list.  */
            while(lp.sitd_ptr)
            {

                /* Not the previous item, just try next.  */
                if (lp.sitd_ptr -> ux_ehci_fsiso_td_next_scan_td != ed_td.sitd_ptr)
                {
                    lp.sitd_ptr = lp.sitd_ptr -> ux_ehci_fsiso_td_next_scan_td;
                    continue;
                }

                /* Found it, unlink and break.  */
                lp.sitd_ptr -> ux_ehci_fsiso_td_next_scan_td = ed_td.sitd_ptr -> ux_ehci_fsiso_td_next_scan_td;
                break;
            }
        }
    }
    else
#endif
    {

        /* Get head.  */
        lp.itd_ptr = hcd_ehci -> ux_hcd_ehci_hsiso_scan_list;

        /* Check if iTD is in head.  */
        if (lp.itd_ptr == ed_td.itd_ptr)

            /* Unlink from list head.  */
            hcd_ehci -> ux_hcd_ehci_hsiso_scan_list = lp.itd_ptr -> ux_ehci_hsiso_td_next_scan_td;
        else
        {

            /* Not in head, there must be previous.  */

            /* Link it's previous to next.  */
            lp.itd_ptr -> ux_ehci_hsiso_td_previous_scan_td -> ux_ehci_hsiso_td_next_scan_td =
                                lp.itd_ptr -> ux_ehci_hsiso_td_next_scan_td;

            /* Link it's next to previous.  */
            if (lp.itd_ptr -> ux_ehci_hsiso_td_next_scan_td)
                lp.itd_ptr -> ux_ehci_hsiso_td_next_scan_td -> ux_ehci_hsiso_td_previous_scan_td =
                                lp.itd_ptr -> ux_ehci_hsiso_td_previous_scan_td;
        }
    }

    /* Update micro-frame loads of anchor.  */

    /* Calculate max packet size.  */
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
    if (endpoint -> ux_endpoint_device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
    {
        max_packet_size = endpoint -> ux_endpoint_descriptor.wMaxPacketSize & UX_MAX_PACKET_SIZE_MASK;

        /* Update according to siTD S-Mask.  */
        max_packet_size = endpoint -> ux_endpoint_descriptor.wMaxPacketSize & UX_MAX_NUMBER_OF_TRANSACTIONS_MASK;

        /* Update according to mask.  */

        for (frindex = 0; frindex < 8; frindex ++)
        {

            /* Update start split related.  */
            if (ed_td.sitd_ptr -> ux_ehci_fsiso_td_cap1 & (UX_EHCI_SMASK_0 << frindex))
            {

                /* Update start split count.  */
                ed_td.sitd_ptr -> ux_ehci_fsiso_td_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_ssplit_count[frindex] --;

                /* Update load for OUT.  */
                if ((endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == 0)
                {
                    split_count = (max_packet_size + 187) / 188;
                    last_frindex = ((ed_td.sitd_ptr -> ux_ehci_fsiso_td_frindex + split_count - 1) & 7);
                    last_size = max_packet_size % 188;
                    if (last_size == 0 &&
                        frindex == last_frindex)
                        ed_td.sitd_ptr -> ux_ehci_fsiso_td_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[frindex] = (USHORT)
                                (ed_td.sitd_ptr -> ux_ehci_fsiso_td_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[frindex] - last_size);
                    else
                        ed_td.sitd_ptr -> ux_ehci_fsiso_td_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[frindex] = (USHORT)
                                (ed_td.sitd_ptr -> ux_ehci_fsiso_td_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[frindex] - 188u);
                }

            }

            /* Update complete split related (IN only).  */
            if (ed_td.sitd_ptr -> ux_ehci_fsiso_td_cap1 & (UX_EHCI_CMASK_0 << frindex))
                ed_td.sitd_ptr -> ux_ehci_fsiso_td_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[frindex] = (USHORT)
                        (ed_td.sitd_ptr -> ux_ehci_fsiso_td_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[frindex] - 188u);
        }
    }
    else
#endif
    {

        /* Get max transfer size.  */
        max_packet_size = ed_td.itd_ptr -> ux_ehci_hsiso_td_max_trans_size;

        /* Update according to mask.  */
        for (frindex = ed -> ux_ehci_hsiso_ed_frindex;
            frindex < 8;
            frindex += ed -> ux_ehci_hsiso_ed_frinterval)
        {

            /* Decrement the microframes scheduled.  */
            ed -> ux_ehci_hsiso_ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[frindex] = (USHORT)(ed -> ux_ehci_hsiso_ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[frindex] - max_packet_size);
        }
    }

    /* Release periodic list.  */
    _ux_host_mutex_off(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);

    /* Now we can safely make the iTD/siTDs free.  */
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
    if (endpoint -> ux_endpoint_device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
    {
        ed_td.sitd_ptr -> ux_ehci_fsiso_td_status = UX_UNUSED;
    }
    else
#endif
    {
        for (frindex = 0; frindex < ed -> ux_ehci_hsiso_ed_nb_tds; frindex ++)
            ed -> ux_ehci_hsiso_ed_fr_td[frindex] -> ux_ehci_hsiso_td_status = UX_UNUSED;
        _ux_utility_memory_free(ed);
    }

    return(UX_SUCCESS);
#endif
}
