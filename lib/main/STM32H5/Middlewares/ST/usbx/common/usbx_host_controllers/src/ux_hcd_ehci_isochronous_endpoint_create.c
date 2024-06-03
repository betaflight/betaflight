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
/*    _ux_hcd_ehci_isochronous_endpoint_create            PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will create an isochronous endpoint.                  */
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
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_free               Free memory                   */
/*    _ux_hcd_ehci_hsisochronous_td_obtain  Obtain a TD                   */
/*    _ux_hcd_ehci_least_traffic_list_get   Get least traffic list        */
/*    _ux_hcd_ehci_poll_rate_entry_get      Get anchor for poll rate      */
/*    _ux_utility_physical_address          Get physical address          */
/*    _ux_host_mutex_on                     Get mutex                     */
/*    _ux_host_mutex_off                    Put mutex                     */
/*    _ux_hcd_ehci_periodic_descriptor_link Link/unlink descriptor        */
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
/*                                            filled max transfer length, */
/*                                            resulting in version 6.1.6  */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ehci_isochronous_endpoint_create(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint)
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

UX_DEVICE                       *device;
UX_EHCI_HSISO_ED                *ed;
UX_EHCI_PERIODIC_LINK_POINTER   itd;
UX_EHCI_ED                      *ed_list;
UX_EHCI_ED                      *ed_anchor;
UX_EHCI_PERIODIC_LINK_POINTER   lp;
UX_EHCI_POINTER                 bp;
UCHAR                           interval;
UCHAR                           interval_shift;
UINT                            poll_depth;
ULONG                           microframe_load[8];
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
ULONG                           microframe_ssplit_count[8];
ULONG                           mask;
UINT                            split_count;
ULONG                           split_last_size;
#else
#define                         microframe_ssplit_count     UX_NULL
#endif
ULONG                           microframe_i;
ULONG                           endpt;
ULONG                           device_address;
ULONG                           max_packet_size;
ULONG                           max_trans_size;
ULONG                           mult;
ULONG                           io;
UINT                            i;
UINT                            status;


    /* Get the pointer to the device.  */
    device =  endpoint -> ux_endpoint_device;

    /* Get the interval value from endpoint descriptor.  */
    interval = (UCHAR)endpoint -> ux_endpoint_descriptor.bInterval;

    /* For ISO, interval 1 ~ 16, means 2^(n-1).  */
    if (interval == 0)
        interval = 1;
    if (interval > 16)
        interval = 16;

    /* Interval shift is base 0.  */
    interval_shift = (UCHAR)(interval - 1);

    /* Keep interval as number of micro-frames.  */
    interval = (UCHAR)(1u << interval_shift);

    /* Get max packet size.  */
    max_packet_size = endpoint -> ux_endpoint_descriptor.wMaxPacketSize & UX_MAX_PACKET_SIZE_MASK;

    /* Get number transactions per micro-frame.  */
    mult = endpoint -> ux_endpoint_descriptor.wMaxPacketSize & UX_MAX_NUMBER_OF_TRANSACTIONS_MASK;
    mult >>= UX_MAX_NUMBER_OF_TRANSACTIONS_SHIFT;
    if (mult < 3)
        mult ++;

    /* Get max transfer size.  */
    max_trans_size = max_packet_size * mult;

    /* We need to take into account the nature of the HCD to define the max size
       of any transfer in the transfer request.  */
    endpoint -> ux_endpoint_transfer_request.ux_transfer_request_maximum_length = max_trans_size;

    /* Get the Endpt, Device Address, I/O, Maximum Packet Size, Mult.  */
    endpt = (endpoint -> ux_endpoint_descriptor.bEndpointAddress << UX_EHCI_HSISO_ENDPT_SHIFT) & UX_EHCI_HSISO_ENDPT_MASK;
    device_address = device -> ux_device_address & UX_EHCI_HSISO_DEVICE_ADDRESS_MASK;
    io = (endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) ? UX_EHCI_HSISO_DIRECTION_IN : UX_EHCI_HSISO_DIRECTION_OUT;

    /* Only high speed transfer supported without split transfer.  */
    if (device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
    {
#if !defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
        return(UX_FUNCTION_NOT_SUPPORTED);
#else

        /* 1 ~ N siTDs ... */
        /* OUT: only start-splits, no complete splits.  */
        /* IN : at most one start-split and one to N complete-splits.  */

        /* TBD.  */
#endif
    }
    else
    {

        /* Allocate memory for ED.  */
        ed = (UX_EHCI_HSISO_ED *)_ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_EHCI_HSISO_ED));
        if (ed == UX_NULL)
            return(UX_MEMORY_INSUFFICIENT);

        /* Obtain iTDs for this new endpoint.
        ** For noncontinuous request address and simplify calculation, allocate
        ** one iTD for two microframes.
        ** - interval   1 (0): 8 micro-frames, 4 iTD
        ** - interval   2 (1): 4 micro-frames, 2 iTD
        ** - interval   4 (2): 2 micro-frames, 1 iTD
        ** - interval >=8 (3): 1 micro-frame , 1 iTD
        ** Two micro-frames in iTD uses BP[3,4] and BP[5,6] to avoid merging of
        ** page buffer and iTD settings.
        */

        /* Get number of iTDs should be allocated.  */
        if (interval > 2)
            ed -> ux_ehci_hsiso_ed_nb_tds = 1;
        else
            ed -> ux_ehci_hsiso_ed_nb_tds = (UCHAR)(4u >> interval_shift);

        /* Obtain iTDs.  */
        status = UX_SUCCESS;
        for (i = 0; i < ed -> ux_ehci_hsiso_ed_nb_tds; i ++)
        {

            /* Get a new free iTD.  */
            itd.itd_ptr = _ux_hcd_ehci_hsisochronous_td_obtain(hcd_ehci);
            if (itd.itd_ptr == UX_NULL)
            {
                status = UX_NO_TD_AVAILABLE;
                break;
            }

            /* Link to ED.  */
            itd.itd_ptr -> ux_ehci_hsiso_td_ed = ed;

            /* Save max transfer size.  */
            itd.itd_ptr -> ux_ehci_hsiso_td_max_trans_size = (USHORT)max_trans_size;

            /* Save the iTD for the micro-frame(s).  */
            ed -> ux_ehci_hsiso_ed_fr_td[i] = itd.itd_ptr;
        }

        /* If there is error, free allocated resources.  */
        if (status != UX_SUCCESS)
        {
            for (i = 0; i < ed -> ux_ehci_hsiso_ed_nb_tds; i ++)
                ed -> ux_ehci_hsiso_ed_fr_td[i] -> ux_ehci_hsiso_td_status = UX_UNUSED;
            _ux_utility_memory_free(ed);
        }

        /* Save information not related to periodic things.  */

        /* Save endpoint.  */
        ed -> ux_ehci_hsiso_ed_endpoint = endpoint;

        /* Save interval.  */
        ed -> ux_ehci_hsiso_ed_frinterval = interval;
        ed -> ux_ehci_hsiso_ed_frinterval_shift = interval_shift;

        /* Disable iTDs for now.  */
        ed -> ux_ehci_hsiso_ed_frstart = 0xFF;
    }

    /* Attach the first iTD as the endpoint container.  */
    endpoint -> ux_endpoint_ed = ed -> ux_ehci_hsiso_ed_fr_td[0];

    /* Match the interval for the endpoint to a EHCI list.
       We match anything that is > 32ms to the 32ms interval layer.
       The 32ms list is layer 0, 16ms list is 1 ... the 1ms list is depth 5.  */

    /* Match > 32ms to 32ms list.  */
    /* Poll depth deeper, interval smaller.  */
    if (interval < 4)
        poll_depth = 5;
    else if (interval > 8)
        poll_depth = 0;
    else
        poll_depth = (UINT)(8u - interval);

    /* Keep only interval < 1ms for micro-frame calculation.  */
    interval_shift &= 0x3;
    interval &= 0x7;

    /* Fill the iTDs/siTDs contents that are not related to periodic list.
       Initialize the fields to be ready for ZLPs if OUT.
       But a zero buffer to underrun IN? */
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
    if (device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
    {

        /* TBD.  */
    }
    else
#endif
    {

        /* Prepare things not related to periodic things.  */

        for (i = 0; i < ed -> ux_ehci_hsiso_ed_nb_tds; i ++)
        {

            /* Get iTD.  */
            itd.itd_ptr = ed -> ux_ehci_hsiso_ed_fr_td[i];

            /* Build next link pointer, if not last one.*/
            if (i < ed -> ux_ehci_hsiso_ed_nb_tds - 1u)
            {
                lp.void_ptr = _ux_utility_physical_address(ed -> ux_ehci_hsiso_ed_fr_td[i + 1]);
                itd.itd_ptr -> ux_ehci_hsiso_td_next_lp = lp;
            }

            /* Build previous pointer, if not first one.  */
            if (i > 0)
            {
                itd.itd_ptr -> ux_ehci_hsiso_td_previous_lp.itd_ptr =
                                        ed -> ux_ehci_hsiso_ed_fr_td[i - 1];
            }

            /* Save Device Address and Endpt @ BP0.  */
            bp.value = device_address | endpt;
            itd.itd_ptr -> ux_ehci_hsiso_td_bp[0] = bp.void_ptr;

            /* Save I/O and max packet size @ BP1.  */
            bp.value = io | max_packet_size;
            itd.itd_ptr -> ux_ehci_hsiso_td_bp[1] = bp.void_ptr;

            /* Save Mult @ BP2.  */
            bp.value = mult;
            itd.itd_ptr -> ux_ehci_hsiso_td_bp[2] = bp.void_ptr;
        }

    }

    /* Lock the periodic list to update.  */
    _ux_host_mutex_on(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);

    /* Get the list index with the least traffic.  */
    ed_list = _ux_hcd_ehci_least_traffic_list_get(hcd_ehci, microframe_load, microframe_ssplit_count);

    /* Now we need to scan the list of EDs from the lowest load entry until we reach the
       appropriate interval node. The depth index is the interval EHCI value and the
       1st entry is pointed by the ED list entry.  */
    ed_anchor = _ux_hcd_ehci_poll_rate_entry_get(hcd_ehci, ed_list, poll_depth);

    /* Calculate packet size with num transactions.  */
    max_packet_size *= mult;

    /* Go through the transaction loads for for start
       index of micro-frame.  */
    for (microframe_i = 0; microframe_i < interval; microframe_i ++)
    {

        /* Skip if load too much.  */
        if (microframe_load[microframe_i] + max_packet_size > UX_MAX_BYTES_PER_MICROFRAME_HS)
            continue;

#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
        if (device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
        {

            /* Skip Y6 since host must not use it.  */
            if (i == 6)
                continue;

            /* Skip if start split count over 16 split.  */
            if (microframe_ssplit_count[i] >= 16)
                continue;
        }
#endif

        /* Use the load.  */
        break;
    }

    /* Sanity check, bandwidth checked before endpoint creation so there should
       not be error but we check it any way.  */
    if (microframe_i >= interval)
    {
        _ux_host_mutex_off(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);
        for (i = 0; i < ed -> ux_ehci_hsiso_ed_nb_tds; i ++)
            ed -> ux_ehci_hsiso_ed_fr_td[i] -> ux_ehci_hsiso_td_status = UX_UNUSED;
        _ux_utility_memory_free(ed);
        return(UX_NO_BANDWIDTH_AVAILABLE);
    }

    /* Now start microframe index is calculated, things related periodic list.  */
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
    if (device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
    {
        /* OUT: each microframe budgeted, 188 (or the remaining data size) data byte.
                never complete-split.
           IN : complete-split must be scheduled for each following microframe.
                L - the last microframe in which a complete-split is scheduled.
                L < Y6, schedule additional complete-splits in microframe L+1 and L+2.
                L == Y6, schedule one complete-split in microframe Y7,
                         schedule one complete-split in microframe Y0 of the next frame,
                         unless the full speed transaction was budgeted to start in microframe Y0.
                L == Y7, schedule one complete-split in microframe Y0 of the next frame,
                         unless the full speed transaction was budgeted to start in microframe Y0.
         */

        /* Save anchor pointer.  */
        itd.sitd_ptr -> ux_ehci_fsiso_td_anchor = ed_anchor;

        /* No back pointer by default.  */
        lp.value = UX_EHCI_T;

        /* OUT or IN?  */
        if (io == 0)
        {

            /* Multiple start split based on max packet size, no complete split.  */
            split_count = (max_packet_size + 187) / 188;
            split_last_size = max_packet_size % 188;

            mask = (UX_EHCI_SMASK_0 << split_count) - UX_EHCI_SMASK_0;
            mask <<= microframe_i;
            if (microframe_i + split_count > 8)
            {
                mask |= mask >> 8;
                mask &= UX_EHCI_SMASK_MASK;

                /* Need back pointer.  */
                lp = itd;
                lp.void_ptr = _ux_utility_physical_address(lp.void_ptr);
            }

            /* Save settings.  */
            itd.sitd_ptr -> ux_ehci_fsiso_td_cap1 = mask;
            itd.sitd_ptr -> ux_ehci_fsiso_td_back_pointer = lp.void_ptr;

            /* Update anchor micro-frame loads and start splits.  */
            for (i = 0; i < 8; i ++)
            {
                if ((mask & (UX_EHCI_SMASK_0 << i)) == 0)
                    continue;

                /* Add to load.  */
                if (split_last_size &&
                    i == ((microframe_i + split_count - 1) & 7))
                {
                    ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[i] = (USHORT)(ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[i] + split_last_size);
                }
                else
                    ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[i] = (USHORT)(ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[i] + 188u);

                /* Increment SSplit count.  */
                ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_ssplit_count[i] ++;
            }
        }
        else
        {

            /* Single start split.  */
            itd.sitd_ptr -> ux_ehci_fsiso_td_cap1 = UX_EHCI_SMASK_0 << microframe_i;

            /* Multiple complete split, start +2, based on max packet size.  */
            split_count = (max_packet_size + 187) / 188;

            /* Adding extra 2 at end.  */
            split_count += 2;
            mask = (UX_EHCI_CMASK_0 << split_count) - UX_EHCI_CMASK_0;
            mask <<= microframe_i + 2;
            if (microframe_i + 2 + split_count > 8)
            {
                mask |= mask >> 8;
                mask &= UX_EHCI_CMASK_MASK;

                /* Need back pointer.  */
                lp = itd;
                lp.void_ptr = _ux_utility_physical_address(lp.void_ptr);
            }

            /* If Y0 has budget, clear complete mask of it.  */
            if (microframe_i == 7)
            {
                if (mask & UX_EHCI_CMASK_0)
                {
                    mask &= ~UX_EHCI_CMASK_0;
                    split_count --;
                }
            }

            /* Save settings.  */
            itd.sitd_ptr -> ux_ehci_fsiso_td_cap1 |= mask;
            itd.sitd_ptr -> ux_ehci_fsiso_td_back_pointer = lp.void_ptr;

            /* Update anchor micro-frame loads and complete splits.  */
            for (i = 0; i < 8; i ++)
            {
                if ((mask & (UX_EHCI_CMASK_0 << i)) == 0)
                    continue;

                /* Add to load.  */
                ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[i] = (USHORT)(ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[i] + 188u);
            }

            /* Increment SSplit count.  */
            ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_ssplit_count[i] ++;
        }

    }
    else
#endif
    {

        /* Save index base of allocated micro-frame.  */
        ed -> ux_ehci_hsiso_ed_frindex = (UCHAR)microframe_i;

        /* Save anchor pointer.  */
        ed -> ux_ehci_hsiso_ed_anchor = ed_anchor;

        /* Update micro-frames.  */
        for (i = microframe_i; i < 8; i += interval)
        {

            /* Update anchor micro-frame loads.  */
            ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[i] = (USHORT)(ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[i] + max_packet_size);

            /* Initialize control with PG -> BP (3, 5).  */
            itd.itd_ptr = ed -> ux_ehci_hsiso_ed_fr_td[i >> 1];

            /* Buffer in page 3,4 or 5,6 to avoid merging settings.  */
            if (i & 1u)
                itd.itd_ptr -> ux_ehci_hsiso_td_control[i] = UX_EHCI_HSISO_IOC |
                                                (5 << UX_EHCI_HSISO_PG_SHIFT);
            else
                itd.itd_ptr -> ux_ehci_hsiso_td_control[i] = UX_EHCI_HSISO_IOC |
                                                (3 << UX_EHCI_HSISO_PG_SHIFT);
        }
    }

    /* Link iTDs to periodic list.  */

    /* Physical LP for anchor (Typ iTD, 0).  */
    lp.void_ptr = _ux_utility_physical_address(ed -> ux_ehci_hsiso_ed_fr_td[0]);

    /* Link to periodic list.  */
    ed -> ux_ehci_hsiso_ed_fr_td[0] -> ux_ehci_hsiso_td_previous_lp.ed_ptr = ed_anchor;
    _ux_hcd_ehci_periodic_descriptor_link(ed_anchor, lp.void_ptr,
        ed -> ux_ehci_hsiso_ed_fr_td[ed -> ux_ehci_hsiso_ed_nb_tds - 1],
        ed_anchor -> ux_ehci_ed_queue_head);

    /* Simply insert all iTD[0]/siTD[0] to head of scan list.  */
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
    if (device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
    {
        itd.sitd_ptr -> ux_ehci_fsiso_td_next_scan_td = hcd_ehci -> ux_hcd_ehci_fsiso_scan_list;
        hcd_ehci -> ux_hcd_ehci_fsiso_scan_list = itd.sitd_ptr;
    }
    else
#endif
    {
        itd.itd_ptr = ed -> ux_ehci_hsiso_ed_fr_td[0];
        itd.itd_ptr -> ux_ehci_hsiso_td_next_scan_td =
                                hcd_ehci -> ux_hcd_ehci_hsiso_scan_list;
        hcd_ehci -> ux_hcd_ehci_hsiso_scan_list = itd.itd_ptr;
        if (itd.itd_ptr -> ux_ehci_hsiso_td_next_scan_td)
            itd.itd_ptr -> ux_ehci_hsiso_td_next_scan_td -> ux_ehci_hsiso_td_previous_scan_td =
                                itd.itd_ptr;
    }

    /* Release the periodic table.  */
    _ux_host_mutex_off(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);

    /* Return successful completion.  */
    return(UX_SUCCESS);
#endif
}
