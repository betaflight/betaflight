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
/*    _ux_hcd_ehci_hsisochronous_tds_process              PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processes the iTDs of an isochronous endpoint.        */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_ehci                              Pointer to HCD EHCI           */
/*    itd                                   Pointer to HSISO TD           */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    UX_EHCI_HSISO_TD                      Pointer to HSISO TD           */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    (ux_transfer_request_completion_function)                           */
/*                                          Transfer Completion function  */
/*    _ux_hcd_ehci_register_read            Read EHCI register            */
/*    _ux_host_semaphore_put                Put semaphore                 */
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved uframe handling,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UX_EHCI_HSISO_TD* _ux_hcd_ehci_hsisochronous_tds_process(
    UX_HCD_EHCI *hcd_ehci,
    UX_EHCI_HSISO_TD* itd)
{

UX_EHCI_HSISO_ED                *ed;
UX_EHCI_HSISO_TD                *next_scan_td;
ULONG                           n_fr;
ULONG                           frindex;
ULONG                           frindex1;
UX_EHCI_HSISO_TD                *fr_td;
UX_EHCI_HSISO_TD                *fr_td1;
ULONG                           control;
ULONG                           control1;
ULONG                           trans_bytes;
UX_TRANSFER                     *transfer;
UX_EHCI_POINTER                 bp;
ULONG                           pg;
ULONG                           pg_addr;
ULONG                           pg_offset;
ULONG                           frindex_now;
ULONG                           frindex_start;
USHORT                          fr_hc;
USHORT                          fr_sw;
UINT                            i;


    /* Get ED.  */
    ed = itd -> ux_ehci_hsiso_td_ed;

    /* Get next scan TD.  */
    next_scan_td = itd -> ux_ehci_hsiso_td_next_scan_td;

    /* Check if iTD is skipped.  */
    if (ed -> ux_ehci_hsiso_ed_frstart == 0xFF)
        return(next_scan_td);

    /*
    ** 1. There is requests loaded
    **    (0) Micro-frame is active (request loaded)
    **    (1) Active is cleared (HC processed)
    **    (2) Handle it
    ** 2. There is micro-frame free
    **    (0) Micro-frame is free
    **    (1) Micro-frame not overlap current FRINDEX
    **    (2) Link request
    **    (3) Build controls to active micro-frame
    */

    /* Get number of frames (8,4,2 or 1).  */
    n_fr = 8u >> ed -> ux_ehci_hsiso_ed_frinterval_shift;

    /* Process if there is requests loaded.  */
    if (ed -> ux_ehci_hsiso_ed_frload > 0)
    {

        /* Process count to target micro frames.  */
        fr_hc = (USHORT)(ed -> ux_ehci_hsiso_ed_fr_hc << ed -> ux_ehci_hsiso_ed_frinterval_shift);
        fr_hc = (USHORT)(fr_hc + ed -> ux_ehci_hsiso_ed_frstart);
        fr_hc &= 0x7u;

        /* Process done iTDs.  */
        for (i = 0;
            i < n_fr; /* 8, 4, 2 or 1.  */
            i += ed -> ux_ehci_hsiso_ed_frinterval)
        {

            /* Get frindex.  */
            frindex = fr_hc + i;
            frindex &= 7u;

            if ((ed -> ux_ehci_hsiso_ed_frload & (1u << frindex)) == 0)
            {
                break;
            }

            /* Get iTD for the micro-frame.  */
            fr_td = ed -> ux_ehci_hsiso_ed_fr_td[frindex >> 1];

            /* Get control.  */
            control = fr_td -> ux_ehci_hsiso_td_control[frindex];

            /* Check next frame state, if multiple micro-frames loaded.  */
            if (n_fr > 1)
            {

                /* Micro-frame is active, check next.  */
                if (control & UX_EHCI_HSISO_STATUS_ACTIVE)
                {

                    /* Get next frindex.  */
                    frindex1 = frindex + ed -> ux_ehci_hsiso_ed_frinterval;
                    frindex1 &= 7u;

                    /* Get next iTD for the micro-frame.  */
                    fr_td1 = ed -> ux_ehci_hsiso_ed_fr_td[frindex1 >> 1];

                    /* Get control.  */
                    control1 = fr_td1 -> ux_ehci_hsiso_td_control[frindex1];

                    /* Next micro-frame is also active, break.  */
                    if (control1 & UX_EHCI_HSISO_STATUS_ACTIVE)
                    {
                        break;
                    }
                    else
                    {

                        /* Next micro-frame is not loaded, break.  */
                        if ((ed -> ux_ehci_hsiso_ed_frload & (1u << frindex1)) == 0)
                        {
                            break;
                        }

                        /* We are here when micro-frame 1 handled before micro-frame 0,
                        ** which means we missed one request.
                        */

                        /* Disable control, discard buffer and handle request.  */
                        control &= ~(UX_EHCI_HSISO_STATUS_ACTIVE |
                                                UX_EHCI_HSISO_XACT_LENGTH_MASK);
                        fr_td -> ux_ehci_hsiso_td_control[frindex] = control;
                    }

                } /* if (control & UX_EHCI_HSISO_STATUS_ACTIVE) */
            }
            else
            {

                /* If iTD is active, break.  */
                if (control & UX_EHCI_HSISO_STATUS_ACTIVE)
                {
                    break;
                }
            }

            /* Clear load map anyway.  */
            fr_td -> ux_ehci_hsiso_td_frload = (UCHAR)(fr_td -> ux_ehci_hsiso_td_frload & ~(1u << frindex));
            ed -> ux_ehci_hsiso_ed_frload = (USHORT)(ed -> ux_ehci_hsiso_ed_frload & ~(1u << frindex));

            /* HC processed.  */
            ed -> ux_ehci_hsiso_ed_fr_hc ++;

            /* Unlink it from iTD.  */
            fr_td -> ux_ehci_hsiso_td_fr_transfer[frindex & 1u] = UX_NULL;

            /* Handle the request.  */
            transfer = ed -> ux_ehci_hsiso_ed_transfer_head;

            /* If there is no transfer linked to, just ignore it.  */
            if (transfer == UX_NULL)
                break;

            /* Convert error code to completion code.  */
            if (control & UX_EHCI_HSISO_STATUS_DATA_BUFFER_ERR)
                transfer -> ux_transfer_request_completion_code = UX_TRANSFER_BUFFER_OVERFLOW;
            else
            {
                if (control & UX_EHCI_HSISO_STATUS_MASK)
                    transfer -> ux_transfer_request_completion_code = UX_TRANSFER_ERROR;
                else
                    transfer -> ux_transfer_request_completion_code = UX_SUCCESS;
            }

            /* Get transfer bytes.  */
            trans_bytes = control & UX_EHCI_HSISO_XACT_LENGTH_MASK;
            trans_bytes >>= UX_EHCI_HSISO_XACT_LENGTH_SHIFT;

            /* Save to actual length.  */
            transfer -> ux_transfer_request_actual_length = trans_bytes;

            /* Unlink it from request list head.  */
            ed -> ux_ehci_hsiso_ed_transfer_head =
                    transfer -> ux_transfer_request_next_transfer_request;

            /* If no more requests, also set tail to NULL.  */
            if (ed -> ux_ehci_hsiso_ed_transfer_head == UX_NULL)
                ed -> ux_ehci_hsiso_ed_transfer_tail = UX_NULL;

            /* Invoke callback.  */
            if (transfer -> ux_transfer_request_completion_function)
                transfer -> ux_transfer_request_completion_function(transfer);

            /* Put semaphore.  */
            _ux_host_semaphore_put(&transfer -> ux_transfer_request_semaphore);

        } /* for (;i < n_fr;)  */
    }

    /* Build request when there is new unloaded requests.  */
    if (ed -> ux_ehci_hsiso_ed_transfer_first_new)
    {

        /* Get current FRINDEX for SW process.  */
        frindex_now = _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCOR_FRAME_INDEX);
        frindex_now &= 0x7u;

        /* If transfer starts.  */
        if (ed -> ux_ehci_hsiso_ed_frstart == 0xFE)
        {

            /* 1 micro-frame in iTD.  */
            if (ed -> ux_ehci_hsiso_ed_frinterval_shift >= 3)
            {

                /* Uses this only micro-frame index.  */
                ed -> ux_ehci_hsiso_ed_frstart = ed -> ux_ehci_hsiso_ed_frindex;
            }

            /* 8,4,2 micro-frames in each iTD.  */
            else
            {

                /* Get start index for sw to link requests.  */
                if (ed -> ux_ehci_hsiso_ed_frinterval_shift == 0)

                    /* Interval 1, just add two.  */
                    frindex_start = frindex_now + 2;
                else
                {

                    /* Interval 2, 4.  */
                    /* Scan indexes, to find an index that has 2 in front or more.  */
                    for (frindex_start = ed -> ux_ehci_hsiso_ed_frindex;
                        frindex_start < 9;
                        frindex_start += ed -> ux_ehci_hsiso_ed_frinterval)
                    {
                        if (frindex_start - frindex_now >= 2)
                            break;
                    }
                }

                /* Target to start index calculated and wrap around in one frame.  */
                ed -> ux_ehci_hsiso_ed_frstart = frindex_start & 7u;
            }
        } /* if (ed -> ux_ehci_hsiso_ed_fr_sw == 0xFE) */

        /* Process count to target micro frames.  */
        fr_sw = (USHORT)(ed -> ux_ehci_hsiso_ed_fr_sw << ed -> ux_ehci_hsiso_ed_frinterval_shift);
        fr_sw = (USHORT)(fr_sw + ed -> ux_ehci_hsiso_ed_frstart);
        fr_sw &= 0x7u;

        /* Build requests.  */
        for (i = 0;
            i < n_fr;
            i += ed -> ux_ehci_hsiso_ed_frinterval)
        {

            /* Get micro-frame index.  */
            frindex = i + fr_sw;
            frindex &= 7u;

            /* Get iTD.  */
            fr_td = ed -> ux_ehci_hsiso_ed_fr_td[frindex >> 1];

            /* Check load status.  */
            if (fr_td -> ux_ehci_hsiso_td_frload & (1u << frindex))
            {
                break;
            }

            /* Get a transfer request.  */
            transfer = ed -> ux_ehci_hsiso_ed_transfer_first_new;
            if (transfer == UX_NULL)
            {
                break;
            }

            /* Get control status.  */
            control = fr_td -> ux_ehci_hsiso_td_control[frindex];

            /* If there are multiple micro-frames, check
            ** (1) If target micro-frame is already loaded
            ** (2) If the micro-frame overlapped current one
            */
            if (n_fr)
            {

                /* If already loaded(active), break.  */
                if (control & UX_EHCI_HSISO_STATUS_ACTIVE)
                {
                    break;
                }

                /* If index overlap, break.  */

                /* Check if frindex exceeds same FRINDEX in next frame.  */
                if ((frindex == frindex_now) || (i + fr_sw >= frindex_now + 8))
                {
                    break;
                }
            }

            /* Now new request is linking.  */

            /* Process count inc.  */
            ed -> ux_ehci_hsiso_ed_fr_sw ++;

            /* Sanity check, iTD is not linked.  */
            while(fr_td -> ux_ehci_hsiso_td_fr_transfer[frindex & 1u] != UX_NULL);

            /* Link it to iTD.  */
            fr_td -> ux_ehci_hsiso_td_fr_transfer[frindex & 1u] = transfer;

            /* Remove it from new free list.  */
            ed -> ux_ehci_hsiso_ed_transfer_first_new =
                        transfer -> ux_transfer_request_next_transfer_request;

            /* Update load map.  */
            fr_td -> ux_ehci_hsiso_td_frload = (UCHAR)(fr_td -> ux_ehci_hsiso_td_frload | (1u << frindex));
            ed -> ux_ehci_hsiso_ed_frload = (UCHAR)(ed -> ux_ehci_hsiso_ed_frload | (1u << frindex));

            /* Get transfer size.  */
            trans_bytes = transfer -> ux_transfer_request_requested_length;

            /* Limit max transfer size.  */
            if (trans_bytes > fr_td -> ux_ehci_hsiso_td_max_trans_size)
                trans_bytes = fr_td -> ux_ehci_hsiso_td_max_trans_size;

            /* Build the control.  */

            /* Keep IOC and PG.  */
            control &= UX_EHCI_HSISO_IOC | UX_EHCI_HSISO_PG_MASK;

            /* New transfer size.  */
            control |= (trans_bytes << UX_EHCI_HSISO_XACT_LENGTH_SHIFT);

            /* Active.  */
            control |= (UX_EHCI_HSISO_STATUS_ACTIVE);

            /* Build offset & BPs (5,6/3,4).  */

            /* Get physical buffer address.  */
            bp.void_ptr = _ux_utility_physical_address(transfer -> ux_transfer_request_data_pointer);

            /* Get page offset.  */
            pg_addr = bp.value & UX_EHCI_PAGE_ALIGN;
            pg_offset = bp.value & UX_EHCI_HSISO_XACT_OFFSET_MASK;

            /* Offset in control.  */
            control |= pg_offset;

            /* Save BPs.  */
            pg = (frindex & 1u) ? 5 : 3;

            /* PG in control.  */
            control |= pg << UX_EHCI_HSISO_PG_SHIFT;

            /* Save BPs.  */

            /* First BP.  */
            bp.value = pg_addr;
            fr_td -> ux_ehci_hsiso_td_bp[pg] = bp.void_ptr;

            /* Next page information.  */
            pg ++;
            pg_addr += UX_EHCI_PAGE_SIZE;

            /* Next BP.  */
            bp.value = pg_addr;
            fr_td -> ux_ehci_hsiso_td_bp[pg] = bp.void_ptr;

            /* Save control.  */
            UX_DATA_MEMORY_BARRIER
            fr_td -> ux_ehci_hsiso_td_control[frindex] = control;

        } /* for(i = 0; i < n_fr; )  */

    } /* if (ed -> ux_ehci_hsiso_ed_transfer_first_new) */

    /* If there is no transfer, need start again any way.  */
    if (ed -> ux_ehci_hsiso_ed_frload == 0)
    {
        ed -> ux_ehci_hsiso_ed_frstart = 0xFF;
        ed -> ux_ehci_hsiso_ed_fr_sw = 0;
        ed -> ux_ehci_hsiso_ed_fr_hc = 0;
    }

   /* Return next iTD in scan list.  */
   return(next_scan_td);
}
