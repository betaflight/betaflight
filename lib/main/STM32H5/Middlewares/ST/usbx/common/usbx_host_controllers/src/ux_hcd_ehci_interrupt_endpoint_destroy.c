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
/*    _ux_hcd_ehci_interrupt_endpoint_destroy             PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will destroy an interrupt endpoint.                   */
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
/*    _ux_hcd_ehci_door_bell_wait           Setup doorbell wait           */
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
/*                                            resulting in version 6.1.6  */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ehci_interrupt_endpoint_destroy(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint)
{

UX_EHCI_ED                    *ed;
UX_EHCI_ED                    *prev_ed;
ULONG                         frindex;
ULONG                         max_packet_size;


    /* From the endpoint container fetch the EHCI ED descriptor.  */
    ed =  (UX_EHCI_ED *) endpoint -> ux_endpoint_ed;

    /* Access to periodic list.  */
    _ux_host_mutex_on(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);

    /* Unlink the periodic item.  */
    _ux_hcd_ehci_periodic_descriptor_link(ed -> ux_ehci_ed_previous_ed,
            UX_NULL, UX_NULL, ed -> ux_ehci_ed_queue_head);

    /* Update the interrupt ED scan list.  */
    prev_ed = hcd_ehci -> ux_hcd_ehci_interrupt_ed_list;

    /* Check if ED in head of scan list.  */
    if (prev_ed == ed)

        /* Point head to the next ED.  */
        hcd_ehci -> ux_hcd_ehci_interrupt_ed_list = ed -> ux_ehci_ed_next_ed;
    else
    {

        /* Try to find previous ED in scan list.  */
        /* It's at head now.  */
        while(prev_ed -> ux_ehci_ed_next_ed)
        {

            /* The expected ED is found.  */
            if (prev_ed -> ux_ehci_ed_next_ed == ed)
            {

                /* Point next to next of the ED.  */
                prev_ed -> ux_ehci_ed_next_ed = ed -> ux_ehci_ed_next_ed;
                break;
            }

            /* Try next ED.  */
            prev_ed = prev_ed -> ux_ehci_ed_next_ed;
        }
    }

    /* Update micro-frame loads of anchor.  */

    /* Calculate max packet size.  */
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
    if (endpoint -> ux_endpoint_device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
        max_packet_size = endpoint -> ux_endpoint_descriptor.wMaxPacketSize & UX_MAX_PACKET_SIZE_MASK;
    else
#endif
    {
        max_packet_size = endpoint -> ux_endpoint_descriptor.wMaxPacketSize & UX_MAX_NUMBER_OF_TRANSACTIONS_MASK;
        max_packet_size >>= UX_MAX_NUMBER_OF_TRANSACTIONS_SHIFT;
        if (max_packet_size < 3)
            max_packet_size ++;
        max_packet_size *= endpoint -> ux_endpoint_descriptor.wMaxPacketSize & UX_MAX_PACKET_SIZE_MASK;
    }

    /* Update according to ED S-Mask.  */
    for(frindex = 0; frindex < 8; frindex ++)
    {

        /* Check schedule mask.  */
        if ((ed -> ux_ehci_ed_cap1 & (UX_EHCI_SMASK_0 << frindex)))
        {

#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)

            /* Start split check.  */
            if (endpoint -> ux_endpoint_device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
            {

                /* Decrement the start split count.  */
                ed -> REF_AS.INTR.ux_ehci_ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_ssplit_count[frindex] --;

                /* Check next endpoint if it's IN (load in C-Mask).  */
                if (endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION)
                    continue;
            }
#endif
            /* Decrement the microframe load.  */
            ed -> REF_AS.INTR.ux_ehci_ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[frindex] = (USHORT)(ed -> REF_AS.INTR.ux_ehci_ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[frindex] - max_packet_size);

            /* S-Mask found, no C-Mask at the same time, skip C-Mask check.  */
            continue;
        }

#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)

        /* Complete split interrupt IN check in C-Mask.  */
        if ((endpoint -> ux_endpoint_device -> ux_device_speed != UX_HIGH_SPEED_DEVICE) &&
            (endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) &&
            (ed -> ux_ehci_ed_cap1 & (UX_EHCI_CMASK_0 << frindex)))
        {

            /* Decrement the microframe load.  */
            ed -> REF_AS.INTR.ux_ehci_ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[frindex] = (USHORT)(ed -> REF_AS.INTR.ux_ehci_ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[frindex] - max_packet_size);
        }
#endif
    }

    /* Release periodic list.  */
    _ux_host_mutex_off(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);

    /* Arm the doorbell and wait for its completion.  */
    _ux_hcd_ehci_door_bell_wait(hcd_ehci);

    /* Now we can safely make the ED free.  */
    ed -> ux_ehci_ed_status =  UX_UNUSED;

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

