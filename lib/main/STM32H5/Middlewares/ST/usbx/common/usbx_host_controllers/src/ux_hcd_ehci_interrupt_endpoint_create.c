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
/*    _ux_hcd_ehci_interrupt_endpoint_create              PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will create an interrupt endpoint. The interrupt      */ 
/*    endpoint has an interval of operation from 1 to 255. In EHCI, the   */ 
/*    hardware assisted interrupt is from 1 to 32.                        */
/*                                                                        */
/*    This routine will match the best interval for the EHCI hardware.    */
/*    It will also determine the best node to hook the endpoint based on  */ 
/*    the load that already exists on the horizontal ED chain.            */
/*                                                                        */
/*    For the ones curious about this coding. The tricky part is to       */
/*    understand how the interrupt matrix is constructed. We have used    */ 
/*    eds with the skip bit on to build a frame of anchor eds. Each ED    */ 
/*    creates a node for an appropriate combination of interval frequency */ 
/*    in the list.                                                        */
/*                                                                        */
/*    After obtaining a pointer to the list with the lowest traffic, we   */
/*    traverse the list from the highest interval until we reach the      */ 
/*    interval required. At that node, we anchor our real ED to the node  */ 
/*    and link the ED that was attached to the node to our ED.            */ 
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
/*    _ux_hcd_ehci_ed_obtain                Obtain an ED                  */ 
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
/*                                            optimized based on compile  */
/*                                            definitions,                */
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
UINT  _ux_hcd_ehci_interrupt_endpoint_create(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint)
{

UX_DEVICE                       *device;
UX_EHCI_ED                      *ed;
UX_EHCI_ED                      *ed_list;
UX_EHCI_ED                      *ed_anchor;
UINT                            interval;
UINT                            poll_depth;
ULONG                           max_packet_size;
ULONG                           num_transaction;
ULONG                           microframe_load[8];
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
ULONG                           microframe_ssplit_count[8];
UINT                            csplit_count;
ULONG                           cmask;
#else
#define                         microframe_ssplit_count     UX_NULL
#endif
UX_EHCI_PERIODIC_LINK_POINTER   lp;
UINT                            i;


    /* Get the pointer to the device.  */
    device =  endpoint -> ux_endpoint_device;

#if !defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)

    /* Only high speed transfer supported without split transfer.  */
    if (device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
        return(UX_FUNCTION_NOT_SUPPORTED);
#endif

    /* We need to take into account the nature of the HCD to define the max size
       of any transfer in the transfer request.  */
    endpoint -> ux_endpoint_transfer_request.ux_transfer_request_maximum_length =  UX_EHCI_MAX_PAYLOAD;

    /* Obtain a ED for this new endpoint. This ED will live as long as the endpoint is 
       active and will be the container for the tds.  */
    ed =  _ux_hcd_ehci_ed_obtain(hcd_ehci);
    if (ed == UX_NULL)
        return(UX_NO_ED_AVAILABLE);
                
    /* Attach the ED to the endpoint container.  */
    endpoint -> ux_endpoint_ed =  (VOID *) ed;
    
    /* Now do the opposite, attach the ED container to the physical ED.  */
    ed -> REF_AS.INTR.ux_ehci_ed_endpoint =  endpoint;

    /* Set the default MPS Capability info in the ED.  */
    max_packet_size = endpoint -> ux_endpoint_descriptor.wMaxPacketSize & UX_MAX_PACKET_SIZE_MASK;
    ed -> ux_ehci_ed_cap0 =  max_packet_size << UX_EHCI_QH_MPS_LOC;
    
    /* Set the device address.  */
    ed -> ux_ehci_ed_cap0 |=  device -> ux_device_address;

    /* Add the endpoint address.  */
    ed -> ux_ehci_ed_cap0 |=  (endpoint -> ux_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION) << UX_EHCI_QH_ED_AD_LOC;

    /* Set the High Bandwidth Pipe Multiplier to number transactions.  */    
    num_transaction = (endpoint -> ux_endpoint_descriptor.wMaxPacketSize & UX_MAX_NUMBER_OF_TRANSACTIONS_MASK) >> UX_MAX_NUMBER_OF_TRANSACTIONS_SHIFT;
    if (num_transaction < 3)
        num_transaction ++;
    ed -> ux_ehci_ed_cap1 |= (num_transaction << UX_EHCI_QH_HBPM_LOC);

    /* Set the device speed for full and low speed devices behind a HUB. The HUB address and the 
       port index must be stored in the endpoint. For low/full speed devices, the C-mask field must be set.  */
    switch (device -> ux_device_speed)
    {

    case UX_HIGH_SPEED_DEVICE:
        ed -> ux_ehci_ed_cap0 |=  UX_EHCI_QH_HIGH_SPEED;
        break;

    case UX_LOW_SPEED_DEVICE:
        ed -> ux_ehci_ed_cap0 |=  UX_EHCI_QH_LOW_SPEED;

        /* Fall through.  */
    default:

#if UX_MAX_DEVICES > 1
        /* The device must be on a hub for this code to execute. We still do a sanity check.  */
        if (device -> ux_device_parent != UX_NULL)
        {

            /* Store the parent hub device address.  */
            ed -> ux_ehci_ed_cap1 |=  device -> ux_device_parent -> ux_device_address << UX_EHCI_QH_HUB_ADDR_LOC;

            /* And the port index onto which this device is attached.  */                                    
            ed -> ux_ehci_ed_cap1 |=  device -> ux_device_port_location << UX_EHCI_QH_PORT_NUMBER_LOC;
        }
#endif
        break;
    }

    /* Get the interval for the endpoint and match it to a EHCI list.
       We match anything that is > 32ms to the 32ms interval layer.
       The 32ms list is layer 5, 16ms list is 4 ... the 1ms list is depth 0.  */
    interval = endpoint -> ux_endpoint_descriptor.bInterval;
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
    if (device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
    {

        /* Convert from ms to 2^i.  */
        for (i = 0; i < 16; i ++)
        {
            if (interval <= (1u << i))
                break;
        }

        /* Index 0 for 1ms (interval 4).  */
        interval = 4 + i;
    }
    else
#endif
    {
        /* High-speed interval is 2^(interval - 1) * 1/2^3.   */
        if (interval <= 4)
            i = 0;
        else
            i = interval - 4;

        /* Index 0 for 1ms.  */
    }

    /* Match > 32ms to 32ms list.  */
    /* Poll depth deeper, interval smaller.  */
    if (i > 5)
        poll_depth = 0;
    else
        poll_depth = 5 - i;

    /* Keep interval < 1ms for micro-frame calculation.  */
    /* Make it index steps to move.  */
    if (interval > 0)
    {
        interval --;
        interval &= 0x3;
    }
    interval = (1u << interval); /* 1 (1/8ms), 2, 4, 8 (1ms)  */

    /* We are now updating the periodic list.  */
    _ux_host_mutex_on(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);

    /* Get the list index with the least traffic.  */
    ed_list =  _ux_hcd_ehci_least_traffic_list_get(hcd_ehci, microframe_load, microframe_ssplit_count);

    /* Now we need to scan the list of eds from the lowest load entry until we reach the 
       appropriate interval node. The depth index is the interval EHCI value and the 
       1st entry is pointed by the ED list entry.  */
    ed_anchor = _ux_hcd_ehci_poll_rate_entry_get(hcd_ehci, ed_list, poll_depth);

    /* Save anchor pointer for interrupt ED.  */
    ed -> REF_AS.INTR.ux_ehci_ed_anchor = ed_anchor;

    /* Calculate packet size with num transactions.  */
    max_packet_size *= num_transaction;

    /* Go through the transaction loads for start
       index of micro-frame.  */
    for (i = 0; i < interval; i ++)
    {

        /* Skip if load too much.  */
        if (microframe_load[i] + max_packet_size > UX_MAX_BYTES_PER_MICROFRAME_HS)
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
    if (i >= interval)
    {
        _ux_host_mutex_off(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);
        ed -> ux_ehci_ed_status = UX_UNUSED;
        return(UX_NO_BANDWIDTH_AVAILABLE);
    }

    /* Now start microframe index is calculated, build masks.  */

    /* It's interval is larger than 1ms, use any of micro-frame.  */
    if (interval >= 8)
    {

        /* Interrupt schedule.  */
        ed -> ux_ehci_ed_cap1 |= (UX_EHCI_SMASK_0 << i);

#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
        /* For split transfer, complete split should be scheduled.  */
        if (device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
        {

            /* Interrupt IN/OUT:
               must schedule a complete-split transaction in each of the two
               microframes following the first microframe in which the
               full/low speed transaction is budgeted. An additional
               complete-split must also be scheduled in the third following
               microframe unless the full/low speed transaction was budgeted
               to start in Y6. */

            if (i == 5)
            {

                /* Budgeted in Y6, Follow two (C7, C0).  */
                cmask = UX_EHCI_CMASK_INT_Y5;
                csplit_count = 2;
            }
            else
            {

                /* Follow three.  */
                cmask = UX_EHCI_CMASK_INT_Y0 << i;
                if (i > 3)
                {
                    cmask |= cmask >> 8;
                    cmask &= UX_EHCI_CMASK_MASK;
                }
                csplit_count = 3;
            }

            /* Reserve count for SSplit (Max 16).  */
            ed_anchor ->REF_AS.ANCHOR.ux_ehci_ed_microframe_ssplit_count[i] ++;

            /* Reserve packet bytes for microframe load.  */
            if (endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION)
            {

                /* Reserve load for CSplit.  */
                ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[(i + 2)&7] = (USHORT)(ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[(i + 2)&7] + max_packet_size);
                ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[(i + 3)&7] = (USHORT)(ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[(i + 3)&7] + max_packet_size);

                /* Need additional CSplit.  */
                if (csplit_count > 2)
                    ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[(i + 4)&7] = (USHORT)(ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[(i + 4)&7] + max_packet_size);
            }
            else
            {

                /* Reserve load for SSplit.  */
                ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[i] = (USHORT)(ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[i] + max_packet_size);
            }

            /* Update schedule masks.  */
            ed -> ux_ehci_ed_cap1 |= cmask;
        }
        else
#endif
        {
            /* Update anchor micro-frame load.  */
            ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[i] = (USHORT)(ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[i] + max_packet_size);
        }
    }
    else
    {

        /* It must be high speed high bandwidth one.  */
        switch(interval)
        {
        case 1:
            ed -> ux_ehci_ed_cap1 |= UX_EHCI_SMASK_INTERVAL_1;
            break;
        case 2:
            ed -> ux_ehci_ed_cap1 |= UX_EHCI_SMASK_INTERVAL_2 << i;
            break;
        default: /* 4, interval 3, 1/2ms */
            ed -> ux_ehci_ed_cap1 |= UX_EHCI_SMASK_INTERVAL_3 << i;
            break;
        }

        /* Update anchor micro-frame loads.  */
        for (; i < 8; i += interval)
            ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[i] = (USHORT)(ed_anchor -> REF_AS.ANCHOR.ux_ehci_ed_microframe_load[i] + max_packet_size);
    }

    /* We found the node entry of the ED pointer that will be the anchor for this interrupt 
       endpoint. Now we attach this endpoint to the anchor and rebuild the chain.  */

    /* Physical LP, with Typ QH, clear T.  */
    lp.void_ptr = _ux_utility_physical_address(ed);
    lp.value |= UX_EHCI_TYP_QH;

    /* Save previous LP: to anchor.  */
    ed -> ux_ehci_ed_previous_ed = ed_anchor;

    /* Link the QH at next to anchor.  */
    _ux_hcd_ehci_periodic_descriptor_link(ed_anchor, lp.void_ptr, ed, ed_anchor -> ux_ehci_ed_queue_head);

    /* Insert ED to interrupt scan list for fast done queue scan.  */
    ed -> ux_ehci_ed_next_ed = hcd_ehci -> ux_hcd_ehci_interrupt_ed_list;
    hcd_ehci -> ux_hcd_ehci_interrupt_ed_list = ed;

    /* Release the periodic list.  */
    _ux_host_mutex_off(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);

    /* Return successful completion.  */
    return(UX_SUCCESS);         
}
