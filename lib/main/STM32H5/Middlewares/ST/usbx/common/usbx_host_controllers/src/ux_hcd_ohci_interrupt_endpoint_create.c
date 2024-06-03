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
/*    _ux_hcd_ohci_interrupt_endpoint_create              PORTABLE C      */ 
/*                                                           6.1.6        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function will create an interrupt endpoint. The interrupt     */ 
/*     endpoint has an interval of operation from 1 to 255. In OHCI, the  */ 
/*     hardware assisted interrupt is from 1 to 32.                       */
/*                                                                        */
/*     This routine will match the best interval for the OHCI hardware.   */
/*     It will also determine the best node to hook the endpoint based on */
/*     the load that already exists on the horizontal ED chain.           */
/*                                                                        */
/*     For the ones curious about this coding. The tricky part is to      */
/*     understand how the interrupt matrix is constructed. We have used   */
/*     eds with the skip bit on to build a frame of anchor eds. Each ED   */ 
/*     creates a node for an appropriate combination of interval          */ 
/*     frequency in the list.                                             */
/*                                                                        */
/*     After obtaining a pointer to the list with the lowest traffic, we  */
/*     traverse the list from the highest interval until we reach the     */ 
/*     interval required. At that node, we anchor our real ED to the node */ 
/*     and link the ED that was attached to the node to our ED.           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ohci                              Pointer to OHCI               */ 
/*    endpoint                              Pointer to endpoint           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ohci_ed_obtain                Obtain OHCI ED                */ 
/*    _ux_hcd_ohci_least_traffic_list_get   Get least traffic list        */ 
/*    _ux_hcd_ohci_regular_td_obtain        Obtain OHCI regular TD        */ 
/*    _ux_utility_physical_address          Get physical address          */ 
/*    _ux_utility_virtual_address           Get virtual address           */ 
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
/*                                            fixed physical and virtual  */
/*                                            address conversion,         */
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            filled max transfer length, */
/*                                            resulting in version 6.1.6  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ohci_interrupt_endpoint_create(UX_HCD_OHCI *hcd_ohci, UX_ENDPOINT *endpoint)
{

UX_DEVICE       *device;
UX_OHCI_ED      *ed;
UX_OHCI_ED      *ed_list;
UX_OHCI_ED      *next_ed;
UX_OHCI_TD      *td;
UINT            interval;
UINT            interval_index;
UINT            interval_ohci;


    /* Obtain a ED for this new endpoint. This ED will live as long as the endpoint 
       is active and will be the container for the tds.  */
    ed =  _ux_hcd_ohci_ed_obtain(hcd_ohci);
    if (ed == UX_NULL)
        return(UX_NO_ED_AVAILABLE);

    /* Obtain a dummy TD for terminating the ED transfer chain.  */
    td =  _ux_hcd_ohci_regular_td_obtain(hcd_ohci);
    if (td == UX_NULL)
    {
    
        ed -> ux_ohci_ed_status =  UX_UNUSED;
        return(UX_NO_TD_AVAILABLE);
    }

    /* Attach the ED to the endpoint container.  */
    endpoint -> ux_endpoint_ed =  (VOID *) ed;

    /* We need to take into account the nature of the HCD to define the max size
       of any transfer in the transfer request.  */
    endpoint -> ux_endpoint_transfer_request.ux_transfer_request_maximum_length =  UX_OHCI_MAX_PAYLOAD;

    /* Program the ED for subsequent transfers we need to set the following things:
        1) Address of the device 
        2) endpoint number 
        3) speed
        4) format of TD 
        5) maximum packet size */
    device =                endpoint -> ux_endpoint_device;
    ed -> ux_ohci_ed_dw0 =  device -> ux_device_address |
                                ((ULONG) (endpoint -> ux_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION)) << 7 |
                                ((ULONG) endpoint -> ux_endpoint_descriptor.wMaxPacketSize) << 16;
            
    if (device -> ux_device_speed == UX_LOW_SPEED_DEVICE)
        ed -> ux_ohci_ed_dw0 |=  UX_OHCI_ED_LOW_SPEED;
        
    /* Hook the TD to both the tail and head of the ED.  */
    ed -> ux_ohci_ed_tail_td =  _ux_utility_physical_address(td);
    ed -> ux_ohci_ed_head_td =  _ux_utility_physical_address(td);

    /* Get the list index with the least traffic.  */
    ed_list =  _ux_hcd_ohci_least_traffic_list_get(hcd_ohci);
    
    /* Get the interval for the endpoint and match it to a OHCI list. We match anything that 
       is > 32ms to the 32ms interval list. The 32ms list is list 0, 16ms list is 1 ...
       the 1ms list is number 5.  */
    interval =        endpoint -> ux_endpoint_descriptor.bInterval;
    interval_index =  0x10;
    interval_ohci =   1;

    /* Do a sanity check if the frequency is 0. That should not happen, so treat it as 1.  */
    if (interval == 0)
    {

        interval =  1;
    }

    /* If the frequency is beyond the OHCI framework, make it the maximum of 32.  */
    if (interval >= 32)
    {

        interval_ohci =  0;
    }
    else
    {

        /* We parse the interval from the high bits. This gives us the first power of 2 entry in the tree.  */
        while (interval_index != 0)
        {

            /* When we find the first bit of the interval the current value of interval_ohci is set to the the list index.  */
            if (interval & interval_index)
                break;
                
            /* Go down the tree one entry.  */
            interval_ohci++;
            
            /* And shift the bit of the device interval to check.  */
            interval_index =  interval_index >> 1;
        }
    }

    /* Now we need to scan the list of eds from the lowest load entry until we reach the 
       appropriate interval node. The depth index is the interval OHCI value and the 1st 
       entry is pointed by the ED list entry.  */
    while (interval_ohci--)
    {

        ed_list =  _ux_utility_virtual_address(ed_list -> ux_ohci_ed_next_ed);
        while (!(ed_list -> ux_ohci_ed_dw0 & UX_OHCI_ED_SKIP))
            ed_list =  _ux_utility_virtual_address(ed_list -> ux_ohci_ed_next_ed);
    }  
         
    /* We found the node entry of the ED pointer that will be the anchor for this interrupt 
       endpoint. Now we attach this endpoint to the anchor and rebuild the chain.   */
    next_ed =  ed_list -> ux_ohci_ed_next_ed;

    /* Check for end of tree which happens for devices with interval of 1. In this case
       there might not be a next_ed.  */
    if (next_ed != UX_NULL)
    {
        next_ed = _ux_utility_virtual_address(next_ed);
        next_ed -> ux_ohci_ed_previous_ed =  ed;
    }
    ed -> ux_ohci_ed_next_ed =  _ux_utility_physical_address(next_ed);
    ed -> ux_ohci_ed_previous_ed =  ed_list;
    ed_list -> ux_ohci_ed_next_ed =  _ux_utility_physical_address(ed);

    /* Return successful completion.  */
    return(UX_SUCCESS);         
}

