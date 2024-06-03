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
/*    _ux_hcd_ehci_asynchronous_endpoint_create           PORTABLE C      */ 
/*                                                           6.1.2        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will create an asynchronous endpoint. The control     */
/*    and bulk endpoints fall into this category.                         */ 
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
/*    _ux_hcd_ehci_ed_obtain                Obtain EHCI ED                */ 
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
/*                                            optimized based on compile  */
/*                                            definitions,                */
/*                                            resulting in version 6.1    */
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile warnings,     */
/*                                            resulting in version 6.1.2  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ehci_asynchronous_endpoint_create(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint)
{

UX_DEVICE               *device;
UX_EHCI_ED              *ed;
UX_EHCI_LINK_POINTER    queue_head;


    /* We need to take into account the nature of the HCD to define the max size
       of any transfer in the transfer request.  */
    endpoint -> ux_endpoint_transfer_request.ux_transfer_request_maximum_length =  UX_EHCI_MAX_PAYLOAD;
    
    /* Obtain a ED for this new endpoint. This ED will live as long as the endpoint is active 
       and will be the container for the tds.  */
    ed =  _ux_hcd_ehci_ed_obtain(hcd_ehci);
    if (ed == UX_NULL)
        return(UX_NO_ED_AVAILABLE);

    /* Attach the ED to the endpoint container.  */
    endpoint -> ux_endpoint_ed =  (VOID *) ed;

    /* Now do the opposite, attach the ED container to the physical ED.  */
    ed -> REF_AS.INTR.ux_ehci_ed_endpoint =  endpoint;

    /* Set the default MPS Capability info in the ED.  */
    ed -> ux_ehci_ed_cap0 =  endpoint -> ux_endpoint_descriptor.wMaxPacketSize << UX_EHCI_QH_MPS_LOC;
    
    /* Set the default NAK reload count.  */
    ed -> ux_ehci_ed_cap0 |=  UX_EHCI_QH_NCR;
    
    /* If the device is not high speed and the endpoint is control, then the CEF bit must be set to on.  */
    device =  endpoint -> ux_endpoint_device;
    if ((device -> ux_device_speed != UX_HIGH_SPEED_DEVICE) &&
        ((endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_CONTROL_ENDPOINT))
        ed -> ux_ehci_ed_cap0 |=  UX_EHCI_QH_CEF;

    /* Set the device address.  */
    ed -> ux_ehci_ed_cap0 |=  device -> ux_device_address;
    
    /* Add the endpoint address.  */
    ed -> ux_ehci_ed_cap0 |=  (endpoint -> ux_endpoint_descriptor.bEndpointAddress &
                                            ~UX_ENDPOINT_DIRECTION) << UX_EHCI_QH_ED_AD_LOC;

    /* Set the High Bandwidth Pipe Multiplier to 1.  */    
    ed -> ux_ehci_ed_cap1 |=  UX_EHCI_QH_HBPM;
    
    /* Set the device speed for full and low speed devices behind a hub the hub address and the 
       port index must be stored in the endpoint.  */
    switch (device -> ux_device_speed)
    {

    case  UX_HIGH_SPEED_DEVICE:

        ed -> ux_ehci_ed_cap0 |=  UX_EHCI_QH_HIGH_SPEED;
        break;


    case  UX_LOW_SPEED_DEVICE:
        
        ed -> ux_ehci_ed_cap0 |=  UX_EHCI_QH_LOW_SPEED;
        break;

    case  UX_FULL_SPEED_DEVICE:

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
            
    /* We need to insert this new endpoint into the asynchronous list. All new EDs are inserted at the 
       end of the list. The current ED will be pointing to the first ED in the list.  */
    queue_head.void_ptr = _ux_utility_physical_address(hcd_ehci -> ux_hcd_ehci_asynch_first_list);
    queue_head.value |= UX_EHCI_QH_TYP_QH;
    ed -> ux_ehci_ed_queue_head =  queue_head.ed_ptr;
    ed -> ux_ehci_ed_next_ed =     hcd_ehci -> ux_hcd_ehci_asynch_first_list;

    /* Now we compute the address and the data type to fill the QH pointer with.  */
    queue_head.void_ptr = _ux_utility_physical_address(ed);
    queue_head.value |= UX_EHCI_QH_TYP_QH;
    hcd_ehci -> ux_hcd_ehci_asynch_last_list -> ux_ehci_ed_queue_head = queue_head.ed_ptr;
    ed -> ux_ehci_ed_previous_ed =  hcd_ehci -> ux_hcd_ehci_asynch_last_list;

    /* Update the link of the previous ED.  */
    ed -> ux_ehci_ed_previous_ed -> ux_ehci_ed_next_ed =  ed;
    
    /* Remember the new last QH.  */
    hcd_ehci -> ux_hcd_ehci_asynch_last_list =  ed;

    /* Return successful completion.  */
    return(UX_SUCCESS);         
}

