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
/*    _ux_hcd_ohci_isochronous_endpoint_create            PORTABLE C      */ 
/*                                                           6.1.6        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function creates an isochronous endpoint.                     */ 
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
/*    _ux_hcd_ohci_ed_obtain                Obtain an OHCI ED             */ 
/*    _ux_hcd_ohci_isochronous_td_obtain    Obtain an OHCI TD             */ 
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
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            filled max transfer length, */
/*                                            resulting in version 6.1.6  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ohci_isochronous_endpoint_create(UX_HCD_OHCI *hcd_ohci, UX_ENDPOINT *endpoint)
{

UX_HCD_OHCI_HCCA        *ohci_hcca;
UX_DEVICE               *device;
UX_OHCI_ED              *ed;
UX_OHCI_ED              *ed_list;
UX_OHCI_ISO_TD          *td;


    /* Get the pointer to the HCCA.  */
    ohci_hcca =  hcd_ohci -> ux_hcd_ohci_hcca;
    
    /* Obtain a ED for this new endpoint. This ED will live as long as
       the endpoint is active and will be the container for the tds.   */
    ed =  _ux_hcd_ohci_ed_obtain(hcd_ohci);
    if(ed==UX_NULL)
        return(UX_NO_ED_AVAILABLE);

    /* Obtain a dummy isoch TD for terminating the ED transfer chain.  */
    td =  _ux_hcd_ohci_isochronous_td_obtain(hcd_ohci);
    if (td == UX_NULL)
    {
        ed -> ux_ohci_ed_status =  UX_UNUSED;
        return(UX_NO_TD_AVAILABLE);
    }

    /* Attach the ED to the endpoint container.  */
    endpoint -> ux_endpoint_ed =  (VOID *) ed;

    /* We need to take into account the nature of the HCD to define the max size
       of any transfer in the transfer request.  */
    endpoint -> ux_endpoint_transfer_request.ux_transfer_request_maximum_length =
                            endpoint -> ux_endpoint_descriptor.wMaxPacketSize;

    /* Program the ED for subsequent transfers we need to set the following things:
        1) Address of the device 
        2) endpoint number 
        3) speed (always full speed for iso)
        4) format of TD 
        5) maximum packet size  */
        
    device =  endpoint -> ux_endpoint_device;
    ed -> ux_ohci_ed_dw0 =  device -> ux_device_address | UX_OHCI_ED_ISOCHRONOUS | 
                                ((ULONG) (endpoint -> ux_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION)) << 7 |
                                ((ULONG) (endpoint -> ux_endpoint_descriptor.wMaxPacketSize)) << 16;
            
    /* Hook the TD to both the tail and head of the ED.  */
    ed -> ux_ohci_ed_tail_td =  _ux_utility_physical_address(td);
    ed -> ux_ohci_ed_head_td =  _ux_utility_physical_address(td);

    /* Attach the ED to the 1ms interrupt tree. We scan the interrupt tree until the last entry.  */
    ed_list =  _ux_utility_virtual_address(ohci_hcca -> ux_hcd_ohci_hcca_ed[0]);
    while (ed_list -> ux_ohci_ed_next_ed != UX_NULL)
        ed_list =  _ux_utility_virtual_address(ed_list -> ux_ohci_ed_next_ed);

    /* Now ed_list, points to the last ED, which is in the 1ms entry.  */
    ed -> ux_ohci_ed_previous_ed =   ed_list;
    ed_list -> ux_ohci_ed_next_ed =  _ux_utility_physical_address(ed);
    
    /* Return success.  */
    return(UX_SUCCESS);         
}

