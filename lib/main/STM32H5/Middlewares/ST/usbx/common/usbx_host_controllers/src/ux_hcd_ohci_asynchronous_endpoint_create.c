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
/*    _ux_hcd_ohci_asynchronous_endpoint_create           PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function will create an asynchronous endpoint. The control    */
/*     and bulk endpoints fall into this category.                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ohci                              Pointer to OHCI HCD           */ 
/*    endpoint                              Pointer to endpoint           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ohci_ed_obtain                Obtain a new ED               */ 
/*    _ux_hcd_ohci_register_read            Read OHCI register            */ 
/*    _ux_hcd_ohci_register_write           Write OHCI register           */ 
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
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed an addressing issue,  */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed addressing issues,    */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ohci_asynchronous_endpoint_create(UX_HCD_OHCI *hcd_ohci, UX_ENDPOINT *endpoint)
{

UX_DEVICE       *device;
UX_OHCI_ED      *ed;
UX_OHCI_ED      *head_ed;
UX_OHCI_TD      *td;
ULONG           ohci_register;


    /* We need to take into account the nature of the HCD to define the max size
       of any transfer in the transfer request.  */
    endpoint -> ux_endpoint_transfer_request.ux_transfer_request_maximum_length =  UX_OHCI_MAX_PAYLOAD;
    
    /* Obtain a ED for this new endpoint. This ED will live as long as the endpoint is active and 
       will be the container for the TDs.  */
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

    /* Now do the opposite, attach the ED container to the physical ED.  */
    ed -> ux_ohci_ed_endpoint =  endpoint;
    
    /* Program the ED for subsequent transfers. We need to set the following things:
        1) Address of the device 
        2) endpoint number 
        3) speed
        4) format of TD 
        5) maximum packet size */
    device =  endpoint -> ux_endpoint_device;
    ed -> ux_ohci_ed_dw0 =  device -> ux_device_address |
                            ((ULONG) (endpoint -> ux_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION)) << 7 |
                            ((ULONG) endpoint -> ux_endpoint_descriptor.wMaxPacketSize) << 16;
            
    if (device -> ux_device_speed == UX_LOW_SPEED_DEVICE)
        ed -> ux_ohci_ed_dw0 |=  UX_OHCI_ED_LOW_SPEED;
        
    /* Hook the TD to both the tail and head of the ED.  */
    ed -> ux_ohci_ed_tail_td =  _ux_utility_physical_address(td);
    ed -> ux_ohci_ed_head_td =  _ux_utility_physical_address(td);
    
    /* We now need to get the type of transfer (control or bulk) to hook this ED to the appropriate list. 
       We also enable the appropriate list.  */
    switch ((endpoint -> ux_endpoint_descriptor.bmAttributes) & UX_MASK_ENDPOINT_TYPE)
    {

    case UX_CONTROL_ENDPOINT:

        head_ed =  (UX_OHCI_ED *) _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_CONTROL_HEAD_ED);
        ed -> ux_ohci_ed_next_ed =  head_ed;
        _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_CONTROL_HEAD_ED, (ULONG) _ux_utility_physical_address(ed));
        ohci_register =  _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_CONTROL);
        ohci_register |=  OHCI_HC_CR_CLE;
        _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_CONTROL, ohci_register);
        break;


    case UX_BULK_ENDPOINT:

        head_ed =  (UX_OHCI_ED *) _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_BULK_HEAD_ED);
        ed -> ux_ohci_ed_next_ed =  head_ed;
        _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_BULK_HEAD_ED, (ULONG) _ux_utility_physical_address(ed));
        ohci_register =  _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_CONTROL);
        ohci_register |=  OHCI_HC_CR_BLE;
        _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_CONTROL, ohci_register);
        break;

    default:

        head_ed =  UX_NULL;
    }

    /* Build the back chaining pointer. The previous head ED needs to know about the
       inserted ED. */
    if (head_ed != UX_NULL)
    {
        head_ed = _ux_utility_virtual_address(head_ed);
        head_ed -> ux_ohci_ed_previous_ed =  ed;
    }

    /* Return successful completion.  */
    return(UX_SUCCESS);         
}

