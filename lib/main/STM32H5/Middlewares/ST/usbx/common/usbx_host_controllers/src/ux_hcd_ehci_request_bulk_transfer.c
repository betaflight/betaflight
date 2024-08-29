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
/*    _ux_hcd_ehci_request_bulk_transfer                  PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function performs a bulk transfer request. A bulk transfer    */ 
/*     can be larger than the size of the EHCI buffer so it may be        */ 
/*     required to chain multiple tds to accommodate this request. A bulk */ 
/*     transfer is non blocking, so we return before the request is       */
/*     completed.                                                         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ehci                              Pointer to EHCI controller    */ 
/*    transfer_request                      Pointer to transfer request   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ehci_request_transfer_add     Add transfer to ED            */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ehci_request_bulk_transfer(UX_HCD_EHCI *hcd_ehci, UX_TRANSFER *transfer_request)
{

UX_ENDPOINT     *endpoint;
UX_EHCI_ED      *ed;
ULONG           transfer_request_payload_length;
ULONG           bulk_packet_payload_length;
UCHAR *         data_pointer;
ULONG           pid;
ULONG           td_component;
UINT            status;
ULONG           zlp_flag;


    /* Get the pointer to the Endpoint.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

    /* Now get the physical ED attached to this endpoint.  */
    ed =  endpoint -> ux_endpoint_ed;

    /* The overlay parameters should be reset now.  */
    ed -> ux_ehci_ed_current_td =     UX_NULL;
    ed -> ux_ehci_ed_queue_element =  (UX_EHCI_TD *)UX_EHCI_TD_T;
    ed -> ux_ehci_ed_alternate_td =   (UX_EHCI_TD *)UX_EHCI_TD_T;
    ed -> ux_ehci_ed_state &=         UX_EHCI_QH_TOGGLE;
    ed -> ux_ehci_ed_bp0 =            UX_NULL;
    ed -> ux_ehci_ed_bp0 =            UX_NULL;
    ed -> ux_ehci_ed_bp1 =            UX_NULL;
    ed -> ux_ehci_ed_bp2 =            UX_NULL;
    ed -> ux_ehci_ed_bp3 =            UX_NULL;
    ed -> ux_ehci_ed_bp4 =            UX_NULL;

    /* It may take more than one TD if the transfer_request length is more than the
       maximum length for an EHCI TD (this is irrelevant of the MaxPacketSize value 
       in the endpoint descriptor). EHCI data payload has a maximum size of 16K.  */
    transfer_request_payload_length =  transfer_request -> ux_transfer_request_requested_length;
    data_pointer =  transfer_request -> ux_transfer_request_data_pointer;
    
    /* Check for ZLP condition.  */
    if (transfer_request_payload_length == 0)
        
        /* We have a zlp condition.  */
        zlp_flag = UX_TRUE;
    else
    
        /* We do not have a zlp.  */
        zlp_flag = UX_FALSE;
                
    /* Build all necessary TDs.  */
    while ((transfer_request_payload_length != 0) || zlp_flag == UX_TRUE)
    {

        /* Reset ZLP now.  */
        zlp_flag = UX_FALSE;        
      
        /* Check if we are exceeding the max payload. */
        if (transfer_request_payload_length > UX_EHCI_MAX_PAYLOAD)
            bulk_packet_payload_length =  UX_EHCI_MAX_PAYLOAD;
        else
            bulk_packet_payload_length =  transfer_request_payload_length;

        /* Add this transfer request to the ED.  */
        if ((transfer_request -> ux_transfer_request_type&UX_REQUEST_DIRECTION) == UX_REQUEST_IN)
            pid =  UX_EHCI_PID_IN;
        else            
            pid =  UX_EHCI_PID_OUT;

        status =  _ux_hcd_ehci_request_transfer_add(hcd_ehci, ed, 0, pid, 0,
                                    data_pointer, bulk_packet_payload_length, transfer_request);

        if (status != UX_SUCCESS)
            return(status);
            
        /* Adjust the data payload length and the data payload pointer.  */
        transfer_request_payload_length -=  bulk_packet_payload_length;
        data_pointer +=  bulk_packet_payload_length;
    }        

    /* Set the IOC bit in the last TD.  */
    ed -> ux_ehci_ed_last_td -> ux_ehci_td_control |=  UX_EHCI_TD_IOC;

    /* Ensure the IOC bit is set before activating the TD. This is necessary 
       for some processors that perform writes out of order as an optimization.  */
    UX_DATA_MEMORY_BARRIER

    /* Activate the first TD linked to the ED.  */
    td_component =  (ULONG) ed -> ux_ehci_ed_queue_element;
    td_component &=  ~UX_EHCI_TD_T;
    ed -> ux_ehci_ed_queue_element =  (UX_EHCI_TD *) td_component;

    /* Return successful completion.  */
    return(UX_SUCCESS);           
}

