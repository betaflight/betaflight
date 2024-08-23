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
/*    _ux_hcd_ohci_request_isochronous_transfer           PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function performs an isochronous transfer request. This       */ 
/*     function does not support multiple packets per TD as there can be  */
/*     issues with packets crossing 4K pages.                             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ohci                              Pointer to OHCI controller    */ 
/*    transfer_request                      Pointer to transfer request   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ohci_isochronous_td_obtain    Get isochronous TD            */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ohci_request_isochronous_transfer(UX_HCD_OHCI *hcd_ohci, UX_TRANSFER *transfer_request)
{

UX_ENDPOINT         *endpoint;
UX_OHCI_ISO_TD      *data_td;
UX_OHCI_ISO_TD      *start_data_td;
UX_OHCI_ISO_TD      *next_data_td;
UX_OHCI_ISO_TD      *previous_td;
UX_OHCI_ISO_TD      *tail_td;
UX_OHCI_ED          *ed;
ULONG               transfer_request_payload_length;
ULONG               isoch_packet_payload_length;
UCHAR *             data_pointer;
ULONG               current_frame_number;
    

    /* Get the pointer to the Endpoint.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

    /* Now get the physical ED attached to this endpoint.  */
    ed =  endpoint -> ux_endpoint_ed;

    /* Reset the MPS value of the ED.  */
    ed -> ux_ohci_ed_dw0 &=  UX_OHCI_ED_MPS;
    
    /* If the transfer request specifies a max packet length other than the endpoint
       size, we force the transfer request value into the endpoint.  */
    if (transfer_request -> ux_transfer_request_packet_length == 0)
        transfer_request -> ux_transfer_request_packet_length =  (ULONG) endpoint -> ux_endpoint_descriptor.wMaxPacketSize;

    /* Set the packet length in the ED.  */
    ed -> ux_ohci_ed_dw0 |=  transfer_request -> ux_transfer_request_packet_length << 16;
    isoch_packet_payload_length =  transfer_request -> ux_transfer_request_packet_length;

    /* Use the TD pointer by ed -> tail for the first TD of this transfer
        and chain from this one on.  */
    data_td =  _ux_utility_virtual_address(ed -> ux_ohci_ed_tail_td);
    previous_td =  data_td;

    /* Reset the first obtained data TD in case there is a TD shortage while building the list of TDs.  */
    start_data_td =  UX_NULL;

    /* Calculate the frame number to be used to send this payload. If there are no current transfers, 
       we take the current frame number and add a safety value (2-5) to it. If here is pending transactions,
       we use the frame number stored in the transfer request.  */
    if (ed -> ux_ohci_ed_tail_td == ed -> ux_ohci_ed_head_td)
    {

        current_frame_number =  hcd_ohci -> ux_hcd_ohci_hcca -> ux_hcd_ohci_hcca_frame_number + UX_OHCI_FRAME_DELAY;
        ed -> ux_ohci_ed_frame =  current_frame_number;
    }
    else
        current_frame_number =  ed -> ux_ohci_ed_frame;

    /* Load the start buffer address and URB length to split the URB in multiple TD transfer.  */
    transfer_request_payload_length =  transfer_request -> ux_transfer_request_requested_length;
    data_pointer =  transfer_request -> ux_transfer_request_data_pointer;
    
    while (transfer_request_payload_length != 0)
    {

        /* Set the default CC value. Default is 1 packet per TD frame.  */
        data_td -> ux_ohci_iso_td_dw0 =  UX_OHCI_TD_DEFAULT_DW0;
        
        /* Set the direction. In Iso, this is done at the ED level.  */
        if ((transfer_request -> ux_transfer_request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN)

            ed -> ux_ohci_ed_dw0 |=  UX_OHCI_ED_IN;
        else
            ed -> ux_ohci_ed_dw0 |=  UX_OHCI_ED_OUT;

        /* Set the frame number.  */
        data_td -> ux_ohci_iso_td_dw0 |=  (USHORT) current_frame_number;

        /* The buffer address is divided into a page and an offset.  */
        data_td -> ux_ohci_iso_td_bp0 =  _ux_utility_physical_address((VOID *)((ULONG) data_pointer & UX_OHCI_ISO_TD_BASE));
        data_td -> ux_ohci_iso_td_offset_psw[0] =  (USHORT)((ULONG) _ux_utility_physical_address(data_pointer) & UX_OHCI_ISO_TD_OFFSET);

        /* Set the condition code for the current packet.  */
        data_td -> ux_ohci_iso_td_offset_psw[0] |=  (USHORT) UX_OHCI_ISO_TD_PSW_CC;
        
        /* Program the end address of the buffer.  */
        data_td -> ux_ohci_iso_td_be =  ((UCHAR *) _ux_utility_physical_address(data_pointer)) + isoch_packet_payload_length - 1;

        /* Update the length of the transfer for this TD.  */
        data_td -> ux_ohci_iso_td_length =  isoch_packet_payload_length;

        /* Attach the endpoint and transfer request to the TD.  */
        data_td -> ux_ohci_iso_td_transfer_request =  transfer_request;
        data_td -> ux_ohci_iso_td_ed =  ed;

        /* Adjust the data payload length and the data payload pointer.  */
        transfer_request_payload_length -=  isoch_packet_payload_length;
        data_pointer +=  isoch_packet_payload_length;

        /* Prepare the next frame for the next TD in advance.  */
        current_frame_number++;

        /* Check if there will be another transaction.  */
        if (transfer_request_payload_length != 0)
        {

            /* Get a new TD to hook this payload.  */
            data_td =  _ux_hcd_ohci_isochronous_td_obtain(hcd_ohci);
            if (data_td == UX_NULL)
            {

                /* If there was already a TD chain in progress, free it.  */
                if (start_data_td != UX_NULL)
                {

                    data_td =  start_data_td;
                    while(data_td)
                    {

                        next_data_td =  _ux_utility_virtual_address(data_td -> ux_ohci_iso_td_next_td);
                        data_td -> ux_ohci_iso_td_status =  UX_UNUSED;
                        data_td =  next_data_td;
                    }
                }

                return(UX_NO_TD_AVAILABLE);
            }

            /* the first obtained TD in the chain has to be remembered.  */
            if (start_data_td == UX_NULL)
                start_data_td =  data_td;

            /* Attach this new TD to the previous one.  */                                
            previous_td -> ux_ohci_iso_td_next_td =  _ux_utility_physical_address(data_td);
            previous_td =  data_td;
        }
    }
        
    /* Memorize the next frame number for this ED.  */
    ed -> ux_ohci_ed_frame =  current_frame_number;

    /* At this stage, the Head and Tail in the ED are still the same and
       the OHCI controller will skip this ED until we have hooked the new
       tail TD.  */
    tail_td =  _ux_hcd_ohci_isochronous_td_obtain(hcd_ohci);
    if (tail_td == UX_NULL)
    {

        /* If there was already a TD chain in progress, free it.  */
        if (start_data_td != UX_NULL)
        {

            data_td =  start_data_td;
            while (data_td)
            {

                next_data_td =  _ux_utility_virtual_address(data_td -> ux_ohci_iso_td_next_td);
                data_td -> ux_ohci_iso_td_status =  UX_UNUSED;
                data_td =  next_data_td;
            }
        }

        return(UX_NO_TD_AVAILABLE);
    }

    /* Attach the tail TD to the last data TD.  */
    data_td -> ux_ohci_iso_td_next_td =  _ux_utility_physical_address(tail_td);

    /* Adjust the ED tail pointer, the controller can now start this transfer
       at the chosen frame number.  */
    ed -> ux_ohci_ed_tail_td =  _ux_utility_physical_address(tail_td);

    /* Return successful completion.  */
    return(UX_SUCCESS);           
}

