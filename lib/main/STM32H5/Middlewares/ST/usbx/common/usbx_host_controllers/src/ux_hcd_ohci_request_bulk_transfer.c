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
/*    _ux_hcd_ohci_request_bulk_transfer                  PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function performs a bulk transfer request. A bulk transfer    */ 
/*     can be larger than the size of the OHCI buffer so it may be        */ 
/*     required to chain multiple tds to accommodate this transfer        */
/*     request. A bulk transfer is non blocking, so we return before the  */ 
/*     transfer request is completed.                                     */ 
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
/*    _ux_hcd_ohci_register_read            Read OHCI register            */ 
/*    _ux_hcd_ohci_register_write           Write OHCI register           */ 
/*    _ux_hcd_ohci_regular_td_obtain        Get regular TD                */ 
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
UINT  _ux_hcd_ohci_request_bulk_transfer(UX_HCD_OHCI *hcd_ohci, UX_TRANSFER *transfer_request)
{

UX_ENDPOINT     *endpoint;
UX_OHCI_TD      *data_td;
UX_OHCI_TD      *start_data_td;
UX_OHCI_TD      *next_data_td;
UX_OHCI_TD      *previous_td;
UX_OHCI_TD      *tail_td;
UX_OHCI_ED      *ed;
ULONG           transfer_request_payload_length;
ULONG           bulk_packet_payload_length;
UCHAR *         data_pointer;
ULONG           ohci_register;
ULONG           zlp_flag;
    

    /* Get the pointer to the Endpoint.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

    /* Now get the physical ED attached to this endpoint.  */
    ed =  endpoint -> ux_endpoint_ed;

    /* Use the TD pointer by ed -> tail for the first TD of this transfer
        and chain from this one on.  */
    data_td =  _ux_utility_virtual_address(ed -> ux_ohci_ed_tail_td);
    previous_td =  data_td;

    /* Reset the first obtained data TD in case there is a TD shortage while building the list of tds.  */
    start_data_td =  0;

    /* It may take more than one TD if the transfer_request length is more than the
       maximum length for a OHCI TD (this is irrelevant of the MaxPacketSize value in 
       the endpoint descriptor). OHCI data payload has a maximum size of 4K.  */
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
        if (transfer_request_payload_length > UX_OHCI_MAX_PAYLOAD)

            bulk_packet_payload_length =  UX_OHCI_MAX_PAYLOAD;
        else

            bulk_packet_payload_length =  transfer_request_payload_length;

        /* IN transfer ? */
        if ((transfer_request -> ux_transfer_request_type&UX_REQUEST_DIRECTION) == UX_REQUEST_IN)

            data_td -> ux_ohci_td_dw0 =  UX_OHCI_TD_IN | UX_OHCI_TD_DEFAULT_DW0;
        else

            data_td -> ux_ohci_td_dw0 =  UX_OHCI_TD_OUT | UX_OHCI_TD_DEFAULT_DW0;

        /* Store the beginning of the buffer address in the TD.  */
        data_td -> ux_ohci_td_cbp =  _ux_utility_physical_address(data_pointer);

        /* Store the end buffer address in the TD.  */
        data_td -> ux_ohci_td_be =  data_td -> ux_ohci_td_cbp + bulk_packet_payload_length - 1;

        /* Update the length of the transfer for this TD.  */
        data_td -> ux_ohci_td_length =  bulk_packet_payload_length;

        /* Attach the endpoint and transfer request to the TD.  */
        data_td -> ux_ohci_td_transfer_request =  transfer_request;
        data_td -> ux_ohci_td_ed =  ed;

        /* Adjust the data payload length and the data payload pointer.  */
        transfer_request_payload_length -=  bulk_packet_payload_length;
        data_pointer +=  bulk_packet_payload_length;

        /* Check if there will be another transaction.  */
        if (transfer_request_payload_length != 0)
        {

            /* Get a new TD to hook this payload.  */
            data_td =  _ux_hcd_ohci_regular_td_obtain(hcd_ohci);

            if (data_td == UX_NULL)
            {

                /* If there was already a TD chain in progress, free it.  */
                if (start_data_td != UX_NULL)
                {

                    data_td =  start_data_td;
                    while(data_td)
                    {

                        next_data_td =  _ux_utility_virtual_address(data_td -> ux_ohci_td_next_td);
                        data_td -> ux_ohci_td_status =  UX_UNUSED;
                        data_td =  next_data_td;
                    }
                }

                return(UX_NO_TD_AVAILABLE);
            }

            /* the first obtained TD in the chain has to be remembered.  */
            if (start_data_td == UX_NULL)
                start_data_td =  data_td;

            /* Attach this new TD to the previous one.  */                                
            previous_td -> ux_ohci_td_next_td =  _ux_utility_physical_address(data_td);
            previous_td -> ux_ohci_td_next_td_transfer_request =  data_td;
            previous_td =  data_td;
        }
    }
        
    /* At this stage, the Head and Tail in the ED are still the same and the OHCI controller 
       will skip this ED until we have hooked the new tail TD.  */
    tail_td =  _ux_hcd_ohci_regular_td_obtain(hcd_ohci);
    if (tail_td == UX_NULL)
    {
     
        /* If there was already a TD chain in progress, free it.  */
        if (start_data_td != UX_NULL)
        {

            data_td =  start_data_td;
            while(data_td)
            {

                next_data_td =  _ux_utility_virtual_address(data_td -> ux_ohci_td_next_td);
                data_td -> ux_ohci_td_status =  UX_UNUSED;
                data_td =  next_data_td;
            }
        }

        return(UX_NO_TD_AVAILABLE);
    }

    /* Attach the tail TD to the last data TD.  */
    data_td -> ux_ohci_td_next_td =  _ux_utility_physical_address(tail_td);

    /* Store the new tail TD.  */
    ed -> ux_ohci_ed_tail_td =  _ux_utility_physical_address(tail_td);

    /* Now, we must tell the OHCI controller that there is something in the
       bulk queue.  */
    ohci_register =  _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_COMMAND_STATUS);
    ohci_register |=  OHCI_HC_CS_BLF;
    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_COMMAND_STATUS, ohci_register);

    /* Return successful completion.  */
    return(UX_SUCCESS);           
}

