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
/**   Host Simulator Controller Driver                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_hcd_sim_host.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_hcd_sim_host_request_isochronous_transfer       PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function performs an isochronous transfer request.            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_sim_host                          Pointer to host controller    */ 
/*    transfer_request                      Pointer to transfer request   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_sim_host_frame_number_get       Get frame number            */ 
/*    _ux_hcd_sim_host_isochronous_td_obtain  Obtain isochronous TD       */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Host Simulator Controller Driver                                    */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed payload calculation,  */
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed partial transfer,     */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_sim_host_request_isochronous_transfer(UX_HCD_SIM_HOST *hcd_sim_host, UX_TRANSFER *transfer_request)
{
                        
UX_ENDPOINT                 *endpoint;
UX_HCD_SIM_HOST_ISO_TD      *data_td;
UX_HCD_SIM_HOST_ISO_TD      *start_data_td;
UX_HCD_SIM_HOST_ISO_TD      *next_data_td;
UX_HCD_SIM_HOST_ISO_TD      *previous_td;
UX_HCD_SIM_HOST_ISO_TD      *tail_td;
UX_HCD_SIM_HOST_ED          *ed;
ULONG                       transfer_request_payload_length;
ULONG                       isoch_packet_payload_length;
UCHAR *                     data_pointer;
ULONG                       current_frame_number;
ULONG                       n_trans, packet_size;
    

    /* Get the pointer to the Endpoint.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

    /* Now get the physical ED attached to this endpoint.  */
    ed =  endpoint -> ux_endpoint_ed;

    /* If the transfer_request specifies a max packet length other than the endpoint
       size, we force the transfer request value into the endpoint.  */
    if (transfer_request -> ux_transfer_request_packet_length == 0)
    {

        /* For wMaxPacketSize, bits 10..0 specify the maximum packet size (max 1024),
           bits 12..11 specify the number of additional transactions (max 2),
           the calculation below will not cause overflow using 32-bit operation.
           Note wMaxPacketSize validation has been done in ux_host_stack_new_endpoint_create.c,
           before endpoint creation, so the value can be directly used here.  */
        packet_size = endpoint -> ux_endpoint_descriptor.wMaxPacketSize & UX_MAX_PACKET_SIZE_MASK;
        n_trans = endpoint -> ux_endpoint_descriptor.wMaxPacketSize & UX_MAX_NUMBER_OF_TRANSACTIONS_MASK;
        if (n_trans)
        {
            n_trans >>= UX_MAX_NUMBER_OF_TRANSACTIONS_SHIFT;
            n_trans ++;
            packet_size *= n_trans;
        }
        transfer_request -> ux_transfer_request_packet_length = packet_size;
    }

    /* Remember the packet length.  */
    isoch_packet_payload_length =  transfer_request -> ux_transfer_request_packet_length;

    /* Use the TD pointer by ed -> tail for the first TD of this transfer and chain from this one on.  */
    data_td =  (UX_HCD_SIM_HOST_ISO_TD *) ((void *) ed -> ux_sim_host_ed_tail_td);
    previous_td =  data_td;

    /* Reset the first obtained data TD in case there is a TD shortage while building the list of TDs.  */
    start_data_td =  UX_NULL;

    /* Calculate the frame number to be used to send this payload. If there are no current transfers, 
       we take the current frame number and add a safety value (2-5) to it. If here is pending transactions,
       we use the frame number stored in the transfer request.  */
    if (ed -> ux_sim_host_ed_tail_td == ed -> ux_sim_host_ed_head_td)
    {

        _ux_hcd_sim_host_frame_number_get(hcd_sim_host, &current_frame_number);
        ed -> ux_sim_host_ed_frame =  current_frame_number + UX_HCD_SIM_HOST_FRAME_DELAY;
    }
    else
    {

        current_frame_number =  ed -> ux_sim_host_ed_frame;
    }

    /* Load the start buffer address and URB length to split the URB in multiple TD transfer.  */
    transfer_request_payload_length =  transfer_request -> ux_transfer_request_requested_length;
    data_pointer =  transfer_request -> ux_transfer_request_data_pointer;
    
    while (transfer_request_payload_length != 0)
    {

        /* Set the direction of the TD.  */
        if ((transfer_request -> ux_transfer_request_type&UX_REQUEST_DIRECTION) == UX_REQUEST_IN)

            data_td -> ux_sim_host_iso_td_direction =  UX_HCD_SIM_HOST_TD_IN;
        else

            data_td -> ux_sim_host_iso_td_direction =  UX_HCD_SIM_HOST_TD_OUT;

        /* Mark the TD with the DATA phase.  */
        data_td -> ux_sim_host_iso_td_status |=  UX_HCD_SIM_HOST_TD_DATA_PHASE;

        /* Set the frame number.  */
        ed -> ux_sim_host_ed_frame =  current_frame_number;

        /* Set the buffer address.  */
        data_td -> ux_sim_host_iso_td_buffer =  data_pointer;

        /* Update the length of the transfer for this TD.  */
        data_td -> ux_sim_host_iso_td_length = UX_MIN(transfer_request_payload_length, isoch_packet_payload_length);

        /* Attach the endpoint and transfer request to the TD.  */
        data_td -> ux_sim_host_iso_td_transfer_request =  transfer_request;
        data_td -> ux_sim_host_iso_td_ed =  ed;

        /* Adjust the data payload length and the data payload pointer.  */
        transfer_request_payload_length -= data_td -> ux_sim_host_iso_td_length;
        data_pointer += data_td -> ux_sim_host_iso_td_length;

        /* Prepare the next frame for the next TD in advance.  */
        current_frame_number++;

        /* Check if there will be another transaction.  */
        if (transfer_request_payload_length != 0)
        {

            /* Get a new TD to hook this payload.  */
            data_td =  _ux_hcd_sim_host_isochronous_td_obtain(hcd_sim_host);
            if (data_td == UX_NULL)
            {

                /* If there was already a TD chain in progress, free it.  */
                if (start_data_td != UX_NULL)
                {

                    data_td =  start_data_td;
                    while(data_td)
                    {

                        next_data_td =  data_td -> ux_sim_host_iso_td_next_td;
                        data_td -> ux_sim_host_iso_td_status =  UX_UNUSED;
                        data_td =  next_data_td;
                    }
                }

                return(UX_NO_TD_AVAILABLE);
            }

            /* the first obtained TD in the chain has to be remembered.  */
            if (start_data_td == UX_NULL)
                start_data_td =  data_td;

            /* Attach this new TD to the previous one.  */                                
            previous_td -> ux_sim_host_iso_td_next_td =  data_td;
            previous_td =  data_td;
        }
    }
        
    /* Memorize the next frame number for this ED.  */
    ed -> ux_sim_host_ed_frame =  current_frame_number;

    /* At this stage, the Head and Tail in the ED are still the same and the host simulator controller 
       will skip this ED until we have hooked the new tail TD.  */
    tail_td =  _ux_hcd_sim_host_isochronous_td_obtain(hcd_sim_host);
    if (tail_td == UX_NULL)
    {

        /* If there was already a TD chain in progress, free it.  */
        if (start_data_td != UX_NULL)
        {

            data_td =  start_data_td;
            while(data_td)
            {

                next_data_td =  data_td -> ux_sim_host_iso_td_next_td;
                data_td -> ux_sim_host_iso_td_status =  UX_UNUSED;
                data_td =  next_data_td;
            }
        }

        return(UX_NO_TD_AVAILABLE);
    }

    /* Attach the tail TD to the last data TD.  */
    data_td -> ux_sim_host_iso_td_next_td =  tail_td;

    /* Adjust the ED tail pointer, the controller can now start this transfer
       at the chosen frame number.  */
    ed -> ux_sim_host_ed_tail_td =  (UX_HCD_SIM_HOST_TD *) ((void *) tail_td);

    /* Return successful completion.  */
    return(UX_SUCCESS);           
}

