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
/*    _ux_hcd_sim_host_request_bulk_transfer              PORTABLE C      */ 
/*                                                           6.1.3        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function performs a bulk transfer request. A bulk transfer    */ 
/*     can be larger than the size of the sim_host buffer so it may be    */ 
/*     required to chain multiple TDs to accommodate this transfer        */ 
/*     request. A bulk transfer is non blocking, so we return before the  */ 
/*     transfer request is completed.                                     */ 
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
/*    _ux_hcd_sim_host_regular_td_obtain    Obtain regular TD             */ 
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
/*  12-31-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed ZLP sending,          */
/*                                            resulting in version 6.1.3  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_sim_host_request_bulk_transfer(UX_HCD_SIM_HOST *hcd_sim_host, UX_TRANSFER *transfer_request)
{

UX_ENDPOINT             *endpoint;
UX_HCD_SIM_HOST_TD      *data_td;
UX_HCD_SIM_HOST_TD      *start_data_td;
UX_HCD_SIM_HOST_TD      *next_data_td;
UX_HCD_SIM_HOST_TD      *previous_td;
UX_HCD_SIM_HOST_TD      *tail_td;
UX_HCD_SIM_HOST_ED      *ed;
ULONG                   transfer_request_payload_length;
ULONG                   bulk_packet_payload_length;
UCHAR *                 data_pointer;
    

    /* Get the pointer to the Endpoint.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

    /* Now get the physical ED attached to this endpoint.  */
    ed =  endpoint -> ux_endpoint_ed;

    /* Use the TD pointer by ed -> tail for the first TD of this transfer
       and chain from this one on.  */
    data_td =  ed -> ux_sim_host_ed_tail_td;
    previous_td =  data_td;

    /* Reset the first obtained data TD in case there is a TD shortage while building 
       the list of TDs.  */
    start_data_td =  0;

    /* It may take more than one TD if the transfer_request length is more than the
       maximum length for a host simulator TD (this is irrelevant of the MaxPacketSize value 
       in the endpoint descriptor). Host simulator data payload has a maximum size of 4K.  */
    transfer_request_payload_length =  transfer_request -> ux_transfer_request_requested_length;
    data_pointer =  transfer_request -> ux_transfer_request_data_pointer;
    
    do
    {

        if (transfer_request_payload_length > UX_HCD_SIM_HOST_MAX_PAYLOAD)

            bulk_packet_payload_length =  UX_HCD_SIM_HOST_MAX_PAYLOAD;
        else

            bulk_packet_payload_length =  transfer_request_payload_length;

        /* Store the beginning of the buffer address in the TD.  */
        data_td -> ux_sim_host_td_buffer =  data_pointer;

        /* Update the length of the transfer for this TD.  */
        data_td -> ux_sim_host_td_length =  bulk_packet_payload_length;

        /* Attach the endpoint and transfer_request to the TD.  */
        data_td -> ux_sim_host_td_transfer_request =  transfer_request;
        data_td -> ux_sim_host_td_ed =  ed;

        /* Adjust the data payload length and the data payload pointer.  */
        transfer_request_payload_length -=  bulk_packet_payload_length;
        data_pointer +=  bulk_packet_payload_length;

        /* The direction of the transaction is set in the TD.  */
        if ((transfer_request -> ux_transfer_request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN)

            data_td -> ux_sim_host_td_direction =  UX_HCD_SIM_HOST_TD_IN;
        else
            data_td -> ux_sim_host_td_direction =  UX_HCD_SIM_HOST_TD_OUT;

        /* Mark the TD with the DATA phase.  */
        data_td -> ux_sim_host_td_status |=  UX_HCD_SIM_HOST_TD_DATA_PHASE;

        /* The Toggle value is in the ED.  */
        data_td -> ux_sim_host_td_toggle =  UX_HCD_SIM_HOST_TD_TOGGLE_FROM_ED;

        /* Check if there will be another transaction.  */
        if (transfer_request_payload_length != 0)
        {

            /* Get a new TD to hook this payload.  */
            data_td =  _ux_hcd_sim_host_regular_td_obtain(hcd_sim_host);
            if (data_td == UX_NULL)
            {

                /* If there was already a TD chain in progress, free it.  */
                if (start_data_td != UX_NULL)
                {

                    data_td =  start_data_td;
                    while (data_td)
                    {

                        next_data_td =  data_td -> ux_sim_host_td_next_td;
                        data_td -> ux_sim_host_td_status =  UX_UNUSED;
                        data_td =  next_data_td;
                    }
                }

                return(UX_NO_TD_AVAILABLE);
            }

            /* the first obtained TD in the chain has to be remembered.  */
            if (start_data_td == UX_NULL)
                start_data_td =  data_td;

            /* Attach this new TD to the previous one.  */                                
            previous_td -> ux_sim_host_td_next_td =  data_td;
            previous_td -> ux_sim_host_td_next_td_transfer_request =  data_td;
            previous_td =  data_td;
        }
    } while (transfer_request_payload_length != 0);
        
    /* At this stage, the Head and Tail in the ED are still the same and the host simulator 
       controller will skip this ED until we have hooked the new tail TD.  */
    tail_td =  _ux_hcd_sim_host_regular_td_obtain(hcd_sim_host);
    if (tail_td == UX_NULL)
    {

        /* If there was already a TD chain in progress, free it.  */
        if (start_data_td != UX_NULL)
        {
 
            data_td =  start_data_td;
            while (data_td)
            {

                next_data_td =  data_td -> ux_sim_host_td_next_td;
                data_td -> ux_sim_host_td_status =  UX_UNUSED;
                data_td =  next_data_td;
            }
        }

        return(UX_NO_TD_AVAILABLE);
    }

    /* Attach the tail TD to the last data TD.  */
    data_td -> ux_sim_host_td_next_td =  tail_td;

    /* Store the new tail TD.  */
    ed -> ux_sim_host_ed_tail_td =  tail_td;

    /* Now we can tell the scheduler to wake up.  */
    hcd_sim_host -> ux_hcd_sim_host_queue_empty =  UX_FALSE;

    /* Return successful completion.  */
    return(UX_SUCCESS);           
}

