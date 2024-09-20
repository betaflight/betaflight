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
/*    _ux_hcd_sim_host_asynchronous_endpoint_create       PORTABLE C      */ 
/*                                                           6.1          */
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
/*    hcd_sim_host                          Pointer to host controller    */ 
/*    endpoint                              Pointer to endpoint           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_sim_host_ed_obtain            Obtain host ED                */ 
/*    _ux_hcd_sim_host_regular_td_obtain    Obtain host regular TD        */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_sim_host_asynchronous_endpoint_create(UX_HCD_SIM_HOST *hcd_sim_host, UX_ENDPOINT *endpoint)
{

UX_HCD_SIM_HOST_ED      *ed;
UX_HCD_SIM_HOST_ED      *head_ed;
UX_HCD_SIM_HOST_TD      *td;


    /* We need to take into account the nature of the HCD to define the max size
       of any transfer in the transfer request.  */
    endpoint -> ux_endpoint_transfer_request.ux_transfer_request_maximum_length =  UX_HCD_SIM_HOST_MAX_PAYLOAD;
    
    /* Obtain a ED for this new endpoint. This ED will live as long as the endpoint is active 
       and will be the container for the TDs.  */
    ed =  _ux_hcd_sim_host_ed_obtain(hcd_sim_host);
    if (ed == UX_NULL)
        return(UX_NO_ED_AVAILABLE);

    /* Obtain a dummy TD for terminating the ED transfer chain.  */
    td =  _ux_hcd_sim_host_regular_td_obtain(hcd_sim_host);
    if (td == UX_NULL)
    {

        ed -> ux_sim_host_ed_status =  UX_UNUSED;
        return(UX_NO_TD_AVAILABLE);
    }

    /* Attach the ED to the endpoint container.  */
    endpoint -> ux_endpoint_ed =  (VOID *) ed;

    /* Now do the opposite, attach the ED container to the physical ED.  */
    ed -> ux_sim_host_ed_endpoint =  endpoint;
    
    /* Hook the TD to both the tail and head of the ED.  */
    ed -> ux_sim_host_ed_tail_td =  td;
    ed -> ux_sim_host_ed_head_td =  td;
    
    /* Attach this ED to the asynch list.  */
    head_ed =  hcd_sim_host -> ux_hcd_sim_host_asynch_head_ed;
    ed -> ux_sim_host_ed_next_ed =  head_ed;
    hcd_sim_host -> ux_hcd_sim_host_asynch_head_ed =  ed;

    /* Build the back chaining pointer. The previous head ED needs to know about the
       inserted ED. */
    if (head_ed != UX_NULL)
        head_ed -> ux_sim_host_ed_previous_ed =  ed;

    /* Return successful completion.  */
    return(UX_SUCCESS);         
}

