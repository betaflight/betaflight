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
/*    _ux_hcd_sim_host_asynchronous_endpoint_destroy      PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will destroy an asynchronous endpoint. The control    */
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
/*    None                                                                */ 
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
UINT  _ux_hcd_sim_host_asynchronous_endpoint_destroy(UX_HCD_SIM_HOST *hcd_sim_host, UX_ENDPOINT *endpoint)
{

UX_HCD_SIM_HOST_ED      *ed;
UX_HCD_SIM_HOST_ED      *previous_ed;
UX_HCD_SIM_HOST_ED      *next_ed;
UX_HCD_SIM_HOST_TD      *td;

    
    /* From the endpoint container fetch the host simulator ED descriptor.  */
    ed =  (UX_HCD_SIM_HOST_ED *) endpoint -> ux_endpoint_ed;

    /* Get the previous ED in the list for this ED.  */
    previous_ed =  ed -> ux_sim_host_ed_previous_ed;

    /* Get the next ED in the list for this ED.  */
    next_ed =  ed -> ux_sim_host_ed_next_ed;

    /* If the previous ED is NULL, we are at trying to remove the head ED.  */
    if (previous_ed == UX_NULL)

        /* Store the new endpoint in the control list.  */
        hcd_sim_host -> ux_hcd_sim_host_asynch_head_ed =  next_ed;
    else

        /* The previous ED points now to the ED after the ED we are removing.  */
        previous_ed -> ux_sim_host_ed_next_ed =  next_ed;

    /* Update the previous ED pointer in the next ED if exists.  */
    if (next_ed != UX_NULL)
        next_ed -> ux_sim_host_ed_previous_ed =  previous_ed;
        
    /* Determine if we need to adjust the current ED.  */
    if (hcd_sim_host -> ux_hcd_sim_host_asynch_current_ed == ed)
    {
    
        /* Move the current to the next.  */
        hcd_sim_host -> ux_hcd_sim_host_asynch_current_ed =  next_ed;
    }

    /* We need to free the dummy TD that was attached to the ED.  */
    td =  ed -> ux_sim_host_ed_tail_td;
    td -> ux_sim_host_td_status =  UX_UNUSED;

    /* Now we can safely make the ED free.  */
    ed -> ux_sim_host_ed_status =  UX_UNUSED;

    /* Return successful completion.  */
    return(UX_SUCCESS);         
}

