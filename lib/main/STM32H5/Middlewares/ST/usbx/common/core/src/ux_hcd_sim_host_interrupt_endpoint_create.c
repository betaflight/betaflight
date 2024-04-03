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
/*    _ux_hcd_sim_host_interrupt_endpoint_create          PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will create an interrupt endpoint. The interrupt      */ 
/*    endpoint has an interval of operation from 1 to 255. The host       */
/*    has no hardware scheduler but we still build an interrupt tree      */ 
/*    similar to the host simulator controller.                           */
/*                                                                        */
/*    This routine will match the best interval for the host              */ 
/*    simulator. It will also determine the best node to hook the         */ 
/*    endpoint based on the load that already exists on the horizontal    */ 
/*    ED chain.                                                           */
/*                                                                        */
/*    For the ones curious about this coding. The tricky part is to       */
/*    understand how the interrupt matrix is constructed. We have used    */ 
/*    EDs with the skip bit on to build a frame of anchor EDs. Each ED    */ 
/*    creates a node for an appropriate combination of interval frequency */ 
/*    in the list.                                                        */ 
/*                                                                        */
/*    After obtaining a pointer to the list with the lowest traffic, we   */
/*    traverse the list from the highest interval until we reach the      */ 
/*    interval required. At that node, we anchor our real ED to the node  */ 
/*    and link the ED that was attached to the node to our ED.            */ 
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
/*    _ux_hcd_sim_host_ed_obtain              Obtain ED                   */ 
/*    _ux_hcd_sim_host_regular_td_obtain      Obtain regular TD           */ 
/*    _ux_hcd_sim_host_least_traffic_list_get Get least traffic list      */ 
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
UINT  _ux_hcd_sim_host_interrupt_endpoint_create(UX_HCD_SIM_HOST *hcd_sim_host, UX_ENDPOINT *endpoint)
{

UX_HCD_SIM_HOST_ED      *ed;
UX_HCD_SIM_HOST_ED      *ed_list;
UX_HCD_SIM_HOST_ED      *next_ed;
UX_HCD_SIM_HOST_TD      *td;
UINT                    interval;
UINT                    interval_index;
UINT                    interval_sim_host;


    /* Obtain a ED for this new endpoint. This ED will live as long as
       the endpoint is active and will be the container for the TDs.  */
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
    endpoint -> ux_endpoint_ed =  (VOID *)ed;
    
    /* Now do the opposite, attach the ED container to the physical ED.  */
    ed -> ux_sim_host_ed_endpoint =  endpoint;

    /* Hook the TD to both the tail and head of the ED.  */
    ed -> ux_sim_host_ed_tail_td =  td;
    ed -> ux_sim_host_ed_head_td =  td;

    /* Get the list index with the least traffic.  */
    ed_list =  _ux_hcd_sim_host_least_traffic_list_get(hcd_sim_host);
    
    /* Get the interval for the endpoint and match it to a host simulator list. We match anything 
       that is > 32ms to the 32ms interval list, the 32ms list is list 0, 16ms list is 1...
       the 1ms list is number 5.  */
    interval =          endpoint -> ux_endpoint_descriptor.bInterval;
    interval_index =    1;
    interval_sim_host =  5;
    if (interval >= 32)
    {

        interval_sim_host =  0;
    }
    else
    {

        while (interval_index < 32)
        {
 
            if (interval&interval_index)
                interval_sim_host--;
            interval_index =  interval_index << 1;
        }
    }

    /* Now we need to scan the list of eds from the lowest load entry until we reach 
       the appropriate interval node. The depth index is the interval_sim_host value 
       and the 1st entry is pointed by the ED list entry.  */
    while (interval_sim_host--)
    {

        ed_list =  ed_list -> ux_sim_host_ed_next_ed;
        while (!(ed_list -> ux_sim_host_ed_status & UX_HCD_SIM_HOST_ED_STATIC))
            ed_list =  ed_list -> ux_sim_host_ed_next_ed;
    }       

    /* We found the node entry of the ED pointer that will be the anchor for this interrupt 
       endpoint. Now we attach this endpoint to the anchor and rebuild the chain .  */
    next_ed =                                ed_list -> ux_sim_host_ed_next_ed;
    /* Note that if there is a crash here, it is most likely due to an invalid bInterval.  */
    next_ed -> ux_sim_host_ed_previous_ed =  ed;
    ed -> ux_sim_host_ed_next_ed =           next_ed;
    ed -> ux_sim_host_ed_previous_ed =       ed_list;
    ed_list -> ux_sim_host_ed_next_ed =      ed;

    /* There is activity in the periodic tree, the scheduler has to be active all the time.  */
    hcd_sim_host -> ux_hcd_sim_host_periodic_scheduler_active++;

    /* Return successful completion.  */
    return(UX_SUCCESS);         
}

