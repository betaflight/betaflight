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
/*    _ux_hcd_sim_host_least_traffic_list_get             PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function return a pointer to the first ED in the periodic     */ 
/*     tree that has the least traffic registered.                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_sim_host                          Pointer to host controller    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    UX_HCD_SIM_HOST_ED *                  Pointer to host ED            */ 
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
UX_HCD_SIM_HOST_ED  *_ux_hcd_sim_host_least_traffic_list_get(UX_HCD_SIM_HOST *hcd_sim_host)
{
                    
UX_HCD_SIM_HOST_ED      *min_bandwidth_ed;
UX_HCD_SIM_HOST_ED      *begin_ed;
UX_HCD_SIM_HOST_ED      *ed;
UX_ENDPOINT             *endpoint;
UINT                    list_index;
ULONG                   min_bandwidth_used;
ULONG                   bandwidth_used;


    /* Reset the min bandwidth used.  */
    min_bandwidth_used =  0;

    /* The first ED is the list candidate for now.  */
    min_bandwidth_ed =  hcd_sim_host -> ux_hcd_sim_host_interrupt_ed_list[0];
    
    /* All list will be scanned.  */
    for (list_index = 0; list_index < 32; list_index++)
    {

        /* Reset the bandwidth for this list.  */
        bandwidth_used =  0;

        /* Get the ED of the beginning of the list we parse now.  */
        ed =  hcd_sim_host -> ux_hcd_sim_host_interrupt_ed_list[list_index];

        /* We keep track of the first ED for the current list.  */
        begin_ed =  ed;

        /* Parse the EDs in the list.  */
        while (ed -> ux_sim_host_ed_next_ed != UX_NULL)
        {

            /* Get the endpoint pointer from the physical ED.  */
            endpoint =  ed -> ux_sim_host_ed_endpoint;

            if (endpoint != UX_NULL)
            {
            
                /* Add to the bandwidth used the max packet size pointed by this ED.  */
                bandwidth_used +=  (ULONG) endpoint -> ux_endpoint_descriptor.wMaxPacketSize;
            }

            /* Move to next ED.  */           
            ed =  ed -> ux_sim_host_ed_next_ed;
        }

        /* We have processed a list, check the bandwidth used by this list.
           If this bandwidth is the minimum, we memorize the ED.  */        
        if (bandwidth_used < min_bandwidth_used)
        {

            /* We have found a better list with a lower used bandwidth, memorize the bandwidth 
               for this list.  */
            min_bandwidth_used =  bandwidth_used;
            
            /* Memorize the begin ED for this list.  */
            min_bandwidth_ed =  begin_ed;
        }
    }
    
    /* Return the ED list with the lowest bandwidth.  */
    return(min_bandwidth_ed);   
}

