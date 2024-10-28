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
/*    _ux_hcd_sim_host_ed_obtain                          PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function obtains a free ED from the ED list.                   */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_sim_host                          Pointer to host controller    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    UX_HCD_SIM_HOST_ED *                  Pointer to ED                 */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_set                Set memory block              */ 
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
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UX_HCD_SIM_HOST_ED  *_ux_hcd_sim_host_ed_obtain(UX_HCD_SIM_HOST *hcd_sim_host)
{

UX_HCD_SIM_HOST_ED      *ed;
ULONG                   ed_index;


    /* Start the search from the beginning of the list.  */
    ed =  hcd_sim_host -> ux_hcd_sim_host_ed_list;
    for(ed_index = 0; ed_index < _ux_system_host -> ux_system_host_max_ed; ed_index++)
    {

        /* Check the ED status, a free ED is marked with the UNUSED flag.  */
        if (ed -> ux_sim_host_ed_status == UX_UNUSED)
        {

            /* The ED may have been used, so we reset all fields.  */
            _ux_utility_memory_set(ed, 0, sizeof(UX_HCD_SIM_HOST_ED)); /* Use case of memset is verified. */

            /* This ED is now marked as USED.  */
            ed -> ux_sim_host_ed_status =  UX_USED;

            /* Return ED pointer.  */
            return(ed);
        }

        /* Point to the next ED.  */
        ed++;
    }

    /* There is no available ED in the ED list.  */
    return(UX_NULL);
}

