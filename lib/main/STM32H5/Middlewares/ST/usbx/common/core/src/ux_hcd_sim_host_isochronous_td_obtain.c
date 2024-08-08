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
/*    _ux_hcd_sim_host_isochronous_td_obtain              PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function obtains a free TD from the isochronous TD list.      */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_sim_host                          Pointer to host controller    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    UX_HCD_SIM_HOST_ISO_TD *              Pointer to host ISO ED        */ 
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
UX_HCD_SIM_HOST_ISO_TD  *_ux_hcd_sim_host_isochronous_td_obtain(UX_HCD_SIM_HOST *hcd_sim_host)
{

UX_HCD_SIM_HOST_ISO_TD      *td;
ULONG                       td_index;


    /* Start the search from the beginning of the list.  */
    td =  hcd_sim_host -> ux_hcd_sim_host_iso_td_list;
    for (td_index = 0; td_index < _ux_system_host -> ux_system_host_max_iso_td; td_index++)
    {

        /* Check the TD status, a free TD is marked with the UX_USED flag.  */
        if (td -> ux_sim_host_iso_td_status == UX_UNUSED)
        {

            /* The TD may have been used, so we reset all fields.  */
            _ux_utility_memory_set(td, 0, sizeof(UX_HCD_SIM_HOST_ISO_TD)); /* Use case of memset is verified. */

            /* This TD is now marked as USED.  */
            td -> ux_sim_host_iso_td_status = UX_USED;

            /* Success, return pointer to TD.  */
            return(td);
        }

        /* Move to next TD.  */
        td++;
    }

    /* There is no available TD in the TD list, return a NULL.  */
    return(UX_NULL);
}

