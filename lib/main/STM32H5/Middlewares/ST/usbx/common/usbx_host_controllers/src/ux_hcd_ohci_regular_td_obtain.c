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
/*    _ux_hcd_ohci_regular_td_obtain                      PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function obtains a free TD from the regular TD list.          */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ohci                              Pointer to OHCI controller    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    *UX_OHCI_TD                           Regular TD pointer            */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_set                Set memory block              */ 
/*    _ux_host_mutex_on                     Get protection mutex          */
/*    _ux_host_mutex_off                    Release protection mutex      */ 
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
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UX_OHCI_TD  *_ux_hcd_ohci_regular_td_obtain(UX_HCD_OHCI *hcd_ohci)
{

UX_OHCI_TD      *td;
ULONG           td_index;


    /* Set the Mutex as this is a critical section.  */
    _ux_host_mutex_on(&_ux_system -> ux_system_mutex);

    /* Start the search from the beginning of the regular TD list.  */
    td =  hcd_ohci -> ux_hcd_ohci_td_list;

    for (td_index = 0; td_index < _ux_system_host -> ux_system_host_max_td; td_index++)
    {

        /* Check the TD status, a free TD is marked with the UNUSED flag.  */
        if (td -> ux_ohci_td_status == UX_UNUSED)
        {

            /* The TD may have been used, so we reset all fields.  */
            _ux_utility_memory_set(td, 0, sizeof(UX_OHCI_TD)); /* Use case of memset is verified. */

            /* This TD is now marked as USED.  */
            td -> ux_ohci_td_status =  UX_USED;

            /* Release the protection.  */
            _ux_host_mutex_off(&_ux_system -> ux_system_mutex);

            /* Return TD pointer - success!  */
            return(td);
        }

        /* Move to the next TD.  */
        td++;
    }

    /* There is no available TD in the TD list.  */

    /* Release protection.  */
    _ux_host_mutex_off(&_ux_system -> ux_system_mutex);

    /* Return NULL to caller.  */
    return(UX_NULL);
}

