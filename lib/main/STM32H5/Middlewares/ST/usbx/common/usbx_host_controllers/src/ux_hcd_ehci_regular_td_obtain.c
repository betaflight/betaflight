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
/**   EHCI Controller Driver                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_hcd_ehci.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_hcd_ehci_regular_td_obtain                      PORTABLE C      */ 
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
/*    hcd_ehci                              Pointer to EHCI controller    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    UX_EHCI_TD *                          Pointer to TD                 */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_set                Set memory block              */ 
/*    _ux_host_mutex_on                     Get protection mutex          */
/*    _ux_host_mutex_off                    Release protection mutex      */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    EHCI Controller Driver                                              */
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
UX_EHCI_TD  *_ux_hcd_ehci_regular_td_obtain(UX_HCD_EHCI *hcd_ehci)
{

UX_EHCI_TD      *td;
ULONG           td_index;
ULONG           td_element;


    /* Get the mutex as this is a critical section.  */
    _ux_host_mutex_on(&_ux_system -> ux_system_mutex);

    /* Start the search from the beginning of the list.  */
    td =  hcd_ehci -> ux_hcd_ehci_td_list;
    for (td_index = 0; td_index < _ux_system_host -> ux_system_host_max_td; td_index++)
    {

        /* Check the TD status, a free TD is marked with the USED flag.  */
        if (td -> ux_ehci_td_status == UX_UNUSED)
        {

            /* The TD may have been used, so we reset all fields.  */
            _ux_utility_memory_set(td, 0, sizeof(UX_EHCI_TD)); /* Use case of memset is verified. */

            /* This TD is now marked as USED.  */
            td -> ux_ehci_td_status =  UX_USED;

            /* Initialize the link pointer and alternate TD fields.  */
            td_element =  UX_EHCI_TD_T;
            td -> ux_ehci_td_link_pointer =  (UX_EHCI_TD *) td_element;
            td -> ux_ehci_td_alternate_link_pointer =  (UX_EHCI_TD *) td_element;
            
            /* Release the protection.  */
            _ux_host_mutex_off(&_ux_system -> ux_system_mutex);

            /* Success, return TD pointer.  */
            return(td);
        }

        /* Look at next TD.  */
        td++;
    }

    /* There is no available TD in the TD list.  */

    /* Release protection.  */
    _ux_host_mutex_off(&_ux_system -> ux_system_mutex);

    /* Error, return a null.  */
    return(UX_NULL);
}

