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
/*    _ux_hcd_ohci_ed_obtain                              PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function obtains a free ED from the ED list.                  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ohci                              Pointer to OHCI HCD           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    UX_OHCI_ED  *                         Pointer to ED                 */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_set                Set memory block              */ 
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
/*                                                                        */
/**************************************************************************/
UX_OHCI_ED  *_ux_hcd_ohci_ed_obtain(UX_HCD_OHCI *hcd_ohci)
{

UX_OHCI_ED      *ed;
ULONG           ed_index;


    /* Start the search from the beginning of the list.  */
    ed =  hcd_ohci -> ux_hcd_ohci_ed_list;
    for (ed_index = 0; ed_index < _ux_system_host -> ux_system_host_max_ed; ed_index++)
    {

        /* Check the ED status, a free ED is marked with the UNUSED flag.  */
        if (ed -> ux_ohci_ed_status == UX_UNUSED)
        {

            /* The ED may have been used, so we reset all fields.  */
            _ux_utility_memory_set(ed, 0, sizeof(UX_OHCI_ED)); /* Use case of memset is verified. */

            /* This ED is now marked as USED.  */
            ed -> ux_ohci_ed_status =  UX_USED;

            /* Success, return the ED pointer.  */
            return(ed);
        }
 
        /* Point to the next ED.  */
        ed++;
    }

    /* There is no available ED in the ED list, return a NULL.  */
    return(UX_NULL);
}

