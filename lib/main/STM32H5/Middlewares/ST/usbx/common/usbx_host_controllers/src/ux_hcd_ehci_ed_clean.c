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
/*    _ux_hcd_ehci_ed_clean                               PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function cleans the tds attached to a ED in case a transfer    */ 
/*    has to be aborted.                                                  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    ed                                    Pointer to ED                 */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_virtual_address           Get virtual address           */ 
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
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ehci_ed_clean(UX_EHCI_ED *ed)
{

UX_EHCI_TD      *td;
UX_EHCI_TD      *next_td;
    

    /* Get the first pointer to the TD.  */   
    td =  ed -> ux_ehci_ed_queue_element;
    td =  (UX_EHCI_TD *) ((ULONG) td & ~UX_EHCI_QH_T);
    td =  _ux_utility_virtual_address(td);

    /* Mark the TD link of the endpoint as terminated.  */
    ed -> ux_ehci_ed_queue_element =  (UX_EHCI_TD *) UX_EHCI_TD_T;

    /* Free all tds attached to the ED.  */
    while (td != UX_NULL)
    {

        /* Get the next TD pointed by the current TD.  */
        next_td =  td -> ux_ehci_td_link_pointer;
        next_td =  (UX_EHCI_TD *) ((ULONG) next_td & ~UX_EHCI_TD_T);
        next_td =  _ux_utility_virtual_address(next_td);

        /* Mark the current TD as free.  */
        td -> ux_ehci_td_status =  UX_UNUSED;

        td =  next_td;
    }

    /* Reset the first TD.  */
    ed -> ux_ehci_ed_first_td =  UX_NULL;

    /* Reset the last TD.  */
    ed -> ux_ehci_ed_last_td =  UX_NULL;

    /* Return successful completion.  */
    return(UX_SUCCESS);          
}

