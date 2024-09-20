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
/*    _ux_hcd_ehci_hsisochronous_td_obtain                PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function obtains a free TD from the high speed isochronous     */
/*    TD list.                                                            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_ehci                              Pointer to EHCI controller    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    UX_EHCI_HSISO_TD *                    Pointer to EHCI HSISO TD      */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    None                                                                */
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
/*                                                                        */
/**************************************************************************/
UX_EHCI_HSISO_TD  *_ux_hcd_ehci_hsisochronous_td_obtain(UX_HCD_EHCI *hcd_ehci)
{
#if UX_MAX_ISO_TD == 0

    /* This function is not yet supported, return NULL. */
    UX_PARAMETER_NOT_USED(hcd_ehci);
    return(UX_NULL);
#else

UX_EHCI_HSISO_TD    *td;
ULONG               td_index;


    /* Start the search from the beginning of the list.  */
    td =  hcd_ehci -> ux_hcd_ehci_hsiso_td_list;
    for (td_index = 0; td_index < _ux_system_host -> ux_system_host_max_iso_td; td_index++)
    {

        /* Check the TD status, a free TD is marked with the USED flag.  */
        if (td -> ux_ehci_hsiso_td_status == UX_UNUSED)
        {

            /* The TD may have been used, so we reset all fields.  */
            _ux_utility_memory_set(td, 0, sizeof(UX_EHCI_HSISO_TD)); /* Use case of memset is verified. */

            /* This TD is now marked as USED.  */
            td -> ux_ehci_hsiso_td_status =  UX_USED;

            /* Initialize the link pointer TD fields.  */
            td -> ux_ehci_hsiso_td_next_lp.value = UX_EHCI_HSISO_T;

            /* Success, return TD pointer.  */
            return(td);
        }

        /* Look at next TD.  */
        td++;
    }

    /* There is no available TD in the TD list.  */

    /* Error, return a null.  */
    return(UX_NULL);
#endif
}

