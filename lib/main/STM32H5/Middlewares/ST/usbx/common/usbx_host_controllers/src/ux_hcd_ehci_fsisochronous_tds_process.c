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
/*    _ux_hcd_ehci_fsisochronous_tds_process              PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processes the isochronous, periodic and asynchronous  */
/*    lists in search for transfers that occurred in the past             */
/*    (micro-)frame.                                                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_ehci                              Pointer to HCD EHCI           */
/*    prev_itd                              Pointer to pointer point to   */
/*                                          previous FSISO TD             */
/*    itd                                   Pointer to FSISO TD           */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    UX_EHCI_HSISO_TD                      Pointer to FSISO TD           */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    (ux_transfer_request_completion_function) Completion function       */
/*    _ux_host_semaphore_put                Put semaphore                 */
/*    _ux_utility_physical_address          Get physical address          */
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UX_EHCI_FSISO_TD* _ux_hcd_ehci_fsisochronous_tds_process(
    UX_HCD_EHCI *hcd_ehci,
    UX_EHCI_FSISO_TD* itd)
{

    UX_PARAMETER_NOT_USED(hcd_ehci);

    /* TBD  */

    return(itd -> ux_ehci_fsiso_td_next_scan_td);
}

