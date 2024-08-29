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
/*    _ux_hcd_ehci_request_isochronous_transfer           PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*     This function performs an isochronous transfer request (list).     */
/*                                                                        */
/*     Note: the request max length is endpoint max packet size, multiple */
/*     endpoint max number of transactions.                               */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_ehci                              Pointer to EHCI controller    */
/*    transfer_request                      Pointer to transfer request.  */
/*                                          If next transfer request is   */
/*                                          valid the whole request list  */
/*                                          is added, until next transfer */
/*                                          request being NULL. If next   */
/*                                          next transfer request is not  */
/*                                          valid (being NULL) single     */
/*                                          transfer request is added.    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_mutex_on                     Get mutex                     */
/*    _ux_host_mutex_off                    Put mutex                     */
/*    _ux_host_semaphore_put                Put semaphore                 */
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
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved iso start up,      */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ehci_request_isochronous_transfer(UX_HCD_EHCI *hcd_ehci, UX_TRANSFER *transfer_request)
{
#if UX_MAX_ISO_TD == 0

    UX_PARAMETER_NOT_USED(hcd_ehci);
    UX_PARAMETER_NOT_USED(transfer_request);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* Not supported yet - return error.  */
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_ENDPOINT                     *endpoint;
UX_EHCI_PERIODIC_LINK_POINTER   lp;
UX_EHCI_HSISO_ED                *ied;
UX_TRANSFER                     **head;
UX_TRANSFER                     **tail;
UX_TRANSFER                     **first_new;
UX_TRANSFER                     *request_list;
UCHAR                           start = UX_FALSE;


    /* Get the pointer to the endpoint.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

    /* Now get the physical iTD/siTD attached to this endpoint.  */
    lp.ed_ptr =  endpoint -> ux_endpoint_ed;

    /* Lock the periodic list to update.  */
    _ux_host_mutex_on(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);

    /* Append the request to iTD/siTD request list tail.  */
#if defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
    if (endpoint -> ux_endpoint_device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
        head = &lp.sitd_ptr -> ux_ehci_fsiso_td_transfer_head;
    else
#endif
    {

        /* Get pointer of ED.  */
        ied = lp.itd_ptr -> ux_ehci_hsiso_td_ed;

        /* Get pointer locations.  */
        head = &ied -> ux_ehci_hsiso_ed_transfer_head;
        tail = &ied -> ux_ehci_hsiso_ed_transfer_tail;
        first_new = &ied -> ux_ehci_hsiso_ed_transfer_first_new;

        /* If there is no transfer, start.  */
        if (ied -> ux_ehci_hsiso_ed_frstart == 0xFF)
        {
            ied -> ux_ehci_hsiso_ed_frstart = 0xFE;
            ied -> ux_ehci_hsiso_ed_fr_sw = 0;
            ied -> ux_ehci_hsiso_ed_fr_hc = 0;
            start = UX_TRUE;
        }
    }

    request_list = (*head);
    if (request_list == UX_NULL)
    {

        /* Link to head.  */
        (*head) = transfer_request;
        (*tail) = transfer_request;
        (*first_new) = transfer_request;
    }
    else
    {

        /* Link to tail of the list.  */
        (*tail) -> ux_transfer_request_next_transfer_request = transfer_request;

        /* In case there is nothing to load, set new ones.  */
        if (*first_new == UX_NULL)
            *first_new = transfer_request;
    }

    /* Move tail until it's real last one.  */
    while((*tail) -> ux_transfer_request_next_transfer_request != UX_NULL)
        (*tail) = ((*tail) -> ux_transfer_request_next_transfer_request);

    /* Release the periodic table.  */
    _ux_host_mutex_off(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);

    /* Simulate iTD/siTD done to start - HCD signal.  */
    if (start)
    {
        hcd_ehci -> ux_hcd_ehci_hcd_owner -> ux_hcd_thread_signal ++;
        _ux_host_semaphore_put(&_ux_system_host -> ux_system_host_hcd_semaphore);
    }

    /* Return completion status.  */
    return(UX_SUCCESS);
#endif
}

