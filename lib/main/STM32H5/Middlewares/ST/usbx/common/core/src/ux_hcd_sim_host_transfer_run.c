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


#if defined(UX_HOST_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_hcd_sim_host_transfer_run                       PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*     This function is the handler for all the transactions on the USB.  */
/*     The transfer request passed as parameter contains the endpoint and */
/*     the device descriptors in addition to the type of transaction de   */
/*     be executed. This function routes the transfer request to          */
/*     according to the type of transfer to be executed.                  */
/*                                                                        */
/*     It's for standalone mode.                                          */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_sim_host                          Pointer to host controller    */
/*    transfer_request                      Pointer to transfer request   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_hcd_sim_host_request_bulk_transfer        Request bulk transfer */
/*    _ux_hcd_sim_host_request_control_transfer     Request control       */
/*                                                  transfer              */
/*    _ux_hcd_sim_host_request_interrupt_transfer   Request interrupt     */
/*                                                  transfer              */
/*    _ux_hcd_sim_host_request_isochronous_transfer Request isochronous   */
/*                                                  transfer              */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Host Simulator Controller Driver                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_sim_host_transfer_run(UX_HCD_SIM_HOST *hcd_sim_host, UX_TRANSFER *transfer_request)
{

UX_INTERRUPT_SAVE_AREA
UX_ENDPOINT         *endpoint;
UX_HCD_SIM_HOST_ED  *ed;
UINT                status = 0;


    /* Get the pointer to the Endpoint.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

    /* Sanity check.  */
    if (endpoint == UX_NULL)
        return(UX_STATE_EXIT);

    /* Get ED.  */
    ed = (UX_HCD_SIM_HOST_ED *)endpoint -> ux_endpoint_ed;

    /* Sanity check.  */
    if (ed == UX_NULL)
        return(UX_STATE_EXIT);

    UX_DISABLE

    /* If transfer started, check status.  */
    if (ed -> ux_sim_host_ed_status & UX_HCD_SIM_HOST_ED_TRANSFER)
    {
        if (ed -> ux_sim_host_ed_head_td != ed -> ux_sim_host_ed_tail_td)
        {
            UX_RESTORE
            return(UX_STATE_WAIT);
        }

        /* Check if it's transfer waiting state.  */
        if (transfer_request -> ux_transfer_request_status != UX_TRANSFER_STATUS_NOT_PENDING)
        {

            /* Yes, polling pending status, report and transfer done.  */
            ed -> ux_sim_host_ed_status &= ~UX_HCD_SIM_HOST_ED_TRANSFER;
            UX_RESTORE
            return(UX_STATE_NEXT);
        }

        /* Maybe transfer completed but state not reported yet.  */
    }
    ed -> ux_sim_host_ed_status |= UX_HCD_SIM_HOST_ED_TRANSFER;
    transfer_request -> ux_transfer_request_status = UX_TRANSFER_STATUS_PENDING;

    UX_RESTORE

    /* We reset the actual length field of the transfer request as a safety measure.  */
    transfer_request -> ux_transfer_request_actual_length =  0;

    /* Isolate the endpoint type and route the transfer request.  */
    switch ((endpoint -> ux_endpoint_descriptor.bmAttributes) & UX_MASK_ENDPOINT_TYPE)
    {

    case UX_CONTROL_ENDPOINT:

        status =  _ux_hcd_sim_host_request_control_transfer(hcd_sim_host, transfer_request);
        break;


    case UX_BULK_ENDPOINT:

        status =  _ux_hcd_sim_host_request_bulk_transfer(hcd_sim_host, transfer_request);
        break;

    case UX_INTERRUPT_ENDPOINT:

        status =  _ux_hcd_sim_host_request_interrupt_transfer(hcd_sim_host, transfer_request);
        break;

    case UX_ISOCHRONOUS_ENDPOINT:

        status =  _ux_hcd_sim_host_request_isochronous_transfer(hcd_sim_host, transfer_request);
        break;

    }

    return (status == UX_SUCCESS) ? (UX_STATE_WAIT) : (UX_STATE_ERROR);
}
#endif
