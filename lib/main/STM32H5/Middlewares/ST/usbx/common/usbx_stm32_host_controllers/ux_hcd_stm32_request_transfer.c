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
/**   STM32 Controller Driver                                             */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE
#define UX_HCD_STM32_SOURCE_CODE

#include "ux_api.h"
#include "ux_hcd_stm32.h"
#include "ux_host_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_hcd_stm32_request_transfer                      PORTABLE C      */
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
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_stm32                             Pointer to STM32 controller   */
/*    transfer_request                      Pointer to transfer request   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_hcd_stm32_request_bulk_transfer         Request bulk transfer   */
/*    _ux_hcd_stm32_request_control_transfer      Request control         */
/*                                                    transfer            */
/*    _ux_hcd_stm32_request_periodic_transfer     Request periodic        */
/*                                                    transfer            */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 Controller Driver                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_stm32_request_transfer(UX_HCD_STM32 *hcd_stm32, UX_TRANSFER *transfer_request)
{

UX_ENDPOINT     *endpoint;
UINT            status;

    /* Device Connection Status.  */
    if (hcd_stm32 -> ux_hcd_stm32_controller_flag & UX_HCD_STM32_CONTROLLER_FLAG_DEVICE_ATTACHED)
    {

        /* Get the pointer to the Endpoint.  */
        endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

#if !defined(UX_HOST_STANDALONE)

        /* We reset the actual length field of the transfer request as a safety measure.  */
        transfer_request -> ux_transfer_request_actual_length =  0;
#endif /* !defined(UX_HOST_STANDALONE) */

        /* Isolate the endpoint type and route the transfer request.  */
        switch ((endpoint -> ux_endpoint_descriptor.bmAttributes) & UX_MASK_ENDPOINT_TYPE)
        {

        case UX_CONTROL_ENDPOINT:

            status = _ux_hcd_stm32_request_control_transfer(hcd_stm32, transfer_request);
            break;

        case UX_BULK_ENDPOINT:

            status = _ux_hcd_stm32_request_bulk_transfer(hcd_stm32, transfer_request);
            break;

        case UX_INTERRUPT_ENDPOINT:
        case UX_ISOCHRONOUS_ENDPOINT:

            status = _ux_hcd_stm32_request_periodic_transfer(hcd_stm32, transfer_request);
            break;

        default:

#if defined(UX_HOST_STANDALONE)
            status =  UX_ERROR;
#else
            transfer_request -> ux_transfer_request_completion_code = UX_ERROR;
            return(UX_STATE_EXIT);
#endif /* defined(UX_HOST_STANDALONE) */
        }
    }
    else
    {

        /* Error, no device attached.  */
#if defined(UX_HOST_STANDALONE)
        status = UX_NO_DEVICE_CONNECTED;
#else
        transfer_request -> ux_transfer_request_completion_code = UX_NO_DEVICE_CONNECTED;
        status = UX_STATE_EXIT;
#endif /* defined(UX_HOST_STANDALONE) */

    }

    return(status);
}

