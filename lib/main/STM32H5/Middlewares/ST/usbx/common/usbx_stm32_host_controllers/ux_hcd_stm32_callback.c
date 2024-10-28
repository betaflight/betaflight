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
/*    HAL_HCD_Connect_Callback                            PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function handles callback from HAL driver.                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hhcd                                  Pointer to HCD handle         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_semaphore_put             Put semaphore                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    stm32 Controller Driver                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
void HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd)
{

UX_HCD              *hcd;
UX_HCD_STM32        *hcd_stm32;


    /* Get the pointer to the HCD & HCD_STM32.  */
    hcd = (UX_HCD*)hhcd -> pData;
    hcd_stm32 = (UX_HCD_STM32*)hcd -> ux_hcd_controller_hardware;

    /* Something happened on the root hub port. Signal it to the root hub     thread.  */
    hcd -> ux_hcd_root_hub_signal[0]++;

    /* The controller has issued a ATTACH Root HUB signal.  */
    hcd_stm32 -> ux_hcd_stm32_controller_flag |= UX_HCD_STM32_CONTROLLER_FLAG_DEVICE_ATTACHED;
    hcd_stm32 -> ux_hcd_stm32_controller_flag &= ~UX_HCD_STM32_CONTROLLER_FLAG_DEVICE_DETACHED;

    /* Wake up the root hub thread.  */
    _ux_host_semaphore_put(&_ux_system_host -> ux_system_host_enum_semaphore);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    HAL_HCD_Disconnect_Callback                         PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function handles callback from HAL driver.                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hhcd                                  Pointer to HCD handle         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_semaphore_put             Put semaphore                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    stm32 Controller Driver                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd)
{

UX_HCD              *hcd;
UX_HCD_STM32        *hcd_stm32;


    /* Get the pointer to the HCD & HCD_STM32.  */
    hcd = (UX_HCD*)hhcd -> pData;
    hcd_stm32 = (UX_HCD_STM32*)hcd -> ux_hcd_controller_hardware;

    /* Something happened on the root hub port. Signal it to the root hub     thread.  */
    hcd -> ux_hcd_root_hub_signal[0]++;

    /* The controller has issued a DETACH Root HUB signal.  */
    hcd_stm32 -> ux_hcd_stm32_controller_flag |= UX_HCD_STM32_CONTROLLER_FLAG_DEVICE_DETACHED;
    hcd_stm32 -> ux_hcd_stm32_controller_flag &= ~UX_HCD_STM32_CONTROLLER_FLAG_DEVICE_ATTACHED;

    /* Wake up the root hub thread.  */
    _ux_host_semaphore_put(&_ux_system_host -> ux_system_host_enum_semaphore);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    HAL_HCD_Disconnect_Callback                         PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function handles callback from HAL driver.                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hhcd                                  Pointer to HCD handle         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_semaphore_put             Put semaphore                 */
/*    HAL_HCD_HC_SubmitRequest              Submit request                */
/*    HAL_HCD_HC_Halt                       Halt channel                  */
/*    HAL_HCD_HC_GetXferCount               Get transfer count            */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    stm32 Controller Driver                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added ISO transfer support, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
void HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum, HCD_URBStateTypeDef urb_state)
{

UX_HCD              *hcd;
UX_HCD_STM32        *hcd_stm32;
UX_HCD_STM32_ED     *ed;
UX_TRANSFER         *transfer_request;
UX_TRANSFER         *transfer_next;


    /* Check the URB state.  */
    if (urb_state == URB_DONE || urb_state == URB_STALL || urb_state == URB_ERROR || urb_state == URB_NOTREADY)
    {

        /* Get the pointer to the HCD & HCD_STM32.  */
        hcd = (UX_HCD*)hhcd -> pData;
        hcd_stm32 = (UX_HCD_STM32*)hcd -> ux_hcd_controller_hardware;

        /* Check if driver is still valid.  */
        if (hcd_stm32 == UX_NULL)
            return;

        /* Load the ED for the channel.  */
        ed =  hcd_stm32 -> ux_hcd_stm32_channels_ed[chnum];

        /* Check if ED is still valid.  */
        if (ed == UX_NULL)
        {
            return;
        }

        /* Get transfer request.  */
        transfer_request = ed -> ux_stm32_ed_transfer_request;

        /* Check if request is still valid.  */
        if (transfer_request == UX_NULL)
        {
            return;
        }

        /* Check if URB state is not URB_NOTREADY.  */
        if (urb_state != URB_NOTREADY)
        {

            /* Handle URB states.  */
            switch (urb_state)
            {
            case URB_STALL:

                /* Set the completion code to stalled.  */
                transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_STALLED;
                break;

            case URB_DONE:

                /* Check the request direction.  */
                if (ed -> ux_stm32_ed_dir == 1)
                {
                  if ((ed -> ux_stm32_ed_type == EP_TYPE_CTRL) || (ed -> ux_stm32_ed_type == EP_TYPE_BULK))
                  {
                    /* Get transfer size for receiving direction. */
                    transfer_request -> ux_transfer_request_actual_length += HAL_HCD_HC_GetXferCount(hcd_stm32 -> hcd_handle, ed -> ux_stm32_ed_channel);

                    /* Check if there is more data to be received. */
                    if ((transfer_request -> ux_transfer_request_requested_length > transfer_request -> ux_transfer_request_actual_length) &&
                       (HAL_HCD_HC_GetXferCount(hcd_stm32 -> hcd_handle, ed -> ux_stm32_ed_channel) == ed->ux_stm32_ed_endpoint->ux_endpoint_descriptor.wMaxPacketSize))
                    {
                      /* Adjust the transmit length.  */
                      ed -> ux_stm32_ed_packet_length = UX_MIN(ed->ux_stm32_ed_endpoint->ux_endpoint_descriptor.wMaxPacketSize,
                                                               transfer_request -> ux_transfer_request_requested_length - transfer_request -> ux_transfer_request_actual_length);

                      /* Submit the transmit request.  */
                      HAL_HCD_HC_SubmitRequest(hcd_stm32 -> hcd_handle, ed -> ux_stm32_ed_channel,
                                               1U,
                                               ed -> ux_stm32_ed_type,
                                               USBH_PID_DATA,
                                               ed -> ux_stm32_ed_data + transfer_request -> ux_transfer_request_actual_length,
                                               ed -> ux_stm32_ed_packet_length, 0);
                      return;
                    }
                  }
                  else
                  {
                    /* Get transfer size for receiving direction. */
                    transfer_request -> ux_transfer_request_actual_length = HAL_HCD_HC_GetXferCount(hcd_stm32 -> hcd_handle, ed -> ux_stm32_ed_channel);
                  }
                }

                /* Check if the request is for OUT transfer.  */
                if (ed -> ux_stm32_ed_dir == 0U)
                {

#if defined (USBH_HAL_HUB_SPLIT_SUPPORTED)
                  if ((hcd_stm32->hcd_handle->hc[ed -> ux_stm32_ed_channel].do_ssplit == 1U) && (ed -> ux_stm32_ed_type == EP_TYPE_ISOC) &&
                      (ed -> ux_stm32_ed_packet_length > hcd_stm32->hcd_handle->hc[ed -> ux_stm32_ed_channel].max_packet))
                  {
                    /* Update actual transfer length with ISOC max packet size for split transaction  */
                    transfer_request -> ux_transfer_request_actual_length += hcd_stm32->hcd_handle->hc[ed -> ux_stm32_ed_channel].max_packet;
                  }
                  else
#endif /*defined (USBH_HAL_HUB_SPLIT_SUPPORTED) */
                  {
                    /* Update actual transfer length.  */
                    transfer_request -> ux_transfer_request_actual_length += ed -> ux_stm32_ed_packet_length;
                  }

                    /* Check if there is more data to send.  */
                    if (transfer_request -> ux_transfer_request_requested_length >
                        transfer_request -> ux_transfer_request_actual_length)
                    {

#if defined (USBH_HAL_HUB_SPLIT_SUPPORTED)
                      if ((hcd_stm32->hcd_handle->hc[ed -> ux_stm32_ed_channel].do_ssplit == 1U) && (ed -> ux_stm32_ed_type == EP_TYPE_ISOC) &&
                          (ed -> ux_stm32_ed_packet_length > hcd_stm32->hcd_handle->hc[ed -> ux_stm32_ed_channel].max_packet))
                      {
                        /* Adjust the transmit length.  */
                        ed -> ux_stm32_ed_packet_length = transfer_request -> ux_transfer_request_packet_length - \
                                                          hcd_stm32->hcd_handle->hc[ed -> ux_stm32_ed_channel].max_packet;

                        /* Submit the transmit request.  */
                        HAL_HCD_HC_SubmitRequest(hcd_stm32 -> hcd_handle, ed -> ux_stm32_ed_channel,
                                                 0, ed -> ux_stm32_ed_type, USBH_PID_DATA,
                                                 ed -> ux_stm32_ed_data + transfer_request -> ux_transfer_request_actual_length,
                                                 ed -> ux_stm32_ed_packet_length, 0);
                        return;
                      }
#endif /* defined (USBH_HAL_HUB_SPLIT_SUPPORTED) */

                        /* Periodic transfer that needs schedule is not started here.  */
                        if (ed -> ux_stm32_ed_sch_mode)
                            return;

                        /* Adjust the transmit length.  */
                        ed -> ux_stm32_ed_packet_length =
                            UX_MIN(transfer_request -> ux_transfer_request_packet_length,
                                   transfer_request -> ux_transfer_request_requested_length -
                                   transfer_request -> ux_transfer_request_actual_length);

                        /* Submit the transmit request.  */
                        HAL_HCD_HC_SubmitRequest(hcd_stm32 -> hcd_handle, ed -> ux_stm32_ed_channel,
                                                 0, ed -> ux_stm32_ed_type, USBH_PID_DATA,
                                                 ed -> ux_stm32_ed_data + transfer_request -> ux_transfer_request_actual_length,
                                                 ed -> ux_stm32_ed_packet_length, 0);
                        return;
                    }
                }

                /* Set the completion code to SUCCESS.  */
                transfer_request -> ux_transfer_request_completion_code =  UX_SUCCESS;
                break;

            default:
                /* Set the completion code to transfer error.  */
                transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_ERROR;
            }

            /* Finish current transfer.  */
            _ux_hcd_stm32_request_trans_finish(hcd_stm32, ed);

            /* Move to next transfer.  */
            transfer_next = transfer_request -> ux_transfer_request_next_transfer_request;
            ed -> ux_stm32_ed_transfer_request = transfer_next;

            /* If there is transfer to start, start it.  */
            if (transfer_next)
            {

                /* If transfer is not started by schedular, start here.  */
                if (!ed -> ux_stm32_ed_sch_mode)
                {

                    /* For ISO OUT, packet size is from request variable,
                    * otherwise, use request length.  */
                    if ((ed -> ux_stm32_ed_type == EP_TYPE_ISOC) && (ed -> ux_stm32_ed_dir == 0))
                        ed -> ux_stm32_ed_packet_length = transfer_next -> ux_transfer_request_packet_length;
                    else
                        ed -> ux_stm32_ed_packet_length = transfer_next -> ux_transfer_request_requested_length;

                    /* Prepare transactions.  */
                    _ux_hcd_stm32_request_trans_prepare(hcd_stm32, ed, transfer_next);

                    /* Call HAL driver to submit the transfer request.  */
                    HAL_HCD_HC_SubmitRequest(hcd_stm32 -> hcd_handle, ed -> ux_stm32_ed_channel,
                                             ed -> ux_stm32_ed_dir,
                                             ed -> ux_stm32_ed_type, USBH_PID_DATA,
                                             ed -> ux_stm32_ed_data + transfer_next -> ux_transfer_request_actual_length,
                                             ed -> ux_stm32_ed_packet_length, 0);
                }
            }
            else
            {

                /* Transfer not continued, periodic needs re-schedule.  */
                if ((ed -> ux_stm32_ed_type == EP_TYPE_INTR) ||
                    (ed -> ux_stm32_ed_type == EP_TYPE_ISOC))
                    ed -> ux_stm32_ed_sch_mode = 1;
                }

#if defined(UX_HOST_STANDALONE)
            transfer_request -> ux_transfer_request_status = UX_TRANSFER_STATUS_COMPLETED;
            ed -> ux_stm32_ed_status |= UX_HCD_STM32_ED_STATUS_TRANSFER_DONE;
#endif /* defined(UX_HOST_STANDALONE) */

            /* Invoke callback function.  */
            if (transfer_request -> ux_transfer_request_completion_function)
                transfer_request -> ux_transfer_request_completion_function(transfer_request);

            /* Wake up the transfer request thread.  */
            _ux_host_semaphore_put(&transfer_request -> ux_transfer_request_semaphore);

        }
        else
        {

            /* Handle URB_NOTREADY state here.  */
            /* Check if we need to retry the transfer by checking the status.  */
            if ((ed -> ux_stm32_ed_status == UX_HCD_STM32_ED_STATUS_CONTROL_SETUP) ||
                (ed -> ux_stm32_ed_status == UX_HCD_STM32_ED_STATUS_CONTROL_DATA_OUT) ||
                (ed -> ux_stm32_ed_status == UX_HCD_STM32_ED_STATUS_CONTROL_STATUS_OUT) ||
                (ed -> ux_stm32_ed_status == UX_HCD_STM32_ED_STATUS_BULK_OUT))
            {

                /* Submit the transmit request.  */
                HAL_HCD_HC_SubmitRequest(hcd_stm32 -> hcd_handle, ed -> ux_stm32_ed_channel, 0,
                                        ((ed -> ux_stm32_ed_endpoint -> ux_endpoint_descriptor.bmAttributes) & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT ? EP_TYPE_BULK : EP_TYPE_CTRL,
                                         ed -> ux_stm32_ed_status == UX_HCD_STM32_ED_STATUS_CONTROL_SETUP ? USBH_PID_SETUP : USBH_PID_DATA,
                                         ed -> ux_stm32_ed_data + transfer_request -> ux_transfer_request_actual_length,
                                         ed -> ux_stm32_ed_packet_length, 0);
            }

        }
    }
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    HAL_HCD_SOF_Callback                                PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function handles callback from HAL driver.                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hhcd                                  Pointer to HCD handle         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_semaphore_put             Put semaphore                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    stm32 Controller Driver                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
void HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd)
{

UX_HCD              *hcd;
UX_HCD_STM32        *hcd_stm32;


    /* Get the pointer to the HCD & HCD_STM32.  */
    hcd = (UX_HCD*)hhcd -> pData;
    hcd_stm32 = (UX_HCD_STM32*)hcd -> ux_hcd_controller_hardware;

    if ((hcd_stm32 -> ux_hcd_stm32_controller_flag & UX_HCD_STM32_CONTROLLER_FLAG_SOF) == 0)
    {
        hcd_stm32 -> ux_hcd_stm32_controller_flag |= UX_HCD_STM32_CONTROLLER_FLAG_SOF;
        hcd -> ux_hcd_thread_signal++;

        /* Wake up the scheduler.  */
        _ux_host_semaphore_put(&_ux_system_host -> ux_system_host_hcd_semaphore);
    }
}
