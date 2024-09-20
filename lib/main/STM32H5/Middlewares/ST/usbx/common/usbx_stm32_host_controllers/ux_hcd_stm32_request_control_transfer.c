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


static inline VOID _ux_hcd_stm32_request_control_setup(UX_HCD_STM32 *hcd_stm32,
    UX_HCD_STM32_ED *ed, UX_ENDPOINT *endpoint, UX_TRANSFER *transfer_request);

static inline VOID _ux_hcd_stm32_request_control_data(UX_HCD_STM32 *hcd_stm32,
    UX_HCD_STM32_ED *ed, UX_ENDPOINT *endpoint, UX_TRANSFER *transfer_request);

static inline VOID _ux_hcd_stm32_request_control_status(UX_HCD_STM32 *hcd_stm32,
    UX_HCD_STM32_ED *ed, UX_ENDPOINT *endpoint, UX_TRANSFER *transfer_request);


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_hcd_stm32_request_control_transfer              PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*     This function performs a control transfer from a transfer request. */
/*     The USB control transfer is in 3 phases (setup, data, status).     */
/*     This function will chain all phases of the control sequence before */
/*     setting the stm32 endpoint as a candidate for transfer.            */
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
/*    _ux_hcd_stm32_regular_td_obtain       Obtain regular TD             */
/*    _ux_host_stack_transfer_request_abort Abort transfer request        */
/*    _ux_utility_memory_allocate           Allocate memory block         */
/*    _ux_utility_memory_free               Release memory block          */
/*    _ux_utility_semaphore_get             Get semaphore                 */
/*    _ux_utility_short_put                 Write 16-bit value            */
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
/*                                            prefixed UX to MS_TO_TICK,  */
/*                                            refined macros names,       */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_stm32_request_control_transfer(UX_HCD_STM32 *hcd_stm32, UX_TRANSFER *transfer_request)
{

#if defined(UX_HOST_STANDALONE)
UX_INTERRUPT_SAVE_AREA
#else
UINT                    status;
#endif /* defined(UX_HOST_STANDALONE) */
UX_ENDPOINT             *endpoint;
UX_HCD_STM32_ED         *ed;

    /* Get the pointer to the Endpoint.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

    /* Now get the physical ED attached to this endpoint.  */
    ed =  endpoint -> ux_endpoint_ed;

#if defined(UX_HOST_STANDALONE)
    UX_DISABLE
    switch(ed -> ux_stm32_ed_status)
    {
    case UX_HCD_STM32_ED_STATUS_ALLOCATED:
    /* Fall through.  */
    case UX_HCD_STM32_ED_STATUS_ABORTED:

        /* Setup for SETUP packet.  */
        _ux_hcd_stm32_request_control_setup(hcd_stm32, ed, endpoint, transfer_request);
        UX_RESTORE
        if (ed -> ux_stm32_ed_setup == UX_NULL)
        {
            transfer_request -> ux_transfer_request_completion_code = UX_MEMORY_INSUFFICIENT;
            return(UX_STATE_ERROR);
        }
        return(UX_STATE_WAIT);

    case UX_HCD_STM32_ED_STATUS_CONTROL_SETUP | UX_HCD_STM32_ED_STATUS_TRANSFER_DONE:

        /* Free allocated memory.  */
        _ux_utility_memory_free(ed -> ux_stm32_ed_setup);
        ed -> ux_stm32_ed_setup = UX_NULL;

        /* Restore request information.  */
        transfer_request -> ux_transfer_request_requested_length =
                                                ed -> ux_stm32_ed_saved_length;
        transfer_request -> ux_transfer_request_actual_length = 0;

        /* Check completion code.  */
        if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
        {
            ed -> ux_stm32_ed_status = UX_HCD_STM32_ED_STATUS_ALLOCATED;
            UX_RESTORE
            return(UX_STATE_NEXT);
        }

        if (ed -> ux_stm32_ed_saved_length)
        {

            /* To data stage.  */
            _ux_hcd_stm32_request_control_data(hcd_stm32, ed, endpoint, transfer_request);
        }
        else
        {

            /* To status stage.  */
            _ux_hcd_stm32_request_control_status(hcd_stm32, ed, endpoint, transfer_request);
        }
        UX_RESTORE
        return(UX_STATE_WAIT);

    case UX_HCD_STM32_ED_STATUS_CONTROL_DATA_IN | UX_HCD_STM32_ED_STATUS_TRANSFER_DONE:
    /* Fall through.  */
    case UX_HCD_STM32_ED_STATUS_CONTROL_DATA_OUT | UX_HCD_STM32_ED_STATUS_TRANSFER_DONE:

        /* Check completion code.  */
        if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
        {
            ed -> ux_stm32_ed_status = UX_HCD_STM32_ED_STATUS_ALLOCATED;
            UX_RESTORE
            return(UX_STATE_NEXT);
        }

        /* Get request actual length for IN transfer.  */
        if (ed -> ux_stm32_ed_dir)
        {

            /* Get the actual transfer length.  */
            transfer_request -> ux_transfer_request_actual_length =
                            HAL_HCD_HC_GetXferCount(hcd_stm32 -> hcd_handle,
                                                    ed -> ux_stm32_ed_channel);
        }
        else
        {

            /* For OUT, all data is sent.  */
            transfer_request -> ux_transfer_request_actual_length =
                    transfer_request -> ux_transfer_request_requested_length;
        }

        /* To status stage.  */
        _ux_hcd_stm32_request_control_status(hcd_stm32, ed, endpoint, transfer_request);
        UX_RESTORE
        return(UX_STATE_WAIT);

    case UX_HCD_STM32_ED_STATUS_CONTROL_STATUS_IN | UX_HCD_STM32_ED_STATUS_TRANSFER_DONE:
    /* Fall through.  */
    case UX_HCD_STM32_ED_STATUS_CONTROL_STATUS_OUT | UX_HCD_STM32_ED_STATUS_TRANSFER_DONE:

        /* All done, reset status.  */
        ed -> ux_stm32_ed_status = UX_HCD_STM32_ED_STATUS_ALLOCATED;

        /* Restore saved things.  */
        transfer_request -> ux_transfer_request_requested_length = ed -> ux_stm32_ed_saved_length;
        transfer_request -> ux_transfer_request_actual_length = ed -> ux_stm32_ed_saved_actual_length;

        UX_RESTORE
        return(UX_STATE_NEXT);

    case UX_HCD_STM32_ED_STATUS_CONTROL_SETUP:
    /* Fall through.  */
    case UX_HCD_STM32_ED_STATUS_CONTROL_DATA_IN:
    /* Fall through.  */
    case UX_HCD_STM32_ED_STATUS_CONTROL_DATA_OUT:
    /* Fall through.  */
    case UX_HCD_STM32_ED_STATUS_CONTROL_STATUS_IN:
    /* Fall through.  */
    case UX_HCD_STM32_ED_STATUS_CONTROL_STATUS_OUT:

        /* Keep waiting.  */
        UX_RESTORE
        return(UX_STATE_WAIT);

    default:
        UX_RESTORE

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_INVALID_STATE);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_INVALID_STATE, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_STATE_EXIT);
    }
#else

    /* Setup for SETUP packet.  */
    _ux_hcd_stm32_request_control_setup(hcd_stm32, ed, endpoint, transfer_request);
    if (ed -> ux_stm32_ed_setup == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Wait for the completion of the transfer request.  */
    status =  _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore, MS_TO_TICK(UX_CONTROL_TRANSFER_TIMEOUT));

    /* Free the resources.  */
    _ux_utility_memory_free(ed -> ux_stm32_ed_setup);

    /* If the semaphore did not succeed we probably have a time out.  */
    if (status != UX_SUCCESS)
    {

        /* All transfers pending need to abort. There may have been a partial transfer.  */
        _ux_host_stack_transfer_request_abort(transfer_request);

        /* There was an error, return to the caller.  */
        transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_TIMEOUT;

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_TIMEOUT, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_TRANSFER_TIMEOUT);
    }

    /* Check the transfer request completion code.  */
    if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Return completion to caller.  */
        return(transfer_request -> ux_transfer_request_completion_code);
    }

    /* Check if there is data phase.  */
    if (ed -> ux_stm32_ed_saved_length)
    {

        /* Prepare data stage.  */
        _ux_hcd_stm32_request_control_data(hcd_stm32, ed, endpoint, transfer_request);

        /* Wait for the completion of the transfer request.  */
        status =  _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore, MS_TO_TICK(UX_CONTROL_TRANSFER_TIMEOUT));

        /* If the semaphore did not succeed we probably have a time out.  */
        if (status != UX_SUCCESS)
        {

            /* All transfers pending need to abort. There may have been a partial transfer.  */
            _ux_host_stack_transfer_request_abort(transfer_request);

            /* There was an error, return to the caller.  */
            transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_TIMEOUT;

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_TIMEOUT, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_TRANSFER_TIMEOUT);

        }

        /* Check the transfer request completion code.  */
        if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
        {

            /* Return completion to caller.  */
            return(transfer_request -> ux_transfer_request_completion_code);
        }
    }

    /* Prepare status stage.  */
    _ux_hcd_stm32_request_control_status(hcd_stm32, ed, endpoint, transfer_request);

    /* Wait for the completion of the transfer request.  */
    status =  _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore, UX_MS_TO_TICK(UX_CONTROL_TRANSFER_TIMEOUT));

    /* Restore the original transfer parameter.  */
    transfer_request -> ux_transfer_request_requested_length = ed -> ux_stm32_ed_saved_length;
    transfer_request -> ux_transfer_request_actual_length    = ed -> ux_stm32_ed_saved_actual_length;

    /* If the semaphore did not succeed we probably have a time out.  */
    if (status != UX_SUCCESS)
    {

        /* All transfers pending need to abort. There may have been a partial transfer.  */
        _ux_host_stack_transfer_request_abort(transfer_request);

        /* There was an error, return to the caller.  */
        transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_TIMEOUT;

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_TIMEOUT, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

    }

    /* Return completion to caller.  */
    return(transfer_request -> ux_transfer_request_completion_code);
#endif /* defined(UX_HOST_STANDALONE) */
}

static inline VOID _ux_hcd_stm32_request_control_setup(UX_HCD_STM32 *hcd_stm32,
    UX_HCD_STM32_ED *ed, UX_ENDPOINT *endpoint, UX_TRANSFER *transfer_request)
{
UCHAR                   *setup_request;

    /* Save the pending transfer in the ED.  */
    ed -> ux_stm32_ed_transfer_request = transfer_request;

    /* Build the SETUP packet (phase 1 of the control transfer).  */
    ed -> ux_stm32_ed_setup = UX_NULL;
    setup_request = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, UX_SETUP_SIZE);

    if (setup_request == UX_NULL)
        return;

    ed -> ux_stm32_ed_setup = setup_request;

    /* Build the SETUP request.  */
    *(setup_request + UX_SETUP_REQUEST_TYPE) =  transfer_request -> ux_transfer_request_type;
    *(setup_request + UX_SETUP_REQUEST) =       transfer_request -> ux_transfer_request_function;
    _ux_utility_short_put(setup_request + UX_SETUP_VALUE, transfer_request -> ux_transfer_request_value);
    _ux_utility_short_put(setup_request + UX_SETUP_INDEX, transfer_request -> ux_transfer_request_index);
    _ux_utility_short_put(setup_request + UX_SETUP_LENGTH, (USHORT) transfer_request -> ux_transfer_request_requested_length);

    /* Save the original transfer parameter.  */
    ed -> ux_stm32_ed_saved_length = transfer_request -> ux_transfer_request_requested_length;
    ed -> ux_stm32_ed_data = setup_request;

    /* Reset requested length for SETUP packet.  */
    transfer_request -> ux_transfer_request_requested_length = 0;

    /* Set the packet length for SETUP packet.  */
    ed -> ux_stm32_ed_packet_length = 8;

    /* Set the current status.  */
    ed -> ux_stm32_ed_status = UX_HCD_STM32_ED_STATUS_CONTROL_SETUP;

    /* Set device speed.  */
    switch (endpoint -> ux_endpoint_device -> ux_device_speed)
    {
    case UX_HIGH_SPEED_DEVICE:
        ed -> ux_stm32_ed_speed =  HCD_DEVICE_SPEED_HIGH;
        break;
    case UX_LOW_SPEED_DEVICE:
        ed -> ux_stm32_ed_speed =  HCD_DEVICE_SPEED_LOW;
        break;
    default:
        ed -> ux_stm32_ed_speed =  HCD_DEVICE_SPEED_FULL;
        break;
    }

    /* Initialize the host channel for SETUP phase.  */
    ed -> ux_stm32_ed_dir = 0;
    HAL_HCD_HC_Init(hcd_stm32 -> hcd_handle,
                    ed -> ux_stm32_ed_channel,
                    0,
                    endpoint -> ux_endpoint_device -> ux_device_address,
                    ed -> ux_stm32_ed_speed,
                    EP_TYPE_CTRL,
                    endpoint -> ux_endpoint_descriptor.wMaxPacketSize);

#if defined (USBH_HAL_HUB_SPLIT_SUPPORTED)
    /* Check if device connected to hub  */
    if (endpoint->ux_endpoint_device->ux_device_parent != NULL)
    {
      HAL_HCD_HC_SetHubInfo(hcd_stm32->hcd_handle, ed->ux_stm32_ed_channel,
                            endpoint->ux_endpoint_device->ux_device_parent->ux_device_address,
                            endpoint->ux_endpoint_device->ux_device_port_location);
    }
#endif /* USBH_HAL_HUB_SPLIT_SUPPORTED */

    /* Send the SETUP packet.  */
    HAL_HCD_HC_SubmitRequest(hcd_stm32 -> hcd_handle, ed -> ux_stm32_ed_channel,
                             0, EP_TYPE_CTRL, USBH_PID_SETUP, setup_request, 8, 0);
}

static inline VOID _ux_hcd_stm32_request_control_data(UX_HCD_STM32 *hcd_stm32,
    UX_HCD_STM32_ED *ed, UX_ENDPOINT *endpoint, UX_TRANSFER *transfer_request)
{

    /* Check the direction of the transaction.  */
    if ((transfer_request -> ux_transfer_request_type & UX_REQUEST_DIRECTION) ==
         UX_REQUEST_IN)
    {

        /* Re-initialize the host channel to IN direction.  */
        ed -> ux_stm32_ed_dir = 1;
        HAL_HCD_HC_Init(hcd_stm32 -> hcd_handle,
                        ed -> ux_stm32_ed_channel,
                        0x80,
                        endpoint -> ux_endpoint_device -> ux_device_address,
                        ed -> ux_stm32_ed_speed,
                        EP_TYPE_CTRL,
                        endpoint -> ux_endpoint_descriptor.wMaxPacketSize);

#if defined (USBH_HAL_HUB_SPLIT_SUPPORTED)
        /* Check if device connected to hub  */
        if (endpoint->ux_endpoint_device->ux_device_parent != NULL)
        {
          HAL_HCD_HC_SetHubInfo(hcd_stm32->hcd_handle, ed->ux_stm32_ed_channel,
                                endpoint->ux_endpoint_device->ux_device_parent->ux_device_address,
                                endpoint->ux_endpoint_device->ux_device_port_location);
        }
#endif /* USBH_HAL_HUB_SPLIT_SUPPORTED */

        /* Set the current status to data IN.  */
        ed -> ux_stm32_ed_status = UX_HCD_STM32_ED_STATUS_CONTROL_DATA_IN;
    }
    else
    {

        /* Set the current status to data OUT.  */
        ed -> ux_stm32_ed_status = UX_HCD_STM32_ED_STATUS_CONTROL_DATA_OUT;
    }

    /* Save the pending transfer in the ED.  */
    ed -> ux_stm32_ed_transfer_request = transfer_request;

    /* Set the transfer to pending.  */
    transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_STATUS_PENDING;

    /* Restore requested length.  */
    transfer_request -> ux_transfer_request_requested_length = ed -> ux_stm32_ed_saved_length;

    /* If the direction is OUT, request size is larger than MPS, and DMA is not used, we need to set transfer length to MPS.  */
    if ((ed -> ux_stm32_ed_dir == 0) &&
            (transfer_request -> ux_transfer_request_requested_length > endpoint -> ux_endpoint_descriptor.wMaxPacketSize) &&
            (hcd_stm32 -> hcd_handle -> Init.dma_enable == 0))
    {

        /* Set transfer length to MPS.  */
        ed -> ux_stm32_ed_packet_length = endpoint -> ux_endpoint_descriptor.wMaxPacketSize;
    }
    else
    {

        /* Keep the original transfer length.  */
        ed -> ux_stm32_ed_packet_length = transfer_request -> ux_transfer_request_requested_length;
    }

    /* Reset actual length.  */
    transfer_request -> ux_transfer_request_actual_length = 0;

    /* Prepare transactions.  */
    _ux_hcd_stm32_request_trans_prepare(hcd_stm32, ed, transfer_request);

    /* Submit the transfer request.  */
    HAL_HCD_HC_SubmitRequest(hcd_stm32 -> hcd_handle, ed -> ux_stm32_ed_channel,
                                ed -> ux_stm32_ed_dir,
                                EP_TYPE_CTRL, USBH_PID_DATA,
                                ed -> ux_stm32_ed_data,
                                ed -> ux_stm32_ed_packet_length, 0);
}

static inline VOID _ux_hcd_stm32_request_control_status(UX_HCD_STM32 *hcd_stm32,
    UX_HCD_STM32_ED *ed, UX_ENDPOINT *endpoint, UX_TRANSFER *transfer_request)
{

    /* Setup status phase direction.  */
    ed -> ux_stm32_ed_dir = !ed -> ux_stm32_ed_dir;
    HAL_HCD_HC_Init(hcd_stm32 -> hcd_handle,
                ed -> ux_stm32_ed_channel,
                ed -> ux_stm32_ed_dir ? 0x80 : 0,
                endpoint -> ux_endpoint_device -> ux_device_address,
                ed -> ux_stm32_ed_speed,
                EP_TYPE_CTRL,
                endpoint -> ux_endpoint_descriptor.wMaxPacketSize);

#if defined (USBH_HAL_HUB_SPLIT_SUPPORTED)
    /* Check if device connected to hub  */
    if (endpoint->ux_endpoint_device->ux_device_parent != NULL)
    {
      HAL_HCD_HC_SetHubInfo(hcd_stm32->hcd_handle, ed->ux_stm32_ed_channel,
                            endpoint->ux_endpoint_device->ux_device_parent->ux_device_address,
                            endpoint->ux_endpoint_device->ux_device_port_location);
    }
#endif /* USBH_HAL_HUB_SPLIT_SUPPORTED */

    /* Save the pending transfer in the ED.  */
    ed -> ux_stm32_ed_transfer_request = transfer_request;

    /* Set the transfer to pending.  */
    transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_STATUS_PENDING;

    /* Save the original transfer parameter.  */
    ed -> ux_stm32_ed_saved_length = (USHORT)
                    transfer_request -> ux_transfer_request_requested_length;
    transfer_request -> ux_transfer_request_requested_length = 0;

    ed -> ux_stm32_ed_saved_actual_length = (USHORT)
                    transfer_request -> ux_transfer_request_actual_length;
    transfer_request -> ux_transfer_request_actual_length = 0;

    /* Reset the packet length.  */
    ed -> ux_stm32_ed_packet_length = 0;

    /* Set the current status to data OUT.  */
    ed -> ux_stm32_ed_status = ed -> ux_stm32_ed_dir ?
                                    UX_HCD_STM32_ED_STATUS_CONTROL_STATUS_IN :
                                    UX_HCD_STM32_ED_STATUS_CONTROL_STATUS_OUT;

    /* Submit the request for status phase.  */
    HAL_HCD_HC_SubmitRequest(hcd_stm32 -> hcd_handle, ed -> ux_stm32_ed_channel,
                             ed -> ux_stm32_ed_dir,
                             EP_TYPE_CTRL, USBH_PID_DATA, 0, 0, 0);
}
