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

#define UX_SOURCE_CODE
#define UX_DCD_STM32_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_stm32.h"
#include "ux_device_stack.h"
#include "ux_utility.h"


static inline void _ux_dcd_stm32_setup_in(UX_DCD_STM32_ED * ed, UX_SLAVE_TRANSFER *transfer_request)
{

    /* The endpoint is IN.  This is important to memorize the direction for the control endpoint
        in case of a STALL. */
    ed -> ux_dcd_stm32_ed_direction = UX_ENDPOINT_IN;

    /* Set the state to TX.  */
    ed -> ux_dcd_stm32_ed_state = UX_DCD_STM32_ED_STATE_DATA_TX;

    /* Call the Control Transfer dispatcher.  */
    _ux_device_stack_control_request_process(transfer_request);
}

static inline void _ux_dcd_stm32_setup_out(UX_DCD_STM32_ED * ed, UX_SLAVE_TRANSFER *transfer_request,
                                           PCD_HandleTypeDef *hpcd)
{

    /* Set the completion code to no error.  */
    transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;

    /* The endpoint is IN.  This is important to memorize the direction for the control endpoint
        in case of a STALL. */
    ed -> ux_dcd_stm32_ed_direction = UX_ENDPOINT_IN;

    /* We are using a Control endpoint on a OUT transaction and there was a payload.  */
    if (_ux_device_stack_control_request_process(transfer_request) == UX_SUCCESS)
    {

        /* Set the state to STATUS phase TX.  */
        ed -> ux_dcd_stm32_ed_state = UX_DCD_STM32_ED_STATE_STATUS_TX;

        /* Arm the status transfer.  */
        HAL_PCD_EP_Transmit(hpcd, 0x00U, UX_NULL, 0U);
    }
}

static inline void _ux_dcd_stm32_setup_status(UX_DCD_STM32_ED * ed, UX_SLAVE_TRANSFER *transfer_request,
                                              PCD_HandleTypeDef *hpcd)
{

    /* The endpoint is IN.  This is important to memorize the direction for the control endpoint
            in case of a STALL. */
    ed -> ux_dcd_stm32_ed_direction = UX_ENDPOINT_IN;

    /* Call the Control Transfer dispatcher.  */
    if (_ux_device_stack_control_request_process(transfer_request) == UX_SUCCESS)
    {

        /* Set the state to STATUS RX.  */
        ed -> ux_dcd_stm32_ed_state = UX_DCD_STM32_ED_STATE_STATUS_RX;
        HAL_PCD_EP_Transmit(hpcd, 0x00U, UX_NULL, 0U);
    }
}

#if defined(UX_DEVICE_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                                RELEASE       */
/*                                                                        */
/*    _ux_dcd_stm32_setup_isr_pending                      PORTABLE C     */
/*                                                            6.1.10      */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function handles pending control ISR from HAL driver.          */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hpcd                                  Pointer to PCD handle         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_control_request_process                            */
/*                                          Process control request       */
/*    HAL_PCD_EP_Transmit                   Transmit data                 */
/*    HAL_PCD_EP_Receive                    Receive data                  */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 HAL Driver                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*                                                                        */
/**************************************************************************/
VOID     _ux_dcd_stm32_setup_isr_pending(UX_DCD_STM32 *dcd_stm32)
{
UX_INTERRUPT_SAVE_AREA
UX_DCD_STM32_ED         *ed;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                   ed_status;

    /* Fetch the address of the physical endpoint.  */
    ed =  &dcd_stm32 -> ux_dcd_stm32_ed[0];

    /* Get the pointer to the transfer request.  */
    transfer_request =  &ed -> ux_dcd_stm32_ed_endpoint -> ux_slave_endpoint_transfer_request;

    UX_DISABLE

    /* Get the ED status.  */
    ed_status = ed -> ux_dcd_stm32_ed_status;

    /* Check if Task is pending to avoid re-entry.  */
    if (ed_status & UX_DCD_STM32_ED_STATUS_TASK_PENDING)
    {
        UX_RESTORE
        return;
    }

    /* Check if SETUP ISR is pending.  */
    ed_status &= UX_DCD_STM32_ED_STATUS_SETUP;
    if (ed_status == 0)
    {
        UX_RESTORE
        return;
    }
    ed -> ux_dcd_stm32_ed_status &= ~UX_DCD_STM32_ED_STATUS_SETUP;
    ed -> ux_dcd_stm32_ed_status |= UX_DCD_STM32_ED_STATUS_TASK_PENDING;
    UX_RESTORE

    /* Handle different SETUP cases.  */
    switch(ed_status)
    {
    case UX_DCD_STM32_ED_STATUS_SETUP_IN:
        _ux_dcd_stm32_setup_in(ed, transfer_request);
        break;

    case UX_DCD_STM32_ED_STATUS_SETUP_OUT:
        _ux_dcd_stm32_setup_out(ed, transfer_request, dcd_stm32 -> pcd_handle);
        break;

    default: /* UX_DCD_STM32_ED_STATUS_SETUP_STATUS  */
        _ux_dcd_stm32_setup_status(ed, transfer_request, dcd_stm32 -> pcd_handle);
        break;
    }

    /* Task is done.  */
    ed -> ux_dcd_stm32_ed_status &= ~UX_DCD_STM32_ED_STATUS_TASK_PENDING;
}
#endif

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    HAL_PCD_SetupStageCallback                          PORTABLE C      */
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
/*    hpcd                                  Pointer to PCD handle         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_control_request_process                            */
/*                                          Process control request       */
/*    HAL_PCD_EP_Transmit                   Transmit data                 */
/*    HAL_PCD_EP_Receive                    Receive data                  */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 HAL Driver                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  09-30-2020     Chaoqiong Xiao           Initial Version 6.1           */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{

UX_SLAVE_DCD            *dcd;
UX_DCD_STM32            *dcd_stm32;
UX_DCD_STM32_ED         *ed;
UX_SLAVE_TRANSFER       *transfer_request;
UX_SLAVE_ENDPOINT       *endpoint;


    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the STM32 DCD.  */
    dcd_stm32 = (UX_DCD_STM32 *) dcd -> ux_slave_dcd_controller_hardware;

    /* Fetch the address of the physical endpoint.  */
    ed =  &dcd_stm32 -> ux_dcd_stm32_ed[0];

    /* Get the pointer to the transfer request.  */
    transfer_request =  &ed -> ux_dcd_stm32_ed_endpoint -> ux_slave_endpoint_transfer_request;

    /* Copy setup data to transfer request.  */
    _ux_utility_memory_copy(transfer_request->ux_slave_transfer_request_setup, hpcd -> Setup, UX_SETUP_SIZE);

    /* Clear the length of the data received.  */
    transfer_request -> ux_slave_transfer_request_actual_length =  0;

    /* Mark the phase as SETUP.  */
    transfer_request -> ux_slave_transfer_request_type =  UX_TRANSFER_PHASE_SETUP;

    /* Mark the transfer as successful.  */
    transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;

    /* Set the status of the endpoint to not stalled.  */
    ed -> ux_dcd_stm32_ed_status &= ~(UX_DCD_STM32_ED_STATUS_STALLED |
                                      UX_DCD_STM32_ED_STATUS_TRANSFER |
                                      UX_DCD_STM32_ED_STATUS_DONE);

    /* Check if the transaction is IN.  */
    if (*transfer_request -> ux_slave_transfer_request_setup & UX_REQUEST_IN)
    {
#if defined(UX_DEVICE_STANDALONE)
        ed -> ux_dcd_stm32_ed_status |= UX_DCD_STM32_ED_STATUS_SETUP_IN;
#else
        _ux_dcd_stm32_setup_in(ed, transfer_request);
#endif
    }
    else
    {

        /* The endpoint is OUT.  This is important to memorize the direction for the control endpoint
           in case of a STALL. */
        ed -> ux_dcd_stm32_ed_direction  = UX_ENDPOINT_OUT;

        /* We are in a OUT transaction. Check if there is a data payload. If so, wait for the payload
           to be delivered.  */
        if (*(transfer_request -> ux_slave_transfer_request_setup + 6) == 0 &&
            *(transfer_request -> ux_slave_transfer_request_setup + 7) == 0)
        {
#if defined(UX_DEVICE_STANDALONE)
            ed -> ux_dcd_stm32_ed_status |= UX_DCD_STM32_ED_STATUS_SETUP_STATUS;
#else
            _ux_dcd_stm32_setup_status(ed, transfer_request, hpcd);
#endif
        }
        else
        {

            /* Get the pointer to the logical endpoint from the transfer request.  */
            endpoint =  transfer_request -> ux_slave_transfer_request_endpoint;

            /* Get the length we expect from the SETUP packet.  */
            transfer_request -> ux_slave_transfer_request_requested_length = _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + 6);

            /* Check if we have enough space for the request.  */
            if (transfer_request -> ux_slave_transfer_request_requested_length > UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH)
            {

                /* No space available, stall the endpoint.  */
                _ux_dcd_stm32_endpoint_stall(dcd_stm32, endpoint);

                /* Next phase is a SETUP.  */
                ed -> ux_dcd_stm32_ed_state =  UX_DCD_STM32_ED_STATE_IDLE;

#if defined(UX_DEVICE_STANDALONE)
                ed -> ux_dcd_stm32_ed_status |= UX_DCD_STM32_ED_STATUS_SETUP_STATUS;
#endif

                /* We are done.  */
                return;
            }
            else
            {

                /* Reset what we have received so far.  */
                transfer_request -> ux_slave_transfer_request_actual_length =  0;

                /* And reprogram the current buffer address to the beginning of the buffer.  */
                transfer_request -> ux_slave_transfer_request_current_data_pointer =  transfer_request -> ux_slave_transfer_request_data_pointer;

                /* Receive data.  */
                HAL_PCD_EP_Receive(hpcd,
                            endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress,
                            transfer_request -> ux_slave_transfer_request_current_data_pointer,
                            transfer_request -> ux_slave_transfer_request_requested_length);

                /* Set the state to RX.  */
                ed -> ux_dcd_stm32_ed_state =  UX_DCD_STM32_ED_STATE_DATA_RX;
            }
        }
    }
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    HAL_PCD_DataInStageCallback                         PORTABLE C      */
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
/*    hpcd                                  Pointer to PCD handle         */
/*    epnum                                 Endpoint number               */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_semaphore_put             Put semaphore                 */
/*    HAL_PCD_EP_Transmit                   Transmit data                 */
/*    HAL_PCD_EP_Receive                    Receive data                  */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 HAL Driver                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  09-30-2020     Chaoqiong Xiao           Initial Version 6.1           */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            added bi-dir EP support,    */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed transmit ZLP issue,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{

UX_SLAVE_DCD            *dcd;
UX_DCD_STM32            *dcd_stm32;
UX_DCD_STM32_ED         *ed;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                   transfer_length;
UX_SLAVE_ENDPOINT       *endpoint;


    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the STM32 DCD.  */
    dcd_stm32 = (UX_DCD_STM32 *) dcd -> ux_slave_dcd_controller_hardware;

    /* Fetch the address of the physical endpoint.  */
#if defined(UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT)
    if ((epnum & 0xF) != 0)
        ed =  &dcd_stm32 -> ux_dcd_stm32_ed_in[epnum & 0xF];
    else
#endif /* defined(UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT) */
    ed =  &dcd_stm32 -> ux_dcd_stm32_ed[epnum & 0xF];

    /* Get the pointer to the transfer request.  */
    transfer_request =  &(ed -> ux_dcd_stm32_ed_endpoint -> ux_slave_endpoint_transfer_request);

    /* Endpoint 0 is different.  */
    if (epnum == 0U)
    {

        /* Get the pointer to the logical endpoint from the transfer request.  */
        endpoint =  transfer_request -> ux_slave_transfer_request_endpoint;

        /* Check if we need to send data again on control endpoint. */
        if (ed -> ux_dcd_stm32_ed_state == UX_DCD_STM32_ED_STATE_DATA_TX)
        {

            /* Arm Status transfer.  */
            HAL_PCD_EP_Receive(hpcd, 0, 0, 0);

            /* Are we done with this transfer ? */
            if (transfer_request -> ux_slave_transfer_request_in_transfer_length <=
                endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize)
            {

                /* There is no data to send but we may need to send a Zero Length Packet.  */
                if (transfer_request -> ux_slave_transfer_request_force_zlp ==  UX_TRUE)
                {

                    /* Arm a ZLP packet on IN.  */
                    HAL_PCD_EP_Transmit(hpcd,
                            endpoint->ux_slave_endpoint_descriptor.bEndpointAddress, 0, 0);

                    /* Reset the ZLP condition.  */
                    transfer_request -> ux_slave_transfer_request_force_zlp =  UX_FALSE;

                }
                else
                {

                    /* Set the completion code to no error.  */
                    transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;

                    /* The transfer is completed.  */
                    transfer_request -> ux_slave_transfer_request_status =  UX_TRANSFER_STATUS_COMPLETED;
                    transfer_request -> ux_slave_transfer_request_actual_length =
                        transfer_request -> ux_slave_transfer_request_requested_length;

#if defined(UX_DEVICE_STANDALONE)

                    /* Control status phase done.  */
                    ed -> ux_dcd_stm32_ed_status |= UX_DCD_STM32_ED_STATUS_DONE;
#endif

                    /* We are using a Control endpoint, if there is a callback, invoke it. We are still under ISR.  */
                    if (transfer_request -> ux_slave_transfer_request_completion_function)
                        transfer_request -> ux_slave_transfer_request_completion_function (transfer_request) ;

                    /* State is now STATUS RX.  */
                    ed -> ux_dcd_stm32_ed_state = UX_DCD_STM32_ED_STATE_STATUS_RX;
                }
            }
            else
            {

                /* Get the size of the transfer.  */
                transfer_length = transfer_request -> ux_slave_transfer_request_in_transfer_length - endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize;

                /* Check if the endpoint size is bigger that data requested. */
                if (transfer_length > endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize)
                {

                    /* Adjust the transfer size.  */
                    transfer_length =  endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize;
                }

                /* Adjust the data pointer.  */
                transfer_request -> ux_slave_transfer_request_current_data_pointer += endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize;

                /* Adjust the transfer length remaining.  */
                transfer_request -> ux_slave_transfer_request_in_transfer_length -= transfer_length;

                /* Transmit data.  */
                HAL_PCD_EP_Transmit(hpcd,
                            endpoint->ux_slave_endpoint_descriptor.bEndpointAddress,
                            transfer_request->ux_slave_transfer_request_current_data_pointer,
                            transfer_length);
            }
        }
    }
    else
    {

        /* Check if a ZLP should be armed.  */
        if (transfer_request -> ux_slave_transfer_request_force_zlp &&
            transfer_request -> ux_slave_transfer_request_requested_length)
        {

            /* Reset the ZLP condition.  */
            transfer_request -> ux_slave_transfer_request_force_zlp =  UX_FALSE;
            transfer_request -> ux_slave_transfer_request_in_transfer_length = 0;

            /* Arm a ZLP packet on IN.  */
            HAL_PCD_EP_Transmit(hpcd, epnum, 0, 0);

        }
        else
        {

            /* Set the completion code to no error.  */
            transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;

            /* The transfer is completed.  */
            transfer_request -> ux_slave_transfer_request_status =  UX_TRANSFER_STATUS_COMPLETED;
            transfer_request -> ux_slave_transfer_request_actual_length =
                transfer_request -> ux_slave_transfer_request_requested_length;

#if defined(UX_DEVICE_STANDALONE)
        ed -> ux_dcd_stm32_ed_status |= UX_DCD_STM32_ED_STATUS_DONE;
#else

        /* Non control endpoint operation, use semaphore.  */
        _ux_utility_semaphore_put(&transfer_request -> ux_slave_transfer_request_semaphore);
#endif /* defined(UX_DEVICE_STANDALONE) */
        }
    }
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    HAL_PCD_DataOutStageCallback                        PORTABLE C      */
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
/*    hpcd                                  Pointer to PCD handle         */
/*    epnum                                 Endpoint number               */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_control_request_process                            */
/*                                          Process control request       */
/*    _ux_utility_semaphore_put             Put semaphore                 */
/*    HAL_PCD_EP_Transmit                   Transmit data                 */
/*    HAL_PCD_EP_Receive                    Receive data                  */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 HAL Driver                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  09-30-2020     Chaoqiong Xiao           Initial Version 6.1           */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{

UX_SLAVE_DCD            *dcd;
UX_DCD_STM32            *dcd_stm32;
UX_DCD_STM32_ED         *ed;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                   transfer_length;
UX_SLAVE_ENDPOINT       *endpoint;


    /* Get the pointer to the DCD.  */
    dcd = &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the STM32 DCD.  */
    dcd_stm32 = (UX_DCD_STM32 *) dcd -> ux_slave_dcd_controller_hardware;

    /* Fetch the address of the physical endpoint.  */
    ed = &dcd_stm32 -> ux_dcd_stm32_ed[epnum & 0xF];

    /* Get the pointer to the transfer request.  */
    transfer_request = &(ed -> ux_dcd_stm32_ed_endpoint -> ux_slave_endpoint_transfer_request);

    /* Endpoint 0 is different.  */
    if (epnum == 0U)
    {

        /* Check if we have received something on endpoint 0 during data phase .  */
        if (ed -> ux_dcd_stm32_ed_state == UX_DCD_STM32_ED_STATE_DATA_RX)
        {

            /* Get the pointer to the logical endpoint from the transfer request.  */
            endpoint = transfer_request -> ux_slave_transfer_request_endpoint;

            /* Read the received data length for the Control endpoint.  */
            transfer_length = HAL_PCD_EP_GetRxCount(hpcd, epnum);

            /* Update the length of the data received.  */
            transfer_request -> ux_slave_transfer_request_actual_length += transfer_length;

            /* Can we accept this much?  */
            if (transfer_request -> ux_slave_transfer_request_actual_length <=
                transfer_request -> ux_slave_transfer_request_requested_length)
            {

                /* Are we done with this transfer ? */
                if ((transfer_request -> ux_slave_transfer_request_actual_length ==
                     transfer_request -> ux_slave_transfer_request_requested_length) ||
                    (transfer_length != endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize))
                {
#if defined(UX_DEVICE_STANDALONE)
                    ed -> ux_dcd_stm32_ed_status |= UX_DCD_STM32_ED_STATUS_SETUP_OUT;
#else
                    _ux_dcd_stm32_setup_out(ed, transfer_request, hpcd);
#endif
                }
                else
                {

                    /* Rearm the OUT control endpoint for one packet. */
                    transfer_request -> ux_slave_transfer_request_current_data_pointer += endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize;
                    HAL_PCD_EP_Receive(hpcd,
                                endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress,
                                transfer_request -> ux_slave_transfer_request_current_data_pointer,
                                endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize);
                }
            }
            else
            {

                /*  We have an overflow situation. Set the completion code to overflow.  */
                transfer_request -> ux_slave_transfer_request_completion_code =  UX_TRANSFER_BUFFER_OVERFLOW;

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_BUFFER_OVERFLOW, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

#if defined(UX_DEVICE_STANDALONE)

                /* Control status phase done.  */
                ed -> ux_dcd_stm32_ed_status |= UX_DCD_STM32_ED_STATUS_DONE;
#endif

                /* We are using a Control endpoint, if there is a callback, invoke it. We are still under ISR.  */
                if (transfer_request -> ux_slave_transfer_request_completion_function)
                    transfer_request -> ux_slave_transfer_request_completion_function (transfer_request) ;
            }
        }
    }
    else
    {


        /* Update the length of the data sent in previous transaction.  */
        transfer_request -> ux_slave_transfer_request_actual_length =  HAL_PCD_EP_GetRxCount(hpcd, epnum);

        /* Set the completion code to no error.  */
        transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;

        /* The transfer is completed.  */
        transfer_request -> ux_slave_transfer_request_status =  UX_TRANSFER_STATUS_COMPLETED;

#if defined(UX_DEVICE_STANDALONE)
        ed -> ux_dcd_stm32_ed_status |= UX_DCD_STM32_ED_STATUS_DONE;
#else

        /* Non control endpoint operation, use semaphore.  */
        _ux_utility_semaphore_put(&transfer_request -> ux_slave_transfer_request_semaphore);
#endif
    }

}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    HAL_PCD_ResetCallback                               PORTABLE C      */
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
/*    hpcd                                  Pointer to PCD handle         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_disconnect           Disconnect device             */
/*    _ux_dcd_stm32_initialize_complete     Complete initialization       */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 HAL Driver                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  09-30-2020     Chaoqiong Xiao           Initial Version 6.1           */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{

    /* If the device is attached or configured, we need to disconnect it.  */
    if (_ux_system_slave -> ux_system_slave_device.ux_slave_device_state !=  UX_DEVICE_RESET)
    {

        /* Disconnect the device.  */
        _ux_device_stack_disconnect();
    }

    /* Set USB Current Speed */
    switch(hpcd -> Init.speed)
    {
#ifdef PCD_SPEED_HIGH
    case PCD_SPEED_HIGH:

        /* We are connected at high speed.  */
        _ux_system_slave -> ux_system_slave_speed =  UX_HIGH_SPEED_DEVICE;
        break;
#endif
    case PCD_SPEED_FULL:

        /* We are connected at full speed.  */
        _ux_system_slave -> ux_system_slave_speed =  UX_FULL_SPEED_DEVICE;
        break;

    default:

        /* We are connected at full speed.  */
        _ux_system_slave -> ux_system_slave_speed =  UX_FULL_SPEED_DEVICE;
        break;
    }

    /* Complete the device initialization.  */
    _ux_dcd_stm32_initialize_complete();

    /* Mark the device as attached now.  */
    _ux_system_slave -> ux_system_slave_device.ux_slave_device_state =  UX_DEVICE_ATTACHED;
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    HAL_PCD_ConnectCallback                             PORTABLE C      */
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
/*    hpcd                                  Pointer to PCD handle         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 HAL Driver                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.12 */
/**************************************************************************/
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{

    /* Check the status change callback.  */
    if (_ux_system_slave -> ux_system_slave_change_function != UX_NULL)
    {

       /* Inform the application if a callback function was programmed.  */
        _ux_system_slave -> ux_system_slave_change_function(UX_DCD_STM32_DEVICE_CONNECTED);
    }
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    HAL_PCD_DisconnectCallback                          PORTABLE C      */
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
/*    hpcd                                  Pointer to PCD handle         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_disconnect           Disconnect device             */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 HAL Driver                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  09-30-2020     Chaoqiong Xiao           Initial Version 6.1           */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{

    /* Check the status change callback.  */
    if (_ux_system_slave -> ux_system_slave_change_function != UX_NULL)
    {

       /* Inform the application if a callback function was programmed.  */
        _ux_system_slave -> ux_system_slave_change_function(UX_DCD_STM32_DEVICE_DISCONNECTED);
    }

    /* Check if the device is attached or configured.  */
    if (_ux_system_slave -> ux_system_slave_device.ux_slave_device_state !=  UX_DEVICE_RESET)
    {

        /* Disconnect the device.  */
        _ux_device_stack_disconnect();
    }
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    HAL_PCD_SuspendCallback                             PORTABLE C      */
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
/*    hpcd                                  Pointer to PCD handle         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 HAL Driver                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{

    /* Check the status change callback.  */
    if (_ux_system_slave -> ux_system_slave_change_function != UX_NULL)
    {

       /* Inform the application if a callback function was programmed.  */
        _ux_system_slave -> ux_system_slave_change_function(UX_DCD_STM32_DEVICE_SUSPENDED);
    }
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    HAL_PCD_ResumeCallback                              PORTABLE C      */
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
/*    hpcd                                  Pointer to PCD handle         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 HAL Driver                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{

    /* Check the status change callback.  */
    if (_ux_system_slave -> ux_system_slave_change_function != UX_NULL)
    {

       /* Inform the application if a callback function was programmed.  */
        _ux_system_slave -> ux_system_slave_change_function(UX_DCD_STM32_DEVICE_RESUMED);
    }
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    HAL_PCD_SOFCallback                                 PORTABLE C      */
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
/*    hpcd                                  Pointer to PCD handle         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 HAL Driver                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{

    /* Check the status change callback.  */
    if (_ux_system_slave -> ux_system_slave_change_function != UX_NULL)
    {

       /* Inform the application if a callback function was programmed.  */
        _ux_system_slave -> ux_system_slave_change_function(UX_DCD_STM32_SOF_RECEIVED);
    }
}


#if defined(USBD_HAL_ISOINCOMPLETE_CALLBACK)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                                RELEASE       */
/*                                                                        */
/*    HAL_PCD_ISOINIncompleteCallback                      PORTABLE C     */
/*                                                            6.1.10      */
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
/*    hpcd                                  Pointer to PCD handle         */
/*    epnum                                 Endpoint number               */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    HAL_PCD_EP_Transmit                   Transmit data                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 HAL Driver                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*                                                                        */
/**************************************************************************/
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
UX_SLAVE_DCD            *dcd;
UX_DCD_STM32            *dcd_stm32;
UX_DCD_STM32_ED         *ed;
UX_SLAVE_ENDPOINT       *endpoint;

    UX_PARAMETER_NOT_USED(epnum);

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the STM32 DCD.  */
    dcd_stm32 = (UX_DCD_STM32 *) dcd -> ux_slave_dcd_controller_hardware;

#if defined(UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT)
    ed =  &dcd_stm32 -> ux_dcd_stm32_ed_in[epnum & 0xF];
#else
    ed =  &dcd_stm32 -> ux_dcd_stm32_ed[epnum & 0xF];
#endif /* defined(UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT) */

    if ((ed -> ux_dcd_stm32_ed_status & UX_DCD_STM32_ED_STATUS_USED) == 0U)
        return;

    endpoint = ed->ux_dcd_stm32_ed_endpoint;

    if ((endpoint->ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == 1 &&
        (endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != 0)
    {

        /* Incomplete, discard data and retry.  */
        HAL_PCD_EP_Transmit(dcd_stm32 -> pcd_handle,
                        endpoint->ux_slave_endpoint_descriptor.bEndpointAddress,
                        endpoint->ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer,
                        endpoint->ux_slave_endpoint_transfer_request.ux_slave_transfer_request_requested_length);
    }
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                                RELEASE       */
/*                                                                        */
/*    HAL_PCD_ISOOUTIncompleteCallback                     PORTABLE C     */
/*                                                            6.1.10      */
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
/*    hpcd                                  Pointer to PCD handle         */
/*    epnum                                 Endpoint number               */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    HAL_PCD_EP_Receive                    Receive data                  */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 HAL Driver                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*                                                                        */
/**************************************************************************/
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
UX_SLAVE_DCD            *dcd;
UX_DCD_STM32            *dcd_stm32;
UX_DCD_STM32_ED         *ed;
UX_SLAVE_ENDPOINT       *endpoint;

    UX_PARAMETER_NOT_USED(epnum);

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the STM32 DCD.  */
    dcd_stm32 = (UX_DCD_STM32 *) dcd -> ux_slave_dcd_controller_hardware;

    ed =  &dcd_stm32 -> ux_dcd_stm32_ed[epnum & 0xF];
    if ((ed -> ux_dcd_stm32_ed_status & UX_DCD_STM32_ED_STATUS_USED) == 0)
        return;

    endpoint = ed->ux_dcd_stm32_ed_endpoint;

    if ((endpoint->ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == 1 &&
        (endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == 0)
    {

        /* Incomplete, discard data and retry.  */
        HAL_PCD_EP_Receive(dcd_stm32 -> pcd_handle,
                        endpoint->ux_slave_endpoint_descriptor.bEndpointAddress,
                        endpoint->ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer,
                        endpoint->ux_slave_endpoint_transfer_request.ux_slave_transfer_request_requested_length);
    }
}
#endif /* defined(USBD_HAL_ISOINCOMPLETE_CALLBACK) */
