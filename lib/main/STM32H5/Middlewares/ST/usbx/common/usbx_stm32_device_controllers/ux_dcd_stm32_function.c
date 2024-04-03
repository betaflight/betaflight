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


#if defined(UX_DEVICE_STANDALONE)
extern VOID     _ux_dcd_stm32_setup_isr_pending(UX_DCD_STM32 *);
#endif

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_dcd_stm32_function                              PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function dispatches the DCD function internally to the STM32   */
/*    controller.                                                         */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd                                   Pointer to device controller  */
/*    function                              Function requested            */
/*    parameter                             Pointer to function parameters*/
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_dcd_stm32_endpoint_create         Create endpoint               */
/*    _ux_dcd_stm32_endpoint_destroy        Destroy endpoint              */
/*    _ux_dcd_stm32_endpoint_reset          Reset endpoint                */
/*    _ux_dcd_stm32_endpoint_stall          Stall endpoint                */
/*    _ux_dcd_stm32_endpoint_status         Get endpoint status           */
/*    _ux_dcd_stm32_frame_number_get        Get frame number              */
/*    _ux_dcd_stm32_transfer_request        Request data transfer         */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Device Stack                                                   */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s), used ST  */
/*                                            HAL library to drive the    */
/*                                            controller,                 */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_dcd_stm32_function(UX_SLAVE_DCD *dcd, UINT function, VOID *parameter)
{

UINT             status;
UX_DCD_STM32     *dcd_stm32;


    /* Check the status of the controller.  */
    if (dcd -> ux_slave_dcd_status == UX_UNUSED)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_DCD, UX_CONTROLLER_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONTROLLER_UNKNOWN, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_CONTROLLER_UNKNOWN);
    }

    /* Get the pointer to the STM32 DCD.  */
    dcd_stm32 =  (UX_DCD_STM32 *) dcd -> ux_slave_dcd_controller_hardware;

    /* Look at the function and route it.  */
    switch(function)
    {

    case UX_DCD_GET_FRAME_NUMBER:

        status =  _ux_dcd_stm32_frame_number_get(dcd_stm32, (ULONG *) parameter);
        break;

    case UX_DCD_TRANSFER_REQUEST:

#if defined(UX_DEVICE_STANDALONE)
        status =  _ux_dcd_stm32_transfer_run(dcd_stm32, (UX_SLAVE_TRANSFER *) parameter);
#else
        status =  _ux_dcd_stm32_transfer_request(dcd_stm32, (UX_SLAVE_TRANSFER *) parameter);
#endif /* defined(UX_DEVICE_STANDALONE) */
        break;

    case UX_DCD_TRANSFER_ABORT:
        status = _ux_dcd_stm32_transfer_abort(dcd_stm32, parameter);
        break;

    case UX_DCD_CREATE_ENDPOINT:

        status =  _ux_dcd_stm32_endpoint_create(dcd_stm32, parameter);
        break;

    case UX_DCD_DESTROY_ENDPOINT:

        status =  _ux_dcd_stm32_endpoint_destroy(dcd_stm32, parameter);
        break;

    case UX_DCD_RESET_ENDPOINT:

        status =  _ux_dcd_stm32_endpoint_reset(dcd_stm32, parameter);
        break;

    case UX_DCD_STALL_ENDPOINT:

        status =  _ux_dcd_stm32_endpoint_stall(dcd_stm32, parameter);
        break;

    case UX_DCD_SET_DEVICE_ADDRESS:

        status =  HAL_PCD_SetAddress(dcd_stm32 -> pcd_handle, (uint8_t)(ULONG) parameter);
        break;

    case UX_DCD_CHANGE_STATE:

        if ((ULONG) parameter == UX_DEVICE_FORCE_DISCONNECT)
        {
          /* Disconnect the USB device */
          status =  HAL_PCD_Stop(dcd_stm32 -> pcd_handle);
        }
        else
        {
          status = UX_SUCCESS;
        }

        break;

    case UX_DCD_ENDPOINT_STATUS:

        status =  _ux_dcd_stm32_endpoint_status(dcd_stm32, (ULONG) parameter);
        break;

#if defined(UX_DEVICE_STANDALONE)
    case UX_DCD_ISR_PENDING:

        _ux_dcd_stm32_setup_isr_pending(dcd_stm32);
        status = UX_SUCCESS;
        break;
#endif /* defined(UX_DEVICE_STANDALONE) */

    default:

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_DCD, UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        status =  UX_FUNCTION_NOT_SUPPORTED;
        break;
    }

    /* Return completion status.  */
    return(status);
}

