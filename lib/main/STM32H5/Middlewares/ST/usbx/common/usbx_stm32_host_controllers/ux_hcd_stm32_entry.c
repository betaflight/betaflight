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
/*    _ux_hcd_stm32_entry                                 PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function dispatch the HCD function internally to the STM32     */
/*    controller driver.                                                  */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    HCD                                   Pointer to HCD                */
/*    function                              Function for driver to perform*/
/*    parameter                             Pointer to parameter(s)       */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_hcd_stm32_asynchronous_endpoint_create  Create async endpoint   */
/*    _ux_hcd_stm32_asynchronous_endpoint_destroy Destroy async endpoint  */
/*    _ux_hcd_stm32_controller_disable            Disable controller      */
/*    _ux_hcd_stm32_endpoint_reset                Reset endpoint          */
/*    _ux_hcd_stm32_frame_number_get              Get frame number        */
/*    _ux_hcd_stm32_interrupt_endpoint_create     Create endpoint         */
/*    _ux_hcd_stm32_periodic_endpoint_destroy     Destroy endpoint        */
/*    _ux_hcd_stm32_periodic_schedule             Schedule periodic       */
/*    _ux_hcd_stm32_port_enable                   Enable port             */
/*    _ux_hcd_stm32_port_disable                  Disable port            */
/*    _ux_hcd_stm32_port_reset                    Reset port              */
/*    _ux_hcd_stm32_port_resume                   Resume port             */
/*    _ux_hcd_stm32_port_status_get               Get port status         */
/*    _ux_hcd_stm32_port_suspend                  Suspend port            */
/*    _ux_hcd_stm32_power_down_port               Power down port         */
/*    _ux_hcd_stm32_power_on_port                 Power on port           */
/*    _ux_hcd_stm32_request_transfer              Request transfer        */
/*    _ux_hcd_stm32_transfer_abort                Abort transfer          */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Host Stack                                                          */
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
UINT  _ux_hcd_stm32_entry(UX_HCD *hcd, UINT function, VOID *parameter)
{

UINT                status;
UX_HCD_STM32       *hcd_stm32;
UX_INTERRUPT_SAVE_AREA


    /* Check the status of the controller.  */
    if (hcd -> ux_hcd_status == UX_UNUSED)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_CONTROLLER_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONTROLLER_UNKNOWN, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_CONTROLLER_UNKNOWN);
    }

    /* Get the pointer to the STM32 HCD.  */
    hcd_stm32 =  (UX_HCD_STM32 *) hcd -> ux_hcd_controller_hardware;

    /* look at the function and route it.  */
    switch(function)
    {

    case UX_HCD_DISABLE_CONTROLLER:

        status =  _ux_hcd_stm32_controller_disable(hcd_stm32);
        break;


    case UX_HCD_GET_PORT_STATUS:

        status =  _ux_hcd_stm32_port_status_get(hcd_stm32, (ULONG) parameter);
        break;


    case UX_HCD_ENABLE_PORT:

        status =  _ux_hcd_stm32_port_enable(hcd_stm32, (ULONG) parameter);
        break;


    case UX_HCD_DISABLE_PORT:

        status =  _ux_hcd_stm32_port_disable(hcd_stm32, (ULONG) parameter);
        break;


    case UX_HCD_POWER_ON_PORT:

        status =  _ux_hcd_stm32_power_on_port(hcd_stm32, (ULONG) parameter);
        break;


    case UX_HCD_POWER_DOWN_PORT:

        status =  _ux_hcd_stm32_power_down_port(hcd_stm32, (ULONG) parameter);
        break;


    case UX_HCD_SUSPEND_PORT:

        status =  _ux_hcd_stm32_port_suspend(hcd_stm32, (ULONG) parameter);
        break;


    case UX_HCD_RESUME_PORT:

        status =  _ux_hcd_stm32_port_resume(hcd_stm32, (UINT) parameter);
        break;


    case UX_HCD_RESET_PORT:

        status =  _ux_hcd_stm32_port_reset(hcd_stm32, (ULONG) parameter);
        break;


    case UX_HCD_GET_FRAME_NUMBER:

        status =  _ux_hcd_stm32_frame_number_get(hcd_stm32, (ULONG *) parameter);
        break;


    case UX_HCD_TRANSFER_REQUEST:

        status =  _ux_hcd_stm32_request_transfer(hcd_stm32, (UX_TRANSFER *) parameter);
        break;


    case UX_HCD_TRANSFER_ABORT:

        status =  _ux_hcd_stm32_transfer_abort(hcd_stm32, (UX_TRANSFER *) parameter);
        break;


    case UX_HCD_CREATE_ENDPOINT:

        status =  _ux_hcd_stm32_endpoint_create(hcd_stm32, (UX_ENDPOINT*) parameter);
        break;

    case UX_HCD_DESTROY_ENDPOINT:

        status =  _ux_hcd_stm32_endpoint_destroy(hcd_stm32, (UX_ENDPOINT*) parameter);
        break;

    case UX_HCD_RESET_ENDPOINT:

        status =  _ux_hcd_stm32_endpoint_reset(hcd_stm32, (UX_ENDPOINT*) parameter);
        break;

    case UX_HCD_PROCESS_DONE_QUEUE:

        /* Process periodic queue.  */
        _ux_hcd_stm32_periodic_schedule(hcd_stm32);

        /* Reset the SOF flag.  */
        UX_DISABLE
        hcd_stm32 -> ux_hcd_stm32_controller_flag &= ~UX_HCD_STM32_CONTROLLER_FLAG_SOF;
        UX_RESTORE

        status =  UX_SUCCESS;
        break;

    case UX_HCD_UNINITIALIZE:

        /* free HCD resources */
        if (hcd_stm32 != UX_NULL)
        {
          _ux_utility_memory_free(hcd_stm32 -> ux_hcd_stm32_ed_list);
          _ux_utility_memory_free(hcd_stm32);
        }

        status =  UX_SUCCESS;
        break;

    default:

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Unknown request, return an error.  */
        status =  UX_FUNCTION_NOT_SUPPORTED;
        break;

    }

    /* Return completion status.  */
    return(status);
}

