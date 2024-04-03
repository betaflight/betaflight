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
/*    _ux_hcd_stm32_port_reset                            PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will reset a specific port attached to the root       */
/*    HUB.                                                                */
/*                                                                        */
/*    Note since ThreadX delay is used, the function must not be used in  */
/*    interrupts.                                                         */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_stm32                             Pointer to STM32 controller   */
/*    port_index                            Port index                    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_hcd_stm32_hcor_register_read      Read STM32 register           */
/*    _ux_hcd_stm32_hcor_register_write     Write STM32 register          */
/*    _ux_utility_delay_ms                  Delay                         */
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
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_stm32_port_reset(UX_HCD_STM32 *hcd_stm32, ULONG port_index)
{

    /* Check to see if this port is valid on this controller.  On STM32, there is only one. */
    if (port_index != 0)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_PORT_INDEX_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_PORT_INDEX_UNKNOWN, port_index, 0, 0, UX_TRACE_ERRORS, 0, 0)

#if defined(UX_HOST_STANDALONE)
        return(UX_STATE_ERROR);
#else
        return(UX_PORT_INDEX_UNKNOWN);
#endif /* defined(UX_HOST_STANDALONE) */
    }

    /* Ensure that the downstream port has a device attached. It is unnatural
       to perform a port reset if there is no device.  */
    if ((hcd_stm32 -> ux_hcd_stm32_controller_flag & UX_HCD_STM32_CONTROLLER_FLAG_DEVICE_ATTACHED) == 0)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_NO_DEVICE_CONNECTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_NO_DEVICE_CONNECTED, port_index, 0, 0, UX_TRACE_ERRORS, 0, 0)

#if defined(UX_HOST_STANDALONE)
        return(UX_STATE_ERROR);
#else
        return(UX_NO_DEVICE_CONNECTED);
#endif /* defined(UX_HOST_STANDALONE) */
    }

#if defined(UX_HOST_STANDALONE)
    /* There is no way for non-blocking reset in HCD, just do blocking operation here.  */
    HAL_HCD_ResetPort(hcd_stm32 -> hcd_handle);
    return(UX_STATE_NEXT);
#else
    HAL_HCD_ResetPort(hcd_stm32 -> hcd_handle);

    /* This function should never fail.  */
    return(UX_SUCCESS);
#endif /* defined(UX_HOST_STANDALONE) */

}

