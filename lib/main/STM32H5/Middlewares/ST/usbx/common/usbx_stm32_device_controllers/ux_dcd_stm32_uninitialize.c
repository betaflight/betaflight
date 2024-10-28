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


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_dcd_stm32_uninitialize                          PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function uninitializes the USB device controller of the STM32  */
/*    microcontroller from ST.                                            */
/*                                                                        */
/*    Note: only software structures are uninitialized. STM32 HAL APIs    */
/*    must be used to uninitialize controller hardware BEFORE the         */
/*    function is invoked.                                                */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd                                   Address of DCD, not used      */
/*    parameter                             Parameter, STM32 HAL PCD      */
/*                                            pointer is expected         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Device Stack                                                   */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  09-30-2020     Chaoqiong Xiao           Initial Version 6.1           */
/*                                                                        */
/**************************************************************************/
UINT  _ux_dcd_stm32_uninitialize(ULONG dcd_io, ULONG parameter)
{

UX_SLAVE_DCD            *dcd;
UX_DCD_STM32            *dcd_stm32;


    UX_PARAMETER_NOT_USED(dcd_io);

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Set the state of the controller to HALTED now.  */
    dcd -> ux_slave_dcd_status =  UX_DCD_STATUS_HALTED;

    /* Get controller driver.  */
    dcd_stm32 = (UX_DCD_STM32 *)dcd -> ux_slave_dcd_controller_hardware;

    /* Check parameter.  */
    if ((ULONG)dcd_stm32 -> pcd_handle == parameter)
    {
        _ux_utility_memory_free(dcd_stm32);
        dcd -> ux_slave_dcd_controller_hardware = UX_NULL;
        return(UX_SUCCESS);
    }

    /* Parameter not correct.  */
    return(UX_ERROR);
}
