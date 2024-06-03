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
/*    _ux_dcd_stm32_initialize                            PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function initializes the USB device controller of the STM32    */
/*    microcontroller from ST.                                            */
/*                                                                        */
/*    Note: only software structures that are necessary for STM32 HAL to  */
/*    work is initialized, STM32 HAL APIs should be used to initialize    */
/*    controller hardware AFTER the function is invoked.                  */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_io                                Address of DCD, not used      */
/*    parameter                             Parameter, STM32 HAL PCD      */
/*                                            pointer is expected         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    HAL_PCD_Init                          Initialize LL driver          */
/*    _ux_utility_memory_allocate           Allocate memory               */
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_dcd_stm32_initialize(ULONG dcd_io, ULONG parameter)
{

UX_SLAVE_DCD            *dcd;
UX_DCD_STM32            *dcd_stm32;


    UX_PARAMETER_NOT_USED(dcd_io);

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* The controller initialized here is of STM32 type.  */
    dcd -> ux_slave_dcd_controller_type =  UX_DCD_STM32_SLAVE_CONTROLLER;

    /* Allocate memory for this STM32 DCD instance.  */
    dcd_stm32 =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_DCD_STM32));

    /* Check if memory was properly allocated.  */
    if(dcd_stm32 == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Set the pointer to the STM32 DCD.  */
    dcd -> ux_slave_dcd_controller_hardware =  (VOID *) dcd_stm32;

    /* Set the generic DCD owner for the STM32 DCD.  */
    dcd_stm32 -> ux_dcd_stm32_dcd_owner =  dcd;

    /* Initialize the function collector for this DCD.  */
    dcd -> ux_slave_dcd_function =  _ux_dcd_stm32_function;

    dcd_stm32 -> pcd_handle = (PCD_HandleTypeDef *)parameter;

    /* Set the state of the controller to OPERATIONAL now.  */
    dcd -> ux_slave_dcd_status =  UX_DCD_STATUS_OPERATIONAL;

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

