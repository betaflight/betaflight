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
#include "ux_utility.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                                RELEASE       */
/*                                                                        */
/*    _ux_dcd_stm32_transfer_request                       PORTABLE C     */
/*                                                            6.1.10      */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will initiate a transfer to a specific endpoint.      */
/*    If the endpoint is IN, the endpoint register will be set to accept  */
/*    the request.                                                        */
/*                                                                        */
/*    If the endpoint is IN, the endpoint FIFO will be filled with the    */
/*    buffer and the endpoint register set.                               */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_stm32                             Pointer to device controller  */
/*    transfer_request                      Pointer to transfer request   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    HAL_PCD_EP_Transmit                   Transmit data                 */
/*    HAL_PCD_EP_Receive                    Receive data                  */
/*    _ux_utility_semaphore_get             Get semaphore                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 Controller Driver                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_dcd_stm32_transfer_abort(UX_DCD_STM32 *dcd_stm32, UX_SLAVE_TRANSFER *transfer_request)
{

UX_SLAVE_ENDPOINT       *endpoint;


    /* Get the pointer to the logical endpoint from the transfer request.  */
    endpoint =  transfer_request -> ux_slave_transfer_request_endpoint;

    HAL_PCD_EP_Abort(dcd_stm32 -> pcd_handle, endpoint->ux_slave_endpoint_descriptor.bEndpointAddress);
    HAL_PCD_EP_Flush(dcd_stm32 -> pcd_handle, endpoint->ux_slave_endpoint_descriptor.bEndpointAddress);

    /* No semaphore put here since it's already done in stack.  */

    /* Return to caller with success.  */
    return(UX_SUCCESS);
}
