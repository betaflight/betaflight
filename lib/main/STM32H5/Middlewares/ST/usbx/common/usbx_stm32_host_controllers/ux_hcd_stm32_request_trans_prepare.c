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
/*    _ux_hcd_stm32_request_trans_prepare                 PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*     Prepare for data transactions.                                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_stm32                             Pointer to STM32 controller   */
/*    ed                                    Pointer to STM32 ED           */
/*    transfer                              Pointer to transfer request   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_copy               Copy memory                   */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 Controller Driver                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Initial Version 6.1.12        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_stm32_request_trans_prepare(UX_HCD_STM32 *hcd_stm32, UX_HCD_STM32_ED *ed, UX_TRANSFER *transfer)
{

    /* Save transfer data pointer.  */
    ed -> ux_stm32_ed_data = transfer -> ux_transfer_request_data_pointer;

    /* If DMA not enabled, nothing to do.  */
    if (!hcd_stm32 -> hcd_handle -> Init.dma_enable)
        return(UX_SUCCESS);

    /* If there is no data, nothing to do.  */
    if (transfer -> ux_transfer_request_requested_length == 0)
        return(UX_SUCCESS);

    /* If transfer buffer aligned, nothing to do.  */
    if (((ALIGN_TYPE)ed -> ux_stm32_ed_data & 0x3UL) == 0)
        return(UX_SUCCESS);

    /* Allocate aligned data buffer for transfer.  */
    ed -> ux_stm32_ed_data = _ux_utility_memory_allocate(UX_NO_ALIGN,
                            UX_CACHE_SAFE_MEMORY,
                            transfer -> ux_transfer_request_requested_length);
    if (ed -> ux_stm32_ed_data == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* For data IN it's done.  */
    if (ed -> ux_stm32_ed_dir)
        return(UX_SUCCESS);

    /* For data OUT, copy buffer.  */
    _ux_utility_memory_copy(ed -> ux_stm32_ed_data,
                            transfer -> ux_transfer_request_data_pointer,
                            transfer -> ux_transfer_request_requested_length); /* Use case of memcpy is verified.  */

    /* Done.  */
    return(UX_SUCCESS);
}
