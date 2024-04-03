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
/**                                                                       */
/** USBX Component                                                        */
/**                                                                       */
/**   Device DFU Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_dfu.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_dfu_state_sync                     PORTABLE C      */
/*                                                           6.1.6        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function syncs the USB DFU device state.                       */
/*    This allows application to move DFU state out of follow:            */
/*    - dfuDNBUSY -> dfuDNLOAD-SYNC                                       */
/*    - dfuMANIFEST -> dfuMANIFEST-SYNC                                   */
/*    Other states will be kept.                                          */
/*    Note the dfuDNBUSY and dfuMANIFEST is involved by returning busy    */
/*    status in application ux_slave_class_dfu_get_status callback, and   */
/*    needs application to issue ux_device_class_dfu_state_sync to go     */
/*    out of the state.                                                   */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dfu                                   Pointer to DFU instance       */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application Code                                                    */
/*    USBX Source Code                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  04-02-2021     Chaoqiong Xiao           Initial Version 6.1.6         */
/*                                                                        */
/**************************************************************************/
VOID _ux_device_class_dfu_state_sync(UX_SLAVE_CLASS_DFU *dfu)
{
UX_INTERRUPT_SAVE_AREA

    UX_PARAMETER_NOT_USED(dfu);
    UX_DISABLE
    switch(_ux_system_slave -> ux_system_slave_device_dfu_state_machine)
    {
    case UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_DNBUSY:
        _ux_system_slave -> ux_system_slave_device_dfu_state_machine =
                            UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_DNLOAD_SYNC;
        break;
    case UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_MANIFEST:
        _ux_system_slave -> ux_system_slave_device_dfu_state_machine =
                            UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_MANIFEST_SYNC;
        break;
    default:
        break;
    }
    UX_RESTORE
}
