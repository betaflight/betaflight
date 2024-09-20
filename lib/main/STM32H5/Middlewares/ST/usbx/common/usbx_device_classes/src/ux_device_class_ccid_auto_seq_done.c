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
/**   Device CCID Class                                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_ccid.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_ccid_auto_seq_done                 PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function indicates card auto sequencing done of the USB CCID   */
/*    device.                                                             */
/*                                                                        */
/*    Note it must be used after _icc_insert with seq_start TRUE.         */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    ccid                                  Pointer to ccid instance      */
/*    slot                                  Slot inserted                 */
/*    icc_status                            Final card status             */
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
/*    USBX Source Code                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  04-25-2022     Chaoqiong Xiao           Initial Version 6.1.11        */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_ccid_auto_seq_done(UX_DEVICE_CLASS_CCID *ccid, ULONG slot, ULONG icc_status)
{

UX_DEVICE_CLASS_CCID_SLOT       *ccid_slot;

    /* Sanity check.  */
    if (slot >= ccid -> ux_device_class_ccid_parameter.ux_device_class_ccid_max_n_slots)
        return(UX_INVALID_PARAMETER);

    /* Get slot instance.  */
    ccid_slot  = ccid -> ux_device_class_ccid_slots;
    ccid_slot += slot;

    /* Lock states.  */
    _ux_device_mutex_on(&ccid -> ux_device_class_ccid_mutex);

    /* Check sequencing.  */
    if (ccid_slot -> ux_device_class_ccid_slot_flags & UX_DEVICE_CLASS_CCID_FLAG_AUTO_SEQUENCING)
    {
        ccid_slot -> ux_device_class_ccid_slot_icc_status = (UCHAR)icc_status;
        ccid_slot -> ux_device_class_ccid_slot_flags &= (UCHAR)~UX_DEVICE_CLASS_CCID_FLAG_AUTO_SEQUENCING;
    }

    /* Unlock states.  */
    _ux_device_mutex_off(&ccid -> ux_device_class_ccid_mutex);

    /* Return transfer status.  */
    return(UX_SUCCESS);
}
