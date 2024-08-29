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
/*    _ux_device_class_ccid_hardware_error                PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function indicates hardware error of the USB CCID device.      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    ccid                                  Pointer to ccid instance      */
/*    slot                                  Slot inserted                 */
/*    error                                 Error code                    */
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
UINT _ux_device_class_ccid_hardware_error(UX_DEVICE_CLASS_CCID *ccid, ULONG slot, ULONG error)
{

UX_DEVICE_CLASS_CCID_SLOT                           *ccid_slot;
UX_DEVICE_CLASS_CCID_RUNNER                         *runner;
UX_DEVICE_CLASS_CCID_RDR_TO_PC_SLOT_STATUS_HEADER   *rsp;

    /* Sanity check.  */
    if (slot >= ccid -> ux_device_class_ccid_parameter.ux_device_class_ccid_max_n_slots)
        return(UX_INVALID_PARAMETER);

    /* Get slot instance.  */
    ccid_slot  = ccid -> ux_device_class_ccid_slots;
    ccid_slot += slot;

    /* Lock states.  */
    _ux_device_mutex_on(&ccid -> ux_device_class_ccid_mutex);

    /* Check error.  */
    if (!(ccid_slot -> ux_device_class_ccid_slot_flags & UX_DEVICE_CLASS_CCID_FLAG_HW_ERROR))
    {

        /* Save error and error code.  */
        ccid_slot -> ux_device_class_ccid_slot_flags |= UX_DEVICE_CLASS_CCID_FLAG_HW_ERROR;
        ccid_slot -> ux_device_class_ccid_slot_hw_error = (UCHAR)error;
        ccid_slot -> ux_device_class_ccid_slot_hw_error_seq = 0;

        /* Slot deactivated.  */
        ccid_slot -> ux_device_class_ccid_slot_icc_status = UX_DEVICE_CLASS_CCID_ICC_INACTIVE;

        /* Check if command is pending, update response buffer.  */
        if ((signed char)ccid_slot -> ux_device_class_ccid_slot_runner >= 0)
        {

            /* Get running things.  */
            runner = ccid -> ux_device_class_ccid_runners;
            runner += ccid_slot -> ux_device_class_ccid_slot_runner;
            rsp = (UX_DEVICE_CLASS_CCID_RDR_TO_PC_SLOT_STATUS_HEADER *)
                                runner -> ux_device_class_ccid_runner_response;

            /* Response: (1,1,HW_ERROR).  */
            rsp -> bStatus = UX_DEVICE_CLASS_CCID_SLOT_STATUS(1, 1);
            rsp -> bError = UX_DEVICE_CLASS_CCID_HW_ERROR;
        }
    }

    /* Notify if interrupt endpoint exists.  */
    if (ccid -> ux_device_class_ccid_endpoint_notify)
    {
        ccid_slot -> ux_device_class_ccid_slot_flags |= UX_DEVICE_CLASS_CCID_FLAG_NOTIFY_CHANGE;

        /* Unlock states.  */
        _ux_device_mutex_off(&ccid -> ux_device_class_ccid_mutex);

        /* Wakeup interrupt notification.  */
        _ux_device_semaphore_put(&ccid -> ux_device_class_ccid_notify_semaphore);
        return(UX_SUCCESS);
    }

    /* Unlock states.  */
    _ux_device_mutex_off(&ccid -> ux_device_class_ccid_mutex);

    /* Return transfer status.  */
    return(UX_SUCCESS);
}
