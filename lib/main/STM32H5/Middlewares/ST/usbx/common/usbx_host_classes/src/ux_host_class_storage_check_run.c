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
/**   Storage Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_storage.h"
#include "ux_host_stack.h"


#if defined(UX_HOST_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_storage_check_run                    PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function runs state machine to check and mount/unmount storage */
/*    media for current locked logic unit number (LUN).                   */
/*                                                                        */
/*    It's valid only in standalone mode.                                 */
/*    It's non-blocking.                                                  */
/*                                                                        */
/*    Note it must be used in case following conditions are true:         */
/*    - storage insstance is live                                         */
/*    - storage is locked for specific LUN                                */
/*    - if check is not started yet the main storage state must be idle   */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    storage                               Pointer to storage class      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion State Status                                             */
/*    UX_STATE_WAIT                         States started/in progress    */
/*    UX_STATE_NEXT                         Check is done, next call will */
/*                                          start check again.            */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_system_error_handler              The error trap                */
/*    _ux_system_tasks_run                  Run USB system tasks          */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Storage Class                                                       */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_check_run(UX_HOST_CLASS_STORAGE *storage)
{

#if defined UX_HOST_CLASS_STORAGE_STATE_CHECK_ENABLE
UX_INTERRUPT_SAVE_AREA

    /* Check states - storage must be live, locked and main state machine idle.  */
    UX_DISABLE
    if (storage -> ux_host_class_storage_state != UX_HOST_CLASS_INSTANCE_LIVE ||
        (storage -> ux_host_class_storage_flags & UX_HOST_CLASS_STORAGE_FLAG_LOCK) == 0 ||
        (storage -> ux_host_class_storage_op_state != UX_STATE_WAIT &&
         storage -> ux_host_class_storage_state_state != UX_STATE_IDLE))
    {
        UX_RESTORE

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_NOT_READY);

        /* The state is not ready for check operation.  */
        storage -> ux_host_class_storage_status = UX_TRANSFER_NOT_READY;
        storage -> ux_host_class_storage_op_state = UX_STATE_RESET;
        return(UX_STATE_ERROR);
    }
    UX_RESTORE
#endif

    switch(storage -> ux_host_class_storage_op_state)
    {
    case UX_STATE_IDLE:
    case UX_STATE_ERROR:
    case UX_STATE_RESET:

        /* Setup states for immediate LUN check.  */
        storage -> ux_host_class_storage_flags |=
                                    UX_HOST_CLASS_STORAGE_FLAG_CHECK_CURRENT;
        storage -> ux_host_class_storage_check_lun =
                                    (UCHAR)storage -> ux_host_class_storage_lun;
        storage -> ux_host_class_storage_state_state =
                                        UX_HOST_CLASS_STORAGE_STATE_TEST_READY;

        /* Next : wait  */
        storage -> ux_host_class_storage_op_state = UX_STATE_WAIT;

        /* Fall through.  */
    case UX_STATE_WAIT:

        /* Run tasks, including transport task.  */
        _ux_system_host_tasks_run();

        /* In case state is not idle, check if it changes back.  */
        if (storage -> ux_host_class_storage_state_state == UX_STATE_IDLE)
        {
            storage -> ux_host_class_storage_op_state = UX_STATE_IDLE;
            return(UX_STATE_NEXT);
        }

        /* Keep waiting.  */
        return(UX_STATE_WAIT);

    /* Unexpected states.  */
    default:
        storage -> ux_host_class_storage_op_state = UX_STATE_RESET;
        break;
    }

    /* We should never be here.  */
    return(UX_STATE_EXIT);
}
#endif
