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
/*    _ux_host_class_storage_lock                         PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function locks storage for further operations.                 */
/*                                                                        */
/*    It's valid only in standalone mode.                                 */
/*    It's non-blocking if wait is zero, otherwise it blocks.             */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    storage_media                         Pointer to storage to operate */
/*    wait                                  Wait option                   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*    UX_TIMEOUT                            Time out                      */
/*    UX_REENTRY                            Reentry in storage task       */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
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
UINT    _ux_host_class_storage_lock(UX_HOST_CLASS_STORAGE *storage, ULONG wait)
{
UX_INTERRUPT_SAVE_AREA
ULONG           t0, t1;

    t0 = _ux_utility_time_get();
    while(1)
    {
        UX_DISABLE

        /* Check instance lock.  */
        if (storage -> ux_host_class_storage_flags & UX_HOST_CLASS_STORAGE_FLAG_LOCK)
        {

            /* Check once, just return busy.  */
            if (wait == 0)
            {
                UX_RESTORE
                return(UX_TIMEOUT);
            }

            /* Check timeout.  */
            if (wait != UX_WAIT_FOREVER)
            {
                t1 = _ux_utility_time_get();
                t1 = _ux_utility_time_elapsed(t0, t1);
                if (t1 >= wait)
                {
                    UX_RESTORE
                    return(UX_TIMEOUT);
                }
            }
        }
        else
        {
            /* It's free, time to lock it.  */
            break;
        }

        /* Check task reentry.  */
        if (storage -> ux_host_class_storage_flags & UX_HOST_CLASS_STORAGE_FLAG_PROTECT)
        {
            UX_RESTORE

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_REENTRY);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_REENTRY, storage, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_REENTRY);
        }
        UX_RESTORE

        /* Run stack tasks.  */
        _ux_system_host_tasks_run();
    }

    /* Lock storage.  */
    storage -> ux_host_class_storage_flags |= UX_HOST_CLASS_STORAGE_FLAG_LOCK;

    /* Stop main state machine.  */
    storage -> ux_host_class_storage_state_state = UX_STATE_IDLE;

    /* Reset operation state machine.  */
    storage -> ux_host_class_storage_op_state = UX_STATE_RESET;

    UX_RESTORE
    return(UX_SUCCESS);
}
#endif
