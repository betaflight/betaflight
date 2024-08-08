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

extern VOID _ux_host_class_storage_read_initialize(UX_HOST_CLASS_STORAGE *storage,
            ULONG sector_start, ULONG sector_count);

extern VOID _ux_host_class_storage_write_initialize(UX_HOST_CLASS_STORAGE *storage,
            ULONG sector_start, ULONG sector_count);


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_storage_read_write_run               PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function runs state machine to read or write one or more       */
/*    logical sector on a selected media of a storage function.           */
/*                                                                        */
/*    It's valid only in standalone mode.                                 */
/*                                                                        */
/*    Note it must be used in case following conditions are true:         */
/*    - storage insstance is live                                         */
/*    - storage is locked for specific LUN                                */
/*    - the main storage state must be idle                               */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    storage                               Pointer to storage class      */
/*    read_write                            Set to UX_TRUE to read        */
/*    sector_start                          Starting sector               */
/*    sector_count                          Number of sectors to read     */
/*    data_pointer                          Pointer to data to read       */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*    UX_STATE_WAIT                         States started/in progress    */
/*    UX_STATE_NEXT                         Read/write is done, next call */
/*                                          starts read/write again.      */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_class_storage_read_initialize                              */
/*                                          Initialize the CBW            */
/*    _ux_host_class_storage_write_initialize                              */
/*                                          Initialize the CBW            */
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
UINT  _ux_host_class_storage_read_write_run(UX_HOST_CLASS_STORAGE *storage,
                ULONG read_write,
                ULONG sector_start, ULONG sector_count, UCHAR *data_pointer)
{

#if defined UX_HOST_CLASS_STORAGE_STATE_CHECK_ENABLE
UX_INTERRUPT_SAVE_AREA

    /* Check states - storage must be live, locked and main state machine idle.  */
    UX_DISABLE
    if (storage -> ux_host_class_storage_state != UX_HOST_CLASS_INSTANCE_LIVE ||
        (storage -> ux_host_class_storage_flags & UX_HOST_CLASS_STORAGE_FLAG_LOCK) == 0 ||
        storage -> ux_host_class_storage_state_state != UX_STATE_IDLE)
    {
        UX_RESTORE

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_NOT_READY);

        /* The state is not ready for read.  */
        storage -> ux_host_class_storage_status = UX_TRANSFER_NOT_READY;
        storage -> ux_host_class_storage_op_state = UX_STATE_RESET;
        return(UX_STATE_ERROR);
    }
    UX_RESTORE
#endif

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_STORAGE_MEDIA_READ, storage, sector_start, sector_count, data_pointer, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    switch(storage -> ux_host_class_storage_op_state)
    {
    case UX_STATE_IDLE:     /* Fall through.  */
    case UX_STATE_ERROR:    /* Fall through.  */
    case UX_STATE_RESET:

        /* Initialize CBW.  */
        if (read_write)
            _ux_host_class_storage_read_initialize(storage, sector_start, sector_count);
        else
            _ux_host_class_storage_write_initialize(storage, sector_start, sector_count);

        /* Initialize for transport.  */
        UX_HOST_CLASS_STORAGE_TRANS_STATE_RESET(storage);
        storage -> ux_host_class_storage_trans_data = data_pointer;

        /* Next : wait  */
        storage -> ux_host_class_storage_op_state = UX_STATE_WAIT;

        /* Fall through.  */
    case UX_STATE_WAIT:

        /* Run tasks, including transport task.  */
        _ux_system_host_tasks_run();

        /* Fatal error.  */
        if (storage -> ux_host_class_storage_op_state < UX_STATE_IDLE)
            return(UX_STATE_EXIT);

        /* It's done with/without error.  */
        if (storage -> ux_host_class_storage_op_state <= UX_STATE_NEXT)
            return(UX_STATE_NEXT);

        /* Wait.  */
        return(UX_STATE_WAIT);

    /* Unexpected states.  */
    default:
        storage -> ux_host_class_storage_op_state = UX_STATE_RESET;
        break;
    }

    /* Return fatal exit state status.  */
    return(UX_STATE_EXIT);
}
#endif
