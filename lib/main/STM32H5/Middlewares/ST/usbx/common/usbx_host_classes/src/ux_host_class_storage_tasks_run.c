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


static inline VOID _ux_host_class_storage_inst_tasks_run(UX_HOST_CLASS_STORAGE *storage);

static inline UINT _ux_host_class_storage_lun_is_removable(UX_HOST_CLASS_STORAGE *storage);
static inline UINT _ux_host_class_storage_lun_type_is_known(UX_HOST_CLASS_STORAGE *storage);

static inline VOID _ux_host_class_storage_max_lun_save(UX_HOST_CLASS_STORAGE *storage);
static inline UINT _ux_host_class_storage_inquiry_save(UX_HOST_CLASS_STORAGE *storage);
static inline VOID _ux_host_class_storage_format_cap_save(UX_HOST_CLASS_STORAGE *storage);
static inline VOID _ux_host_class_storage_capacity_save(UX_HOST_CLASS_STORAGE *storage);
static inline VOID _ux_host_class_storage_unit_ready_check(UX_HOST_CLASS_STORAGE *storage);

static inline VOID _ux_host_class_storage_lun_media_insert(UX_HOST_CLASS_STORAGE *storage);

static inline UINT _ux_host_class_storage_transport_sense_check(UX_HOST_CLASS_STORAGE *storage);

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_storage_tasks_run                    PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is awaken every 2 seconds to check if there was a     */
/*    device insertion on a specific media. This is the only way we can   */
/*    remount a media after the storage instance has opened the media to  */
/*    UX_MEDIA (default FileX) and the media is either not present        */
/*    or was removed and is being re-inserted.                            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    storage                               Pointer to storage instance   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_class_storage_device_reset   Reset device                  */
/*    _ux_host_class_storage_media_mount    Mount the media               */
/*    _ux_host_class_storage_unit_ready_test                              */
/*                                          Test for unit ready           */
/*    _ux_host_class_storage_media_characteristics_get                    */
/*                                          Get media characteristics     */
/*    _ux_host_class_storage_media_format_capacity_get                    */
/*                                          Get media format capacity     */
/*    _ux_utility_memory_free               Free memory block             */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Host Stack                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved internal logic,    */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_tasks_run(UX_HOST_CLASS *storage_class)
{
UX_HOST_CLASS_STORAGE           *storage;

    /* Validate class entry.  */
    if (storage_class -> ux_host_class_status != UX_USED ||
        storage_class -> ux_host_class_entry_function != _ux_host_class_storage_entry)
        return(UX_STATE_IDLE);

    /* Run for class instances.  */
    storage = (UX_HOST_CLASS_STORAGE *)storage_class -> ux_host_class_first_instance;
    while(storage)
    {

        /* Run tasks for each storage instance.  */
        storage -> ux_host_class_storage_flags |= UX_HOST_CLASS_STORAGE_FLAG_PROTECT;
        _ux_host_class_storage_inst_tasks_run(storage);
        storage -> ux_host_class_storage_flags &= ~UX_HOST_CLASS_STORAGE_FLAG_PROTECT;
        storage = storage -> ux_host_class_storage_next_instance;
    }
    return(UX_STATE_WAIT);
}

static inline VOID _ux_host_class_storage_inst_tasks_run(UX_HOST_CLASS_STORAGE *storage)
{
UX_INTERRUPT_SAVE_AREA
UCHAR           state;
ULONG           tick_now, tick_elapsed;
UINT            status;
UX_TRANSFER     *trans;
UX_INTERFACE    *interface_ptr;
INT             immediate_state;

    /* If storage not live, start initialize.  */
    if (storage -> ux_host_class_storage_state == UX_HOST_CLASS_INSTANCE_MOUNTING)
    {
        if (storage -> ux_host_class_storage_state_state == UX_STATE_RESET)
        {

            /* Start initialize sequence Delay() - GetMaxLUN() - Inquiry() - GetFormatCapacity().  */
            storage -> ux_host_class_storage_state_state = UX_HOST_CLASS_STORAGE_STATE_DELAY_WAIT;
            storage -> ux_host_class_storage_state_next = UX_HOST_CLASS_STORAGE_STATE_MAX_LUN_GET;
            storage -> ux_host_class_storage_delay_start = _ux_utility_time_get();
            storage -> ux_host_class_storage_delay_ms =
                                UX_MS_TO_TICK_NON_ZERO(UX_HOST_CLASS_STORAGE_DEVICE_INIT_DELAY);
            return;
        }

        /* OK to process states.  */
    }
    else if (storage -> ux_host_class_storage_state != UX_HOST_CLASS_INSTANCE_LIVE)
    {

        /* No need to process states.  */
        storage -> ux_host_class_storage_state_state = UX_STATE_RESET;
        return;
    }

    /* Handle read/write states.  */
    if ((storage -> ux_host_class_storage_op_state == UX_STATE_WAIT) &&
        (storage -> ux_host_class_storage_flags &
         UX_HOST_CLASS_STORAGE_FLAG_CHECK_CURRENT) == 0)
    {

        /* Run transport.  */
        status = _ux_host_class_storage_transport_run(storage);

        /* Fatal error.  */
        if (status < UX_STATE_IDLE)
        {
            storage -> ux_host_class_storage_op_state = UX_STATE_RESET;
            return;
        }

        /* Done with/without error.  */
        if (status <= UX_STATE_NEXT)
        {
            storage -> ux_host_class_storage_op_state = UX_STATE_IDLE;
            return;
        }

        /* Keep waiting.  */
        /* Main states are frozen in this case.  */
        return;
    }

    /* Handle main states.  */
    immediate_state = UX_TRUE;
    while(immediate_state)
    {

        /* Get current state.  */
        /* Initial check: delay()-GetMaxLUN()-Inquiry()-GetFormatCap()
            process break on any error.  */
        /* Regular check: delay()-TestReady()-Inquiry()-GetFormatCap()
            process break on any error.  */
        state = storage -> ux_host_class_storage_state_state;
        switch(state)
        {
        case UX_HOST_CLASS_STORAGE_STATE_MAX_LUN_GET:

            /* Issue GetMaxLun().  */
            status = _ux_host_class_storage_max_lun_get(storage);
            if (UX_SUCCESS != status)
            {

                /* This fails storage activation.  */
                interface_ptr = storage -> ux_host_class_storage_interface;
                _ux_host_stack_class_instance_destroy(
                    storage -> ux_host_class_storage_class, (VOID *) storage);
                interface_ptr -> ux_interface_class_instance =  (VOID *) UX_NULL;
                _ux_utility_memory_free(storage);
                return;
            }

            /* Roll back to next state.  */
            /* By default TRANSFER -> MAX_LUN_SAVE -> TEST_READY.  */
            continue;

        case UX_HOST_CLASS_STORAGE_STATE_MAX_LUN_SAVE:
            _ux_host_class_storage_max_lun_save(storage);

            /* Continue to start LUN 0 check from TEST_READY any way.  */
            storage -> ux_host_class_storage_check_lun = 0;
            storage -> ux_host_class_storage_state_state =
                                        UX_HOST_CLASS_STORAGE_STATE_TEST_READY;

            /* Fall through.  */
        case UX_HOST_CLASS_STORAGE_STATE_TEST_READY:

            /* Save the LUN for the follwing sequence to use.  */
            storage -> ux_host_class_storage_lun =
                                    storage -> ux_host_class_storage_check_lun;

            /* If storage is not live, skip to do INQUIRY.  */
            if (storage -> ux_host_class_storage_state != UX_HOST_CLASS_INSTANCE_LIVE)
            {
                storage -> ux_host_class_storage_state_state =
                                            UX_HOST_CLASS_STORAGE_STATE_INQUIRY;
                continue;
            }

            /* If LUN is not removable nor known, skip it to check next LUN.  */
            if (!_ux_host_class_storage_lun_is_removable(storage) ||
                !_ux_host_class_storage_lun_type_is_known(storage))
            {
                storage -> ux_host_class_storage_state_state =
                                        UX_HOST_CLASS_STORAGE_STATE_NEXT_LUN;
                continue;
            }

            /* Prepare TestUnitReady().  */
            _ux_host_class_storage_unit_ready_test(storage);

            /* Roll back to next - TRANSPORT - TEST_CHECK.  */
            continue;

        case UX_HOST_CLASS_STORAGE_STATE_TEST_CHECK:
            _ux_host_class_storage_unit_ready_check(storage);

            /* Roll back to next - possible NEXT_LUN/INQUIRY/TEST_READY.  */
            continue;

        case UX_HOST_CLASS_STORAGE_STATE_INQUIRY:
            status = _ux_host_class_storage_media_characteristics_get(storage);
            if (status != UX_SUCCESS)
            {

                /* There is error, break the regular check round.  */
                storage -> ux_host_class_storage_status = status;
                storage -> ux_host_class_storage_state_state =
                                        UX_HOST_CLASS_STORAGE_STATE_CHECK_DONE;
            }

            /* Roll back to normal next - TRANSPORT - INQUIRY_SAVE.  */
            continue;

        case UX_HOST_CLASS_STORAGE_STATE_INQUIRY_SAVE:
            status = _ux_host_class_storage_inquiry_save(storage);
            if (status != UX_SUCCESS)
            {

                /* Check next LUN.  */
                storage -> ux_host_class_storage_state_state = UX_HOST_CLASS_STORAGE_STATE_NEXT_LUN;
                continue;
            }

            /* Next : GetFormatCapacity().  */
            storage -> ux_host_class_storage_state_state = UX_HOST_CLASS_STORAGE_STATE_FORMAT_CAP_GET;

            /* Fall through.  */
        case UX_HOST_CLASS_STORAGE_STATE_FORMAT_CAP_GET:
            status = _ux_host_class_storage_media_format_capacity_get(storage);
            if (status != UX_SUCCESS)
            {

                /* This error breaks regular check round.  */
                storage -> ux_host_class_storage_status = status;
                storage -> ux_host_class_storage_state_state =
                                        UX_HOST_CLASS_STORAGE_STATE_CHECK_DONE;
            }

            /* Roll back to normal next - TRANSPORT - FORMAT_CAP_SAVE.  */
            continue;

        case UX_HOST_CLASS_STORAGE_STATE_FORMAT_CAP_SAVE:
            _ux_host_class_storage_format_cap_save(storage);

            /* Next : GetCapacity()  */
            storage -> ux_host_class_storage_state_state = UX_HOST_CLASS_STORAGE_STATE_CAP_GET;

            /* Fall through.  */
        case UX_HOST_CLASS_STORAGE_STATE_CAP_GET:
            status = _ux_host_class_storage_media_capacity_get(storage);
            if (status != UX_SUCCESS)
            {

                /* This error breaks the regular check round.  */
                storage -> ux_host_class_storage_status = status;
                storage -> ux_host_class_storage_state_state =
                                        UX_HOST_CLASS_STORAGE_STATE_CHECK_DONE;
            }

            /* Roll back to normal next - TRANSPORT - CAP_SAVE.  */
            continue;

        case UX_HOST_CLASS_STORAGE_STATE_CAP_SAVE:
            _ux_host_class_storage_capacity_save(storage);

            /* Final step for a single LUN check, going to next LUN.  */
            storage -> ux_host_class_storage_state_state = UX_HOST_CLASS_STORAGE_STATE_NEXT_LUN;
            continue;

        case UX_HOST_CLASS_STORAGE_STATE_CHECK_START:

            /* Get lock and start check sequence: ready - inquiry - formatCap - cap.  */
            storage -> ux_host_class_storage_state_state = UX_HOST_CLASS_STORAGE_STATE_LOCK_WAIT;
            storage -> ux_host_class_storage_state_next = UX_HOST_CLASS_STORAGE_STATE_TEST_READY;

            /* Fall through.  */
        case UX_HOST_CLASS_STORAGE_STATE_LOCK_WAIT:
            UX_DISABLE
            if (storage -> ux_host_class_storage_flags & UX_HOST_CLASS_STORAGE_FLAG_LOCK)
            {

                /* Locked, keep state to wait.  */
                UX_RESTORE
                return;
            }

            /* I'm locking it.  */
            storage -> ux_host_class_storage_flags |= UX_HOST_CLASS_STORAGE_FLAG_LOCK;
            UX_RESTORE

            /* Next state.  */
            storage -> ux_host_class_storage_state_state =
                                    storage -> ux_host_class_storage_state_next;
            continue;

        case UX_HOST_CLASS_STORAGE_STATE_NEXT_LUN:

            /* If it's immediate check for current LUN, we are done.  */
            if (storage -> ux_host_class_storage_flags &
                UX_HOST_CLASS_STORAGE_FLAG_CHECK_CURRENT)
            {

                /* Go back to idle state.  */
                storage -> ux_host_class_storage_flags &=
                                    ~UX_HOST_CLASS_STORAGE_FLAG_CHECK_CURRENT;
                storage -> ux_host_class_storage_state_state = UX_STATE_IDLE;
                continue;
            }

            if (storage -> ux_host_class_storage_check_lun <
                storage -> ux_host_class_storage_max_lun)
            {

                /* Check next LUN.  */
                storage -> ux_host_class_storage_check_lun ++;
                storage -> ux_host_class_storage_state_state =
                                        UX_HOST_CLASS_STORAGE_STATE_TEST_READY;
            }
            else
            {

                /* All LUN check is done.  */
                storage -> ux_host_class_storage_state_state =
                                        UX_HOST_CLASS_STORAGE_STATE_CHECK_DONE;
            }

            /* Roll back to next state.  */
            continue;

        case UX_HOST_CLASS_STORAGE_STATE_TRANSPORT:
            status = _ux_host_class_storage_transport_run(storage);

            /* Errors.  */
            if (status < UX_STATE_NEXT)
            {

                /* Normal flow is broken, if there is data buffer, free it.  */
                if (storage -> ux_host_class_storage_trans_data)
                    _ux_utility_memory_free(storage -> ux_host_class_storage_trans_data);

                /* No further operations, reset state machine.  */
                storage -> ux_host_class_storage_state_state = UX_STATE_RESET;
                storage -> ux_host_class_storage_flags &= ~UX_HOST_CLASS_STORAGE_FLAG_LOCK;
                return;
            }

            /* Next.  */
            if (status == UX_STATE_NEXT)
            {

                /* If there is sense handled, rollback to next state.  */
                if (_ux_host_class_storage_transport_sense_check(storage))
                {

                    /* Normal flow is broken, if there is data buffer, free it.  */
                    if (storage -> ux_host_class_storage_trans_data)
                        _ux_utility_memory_free(storage -> ux_host_class_storage_trans_data);
                    continue;
                }

                storage -> ux_host_class_storage_state_state =
                                    storage -> ux_host_class_storage_state_next;
                continue;
            }

            /* Wait, state no change.  */
            return;

        case UX_HOST_CLASS_STORAGE_STATE_TRANSFER:
            trans = storage -> ux_host_class_storage_trans;
            status = _ux_host_stack_transfer_run(trans);

            /* Keep waiting if not done.  */
            if (status > UX_STATE_NEXT)
                return;

            /* Check status in next state.  */
            storage -> ux_host_class_storage_state_state =
                            storage -> ux_host_class_storage_state_next;
            continue;

        case UX_HOST_CLASS_STORAGE_STATE_CHECK_DONE:

            /* If it's mounting, do immediate ready check again.  */
            if (storage -> ux_host_class_storage_state == UX_HOST_CLASS_INSTANCE_MOUNTING)
            {
                storage -> ux_host_class_storage_state = UX_HOST_CLASS_INSTANCE_LIVE;
                storage -> ux_host_class_storage_check_lun = 0;
                storage -> ux_host_class_storage_state_state =
                                        UX_HOST_CLASS_STORAGE_STATE_TEST_READY;
                continue;
            }

            /* Release the storage.  */
            storage -> ux_host_class_storage_flags &= ~UX_HOST_CLASS_STORAGE_FLAG_LOCK;

            /* Main state idle.  */
            storage -> ux_host_class_storage_state_state = UX_STATE_IDLE;
            continue;

        case UX_HOST_CLASS_STORAGE_STATE_DELAY_WAIT:
            tick_now = _ux_utility_time_get();
            tick_elapsed = _ux_utility_time_elapsed(
                storage -> ux_host_class_storage_delay_start, tick_now);

            /* If no timeout, state no change, keep waiting.  */
            if (tick_elapsed < storage -> ux_host_class_storage_delay_ms)
                return;

            /* Roll back to do next state.  */
            storage -> ux_host_class_storage_state_state =
                            storage -> ux_host_class_storage_state_next;
            continue;

        case UX_STATE_IDLE:

            /* If it's not locked, start delay for checking.  */
            UX_DISABLE
            if ((storage -> ux_host_class_storage_flags & UX_HOST_CLASS_STORAGE_FLAG_LOCK) == 0)
            {

                /* Just keep regular checking from LUN 0.  */
                storage -> ux_host_class_storage_check_lun = 0;
                storage -> ux_host_class_storage_delay_start = _ux_utility_time_get();
                storage -> ux_host_class_storage_state_state =
                                    UX_HOST_CLASS_STORAGE_STATE_DELAY_WAIT;
                storage -> ux_host_class_storage_state_next =
                                    UX_HOST_CLASS_STORAGE_STATE_CHECK_START;
                storage -> ux_host_class_storage_delay_ms = UX_MS_TO_TICK(
                                    UX_HOST_CLASS_STORAGE_THREAD_SLEEP_TIME);
                UX_RESTORE
                continue;
            }
            UX_RESTORE

            /* Fall through.  */

        case UX_STATE_RESET: /* Fall through.  */
        default:
            break;
        }

        /* Break the loop.  */
        immediate_state = UX_FALSE;
    }
}

static inline VOID _ux_host_class_storage_max_lun_save(UX_HOST_CLASS_STORAGE *storage)
{
UX_TRANSFER *trans = storage -> ux_host_class_storage_trans;

    /* By default max lun is 0.  */
    storage -> ux_host_class_storage_max_lun = 0;

    /* MaxLuN can from success request.  */
    if (trans)
    {

        /* If success, save max LUN.  */
        if (trans -> ux_transfer_request_completion_code == UX_SUCCESS &&
            trans -> ux_transfer_request_actual_length == 1)
        {
            storage -> ux_host_class_storage_max_lun =
                                *trans -> ux_transfer_request_data_pointer;

            /* Is the max LUN index greater than our LUN array's?  */
            if (storage -> ux_host_class_storage_max_lun > UX_MAX_HOST_LUN - 1)
            {

                /* Cap it off.  */
                storage -> ux_host_class_storage_max_lun =  UX_MAX_HOST_LUN - 1;

                /* Notify application so it knows to increase UX_MAX_HOST_LUN.  */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_MEMORY_ERROR);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_MEMORY_ERROR, storage, 0, 0, UX_TRACE_ERRORS, 0, 0)
            }
        }

        /* Allocated buffer for request must be freed here.  */
        _ux_utility_memory_free(trans -> ux_transfer_request_data_pointer);
    }
}
static inline UINT _ux_host_class_storage_inquiry_save(UX_HOST_CLASS_STORAGE *storage)
{
UCHAR       *inquiry_response = storage -> ux_host_class_storage_trans_data;
UINT        lun_index = storage -> ux_host_class_storage_lun;
    storage -> ux_host_class_storage_media_type = *(inquiry_response +
                        UX_HOST_CLASS_STORAGE_INQUIRY_RESPONSE_PERIPHERAL_TYPE);
    storage -> ux_host_class_storage_lun_types[lun_index] =
                                    storage -> ux_host_class_storage_media_type;
    storage -> ux_host_class_storage_lun_removable_media_flags[lun_index] =
                    *(inquiry_response +
                        UX_HOST_CLASS_STORAGE_INQUIRY_RESPONSE_REMOVABLE_MEDIA);

    /* Free response buffer.  */
    _ux_utility_memory_free(inquiry_response);

    /* Set default sector size.  */
    switch(storage -> ux_host_class_storage_media_type)
    {
    case UX_HOST_CLASS_STORAGE_MEDIA_FAT_DISK:
    case UX_HOST_CLASS_STORAGE_MEDIA_IOMEGA_CLICK:
        storage -> ux_host_class_storage_sector_size = UX_HOST_CLASS_STORAGE_SECTOR_SIZE_FAT;
        break;

    case UX_HOST_CLASS_STORAGE_MEDIA_CDROM:
    case UX_HOST_CLASS_STORAGE_MEDIA_OPTICAL_DISK:
        storage -> ux_host_class_storage_sector_size = UX_HOST_CLASS_STORAGE_SECTOR_SIZE_OTHER;
        break;

    default:

        /* Notify application so it knows to increase UX_MAX_HOST_LUN.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_MEDIA_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_MEDIA_NOT_SUPPORTED, storage, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_MEDIA_NOT_SUPPORTED);
    }
    return(UX_SUCCESS);
}
static inline VOID _ux_host_class_storage_capacity_save(UX_HOST_CLASS_STORAGE *storage)
{
UCHAR       *capacity_response = storage -> ux_host_class_storage_trans_data;

    /* If capacity response OK, save last LBA and LB size.  */
    if (storage -> ux_host_class_storage_sense_code == 0)
    {
        storage -> ux_host_class_storage_last_sector_number =
                    _ux_utility_long_get_big_endian(capacity_response +
                                UX_HOST_CLASS_STORAGE_READ_CAPACITY_DATA_LBA);
        storage -> ux_host_class_storage_sector_size =
                    _ux_utility_long_get_big_endian(capacity_response +
                        UX_HOST_CLASS_STORAGE_READ_CAPACITY_DATA_SECTOR_SIZE);

        /* Media inserted to LUN.  */
        _ux_host_class_storage_lun_media_insert(storage);
    }

    /* Free allocated buffer.  */
    _ux_utility_memory_free(capacity_response);
}
static inline VOID _ux_host_class_storage_format_cap_save(UX_HOST_CLASS_STORAGE *storage)
{
UCHAR       *format_cap_response = storage -> ux_host_class_storage_trans_data;

    /* There is nothing to save.  */
    /* Free allocated resource.  */
    _ux_utility_memory_free(format_cap_response);
}

static inline ULONG _ux_host_class_storage_lun_scan(UX_HOST_CLASS_STORAGE *storage,
                                                    ULONG do_unmount)
{
UX_HOST_CLASS                       *class_inst;
UX_HOST_CLASS_STORAGE_MEDIA         *storage_media;
ULONG                               media_index;
ULONG                               n_found;

    /* We may need to unmount this partition if it was mounted before.
        To do so, we need to parse the existing media instance and find out
        if this partition was already mounted.  */
    class_inst = storage -> ux_host_class_storage_class;
    storage_media = (UX_HOST_CLASS_STORAGE_MEDIA *) class_inst -> ux_host_class_media;

    /* Scan all instances of media.   */
    for (media_index = 0, n_found = 0;
        media_index < UX_HOST_CLASS_STORAGE_MAX_MEDIA;
        storage_media++, media_index++)
    {

        /* Check storage instance and lun number.  */
        if (storage_media -> ux_host_class_storage_media_status != UX_USED)
            continue;
        if (storage_media -> ux_host_class_storage_media_storage != storage)
            continue;
        if (storage_media -> ux_host_class_storage_media_lun !=
                                        storage -> ux_host_class_storage_lun)
            continue;

        /* Found mounted media.  */
        n_found ++;

        /* If no unmount, just check next media instance.  */
        if (do_unmount == UX_FALSE)
            continue;

        /* Free the storage media.  */
        storage_media -> ux_host_class_storage_media_status = UX_UNUSED;

        /* Invoke callback for media removal.  */
        if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
        {

            /* Call system change function.  */
            _ux_system_host ->  ux_system_host_change_function(UX_STORAGE_MEDIA_REMOVAL,
                                storage -> ux_host_class_storage_class,
                                (VOID *) storage_media);
        }
    }

    /* Return number of medias found and unmounted.  */
    return(n_found);
}

static inline VOID _ux_host_class_storage_unit_ready_check(UX_HOST_CLASS_STORAGE *storage)
{
ULONG           n;

    /* The LUN is ready.  */
    if (storage -> ux_host_class_storage_sense_code == 0)
    {

        /* Check if LUN is mounted.  */
        n = _ux_host_class_storage_lun_scan(storage, UX_FALSE);

        /* If LUN is already mounted, check next LUN.  */
        if (n > 0)
        {

            /* LUN is mounted, just check next LUN.  */
            storage -> ux_host_class_storage_state_state = UX_HOST_CLASS_STORAGE_STATE_NEXT_LUN;
            return;
        }

        /* Next mounting steps : Inquiry() -> ReadFormatCapacity() -> mount().  */
        storage -> ux_host_class_storage_state_state = UX_HOST_CLASS_STORAGE_STATE_INQUIRY;
        return;
    }

    /* The LUN not ready cases have been done before.  */

    /* Just check next LUN.  */
    storage -> ux_host_class_storage_state_state = UX_HOST_CLASS_STORAGE_STATE_NEXT_LUN;
}

static inline UINT _ux_host_class_storage_lun_is_removable(UX_HOST_CLASS_STORAGE *storage)
{
UINT    lun_index = storage -> ux_host_class_storage_lun;
UINT    removable = storage -> ux_host_class_storage_lun_removable_media_flags[lun_index] &
                    UX_HOST_CLASS_STORAGE_MEDIA_REMOVABLE;
    return(removable);
}
static inline UINT _ux_host_class_storage_lun_type_is_known(UX_HOST_CLASS_STORAGE *storage)
{
UINT    lun_index = storage -> ux_host_class_storage_lun;
UINT    lun_type = storage -> ux_host_class_storage_lun_types[lun_index];
    return((lun_type == UX_HOST_CLASS_STORAGE_MEDIA_FAT_DISK) ||
           (lun_type == UX_HOST_CLASS_STORAGE_MEDIA_OPTICAL_DISK) ||
           (lun_type == UX_HOST_CLASS_STORAGE_MEDIA_IOMEGA_CLICK));
}

static inline VOID _ux_host_class_storage_lun_media_insert(UX_HOST_CLASS_STORAGE *storage)
{
UX_HOST_CLASS                   *class_inst;
UX_HOST_CLASS_STORAGE_MEDIA     *storage_media;
INT                             media_index;

    /* Get class.  */
    class_inst = storage -> ux_host_class_storage_class;

    /* Find a free media slot for inserted media.  */
    storage_media =  (UX_HOST_CLASS_STORAGE_MEDIA *) class_inst -> ux_host_class_media;
    for (media_index = 0; media_index < UX_HOST_CLASS_STORAGE_MAX_MEDIA;
        storage_media ++, media_index ++)
    {

        /* Skip used storage media slots.  */
        if (storage_media -> ux_host_class_storage_media_status != UX_USED)
        {

            /* Use this free storage media slot.  */
            storage_media -> ux_host_class_storage_media_status = UX_USED;
            storage_media -> ux_host_class_storage_media_storage = storage;

            /* Save media information.  */
            storage_media -> ux_host_class_storage_media_lun = (UCHAR)storage -> ux_host_class_storage_lun;
            storage_media -> ux_host_class_storage_media_sector_size = (USHORT)storage -> ux_host_class_storage_sector_size;
            storage_media -> ux_host_class_storage_media_number_sectors = storage -> ux_host_class_storage_last_sector_number + 1;

            /* Invoke callback for media insertion.  */
            if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
            {

                /* Call system change function.  */
                /* In standalone mode, no state running (read/write) expected in callback.  */
                _ux_system_host ->  ux_system_host_change_function(UX_STORAGE_MEDIA_INSERTION,
                                    storage -> ux_host_class_storage_class, (VOID *) storage_media);
            }

            /* Media saved OK.  */
            return;
        }
    }

    /* No free slot.  */
    return;
}

static inline UINT _ux_host_class_storage_transport_sense_check(UX_HOST_CLASS_STORAGE *storage)
{
ULONG           sense_key;

    if (storage -> ux_host_class_storage_sense_code == 0)
        return(UX_FALSE);

    sense_key = UX_HOST_CLASS_STORAGE_SENSE_KEY(storage -> ux_host_class_storage_sense_code);
    switch(sense_key)
    {
    case UX_HOST_CLASS_STORAGE_SENSE_KEY_NOT_READY:

        /* Remove the mounted LUN media.  */
        _ux_host_class_storage_lun_scan(storage, UX_TRUE);

        /* The LUN is not ready, check next any way.  */
        storage -> ux_host_class_storage_state_state =
                                        UX_HOST_CLASS_STORAGE_STATE_NEXT_LUN;

        /* We have changed state.  */
        return(UX_TRUE);

    case UX_HOST_CLASS_STORAGE_SENSE_KEY_UNIT_ATTENTION:

        /* Remove the mounted LUN media.  */
        _ux_host_class_storage_lun_scan(storage, UX_TRUE);

        /* Do immediate ready check again.  */
        storage -> ux_host_class_storage_state_state =
                                    UX_HOST_CLASS_STORAGE_STATE_TEST_READY;

        /* We have changed state.  */
        return(UX_TRUE);

    case UX_HOST_CLASS_STORAGE_SENSE_KEY_DATA_PROTECT:

        /* Fall through.  */
    default:

        /* Nothing to do now.  */
        break;
    }

    /* State manated by others.  */
    return(UX_FALSE);
}
#endif
