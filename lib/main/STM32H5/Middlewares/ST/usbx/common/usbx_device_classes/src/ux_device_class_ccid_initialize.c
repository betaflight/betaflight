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
/*    _ux_device_class_ccid_initialize                    PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function initializes the USB CCID device.                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                               Pointer to ccid command       */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_free               Free memory                   */
/*    _ux_utility_semaphore_create          Create semaphore              */
/*    _ux_utility_semaphore_delete          Delete semaphore              */
/*    _ux_utility_mutex_create              Create mutex                  */
/*    _ux_device_mutex_delete               Delete mutex                  */
/*    _ux_utility_thread_create             Create thread                 */
/*    _ux_utility_thread_delete             Delete thread                 */
/*    _ux_utility_event_flags_create        Create event flags            */
/*    _ux_utility_event_flags_delete        Delete event flags            */
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
UINT  _ux_device_class_ccid_initialize(UX_SLAVE_CLASS_COMMAND *command)
{
#if defined(UX_DEVICE_STANDALONE)
    UX_PARAMETER_NOT_USED(command);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else
UX_DEVICE_CLASS_CCID                    *ccid;
UX_DEVICE_CLASS_CCID_RUNNER             *runner;
UX_DEVICE_CLASS_CCID_SLOT               *slot;
UX_DEVICE_CLASS_CCID_PARAMETER          *ccid_parameter;
UX_SLAVE_CLASS                          *ccid_class;
UINT                                    status;
ULONG                                   memory_size;
ULONG                                   ccid_size, runners_size, slots_size;
ULONG                                   buffer_size, buffers_size;
ULONG                                   stacks_size;
UCHAR                                   *memory;
ULONG                                   i;

    /* Get the class container.  */
    ccid_class =  command -> ux_slave_class_command_class_ptr;

    /* Get the pointer to the application parameters for the ccid class.  */
    ccid_parameter =  command -> ux_slave_class_command_parameter;

    /* Sanity check for parameters.
        - number slots < 32
        - number busy <= number slots
        - message length < max request buffer size
      */
    UX_ASSERT(ccid_parameter -> ux_device_class_ccid_max_n_slots <= UX_DEVICE_CLASS_CCID_MAX_N_SLOTS);
    UX_ASSERT(ccid_parameter -> ux_device_class_ccid_max_n_busy_slots != 0);
    UX_ASSERT(ccid_parameter -> ux_device_class_ccid_max_n_busy_slots <=
                ccid_parameter -> ux_device_class_ccid_max_n_slots)
    UX_ASSERT(ccid_parameter -> ux_device_class_ccid_max_transfer_length <=
                UX_SLAVE_REQUEST_DATA_MAX_LENGTH);
    UX_ASSERT(ccid_parameter->ux_device_class_ccid_handles != UX_NULL);

    /* Calculate size for instance (structures already aligned).  */
    /* Max n_slots 32, considering struct size, no overflow case.  */
    ccid_size = sizeof(UX_DEVICE_CLASS_CCID);
    memory_size = ccid_size;

    runners_size = sizeof(UX_DEVICE_CLASS_CCID_RUNNER);
    runners_size *= ccid_parameter->ux_device_class_ccid_max_n_busy_slots;
    memory_size += runners_size;

    slots_size = sizeof(UX_DEVICE_CLASS_CCID_SLOT);
    slots_size *= ccid_parameter->ux_device_class_ccid_max_n_slots;
    memory_size += slots_size;

    buffer_size = ccid_parameter->ux_device_class_ccid_max_transfer_length;
    buffer_size += 3u;
    buffer_size &= ~3u;
    ccid_parameter->ux_device_class_ccid_max_transfer_length = buffer_size;
    if (UX_OVERFLOW_CHECK_MULC_ULONG(buffer_size, 2))
    {

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ERROR);
        return(UX_ERROR);
    }
    buffers_size = (buffer_size << 1) & 0xFFFFFFFFu;
    if (UX_OVERFLOW_CHECK_MULV_ULONG(buffers_size, ccid_parameter->ux_device_class_ccid_max_n_busy_slots))
    {

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ERROR);
        return(UX_ERROR);
    }
    buffers_size = (buffers_size * ccid_parameter->ux_device_class_ccid_max_n_busy_slots) & 0xFFFFFFFFu;
    if (UX_OVERFLOW_CHECK_ADD_ULONG(memory_size, buffer_size))
    {

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ERROR);
        return(UX_ERROR);
    }
    memory_size = (memory_size + buffers_size) & 0xFFFFFFFFu;

    stacks_size = ccid_parameter->ux_device_class_ccid_max_n_busy_slots;
    if (UX_OVERFLOW_CHECK_MULC_ULONG(stacks_size, UX_DEVICE_CLASS_CCID_RUNNER_THREAD_STACK_SIZE))
    {

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ERROR);
        return(UX_ERROR);
    }
    stacks_size *= UX_DEVICE_CLASS_CCID_RUNNER_THREAD_STACK_SIZE;
    if (UX_OVERFLOW_CHECK_ADD_ULONG(stacks_size, UX_DEVICE_CLASS_CCID_THREAD_STACK_SIZE))
    {

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ERROR);
        return(UX_ERROR);
    }
    stacks_size += UX_DEVICE_CLASS_CCID_THREAD_STACK_SIZE;
    if (UX_OVERFLOW_CHECK_ADD_ULONG(stacks_size, UX_DEVICE_CLASS_CCID_NOTIFY_THREAD_STACK_SIZE))
    {

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ERROR);
        return(UX_ERROR);
    }
    stacks_size += UX_DEVICE_CLASS_CCID_NOTIFY_THREAD_STACK_SIZE;
    if (UX_OVERFLOW_CHECK_ADD_ULONG(memory_size, stacks_size))
    {

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ERROR);
        return(UX_ERROR);
    }
    memory_size = (memory_size + stacks_size) & 0xFFFFFFFFu;

    /* Allocate memory for instance and other resources of the device ccid class.  */
    memory = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, memory_size);

    /* Check for successful allocation.  */
    if (memory == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Create and save resources for ccid, runners, slots, buffers.  */

    /* CCID.  */
    ccid = (UX_DEVICE_CLASS_CCID *)memory;
    memory += ccid_size;

    /* CCID thread.  */
    status =  _ux_utility_thread_create(
                &ccid -> ux_device_class_ccid_thread,
                "ux_device_class_ccid_thread",
                _ux_device_class_ccid_thread_entry,
                (ULONG) (ALIGN_TYPE) ccid,
                (VOID *) memory,
                UX_DEVICE_CLASS_CCID_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);
    if (status != UX_SUCCESS)
        status = UX_THREAD_ERROR;
    else
    {
        ccid -> ux_device_class_ccid_thread_stack = memory;
        memory += UX_DEVICE_CLASS_CCID_THREAD_STACK_SIZE;
        UX_THREAD_EXTENSION_PTR_SET(&(ccid -> ux_device_class_ccid_thread), ccid);
    }

    /* CCID Notify thread.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_utility_thread_create(
                    &ccid -> ux_device_class_ccid_notify_thread,
                    "ux_device_class_ccid_notify_thread",
                    _ux_device_class_ccid_notify_thread_entry,
                    (ULONG) (ALIGN_TYPE) ccid,
                    (VOID *) memory,
                    UX_DEVICE_CLASS_CCID_NOTIFY_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                    UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);
        if (status != UX_SUCCESS)
            status = UX_THREAD_ERROR;
        else
        {
            ccid -> ux_device_class_ccid_notify_thread_stack = memory;
            memory += UX_DEVICE_CLASS_CCID_NOTIFY_THREAD_STACK_SIZE;
            UX_THREAD_EXTENSION_PTR_SET(&(ccid -> ux_device_class_ccid_notify_thread), ccid);
        }
    }

    /* CCID runners.  */
    if (status == UX_SUCCESS)
    {

        ccid -> ux_device_class_ccid_runners = (UX_DEVICE_CLASS_CCID_RUNNER *)memory;
        memory += runners_size;

        runner = ccid -> ux_device_class_ccid_runners;
        for (i = 0; i < ccid_parameter -> ux_device_class_ccid_max_n_busy_slots; i ++)
        {

            /* Save CCID for runner.  */
            runner -> ux_device_class_ccid_runner_ccid = ccid;

            /* Save runner ID.  */
            runner -> ux_device_class_ccid_runner_id = (CHAR)i;

            /* Runner is free.  */
            runner -> ux_device_class_ccid_runner_slot = -1;

            /* Runner command buffer.  */
            runner -> ux_device_class_ccid_runner_command = memory;
            memory += buffer_size;

            /* Runner response buffer.  */
            runner -> ux_device_class_ccid_runner_response = memory;
            memory += buffer_size;

            /* CCID runners threads.  */
            status = _ux_utility_thread_create(
                &runner -> ux_device_class_ccid_runner_thread,
                "ux_device_class_ccid_runner_thread",
                _ux_device_class_ccid_runner_thread_entry,
                (ULONG) (ALIGN_TYPE) runner,
                (VOID *)memory,
                UX_DEVICE_CLASS_CCID_RUNNER_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);
            if (status != UX_SUCCESS)
            {
                status = UX_THREAD_ERROR;
                break;
            }
            else
            {
                runner -> ux_device_class_ccid_runner_thread_stack = memory;
                memory += UX_DEVICE_CLASS_CCID_RUNNER_THREAD_STACK_SIZE;
                UX_THREAD_EXTENSION_PTR_SET(&(runner -> ux_device_class_ccid_runner_thread), runner);
            }

            /* Next runner.  */
            runner ++;
        }
    }

    /* CCID slots.  */
    if (status == UX_SUCCESS)
    {

        ccid -> ux_device_class_ccid_slots = (UX_DEVICE_CLASS_CCID_SLOT *)memory;

        slot = ccid -> ux_device_class_ccid_slots;
        for (i = 0; i < ccid_parameter -> ux_device_class_ccid_max_n_slots; i ++)
        {

            /* Slot not busy.  */
            slot -> ux_device_class_ccid_slot_runner = -1;

            /* Slot ICC no card.  */
            slot -> ux_device_class_ccid_slot_icc_status = UX_DEVICE_CLASS_CCID_ICC_NOT_PRESENT;

            /* Next slot.  */
            slot ++;
        }
    }

    /* CCID mutexes, semaphore and event flags.  */
    if (status == UX_SUCCESS)
    {

        /* Create Event Flags.  */
        status = _ux_utility_event_flags_create(&ccid -> ux_device_class_ccid_events,
                                                "ux_device_class_ccid_events");
        if (status == UX_SUCCESS)
        {

            /* Bulk IN mutex.  */
            status =  _ux_utility_mutex_create(&ccid -> ux_device_class_ccid_response_mutex,
                                                "ux_device_class_ccid_response_mutex");
            if (status == UX_SUCCESS)
            {

                /* Interrupt IN semaphore.  */
                status =  _ux_utility_semaphore_create(&ccid -> ux_device_class_ccid_notify_semaphore,
                                                "ux_device_class_ccid_notify_semaphore", 0);
                if (status == UX_SUCCESS)
                {

                    /* Resource access Mutex.  */
                    status = _ux_utility_mutex_create(&ccid -> ux_device_class_ccid_mutex,
                                                "ux_device_class_ccid_mutex");
                    if (status != UX_SUCCESS)
                        status = UX_MUTEX_ERROR;

                    /* If there is error, allocated semaphore should be deleted.  */
                    if (status != UX_SUCCESS)
                        _ux_utility_semaphore_delete(&ccid -> ux_device_class_ccid_notify_semaphore);
                }
                else
                    status = UX_MUTEX_ERROR;

                /* If there is error, allocated mutex should be deleted.  */
                if (status != UX_SUCCESS)
                    _ux_device_mutex_delete(&ccid -> ux_device_class_ccid_response_mutex);
            }
            else
                status = UX_MUTEX_ERROR;

            /* If there is error, allocated event should be deleted.  */
            if (status != UX_SUCCESS)
                _ux_utility_event_flags_delete(&ccid -> ux_device_class_ccid_events);
        }
        else
            status = UX_EVENT_ERROR;
    }

    /* Success case.  */
    if (status == UX_SUCCESS)
    {

        /* Save the address of the CDC instance inside the CDC container.  */
        ccid_class -> ux_slave_class_instance = (VOID *) ccid;

        /* Store parameters.  */
        _ux_utility_memory_copy(&ccid -> ux_device_class_ccid_parameter, ccid_parameter,
                                sizeof(UX_DEVICE_CLASS_CCID_PARAMETER)); /* Use case of memcpy is verified. */

        /* Return success status.  */
        return(UX_SUCCESS);
    }

    /* Error cases.  */

    /* In this case, mutexes and events are not created or has been handled.  */

    /* Check thread states and free them.  */
    if (ccid -> ux_device_class_ccid_thread_stack)
        _ux_utility_thread_delete(&ccid -> ux_device_class_ccid_thread);
    for (i = 0; i < ccid_parameter -> ux_device_class_ccid_max_n_busy_slots; i ++)
    {
        runner = &ccid -> ux_device_class_ccid_runners[i];
        if (runner -> ux_device_class_ccid_runner_thread_stack)
            _ux_utility_thread_delete(&runner -> ux_device_class_ccid_runner_thread);
    }
    if (ccid -> ux_device_class_ccid_notify_thread_stack)
        _ux_utility_thread_delete(&ccid -> ux_device_class_ccid_notify_thread);

    /* Free the memory.  */
    _ux_utility_memory_free(ccid);

    /* Return completion status.  */
    return(status);
#endif
}

const UX_DEVICE_CLASS_CCID_COMMAND_SETT
_ux_device_class_ccid_command_sett[UX_DEVICE_CLASS_CCID_N_COMMANDS + 1] =
{
    {0x62, UX_DEVICE_CLASS_CCID_COMMAND_RESP_TYPE(0x62),
           UX_DEVICE_CLASS_CCID_COMMAND_FLAGS(0x62), 0},
    {0x63, UX_DEVICE_CLASS_CCID_COMMAND_RESP_TYPE(0x63),
           UX_DEVICE_CLASS_CCID_COMMAND_FLAGS(0x63), 1},
    {0x65, UX_DEVICE_CLASS_CCID_COMMAND_RESP_TYPE(0x65),
           UX_DEVICE_CLASS_CCID_COMMAND_FLAGS(0x65), 2},
    {0x6F, UX_DEVICE_CLASS_CCID_COMMAND_RESP_TYPE(0x6F),
           UX_DEVICE_CLASS_CCID_COMMAND_FLAGS(0x6F), 3},
    {0x6C, UX_DEVICE_CLASS_CCID_COMMAND_RESP_TYPE(0x6C),
           UX_DEVICE_CLASS_CCID_COMMAND_FLAGS(0x6C), 4},
    {0x6D, UX_DEVICE_CLASS_CCID_COMMAND_RESP_TYPE(0x6D),
           UX_DEVICE_CLASS_CCID_COMMAND_FLAGS(0x6D), 5},
    {0x61, UX_DEVICE_CLASS_CCID_COMMAND_RESP_TYPE(0x61),
           UX_DEVICE_CLASS_CCID_COMMAND_FLAGS(0x61), 6},
    {0x6B, UX_DEVICE_CLASS_CCID_COMMAND_RESP_TYPE(0x6B),
           UX_DEVICE_CLASS_CCID_COMMAND_FLAGS(0x6B), 7},
    {0x6E, UX_DEVICE_CLASS_CCID_COMMAND_RESP_TYPE(0x6E),
           UX_DEVICE_CLASS_CCID_COMMAND_FLAGS(0x6E), 8},
    {0x6A, UX_DEVICE_CLASS_CCID_COMMAND_RESP_TYPE(0x6A),
           UX_DEVICE_CLASS_CCID_COMMAND_FLAGS(0x6A), 9},
    {0x69, UX_DEVICE_CLASS_CCID_COMMAND_RESP_TYPE(0x69),
           UX_DEVICE_CLASS_CCID_COMMAND_FLAGS(0x69), 10},
    {0x71, UX_DEVICE_CLASS_CCID_COMMAND_RESP_TYPE(0x71),
           UX_DEVICE_CLASS_CCID_COMMAND_FLAGS(0x71), 11},
    {0x72, UX_DEVICE_CLASS_CCID_COMMAND_RESP_TYPE(0x72),
           UX_DEVICE_CLASS_CCID_COMMAND_FLAGS(0x72), 12},
    {0x73, UX_DEVICE_CLASS_CCID_COMMAND_RESP_TYPE(0x73),
           UX_DEVICE_CLASS_CCID_COMMAND_FLAGS(0x73), 13},
    {0, 0, 0, -1}, /* Command not supported.  */
};
