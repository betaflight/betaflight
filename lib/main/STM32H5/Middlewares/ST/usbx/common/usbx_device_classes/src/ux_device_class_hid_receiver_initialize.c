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
/**   Device HID Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_hid.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_hid_receiver_initialize            PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function initializes the USB HID device receiver.              */
/*    This function is called by the class register function. It is only  */
/*    done once.                                                          */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hid                                  Pointer to hid instance        */
/*    parameter                            Pointer to hid parameter       */
/*    receiver                             Pointer to fill pointer to     */
/*                                           allocated receiver instance  */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_free               Free memory                   */
/*    _ux_utility_thread_create             Create thread                 */
/*    _ux_utility_thread_delete             Delete thread                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Source Code                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added receiver callback,    */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone receiver,  */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_hid_receiver_initialize(UX_SLAVE_CLASS_HID *hid,
                                    UX_SLAVE_CLASS_HID_PARAMETER *parameter,
                                    UX_DEVICE_CLASS_HID_RECEIVER **receiver)
{
#if !defined(UX_DEVICE_CLASS_HID_INTERRUPT_OUT_SUPPORT)
    UX_PARAMETER_NOT_USED(hid);
    UX_PARAMETER_NOT_USED(parameter);
    UX_PARAMETER_NOT_USED(receiver);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else
ULONG                                   memory_size;
ULONG                                   events_size;
UCHAR                                   *memory_receiver;
UCHAR                                   *memory_events;
#if !defined(UX_DEVICE_STANDALONE)
UCHAR                                   *memory_stack;
#endif
UINT                                    status = UX_SUCCESS;


    /* Validate parameters.  */
    UX_ASSERT(parameter -> ux_device_class_hid_parameter_receiver_event_max_length <= UX_SLAVE_REQUEST_DATA_MAX_LENGTH);

    /* Allocate memory for receiver and receiver events.  */

    /* Memory of thread stack and receiver instance.  */
#if !defined(UX_DEVICE_STANDALONE)
    UX_ASSERT(!UX_OVERFLOW_CHECK_ADD_ULONG(UX_DEVICE_CLASS_HID_RECEIVER_THREAD_STACK_SIZE, sizeof(UX_DEVICE_CLASS_HID_RECEIVER)));
    memory_size = UX_DEVICE_CLASS_HID_RECEIVER_THREAD_STACK_SIZE +
                  sizeof(UX_DEVICE_CLASS_HID_RECEIVER);
#else
    memory_size = sizeof(UX_DEVICE_CLASS_HID_RECEIVER);
#endif
    UX_ASSERT(!UX_OVERFLOW_CHECK_ADD_ULONG(parameter -> ux_device_class_hid_parameter_receiver_event_max_length, sizeof(ULONG)));

    /* Memory of events.  */
    events_size  = parameter -> ux_device_class_hid_parameter_receiver_event_max_length + sizeof(ULONG);
    UX_ASSERT(!UX_OVERFLOW_CHECK_MULV_ULONG(events_size, parameter -> ux_device_class_hid_parameter_receiver_event_max_number));
    events_size *= parameter -> ux_device_class_hid_parameter_receiver_event_max_number;
    UX_ASSERT(!UX_OVERFLOW_CHECK_ADD_ULONG(memory_size, events_size));
    memory_size += events_size;

    /* Allocate memory.  */
    memory_receiver = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, memory_size);
    if (memory_receiver == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);
#if !defined(UX_DEVICE_STANDALONE)
    memory_stack = memory_receiver + sizeof(UX_DEVICE_CLASS_HID_RECEIVER);
    memory_events = memory_stack + UX_DEVICE_CLASS_HID_RECEIVER_THREAD_STACK_SIZE;
#else
    memory_events = memory_receiver + sizeof(UX_DEVICE_CLASS_HID_RECEIVER);
#endif

    /* Store receiver instance pointer.  */
    (*receiver) = (UX_DEVICE_CLASS_HID_RECEIVER *)memory_receiver;

#if !defined(UX_DEVICE_STANDALONE)

    /* This instance needs to be running in a different thread. So start
       a new thread. We pass a pointer to the class to the new thread.  This thread
       does not start until we have a instance of the class. */
    if (status == UX_SUCCESS)
        status =  _ux_utility_thread_create(&(*receiver) -> ux_device_class_hid_receiver_thread,
                    "ux_device_class_hid_receiver_thread",
                    _ux_device_class_hid_receiver_thread,
                    (ULONG) (ALIGN_TYPE) hid, (VOID *) memory_stack,
                    UX_DEVICE_CLASS_HID_RECEIVER_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                    UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);
#endif

    /* Check the creation of this thread.  */
    if (status == UX_SUCCESS)
    {

#if !defined(UX_DEVICE_STANDALONE)
        UX_THREAD_EXTENSION_PTR_SET(&((*receiver) -> ux_device_class_hid_receiver_thread), hid)
#else
        hid -> ux_device_class_hid_read_state = UX_DEVICE_CLASS_HID_RECEIVER_START;
        (*receiver) -> ux_device_class_hid_receiver_tasks_run = _ux_device_class_hid_receiver_tasks_run;
#endif

        /* Initialize event buffer size.  */
        (*receiver) -> ux_device_class_hid_receiver_event_buffer_size =
                    parameter -> ux_device_class_hid_parameter_receiver_event_max_length;

        /* Initialize events.  */
        (*receiver) -> ux_device_class_hid_receiver_events =
                        (UX_DEVICE_CLASS_HID_RECEIVED_EVENT *)(memory_events);
        (*receiver) -> ux_device_class_hid_receiver_events_end =
                        (UX_DEVICE_CLASS_HID_RECEIVED_EVENT *)(memory_receiver + memory_size);
        (*receiver) -> ux_device_class_hid_receiver_event_read_pos =
                    (*receiver) -> ux_device_class_hid_receiver_events;
        (*receiver) -> ux_device_class_hid_receiver_event_save_pos =
                    (*receiver) -> ux_device_class_hid_receiver_events;

        /* Initialize uninitialize function.  */
        (*receiver) -> ux_device_class_hid_receiver_uninitialize = _ux_device_class_hid_receiver_uninitialize;

        /* Initialize callback function.  */
        (*receiver) -> ux_device_class_hid_receiver_event_callback =
                    parameter -> ux_device_class_hid_parameter_receiver_event_callback;

        /* Done success.  */
        return(UX_SUCCESS);
    }
    else
        status = (UX_THREAD_ERROR);

    /* Free allocated memory. */
    _ux_utility_memory_free(*receiver);
    (*receiver) =  UX_NULL;

    /* Return completion status.  */
    return(status);
#endif
}
