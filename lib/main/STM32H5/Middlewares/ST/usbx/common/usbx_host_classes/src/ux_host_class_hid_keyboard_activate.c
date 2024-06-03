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
/**   HID Keyboard Client                                                 */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_keyboard.h"
#include "ux_host_stack.h"

/* Define USB HID keyboard mapping tables.  */

#ifndef UX_HOST_CLASS_HID_KEYBOARD_REGULAR_ARRAY_DEFAULT
#define UX_HOST_CLASS_HID_KEYBOARD_REGULAR_ARRAY_DEFAULT UX_HID_KEYBOARD_REGULAR_ARRAY_US
#endif
#ifndef UX_HOST_CLASS_HID_KEYBOARD_SHIFT_ARRAY_DEFAULT
#define UX_HOST_CLASS_HID_KEYBOARD_SHIFT_ARRAY_DEFAULT UX_HID_KEYBOARD_SHIFT_ARRAY_US
#endif
#ifndef UX_HOST_CLASS_HID_KEYBOARD_NUMLOCK_ON_ARRAY_DEFAULT
#define UX_HOST_CLASS_HID_KEYBOARD_NUMLOCK_ON_ARRAY_DEFAULT UX_HID_KEYBOARD_NUMLOCK_ON_ARRAY
#endif
#ifndef UX_HOST_CLASS_HID_KEYBOARD_NUMLOCK_OFF_ARRAY_DEFAULT
#define UX_HOST_CLASS_HID_KEYBOARD_NUMLOCK_OFF_ARRAY_DEFAULT UX_HID_KEYBOARD_NUMLOCK_OFF_ARRAY
#endif
#ifndef UX_HOST_CLASS_HID_KEYBOARD_KEYS_UPPER_RANGE_DEFAULT
#define UX_HOST_CLASS_HID_KEYBOARD_KEYS_UPPER_RANGE_DEFAULT UX_HID_KEYBOARD_KEYS_UPPER_RANGE
#endif
#ifndef UX_HOST_CLASS_HID_KEYBOARD_KEY_LETTER_A_DEFAULT
#define UX_HOST_CLASS_HID_KEYBOARD_KEY_LETTER_A_DEFAULT UX_HID_KEYBOARD_KEY_LETTER_A
#endif
#ifndef UX_HOST_CLASS_HID_KEYBOARD_KEY_LETTER_Z_DEFAULT
#define UX_HOST_CLASS_HID_KEYBOARD_KEY_LETTER_Z_DEFAULT UX_HID_KEYBOARD_KEY_LETTER_Z
#endif
#ifndef UX_HOST_CLASS_HID_KEYBOARD_KEYS_KEYPAD_LOWER_RANGE_DEFAULT
#define UX_HOST_CLASS_HID_KEYBOARD_KEYS_KEYPAD_LOWER_RANGE_DEFAULT UX_HID_KEYBOARD_KEYS_KEYPAD_LOWER_RANGE
#endif
#ifndef UX_HOST_CLASS_HID_KEYBOARD_KEYS_KEYPAD_UPPER_RANGE_DEFAULT
#define UX_HOST_CLASS_HID_KEYBOARD_KEYS_KEYPAD_UPPER_RANGE_DEFAULT UX_HID_KEYBOARD_KEYS_KEYPAD_UPPER_RANGE
#endif

UCHAR ux_host_class_hid_keyboard_regular_array[] =
{
   UX_HOST_CLASS_HID_KEYBOARD_REGULAR_ARRAY_DEFAULT
};

UCHAR ux_host_class_hid_keyboard_shift_array[] =
{
   UX_HOST_CLASS_HID_KEYBOARD_SHIFT_ARRAY_DEFAULT
};

UCHAR ux_host_class_hid_keyboard_numlock_on_array[] =
{
   UX_HOST_CLASS_HID_KEYBOARD_NUMLOCK_ON_ARRAY_DEFAULT
};

UCHAR ux_host_class_hid_keyboard_numlock_off_array[] =
{
   UX_HOST_CLASS_HID_KEYBOARD_NUMLOCK_OFF_ARRAY_DEFAULT
};

UX_HOST_CLASS_HID_KEYBOARD_LAYOUT ux_host_class_hid_keyboard_layout =
{
    ux_host_class_hid_keyboard_regular_array,
    ux_host_class_hid_keyboard_shift_array,
    ux_host_class_hid_keyboard_numlock_on_array,
    ux_host_class_hid_keyboard_numlock_off_array,
    UX_HOST_CLASS_HID_KEYBOARD_KEYS_UPPER_RANGE_DEFAULT,
    UX_HOST_CLASS_HID_KEYBOARD_KEY_LETTER_A_DEFAULT,
    UX_HOST_CLASS_HID_KEYBOARD_KEY_LETTER_Z_DEFAULT,
    UX_HOST_CLASS_HID_KEYBOARD_KEYS_KEYPAD_LOWER_RANGE_DEFAULT,
    UX_HOST_CLASS_HID_KEYBOARD_KEYS_KEYPAD_UPPER_RANGE_DEFAULT,
};

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_keyboard_activate                PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs the enumeration of a HID Keyboard Client.    */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Pointer to command            */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_hid_periodic_report_start                            */
/*                                          Start periodic report         */ 
/*    _ux_host_class_hid_report_callback_register                         */
/*                                          Register callback             */ 
/*    _ux_host_class_hid_report_id_get      Get the report ID             */
/*    _ux_host_class_hid_idle_set           Set the idle rate             */
/*    _ux_host_class_hid_report_set         Do SET_REPORT                 */
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Free memory block             */ 
/*    _ux_host_semaphore_create             Create semaphore              */
/*    _ux_host_semaphore_delete             Delete semaphore              */
/*    _ux_utility_thread_create             Create thread                 */
/*    _ux_utility_thread_delete             Delete thread                 */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HID Class                                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed clients management,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_keyboard_activate(UX_HOST_CLASS_HID_CLIENT_COMMAND *command)
{

UX_HOST_CLASS_HID_REPORT_CALLBACK       call_back;
#if !defined(UX_HOST_STANDALONE)
UX_HOST_CLASS_HID_CLIENT_REPORT         client_report;
#endif
UX_HOST_CLASS_HID_REPORT_GET_ID         report_id;
UX_HOST_CLASS_HID                       *hid;
UX_HOST_CLASS_HID_CLIENT                *hid_client;
UX_HOST_CLASS_HID_CLIENT_KEYBOARD       *client_keyboard;
UX_HOST_CLASS_HID_KEYBOARD              *keyboard_instance;
ULONG                                   event_process_memory_size;
UINT                                    status = UX_SUCCESS;
#ifdef UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE
UX_HOST_CLASS_HID_FIELD                 *field;
#endif


    /* Get the instance to the HID class.  */
    hid =  command -> ux_host_class_hid_client_command_instance;

    /* Get some memory for both the HID class instance and copy of this client
       and for the callback.  */
    client_keyboard =  (UX_HOST_CLASS_HID_CLIENT_KEYBOARD *)
                    _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, 
                                    sizeof(UX_HOST_CLASS_HID_CLIENT_KEYBOARD));
    if (client_keyboard == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Use allocated memory.
     * - create a client copy.
     * - get keyboard instance.
     */
    keyboard_instance = &client_keyboard -> ux_host_class_hid_client_keyboard_keyboard;
    hid_client = &client_keyboard -> ux_host_class_hid_client_keyboard_client;
    _ux_utility_memory_copy(hid_client, hid -> ux_host_class_hid_client, sizeof(UX_HOST_CLASS_HID_CLIENT)); /* Use case of memcpy is verified. */

    /* Attach the remote control instance to the client instance.  */
    hid_client -> ux_host_class_hid_client_local_instance =  (VOID *) keyboard_instance;

    /* Save the HID instance in the client instance.  */
    keyboard_instance -> ux_host_class_hid_keyboard_hid =  hid;

#if defined(UX_HOST_STANDALONE)

    /* Set client task function.  */
    hid_client -> ux_host_class_hid_client_function = _ux_host_class_hid_keyboard_tasks_run;

    /* The instance is mounting now.  */
    keyboard_instance -> ux_host_class_hid_keyboard_state =  UX_HOST_CLASS_INSTANCE_MOUNTING;
#else

    /* The instance is live now.  */
    keyboard_instance -> ux_host_class_hid_keyboard_state =  UX_HOST_CLASS_INSTANCE_LIVE;
#endif

    /* Allocate the round-robin buffer that the remote control instance will use
     * to store the usages as they come in.
     * Size calculation overflow is checked near where _USAGE_ARRAY_LENGTH is defined.
     */
    keyboard_instance -> ux_host_class_hid_keyboard_usage_array =  (ULONG *)
                            _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_HOST_CLASS_HID_KEYBOARD_USAGE_ARRAY_LENGTH*4);
    if (keyboard_instance -> ux_host_class_hid_keyboard_usage_array == UX_NULL)
        status = (UX_MEMORY_INSUFFICIENT);

    /* If we are OK, go on.  */
    if (status == UX_SUCCESS)
    {

        /* Initialize the head and tail of this array.  */
        keyboard_instance -> ux_host_class_hid_keyboard_usage_array_head =  keyboard_instance -> ux_host_class_hid_keyboard_usage_array;
        keyboard_instance -> ux_host_class_hid_keyboard_usage_array_tail =  keyboard_instance -> ux_host_class_hid_keyboard_usage_array;

        /* Get the report ID for the keyboard. The keyboard is a INPUT report.
        This should be 0 but in case. */
        report_id.ux_host_class_hid_report_get_report = UX_NULL;
        report_id.ux_host_class_hid_report_get_type = UX_HOST_CLASS_HID_REPORT_TYPE_INPUT;
        status =  _ux_host_class_hid_report_id_get(hid, &report_id);
    }

    /* If we are OK, go on.  */
    if (status == UX_SUCCESS)
    {

        /* Save the keyboard report ID. */
        keyboard_instance -> ux_host_class_hid_keyboard_id = (USHORT)(report_id.ux_host_class_hid_report_get_id);

#ifdef UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE

        /* Summarize number of usages to allocate state buffer for keyboard report.
           We reserve 3 bits for lock status, 8 bits for modifier status.
           We get number of keys from report of 8-bit array.
           Key usages bytes then saved if it's down.
           - 3 Locks - 8 Modifiers - N regular keys ...
         */
        keyboard_instance -> ux_host_class_hid_keyboard_key_count = 3 + 8;
        field = report_id.ux_host_class_hid_report_get_report -> ux_host_class_hid_report_field;
        while(field)
        {
            if (field -> ux_host_class_hid_field_report_size == 8)
                keyboard_instance -> ux_host_class_hid_keyboard_key_count += field -> ux_host_class_hid_field_report_count;
            field = field -> ux_host_class_hid_field_next_field;
        }

        /* Process memory includes:
            - states for last [usage, value] and new [usage, value], (2 * 2) * max number of keys to log
            - actions for last and new buffers, 2 * max number of keys to log
         */
        UX_UTILITY_MULC_SAFE(keyboard_instance -> ux_host_class_hid_keyboard_key_count, 6, event_process_memory_size, status);

        /* Calculation overflow check.  */
        if (status == UX_SUCCESS)
#else

        /* We reserve 3 bytes for lock keys, 3 for processing.  */
        keyboard_instance -> ux_host_class_hid_keyboard_key_count = 3;

        /* key count 3, multiply 2 is int safe.  */
        event_process_memory_size = keyboard_instance -> ux_host_class_hid_keyboard_key_count * 2;
#endif

        {

        /* Allocate memory for usages states.  */
        keyboard_instance -> ux_host_class_hid_keyboard_key_state = (UCHAR *)
                            _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, event_process_memory_size);
        if (keyboard_instance -> ux_host_class_hid_keyboard_key_state == UX_NULL)
            status = UX_MEMORY_INSUFFICIENT;
    }
    }

    /* If we are OK, go on.  */
    if (status == UX_SUCCESS)
    {

        /* Initialize the report callback.  */
        call_back.ux_host_class_hid_report_callback_id =         keyboard_instance -> ux_host_class_hid_keyboard_id;
        call_back.ux_host_class_hid_report_callback_function =   _ux_host_class_hid_keyboard_callback;
        call_back.ux_host_class_hid_report_callback_buffer =     UX_NULL;
        call_back.ux_host_class_hid_report_callback_flags =      UX_HOST_CLASS_HID_REPORT_DECOMPRESSED;
        call_back.ux_host_class_hid_report_callback_length =     0;

        /* Register the report call back when data comes it on this report.  */
        status =  _ux_host_class_hid_report_callback_register(hid, &call_back);
    }

#if !defined(UX_HOST_STANDALONE)

    /* If we are OK, go on.  */
    if (status == UX_SUCCESS)
    {

        /* We need a semaphore now. This will be used to synchronize the HID report with the keyboard thread.  */
        status =  _ux_host_semaphore_create(&keyboard_instance -> ux_host_class_hid_keyboard_semaphore, "ux_host_class_hid_keyboard_semaphore", 0);
        if(status != UX_SUCCESS)
            status = (UX_SEMAPHORE_ERROR);
    }

    /* If we are OK, go on.  */
    if (status == UX_SUCCESS)
    {

        /* The HID Keyboard needs a Thread to process LED changes. This process is asynchronous
        to the callback because it involves using the commands on the Control Pipe which cannot
        be done during callbacks.  First allocate some stack memory.  */
        keyboard_instance -> ux_host_class_hid_keyboard_thread_stack =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_THREAD_STACK_SIZE);
                                
        /* Check if stack memory was allocated.  */
        if (keyboard_instance -> ux_host_class_hid_keyboard_thread_stack == UX_NULL)
            status = (UX_MEMORY_INSUFFICIENT);
    }

    /* If we are OK, go on.  */
    if (status == UX_SUCCESS)

        /* Then create the actual thread.  */                   
        status =  _ux_utility_thread_create(&keyboard_instance -> ux_host_class_hid_keyboard_thread, "ux_host_stack_keyboard_thread",_ux_host_class_hid_keyboard_thread,
                (ULONG) (ALIGN_TYPE) keyboard_instance, keyboard_instance -> ux_host_class_hid_keyboard_thread_stack,
                UX_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_KEYBOARD,
                UX_THREAD_PRIORITY_KEYBOARD, UX_NO_TIME_SLICE, UX_AUTO_START);

#endif

    /* If we are OK, go on.  */
    if (status == UX_SUCCESS)
    {

#if !defined(UX_HOST_STANDALONE)
        UX_THREAD_EXTENSION_PTR_SET(&(keyboard_instance -> ux_host_class_hid_keyboard_thread), keyboard_instance)
#endif

        /* Default state of keyboard is with NumLock on.  */
        keyboard_instance -> ux_host_class_hid_keyboard_alternate_key_state |= UX_HID_KEYBOARD_STATE_NUM_LOCK;

        /* We need to build the field for the LEDs. */
        keyboard_instance -> ux_host_class_hid_keyboard_led_mask =  keyboard_instance ->  ux_host_class_hid_keyboard_alternate_key_state &
                                                                    UX_HID_KEYBOARD_STATE_MASK_LOCK;

#if defined(UX_HOST_STANDALONE)

        /* Initialize the keyboard layout and use it to decode keys.  */
        keyboard_instance -> ux_host_class_hid_keyboard_layout = &ux_host_class_hid_keyboard_layout;
        keyboard_instance -> ux_host_class_hid_keyboard_keys_decode_disable = UX_FALSE;

        /* Remaining things will be done in ACTIVATE_WAIT.
            - SET_REPORT(LEDs)
            - SET_IDLE(KEYs, 0)
            - _periodic_report_start()  */
        keyboard_instance -> ux_host_class_hid_keyboard_enum_state = UX_STATE_WAIT;

        /* It's fine, replace client with our copy.  */
        hid -> ux_host_class_hid_client = hid_client;
        return(status);
#else

        /* We need to find the OUTPUT report for the keyboard LEDs.  */
        report_id.ux_host_class_hid_report_get_report = UX_NULL;
        report_id.ux_host_class_hid_report_get_type = UX_HOST_CLASS_HID_REPORT_TYPE_OUTPUT;
        status =  _ux_host_class_hid_report_id_get(hid, &report_id);

        /* The report ID should exist.  If there is an error, we do not proceed.  */
        if (status == UX_SUCCESS)
        {
        
            /* Memorize the report pointer.  */
            client_report.ux_host_class_hid_client_report =  report_id.ux_host_class_hid_report_get_report;
            
            /* The report set is raw since the LEDs mask is already in the right format.  */
            client_report.ux_host_class_hid_client_report_flags = UX_HOST_CLASS_HID_REPORT_RAW;
        
            /* The length of this report is 1 byte.  */
            client_report.ux_host_class_hid_client_report_length = 1;
        
            /* The output report buffer is the LED mask field.  */
            client_report.ux_host_class_hid_client_report_buffer = &keyboard_instance -> ux_host_class_hid_keyboard_led_mask;
        
            /* The HID class will perform the SET_REPORT command.  Do not check for error here, this is
               handled by the function itself.  */
            status = _ux_host_class_hid_report_set(hid, &client_report);
        }

        /* If we are OK, go on.  */
        if (status == UX_SUCCESS)
        {

            /* Set the idle rate of the keyboard to 0. This way a report is generated only when there is an activity.  */
            status = _ux_host_class_hid_idle_set(hid, 0, keyboard_instance -> ux_host_class_hid_keyboard_id);
        }

        /* If we are OK, go on.  */
        if (status == UX_SUCCESS)
        {

            /* Initialize the keyboard layout and use it to decode keys.  */
            keyboard_instance -> ux_host_class_hid_keyboard_layout = &ux_host_class_hid_keyboard_layout;
            keyboard_instance -> ux_host_class_hid_keyboard_keys_decode_disable = UX_FALSE;

            /* Start the periodic report.  */
            status =  _ux_host_class_hid_periodic_report_start(hid);
        }

        /* If we are OK, go on.  */
        if (status == UX_SUCCESS)
        {

            /* It's fine, replace client copy.  */
            hid -> ux_host_class_hid_client = hid_client;

            /* If all is fine and the device is mounted, we may need to inform the application
               if a function has been programmed in the system structure.  */
            if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
            {
                
                /* Call system change function.  */
                _ux_system_host ->  ux_system_host_change_function(UX_HID_CLIENT_INSERTION, hid -> ux_host_class_hid_class, (VOID *) hid_client);
            }

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HID_KEYBOARD_ACTIVATE, hid, keyboard_instance, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

            /* Return completion status.  */
            return(status);    
        }

        /* There is error, delete thread.  */
        _ux_utility_thread_delete(&keyboard_instance -> ux_host_class_hid_keyboard_thread);
#endif
    }

    /* We are here if there is error.  */

    /* Free usage state.  */
    if (keyboard_instance -> ux_host_class_hid_keyboard_key_state)
        _ux_utility_memory_free(keyboard_instance -> ux_host_class_hid_keyboard_key_state);

#if !defined(UX_HOST_STANDALONE)

    /* Free stack.  */
    if (keyboard_instance -> ux_host_class_hid_keyboard_thread_stack)
        _ux_utility_memory_free(keyboard_instance -> ux_host_class_hid_keyboard_thread_stack);

    /* Delete semaphore.  */
    if (keyboard_instance -> ux_host_class_hid_keyboard_semaphore.tx_semaphore_id != UX_EMPTY)
        _ux_host_semaphore_delete(&keyboard_instance -> ux_host_class_hid_keyboard_semaphore);

#endif

    /* Free usage array.  */
    if (keyboard_instance -> ux_host_class_hid_keyboard_usage_array)
        _ux_utility_memory_free(keyboard_instance -> ux_host_class_hid_keyboard_usage_array);

    /* Free instance.  */
    _ux_utility_memory_free(keyboard_instance);

    /* Return completion status.  */
    return(status);
}
