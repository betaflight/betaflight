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

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_keyboard_callback                PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the callback mechanism for a report registration.  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    callback                              Pointer to callback           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_copy               Copy memory                   */ 
/*    _ux_utility_memory_set                Set memory                    */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HID Class when a report is generated                                */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_hid_keyboard_callback(UX_HOST_CLASS_HID_REPORT_CALLBACK *callback)
{

/* This array contains the bit for each alternate key (modifier or lock key) 
   that we report to the application.  For example, if you wanted to set the 
   bit for the CAPS_LOCK key, you would do:
        keyboard_instance -> ux_host_class_hid_keyboard_alternate_key_state |= alternate_key_bits[0];
   Index 0 is used since it corresponds to the CAPS_LOCK bit in the array. Note 
   that _alternate_key_state is what we report to the application.  */
const ULONG                         alternate_key_bits[] = {
    UX_HID_KEYBOARD_STATE_CAPS_LOCK,
    UX_HID_KEYBOARD_STATE_NUM_LOCK,
    UX_HID_KEYBOARD_STATE_SCROLL_LOCK,
    UX_HID_KEYBOARD_STATE_LEFT_CTRL,
    UX_HID_KEYBOARD_STATE_LEFT_SHIFT,
    UX_HID_KEYBOARD_STATE_LEFT_ALT,
    UX_HID_KEYBOARD_STATE_LEFT_GUI,
    UX_HID_KEYBOARD_STATE_RIGHT_CTRL,
    UX_HID_KEYBOARD_STATE_RIGHT_SHIFT,
    UX_HID_KEYBOARD_STATE_RIGHT_ALT,
    UX_HID_KEYBOARD_STATE_RIGHT_GUI,
};

/* Define the indices for each alternate key in the alternate key bits array.  */

#define ALTERNATE_KEY_BITS_IDX_CAPS_LOCK        ( 0)
#define ALTERNATE_KEY_BITS_IDX_NUM_LOCK         ( 1)
#define ALTERNATE_KEY_BITS_IDX_SCROLL_LOCK      ( 2)

#define ALTERNATE_KEY_BITS_IDX_LEFT_CTRL        ( 3)
#define ALTERNATE_KEY_BITS_IDX_LEFT_SHIFT       ( 4)
#define ALTERNATE_KEY_BITS_IDX_LEFT_ALT         ( 5)
#define ALTERNATE_KEY_BITS_IDX_LEFT_GUI         ( 6)
#define ALTERNATE_KEY_BITS_IDX_RIGHT_CTRL       ( 7)
#define ALTERNATE_KEY_BITS_IDX_RIGHT_SHIFT      ( 8)
#define ALTERNATE_KEY_BITS_IDX_RIGHT_ALT        ( 9)
#define ALTERNATE_KEY_BITS_IDX_RIGHT_GUI        (10)

/* Define a macro to get the index of a modifier key in the alternate key bits array.  */
#define GET_ALTERNATE_KEY_BITS_IDX(usage)       ((usage) - UX_HID_MODIFIER_KEY_LEFT_CONTROL + ALTERNATE_KEY_BITS_IDX_LEFT_CTRL)

/* Define key states.  */
#define KEY_STATE_REGULAR                       (11)
#define KEY_STATE_NO_KEY                        (12)

#define KEY_UP                                  ( 0)
#define KEY_KEEP                                ( 1)
#define KEY_DOWN                                ( 2)
#define KEY_DEL                                 ( 3)


UX_HOST_CLASS_HID_CLIENT            *hid_client;
UX_HOST_CLASS_HID_KEYBOARD          *keyboard_instance;
UX_HOST_CLASS_HID_KEYBOARD_LAYOUT   *keyboard_layout;
UCHAR                               *keypad_array;
ULONG                               *array_head;
ULONG                               *array_tail;
ULONG                               *array_end;
ULONG                               *array_start;
ULONG                               *report_buffer;
ULONG                               *report_buffer_end;
ULONG                               keyboard_char = 0;
ULONG                               shift_on;
ULONG                               capslock_on;
ULONG                               numlock_on;
ULONG                               key_usage;
ULONG                               key_value;

/* This variable either contains an index into the alternate key bit array,
   or a value that describes the current key i.e. regular key or no key.  */
UINT                                key_state;

#if !defined(UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE)
UCHAR                               *previous_lock_key_states;
UCHAR                               *current_lock_key_states;
#else
UINT                                i, i_save, new_count;
UCHAR                               *state_usage;
UCHAR                               *state_value;
UCHAR                               *state_action;
#endif


    /* Get the HID client instance that issued the callback.  */
    hid_client = callback -> ux_host_class_hid_report_callback_client;

    /* Get the keyboard local instance.  */
    keyboard_instance =  (UX_HOST_CLASS_HID_KEYBOARD *) hid_client -> ux_host_class_hid_client_local_instance;

    /* Get the report buffer.  */
    report_buffer = (ULONG *)callback -> ux_host_class_hid_report_callback_buffer;

    /* Get the end of report buffer.  */
    report_buffer_end = &report_buffer[callback -> ux_host_class_hid_report_callback_actual_length];

#if !defined(UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE)

    /* Get the previous states of the lock keys.  */
    previous_lock_key_states = &keyboard_instance -> ux_host_class_hid_keyboard_key_state[0];

    /* Get the current states of the lock keys and immediately initialize them to zero;
       if a lock key is not pressed in this report, it will remain zero (not pressed).  */
    current_lock_key_states = &keyboard_instance -> ux_host_class_hid_keyboard_key_state[3];
    _ux_utility_memory_set(current_lock_key_states, 0, 3); /* Use case of memset is verified. */

    /* Scan the report buffer and decode it.  */
    while(report_buffer < report_buffer_end)
    {

        /* Get usage and value from buffer.  */
        key_usage = *report_buffer ++;
        key_value = *report_buffer ++;

        /* Analyze the usage we have received.  We eliminate the page from the usage.  */
        key_usage &= 0xFF;
#else

    /* Initialize key states for report processing.  */
    state_usage  = keyboard_instance -> ux_host_class_hid_keyboard_key_state;
    state_value  = state_usage  + keyboard_instance -> ux_host_class_hid_keyboard_key_count * 2;
    state_action = state_value  + keyboard_instance -> ux_host_class_hid_keyboard_key_count * 2;

    /* Reset state actions to DEL(not received).  */
    _ux_utility_memory_set(state_usage + keyboard_instance -> ux_host_class_hid_keyboard_key_count, 0, keyboard_instance -> ux_host_class_hid_keyboard_key_count); /* Use case of memset is verified. */
    _ux_utility_memory_set(state_value + keyboard_instance -> ux_host_class_hid_keyboard_key_count, 0, keyboard_instance -> ux_host_class_hid_keyboard_key_count); /* Use case of memset is verified. */
    _ux_utility_memory_set(state_action, KEY_DEL, keyboard_instance -> ux_host_class_hid_keyboard_key_count * 2); /* Use case of memset is verified. */

    new_count = keyboard_instance -> ux_host_class_hid_keyboard_key_count;
    while(report_buffer < report_buffer_end)
    {

        /* Get usage and value from buffer.  */
        key_usage = *report_buffer ++;
        key_value = *report_buffer ++;

        /* Analyze the usage we have received.  We eliminate the page from the usage.  */
        key_usage &= 0xFF;
        key_value &= 0xFF;

        /* If there is no key or in phantom state (roll over), skip.  */
        if (key_usage <= UX_HID_KEYBOARD_PHANTOM_STATE)
            continue;

        /* Check if the key is previously reported.  */
        for (i = 0; i < keyboard_instance -> ux_host_class_hid_keyboard_key_count; i ++)
        {

            /* Check if it's modified.  */
            if (state_usage[i] == key_usage)
            {

                /* Replace action state.  */
                state_action[i] = (state_value[i] == key_value) ? KEY_KEEP : (key_value ? KEY_DOWN : KEY_UP);

                /* Replace key value.  */
                state_value[i] = key_value;
                break;
            }
        }

        /* When there is new key, add to new key list.  */
        if (i == keyboard_instance -> ux_host_class_hid_keyboard_key_count)
        {

            /* Add key value.  */
            state_usage [new_count] = key_usage;
            state_value [new_count] = key_value;

            /* Add key action.  */
            state_action[new_count] = key_value ? KEY_DOWN : KEY_KEEP;

            new_count ++;
        }
    } /* while(report_buffer < report_buffer_end) */

    /* Process pending key states.  */
    i_save = 0;
    for (i = 0; i < new_count; i ++)
    {

        /* Get state value from buffer.  */
        key_usage = state_usage[i];
        key_value = state_value[i];
        key_state  = state_action[i];

        /* If no key, just skip.  */
        if (key_usage == 0)
            continue;

        /* If key not reported, add up event if it's enabled.  */
        if (key_state == KEY_DEL)
        {

            /* Clear state, do not save.  */
            state_usage[i]  = 0;
            state_action[i] = KEY_UP;
        }

        /* Key reported, process it.  */
        else
        {

            /* We need to save key anyway.  */
            if (i_save < i)
            {
                state_usage[i_save] = key_usage;
                state_value[i_save] = key_value;
            }
            i_save ++;
        }

        /* Skip keep keys.  */
        if (state_action[i] == KEY_KEEP)
            continue;

        /* Now handle key event.  */
        keyboard_instance -> ux_host_class_hid_keyboard_alternate_key_state &= ~(UX_HID_KEYBOARD_STATE_FUNCTION | UX_HID_KEYBOARD_STATE_KEY_UP);
#endif /* UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE */

        /* Determine what key this is.  */
        switch(key_usage)
        {

        /* This is the usage of a modifier key. Set or clear the appropriate 
           bits in the alternate key state bitmap.  */
        case    UX_HID_MODIFIER_KEY_LEFT_SHIFT           :
        case    UX_HID_MODIFIER_KEY_RIGHT_SHIFT          :
        case    UX_HID_MODIFIER_KEY_LEFT_ALT             :
        case    UX_HID_MODIFIER_KEY_RIGHT_ALT            :
        case    UX_HID_MODIFIER_KEY_RIGHT_CONTROL        :
        case    UX_HID_MODIFIER_KEY_LEFT_CONTROL         :
        case    UX_HID_MODIFIER_KEY_RIGHT_GUI            :
        case    UX_HID_MODIFIER_KEY_LEFT_GUI             :

            key_state =  GET_ALTERNATE_KEY_BITS_IDX(key_usage);

            /* We have received a modifier Key. Remember the state. */
            if (key_value > 0)
                keyboard_instance -> ux_host_class_hid_keyboard_alternate_key_state |= alternate_key_bits[key_state];
            else
                keyboard_instance -> ux_host_class_hid_keyboard_alternate_key_state &= ~alternate_key_bits[key_state];

            break;

        /* This is the usage of a LOCK key. Just save the its index in the alternate
           key bit array.  */

        case    UX_HID_LED_KEY_CAPS_LOCK                 :
            key_state = ALTERNATE_KEY_BITS_IDX_CAPS_LOCK;
            break;

        case    UX_HID_LED_KEY_NUM_LOCK                  :
            key_state = ALTERNATE_KEY_BITS_IDX_NUM_LOCK;
            break;

        case    UX_HID_LED_KEY_SCROLL_LOCK               :
            key_state = ALTERNATE_KEY_BITS_IDX_SCROLL_LOCK;
            break;

#if !defined(UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE)

        /* Usage no key.  */
        case    UX_HID_KEYBOARD_NO_KEY                  :
        case    UX_HID_KEYBOARD_PHANTOM_STATE           :
            key_state = KEY_STATE_NO_KEY;
            break;
#endif
        
        /* This is the usage of a regular key. Here, we just get the decoded 
           value; we will add it to the queue later.  */
        default :

            /* By default the key will be saved.  */
            key_state = KEY_STATE_REGULAR;

            /* Skip decode if decode is disabled.  */
            if (keyboard_instance -> ux_host_class_hid_keyboard_keys_decode_disable == UX_TRUE)
            {

                /* Use raw data (scan code) as key code.  */
                keyboard_char = key_value;
                break;
            }

            /* Get keyboard layout instance.  */
            keyboard_layout = keyboard_instance -> ux_host_class_hid_keyboard_layout;

            /* Is this key outside the valid range?   */
            if (key_value > keyboard_layout -> ux_host_class_hid_keyboard_layout_keys_upper_range)
            {

                /* Set a flag to discard it.  */
                key_state = KEY_STATE_NO_KEY;
                break;
            }

            /* We have received a regular key. Depending on the state of the shift or numlock status, the key should be mapped into
                one of the translation tables.  We verify if the key is within our mapping range. */

            /* Get SHIFT, CAPS_LOCK and NUM_LOCK states.  */
            shift_on = (keyboard_instance -> ux_host_class_hid_keyboard_alternate_key_state & UX_HID_KEYBOARD_STATE_SHIFT) ? UX_TRUE : UX_FALSE;
            capslock_on = (keyboard_instance -> ux_host_class_hid_keyboard_alternate_key_state & UX_HID_KEYBOARD_STATE_CAPS_LOCK) ? UX_TRUE : UX_FALSE;
            numlock_on = (keyboard_instance -> ux_host_class_hid_keyboard_alternate_key_state & UX_HID_KEYBOARD_STATE_NUM_LOCK) ? UX_TRUE : UX_FALSE;

            /* Check if we have letters ('a' to 'z').  */
            if (key_value >= keyboard_layout -> ux_host_class_hid_keyboard_layout_letters_lower_range &&
                key_value <= keyboard_layout -> ux_host_class_hid_keyboard_layout_letters_upper_range)
            {

                /* We have letters, check the Shift and CapsLock state.  */
                if (shift_on != capslock_on)

                    /* Shift and CapsLock in different state: upper case.  */
                    keyboard_char = keyboard_layout -> ux_host_class_hid_keyboard_layout_shift_array[key_value];
                else

                    /* Lower case.  */
                    keyboard_char = keyboard_layout -> ux_host_class_hid_keyboard_layout_regular_array[key_value];

                break; /* default: */
            }

            /* Check if we have received a keypad key. They may be multiplexed. */
            if (key_value >= keyboard_layout -> ux_host_class_hid_keyboard_layout_keypad_lower_range &&
                key_value <= keyboard_layout -> ux_host_class_hid_keyboard_layout_keypad_upper_range)
            {

                /* We have a keypad key. Check the NumLock state.  */
                if (numlock_on)

                    /* Numlock is on.  */
                    keypad_array = keyboard_layout -> ux_host_class_hid_keyboard_layout_numlock_on_array;

                else

                    /* Numlock is off. */
                    keypad_array = keyboard_layout -> ux_host_class_hid_keyboard_layout_numlock_off_array;

                /* Decode the keypad key.  */
                keyboard_char = keypad_array[key_value -
                                                keyboard_layout -> ux_host_class_hid_keyboard_layout_keypad_lower_range];

                break; /* default: */
            }

            /* Check the state of the shift.  */
            if (shift_on)

                /* We get the key from the shifted array.  */
                keyboard_char = keyboard_layout -> ux_host_class_hid_keyboard_layout_shift_array[key_value];

            else

                /* We get the key from the regular array.  */
                keyboard_char = keyboard_layout -> ux_host_class_hid_keyboard_layout_regular_array[key_value];

            break; /* default: */

        } /* switch(key_usage)  */

#if defined(UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE)

        if (state_action[i] == KEY_UP)

#if defined(UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE_REPORT_KEY_DOWN_ONLY)

            /* Skip save.  */
            continue;
#else

            /* Save key up state.  */
            keyboard_instance -> ux_host_class_hid_keyboard_alternate_key_state |= UX_HID_KEYBOARD_STATE_KEY_UP;
#endif
#endif

        /* If there is no key, just try next.  */
        if (key_state == KEY_STATE_NO_KEY)
            continue;

        /* Is this a LOCK key (i.e. caps lock, scroll lock or num lock)?  */
        if (key_state <= ALTERNATE_KEY_BITS_IDX_SCROLL_LOCK)
        {

            /* Skip decode if decode is disabled.  */
            if (keyboard_instance -> ux_host_class_hid_keyboard_keys_decode_disable == UX_TRUE)
            {

                /* Use raw data (scan code) as key code.  */
                keyboard_char = key_value;
            }
            else
            {

#if !defined(UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE)

                /* Reflect the press in the current lock key state. */
                current_lock_key_states[key_state] = (UCHAR)key_value;

                /* Take action only if key state changes from up to down (pressed).
                   Remember that the nothing happens when lock keys are released.  */
                if (previous_lock_key_states[key_state] == 0)
#elif !defined(UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE_REPORT_KEY_DOWN_ONLY)

                /* Take action only if key state changes from up to down (pressed).  */
                if (state_action[i] == KEY_DOWN)
#endif
                {

                    /* Reflect the change in the keyboard state.  The state should be inverted.  */
                    if (keyboard_instance -> ux_host_class_hid_keyboard_alternate_key_state & alternate_key_bits[key_state])
                        keyboard_instance -> ux_host_class_hid_keyboard_alternate_key_state &= (ULONG)~alternate_key_bits[key_state];
                    else
                        keyboard_instance -> ux_host_class_hid_keyboard_alternate_key_state |= alternate_key_bits[key_state];

#if defined(UX_HOST_STANDALONE)

                    /* Let background task to set LED status.  */
                    keyboard_instance -> ux_host_class_hid_keyboard_out_state = UX_STATE_WAIT;
#else

                    /* Wake up the keyboard thread semaphore.  */
                    _ux_host_semaphore_put(&keyboard_instance -> ux_host_class_hid_keyboard_semaphore);
#endif
                }

#if defined(UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE) && defined(UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE_REPORT_LOCK_KEYS)

                /* Use usage and UX_HID_KEYBOARD_STATE_FUNCTION.  */
                keyboard_char = key_usage;
                keyboard_instance -> ux_host_class_hid_keyboard_alternate_key_state |= UX_HID_KEYBOARD_STATE_FUNCTION;

#else

                /* Check next usage & value.  */
                continue;
#endif
            }
        }

        /* If it's modifier, check next usage & value.  */
        else if (key_state < KEY_STATE_REGULAR)
        {
#if defined(UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE) && defined(UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE_REPORT_MODIFIER_KEYS)

            /* Use usage and UX_HID_KEYBOARD_STATE_FUNCTION.  */
            keyboard_char = key_usage;
            keyboard_instance -> ux_host_class_hid_keyboard_alternate_key_state |= UX_HID_KEYBOARD_STATE_FUNCTION;
#else

            /* Check next usage & value.  */
            continue;
#endif
        }

        /* If we get here, then we have a regular key. Now it's time to save 
           raw/decoded key in the key queue.  */

        /* This key should now be inserted in the circular array for the application to retrieve it.  */
        array_start =  keyboard_instance -> ux_host_class_hid_keyboard_usage_array;
        array_end =    array_start + UX_HOST_CLASS_HID_KEYBOARD_USAGE_ARRAY_LENGTH;
        array_head =   keyboard_instance -> ux_host_class_hid_keyboard_usage_array_head;
        array_tail =   keyboard_instance -> ux_host_class_hid_keyboard_usage_array_tail;

        /* We have a single usage/value. We have to store it into the array. If the array overflows,
            there is no mechanism for flow control here so we ignore the usage/value until the
            applications makes more room in the array.  */

        /* Is the head at the end of the array and need to loop back?  */
        if ((array_head + 2) >= array_end)
            array_head =  array_start;
        else
            array_head +=  2;

        /* Do we have enough space to store the new usage? */
        if (array_head != array_tail)
        {

            /* Yes, we have some space.  */
            *keyboard_instance -> ux_host_class_hid_keyboard_usage_array_head =        keyboard_char;
            *(keyboard_instance -> ux_host_class_hid_keyboard_usage_array_head + 1) =  keyboard_instance -> ux_host_class_hid_keyboard_alternate_key_state;

            /* Now update the array head.  */
            keyboard_instance -> ux_host_class_hid_keyboard_usage_array_head =  array_head;
        }
        else

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_BUFFER_OVERFLOW);

    } /* while(report_buffer < report_buffer_end) */

#if !defined(UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE)

    /* Copy the current lock key states to the previous states. Note that if
       a lock key wasn't down in this report, its current state would have 
       remained zero (not pressed).  */
    _ux_utility_memory_copy(previous_lock_key_states, current_lock_key_states, 3); /* Use case of memcpy is verified. */
#else

    /* Clear redundant data after last saved key.  */
    _ux_utility_memory_set(state_usage + i_save, 0, keyboard_instance -> ux_host_class_hid_keyboard_key_count - i_save); /* Use case of memset is verified. */
#endif

    /* Return to caller.  */
    return;    
}

