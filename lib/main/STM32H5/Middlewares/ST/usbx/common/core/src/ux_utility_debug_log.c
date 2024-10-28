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
/**   Utility                                                             */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"


#ifdef UX_ENABLE_DEBUG_LOG

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_utility_debug_log                               PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function logs a debug msg in a circular queue. The queue       */ 
/*    must be initialized during the init of USBX.                        */ 
/*                                                                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*   debug_location                         C string to locate the debug, */
/*                                          for example source file name. */
/*   debug_message                          C string of message           */
/*   debug_code                             Debug code                    */
/*   debug_parameter_1                      First parameter               */
/*   debug_parameter_2                      Second parameter              */
/*                                                                        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_string_length_check       Check C string and return     */
/*                                          its length if null-terminated */
/*    _tx_time_get                          Return system clock time      */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases, used UX prefix to    */
/*                                            refer to TX symbols instead */
/*                                            of using them directly,     */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
VOID  _ux_utility_debug_log(UCHAR *debug_location, UCHAR *debug_message, ULONG debug_code,
                            ULONG debug_parameter_1, ULONG debug_parameter_2)
{

UINT    debug_location_string_length;
UINT    debug_message_string_length;
ULONG   total_debug_message_length;
ULONG   parameter_length;
ULONG   parameter_shift;
UCHAR   parameter_hexa;
ULONG   local_parameter_value;
ULONG   current_time;
UX_INTERRUPT_SAVE_AREA

    /* Is USBX system completely initialized ?  */
    if (_ux_system -> ux_system_debug_log_size == 0)
    
        /* Not yet.  */
        return;

    /* Entering critical area. Disable interrupts.  */
    UX_DISABLE
    
    /* Store the debug value as the last debug value recorded.  */
    _ux_system -> ux_system_debug_code = debug_code;

    /* Increment the debug value code count.  */
    _ux_system -> ux_system_debug_count++;

    /* Calculate the string length.  */
    debug_location_string_length = UX_DEBUG_LOG_SIZE;
    _ux_utility_string_length_check(source, &debug_location_string_length, UX_DEBUG_LOG_SIZE);
    debug_message_string_length = UX_DEBUG_LOG_SIZE;
    _ux_utility_string_length_check(source, &debug_message_string_length, UX_DEBUG_LOG_SIZE);
    
    /* Calculate the length of the entire message string.  1 fixed string, then 1 hexa
       decimal, then 2 strings then 2 hexa decimal numbers in the format 0x00000000 
       separated by commas and . at the end, zero terminated.  */
    total_debug_message_length = debug_location_string_length + debug_message_string_length + 10 + 10 + 10 + 10 + 5;

    /* Can we accommodate this debug value message at the current location ?  */
    if (total_debug_message_length >= _ux_system -> ux_system_debug_log_size)
        return;
    if (_ux_system -> ux_system_debug_log_head +  total_debug_message_length > 
        ux_system -> ux_system_debug_log_buffer + _ux_system -> ux_system_debug_log_size)
    {

        /* The debug value log to insert goes beyond the end of the log buffer, rewind to the beginning.  */
        _ux_system -> ux_system_debug_log_head = _ux_system -> ux_system_debug_log_buffer;
    }

    /* Copy the time strings and parameters in the log buffer.  */
    _ux_utility_memory_copy(_ux_system -> ux_system_debug_log_head, "At time : ", 10); /* Use case of memcpy is verified. */
    _ux_system -> ux_system_debug_log_head += 10;

    /* Get the time value from TX.  */
    current_time = _tx_time_get();                        

    /* Reset the value of the length.*/
    parameter_length = 0;

    /* Init the shift value.  */
    parameter_shift = 32 - 4;

    /* We parse the hexa value parameter and build the hexa value one byte at a type.  */
    while(parameter_length < 8)
    {
    
        /* Shift the 4 bit value we are interested in.  We keep the lowest nibble.  */
        local_parameter_value = (current_time >> parameter_shift) & 0x0f;
    
        /* See if this value is from 0-9 or A to F.  */
        if (local_parameter_value <= 9)
            
            /* We have a digit.  */
            parameter_hexa = (UCHAR) (local_parameter_value + '0');
        
        else

            /* We have  'A' to 'F' value.  */
            parameter_hexa = (UCHAR) (local_parameter_value - 10 + 'A');
        
        /* Store the converted hexa value.  */
        *_ux_system -> ux_system_debug_log_head = parameter_hexa;

        /* Next position.  */
        _ux_system -> ux_system_debug_log_head++;
    
        /* Update length.  */
        parameter_length++;

        /* Continue shifting by one nibble.  */
        parameter_shift = parameter_shift - 4;        
    }       

    /* Add the comma after the time.  */
    *_ux_system -> ux_system_debug_log_head = ',';
    _ux_system -> ux_system_debug_log_head++;

    /* Copy the strings and parameters in the log buffer.  */
    _ux_utility_memory_copy(_ux_system -> ux_system_debug_log_head, debug_location, debug_location_string_length); /* Use case of memcpy is verified. */
    _ux_system -> ux_system_debug_log_head += debug_location_string_length;
    *_ux_system -> ux_system_debug_log_head = ',';
    _ux_system -> ux_system_debug_log_head++;
    _ux_utility_memory_copy(_ux_system -> ux_system_debug_log_head, debug_message, debug_message_string_length); /* Use case of memcpy is verified. */
    _ux_system -> ux_system_debug_log_head += debug_message_string_length;
    *_ux_system -> ux_system_debug_log_head = ',';
    _ux_system -> ux_system_debug_log_head++;

    /* Create the hexa string for parameter 1.  */
    *_ux_system -> ux_system_debug_log_head = '0';
    _ux_system -> ux_system_debug_log_head++;
    *_ux_system -> ux_system_debug_log_head = 'x';
    _ux_system -> ux_system_debug_log_head++;

    /* Reset the value of the length.*/
    parameter_length = 0;

    /* Init the shift value.  */
    parameter_shift = 32 - 4;

    /* We parse the hexa value parameter and build the hexa value one byte at a type.  */
    while(parameter_length < 8)
    {
    
        /* Shift the 4 bit value we are interested in.  We keep the lowest nibble.  */
        local_parameter_value = (debug_parameter_1 >> parameter_shift) & 0x0f;
    
        /* See if this value is from 0-9 or A to F.  */
        if (local_parameter_value <= 9)
            
            /* We have a digit.  */
            parameter_hexa = (UCHAR) (local_parameter_value + '0');
        
        else

            /* We have  'A' to 'F' value.  */
            parameter_hexa = (UCHAR) (local_parameter_value - 10 + 'A');
        
        /* Store the converted hexa value.  */
        *_ux_system -> ux_system_debug_log_head = parameter_hexa;

        /* Next position.  */
        _ux_system -> ux_system_debug_log_head++;
    
        /* Update length.  */
        parameter_length++;

        /* Continue shifting by one nibble.  */
        parameter_shift = parameter_shift - 4;        
    }       

    /* Add the comma between the 2 hexa values.  */
    *_ux_system -> ux_system_debug_log_head = ',';
    _ux_system -> ux_system_debug_log_head++;

    /* Create the hexa string for parameter 2.  */
    *_ux_system -> ux_system_debug_log_head = '0';
    _ux_system -> ux_system_debug_log_head++;
    *_ux_system -> ux_system_debug_log_head = 'x';
    _ux_system -> ux_system_debug_log_head++;

    /* Reset the value of the length.*/
    parameter_length = 0;

    /* Init the shift value.  */
    parameter_shift = 32 - 4;

    /* We parse the hexa value parameter and build the hexa value one byte at a type.  */
    while(parameter_length < 8)
    {
    
        /* Shift the 4 bit value we are interested in.  We keep the lowest nibble.  */
        local_parameter_value = (debug_parameter_2 >> parameter_shift) & 0x0f;
    
        /* See if this value is from 0-9 or A to F.  */
        if (local_parameter_value <= 9)
            
            /* We have a digit.  */
            parameter_hexa = (UCHAR) (local_parameter_value + '0');
        
        else

            /* We have  'A' to 'F' value.  */
            parameter_hexa = (UCHAR) (local_parameter_value - 10 + 'A');
        
        /* Store the converted hexa value.  */
        *_ux_system -> ux_system_debug_log_head = parameter_hexa;

        /* Next position.  */
        _ux_system -> ux_system_debug_log_head++;
    
        /* Update length.  */
        parameter_length++;

        /* Continue shifting by one nibble.  */
        parameter_shift = parameter_shift - 4;        
    }       

    /* Add the termination dot at the end.   */
    *_ux_system -> ux_system_debug_log_head = '.';
    _ux_system -> ux_system_debug_log_head++;

    /* Add the CR/LF end of the string.  */
    *_ux_system -> ux_system_debug_log_head = 0x0d;
    _ux_system -> ux_system_debug_log_head++;
    *_ux_system -> ux_system_debug_log_head = 0x0a;
    _ux_system -> ux_system_debug_log_head++;

    /* Add the zero end of the string.  */
    *_ux_system -> ux_system_debug_log_head = 0x00;
    _ux_system -> ux_system_debug_log_head++;

    /* The log string is put into the log buffer.  It can stay here until 
       a break into debugger by the developer or be passed to a callback registered
       by the application.  */
    if (_ux_system -> ux_system_debug_callback_function != UX_NULL)
    {    

        /* The callback function is defined, call it.  */
        _ux_system -> ux_system_debug_callback_function(_ux_system -> ux_system_debug_log_tail, debug_code);

        /* Update the tail.  */
        _ux_system -> ux_system_debug_log_tail += total_debug_message_length;

    }

    /* Restore interrupts.  */
    UX_RESTORE

    /* We are done here. No return codes.  */
    return;
}

#endif
