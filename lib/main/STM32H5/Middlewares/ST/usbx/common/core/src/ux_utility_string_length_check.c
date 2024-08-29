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


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_utility_string_length_check                     PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function checks if a NULL-terminated C string reaches its end  */ 
/*    before specified length exceeds.                                    */ 
/*                                                                        */ 
/*    On success the actual length of C string is written back to UINT    */ 
/*    variable pointed by string_length_ptr (if not NULL).                */ 
/*    Otherwise the variable keeps untouched.                             */ 
/*                                                                        */
/*    Note NULL terminator is not counted in string length                */
/*    (same as C strlen).                                                 */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    string                                Pointer to string             */ 
/*    string_length_ptr                     Pointer to UINT to receive    */ 
/*                                            the string length           */ 
/*    max_string_length                     Max string length             */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    returns success if the string length was less than the max length,  */ 
/*    else it returns error                                               */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX Components                                                     */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_string_length_check(UCHAR *string, UINT *string_length_ptr, UINT max_string_length)
{

UINT    string_length;


    if (string == UX_NULL)
        return(UX_ERROR);

    string_length = 0;

    while (1)
    {

        if (string[string_length] == '\0')
            break;

        string_length++;
        if (string_length > max_string_length)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_UTILITY, UX_ERROR);

            return(UX_ERROR);
        }
    }

    if (string_length_ptr)
        *string_length_ptr = string_length;

    return(UX_SUCCESS); 
}

