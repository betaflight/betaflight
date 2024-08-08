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
/**   Pictbridge Application                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_pictbridge.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_xml_function_input_startjob_printinfo_date           */ 
/*                                                        PORTABLE C      */ 
/*                                                           6.1          */
/*                                                                        */ 
/*                                                                        */ 
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function decodes the "date" tag                                */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pictbridge                             Pictbridge instance          */ 
/*    input_variable                         Pointer to variable          */ 
/*    input_string                           Pointer to string            */ 
/*    xml_parameter                          XML parameter                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_pictbridge_object_parse                                         */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_xml_function_input_startjob_printinfo_date(UX_PICTBRIDGE *pictbridge, 
                            UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter)
{
UINT    status;
UINT    length = 0;

    UX_PARAMETER_NOT_USED(input_string);
    UX_PARAMETER_NOT_USED(input_variable);

    /* Get the length of the parameter. */
    status = _ux_utility_string_length_check(xml_parameter, &length, UX_PICTBRIDGE_MAX_STRING_SIZE);
    if (status != UX_SUCCESS)
        return(status);

    /* Ensure the length is non zero, otherwise that means the tag has no parameter.  */
    if (length == 0)

        /* That's a syntax error.  */
        return(UX_PICTBRIDGE_ERROR_SCRIPT_SYNTAX_ERROR);    
    
    /* We need to copy the date.  */
    _ux_utility_memory_copy(pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_printinfo_current -> ux_pictbridge_printinfo_date,
                            xml_parameter, length); /* Use case of memcpy is verified. */

    /* Operation is successful.  */
    return(UX_SUCCESS);
    
}


