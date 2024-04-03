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
/*    _ux_pictbridge_xml_function_root_xml                PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function decodes the "xml" tag                                 */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pictbridge                             Pictbridge instance          */ 
/*    input_variable                         Pointer to variable          */ 
/*    input_string                           Pointer to string            */ 
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
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_xml_function_root_xml(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, 
                                            UCHAR *input_string, UCHAR *xml_parameter)
{
UINT   status;
UINT   input_length = 0;
UINT   length = 0;

    UX_PARAMETER_NOT_USED(input_string);
    UX_PARAMETER_NOT_USED(xml_parameter);
    UX_PARAMETER_NOT_USED(pictbridge);

    /* Get the length of the variable. */
    status = _ux_utility_string_length_check(input_variable, &input_length, UX_PICTBRIDGE_MAX_VARIABLE_SIZE);
    if (status != UX_SUCCESS)
        return(status);
    
    /* Get the length of the "version " variable.  */
    status = _ux_utility_string_length_check(_ux_pictbridge_xml_variable_version, &length, UX_PICTBRIDGE_MAX_VARIABLE_SIZE);
    if (status != UX_SUCCESS)
        return(status);

    /* If length do not match, no need to check for a match.  */
    if (length == input_length)
    {

        /* Both length match, we may have a variable name match. Check the names */
        if (_ux_utility_memory_compare(_ux_pictbridge_xml_variable_version,input_variable, length) == UX_SUCCESS)
        {
        
            /* The variable name is OK, we do not check the xml version.  */
            return(UX_SUCCESS);
        
        }

    }
    /* We get here when we reached an unexpected end of the XML object.  */
    return(UX_ERROR);    
}

