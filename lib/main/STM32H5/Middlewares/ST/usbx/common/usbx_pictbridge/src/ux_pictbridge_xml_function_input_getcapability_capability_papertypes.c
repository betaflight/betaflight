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
/*    _ux_pictbridge_xml_function_input_getcapability_                    */
/*                              capability_papertypes                     */
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
/*    This function decodes the "layouts" tag                             */ 
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
UINT  _ux_pictbridge_xml_function_input_getcapability_capability_papertypes(UX_PICTBRIDGE *pictbridge, 
                            UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter)
{
UINT    input_length = 0;
UINT    length = 0;
ULONG   papersize;
UINT    status;

    UX_PARAMETER_NOT_USED(xml_parameter);

    /* Get the length of the variable. This is an option and there may be none.*/
    status = _ux_utility_string_length_check(input_variable, &input_length, UX_PICTBRIDGE_MAX_VARIABLE_SIZE);
    if (status != UX_SUCCESS)
        return(status);
    
    /* Check if there is a variable defined.  */
    if (input_length == 0)
    {

        /* No variable. Reset the layouts paperTypes option.  */
        pictbridge -> ux_pictbridge_dpsclient.ux_pictbridge_devinfo_papertypes_papersize =  0;

        /* Return success.  */
        return(UX_SUCCESS);
    }
    
    /* Get the length of the "papersize " variable.  */
    status = _ux_utility_string_length_check(_ux_pictbridge_xml_variable_papersize, &length, UX_PICTBRIDGE_MAX_VARIABLE_SIZE);
    if (status != UX_SUCCESS)
        return(status);

    /* If length do not match, no need to check for a match.  */
    if (length == input_length)
    {

        /* Both length match, we may have a variable name match. Check the names */
        if (_ux_utility_memory_compare(_ux_pictbridge_xml_variable_papersize,input_variable, length) == UX_SUCCESS)
        {
        
            /* Get the value for the papersize. */
            status = _ux_pictbridge_element_to_hexa(input_string, &papersize);
            
            /* Check status. If OK, save the papersize value for layout.  */
            if (status == UX_SUCCESS)
            {

                /* Set the layouts paperTypes option with the string value.  */
                pictbridge -> ux_pictbridge_dpsclient.ux_pictbridge_devinfo_papertypes_papersize =  papersize;

                /* The variable name is OK. We are done.  */
                return(UX_SUCCESS);
            }        
        }

    }
    /* We get here when we reached an unexpected end of the XML object or a format error.  */
    return(UX_PICTBRIDGE_ERROR_SCRIPT_SYNTAX_ERROR);    
}


