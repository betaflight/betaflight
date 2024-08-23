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
/*    _ux_pictbridge_xml_function_input_startjob_printinfo_croppingarea   */ 
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
/*    This function decodes the "croppingarea" tag                        */ 
/*    It is followed by 4 coordinates in hexa form.                       */ 
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
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_xml_function_input_startjob_printinfo_croppingarea(UX_PICTBRIDGE *pictbridge, 
                            UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter)
{
UINT    length = 0;
ULONG   index;      
UINT    status;
UCHAR   element_string[UX_PICTBRIDGE_MAX_ELEMENT_SIZE];
UCHAR   *element_ptr;
ULONG   element_length;
ULONG   hexa_value;

    UX_PARAMETER_NOT_USED(input_string);
    UX_PARAMETER_NOT_USED(input_variable);

    /* The xml parameter is pointing to the beginning of the 4 coordinates. They are separated
       by spaces. Each has 4 bytes in hexadecimal. We need to extract each ones and store them
       in the printinfo area.  */
    status = _ux_utility_string_length_check(xml_parameter, &length, UX_PICTBRIDGE_MAX_TAG_SIZE);
    if (status != UX_SUCCESS)
        return(status);
       
    /* Ensure the length is non zero, otherwise that means the tag has no parameter.  */
    if (length == 0)

        /* That's a syntax error.  */
        return(UX_PICTBRIDGE_ERROR_SCRIPT_SYNTAX_ERROR);    

    /* Reset the cropping area index. */
    index = 0;

    /* Reset the element area.  */
   _ux_utility_memory_set(element_string, 0, UX_PICTBRIDGE_MAX_ELEMENT_SIZE); /* Use case of memset is verified. */
   
   /* And its length.  */
   element_length = 0;
   
   /* Set the pointer to the beginning of the element string.  */
   element_ptr =  element_string;
        
    /* Now parse the line and isolate each component of the cropping area.  */    
    /* Get the length of the "papersize " variable.  */
    while(length != 0)
    {

        /* Get a character from the xml parameter.  Check for delimiter. */
        if (*xml_parameter == UX_PICTBRIDGE_TAG_CHAR_SPACE)
        {
    
            /* Do we have a element ?  */
            if (element_length != 0)
            {

                /* Yes we do, get the hexa value for that element.  */
                status =  _ux_pictbridge_element_to_hexa(element_string, &hexa_value);
                
                /* Check for error.  */
                if (status != UX_SUCCESS)
                    
                    /* We have a element syntax error.  */
                    return(status);
                    
                /* Where to store this element.  */
                switch (index)
                {
                
                    case 0 :
                        
                        /* This is the X coordinate of the cropping area.  */
                        pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_printinfo_current -> ux_pictbridge_printinfo_croppingarea_xcoordinate = hexa_value;
                        
                        break;
                        
                    case 1 :

                        /* This is the Y coordinate of the cropping area.  */
                        pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_printinfo_current -> ux_pictbridge_printinfo_croppingarea_ycoordinate = hexa_value;
                        
                        break;
                    case 2 :

                        /* This is the width of the cropping area.  */
                        pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_printinfo_current -> ux_pictbridge_printinfo_croppingarea_width = hexa_value;
                        
                        break;
                    case 3 :
                        /* This is the height of the cropping area.  */
                        pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_printinfo_current -> ux_pictbridge_printinfo_croppingarea_height = hexa_value;
                        
                        break;
                }                    

                /* Next index now.  */
                index++;

                /* Reset the element area.  */
               _ux_utility_memory_set(element_string, 0, UX_PICTBRIDGE_MAX_ELEMENT_SIZE); /* Use case of memset is verified. */
   
               /* And its length.  */
               element_length = 0;

               /* Set the pointer to the beginning of the element string.  */
               element_ptr =  element_string;
                                
            }            
        }
        else
        {

            /* We have a regular character. Store it in the element string buffer. No need to check
               the syntax here as the string to hexa conversion will do that.  */
            *element_ptr++ = *xml_parameter;
            
            /* Increase the length of the element.  */
            element_length++;

        }


        /* Next position.  */
        xml_parameter++;
    
        /* Update length.  */
        length--;

    }

    /* We get here when we reached the end of the xml parameter. We may still have the last element of the cropping area to parse. 
       The index value will tell us what to do.  */
    switch (index)
    {

        case 3 :

            /* Still need to parse the last element. Get the hexa value for that element.  */
            status =  _ux_pictbridge_element_to_hexa(element_string, &hexa_value);
        
            /* Check for error.  */
            if (status != UX_SUCCESS)
            
                /* We have a element syntax error.  */
                return(status);

            /* This is the height of the cropping area.  */
            pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_printinfo_current -> ux_pictbridge_printinfo_croppingarea_height = hexa_value;

            /* Operation is successful.  */
            return(UX_SUCCESS);

        case 4:

            /* Operation is successful.  */
            return(UX_SUCCESS);

        default :

            /* Syntax error.  Missing element of the cropping area.  */
            return(UX_PICTBRIDGE_ERROR_SCRIPT_SYNTAX_ERROR);    
    }                        
    
}


