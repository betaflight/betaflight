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
/*    _ux_pictbridge_tag_name_get                         PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function isolates a tag name from the XML script.              */ 
/*                                                                        */
/*    Note: the buffer size of tag_name, variable_name, variable_string   */
/*    and xml_parameter must be equal to or greater than                  */
/*    UX_PICTBRIDGE_MAX_TAG_SIZE, UX_PICTBRIDGE_MAX_VARIABLE_SIZE,        */
/*    UX_PICTBRIDGE_MAX_STRING_SIZE and UX_PICTBRIDGE_MAX_STRING_SIZE.    */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    input_buffer                           Pointer to object buffer     */ 
/*    input_length                           Length of the object         */ 
/*    tag_name                               Where to store the tag       */ 
/*    variable_name                          Variable name                */  
/*    variable_string                        Variable string              */  
/*    xml_parameter                          ML parameter                 */  
/*    output_buffer                          Pointer after the tag        */ 
/*    output_length                          Length of the object         */ 
/*    tag_flag                               flag specific to this tag    */ 
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
UINT  _ux_pictbridge_tag_name_get(UCHAR *input_buffer, ULONG input_length, 
                                  UCHAR *tag_name,
                                  UCHAR *variable_name,
                                  UCHAR *variable_string,
                                  UCHAR *xml_parameter,
                                  UCHAR **output_buffer, ULONG *output_length,
                                  ULONG *tag_flag)
{
ULONG                   flag;
ULONG                   char_count;
UCHAR                   *tag_name_end;
UCHAR                   *variable_name_end;
UCHAR                   *variable_string_end;
UCHAR                   *xml_parameter_end;

    /* Reset the local flags.  */
    flag = 0;
    
    /* Reset the caller's tag_flag.  */
    *tag_flag = 0;
    
    /* Char count reset.  */
    char_count = 0;
    
    /* Reset the tag name buffer.  */
   _ux_utility_memory_set(tag_name, 0, UX_PICTBRIDGE_MAX_TAG_SIZE); /* Use case of memset is verified. */
   tag_name_end = tag_name + UX_PICTBRIDGE_MAX_TAG_SIZE - 1;

    /* Reset the variable name buffer.  */
   _ux_utility_memory_set(variable_name, 0, UX_PICTBRIDGE_MAX_VARIABLE_SIZE); /* Use case of memset is verified. */
   variable_name_end = variable_name + UX_PICTBRIDGE_MAX_VARIABLE_SIZE - 1;

    /* Reset the variable string buffer.  */
   _ux_utility_memory_set(variable_string, 0, UX_PICTBRIDGE_MAX_STRING_SIZE); /* Use case of memset is verified. */
   variable_string_end = variable_string + UX_PICTBRIDGE_MAX_STRING_SIZE - 1;

    /* Reset the xml parameter.  */
   _ux_utility_memory_set(xml_parameter, 0, UX_PICTBRIDGE_MAX_STRING_SIZE); /* Use case of memset is verified. */
   xml_parameter_end = xml_parameter + UX_PICTBRIDGE_MAX_STRING_SIZE - 1;

    /* We parse the current xml tag line. We are now positioned at the "<". */
    while(input_length)
    {
        /* Get a character from the tag line.  */
        switch (*input_buffer)
        {
                
            case    UX_PICTBRIDGE_TAG_CHAR_START_BRACKET  :
            
                /* Check to see if we are within a quote.  */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_IN_STRING)
                {

                    /* Yes, store the bracket in the string.  */
                    if (variable_string > variable_string_end)
                        return(UX_BUFFER_OVERFLOW);
                    *variable_string++ = *input_buffer;

                    break;
                }
                
                /* Check to see if we are already in the bracket.  */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_BEGIN)

                    /* Yes, we have a syntax violation.  */
                    return(UX_PICTBRIDGE_ERROR_PARAMETER_MISSING);

                else
                    /* Set the tag flag to start bracket. State machine is now in Tag. */
                    flag |= (UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_IN_TAG);


                break;
                
            
            case    UX_PICTBRIDGE_TAG_CHAR_CR             :
            case    UX_PICTBRIDGE_TAG_CHAR_LF             :

                /* If we are in the middle of a <>, we have a format violation. If not, we may have an empty line. */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_BEGIN)

                    /* We have a XML tag violation.  */
                    return(UX_PICTBRIDGE_ERROR_PARAMETER_MISSING);
                else
                    break;                    
                
            case    UX_PICTBRIDGE_TAG_CHAR_SPACE          :

                /* If we are within a tag already, this masks the end.  If not, continue looking for 
                   an open bracket. */
                if ((flag & UX_PICTBRIDGE_TAG_FLAG_BEGIN) == 0)
                    break;

                /* Check to see if we are within a quote.  */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_IN_STRING)
                {
                    /* Yes, store the character.  */
                    if (variable_string > variable_string_end)
                        return(UX_BUFFER_OVERFLOW);
                    *variable_string++ = UX_PICTBRIDGE_TAG_CHAR_SPACE;

                    break;
                }

                /* If we are in the flag state machine. It marks the end of the flag and maybe the
                   beginning of a variable.  */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_IN_TAG)
                {

                    /* get out of tag state machine.  */
                    flag &= (ULONG)~UX_PICTBRIDGE_TAG_FLAG_IN_TAG;
                    
                    /* Change state machine to variable.  */
                    flag |= (ULONG)UX_PICTBRIDGE_TAG_FLAG_IN_VARIABLE;

                }
                break;
                
            case    UX_PICTBRIDGE_TAG_CHAR_EQUAL          :
            
                /* If we are not within a bracket, this is a loose character ! */
                if ((flag & UX_PICTBRIDGE_TAG_FLAG_BEGIN) == 0)
                    break;

                /* Check to see if we are within a string.  */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_IN_STRING)
                {
                    /* Yes, store the character.  */
                    if (variable_string > variable_string_end)
                        return(UX_BUFFER_OVERFLOW);
                    *variable_string++ = *input_buffer;

                    break;
                }

                /* If we are in the flag state machine we have an error.  */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_IN_TAG)

                    /* Error, we should have been in the variable state machine.  */
                    return(UX_PICTBRIDGE_ERROR_PARAMETER_MISSING);

                /* If we are in the variable state machine, change the state to expecting string.  */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_IN_VARIABLE)
                {

                    /* Get out of the variable state machine.  */
                    flag &= (ULONG)~UX_PICTBRIDGE_TAG_FLAG_IN_VARIABLE;
                    
                    /* Change state machine to expecting string.  */
                    flag |= UX_PICTBRIDGE_TAG_FLAG_EXPECTING_STRING;

                }                   
                break;        

            case    UX_PICTBRIDGE_TAG_CHAR_QUOTE          :

                /* Are we in the string state machine ? */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_IN_STRING)

                    /* Get out of the in string state machine.  We are out of any state now.  */
                    flag &= (ULONG)~UX_PICTBRIDGE_TAG_FLAG_IN_STRING;
                
                else
                {

                    /* Are we expecting a string ? */
                    if (flag & UX_PICTBRIDGE_TAG_FLAG_EXPECTING_STRING)
                    {

                        /* Get out of the expecting string state machine.  */
                        flag &= (ULONG)~UX_PICTBRIDGE_TAG_FLAG_EXPECTING_STRING;
                    
                        /* Change state machine to in string.  */
                        flag |= UX_PICTBRIDGE_TAG_FLAG_IN_STRING;

                    }                   

                    else

                        /* Loose character ! */
                        return(UX_PICTBRIDGE_ERROR_PARAMETER_MISSING);
                    
                }

                break;                

            case    UX_PICTBRIDGE_TAG_CHAR_SLASH          :
                
                /* If we are in no particular state at the moment, we could have a begin slash or end slash.  */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_IN_STRING)
                {                                

                    /* We are in the variable string state machine.  Store the "/" as a normal character. */
                    if (variable_string > variable_string_end)
                        return(UX_BUFFER_OVERFLOW);
                    *variable_string++ = *input_buffer;
                    
                    break;
                }
                /* Check other states.  */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_IN_TAG)
                {
                
                    /* We are in a tag. 2 scenario here. We are either at the beginning or at the end.  */
                    if (char_count != 0) 
        
                        /* We are at the end of the tag, this is a self closing tag.  */
                        *tag_flag |= UX_PICTBRIDGE_TAG_FLAG_SELF_CLOSING;
    
                    else
                        
                        /* This is a normal closing tag.  */
                        *tag_flag |= UX_PICTBRIDGE_TAG_FLAG_CLOSING;
                
                    /* We are done here.  */
                    break;                   
                }

                else
                {
                                
                    /* If we are out of any state machine, it must be a self closing tag.  */
                    if (flag & (UX_PICTBRIDGE_TAG_FLAG_IN_VARIABLE | UX_PICTBRIDGE_TAG_FLAG_EXPECTING_STRING))

                        /* This is a syntax error, abort.  */
                        return(UX_PICTBRIDGE_ERROR_PARAMETER_MISSING);
                    
                    else
                    {
                        /* We are at the end of the tag, this is a self closing tag.  */
                        *tag_flag |= UX_PICTBRIDGE_TAG_FLAG_SELF_CLOSING;

                        /* We are done here.  */
                        break;    
                    }

                }
                
                
            case    UX_PICTBRIDGE_TAG_CHAR_QUESTION_MARK          :
                
                /* If we are in no particular state at the moment, we could have a begin ? or end ?.  */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_IN_STRING)
                                
                    /* We are within a quoted string, do not proceed.  */
                    break;

                else
                
                    /* This is a comment line, no need for a closing tag.  */
                    *tag_flag |= UX_PICTBRIDGE_TAG_FLAG_COMMENT;

                break;                

            case    UX_PICTBRIDGE_TAG_CHAR_END_BRACKET    :

                /* If we are in no particular state at the moment, we could have a begin ? or end ?.  */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_IN_STRING)
                {

                    /* Yes, store the bracket in the string.  */
                    if (variable_string > variable_string_end)
                        return(UX_BUFFER_OVERFLOW);
                    *variable_string++ = *input_buffer;

                    break;
                }
            
                /* Check if we are within a bracket.  */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_BEGIN)
                {

                    /* Skip the closing bracket.  */
                    input_buffer++;
    
                    /* Update length.  */
                    input_length--;
                    
                    /* We may have a xml parameter between the current tag and its closing counterpart. */
                    while(input_length)
                    {

                        /* Get a character between the tag brackets.  */
                        switch (*input_buffer)
                        {

                            case    UX_PICTBRIDGE_TAG_CHAR_CR             :
                            case    UX_PICTBRIDGE_TAG_CHAR_LF             :

                                /* Skip potential CR/LF.  */
                                break;
                            
                            
                            case    UX_PICTBRIDGE_TAG_CHAR_START_BRACKET    :
                            
                                /* We have found the beginning of the next tag.
                                   Set the output buffer position to the next "<".  */
                                *output_buffer = input_buffer;
                
                                /* Set the length remaining.  */
                                *output_length = input_length;

                                /* We are done here.  */
                                return(UX_SUCCESS);
                
                            default :
                            
                                /* Whatever we have now, we store into the XML parameter.  */
                                if (xml_parameter > xml_parameter_end)
                                    return(UX_BUFFER_OVERFLOW);
                                *xml_parameter++ = *input_buffer;
                                break;
                                
                        }

                        /* Next position.  */
                        input_buffer++;
    
                        /* Update length.  */
                        input_length--;
                        
                    }    

                    /* Set the output buffer position.  */
                    *output_buffer = input_buffer;
                
                    /* Set the length remaining.  */
                    *output_length = input_length;

                    /* We have reached the end of the xml object.  */
                    return(UX_SUCCESS);
                    
                }
                else
                    
                    /* We have a syntax error.  */
                    return(UX_PICTBRIDGE_ERROR_SCRIPT_SYNTAX_ERROR);
                
            default :

                /* Check if we are within a bracket.  */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_BEGIN)

                    /* Yes, increase the char count. This will tell us
                       if we are at the beginning of a tag or at the end.  */
                    char_count++;

                /* We have a regular character. Store it in the current state machine.  */
                if (flag & UX_PICTBRIDGE_TAG_FLAG_IN_TAG)
                {
                    /* We are in the tag state machine.  */
                    if (tag_name > tag_name_end)
                        return(UX_BUFFER_OVERFLOW);
                    *tag_name++ = *input_buffer;
                }

                if (flag & UX_PICTBRIDGE_TAG_FLAG_IN_VARIABLE)
                {
                    /* We are in the variable state machine.  */
                    if (variable_name > variable_name_end)
                        return(UX_BUFFER_OVERFLOW);
                    *variable_name++ = *input_buffer;
                }

                if (flag & UX_PICTBRIDGE_TAG_FLAG_IN_STRING)
                {
                    /* We are in the variable string state machine.  */
                    if (variable_string > variable_string_end)
                        return(UX_BUFFER_OVERFLOW);
                    *variable_string++ = *input_buffer;
                }
                
                break;

        }

        /* Next position.  */
        input_buffer++;
    
        /* Update length.  */
        input_length--;
        
    }       

    /* We get here when we reached an unexpected end of the XML object.  
       if we had a begin bracket, there is an unexpected end. If not, we
       have an empty line.  */
    if (flag & UX_PICTBRIDGE_TAG_FLAG_BEGIN)

        /* We have an error.  */
        return(UX_PICTBRIDGE_ERROR_PARAMETER_MISSING);    
        
    else

        /* Not really an error, but we need to inform the caller there is no tag to process.  */
        return(UX_PICTBRIDGE_ERROR_EMPTY_LINE);    
            
}

