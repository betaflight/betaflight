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
/*    _ux_pictbridge_object_tag_line_add                  PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function adds a tag line into the target xml object            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima_object_buffer                    Pointer to object buffer      */
/*    object_length                         Length of the object          */
/*    tag_element                           Tag to insert                 */
/*    tag_flag                              Flags                         */
/*    tag_variable                          Variable if any               */
/*    tag_variable_value                    Variable value if any         */
/*    tag_element                           element to insert after tag   */
/*    pima_object_buffer_updated            Updated Address of the object */
/*    object_length_updated                 Updated length                */
/*                                                                        */
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
/*    _ux_pictbridge_dpshost_object_get                                   */ 
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
UINT _ux_pictbridge_object_tag_line_add(UCHAR *pima_object_buffer, 
                                                 ULONG object_length, 
                                                 UCHAR *tag_element_string,
                                                 ULONG tag_flag,
                                                 UCHAR *tag_variable,
                                                 ULONG  tag_variable_value,
                                                 VOID  *tag_element,
                                                 UCHAR **pima_object_buffer_updated, 
                                                 ULONG *object_length_updated)
{

UINT                                element_length = 0;
ULONG                               value_index;
UINT                                status;

    /* Check if we need to insert a beginning tag.  */
    if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_BEGIN)
    {

        /* Going to add '<'.  */
        object_length += 1;

        /* Calculate the length of the element.  */
        status = _ux_utility_string_length_check(tag_element_string, &element_length, UX_PICTBRIDGE_MAX_ELEMENT_SIZE);
        if (status != UX_SUCCESS)

            /* Bad element.  */
            return(status);

        /* Going to add element.  */
        object_length += element_length;

        /* Do bounds-checking.  */
        if (object_length > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
            return(UX_MEMORY_INSUFFICIENT);

        /* Insert a "<".  */
        *pima_object_buffer++ = UX_PICTBRIDGE_TAG_CHAR_START_BRACKET;
    
        /* Create the element string.  */
       _ux_utility_memory_copy(pima_object_buffer, tag_element_string, element_length); /* Use case of memcpy is verified. */

        /* Update the object pointer position.  */
        pima_object_buffer += element_length;
    
        /* Do we have a variable to insert after the opening tag ?  */
        if (tag_variable != UX_NULL)
        {

            /* Going to add " ".  */
            object_length += 1;

            /* Insert the variable.  First calculate the length of the variable.  */
            status = _ux_utility_string_length_check(tag_variable, &element_length, UX_PICTBRIDGE_MAX_ELEMENT_SIZE);
            if (status != UX_SUCCESS)

                /* Bad element.  */
                return(status);

            /* Going to add element.  */
            object_length += element_length;

            /* Going to insert "=".  */
            object_length += 1;

            /* Going to insert a quote.  */
            object_length += 1;

            /* Going to insert 8 hex chars.  */
            object_length += 8;

            /* Going to insert a quote.  */
            object_length += 1;

            /* Do bounds-checking.  */
            if (object_length > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
                return(UX_MEMORY_INSUFFICIENT);

            /* Insert a " ".  */
            *pima_object_buffer++ = UX_PICTBRIDGE_TAG_CHAR_SPACE;
    
            /* Create the variable string.  */
            _ux_utility_memory_copy(pima_object_buffer, tag_variable, element_length); /* Use case of memcpy is verified. */

            /* Update the object pointer position.  */
            pima_object_buffer += element_length;
            
            /* Insert a "=".  */
            *pima_object_buffer++ = UX_PICTBRIDGE_TAG_CHAR_EQUAL;

            /* Insert a quote.  */
            *pima_object_buffer++ = UX_PICTBRIDGE_TAG_CHAR_QUOTE;

            /* Insert a value in between the quote.  This value is always of type hexa.  */
            status = _ux_pictbridge_hexa_to_element(tag_variable_value, pima_object_buffer);
            if (status != UX_SUCCESS)

                /* Bad element.  */
                return(status);
            
            /* Update the object pointer position.  */
            pima_object_buffer += 8;
            
            /* Insert a quote.  */
            *pima_object_buffer++ = UX_PICTBRIDGE_TAG_CHAR_QUOTE;
        }

        /* Check if we need to insert a slash at the end of the tag.   */
        if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END)
        {

            /* Going to insert a "/".  */
            object_length += 1;

            /* Do bounds-checking.  */
            if (object_length > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
                return(UX_MEMORY_INSUFFICIENT);

            /* Insert a "/".  */
            *pima_object_buffer++ = UX_PICTBRIDGE_TAG_CHAR_SLASH;
        }

        /* Going to insert a ">".  */
        object_length += 1;

        /* Do bounds-checking.  */
        if (object_length > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
            return(UX_MEMORY_INSUFFICIENT);

        /* Insert a ">".  */
        *pima_object_buffer++ = UX_PICTBRIDGE_TAG_CHAR_END_BRACKET;
    }

    /* Is there a tag element to add between the begin tag and end tag ?  */
    /* We do have something to insert, this element can have multiple format and
       can be a single value or an array of values.  */
    if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_VARIABLE)
    {
        
        /* We have a single variable.  Check its nature.  Is it hexa ? */
        if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA)
        {

            /*  Going to insert 8 hex chars. Do bounds-checking.  */
            if (object_length + 8 > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
                return(UX_MEMORY_INSUFFICIENT);

            /* We have a single hexa value to insert.  */
            status = _ux_pictbridge_hexa_to_element((ULONG)(ALIGN_TYPE) tag_element, pima_object_buffer);
            if (status != UX_SUCCESS)

                /* Bad element.  */
                return(status);
            element_length = 8;
        }
                        
        /* Is it decimal ?  */
        if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_VARIABLE_DECIMAL)
        {

            /* Going to insert at maximum 8 chars. Do bounds-checking.  */
            if (object_length + 8 > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
                return(UX_MEMORY_INSUFFICIENT);

            /* We have a single decimal value to insert.  */
            status = _ux_pictbridge_hexa_to_decimal_string((ULONG)(ALIGN_TYPE) tag_element, pima_object_buffer, UX_PICTBRIDGE_LEADING_ZERO_OFF, 8);
            if (status != UX_SUCCESS)

                /* Bad element.  */
                return(status);
            _ux_utility_string_length_check(pima_object_buffer, &element_length, 8);
        }

        /* Is it 3 decimal with leading zeroes ?  */
        if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_VARIABLE_DECIMAL_3DIGITS)
        {

            /* Going to insert at maximum 3 chars. Do bounds-checking.  */
            if (object_length + 3 > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
                return(UX_MEMORY_INSUFFICIENT);

            /* We have a single decimal value to insert but 3 digits only and leading zeroes.  */
            status = _ux_pictbridge_hexa_to_decimal_string((ULONG)(ALIGN_TYPE) tag_element, pima_object_buffer, UX_PICTBRIDGE_LEADING_ZERO_ON, 3);
            if (status != UX_SUCCESS)

                /* Bad element.  */
                return(status);
            element_length = 3;
        }

        /* Is it in the form 00.00 ?  */
        if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_VARIABLE_MAJOR_MINOR)
        {

            /* Going to insert at maximum 5 chars. Do bounds-checking.  */
            if (object_length + 5 > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
                return(UX_MEMORY_INSUFFICIENT);

            /* We have a major minor value to insert.  */
            status = _ux_pictbridge_hexa_to_major_minor((ULONG)(ALIGN_TYPE) tag_element, pima_object_buffer);
            if (status != UX_SUCCESS)

                /* Bad element.  */
                return(status);
            element_length = 5;
        }

        /* Is it a ascii string ?  */
        if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_VARIABLE_STRING)
        {

            /* Calculate the length of the element to be inserted.  */
            status = _ux_utility_string_length_check((UCHAR *) tag_element, &element_length, UX_PICTBRIDGE_MAX_ELEMENT_SIZE);
            if (status != UX_SUCCESS)

                /* Bad element.  */
                return(status);

            /* Going to insert element. Do bounds-checking.  */
            if (object_length + element_length > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
                return(UX_MEMORY_INSUFFICIENT);

            /* We have a string to insert.  */
            _ux_utility_memory_copy(pima_object_buffer, (UCHAR *) tag_element, element_length); /* Use case of memcpy is verified. */
        }

        /* Update the object pointer position.  */
        pima_object_buffer += element_length;
    
        object_length += element_length;
    }

    /* Is there an array to insert ? */
    if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY)
    {
    
        /* We have an array to insert.  First reset the value index.  */
        value_index =  0;

        /* Parse all qualities supported and insert their values.  */
        while( *((ULONG *)tag_element + value_index) != 0)
        {

            /* If this value is not the first, we must use a space separator.  */
            if (value_index != 0)
            {

                /* Going to insert a space.  */
                object_length += 1;

                /* Do bounds-checking.  */
                if (object_length > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
                    return(UX_MEMORY_INSUFFICIENT);
        
                /* Insert a space.  */
                *pima_object_buffer++ = UX_PICTBRIDGE_TAG_CHAR_SPACE;
            }

            /* We have a array variable.  Check its nature.  Is it hexa ? */
            if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA)
            {

                /*  Going to insert 8 hex chars. Do bounds-checking.  */
                if (object_length + 8 > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
                    return(UX_MEMORY_INSUFFICIENT);

                /* We have a single hexa value to insert.  */
                status = _ux_pictbridge_hexa_to_element( *((ULONG *) tag_element + value_index), pima_object_buffer);
                if (status != UX_SUCCESS)

                    /* Bad element.  */
                    return(status);
                element_length = 8;
            }
                        
            /* Is it decimal ?  */
            if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_DECIMAL)
            {

                /* Going to insert at maximum 8 chars. Do bounds-checking.  */
                if (object_length + 8 > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
                    return(UX_MEMORY_INSUFFICIENT);
    
                /* We have a array of decimal value to insert.  */
                status = _ux_pictbridge_hexa_to_decimal_string( *((ULONG *)tag_element + value_index), pima_object_buffer, UX_PICTBRIDGE_LEADING_ZERO_OFF, 8);
                if (status != UX_SUCCESS)

                    /* Bad element.  */
                    return(status);
                _ux_utility_string_length_check(pima_object_buffer, &element_length, 8);
            }
    
            /* Is it in the form 00.00 ?  */
            if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_MAJOR_MINOR)
            {

                /* Going to insert at maximum 5 chars. Do bounds-checking.  */
                if (object_length + 5 > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
                    return(UX_MEMORY_INSUFFICIENT);
    
                /* We have a array of major minor value to insert.  */
                status = _ux_pictbridge_hexa_to_major_minor( *((ULONG *) tag_element + value_index), pima_object_buffer);
                if (status != UX_SUCCESS)

                    /* Bad element.  */
                    return(status);
                element_length = 5;
            }

            /* Is it a ascii string ?  */
            if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_STRING)
            {

                /* Calculate the length of the element to be inserted.  */
                status = _ux_utility_string_length_check((UCHAR *) (*((ALIGN_TYPE *)tag_element)), &element_length, UX_PICTBRIDGE_MAX_ELEMENT_SIZE);
                if (status != UX_SUCCESS)

                    /* Bad element.  */
                    return(status);

                /* Going to insert element. Do bounds-checking.  */
                if (object_length + element_length > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
                    return(UX_MEMORY_INSUFFICIENT);

                /* We have a string to insert.  */
                _ux_utility_memory_copy(pima_object_buffer, (UCHAR *) (*((ALIGN_TYPE *)tag_element)), element_length); /* Use case of memcpy is verified. */
            }
    
            /* Update the object pointer position.  */
            pima_object_buffer += element_length;
    
            /* And update the cumulated length of the object.  */
            object_length += element_length;

            /* Next index value.  */
            value_index++;  
        }
    }

    /* Check if we need to insert an end tag.  */
    if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_END)
    {

        /* Going to insert a "<".  */
        object_length += 1;

        /* Going to insert a "/".  */
        object_length += 1;

        /* Calculate the length of the element.  */
        status = _ux_utility_string_length_check(tag_element_string, &element_length, UX_PICTBRIDGE_MAX_ELEMENT_SIZE);
        if (status != UX_SUCCESS)

            /* Bad element.  */
            return(status);

        /* Going to insert element.  */
        object_length += element_length;

        /* Going to insert a ">".  */
        object_length += 1;

        /* Going to insert a LF.  */
        object_length += 1;

        /* Do bounds-checking.  */
        if (object_length > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
            return(UX_MEMORY_INSUFFICIENT);

        /* Insert a "<".  */
        *pima_object_buffer++ = UX_PICTBRIDGE_TAG_CHAR_START_BRACKET;

        /* Insert a "/".  */
        *pima_object_buffer++ = UX_PICTBRIDGE_TAG_CHAR_SLASH;
    
        /* Create the element string.  */
       _ux_utility_memory_copy(pima_object_buffer, tag_element_string, element_length); /* Use case of memcpy is verified. */

        /* Update the object pointer position.  */
        pima_object_buffer += element_length;

        /* Insert a ">".  */
        *pima_object_buffer++ = UX_PICTBRIDGE_TAG_CHAR_END_BRACKET;

        /* Insert a LF.  */
        *pima_object_buffer++ = UX_PICTBRIDGE_TAG_CHAR_LF;
    }

    /* Check if we need to force a LF at end.  */
    if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_FORCE_LF)
    {

        /* Going to insert a LF.  */
        object_length += 1;

        /* Do bounds-checking.  */
        if (object_length > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
            return(UX_MEMORY_INSUFFICIENT);

        /* Insert a LF.  */
        *pima_object_buffer++ = UX_PICTBRIDGE_TAG_CHAR_LF;
    }

    /* Update the caller's object position.  */
    *pima_object_buffer_updated = pima_object_buffer;
    
    /* Update the caller's object length .  */
    *object_length_updated = object_length;
    
    /* Return completion status.  */
    return(UX_SUCCESS);    
}
