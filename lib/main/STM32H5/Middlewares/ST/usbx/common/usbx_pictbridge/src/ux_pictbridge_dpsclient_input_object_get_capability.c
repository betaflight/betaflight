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
#include "ux_device_class_pima.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_dpsclient_input_get_capability                       */ 
/*                                                        PORTABLE C      */ 
/*                                                           6.1          */
/*                                                                        */ 
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates the tag lines of the capability               */ 
/*    request.                                                            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pictbridge                            Pictbridge instance           */ 
/*    pima_object_buffer                    Pointer to object buffer      */
/*    object_length                         Length of the object          */
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
/*    _ux_pictbridge_dpsclient_input_object_prepare                       */ 
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
UINT _ux_pictbridge_dpsclient_input_object_get_capability(UX_PICTBRIDGE *pictbridge, 
                                                            ULONG input_subcode,
                                                            ULONG input_parameter,
                                                            UCHAR *pima_object_buffer, 
                                                            ULONG object_length, 
                                                            UCHAR **pima_object_buffer_updated, 
                                                            ULONG *object_length_updated)
{

UINT                status;


    /* Add the line <getcapability>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_getcapability, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <capability>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_capability, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Check was subcode we need to for this request. The API might be sending one or multiple requests.  */
    if (input_subcode & UX_PICTBRIDGE_API_QUALITIES)
    {
        /* Add the line <qualities/> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_qualities, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Check was subcode we need to for this request. The API might be sending one or multiple requests.  */
    if (input_subcode & UX_PICTBRIDGE_API_PAPER_SIZES)
    {
        /* Add the line <paperSizes/> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_papersizes, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Check was subcode we need to for this request. The API might be sending one or multiple requests.  */
    if (input_subcode & UX_PICTBRIDGE_API_PAPER_SIZES)
    {

        /* Check if we had a paper size value specified in the input report.  */
        if (input_parameter != 0)
        {
            /* Add the line <paperTypes paperSize = "xxxxxxxx"/> */
            status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_papertypes, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                                _ux_pictbridge_xml_variable_papersize,
                                                input_parameter, 
                                                UX_NULL, &pima_object_buffer, &object_length);
            if (status != UX_SUCCESS)
                return(status);
        }
        else
        {
            /* Add the line <paperTypes/> */
            status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_papertypes, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END,
                                                UX_NULL, 0, 
                                                (VOID *) &pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_papertypes, &pima_object_buffer, &object_length);
            if (status != UX_SUCCESS)
                return(status);
        }
    }
            
    /* Parse the tag code that was memorized during the input.  */
    if (input_subcode & UX_PICTBRIDGE_API_FILE_TYPES)
    {
        /* Add the line <fileTypes/> */
        _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_filetypes, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END,
                                                UX_NULL, 0, UX_NULL , &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Parse the tag code that was memorized during the input.  */
    if (input_subcode & UX_PICTBRIDGE_API_DATE_PRINTS)
    {
        /* Add the line <datePrints/> */
        _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_dateprints, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END,
                                                UX_NULL, 0, UX_NULL , &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Parse the tag code that was memorized during the input.  */
    if (input_subcode & UX_PICTBRIDGE_API_FILE_NAME_PRINTS)
    {
        /* Add the line <namePrints/> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_filenameprints, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Parse the tag code that was memorized during the input.  */
    if (input_subcode & UX_PICTBRIDGE_API_IMAGE_OPTIMIZES)
    {
        /* Add the line <imageOptimizes/> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_imageoptimizes, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Parse the tag code that was memorized during the input.  */
    if (input_subcode & UX_PICTBRIDGE_API_LAYOUTS)
    {

        /* Check if we had a paper size value specified in the input report.  */
        if (input_parameter != 0)
        {
            /* Add the line <layouts paperSize = "xxxxxxxx"/> */
            status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_layouts, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                                _ux_pictbridge_xml_variable_papersize,
                                                input_parameter,  
                                                UX_NULL, &pima_object_buffer, &object_length);
            if (status != UX_SUCCESS)
                return(status);
        }
        else
        {
            /* Add the line <layouts/> */
            status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_layouts, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END,
                                                UX_NULL, 0, 
                                                UX_NULL, &pima_object_buffer, &object_length);
            if (status != UX_SUCCESS)
                return(status);
        }
    }

    /* Parse the tag code that was memorized during the input.  */
    if (input_subcode & UX_PICTBRIDGE_API_FIXED_SIZES)
    {
        /* Add the line <fixedSizes/> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_fixedsizes, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END,
                                                UX_NULL, 0, 
                                                UX_NULL, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Parse the tag code that was memorized during the input.  */
    if (input_subcode & UX_PICTBRIDGE_API_CROPPINGS)
    {
        /* Add the line <croppings/> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_croppings, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Parse the tag code that was memorized during the input.  */
    if (input_subcode & UX_PICTBRIDGE_API_CHAR_REPERTOIRES)
    {
        /* Add the line <charRepertoires/> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_charrepertoires, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END,
                                                UX_NULL, 0, 
                                                UX_NULL, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Add the line </capability>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_capability, 
                                                UX_PICTBRIDGE_TAG_FLAG_END,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line </getcapability>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_getcapability, 
                                                UX_PICTBRIDGE_TAG_FLAG_END,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Update the caller's object position.  */
    *pima_object_buffer_updated = pima_object_buffer;
    
    /* Update the caller's object length .  */
    *object_length_updated = object_length;
    
    /* Return completion status.  */
    return(UX_SUCCESS);    
}

