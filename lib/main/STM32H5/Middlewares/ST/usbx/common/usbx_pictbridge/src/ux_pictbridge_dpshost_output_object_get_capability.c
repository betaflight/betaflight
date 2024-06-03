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
#include "ux_host_class_pima.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_dpshost_output_object_get_capability PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates the tag lines of the get_capability request   */ 
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
/*    _ux_pictbridge_dpshost_object_get                                   */ 
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
UINT _ux_pictbridge_dpshost_output_object_get_capability(UX_PICTBRIDGE *pictbridge, 
                                                 UCHAR *pima_object_buffer, 
                                                 ULONG object_length, 
                                                 UCHAR **pima_object_buffer_updated, 
                                                 ULONG *object_length_updated)
{

UINT                    status;

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

    /* Parse the tag code that was memorized during the input.  */
    if (pictbridge -> ux_pictbridge_input_tags & UX_PICTBRIDGE_IR_GC_QUALITIES)
    {
        /* Add the line <qualities> xxxxxxxx </qualities> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_qualities, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA,
                                                UX_NULL, 0, (VOID *) &pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_qualities, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Parse the tag code that was memorized during the input.  */
    if (pictbridge -> ux_pictbridge_input_tags & UX_PICTBRIDGE_IR_GC_PAPER_SIZES)
    {
        /* Add the line <paperSizes> xxxxxxxx </paperSizes> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_papersizes, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA,
                                                UX_NULL, 0, (VOID *) &pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_papersizes, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Parse the tag code that was memorized during the input.  */
    if (pictbridge -> ux_pictbridge_input_tags & UX_PICTBRIDGE_IR_GC_PAPER_TYPES)
    {

        /* Check if we had a paper size value specified in the input report.  */
        if (pictbridge -> ux_pictbridge_dpsclient.ux_pictbridge_devinfo_papertypes_papersize != 0)

            /* Add the line <paperTypes paperSize = "xxxxxxxx"> xxxxxxxx yyyyyyyy </paperTypes> */
            status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_papertypes, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA,
                                                _ux_pictbridge_xml_variable_papersize, pictbridge -> ux_pictbridge_dpsclient.ux_pictbridge_devinfo_papertypes_papersize, 
                                                (VOID *) &pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_papertypes, &pima_object_buffer, &object_length);
        
        else
            
            /* Add the line <paperTypes> xxxxxxxx yyyyyyyy </paperTypes> */
            status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_papertypes, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END |  UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA,
                                                UX_NULL, 0, 
                                                (VOID *) &pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_papertypes, &pima_object_buffer, &object_length);

        if (status != UX_SUCCESS)
            return(status);
    }
            
    /* Parse the tag code that was memorized during the input.  */
    if (pictbridge -> ux_pictbridge_input_tags & UX_PICTBRIDGE_IR_GC_FILE_TYPES)
    {
        /* Add the line <fileTypes> xxxxxxxx </fileTypes> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_filetypes, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA,
                                                UX_NULL, 0, (VOID *) &pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_filetypes, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Parse the tag code that was memorized during the input.  */
    if (pictbridge -> ux_pictbridge_input_tags & UX_PICTBRIDGE_IR_GC_DATE_PRINTS)
    {
        /* Add the line <datePrints> xxxxxxxx </datePrints> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_dateprints, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA,
                                                UX_NULL, 0, (VOID *) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_dateprints, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }


    /* Parse the tag code that was memorized during the input.  */
    if (pictbridge -> ux_pictbridge_input_tags & UX_PICTBRIDGE_IR_GC_FILE_NAME_PRINTS)
    {
        /* Add the line <namePrints> xxxxxxxx </namePrints> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_filenameprints, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA,
                                                UX_NULL, 0, (VOID *) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_filenameprints, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Parse the tag code that was memorized during the input.  */
    if (pictbridge -> ux_pictbridge_input_tags & UX_PICTBRIDGE_IR_GC_IMAGE_OPTIMIZES)
    {
        /* Add the line <imageOptimizes> xxxxxxxx </imageOptimizes> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_imageoptimizes, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA,
                                                UX_NULL, 0, (VOID *) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_imageoptimizes, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Parse the tag code that was memorized during the input.  */
    if (pictbridge -> ux_pictbridge_input_tags & UX_PICTBRIDGE_IR_GC_LAYOUTS)
    {

        /* Check if we had a paper size value specified in the input report.  */
        if (pictbridge -> ux_pictbridge_dpsclient.ux_pictbridge_devinfo_layouts_papersize != 0)


            /* Add the line <layouts paperSize = "xxxxxxxx"> xxxxxxxx yyyyyyyy </layouts> */
            status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_layouts, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END |  UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA,
                                                _ux_pictbridge_xml_variable_papersize, pictbridge -> ux_pictbridge_dpsclient.ux_pictbridge_devinfo_layouts_papersize, 
                                                (VOID *) &pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_layouts, &pima_object_buffer, &object_length);
        
        else
            
            /* Add the line <layouts> xxxxxxxx yyyyyyyy </layouts> */
            status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_layouts, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END |  UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA,
                                                UX_NULL, 0, 
                                                (VOID *) &pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_layouts, &pima_object_buffer, &object_length);

        if (status != UX_SUCCESS)
            return(status);
    }

    /* Parse the tag code that was memorized during the input.  */
    if (pictbridge -> ux_pictbridge_input_tags & UX_PICTBRIDGE_IR_GC_FIXED_SIZES)
    {
        /* Add the line <fixedSizes> xxxxxxxx yyyyyyyy </fixedSizes> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_fixedsizes, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END |  UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA,
                                            UX_NULL, 0, 
                                            (VOID *) &pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_fixedsizes, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Parse the tag code that was memorized during the input.  */
    if (pictbridge -> ux_pictbridge_input_tags & UX_PICTBRIDGE_IR_GC_CROPPINGS)
    {
        /* Add the line <croppings> xxxxxxxx yyyyyyyy</croppings> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_croppings, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA,
                                                UX_NULL, 0, (VOID *) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_croppings, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);
    }

    /* Parse the tag code that was memorized during the input.  */
    if (pictbridge -> ux_pictbridge_input_tags & UX_PICTBRIDGE_IR_GC_CHAR_REPERTOIRES)
    {
        /* Add the line <charRepertoires> xxxxxxxx yyyyyyyy </charRepertoires> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_charrepertoires, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END |  UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA,
                                            UX_NULL, 0, 
                                            (VOID *) &pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_charrepertoires, &pima_object_buffer, &object_length);
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

