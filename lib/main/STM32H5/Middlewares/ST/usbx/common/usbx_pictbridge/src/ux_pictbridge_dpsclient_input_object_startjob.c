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
/*    _ux_pictbridge_dpsclient_input_start_job                            */ 
/*                                                        PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates the tag lines of the start job                */ 
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
UINT _ux_pictbridge_dpsclient_input_object_startjob(UX_PICTBRIDGE *pictbridge, 
                            ULONG input_subcode,
                            ULONG input_parameter,
                            UCHAR *pima_object_buffer, 
                            ULONG object_length, 
                            UCHAR **pima_object_buffer_updated, 
                            ULONG *object_length_updated)
{

UINT                        status;
UX_PICTBRIDGE_JOBINFO       *jobinfo;
UX_PICTBRIDGE_PRINTINFO     *printinfo;    

    UX_PARAMETER_NOT_USED(input_subcode);
    UX_PARAMETER_NOT_USED(input_parameter);

    /* We can start a new job. Fill in the JobConfig and PrintInfo structures. */
    jobinfo = &pictbridge -> ux_pictbridge_jobinfo;
    printinfo = jobinfo -> ux_pictbridge_jobinfo_printinfo_start;

    /* Add the line <startJob>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_startjob, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <jobConfig>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_jobconfig, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <quality> xxxxxxxx </quality> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_quality, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) jobinfo -> ux_pictbridge_jobinfo_quality, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <paperType> xxxxxxxx </paperType> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_papertype, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) jobinfo -> ux_pictbridge_jobinfo_papertype, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <paperSize> xxxxxxxx </paperSize> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_papersize, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) jobinfo -> ux_pictbridge_jobinfo_papersize, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <fileType> xxxxxxxx </fileType> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_filetype, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) jobinfo -> ux_pictbridge_jobinfo_filetype, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <datePrint> xxxxxxxx </datePrint> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_dateprint, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) jobinfo -> ux_pictbridge_jobinfo_dateprint, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <fileNamePrint> xxxxxxxx </fileNamePrint> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_filenameprint, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) jobinfo -> ux_pictbridge_jobinfo_filenameprint, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <imageOptimize> xxxxxxxx </imageOptimize> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_imageoptimize, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) jobinfo -> ux_pictbridge_jobinfo_imageoptimize, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <layout> xxxxxxxx </layout> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_layout, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) jobinfo -> ux_pictbridge_jobinfo_layout, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line </jobConfig>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_jobconfig, 
                                                UX_PICTBRIDGE_TAG_FLAG_END,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add all the printInfo.  */
    while (printinfo != UX_NULL)
    {
        
        /* Add the line <printInfo>  */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_printinfo, 
                                                    UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                    UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);

        /* Add the line <fileID> xxxxxxxx </fileID> */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_fileid, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                                UX_NULL, 0, (VOID *)(ALIGN_TYPE) printinfo -> ux_pictbridge_printinfo_fileid, 
                                                &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);

        /* Check if there is filename string.  */
        if (printinfo -> ux_pictbridge_printinfo_filename[0] != '\0')
        {

            /* Add the line <fileName> xxxxxxxx </fileName> */
            status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_filename, 
                                                    UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_STRING,
                                                    UX_NULL, 0, (VOID *) printinfo -> ux_pictbridge_printinfo_filename, 
                                                    &pima_object_buffer, &object_length);
            if (status != UX_SUCCESS)
                return(status);
        }

        /* Check if there is date string.  */
        if (printinfo -> ux_pictbridge_printinfo_date[0] != '\0')
        {

            /* Add the line <date> xxxxxxxx </date> */
            status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_date, 
                                                    UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_STRING,
                                                    UX_NULL, 0, (VOID *) printinfo -> ux_pictbridge_printinfo_date, 
                                                    &pima_object_buffer, &object_length);
            if (status != UX_SUCCESS)
                return(status);
        }

        /* Add the line </printInfo>  */
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_printinfo, 
                                                    UX_PICTBRIDGE_TAG_FLAG_END,
                                                    UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
        if (status != UX_SUCCESS)
            return(status);

        /* Go to next print info.  */
        printinfo = printinfo -> ux_pictbridge_printinfo_next;
    }

    /* Add the line </startJob>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_startjob, 
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

