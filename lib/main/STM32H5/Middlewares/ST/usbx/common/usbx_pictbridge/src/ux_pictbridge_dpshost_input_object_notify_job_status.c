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
/*    _ux_pictbridge_dpshost_input_object_notify_job_status               */
/*                                                        PORTABLE C      */ 
/*                                                           6.1.11       */
/*                                                                        */ 
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates the tag lines of the input object for the     */ 
/*    notification of the job status.                                     */ 
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
/*                                        _device_status                  */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*                                                                        */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  04-25-2022     Yajun Xia                Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT _ux_pictbridge_dpshost_input_object_notify_job_status(UX_PICTBRIDGE *pictbridge, 
                                                 UCHAR *pima_object_buffer, 
                                                 ULONG object_length, 
                                                 UCHAR **pima_object_buffer_updated, 
                                                 ULONG *object_length_updated)
{
UX_PICTBRIDGE_JOBINFO               *jobinfo;
UX_PICTBRIDGE_PRINTINFO             *printinfo;
UINT                                 status = UX_SUCCESS;

    /* Get the jobinfo structure pointer.  */
    jobinfo = &pictbridge -> ux_pictbridge_jobinfo;

    /* Get the current printinfo structure.  */
    printinfo =  jobinfo -> ux_pictbridge_jobinfo_printinfo_current;

    /* Add the line <notifyJobStatus>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_notifyjobstatus, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <progress>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_progress, 
                                                    UX_PICTBRIDGE_TAG_FLAG_BEGIN,
                                                    UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Insert 001/001 from current page/total pages in the current printjob.  */

    /* Check length to go.  */
    object_length += 7;
    if (object_length > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
    {
        status = UX_MEMORY_INSUFFICIENT;
        return (status);
    }

    /* Insert xxx decimal.  */
    status = _ux_pictbridge_hexa_to_decimal_string(printinfo -> ux_pictbridge_printinfo_current_page, pima_object_buffer, UX_PICTBRIDGE_LEADING_ZERO_ON, 3);
    if (status != UX_SUCCESS)
        return(status);

    /* Update the address of the destination.  */
    pima_object_buffer += 3;

    /* Insert the '/' between both pages.  */
    *pima_object_buffer++ = '/';

    /* Insert the decimal value of total pages.  */
    status = _ux_pictbridge_hexa_to_decimal_string(printinfo -> ux_pictbridge_printinfo_total_page, pima_object_buffer, UX_PICTBRIDGE_LEADING_ZERO_ON, 3);
    if (status != UX_SUCCESS)
        return(status);

    /* Update the address.  */
    pima_object_buffer += 3;

    /* Add the line </progress>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_progress, 
                                                UX_PICTBRIDGE_TAG_FLAG_END,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <imagesPrinted> xxxxxxxx </imagesPrinted> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_imagesprinted, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_DECIMAL_3DIGITS,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) printinfo -> ux_pictbridge_printinfo_images_printed, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line </notifyJobStatus>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_notifyjobstatus, 
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

