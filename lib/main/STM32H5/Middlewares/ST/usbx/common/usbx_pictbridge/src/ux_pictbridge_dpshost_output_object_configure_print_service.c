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
/*    _ux_pictbridge_dpshost_output_object_configure_print_service        */
/*                                                        PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates the tag lines of the configure_print_service  */ 
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
/*    _ux_pictbridge_dpshost_object_get                                   */ 
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
UINT _ux_pictbridge_dpshost_output_object_configure_print_service(UX_PICTBRIDGE *pictbridge, 
                                                            UCHAR *pima_object_buffer, 
                                                            ULONG object_length, 
                                                            UCHAR **pima_object_buffer_updated, 
                                                            ULONG *object_length_updated)
{

UINT        status;


    /* Add the line <configurePrintService>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_configureprintservice, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <printServiceAvailable> xxxxxxxx </printServiceAvailable> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_printserviceavailable, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_print_service_available, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <dpsVersions>major_minor major_minor</dpsVersions> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_dpsversions, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_MAJOR_MINOR,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_dpsversions, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);


    /* Add the line <vendorName> ascii_string </vendorName> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_vendorname, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_STRING,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_product_name, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);


    /* Add the line <vendorSpecificVersion> major_minor </vendorSpecificVersion> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_vendorspecificversion, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_MAJOR_MINOR,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_vendor_specific_version, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    
    /* Add the line <productName> ascii_string </productName> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_productname, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_STRING,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_product_name, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);


    /* Add the line <serialNo> ascii_string </serialNo> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_serialno, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_STRING,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_serial_no, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);


    /* Add the line </configurePrintService>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_configureprintservice, 
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

