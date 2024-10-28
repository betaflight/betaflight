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
/*    _ux_pictbridge_dpshost_output_object_create         PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates an output report based on the parsing of      */ 
/*    a previous input object.                                            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pictbridge                             Pictbridge instance          */ 
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
UINT  _ux_pictbridge_dpshost_output_object_create(UX_PICTBRIDGE *pictbridge)
{
UINT                                status;
ULONG                               object_length;
UCHAR                               *pima_object_buffer;
UX_HOST_CLASS_PIMA_OBJECT           *pima_object;

    /* Get the address of the object container.  */
    pima_object =  (UX_HOST_CLASS_PIMA_OBJECT *) pictbridge -> ux_pictbridge_object_host;

    /* And its buffer address.  */
    pima_object_buffer = pima_object -> ux_host_class_pima_object_buffer;
    
    /* Reset the object length.  */
    object_length =  0;
    
    /* Clear the object memory buffer.  */
   _ux_utility_memory_set(pima_object_buffer, 0, UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER); /* Use case of memset is verified. */


    /* Add the line <?xml version="1.0"?>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_xmlversion, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <dps xmlns="http://www.cipa.jp/dps/schema/">  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_dpsxmlns, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <output>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_output, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <result> xxxxxxxx </result> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_result, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) pictbridge -> ux_pictbridge_operation_result, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Look into the tag code from the input object and proceed to individual functions.  */
    switch (pictbridge -> ux_pictbridge_input_request) 
    {
                
        case UX_PICTBRIDGE_IR_CONFIGURE_PRINT_SERVICE           :

            /* Insert the ConfigurePrintService tag lines.  */
            status = _ux_pictbridge_dpshost_output_object_configure_print_service(pictbridge, pima_object_buffer, object_length, &pima_object_buffer, &object_length);
            break;
        
        case UX_PICTBRIDGE_IR_GET_CAPABILITY                    :

            /* Insert the Getcapability tag lines.  */
            status = _ux_pictbridge_dpshost_output_object_get_capability(pictbridge, pima_object_buffer, object_length, &pima_object_buffer, &object_length);
            break;
        case UX_PICTBRIDGE_IR_GET_JOB_STATUS                    :
        case UX_PICTBRIDGE_IR_GET_DEVICE_STATUS                 :

            /* Insert the getDeviceStatus tag lines.  */
            status = _ux_pictbridge_dpshost_output_object_get_device_status(pictbridge, pima_object_buffer, object_length, &pima_object_buffer, &object_length);
            break;
        case UX_PICTBRIDGE_IR_START_JOB                         :

            /* The Start Job adds the line </startJob> after the result. Weird syntax ! */
            status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_startjob, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);


            status =  UX_SUCCESS;
            break;
            
        case UX_PICTBRIDGE_IR_ABORT_JOB                         :
        case UX_PICTBRIDGE_IR_CONTINUE_JOB                      :
        case UX_PICTBRIDGE_IR_NOTIFY_JOB_STATUS                 :
        case UX_PICTBRIDGE_IR_NOTIFY_DEVICE_STATUS              :

        default                                                 :
            /* Function not yet supported.  */    
        
            /* We have a syntax error !  */
            status = (UX_ERROR);    
    }    
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line </output>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_output, 
                                                UX_PICTBRIDGE_TAG_FLAG_END,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line </dps>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_dps, 
                                                UX_PICTBRIDGE_TAG_FLAG_END,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Store the length of the new object waiting to be sent out.  
       We do not store the length into the object itself since this function is
       host\client agnostic.  */
    pima_object -> ux_host_class_pima_object_length = object_length;
    
    /* Return completion status.  */
    return(status);    
}

