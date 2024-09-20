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
/*    _ux_pictbridge_dpshost_input_object_notify_device_status            */
/*                                                        PORTABLE C      */ 
/*                                                           6.1          */
/*                                                                        */ 
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates the tag lines of the input object for the     */ 
/*    notification of the device status.                                  */ 
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
/*                                                                        */ 
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
UINT _ux_pictbridge_dpshost_input_object_notify_device_status(UX_PICTBRIDGE *pictbridge, 
                                                 UCHAR *pima_object_buffer, 
                                                 ULONG object_length, 
                                                 UCHAR **pima_object_buffer_updated, 
                                                 ULONG *object_length_updated)
{

UINT        status = UX_SUCCESS;


    /* Add the line <notifyDeviceStatus>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_notifydevicestatus, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <dpsPrintserviceStatus> xxxxxxxx </dpsPrintserviceStatus> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_dpsprintservicestatus, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_dpsprintservicestatus, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <jobEndReason> xxxxxxxx </jobEndReason> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_jobendreason, 
                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                            UX_NULL, 0, (VOID *)(ALIGN_TYPE) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_jobendreason, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <errorStatus> xxxxxxxx </errorStatus> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_errorstatus, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                                UX_NULL, 0, (VOID *)(ALIGN_TYPE) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_errorstatus, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <errorReason> xxxxxxxx </errorReason> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_errorreason, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                                UX_NULL, 0, (VOID *)(ALIGN_TYPE) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_errorreason, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <disconnectEnable> xxxxxxxx </disconnectEnable> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_disconnectenable, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                                UX_NULL, 0, (VOID *)(ALIGN_TYPE) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_disconnectenable, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <capabilityChanged> xxxxxxxx </capabilityChanged> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_capabilitychanged, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                                UX_NULL, 0, (VOID *)(ALIGN_TYPE) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_capabilitychanged, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <newJobOK> xxxxxxxx </newJobOK> */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_newjobok, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                                UX_NULL, 0, (VOID *)(ALIGN_TYPE) pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_newjobok, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line </notifyDeviceStatus>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_notifydevicestatus, 
                                                    UX_PICTBRIDGE_TAG_FLAG_END,
                                                    UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Update the caller's object position.  */
    *pima_object_buffer_updated = pima_object_buffer;
    
    /* Update the caller's object length .  */
    *object_length_updated = object_length;

    /* Return completion status.  */
    return(status);    
}

