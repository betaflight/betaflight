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
/*    _ux_pictbridge_dpsclient_input_object_prepare       PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function prepares a client input object to be sent.            */ 
/*    This function puts a interrupt notification for the host to issue   */ 
/*    a get object request.                                               */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pictbridge                             Pictbridge instance          */ 
/*    input_function                         input function               */ 
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
/*                                            verified memset and memcpy  */
/*                                            cases, used UX prefix to    */
/*                                            refer to TX symbols instead */
/*                                            of using them directly,     */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            cleared compile warning,    */
/*                                            used macros for RTOS calls, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_dpsclient_input_object_prepare(UX_PICTBRIDGE *pictbridge, 
                                                    ULONG input_function, 
                                                    ULONG input_subfunction,
                                                    ULONG input_parameter)
{
UINT                                status;
ULONG                               object_length;
UCHAR                               *pima_object_buffer;
UX_SLAVE_CLASS_PIMA                 *pima;
UX_SLAVE_CLASS_PIMA_OBJECT          *pima_object;
ULONG                               actual_flags;


    /* Check the status of pictbridge, if we are in a host request cycle, cannot add object now. */
    if (pictbridge -> ux_pictbridge_host_client_state_machine & UX_PICTBRIDGE_STATE_MACHINE_HOST_REQUEST)
    {
    
        /* Add to the state machine the client pending request. */
        pictbridge -> ux_pictbridge_host_client_state_machine |= UX_PICTBRIDGE_STATE_MACHINE_CLIENT_REQUEST_PENDING;
    
        /* We should wait for the state machine to allow our operation.  */
        status =  _ux_system_event_flags_get(&pictbridge -> ux_pictbridge_event_flags_group, UX_PICTBRIDGE_EVENT_FLAG_STATE_MACHINE_READY, 
                                        UX_AND_CLEAR, &actual_flags, UX_PICTBRIDGE_EVENT_TIMEOUT);

        /* Check status.  */
        if (status != UX_SUCCESS)
            return(UX_EVENT_ERROR);

        /* Status good means flag match, no need to check variable again, mark it unused.  */
        (void)actual_flags;
    }

    /* Change the state machine to client request.  */
    pictbridge -> ux_pictbridge_host_client_state_machine |= UX_PICTBRIDGE_STATE_MACHINE_CLIENT_REQUEST;

    /* Get the pima instance from the pictbridge container.  */
    pima = (UX_SLAVE_CLASS_PIMA *) pictbridge -> ux_pictbridge_pima;
    
    /* Get the address of the object container.  We choose the client container.  */
    pima_object =  pictbridge -> ux_pictbridge_object_client;

    /* Get the object buffer address.  */
    pima_object_buffer = pima_object -> ux_device_class_pima_object_buffer;
    
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

    /* Add the line <input>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_input, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Look into the tag code from the input object and proceed to individual functions.  */
    switch (input_function) 
    {
                

        case UX_PICTBRIDGE_OR_CONFIGURE_PRINT_SERVICE              :

            status = _ux_pictbridge_dpsclient_input_object_configure_print_service(pictbridge, pima_object_buffer, object_length, 
                                                                            &pima_object_buffer, &object_length);
            break;
            
        case UX_PICTBRIDGE_OR_GET_CAPABILITY                       :

            status = _ux_pictbridge_dpsclient_input_object_get_capability(pictbridge, input_subfunction, input_parameter, pima_object_buffer, object_length, 
                                                                            &pima_object_buffer, &object_length);
            break;

        case UX_PICTBRIDGE_OR_START_JOB                            :

            status = _ux_pictbridge_dpsclient_input_object_startjob(pictbridge, input_subfunction, input_parameter, pima_object_buffer, object_length, 
                                                                            &pima_object_buffer, &object_length);
            break;

        case UX_PICTBRIDGE_OR_ABORT_JOB                            :

            status = _ux_pictbridge_dpsclient_input_object_abortjob(pictbridge, input_subfunction, input_parameter, pima_object_buffer, object_length, 
                                                                            &pima_object_buffer, &object_length);
            break;

        case UX_PICTBRIDGE_OR_CONTINUE_JOB                            :

            status = _ux_pictbridge_dpsclient_input_object_continuejob(pictbridge, input_subfunction, input_parameter, pima_object_buffer, object_length, 
                                                                            &pima_object_buffer, &object_length);
            break;

        case UX_PICTBRIDGE_OR_GET_DEVICE_STATUS                    :

            /* Add the line <getDeviceStatus/>  */
            status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_getdevicestatus, 
                                                    UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END,
                                                    UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
            
            break;

        default                                                    :

            /* Function not yet supported.  We should never come here anyway. */    
            status = (UX_ERROR);    
    }    
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line </input>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_input, 
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

    /* Get the length of the object and store it into the object data set.  */
    pima_object -> ux_device_class_pima_object_length = object_length;         
    pima_object -> ux_device_class_pima_object_compressed_size = object_length;         
    
    /* Save the handle for this object.  */
    pima_object -> ux_device_class_pima_object_handle_id =  UX_PICTBRIDGE_OBJECT_HANDLE_CLIENT_REQUEST;         
    
    /* We need to change the file name to DREQUEST.DPS. Encode the file name from ascii format into Unicode.  */
    _ux_utility_string_to_unicode(_ux_pictbridge_drequest_name, pima_object -> ux_device_class_pima_object_filename);

    /* Send the notification to the host that an object has been added.  */
    status = _ux_device_class_pima_object_add(pima, UX_PICTBRIDGE_OBJECT_HANDLE_CLIENT_REQUEST);

    /* Return completion status.  */
    return(status);    
}

