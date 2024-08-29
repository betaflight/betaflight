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
/**   PIMA Class                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_pima.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_pima_object_info_send                PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function sends an object information block prior to sending    */ 
/*    a new object.                                                       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                       Pointer to pima class    */ 
/*    pima_session                               Pointer to pima session  */ 
/*    storage_id                                 StorageID where to store */ 
/*    parent_object_id                           Parent object id         */ 
/*    object                                     Object info structure    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_pima_command               Pima command function     */
/*    _ux_utility_memory_allocate               Allocate memory           */ 
/*    _ux_utility_memory_copy                   Copy memory               */
/*    _ux_utility_memory_free                   Free allocated memory     */ 
/*    _ux_utility_descriptor_pack               Pack descriptor           */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USB application                                                     */ 
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
UINT  _ux_host_class_pima_object_info_send(UX_HOST_CLASS_PIMA *pima, 
                                        UX_HOST_CLASS_PIMA_SESSION *pima_session,
                                        ULONG storage_id, 
                                        ULONG parent_object_id,
                                        UX_HOST_CLASS_PIMA_OBJECT *object)
{

UX_HOST_CLASS_PIMA_COMMAND           command;
UCHAR                                *object_buffer;
UCHAR                                *object_buffer_end;
UCHAR                                *object_pointer;
ULONG                                unicode_string_length;
ULONG                                object_info_length;
UINT                                 status;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PIMA_OBJECT_INFO_SEND, pima, object, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Check if this session is valid or not.  */
    if (pima_session -> ux_host_class_pima_session_magic != UX_HOST_CLASS_PIMA_MAGIC_NUMBER)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* Check if this session is opened or not.  */
    if (pima_session -> ux_host_class_pima_session_state != UX_HOST_CLASS_PIMA_SESSION_STATE_OPENED)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* Issue command to set the object info.  2 parameter.  */
    command.ux_host_class_pima_command_nb_parameters =  2;
    
    /* Parameter 1 is the Storage ID.  */
    command.ux_host_class_pima_command_parameter_1 =  storage_id;
    
    /* Parameter 2 is the Parent Object ID.  */
    command.ux_host_class_pima_command_parameter_2 =  parent_object_id;
    
    /* Other parameters unused.  */
    command.ux_host_class_pima_command_parameter_3 =  0;
    command.ux_host_class_pima_command_parameter_4 =  0;
    command.ux_host_class_pima_command_parameter_5 =  0;

    /* Then set the command to SEND_OBJECT_INFO.  */
    command.ux_host_class_pima_command_operation_code =  UX_HOST_CLASS_PIMA_OC_SEND_OBJECT_INFO;

    /* Allocate some DMA safe memory for sending the object info block.  */
    object_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_PIMA_OBJECT_MAX_LENGTH);
    if (object == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Get the end of the object buffer.  */
    object_buffer_end =  object_buffer + UX_HOST_CLASS_PIMA_OBJECT_MAX_LENGTH;

    /* The object info structure coming from the application needs to be packed. */
    _ux_utility_descriptor_pack((UCHAR *) object, 
                            _ux_system_class_pima_object_structure,
                            UX_HOST_CLASS_PIMA_OBJECT_ENTRIES,
                            object_buffer);

    /* Copy the object filename  field.  Point to the beginning of the object description string.  */
    object_pointer =  object_buffer + UX_HOST_CLASS_PIMA_OBJECT_VARIABLE_OFFSET;
    
    /* Get the unicode string length for the filename.  */
    unicode_string_length =  ((ULONG) *object -> ux_host_class_pima_object_filename * 2) + 1;

    /* Is there enough room for this string?  */
    if (object_pointer + unicode_string_length > object_buffer_end)

        /* Return error.  */
        status =  UX_MEMORY_INSUFFICIENT;

    else

        /* Continue.  */
        status =  UX_SUCCESS;
    
    /* Is there enough space?  */
    if (status == UX_SUCCESS)
    {

        /* Copy that string into the object description field.  */
        _ux_utility_memory_copy(object_pointer, object -> ux_host_class_pima_object_filename, unicode_string_length); /* Use case of memcpy is verified. */

        /* Point to the next field.  */
        object_pointer += unicode_string_length;
        
        /* Get the unicode string length of the capture date.  */
        unicode_string_length =  ((ULONG) *object -> ux_host_class_pima_object_capture_date * 2) + 1;

        /* Is there enough room for this string?  */
        if (object_pointer + unicode_string_length > object_buffer_end)

            /* Return error.  */
            status =  UX_MEMORY_INSUFFICIENT;
    }

    /* Is there enough space?  */
    if (status == UX_SUCCESS)
    {

        /* Copy that string into the capture date field.  */
        _ux_utility_memory_copy(object_pointer, object -> ux_host_class_pima_object_capture_date, unicode_string_length); /* Use case of memcpy is verified. */

        /* Point to the next field.  */
        object_pointer += unicode_string_length;
        
        /* Get the unicode string length.  */
        unicode_string_length =  ((ULONG) *object -> ux_host_class_pima_object_modification_date * 2) + 1;

        /* Is there enough room for this string?  */
        if (object_pointer + unicode_string_length > object_buffer_end)

            /* Return error.  */
            status =  UX_MEMORY_INSUFFICIENT;
    }

    /* Is there enough space?  */
    if (status == UX_SUCCESS)
    {

        /* Copy that string into the modification date field.  */
        _ux_utility_memory_copy(object_pointer, object -> ux_host_class_pima_object_modification_date, unicode_string_length); /* Use case of memcpy is verified. */

        /* Point to the next field.  */
        object_pointer += unicode_string_length;
        
        /* Get the unicode string length.  */
        unicode_string_length =  ((ULONG) *object -> ux_host_class_pima_object_keywords * 2) + 1;

        /* Is there enough room for this string?  */
        if (object_pointer + unicode_string_length > object_buffer_end)

            /* Return error.  */
            status =  UX_MEMORY_INSUFFICIENT;
    }

    /* Is there enough space?  */
    if (status == UX_SUCCESS)
    {

        /* Copy that string into the keywords field.  */
        _ux_utility_memory_copy(object_pointer, object -> ux_host_class_pima_object_keywords, unicode_string_length); /* Use case of memcpy is verified. */

        /* Point to the next field.  */
        object_pointer += unicode_string_length;
        
        /* Calculate the length of the payload. */
        object_info_length = (ULONG ) (object_pointer - object_buffer);

        /* Issue the command.  */
        status = _ux_host_class_pima_command(pima, &command, UX_HOST_CLASS_PIMA_DATA_PHASE_OUT , object_buffer, 
                                            object_info_length, object_info_length);

        /* If the status is OK, the device sent us the Object handle in the response.  */
        if (status == UX_SUCCESS)
        
            /* Update the object handle.  */
            object -> ux_host_class_pima_object_handle_id = command.ux_host_class_pima_command_parameter_2;
    }
    else
    {

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_MEMORY_INSUFFICIENT, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Report error to application.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);
    }

    /* Free the original object info buffer.  */                       
    _ux_utility_memory_free(object_buffer);
   
    /* Return completion status.  */
    return(status);
}

