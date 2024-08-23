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
/*    _ux_host_class_pima_object_info_get                 PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function gets the current object information block.            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                       Pointer to pima class    */ 
/*    pima_session                               Pointer to pima session  */ 
/*    object_handle                              The object handle        */ 
/*    object                                     Object structure to fill */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_pima_command               Pima command function     */
/*    _ux_utility_descriptor_parse              Unpack descriptor         */ 
/*    _ux_utility_memory_allocate               Allocate memory           */ 
/*    _ux_utility_memory_copy                   Copy memory               */
/*    _ux_utility_memory_free                   Free allocated memory     */ 
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
UINT  _ux_host_class_pima_object_info_get(UX_HOST_CLASS_PIMA *pima, 
                                        UX_HOST_CLASS_PIMA_SESSION *pima_session,
                                        ULONG object_handle, UX_HOST_CLASS_PIMA_OBJECT *object)
{

UX_HOST_CLASS_PIMA_COMMAND           command;
UCHAR                                *object_buffer;
UCHAR                                *object_pointer;
ULONG                                unicode_string_length;
UINT                                 status;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PIMA_OBJECT_INFO_GET, pima, object_handle, object, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Check if this session is valid or not.  */
    if (pima_session -> ux_host_class_pima_session_magic != UX_HOST_CLASS_PIMA_MAGIC_NUMBER)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* Check if this session is opened or not.  */
    if (pima_session -> ux_host_class_pima_session_state != UX_HOST_CLASS_PIMA_SESSION_STATE_OPENED)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* Issue command to get the object info.  1 parameter.  */
    command.ux_host_class_pima_command_nb_parameters =  1;
    
    /* Parameter 1 is the Object Handle.  */
    command.ux_host_class_pima_command_parameter_1 =  object_handle;
    
    /* Other parameters unused.  */
    command.ux_host_class_pima_command_parameter_2 =  0;
    command.ux_host_class_pima_command_parameter_3 =  0;
    command.ux_host_class_pima_command_parameter_4 =  0;
    command.ux_host_class_pima_command_parameter_5 =  0;

    /* Then set the command to GET_OBJECT_INFO.  */
    command.ux_host_class_pima_command_operation_code =  UX_HOST_CLASS_PIMA_OC_GET_OBJECT_INFO;

    /* Allocate some DMA safe memory for receiving the object info block.  */
    object_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_PIMA_OBJECT_MAX_LENGTH);
    if (object == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Issue the command.  */
    status = _ux_host_class_pima_command(pima, &command, UX_HOST_CLASS_PIMA_DATA_PHASE_IN , object_buffer, 
                                        UX_HOST_CLASS_PIMA_OBJECT_MAX_LENGTH, UX_HOST_CLASS_PIMA_OBJECT_MAX_LENGTH);

    /* Check the result. If the result is OK, the object info block was read properly. */
    if (status == UX_SUCCESS)
    {
        /* Uncompress the object descriptor, at least the fixed part.  */
        _ux_utility_descriptor_parse(object_buffer,
                            _ux_system_class_pima_object_structure,
                            UX_HOST_CLASS_PIMA_OBJECT_ENTRIES,
                            (UCHAR *) object);

        /* Copy the object filename  field.  Point to the beginning of the object description string.  */
        object_pointer =  object_buffer + UX_HOST_CLASS_PIMA_OBJECT_VARIABLE_OFFSET;
        
        /* Get the unicode string length.  */
        unicode_string_length =  ((ULONG) *object_pointer * 2) + 1;

        /* Check if string can fit in our buffer.  */
        if (unicode_string_length > UX_HOST_CLASS_PIMA_UNICODE_MAX_LENGTH)

            /* Return error.  */
            status =  UX_MEMORY_INSUFFICIENT;
        
        /* Is there enough space?  */
        if (status == UX_SUCCESS)
        {

            /* Copy that string into the object description field.  */
            _ux_utility_memory_copy(object -> ux_host_class_pima_object_filename, object_pointer, unicode_string_length); /* Use case of memcpy is verified. */

            /* Point to the next field.  */
            object_pointer += unicode_string_length;
            
            /* Get the unicode string length.  */
            unicode_string_length =  ((ULONG) *object_pointer  * 2) + 1;

            /* Ensure the string can fit in our buffer.  */
            if (unicode_string_length > UX_HOST_CLASS_PIMA_DATE_TIME_STRING_MAX_LENGTH)

                /* Return error.  */
                status =  UX_MEMORY_INSUFFICIENT;
        }

        /* Is there enough space?  */
        if (status == UX_SUCCESS)
        {

            /* Copy that string into the capture date field.  */
            _ux_utility_memory_copy(object -> ux_host_class_pima_object_capture_date, object_pointer, unicode_string_length); /* Use case of memcpy is verified. */

            /* Point to the next field.  */
            object_pointer += unicode_string_length;
            
            /* Get the unicode string length.  */
            unicode_string_length =  ((ULONG) *object_pointer  * 2) + 1;

            /* Ensure the string can fit in our buffer.  */
            if (unicode_string_length > UX_HOST_CLASS_PIMA_DATE_TIME_STRING_MAX_LENGTH)

                /* Return error.  */
                status =  UX_MEMORY_INSUFFICIENT;
        }

        /* Is there enough space?  */
        if (status == UX_SUCCESS)
        {

            /* Copy that string into the modification date field.  */
            _ux_utility_memory_copy(object -> ux_host_class_pima_object_modification_date, object_pointer, unicode_string_length); /* Use case of memcpy is verified. */

            /* Point to the next field.  */
            object_pointer += unicode_string_length;
            
            /* Get the unicode string length.  */
            unicode_string_length =  ((ULONG) *object_pointer  * 2) + 1;

            /* Ensure the string can fit in our buffer.  */
            if (unicode_string_length > UX_HOST_CLASS_PIMA_UNICODE_MAX_LENGTH)

                /* Return error.  */
                status =  UX_MEMORY_INSUFFICIENT;
        }

        /* Is there enough space?  */
        if (status == UX_SUCCESS)
        {

            /* Copy that string into the keywords field.  */
            _ux_utility_memory_copy(object -> ux_host_class_pima_object_keywords, object_pointer, unicode_string_length); /* Use case of memcpy is verified. */

            /* Make the object closed.  */
            object -> ux_host_class_pima_object_state = UX_HOST_CLASS_PIMA_OBJECT_STATE_CLOSED;        

            /* Reset the reading\writing offset  */
            object -> ux_host_class_pima_object_offset = 0;
        }
        else
        {

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_MEMORY_INSUFFICIENT, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

            /* Report error to application.  */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);
        }
    }

    /* Free the original object info buffer.  */                       
    _ux_utility_memory_free(object_buffer);
   
    /* Return completion status.  */
    return(status);
}

