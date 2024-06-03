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
/*    _ux_host_class_pima_storage_info_get                PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function gets the current storage information block.           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                       Pointer to pima class    */ 
/*    pima_session                               Pointer to pima session  */ 
/*    storage_id                                 The storage ID           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_pima_command           Pima command function         */
/*    _ux_utility_descriptor_parse          Parse descriptor              */
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_copy               Copy memory                   */
/*    _ux_utility_memory_free               Free memory                   */
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
UINT  _ux_host_class_pima_storage_info_get(UX_HOST_CLASS_PIMA *pima, 
                                        UX_HOST_CLASS_PIMA_SESSION *pima_session,
                                        ULONG storage_id, UX_HOST_CLASS_PIMA_STORAGE *storage)
{

UX_HOST_CLASS_PIMA_COMMAND           command;
UINT                                 status;
UCHAR                                *storage_buffer;
UCHAR                                *storage_pointer;
ULONG                                unicode_string_length;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PIMA_STORAGE_INFO_GET, pima, storage_id, storage, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Check if this session is valid or not.  */
    if (pima_session -> ux_host_class_pima_session_magic != UX_HOST_CLASS_PIMA_MAGIC_NUMBER)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* Check if this session is opened or not.  */
    if (pima_session -> ux_host_class_pima_session_state != UX_HOST_CLASS_PIMA_SESSION_STATE_OPENED)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* Issue command to get the storage IDs.  1 parameter.  */
    command.ux_host_class_pima_command_nb_parameters =  1;
    
    /* Parameter 1 is the Storage ID.  */
    command.ux_host_class_pima_command_parameter_1 =  storage_id;
    
    /* Other parameters unused.  */
    command.ux_host_class_pima_command_parameter_2 =  0;
    command.ux_host_class_pima_command_parameter_3 =  0;
    command.ux_host_class_pima_command_parameter_4 =  0;
    command.ux_host_class_pima_command_parameter_5 =  0;

    /* Then set the command to GET_STORAGE_INFO.  */
    command.ux_host_class_pima_command_operation_code =  UX_HOST_CLASS_PIMA_OC_GET_STORAGE_INFO;

    /* Allocate some DMA safe memory for receiving the storage info block.  */
    storage_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_PIMA_STORAGE_MAX_LENGTH);
    if (storage == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Issue the command.  */
    status = _ux_host_class_pima_command(pima, &command, UX_HOST_CLASS_PIMA_DATA_PHASE_IN , storage_buffer, 
                                        UX_HOST_CLASS_PIMA_STORAGE_MAX_LENGTH, UX_HOST_CLASS_PIMA_STORAGE_MAX_LENGTH);

    /* Check the result. If the result is OK, the storage info block was read properly. */
    if (status == UX_SUCCESS)
    {
        /* Uncompress the storage descriptor, at least the fixed part.  */
        _ux_utility_descriptor_parse(storage_buffer,
                            _ux_system_class_pima_object_structure,
                            UX_HOST_CLASS_PIMA_OBJECT_ENTRIES,
                            (UCHAR *) storage);

        /* Copy the storage description field.  Point to the beginning of the storage description string.  */
        storage_pointer =  storage_buffer + UX_HOST_CLASS_PIMA_STORAGE_VARIABLE_OFFSET;
        
        /* Get the unicode string length.  */
        unicode_string_length =  (ULONG) *storage_pointer ;

        /* Copy that string into the storage description field.  */
        _ux_utility_memory_copy(storage -> ux_host_class_pima_storage_description, storage_pointer, unicode_string_length); /* Use case of memcpy is verified. */

        /* Point to the volume label.  */
        storage_pointer =  storage_buffer + UX_HOST_CLASS_PIMA_STORAGE_VARIABLE_OFFSET + unicode_string_length;
        
        /* Get the unicode string length.  */
        unicode_string_length =  (ULONG) *storage_pointer ;

        /* Copy that string into the storage volume label field.  */
        _ux_utility_memory_copy(storage -> ux_host_class_pima_storage_volume_label, storage_pointer, unicode_string_length); /* Use case of memcpy is verified. */

    }

    /* Free the original storage info buffer.  */
    _ux_utility_memory_free(storage_buffer);
    
    /* Return completion status.  */
    return(status);
}

