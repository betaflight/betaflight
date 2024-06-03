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
/*    _ux_host_class_pima_object_handles_get              PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function gets a list if the current valid Storage IDS. There   */
/*    is one Storage ID for each valid logical store.                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    pima                                       Pointer to pima class    */
/*    pima_session                               Pointer to pima session  */
/*    object_handles_array                       Pointer to store handles */
/*    object_handles_length                      Array length in handles  */
/*    object_format_code                         Object Format Code       */
/*    object_handle_association                  Object Handle            */
/*                                               Association              */
/*                                                                        */
/*    The 2 last parameter are optional and should be set to 0 if not     */
/*    used.                                                               */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*  _ux_host_class_pima_command                 Pima command function     */
/*  _ux_utility_long_get                        Get 32 bit value          */
/*  _ux_utility_memory_allocate                 Allocate some memory      */
/*  _ux_utility_memory_free                     Free some memory          */
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
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved num objects check, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_pima_object_handles_get(UX_HOST_CLASS_PIMA *pima, UX_HOST_CLASS_PIMA_SESSION *pima_session,
                                    ULONG *object_handles_array, ULONG object_handles_length,
                                    ULONG storage_id, ULONG object_format_code, ULONG object_handle_association)
{

UX_HOST_CLASS_PIMA_COMMAND           command;
UCHAR                                *object_handles_array_raw;
ULONG                                object_handle_length_raw;
ULONG                                count_object_handles;
ULONG                                nb_object_handles;
UINT                                 status;

    /* Check if this session is valid or not.  */
    if (pima_session -> ux_host_class_pima_session_magic != UX_HOST_CLASS_PIMA_MAGIC_NUMBER)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* Check if this session is opened or not.  */
    if (pima_session -> ux_host_class_pima_session_state != UX_HOST_CLASS_PIMA_SESSION_STATE_OPENED)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* Check the number of handles and compare with the size of the array given by the user.  */
    if (pima_session -> ux_host_class_pima_session_nb_objects > object_handles_length)
        return(UX_MEMORY_INSUFFICIENT);

    /* Issue command to get the storage IDs.  3 parameters.  */
    command.ux_host_class_pima_command_nb_parameters =  3;

    /* Parameter 1 is the Storage ID.  */
    command.ux_host_class_pima_command_parameter_1 =  storage_id;

    /* Parameter 2 is optional. It is the Object Format Code.  */
    command.ux_host_class_pima_command_parameter_2 =  object_format_code;

    /* Parameter 3 is optional. It is the object handle association.  */
    command.ux_host_class_pima_command_parameter_3 =  object_handle_association;

    /* Other parameters unused.  */
    command.ux_host_class_pima_command_parameter_4 =  0;
    command.ux_host_class_pima_command_parameter_5 =  0;

    /* Then set the command to GET_STORAGE_IDS.  */
    command.ux_host_class_pima_command_operation_code =  UX_HOST_CLASS_PIMA_OC_GET_OBJECT_HANDLES;

    /* Calculate the length the raw array.  We multiply the number of handles by the size on the handle and
       add a ULONG for the number of handles stored at the beginning of the array.  */
    status = UX_SUCCESS;
    object_handle_length_raw = 0;
    UX_UTILITY_ADD_SAFE(pima_session -> ux_host_class_pima_session_nb_objects, 1, object_handle_length_raw, status);
    if (status != UX_SUCCESS)
        return(status);
    UX_UTILITY_MULC_SAFE(object_handle_length_raw, (ULONG)sizeof(ULONG), object_handle_length_raw, status);
    if (status != UX_SUCCESS)
        return(status);

    /* Allocate some DMA safe memory for receiving the handles  */
    object_handles_array_raw =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, object_handle_length_raw);
    if (object_handles_array_raw == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Issue the command.  */
    status = _ux_host_class_pima_command(pima, &command, UX_HOST_CLASS_PIMA_DATA_PHASE_IN , object_handles_array_raw,
                                        object_handle_length_raw, object_handle_length_raw);

    /* Check the result. If OK, the object handles array is returned properly.  */
    if (status == UX_SUCCESS)
    {

        /* Read the number of Object handles in the returned array.  */
        nb_object_handles =  _ux_utility_long_get(object_handles_array_raw);

        /* Save the number of object handles.  */
        pima_session ->  ux_host_class_pima_session_nb_objects =  nb_object_handles;

        /* Check if the user gave us enough memory.  */
        if (nb_object_handles > object_handles_length)

            /* No, not enough memory to store the array.  */
            return(UX_MEMORY_INSUFFICIENT);

        /* Unpack all object handles.  */
        for(count_object_handles = 0; count_object_handles < nb_object_handles; count_object_handles++)

            /* Unpack one object handle at a time  */
            *(object_handles_array + count_object_handles) = _ux_utility_long_get(object_handles_array_raw + sizeof(ULONG) +
                                                                                    (count_object_handles * sizeof(ULONG)));
    }

    /* Free the original raw array.  */
    _ux_utility_memory_free(object_handles_array_raw);

    /* Return completion status.  */
    return(status);
}
