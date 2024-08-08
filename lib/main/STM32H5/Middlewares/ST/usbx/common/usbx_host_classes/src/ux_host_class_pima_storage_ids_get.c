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
/*    _ux_host_class_pima_storage_ids_get                 PORTABLE C      */
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
/*    storage_ids_array                          Pointer to buffer to     */
/*                                               fill storage IDs         */
/*    storage_id_length                          Array length in N of IDs */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_class_pima_command           Pima command function         */
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_free               Free memory                   */
/*    _ux_utility_long_get                  Get 32-bit value              */
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
/*                                            improved array size check,  */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_pima_storage_ids_get(UX_HOST_CLASS_PIMA *pima, UX_HOST_CLASS_PIMA_SESSION *pima_session,
                                    ULONG *storage_ids_array, ULONG storage_id_length)
{

UX_HOST_CLASS_PIMA_COMMAND           command;
UINT                                 status;
UCHAR                                *storage_ids;
ULONG                                count_storage_ids;
ULONG                                nb_storage_ids;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PIMA_STORAGE_IDS_GET, pima, storage_ids_array, storage_id_length, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Check if this session is valid or not.  */
    if (pima_session -> ux_host_class_pima_session_magic != UX_HOST_CLASS_PIMA_MAGIC_NUMBER)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* Check if this session is opened or not.  */
    if (pima_session -> ux_host_class_pima_session_state != UX_HOST_CLASS_PIMA_SESSION_STATE_OPENED)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* Issue command to get the storage IDs.  No parameter.  */
    command.ux_host_class_pima_command_nb_parameters =  0;

    /* Other parameters unused.  */
    command.ux_host_class_pima_command_parameter_1 =  0;
    command.ux_host_class_pima_command_parameter_2 =  0;
    command.ux_host_class_pima_command_parameter_3 =  0;
    command.ux_host_class_pima_command_parameter_4 =  0;
    command.ux_host_class_pima_command_parameter_5 =  0;

    /* Then set the command to GET_STORAGE_IDS.  */
    command.ux_host_class_pima_command_operation_code =  UX_HOST_CLASS_PIMA_OC_GET_STORAGE_IDS;

    /* Allocate some DMA safe memory for receiving the IDs.
     * UX_HOST_CLASS_PIMA_STORAGE_IDS_LENGTH = (UX_HOST_CLASS_PIMA_MAX_STORAGE_IDS + 1) * 4,
     * it is checked no calculation overflow.
     */
    storage_ids =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_PIMA_STORAGE_IDS_LENGTH);
    if (storage_ids == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Issue the command.  */
    status = _ux_host_class_pima_command(pima, &command, UX_HOST_CLASS_PIMA_DATA_PHASE_IN , storage_ids,
                                        UX_HOST_CLASS_PIMA_STORAGE_IDS_LENGTH, UX_HOST_CLASS_PIMA_STORAGE_IDS_LENGTH);

    /* Check the result. If OK, the storage ID array is returned properly.  */
    if (status == UX_SUCCESS)
    {

        /* Read the number of Storage IDs in the returned array.  */
        nb_storage_ids =  _ux_utility_long_get(storage_ids);

        /* Ensure we do read the array beyond the allocated memory. */
        if (nb_storage_ids > UX_HOST_CLASS_PIMA_MAX_STORAGE_IDS)

            /* If we get here we should probably increase the value for UX_HOST_CLASS_PIMA_STORAGE_IDS_LENGTH  */
            nb_storage_ids =  UX_HOST_CLASS_PIMA_MAX_STORAGE_IDS;

        /* Save the number of storage IDs.  */
        pima_session ->  ux_host_class_pima_session_nb_storage_ids =  nb_storage_ids;

        /* Check if the user gave us enough memory.  */
        if (nb_storage_ids > storage_id_length)

            /* No, not enough memory to store the array.  */
            return(UX_MEMORY_INSUFFICIENT);

        /* Unpack all storage IDS.  */
        for(count_storage_ids = 0; count_storage_ids < nb_storage_ids; count_storage_ids++)

            /* Unpack one ID at a time  */
            *(storage_ids_array + count_storage_ids) = _ux_utility_long_get(storage_ids + sizeof(ULONG) +
                                                                                                        (count_storage_ids * sizeof(ULONG)));
    }

    /* Free the original storage ID buffer.  */
    _ux_utility_memory_free(storage_ids);

    /* Return completion status.  */
    return(status);
}
