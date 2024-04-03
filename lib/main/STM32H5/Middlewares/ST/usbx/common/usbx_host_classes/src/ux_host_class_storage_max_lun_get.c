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
/**   Storage Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_storage.h"
#include "ux_host_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_storage_max_lun_get                  PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function retrieves the maximum number of LUNs from the device. */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    storage                               Pointer to storage class      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_stack_transfer_request       Process transfer request      */
/*    _ux_utility_memory_allocate           Allocate memory block         */
/*    _ux_utility_memory_free               Release memory block          */
/*    _ux_host_semaphore_get                Get semaphore                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Storage Class                                                       */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_max_lun_get(UX_HOST_CLASS_STORAGE *storage)
{

UX_ENDPOINT     *control_endpoint;
UX_TRANSFER     *transfer_request;
UINT            status;
UCHAR           *storage_data_buffer;


    /* In the case of non BO device or an error on the command the number of lun should be
       set to 0, indicating 1 lun.  */
    storage -> ux_host_class_storage_max_lun =  0;

#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    /* Check the device type. */
    if (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceProtocol == UX_HOST_CLASS_STORAGE_PROTOCOL_BO)
    {
#endif

    /* We need to get the default control endpoint transfer_request pointer.  */
    control_endpoint =  &storage -> ux_host_class_storage_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* We need to prevent other threads from simultaneously using the control endpoint. */
    status = _ux_host_semaphore_get(&control_endpoint -> ux_endpoint_device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);
    if (status != UX_SUCCESS)
        return(status);

    /* Need to allocate memory for the descriptor.  */
    storage_data_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 1);
    if (storage_data_buffer == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Create a transfer_request for the GET_MAX_LUN request.  */
    transfer_request -> ux_transfer_request_data_pointer =      storage_data_buffer;
    transfer_request -> ux_transfer_request_requested_length =  1;
    transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_STORAGE_GET_MAX_LUN;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
    transfer_request -> ux_transfer_request_value =             0;
    transfer_request -> ux_transfer_request_index =             storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceNumber;

#if defined(UX_HOST_STANDALONE)
    storage -> ux_host_class_storage_memory = storage_data_buffer;
    storage -> ux_host_class_storage_trans = transfer_request;
    storage -> ux_host_class_storage_state_state = UX_HOST_CLASS_STORAGE_STATE_TRANSFER;
    storage -> ux_host_class_storage_state_next = UX_HOST_CLASS_STORAGE_STATE_MAX_LUN_SAVE;
    UX_TRANSFER_STATE_RESET(transfer_request);
#else

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check for correct transfer and entire descriptor returned.  */
    if ((status == UX_SUCCESS) && (transfer_request -> ux_transfer_request_actual_length == 1))
    {

        /* Retrieve the number of max LUN for this device.  */
        storage -> ux_host_class_storage_max_lun =  *storage_data_buffer;

        /* Is the max LUN index greater than our LUN array's?  */
        if (storage -> ux_host_class_storage_max_lun > UX_MAX_HOST_LUN - 1)
        {

            /* Cap it off.  */
            storage -> ux_host_class_storage_max_lun =  UX_MAX_HOST_LUN - 1;

            /* Notify application so it knows to increase UX_MAX_HOST_LUN.  */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_MEMORY_ERROR);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_MEMORY_ERROR, storage, 0, 0, UX_TRACE_ERRORS, 0, 0)
        }
    }

    /* Free all used resources.  */
    _ux_utility_memory_free(storage_data_buffer);
#endif

#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    }
#if defined(UX_HOST_STANDALONE)
    else
    {
        storage -> ux_host_class_storage_trans = UX_NULL;
        storage -> ux_host_class_storage_state_state = UX_HOST_CLASS_STORAGE_STATE_MAX_LUN_SAVE;
    }
#endif
#endif

    /* We always succeed.  */
    return(UX_SUCCESS);
}

