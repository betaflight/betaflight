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
/**   Device Pima Class                                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_pima.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_pima_storage_info_get              PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function returns the storage info structure to the host.       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                  Pointer to pima class         */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_request     Transfer request              */
/*    _ux_utility_long_put                  Put 32-bit value              */
/*    _ux_utility_short_put                 Put 32-bit value              */
/*    _ux_utility_string_to_unicode         Ascii string to unicode       */
/*    _ux_device_class_pima_response_send   Send PIMA response            */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Device Storage Class                                                */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved sanity checks,     */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added no-callback handling, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_storage_info_get(UX_SLAVE_CLASS_PIMA *pima, ULONG storage_id)
{

UINT                    status;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                   storage_info_length;
UCHAR                   *storage_info;
UCHAR                   *storage_info_pointer;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_STORAGE_INFO_SEND, pima, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &pima -> ux_device_class_pima_bulk_in_endpoint -> ux_slave_endpoint_transfer_request;

    /* Obtain memory for this object info. Use the transfer request pre-allocated memory.  */
    storage_info =  transfer_request -> ux_slave_transfer_request_data_pointer;
    
    /* Update the storage information. We get the volatile parameters from the application.  */
    if (pima -> ux_device_class_pima_storage_info_get)
        status = pima -> ux_device_class_pima_storage_info_get(pima, storage_id);
    else
    {
        if (storage_id == pima -> ux_device_class_pima_storage_id)
            status = UX_SUCCESS;
        else
            status = UX_DEVICE_CLASS_PIMA_RC_INVALID_STORAGE_ID;
    }

    /* Check for error.  */
    if (status != UX_SUCCESS)

        /* We return an error.  */
        _ux_device_class_pima_response_send(pima, status, 0, 0, 0, 0);
    
    else
    {    

        /* Fill in the data container type.  */
        _ux_utility_short_put(storage_info + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TYPE,
                                UX_DEVICE_CLASS_PIMA_CT_DATA_BLOCK);
        
        /* Fill in the data code.  */
        _ux_utility_short_put(storage_info + UX_DEVICE_CLASS_PIMA_DATA_HEADER_CODE,
                                UX_DEVICE_CLASS_PIMA_OC_GET_STORAGE_INFO);
        
        /* Fill in the Transaction ID.  */
        _ux_utility_long_put(storage_info + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TRANSACTION_ID, 
                                pima -> ux_device_class_pima_transaction_id);
            
        /* Allocate the device info pointer to the beginning of the dynamic storage info field.  */
        storage_info_pointer = storage_info + UX_DEVICE_CLASS_PIMA_STORAGE_FREE_STORAGE_DESCRIPTION;
    
        /* Fill in the storage type.  */
        _ux_utility_short_put(storage_info + UX_DEVICE_CLASS_PIMA_STORAGE_TYPE, 
                (USHORT)pima -> ux_device_class_pima_storage_type);
        
        /* Fill in the file system type.  */
        _ux_utility_short_put(storage_info + UX_DEVICE_CLASS_PIMA_STORAGE_FILE_SYSTEM_TYPE, 
                (USHORT)pima -> ux_device_class_pima_storage_file_system_type);
    
        /* Fill in the access capability.  */
        _ux_utility_short_put(storage_info + UX_DEVICE_CLASS_PIMA_STORAGE_ACCESS_CAPABILITY, 
                (USHORT)pima -> ux_device_class_pima_storage_access_capability);
    
        /* Fill in the low dword of max capacity.  */
        _ux_utility_long_put(storage_info + UX_DEVICE_CLASS_PIMA_STORAGE_MAX_CAPACITY_LOW, 
                                pima -> ux_device_class_pima_storage_max_capacity_low);
    
        /* Fill in the high dword of max capacity.  */
        _ux_utility_long_put(storage_info + UX_DEVICE_CLASS_PIMA_STORAGE_MAX_CAPACITY_HIGH, 
                                pima -> ux_device_class_pima_storage_max_capacity_high);
    
        /* Fill in the low dword of free space.  */
        _ux_utility_long_put(storage_info + UX_DEVICE_CLASS_PIMA_STORAGE_FREE_SPACE_LOW, 
                                pima -> ux_device_class_pima_storage_free_space_low);
    
        /* Fill in the high dword of free space.  */
        _ux_utility_long_put(storage_info + UX_DEVICE_CLASS_PIMA_STORAGE_FREE_SPACE_HIGH, 
                                pima -> ux_device_class_pima_storage_free_space_high);
    
        /* Fill in the free space in image.  */
        _ux_utility_long_put(storage_info + UX_DEVICE_CLASS_PIMA_STORAGE_FREE_SPACE_IMAGE, 
                                pima -> ux_device_class_pima_storage_free_space_image);

        /* Sanity check for buffer length.  */
        UX_ASSERT(UX_DEVICE_CLASS_PIMA_STORAGE_FREE_STORAGE_DESCRIPTION + 2 +
                _ux_utility_string_length_get(pima -> ux_device_class_pima_storage_description) * 2 +
                _ux_utility_string_length_get(pima -> ux_device_class_pima_storage_volume_label) * 2);

        /* Fill in the storage description string.  */
        _ux_utility_string_to_unicode(pima -> ux_device_class_pima_storage_description, storage_info_pointer); 
    
        /* Update the storage info pointer.  */
        storage_info_pointer += (ULONG) (*storage_info_pointer * 2) + 1;
    
        /* Fill in the volume label  string.  */
        _ux_utility_string_to_unicode(pima -> ux_device_class_pima_storage_volume_label, storage_info_pointer); 
    
        /* Update the storage info pointer.  */
        storage_info_pointer += (ULONG) (*storage_info_pointer* 2) + 1;
    
        /* Compute the overall length of the storage info structure.  */
        storage_info_length = (ULONG) ((ALIGN_TYPE) storage_info_pointer - (ALIGN_TYPE) storage_info);
        
        /* Fill in the size of the response header.  */
        _ux_utility_long_put(storage_info + UX_DEVICE_CLASS_PIMA_DATA_HEADER_LENGTH, 
                                storage_info_length);
        
        /* Send a data payload with the storage info data set.  */
        status =  _ux_device_stack_transfer_request(transfer_request, storage_info_length, 0);
        
        /* Now we return a response with success.  */
        _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_OK, 0, 0, 0, 0);
    }
    
    /* Return completion status.  */
    return(status);
}
