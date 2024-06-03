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
/*    _ux_host_class_storage_media_capacity_get           PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will send a READ_CAPACITY command to the storage      */
/*    device.                                                             */
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
/*    _ux_host_class_storage_cbw_initialize Initialize CBW                */
/*    _ux_host_class_storage_transport      Send command                  */
/*    _ux_host_class_storage_media_format_capacity_get                    */
/*                                          Get format capacity           */
/*    _ux_utility_memory_allocate           Allocate memory block         */
/*    _ux_utility_memory_free               Release memory block          */
/*    _ux_utility_long_get_big_endian       Get 32-bit big endian         */
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
/*                                            added option to disable FX  */
/*                                            media integration,          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_media_capacity_get(UX_HOST_CLASS_STORAGE *storage)
{

UINT            status;
UCHAR           *cbw;
UCHAR           *capacity_response;
UINT            command_length;
#if !defined(UX_HOST_STANDALONE)
ULONG           command_retry;
#endif

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_STORAGE_MEDIA_CAPACITY_GET, storage, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

#if !defined(UX_HOST_STANDALONE)

    /* Set the default to either 512 or 2048 bytes.  */
    switch (storage -> ux_host_class_storage_media_type)
    {

    case UX_HOST_CLASS_STORAGE_MEDIA_FAT_DISK:
    case UX_HOST_CLASS_STORAGE_MEDIA_IOMEGA_CLICK:

        storage -> ux_host_class_storage_sector_size =  UX_HOST_CLASS_STORAGE_SECTOR_SIZE_FAT;
        break;

    case UX_HOST_CLASS_STORAGE_MEDIA_CDROM:
    case UX_HOST_CLASS_STORAGE_MEDIA_OPTICAL_DISK:

        storage -> ux_host_class_storage_sector_size =  UX_HOST_CLASS_STORAGE_SECTOR_SIZE_OTHER;
        break;

    default:

        /* Unsupported device.  */
        return(UX_HOST_CLASS_MEDIA_NOT_SUPPORTED);
    }

    /* First, we test if the device is ready.  Then try to read its capacity.
       On floppies, this operation tends to fail a few times. So we try harder.  */
    for (command_retry = 0; command_retry < UX_HOST_CLASS_STORAGE_REQUEST_SENSE_RETRY; command_retry++)
    {

        /* Some devices require we do this.  */
        status =  _ux_host_class_storage_media_format_capacity_get(storage);
        if (status != UX_SUCCESS)
            return(status);
#endif

        /* Use a pointer for the cbw, easier to manipulate.  */
        cbw =  (UCHAR *) storage -> ux_host_class_storage_cbw;

        /* Get the READ_CAPACITY command Length.  */
#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
        if (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_STORAGE_SUBCLASS_UFI)
            command_length =  UX_HOST_CLASS_STORAGE_READ_CAPACITY_COMMAND_LENGTH_UFI;
        else
            command_length =  UX_HOST_CLASS_STORAGE_READ_CAPACITY_COMMAND_LENGTH_SBC;
#else
        command_length =  UX_HOST_CLASS_STORAGE_READ_CAPACITY_COMMAND_LENGTH_SBC;
#endif

        /* Initialize the CBW for this command.  */
        _ux_host_class_storage_cbw_initialize(storage, UX_HOST_CLASS_STORAGE_DATA_IN, UX_HOST_CLASS_STORAGE_READ_CAPACITY_RESPONSE_LENGTH, command_length);

        /* Prepare the READ_CAPACITY command block.  */
        *(cbw + UX_HOST_CLASS_STORAGE_CBW_CB + UX_HOST_CLASS_STORAGE_READ_CAPACITY_OPERATION) =  UX_HOST_CLASS_STORAGE_SCSI_READ_CAPACITY;

        /* Obtain a block of memory for the answer.  */
        capacity_response =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_STORAGE_READ_CAPACITY_RESPONSE_LENGTH);
        if (capacity_response == UX_NULL)
            return(UX_MEMORY_INSUFFICIENT);

#if defined(UX_HOST_STANDALONE)

        /* Initialize state for transport.  */
        UX_HOST_CLASS_STORAGE_TRANS_STATE_RESET(storage);
        storage -> ux_host_class_storage_state_state = UX_HOST_CLASS_STORAGE_STATE_TRANSPORT;
        storage -> ux_host_class_storage_state_next = UX_HOST_CLASS_STORAGE_STATE_CAP_SAVE;
        storage -> ux_host_class_storage_trans_data = capacity_response;
        status = UX_SUCCESS;
        return(status);
#else

        /* Send the command to transport layer.  */
        status =  _ux_host_class_storage_transport(storage, capacity_response);

        /* Check for error during transfer.  */
        if (status != UX_SUCCESS)
        {

            /* Free the memory resource used for the command response.  */
            _ux_utility_memory_free(capacity_response);

            /* We return a sad status.  */
            return(status);
        }

        /* Check the sense code */
        if (storage -> ux_host_class_storage_sense_code == UX_SUCCESS)
        {

#if defined(UX_HOST_CLASS_STORAGE_NO_FILEX)
            /* Save the number of sectors.  */
            storage -> ux_host_class_storage_last_sector_number = _ux_utility_long_get_big_endian(capacity_response + UX_HOST_CLASS_STORAGE_READ_CAPACITY_DATA_LBA);
#endif

            /* The data is valid, save the sector size.  */
            storage -> ux_host_class_storage_sector_size =  _ux_utility_long_get_big_endian(capacity_response + UX_HOST_CLASS_STORAGE_READ_CAPACITY_DATA_SECTOR_SIZE);

            /* Free the memory resource used for the command response.  */
            _ux_utility_memory_free(capacity_response);

            /* We return a happy status.  */
            return(UX_SUCCESS);
        }

        /* Free the memory resource used for the command response.  */
        _ux_utility_memory_free(capacity_response);
    }

    /* We get here when we could not retrieve the sector size through the READ_CAPACITY command.
       It's OK, we still calculated the default based on the device type.  */
    return(UX_SUCCESS);
#endif
}
