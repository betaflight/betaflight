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
/*    _ux_host_class_storage_media_mount                  PORTABLE C      */
/*                                                           6.1.9        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function tries to read the first sector on the media. It then  */
/*    determines if this is a partition sector or a boot sector. If the   */
/*    sector contains partition information, each partition is checked    */
/*    for a DOS aware partition and mounted by UX_MEDIA (default FileX).  */
/*                                                                        */
/*    If there is no partition sector or boot sector, we simply inform    */
/*    UX_MEDIA (default FileX) that a device is present and that its boot */
/*    sector  should be periodically read to see if the device is mounted.*/
/*    This mechanism applies to storage where the media can be removed    */
/*    (floppy, ZIP, flash/smart media readers).                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    storage                               Pointer to storage class      */
/*    sector                                Boot sector start             */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_class_storage_media_mount    Mount media                   */
/*    _ux_host_class_storage_media_open     Open media                    */
/*    _ux_host_class_storage_partition_read Read partition                */
/*    _ux_host_class_storage_media_read     Read sectors of media         */
/*    _ux_host_class_storage_start_stop     Start the media               */
/*    _ux_host_class_storage_unit_ready_test                              */
/*                                          Test unit ready               */
/*    _ux_utility_memory_allocate           Allocate memory block         */
/*    _ux_utility_memory_free               Release memory block          */
/*    _ux_utility_short_get                 Get 16-bit value              */
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
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed exFAT mounting,       */
/*                                            resulting in version 6.1.9  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_media_mount(UX_HOST_CLASS_STORAGE *storage, ULONG sector)
{

#if defined(UX_HOST_CLASS_STORAGE_NO_FILEX)
    UX_PARAMETER_NOT_USED(storage);
    UX_PARAMETER_NOT_USED(sector);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else
UCHAR           *sector_memory;
UINT            status;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_STORAGE_MEDIA_MOUNT, storage, sector, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    /* If the device is UFI, we need to start it.  */
    if (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceProtocol == UX_HOST_CLASS_STORAGE_PROTOCOL_CBI)
    {

        /* Start the media.  */
        status =  _ux_host_class_storage_start_stop(storage, UX_HOST_CLASS_STORAGE_START_MEDIA);

        /* Check the status. If the device does not start, fail the operation.
           But we still return SUCCESS otherwise the storage thread has no chance to inspect this instance.  */
        if (status != UX_SUCCESS)
            return(UX_SUCCESS);
    }
#endif

    /* Obtain memory for reading the partition sector, we do not use the storage instance
       memory because this function is reentrant.  */
    sector_memory =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, storage -> ux_host_class_storage_sector_size);
    if (sector_memory == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Check if the device is now ready.  */
    status =  _ux_host_class_storage_unit_ready_test(storage);

    /* Check the status. There may be a transport error or the device is not ready.  */
    if (status == UX_SUCCESS)

        /* Read the very first sector from the media.  */
        status =  _ux_host_class_storage_media_read(storage, sector, 1, sector_memory);

    /* Check the status. There may be a transport error or the device is not ready.  */
    if (status != UX_SUCCESS)
    {

        /* In any case, we free the sector memory.  */
        _ux_utility_memory_free(sector_memory);

        if (status == UX_HOST_CLASS_STORAGE_SENSE_ERROR)

            /* The media is not there, so we will try to check for it at the storage
               thread level every x seconds.  */
            return(UX_SUCCESS);

        /* Since there was a transport error or command error, we do not try to mount
           a fake media.  */
        return(status);
    }

    /* We need to examine this sector and determine if this is a partition sector
       or a boot sector.  */
    if (_ux_utility_short_get(sector_memory + 510) == UX_HOST_CLASS_STORAGE_PARTITION_SIGNATURE)
    {

        /* If we have a signature, we know it is a valid media and it may be formatted.
           If there is no signature, the media exist but maybe not formatted. Now determine
           if this is a boot sector. The boot sector has a special signature in the first
           couple of bytes.
           BUT !!! The first boot sector of an iPod is actually a partition, so we need to
           look further and see if the number of sector per FAT and the first partition are
           set to 0. This will indicate a partition sector. Though exFAT has no BIOS parameters
           block and should be excluded in this case.
           */
        if ((*sector_memory == 0xe9) || ((*sector_memory == 0xeb) && *(sector_memory + 2) == 0x90))
        {

            /* Check for fake boot sector.  */
            if (_ux_utility_short_get(sector_memory + 0x16) != 0x0 ||
                _ux_utility_long_get(sector_memory + 0x24) != 0x0 ||
                (*(sector_memory + 1) == 0x76) /* exFAT volume  */)
            {

                /* This is a boot sector signature.  */
                _ux_host_class_storage_media_open(storage, sector);
                _ux_utility_memory_free(sector_memory);
                return(UX_SUCCESS);
            }
        }

        _ux_host_class_storage_partition_read(storage, sector_memory, sector);
        _ux_utility_memory_free(sector_memory);
        return(UX_SUCCESS);
    }
    else
    {

        /* The drive does not contain a partition table or FAT MBR at LBA 0*/
        status =  UX_ERROR;
    }

    /* Free all resources used.  */
    _ux_utility_memory_free(sector_memory);

    /* Return completion status.  */
    return(status);
#endif /* !defined(UX_HOST_CLASS_STORAGE_NO_FILEX) */
}
