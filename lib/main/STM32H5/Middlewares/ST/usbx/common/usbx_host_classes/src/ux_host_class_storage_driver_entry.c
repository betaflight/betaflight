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


/* Defined if FileX is not integrated in USBX
   and it's used as external module for file system.  */

/* #define UX_HOST_CLASS_STORAGE_EXT_FILEX */

#if defined(UX_HOST_CLASS_STORAGE_NO_FILEX)

#ifdef FX_API_H /* For test, confirm FX is not included before.  */
#error fx_api.h should not be included in this mode
#endif

/* FX not integrated in UX, but used as external module.  */
#if defined(UX_HOST_CLASS_STORAGE_EXT_FILEX)

/* FX related things needs define here.  */
#include "fx_api.h"
#define UX_MEDIA                                    FX_MEDIA
VOID    _ux_host_class_storage_driver_entry(UX_MEDIA *media);

/* FX driver is available to support FX as external module.  */
#ifndef UX_HOST_CLASS_STORAGE_DRIVER_ENTRY_ENABLE
#define UX_HOST_CLASS_STORAGE_DRIVER_ENTRY_ENABLE
#endif
#endif
#else

/* FX driver is used for RTOS mode by default.  */
#ifndef UX_HOST_CLASS_STORAGE_DRIVER_ENTRY_ENABLE
#define UX_HOST_CLASS_STORAGE_DRIVER_ENTRY_ENABLE
#endif
#endif


#if defined(UX_HOST_CLASS_STORAGE_DRIVER_ENTRY_ENABLE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_storage_driver_entry                 PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the entry point for the FileX file system. All     */
/*    FileX driver I/O calls are are multiplexed here and rerouted to     */
/*    the proper USB storage class functions.                             */
/*                                                                        */
/*    When the entry is for storage with FX support (not in standalone    */
/*    mode, and with FileX), the FX media is openned in storage mount     */
/*    flow, and can be directly used in application after mounted.        */
/*                                                                        */
/*    When the entry is for no FX mode (FX in external module, and USBX   */
/*    is compiled without FileX integration), it is an example with       */
/*    disk partition support. In this case the FX media does not operate  */
/*    inside the storage flow. Actions are taken when application mounts  */
/*    media to FX_MEDIA and then have access to media APIs.               */
/*                                                                        */
/*    In no FX mode demo, it assumes media is managed with partition      */
/*    start from sector address of FX_MEDIA::fx_media_reserved_for_user.  */
/*                                                                        */
/*    The following links are not initialized in no FX mode, they must be */
/*    initialized before using the entry in no FX mode:                   */
/*    - FX_MEDIA::fx_media_reserved_for_user                              */
/*                                          Partition start sector inside */
/*                                          the whole storage media, must */
/*                                          set before media open         */
/*    - FX_MEDIA::fx_media_driver_info      Pointer to storage media,     */
/*                                          assigned while calling media  */
/*                                          open (fx_media_open)          */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    media                                 FileX media pointer           */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_class_storage_sense_code_translate                         */
/*                                          Translate error status codes  */
/*    _ux_host_class_storage_media_read     Read sector(s)                */
/*    _ux_host_class_storage_media_write    Write sector(s)               */
/*    _ux_host_semaphore_get                Get protection semaphore      */
/*    _ux_host_semaphore_put                Release protection semaphore  */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    FileX                                                               */
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
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added implement to support  */
/*                                            external FX mode,           */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved external FX mode,  */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_storage_driver_entry(FX_MEDIA *media)
{

UINT                            status;
UX_HOST_CLASS_STORAGE           *storage;
UX_HOST_CLASS_STORAGE_MEDIA     *storage_media;
ULONG                           partition_start;


    /* Get the pointers to the instances and partition start.  */
#if !defined(UX_HOST_CLASS_STORAGE_NO_FILEX)
    storage =  (UX_HOST_CLASS_STORAGE *) media -> fx_media_driver_info;
    storage_media =  (UX_HOST_CLASS_STORAGE_MEDIA *) media -> fx_media_reserved_for_user;
    partition_start = storage_media -> ux_host_class_storage_media_partition_start;
#else
    storage_media = (UX_HOST_CLASS_STORAGE_MEDIA *) media -> fx_media_driver_info;
    storage = storage_media -> ux_host_class_storage_media_storage;
    partition_start = (ULONG) media -> fx_media_reserved_for_user;
#endif

    /* Ensure the instance is valid.  */
    if ((storage -> ux_host_class_storage_state !=  UX_HOST_CLASS_INSTANCE_LIVE) &&
        (storage -> ux_host_class_storage_state !=  UX_HOST_CLASS_INSTANCE_MOUNTING))
    {

        /* Class instance is invalid. Return an error!  */
        media -> fx_media_driver_status =  FX_PTR_ERROR;
        return;
    }

#if defined(UX_HOST_CLASS_STORAGE_NO_FILEX)

    /* Ensure the media is valid.  */
    if ((storage_media -> ux_host_class_storage_media_storage != storage) ||
        (storage_media -> ux_host_class_storage_media_status != UX_USED))
    {

        /* Media instance is invalid.  */
        media -> fx_media_driver_status =  FX_PTR_ERROR;
        return;
    }
#endif

    /* Protect Thread reentry to this instance.  */
    status = _ux_host_class_storage_lock(storage, UX_WAIT_FOREVER);
    if (status != UX_SUCCESS)
    {

        /* Unable to lock, return an error.  */
        media -> fx_media_driver_status =  FX_INVALID_STATE;
        return;
    }

    /* Restore the LUN number from the media instance.  */
    storage -> ux_host_class_storage_lun =  storage_media -> ux_host_class_storage_media_lun;

    /* And the sector size.  */
    storage -> ux_host_class_storage_sector_size =
                storage_media -> ux_host_class_storage_media_sector_size;

#if defined(UX_HOST_CLASS_STORAGE_NO_FILEX)

    /* Restore current used last sector number.  */
    storage -> ux_host_class_storage_last_sector_number =
                storage_media -> ux_host_class_storage_media_number_sectors - 1;
#endif

    /* Look at the request specified by the FileX caller.  */
    switch (media -> fx_media_driver_request)
    {

    case FX_DRIVER_READ:

        /* Read one or more sectors.  */
        status =  _ux_host_class_storage_media_read(storage,
                                media -> fx_media_driver_logical_sector + partition_start,
                                media -> fx_media_driver_sectors,
                                media -> fx_media_driver_buffer);

        /* Check completion status.  */
        if (status == UX_SUCCESS)
            media -> fx_media_driver_status =  FX_SUCCESS;
        else
        {

#if defined(UX_HOST_STANDALONE)

            /* Poll status.  */
            _ux_host_class_storage_media_check(storage);
#endif

            media -> fx_media_driver_status =
                _ux_host_class_storage_sense_code_translate(storage, status);
        }
        break;


    case FX_DRIVER_WRITE:

        /* Write one or more sectors.  */
        status =  _ux_host_class_storage_media_write(storage,
                                media -> fx_media_driver_logical_sector + partition_start,
                                media -> fx_media_driver_sectors,
                                media -> fx_media_driver_buffer);

        /* Check completion status.  */
        if (status == UX_SUCCESS)
            media -> fx_media_driver_status =  FX_SUCCESS;
        else
        {

#if defined(UX_HOST_STANDALONE)

            /* Poll status.  */
            _ux_host_class_storage_media_check(storage);
#endif

            media -> fx_media_driver_status =
                _ux_host_class_storage_sense_code_translate(storage,status);
        }
        break;


    case FX_DRIVER_FLUSH:

        /* Nothing to do. Just return a good status!  */
        media -> fx_media_driver_status =  FX_SUCCESS;
        break;


    case FX_DRIVER_ABORT:

        /* Nothing to do. Just return a good status!  */
        media -> fx_media_driver_status =  FX_SUCCESS;
        break;


    case FX_DRIVER_INIT:

#if defined(UX_HOST_STANDALONE)

            /* Poll status.  */
            _ux_host_class_storage_media_check(storage);
#endif

        /* Check for media protection.  We must do this operation here because FileX clears all the
           media fields before init.  */
        if (storage -> ux_host_class_storage_write_protected_media ==  UX_TRUE)

            /* The media is Write Protected. We tell FileX.  */
            media -> fx_media_driver_write_protect = UX_TRUE;

        /* This function always succeeds.  */
        media -> fx_media_driver_status =  FX_SUCCESS;
        break;


    case FX_DRIVER_UNINIT:

        /* Nothing to do. Just return a good status!  */
        media -> fx_media_driver_status =  FX_SUCCESS;
        break;


    case FX_DRIVER_BOOT_READ:

        /* Read the media boot sector.  */
        status =  _ux_host_class_storage_media_read(storage,
                partition_start, 1, media -> fx_media_driver_buffer);

        /* Check completion status.  */
        if (status == UX_SUCCESS)
            media -> fx_media_driver_status =  FX_SUCCESS;
        else
        {

#if defined(UX_HOST_STANDALONE)

            /* Poll status.  */
            _ux_host_class_storage_media_check(storage);
#endif

            media -> fx_media_driver_status =
                _ux_host_class_storage_sense_code_translate(storage,status);
        }
        break;


    case FX_DRIVER_BOOT_WRITE:

        /* Write the boot sector.  */
        status =  _ux_host_class_storage_media_write(storage,
                partition_start, 1, media -> fx_media_driver_buffer);

        /* Check completion status.  */
        if (status == UX_SUCCESS)
            media -> fx_media_driver_status =  FX_SUCCESS;
        else
        {

#if defined(UX_HOST_STANDALONE)

            /* Poll status.  */
            _ux_host_class_storage_media_check(storage);
#endif

            media -> fx_media_driver_status =
                _ux_host_class_storage_sense_code_translate(storage,status);
        }
        break;


    default:

        /* Invalid request from FileX */
        media -> fx_media_driver_status =  FX_IO_ERROR;
        break;
    }

    /* Unprotect thread reentry to this instance.  */
    _ux_host_class_storage_unlock(storage);
}
#endif
