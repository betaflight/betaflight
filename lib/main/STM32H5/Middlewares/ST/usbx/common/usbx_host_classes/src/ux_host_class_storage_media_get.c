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


#if defined(UX_HOST_CLASS_STORAGE_NO_FILEX)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_storage_media_get                    PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function retrieves storage media from the storage device.      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    storage                               Pointer to storage class      */
/*    media_lun                             Media logical unit No. (LUN)  */
/*    storage_media                         Holds returned storage media  */
/*                                          pointer                       */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Storage Class                                                       */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  09-30-2020     Chaoqiong Xiao           Initial Version 6.1           */
/*  08-02-2021     Wen Wang                 Modified comment(s),          */
/*                                            fixed spelling error,       */
/*                                            resulting in version 6.1.8  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined media to search,    */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            cleared CSTAT warning,      */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT    _ux_host_class_storage_media_get(UX_HOST_CLASS_STORAGE *storage,
                                         ULONG media_lun,
                                         UX_HOST_CLASS_STORAGE_MEDIA **storage_media)
{
#if !defined(UX_HOST_CLASS_STORAGE_NO_FILEX)
    UX_PARAMETER_NOT_USED(storage);
    UX_PARAMETER_NOT_USED(media_index);
    UX_PARAMETER_NOT_USED(storage_media);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_HOST_CLASS_STORAGE_MEDIA     *storage_medias;
UX_HOST_CLASS_STORAGE_MEDIA     *storage_media_inst;
UX_HOST_CLASS                   *class_inst;
UINT                            scan_index;


    /* If storage is not live, media is not ready.  */
    if (storage -> ux_host_class_storage_state != UX_HOST_CLASS_INSTANCE_LIVE)
        return(UX_ERROR);

    /* Get storage class instance.  */
    class_inst = storage -> ux_host_class_storage_class;

    /* Get shared storage media array.  */
    storage_medias = (UX_HOST_CLASS_STORAGE_MEDIA *)class_inst -> ux_host_class_media;

    /* Search media to find the right one.  */
    for(scan_index = 0; scan_index < UX_HOST_CLASS_STORAGE_MAX_MEDIA; scan_index ++)
    {
        storage_media_inst = &storage_medias[scan_index];

        /* Skip storage media not used.  */
        if (storage_media_inst -> ux_host_class_storage_media_status != UX_USED)
            continue;

        /* Skip storage media not belong to the storage.  */
        if (storage_media_inst -> ux_host_class_storage_media_storage != storage)
            continue;

        /* Skip storage media with different LUN.  */
        if (storage_media_inst -> ux_host_class_storage_media_lun != media_lun)
            continue;

        /* Store the media instance.  */
        {
            *storage_media = storage_media_inst;
            return(UX_SUCCESS);
        }
    }

    /* Media not found.  */
    return(UX_ERROR);
#endif
}
#endif
