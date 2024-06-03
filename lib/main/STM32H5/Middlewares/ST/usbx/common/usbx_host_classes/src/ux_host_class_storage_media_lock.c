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
/*    _ux_host_class_storage_media_lock                   PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function lock storage and select storage media for read/write. */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    storage_media                         Pointer to storage media to   */
/*                                          select                        */
/*    wait                                  Wait option                   */
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT    _ux_host_class_storage_media_lock(UX_HOST_CLASS_STORAGE_MEDIA *storage_media, ULONG wait)
{
#if !defined(UX_HOST_CLASS_STORAGE_NO_FILEX)
    UX_PARAMETER_NOT_USED(storage_media);
    UX_PARAMETER_NOT_USED(wait);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_HOST_CLASS_STORAGE           *storage;
UINT                            status;


    /* Get storage instance.  */
    storage = storage_media -> ux_host_class_storage_media_storage;
    if (storage == UX_NULL)
        return(UX_ERROR);

    /* Protect thread reentry to this instance.  */
    status = _ux_host_class_storage_lock(storage, wait);
    if (status != UX_SUCCESS)
        return(status);

    /* Select the media if success.  */
    storage -> ux_host_class_storage_lun = storage_media -> ux_host_class_storage_media_lun;
    storage -> ux_host_class_storage_sector_size = storage_media -> ux_host_class_storage_media_sector_size;
    storage -> ux_host_class_storage_last_sector_number = storage_media -> ux_host_class_storage_media_number_sectors - 1;

    /* Return success.  */
    return(UX_SUCCESS);
#endif
}
#endif
