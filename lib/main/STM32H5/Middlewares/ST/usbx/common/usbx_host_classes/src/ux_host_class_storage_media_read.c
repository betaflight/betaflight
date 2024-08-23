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


#if defined(UX_HOST_STANDALONE)
VOID _ux_host_class_storage_read_initialize(UX_HOST_CLASS_STORAGE *storage,
                ULONG sector_start, ULONG sector_count);

VOID
#else
static inline VOID
#endif
_ux_host_class_storage_read_initialize(UX_HOST_CLASS_STORAGE *storage,
                ULONG sector_start, ULONG sector_count)
{
UCHAR       *cbw;
UCHAR       *cbw_cb;
ULONG       command_length;

    /* Use a pointer for the cbw, easier to manipulate.  */
    cbw = (UCHAR *)storage -> ux_host_class_storage_cbw;
    cbw_cb = cbw + UX_HOST_CLASS_STORAGE_CBW_CB;

    /* Get the Read Command Length.  */
#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    if (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceSubClass ==
        UX_HOST_CLASS_STORAGE_SUBCLASS_UFI)
        command_length =  UX_HOST_CLASS_STORAGE_READ_COMMAND_LENGTH_UFI;
    else
        command_length =  UX_HOST_CLASS_STORAGE_READ_COMMAND_LENGTH_SBC;
#else
    command_length =  UX_HOST_CLASS_STORAGE_READ_COMMAND_LENGTH_SBC;
#endif

    /* Initialize the CBW for this command.  */
    _ux_host_class_storage_cbw_initialize(storage,
                    UX_HOST_CLASS_STORAGE_DATA_IN,
                    sector_count * storage -> ux_host_class_storage_sector_size,
                    command_length);
    
    /* Prepare the MEDIA READ command block.  */
    *(cbw_cb + UX_HOST_CLASS_STORAGE_READ_OPERATION) =  UX_HOST_CLASS_STORAGE_SCSI_READ16;

    /* Store the sector start (LBA field).  */
    _ux_utility_long_put_big_endian(cbw_cb + UX_HOST_CLASS_STORAGE_READ_LBA, sector_start);

    /* Store the number of sectors to read.  */
    _ux_utility_short_put_big_endian(cbw_cb + UX_HOST_CLASS_STORAGE_READ_TRANSFER_LENGTH, (USHORT) sector_count);
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_storage_media_read                   PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will read one or more logical sector from the media.  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    storage                               Pointer to storage class      */ 
/*    sector_start                          Starting sector               */ 
/*    sector_count                          Number of sectors to read     */ 
/*    data_pointer                          Pointer to data to read       */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_storage_cbw_initialize Initialize the CBW            */ 
/*    _ux_host_class_storage_transport      Send command                  */ 
/*    _ux_utility_long_put_big_endian       Put 32-bit word               */ 
/*    _ux_utility_short_put_big_endian      Put 16-bit word               */ 
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
UINT  _ux_host_class_storage_media_read(UX_HOST_CLASS_STORAGE *storage, ULONG sector_start,
                                    ULONG sector_count, UCHAR *data_pointer)
{
#if defined(UX_HOST_STANDALONE)
UINT            status;
    do {
        status = _ux_host_class_storage_read_write_run(storage, UX_TRUE,
                                    sector_start, sector_count, data_pointer);
    } while(status == UX_STATE_WAIT);
    return(storage -> ux_host_class_storage_status);
#else
UINT            status;
UINT            media_retry;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_STORAGE_MEDIA_READ, storage, sector_start, sector_count, data_pointer, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Reset the retry count.  */
    media_retry =  UX_HOST_CLASS_STORAGE_REQUEST_SENSE_RETRY;

    /* We may need several attempts.  */
    while (media_retry-- != 0)
    {

        /* Initialize CBW.  */
        _ux_host_class_storage_read_initialize(storage, sector_start, sector_count);

        /* Send the command to transport layer.  */
        status =  _ux_host_class_storage_transport(storage, data_pointer);
        if (status != UX_SUCCESS)
            return(status);

        /* Did the command succeed?  */
        if (storage -> ux_host_class_storage_sense_code == UX_SUCCESS)
        {

            /* Check for completeness of sector read.  */
            if (storage -> ux_host_class_storage_data_phase_length != sector_count * storage -> ux_host_class_storage_sector_size)
            {

                /* This can happen if the device sent less data than the host
                   requested. This does not fit our definition of success and
                   retrying shouldn't change the outcome, so we return an error.  */

                /* We got an error during read. Packet not complete.  */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_DATA_LESS_THAN_EXPECTED);

                /* Return to UX_MEDIA (default FileX).  */
                return(UX_ERROR);
            }

            /* The read succeeded.  */
            return(UX_SUCCESS);
        }

        /* The command did not succeed. Retry.  */
    }

    /* Check if the media in the device has been removed. If so
       we have to tell UX_MEDIA (default FileX) that the media is closed.  */
    return(UX_HOST_CLASS_STORAGE_SENSE_ERROR);                                            
#endif
}
