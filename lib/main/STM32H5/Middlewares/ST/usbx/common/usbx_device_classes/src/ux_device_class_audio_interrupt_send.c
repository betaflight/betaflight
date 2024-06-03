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
/**   Device Audio Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_audio.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_audio_interrupt_send               PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function queues audio interrupt data.                          */
/*                                                                        */
/*    Note the interrupt data size is predefined on initialization:       */
/*    - for Audio 1.0 interrupt status word is 2 bytes                    */
/*    - for Audio 2.0 interrupt data message is 6 bytes                   */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    audio                                 Address of audio instance     */
/*    int_data                              Interrupt data (2 or 6 bytes) */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Initial Version 6.1.12        */
/*  10-31-2022     Yajun Xia                Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT    _ux_device_class_audio_interrupt_send(UX_DEVICE_CLASS_AUDIO *audio, UCHAR *int_data)
{
#if !defined(UX_DEVICE_CLASS_AUDIO_INTERRUPT_SUPPORT)
    UX_PARAMETER_NOT_USED(audio);
    UX_PARAMETER_NOT_USED(int_data);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_SLAVE_DEVICE             *device;
UX_SLAVE_ENDPOINT           *endpoint;
UCHAR                       *buff, *end;
ULONG                       size;
ULONG                       i;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* As long as the device is in the CONFIGURED state.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
    {

        /* Cannot proceed with command, the interface is down.  */
        return(UX_CONFIGURATION_HANDLE_UNKNOWN);
    }

    /* Check if endpoint is available.  */
    endpoint = audio -> ux_device_class_audio_interrupt;
    if (endpoint == UX_NULL)
        return(UX_FUNCTION_NOT_SUPPORTED);

    /* Get interrupt data size.  */
    size = audio -> ux_device_class_audio_status_size;

    /* Protect queue status.  */
    _ux_device_mutex_on(&audio -> ux_device_class_audio_status_mutex);

    /* Check if data exist.  */
    buff = audio -> ux_device_class_audio_status_tail;
    end = audio -> ux_device_class_audio_status_queue + audio -> ux_device_class_audio_status_queue_bytes;
    for (i = 0; i < audio -> ux_device_class_audio_status_queued; i += size)
    {

        /* Check if data match.  */
        if (_ux_utility_memory_compare(buff, int_data, size) == UX_SUCCESS)
        {

            /* Already queued.  */
            _ux_device_mutex_off(&audio -> ux_device_class_audio_status_mutex);
            return(UX_SUCCESS);
        }

        /* Next saved data.  */
        buff += size;
        if (buff >= end)
            buff = audio -> ux_device_class_audio_status_queue;
    }

    /* No data match before buff achieve head.  */
    UX_ASSERT(buff == audio -> ux_device_class_audio_status_head);

    /* If no free space, return busy (pending).  */
    if (audio -> ux_device_class_audio_status_queued >=
        audio -> ux_device_class_audio_status_queue_bytes)
    {

        /* No queue space, pending.  */
        _ux_device_mutex_off(&audio -> ux_device_class_audio_status_mutex);
        return(UX_BUSY);
    }

    /* Copy data to head.  */
    _ux_utility_memory_copy(buff, int_data, size); /* Use case of memcpy is verified. */

    /* Move head.  */
    buff += size;
    if (buff >= end)
        buff = audio -> ux_device_class_audio_status_queue;
    audio -> ux_device_class_audio_status_head = buff;

    /* Add to queued bytes.  */
    audio -> ux_device_class_audio_status_queued += size;

    /* Unprotect queue status.  */
    _ux_device_mutex_off(&audio -> ux_device_class_audio_status_mutex);

    /* Notify status thread to issue interrupt request.  */
    _ux_device_semaphore_put(&audio -> ux_device_class_audio_status_semaphore);

    /* Resume interrupt thread.  */
    _ux_device_thread_resume(&audio -> ux_device_class_audio_class -> ux_slave_class_thread);

    /* Return success.  */
    return(UX_SUCCESS);
#endif
}
