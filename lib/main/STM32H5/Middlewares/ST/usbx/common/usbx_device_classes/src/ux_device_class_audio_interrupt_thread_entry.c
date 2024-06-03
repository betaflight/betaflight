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


#if !defined(UX_DEVICE_STANDALONE) && defined(UX_DEVICE_CLASS_AUDIO_INTERRUPT_SUPPORT)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_audio_interrupt_thread_entry       PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is thread of INTERRUPT IN from the Audio class.       */
/*                                                                        */
/*    It's for RTOS mode.                                                 */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    audio                                 Address of audio instance     */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_system_error_handler              System error trap             */
/*    _ux_device_thread_suspend             Suspend thread used           */
/*    _ux_device_stack_transfer_request     Issue transfer request        */
/*    _ux_utility_memory_copy               Copy data                     */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    ThreadX                                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Initial Version 6.1.12        */
/*                                                                        */
/**************************************************************************/
VOID _ux_device_class_audio_interrupt_thread_entry(ULONG audio_inst)
{

UINT                            status;
UX_DEVICE_CLASS_AUDIO           *audio;
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_ENDPOINT               *endpoint;
UX_SLAVE_TRANSFER               *transfer;
ULONG                           size;
UCHAR                           *buff;


    /* Get Audio class instance.  */
    UX_THREAD_EXTENSION_PTR_GET(audio, UX_DEVICE_CLASS_AUDIO, audio_inst)

    /* Get stack device instance.  */
    device = audio -> ux_device_class_audio_device;

    /* This thread runs forever but can be suspended or resumed.  */
    while(1)
    {
        while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED)
        {

            /* Get endpoint instance.  */
            endpoint = audio -> ux_device_class_audio_interrupt;

            /* Endpoint not available, maybe it's alternate setting 0.  */
            if (endpoint == UX_NULL)
                break;

            /* Get transfer instance.  */
            transfer = &endpoint -> ux_slave_endpoint_transfer_request;

            /* Get semaphore before start transfer.  */
            status = _ux_device_semaphore_get(&audio -> ux_device_class_audio_status_semaphore, UX_WAIT_FOREVER);
            if (status != UX_SUCCESS)
            {

                /* Error notification!  */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_SEMAPHORE_ERROR);
                break;
            }

            /* Get interrupt data size.  */
            size = audio -> ux_device_class_audio_status_size;
            UX_ASSERT(size <= transfer -> ux_slave_transfer_request_transfer_length);

            /* Copy data in tail.  */
            buff = audio -> ux_device_class_audio_status_tail;
            _ux_utility_memory_copy(transfer -> ux_slave_transfer_request_data_pointer,
                                    buff, size); /* Use case of memcpy is verified. */

            /* Start frame transfer anyway.  */
            status = _ux_device_stack_transfer_request(transfer, size, size);

            /* Check transfer status.  */
            if (status != UX_SUCCESS)
            {

                /* Error notification!  */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_ERROR);
                break;
            }

            /* Calculate next tail.  */
            buff += size;
            if (buff >= (audio -> ux_device_class_audio_status_queue + audio -> ux_device_class_audio_status_queue_bytes))
                buff = audio -> ux_device_class_audio_status_queue;

            /* Update queue status.  */
            _ux_device_mutex_on(&audio -> ux_device_class_audio_status_mutex);
            audio -> ux_device_class_audio_status_tail = buff;
            audio -> ux_device_class_audio_status_queued -= size;
            _ux_device_mutex_off(&audio -> ux_device_class_audio_status_mutex);

        } /* while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED) */

        /* We need to suspend ourselves. We will be resumed by the device enumeration module or when a change of alternate setting happens.  */
        _ux_device_thread_suspend(&audio -> ux_device_class_audio_class -> ux_slave_class_thread);
    }
}
#endif
