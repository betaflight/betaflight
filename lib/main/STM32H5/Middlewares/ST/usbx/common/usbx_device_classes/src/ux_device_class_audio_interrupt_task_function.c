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

#if defined(UX_DEVICE_STANDALONE) && defined(UX_DEVICE_CLASS_AUDIO_INTERRUPT_SUPPORT)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_audio_interrupt_task_function      PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yajun Xia, Microsoft Corporation                                    */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is task of INTERRUPT IN from the Audio class.         */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    audio                                 Address of audio instance     */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    State machine Status to check                                       */
/*    UX_STATE_EXIT                         Abnormal, to reset state      */
/*    UX_STATE_IDLE                         No interrupt transfer running */
/*    UX_STATE_WAIT                         Keep running, waiting         */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_system_error_handler              System error trap             */
/*    _ux_utility_memory_copy               Copy memory                   */
/*    _ux_device_stack_transfer_run         Run Transfer state machine    */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Audio Class (task)                                                  */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  10-31-2022     Yajun Xia                Initial Version 6.2.0         */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_audio_interrupt_task_function(UX_DEVICE_CLASS_AUDIO *audio)
{

UINT                            status;
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_ENDPOINT               *endpoint;
UX_SLAVE_TRANSFER               *transfer;
UCHAR                           *buff;


    /* Get stack device instance.  */
    device = audio -> ux_device_class_audio_device;

    /* Check if the device is configured.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CONFIGURATION_HANDLE_UNKNOWN);

        audio -> ux_device_class_audio_interrupt_task_state = UX_STATE_EXIT;
        return(UX_STATE_EXIT);
    }
    
    /* Get endpoint instance.  */
    endpoint = audio -> ux_device_class_audio_interrupt;

    /* Endpoint not available, maybe it's alternate setting 0.  */
    if (endpoint == UX_NULL)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ENDPOINT_HANDLE_UNKNOWN);

        audio -> ux_device_class_audio_interrupt_task_state = UX_STATE_RESET;
        return(UX_STATE_IDLE);
    }

    /* No packet in queue */
    if (audio -> ux_device_class_audio_status_queued == 0)
    {
        audio -> ux_device_class_audio_interrupt_task_state = UX_STATE_RESET;
        audio -> ux_device_class_audio_interrupt_task_status = UX_TRANSFER_NOT_READY;
        return(UX_STATE_EXIT);
    }

    /* Get transfer instance.  */
    transfer = &endpoint -> ux_slave_endpoint_transfer_request;

    /* Handle state cases.  */
    switch(audio -> ux_device_class_audio_interrupt_task_state)
    {
    case UX_STATE_RESET:
        audio -> ux_device_class_audio_interrupt_task_state = UX_DEVICE_CLASS_AUDIO_INTERRUPT_START;
        audio -> ux_device_class_audio_interrupt_task_status = UX_TRANSFER_NO_ANSWER;

    /* Fall through.  */
    case UX_DEVICE_CLASS_AUDIO_INTERRUPT_START:
        if (audio -> ux_device_class_audio_status_size > transfer -> ux_slave_transfer_request_transfer_length)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_ERROR);

            audio -> ux_device_class_audio_interrupt_task_state = UX_STATE_RESET;
            audio -> ux_device_class_audio_interrupt_task_status = UX_ERROR;
            return(UX_STATE_EXIT);
        }

        _ux_utility_memory_copy(transfer -> ux_slave_transfer_request_data_pointer,
                                audio -> ux_device_class_audio_status_tail, audio -> ux_device_class_audio_status_size); /* Use case of memcpy is verified. */

        audio -> ux_device_class_audio_interrupt_task_state = UX_DEVICE_CLASS_AUDIO_INTERRUPT_WAIT;

    /* Fall through.  */
    case UX_DEVICE_CLASS_AUDIO_INTERRUPT_WAIT:

        /* Start frame transfer anyway.  */
        status = _ux_device_stack_transfer_run(transfer, audio -> ux_device_class_audio_status_size, 
                                                audio -> ux_device_class_audio_status_size);

        /* Any error or success case.  */
        if (status < UX_STATE_NEXT)
        {

            /* Error notification!  */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_ERROR);

            audio -> ux_device_class_audio_interrupt_task_state = UX_STATE_RESET;
            audio -> ux_device_class_audio_interrupt_task_status = 
                    transfer -> ux_slave_transfer_request_completion_code;
            return(UX_STATE_EXIT);
        }

        /* Success case.  */
        if (status == UX_STATE_NEXT)
        {
            buff = audio -> ux_device_class_audio_status_tail;
            buff += audio -> ux_device_class_audio_status_size;

            if (buff >= (audio -> ux_device_class_audio_status_queue + audio -> ux_device_class_audio_status_queue_bytes))
                buff = audio -> ux_device_class_audio_status_queue;
            
            audio -> ux_device_class_audio_status_tail = buff;
            audio -> ux_device_class_audio_status_queued -= audio -> ux_device_class_audio_status_size;
            audio -> ux_device_class_audio_interrupt_task_state = UX_DEVICE_CLASS_AUDIO_INTERRUPT_START;
            audio -> ux_device_class_audio_interrupt_task_status = 
                    transfer -> ux_slave_transfer_request_completion_code;
        }

        /* Keep waiting.  */
        return(UX_STATE_WAIT);

    default: /* Error.  */
        audio -> ux_device_class_audio_interrupt_task_state = UX_STATE_RESET;
        audio -> ux_device_class_audio_interrupt_task_status = UX_INVALID_STATE;
        break;
    }

    /* Error case.  */
    return(UX_STATE_EXIT);
}
#endif
