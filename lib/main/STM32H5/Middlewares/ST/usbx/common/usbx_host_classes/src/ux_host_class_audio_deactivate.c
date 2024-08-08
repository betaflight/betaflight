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
/**   Audio Class                                                         */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_audio.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_audio_deactivate                     PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called when this instance of the audio has been    */
/*    removed from the bus either directly or indirectly. The iso pipes   */ 
/*    will be destroyed and the instanced removed.                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Pointer to command            */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_class_instance_destroy Destroy class instance        */ 
/*    _ux_host_stack_endpoint_transfer_abort Abort outstanding transfer   */ 
/*    _ux_host_mutex_on                     Get mutex                     */
/*    _ux_host_mutex_delete                 Delete mutex                  */
/*    _ux_utility_memory_free               Release memory block          */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Audio Class                                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added interrupt support,    */
/*                                            protect reentry with mutex, */
/*                                            added feedback support,     */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_audio_deactivate(UX_HOST_CLASS_COMMAND *command)
{

UX_HOST_CLASS_AUDIO     *audio;
#if defined(UX_HOST_CLASS_AUDIO_INTERRUPT_SUPPORT)
UX_HOST_CLASS_AUDIO_AC  *ac;
#endif

    /* Get the instance for this class.  */
    audio = (UX_HOST_CLASS_AUDIO *) command -> ux_host_class_command_instance;

#if defined(UX_HOST_CLASS_AUDIO_INTERRUPT_SUPPORT)
    if (_ux_host_class_audio_subclass_get(audio) == UX_HOST_CLASS_AUDIO_SUBCLASS_CONTROL)
    {
        ac = (UX_HOST_CLASS_AUDIO_AC *)audio;

        /* Stop interrupt and free allocated buffer for it.  */
        if (ac -> ux_host_class_audio_interrupt_endpoint)
        {
            _ux_host_stack_endpoint_transfer_abort(ac -> ux_host_class_audio_interrupt_endpoint);
            _ux_utility_memory_free(ac -> ux_host_class_audio_interrupt_endpoint ->
                                    ux_endpoint_transfer_request.ux_transfer_request_data_pointer);
        }

        /* The enumeration thread needs to sleep a while to allow the application or the class that may be using
        endpoints to exit properly.  */
        _ux_host_thread_schedule_other(UX_THREAD_PRIORITY_ENUM); 

        /* Destroy the instance.  */
        _ux_host_stack_class_instance_destroy(audio -> ux_host_class_audio_class, (VOID *) audio);
    }
    else
#endif
    {

        /* The audio is being shut down.  */
        audio -> ux_host_class_audio_state =  UX_HOST_CLASS_INSTANCE_SHUTDOWN;

        /* Protect thread reentry to this instance.  */
        _ux_host_mutex_on(&audio -> ux_host_class_audio_mutex);
        
        /* We need to abort transactions on the iso pipe.  */
        if (audio -> ux_host_class_audio_isochronous_endpoint)
            _ux_host_stack_endpoint_transfer_abort(audio -> ux_host_class_audio_isochronous_endpoint);
#if defined(UX_HOST_CLASS_AUDIO_FEEDBACK_SUPPORT)
    if (audio -> ux_host_class_audio_feedback_endpoint)
    {
        _ux_host_stack_endpoint_transfer_abort(audio -> ux_host_class_audio_feedback_endpoint);
        _ux_utility_memory_free(audio -> ux_host_class_audio_feedback_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer);
    }
#endif

        /* The enumeration thread needs to sleep a while to allow the application or the class that may be using
        endpoints to exit properly.  */
        _ux_host_thread_schedule_other(UX_THREAD_PRIORITY_ENUM); 

        /* Destroy the instance.  */
        _ux_host_stack_class_instance_destroy(audio -> ux_host_class_audio_class, (VOID *) audio);

        /* Destroy the semaphore.  */
        _ux_host_mutex_delete(&audio -> ux_host_class_audio_mutex);
    }

    /* Before we free the device resources, we need to inform the application
        that the device is removed.  */
    if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
    {
        
        /* Inform the application the device is removed.  */
        _ux_system_host -> ux_system_host_change_function(UX_DEVICE_REMOVAL, audio -> ux_host_class_audio_class, (VOID *) audio);
    }
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_AUDIO_DEACTIVATE, audio, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_UNREGISTER(audio);

    /* Free the audio instance memory.  */
    _ux_utility_memory_free(audio);
    
    /* Return successful completion.  */
    return(UX_SUCCESS);         
}

