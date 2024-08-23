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

UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_MULC_ULONG(sizeof(UX_HOST_CLASS_STORAGE_MEDIA), UX_HOST_CLASS_STORAGE_MAX_MEDIA), UX_HOST_CLASS_STORAGE_MAX_MEDIA_mul_ovf)

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_storage_entry                        PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the entry point of the storage class. It will be   */ 
/*    called by the USBX stack enumeration module when there is a new     */ 
/*    USB disk on the bus or when the USB disk is removed.                */
/*                                                                        */
/*    Version 2.0 of the storage class only supports USB FAT media and    */ 
/*    not CD-ROM.                                                         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Pointer to class command      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_storage_activate       Activate storage class        */ 
/*    _ux_host_class_storage_deactivate     Deactivate storage class      */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Free memory block             */
/*    _ux_utility_thread_create             Create storage class thread   */
/*    _ux_utility_thread_delete             Delete storage class thread   */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Host Stack                                                          */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added destroy command,      */
/*                                            used host class extension   */
/*                                            pointer for class specific  */
/*                                            structured data,            */
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed class ext access,     */
/*                                            resulting in version 6.1.2  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_entry(UX_HOST_CLASS_COMMAND *command)
{     

UINT                        status;
UX_HOST_CLASS               *class_inst;
#if !defined(UX_HOST_STANDALONE)
UX_HOST_CLASS_STORAGE_EXT   *class_ext;
#endif


    /* The command request will tell us we need to do here, either a enumeration
       query, an activation or a deactivation.  */
    switch (command -> ux_host_class_command_request)
    {

    case UX_HOST_CLASS_COMMAND_QUERY:

        /* The query command is used to let the stack enumeration process know if we want to own
           this device or not.  */
        if ((command -> ux_host_class_command_usage == UX_HOST_CLASS_COMMAND_USAGE_CSP) &&
                            (command -> ux_host_class_command_class == UX_HOST_CLASS_STORAGE_CLASS))
            return(UX_SUCCESS);                        
        else            
            return(UX_NO_CLASS_MATCH);                        
                
    case UX_HOST_CLASS_COMMAND_ACTIVATE:

        /* We are assuming the device will mount. If this is the first activation of
           the storage class, we have to fire up one thread for the media insertion
           and acquire some memory for the media array.  */

        /* Get class.  */
        class_inst = command -> ux_host_class_command_class_ptr;

#if !defined(UX_HOST_STANDALONE)

        /* Allocate UX_HOST_CLASS_STORAGE_EXT.  */
        if (class_inst -> ux_host_class_ext == UX_NULL)
        {

            /* Need memory for extension fields.  */
            class_ext = _ux_utility_memory_allocate(UX_NO_ALIGN,
                                            UX_REGULAR_MEMORY,
                                            sizeof(UX_HOST_CLASS_STORAGE_EXT));

            /* Check completion status.  */
            if (class_ext == UX_NULL)
                return(UX_MEMORY_INSUFFICIENT);

            /* Create the storage class thread.  */
            status =  _ux_host_thread_create(&class_ext -> ux_host_class_thread,
                                    "ux_host_storage_thread",
                                    _ux_host_class_storage_thread_entry,
                                    (ULONG) (ALIGN_TYPE) class_inst, 
                                    class_ext -> ux_host_class_thread_stack,
                                    UX_HOST_CLASS_STORAGE_THREAD_STACK_SIZE, 
                                    UX_HOST_CLASS_STORAGE_THREAD_PRIORITY_CLASS,
                                    UX_HOST_CLASS_STORAGE_THREAD_PRIORITY_CLASS,
                                    UX_NO_TIME_SLICE, UX_DONT_START);

            /* Check the completion status.  */
            if (status != UX_SUCCESS)
            {
                _ux_utility_memory_free(class_ext);
                class_inst -> ux_host_class_ext = UX_NULL;
                return(UX_THREAD_ERROR);
            }

            /* Set thead ext ptr.  */
            UX_THREAD_EXTENSION_PTR_SET(&(class_ext -> ux_host_class_thread), class_inst);

            /* Save extension.  */
            class_inst -> ux_host_class_ext = (VOID *)class_ext;
        }
        else
        {

            /* Get storage class extension.  */
            class_ext = (UX_HOST_CLASS_STORAGE_EXT *)class_inst -> ux_host_class_ext;
        }
#endif

        /* Allocate some memory for the media structures used by UX_MEDIA (default FileX).  */
        if (class_inst -> ux_host_class_media == UX_NULL)
        {

            /* UX_HOST_CLASS_STORAGE_MAX_MEDIA*sizeof(UX_HOST_CLASS_STORAGE_MEDIA) overflow
            * is checked outside of function.
            */
            class_inst -> ux_host_class_media =
                    _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY,
                            UX_HOST_CLASS_STORAGE_MAX_MEDIA*sizeof(UX_HOST_CLASS_STORAGE_MEDIA));

            /* Check the completion status.  */
            if (class_inst -> ux_host_class_media == UX_NULL)
                return(UX_MEMORY_INSUFFICIENT);
        }

        /* Now that the extension pointer has been set, resume the thread.  */
        _ux_host_thread_resume(&class_ext -> ux_host_class_thread);

        /* The activate command is used when the device inserted has found a parent and
           is ready to complete the enumeration.   */
        status =  _ux_host_class_storage_activate(command);

        /* Return the completion status.  */
        return(status);

    case UX_HOST_CLASS_COMMAND_ACTIVATE_WAIT:
        return(UX_STATE_NEXT);

    case UX_HOST_CLASS_COMMAND_DEACTIVATE:

        /* The deactivate command is used when the device has been extracted either      
           directly or when its parents has been extracted.  */
        status =  _ux_host_class_storage_deactivate(command);

        /* Return the completion status.  */
        return(status);

    case UX_HOST_CLASS_COMMAND_DESTROY:

        /* The destroy command is used when the class is unregistered.  */

        /* Get class.  */
        class_inst = command -> ux_host_class_command_class_ptr;

        /* Free allocated media structures.  */
        if (class_inst -> ux_host_class_media)
        {
            _ux_utility_memory_free(class_inst -> ux_host_class_media);
            class_inst -> ux_host_class_media = UX_NULL;
        }

#if !defined(UX_HOST_STANDALONE)

        /* Free class extension resources.  */
        if (class_inst -> ux_host_class_ext)
        {

            /* Get storage class extension.  */
            class_ext = (UX_HOST_CLASS_STORAGE_EXT *)class_inst -> ux_host_class_ext;

            /* Delete storage thread.  */
            _ux_host_thread_delete(&class_ext -> ux_host_class_thread);

            /* Free class extension memory.  */
            _ux_utility_memory_free(class_ext);

            /* Set extension pointer to NULL.  */
            class_inst -> ux_host_class_ext = UX_NULL;
        }
#endif

        /* Return success.  */
        return(UX_SUCCESS);

    default: 

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Return an error.  */
        return(UX_FUNCTION_NOT_SUPPORTED);
    }   
}

