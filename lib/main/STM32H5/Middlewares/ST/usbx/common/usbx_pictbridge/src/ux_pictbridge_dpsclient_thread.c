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
/**   Pictbridge Application                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_pictbridge.h"
#include "ux_device_class_pima.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_dpsclient_thread                     PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This is the Pictbridge dpsclient thread that receives and execute   */ 
/*    commands from the dpshost.                                          */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pictbridge                             Pictbridge instance          */ 
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
/*    user application                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases, used UX prefix to    */
/*                                            refer to TX symbols instead */
/*                                            of using them directly,     */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used macros for RTOS calls, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_pictbridge_dpsclient_thread(ULONG parameter)
{

UX_PICTBRIDGE                       *pictbridge;
ULONG                               actual_flags;
UINT                                status = UX_SUCCESS;
ULONG                               object_length;
UCHAR                               *pima_object_buffer;
UX_SLAVE_CLASS_PIMA                 *pima;
UX_SLAVE_CLASS_PIMA_OBJECT          *pima_object;

    /* Cast the parameter passed in the thread into the pictbridge pointer.  */
    UX_THREAD_EXTENSION_PTR_GET(pictbridge, UX_PICTBRIDGE, parameter)

    /* Loop forever waiting for changes signaled through the semaphore. */     
    while (1)
    {

        /* Report errors via error handler.  */
        if (status != UX_SUCCESS)
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);

        /* We should wait for the host to send a script with a command.  */
        status =  _ux_system_event_flags_get(&pictbridge -> ux_pictbridge_event_flags_group, (UX_PICTBRIDGE_EVENT_FLAG_NOTIFY_JOB_STATUS | 
                                                                                                UX_PICTBRIDGE_EVENT_FLAG_NOTIFY_DEVICE_STATUS), 
                                                                                                UX_OR_CLEAR, &actual_flags, UX_PICTBRIDGE_EVENT_TIMEOUT);

        /* Check the status.  */
        if (status == UX_SUCCESS)
        {

            /* The event flag is meaningful. What did we get ? */
            if ((actual_flags & UX_PICTBRIDGE_EVENT_FLAG_NOTIFY_JOB_STATUS) || (actual_flags & UX_PICTBRIDGE_EVENT_FLAG_NOTIFY_DEVICE_STATUS))
            {
        
                /* We need to send a response now.  */
                /* Get the pima instance from the pictbridge container.  */
                pima = (UX_SLAVE_CLASS_PIMA *) pictbridge -> ux_pictbridge_pima;
                
                /* Get the address of the object container.  We need the host container here, as it is a response for the host.  */
                pima_object =  (UX_SLAVE_CLASS_PIMA_OBJECT *) pictbridge -> ux_pictbridge_object_host;
            
                /* Get the object buffer address.  */
                pima_object_buffer = pima_object -> ux_device_class_pima_object_buffer;
                
                /* Reset the object length.  */
                object_length =  0;
                
                /* Clear the object memory buffer.  */
                _ux_utility_memory_set(pima_object_buffer, 0, UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER); /* Use case of memset is verified. */
            
                /* Add the line <?xml version="1.0"?>  */
                status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_xmlversion, 
                                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                            UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
                if (status != UX_SUCCESS)
                    continue;
            
                /* Add the line <dps xmlns="http://www.cipa.jp/dps/schema/">  */
                status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_dpsxmlns, 
                                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                            UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
                if (status != UX_SUCCESS)
                    continue;
            
                /* Add the line <output>  */
                status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_output, 
                                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                            UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
                if (status != UX_SUCCESS)
                    continue;

                /* Add the line <result> xxxxxxxx </result> */
                status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_result, 
                                                        UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_END | UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA,
                                                        UX_NULL, 0, (VOID *)(ALIGN_TYPE) pictbridge -> ux_pictbridge_operation_result, &pima_object_buffer, &object_length);
                if (status != UX_SUCCESS)
                    continue;

                /* Check what function was demanded.  */
                if (actual_flags & UX_PICTBRIDGE_EVENT_FLAG_NOTIFY_JOB_STATUS)
                
                    /* Add the line <notifyJobStatus/> */
                    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_notifyjobstatus, 
                                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END,
                                                            UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
                
                else
                
                
                    /* Add the line <notifyDeviceStatus/> */
                    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_notifydevicestatus, 
                                                            UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END,
                                                            UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
                if (status != UX_SUCCESS)
                    continue;

                /* Add the line </output>  */
                status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_output, 
                                                UX_PICTBRIDGE_TAG_FLAG_END,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
                if (status != UX_SUCCESS)
                    continue;

                /* Add the line </dps>  */
                status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_dps, 
                                                            UX_PICTBRIDGE_TAG_FLAG_END,
                                                            UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
                if (status != UX_SUCCESS)
                    continue;

                /* Get the length of the object and store it into the object data set.  */
                pima_object -> ux_device_class_pima_object_length = object_length;         
                pima_object -> ux_device_class_pima_object_compressed_size = object_length;         
                
                /* Save the handle for this object.  */
                pima_object -> ux_device_class_pima_object_handle_id =  UX_PICTBRIDGE_OBJECT_HANDLE_HOST_RESPONSE;         
                
                /* We need to change the file name to DRSPONSE.DPS. Encode the file name from ascii format into Unicode.  */
                _ux_utility_string_to_unicode(_ux_pictbridge_drsponse_name, pima_object -> ux_device_class_pima_object_filename);
            
                /* Send the notification to the host that an object has been added.  */
                status = _ux_device_class_pima_object_add(pima, UX_PICTBRIDGE_OBJECT_HANDLE_HOST_RESPONSE);
                if (status != UX_SUCCESS)
                    continue;

                /* Check if user registered callback function is present.  */
                if (pictbridge -> ux_pictbridge_dps_event_callback_function != UX_NULL)
                {

                    /* Call the user callback function.  */
                    (pictbridge -> ux_pictbridge_dps_event_callback_function)(pictbridge, actual_flags);
                }

            }
        }
    }    
}

