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
#include "ux_host_class_pima.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_dpshost_input_object_send            PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates an input report for the dpshost               */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pictbridge                             Pictbridge instance          */ 
/*    input_function                         input tag                    */ 
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
/*                                                                        */ 
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
/*                                            cleared compile warning,    */
/*                                            used macros for RTOS calls, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_dpshost_input_object_send(UX_PICTBRIDGE *pictbridge, ULONG input_function)
{
UINT                                status;
ULONG                               object_length;
UCHAR                               *pima_object_buffer;
UX_HOST_CLASS_PIMA                  *pima;
UX_HOST_CLASS_PIMA_SESSION          *pima_session;
UX_HOST_CLASS_PIMA_OBJECT           *pima_object;
ULONG                               requested_length;
ULONG                               object_handle;
ULONG                               actual_flags;

    /* Check the state machine.  */
    if (pictbridge -> ux_pictbridge_host_client_state_machine != UX_PICTBRIDGE_STATE_MACHINE_IDLE)
    {
        
        /* Set the state machine to Host Request pending.  */
        pictbridge -> ux_pictbridge_host_client_state_machine |= UX_PICTBRIDGE_STATE_MACHINE_HOST_REQUEST_PENDING;

        /* Wait for the client event pending request to be completed.  */
        status =  _ux_system_event_flags_get(&pictbridge -> ux_pictbridge_event_flags_group, 
                                        UX_PICTBRIDGE_EVENT_FLAG_STATE_MACHINE_READY, 
                                        UX_AND_CLEAR, &actual_flags, UX_PICTBRIDGE_EVENT_TIMEOUT);

        /* Reset the state machine to not Host Request pending.  */
        pictbridge -> ux_pictbridge_host_client_state_machine &= (UINT)~UX_PICTBRIDGE_STATE_MACHINE_HOST_REQUEST_PENDING;

        /* Check status.  */
        if (status != UX_SUCCESS)
            return(UX_ERROR);

        /* Status good means flag match, no need to check variable again, mark it unused.  */
        (void)actual_flags;
    }

    /* Change state machine to host request pending.  */
    pictbridge -> ux_pictbridge_host_client_state_machine |=  UX_PICTBRIDGE_STATE_MACHINE_HOST_REQUEST;

    /* Get the pima instance from the pictbridge container.  */
    pima = (UX_HOST_CLASS_PIMA *) pictbridge -> ux_pictbridge_pima;
    
    /* And now the session since we use it.  */
    pima_session =  (UX_HOST_CLASS_PIMA_SESSION *)pictbridge -> ux_pictbridge_session;

    /* Get the address of the object container.  */
    pima_object =  (UX_HOST_CLASS_PIMA_OBJECT *) pictbridge -> ux_pictbridge_object_host;

    /* Get the object buffer address.  */
    pima_object_buffer = pima_object -> ux_host_class_pima_object_buffer;
    
    /* Reset the object length.  */
    object_length =  0;
    
    /* Clear the object memory buffer.  */
    _ux_utility_memory_set(pima_object_buffer, 0, UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER); /* Use case of memset is verified. */

    /* Add the line <?xml version="1.0"?>  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_xmlversion, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    if (status != UX_SUCCESS)
        return(status);

    /* Add the line <dps xmlns="http://www.cipa.jp/dps/schema/">  */
    status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_dpsxmlns, 
                                                UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);

    /* Add the line <input>  */
    if (status == UX_SUCCESS)
    {
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_input, 
                                                    UX_PICTBRIDGE_TAG_FLAG_BEGIN | UX_PICTBRIDGE_TAG_FLAG_FORCE_LF,
                                                    UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    }

    /* Look into the tag code from the input object and proceed to individual functions.  */
    if (status == UX_SUCCESS)
    {
        switch (input_function) 
        {
                    
            case UX_PICTBRIDGE_IR_NOTIFY_JOB_STATUS                    :

                /* Insert the notifyJobStatus tag lines.  */
                status = _ux_pictbridge_dpshost_input_object_notify_job_status(pictbridge, pima_object_buffer, object_length, &pima_object_buffer, &object_length);
                break;

            case UX_PICTBRIDGE_IR_NOTIFY_DEVICE_STATUS                 :

                /* Insert the notifyDeviceStatus tag lines.  */
                status = _ux_pictbridge_dpshost_input_object_notify_device_status(pictbridge, pima_object_buffer, object_length, &pima_object_buffer, &object_length);
                break;

            default                                                    :

                /* Function not yet supported.  We should never come here anyway. */
                status = UX_ERROR;

        }    
    }

    /* Add the line </input>  */
    if (status == UX_SUCCESS)
    {
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_input, 
                                                    UX_PICTBRIDGE_TAG_FLAG_END,
                                                    UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    }

    /* Add the line </dps>  */
    if (status == UX_SUCCESS)
    {
        status = _ux_pictbridge_object_tag_line_add(pima_object_buffer, object_length, _ux_pictbridge_xml_tag_line_dps, 
                                                    UX_PICTBRIDGE_TAG_FLAG_END,
                                                    UX_NULL, 0, UX_NULL, &pima_object_buffer, &object_length);
    }

    /* Handle error.  */
    if (status != UX_SUCCESS)
    {

        /* Do we have a pending client event ?  */
        if (pictbridge -> ux_pictbridge_host_client_state_machine & UX_PICTBRIDGE_STATE_MACHINE_CLIENT_REQUEST_PENDING)
        {

            /* Yes, so we need to set the event that advertise the completion of the host request.  */
            _ux_system_event_flags_set(&pictbridge -> ux_pictbridge_event_flags_group,
                                    UX_PICTBRIDGE_EVENT_FLAG_STATE_MACHINE_READY, 
                                    UX_AND);

        }

        /* Change state machine to idle.  */
        pictbridge -> ux_pictbridge_host_client_state_machine &=  (UINT)~UX_PICTBRIDGE_STATE_MACHINE_HOST_REQUEST;
        return(UX_ERROR);    
    }

    /* Store the length of the new object waiting to be sent out.  
       We do not store the length into the object itself since this function is
       host\client agnostic.  */
    pima_object -> ux_host_class_pima_object_length =  object_length;

    /* Get the length of the object and store it into the object data set.  */
    pima_object -> ux_host_class_pima_object_compressed_size = object_length;

    /* We need to change the file name to HREQUEST.DPS. Encode the file name from ascii format into Unicode.  */
    _ux_utility_string_to_unicode(_ux_pictbridge_hrequest_name, pima_object -> ux_host_class_pima_object_filename);

    /* Set the object handle.  */
    object_handle = 1;
    
    /* Send a script info.  */
    status = _ux_host_class_pima_object_info_send(pima, pima_session, 0, 0, pima_object);
    if (status != UX_SUCCESS)
    {
        
        /* Do we have a pending client event ?  */
        if (pictbridge -> ux_pictbridge_host_client_state_machine & UX_PICTBRIDGE_STATE_MACHINE_CLIENT_REQUEST_PENDING)
        {

            /* Yes, so we need to set the event that advertise the completion of the host request.  */
            _ux_system_event_flags_set(&pictbridge -> ux_pictbridge_event_flags_group, 
                                    UX_PICTBRIDGE_EVENT_FLAG_STATE_MACHINE_READY, 
                                    UX_AND);

        }
        
        /* Change state machine to idle.  */
        pictbridge -> ux_pictbridge_host_client_state_machine &=  (UINT)~UX_PICTBRIDGE_STATE_MACHINE_HOST_REQUEST;

        /* Return status.  */
        return(status);    
    }



    
    /* Open the object.  */
    status = _ux_host_class_pima_object_open(pima, pima_session, 
                                             object_handle,
                                             pima_object);
    if (status != UX_SUCCESS)
    {
        
        /* Do we have a pending client event ?  */
        if (pictbridge -> ux_pictbridge_host_client_state_machine & UX_PICTBRIDGE_STATE_MACHINE_CLIENT_REQUEST_PENDING)
        {

            /* Yes, so we need to set the event that advertise the completion of the host request.  */
            _ux_system_event_flags_set(&pictbridge -> ux_pictbridge_event_flags_group, 
                                    UX_PICTBRIDGE_EVENT_FLAG_STATE_MACHINE_READY, 
                                    UX_AND);

        }

        /* Change state machine to idle.  */
        pictbridge -> ux_pictbridge_host_client_state_machine &=  (UINT)~UX_PICTBRIDGE_STATE_MACHINE_HOST_REQUEST;

        /* Return status.  */
        return(status);    
    }
    
    /* Get the object length.  */
    object_length =  pima_object -> ux_host_class_pima_object_compressed_size;
    
    /* Recall the object buffer address.  */
    pima_object_buffer = pima_object -> ux_host_class_pima_object_buffer;
    
    /* Send all the object data.  */
    while(object_length != 0)
    {            
    
        /* Calculate what length to request.  */
        if (object_length > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
            
            /* Request maximum length.  */
            requested_length = UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER;
            
        else
                            
            /* Request remaining length.  */
            requested_length = object_length;
            
        
        /* Send the object data.  */
        status = _ux_host_class_pima_object_send(pima, pima_session, pima_object, 
                                                    pima_object_buffer, requested_length);
        if (status != UX_SUCCESS)
        {
    
            /* Abort the transfer.  */
            _ux_host_class_pima_object_transfer_abort(pima, pima_session, object_handle, pima_object);

            /* Do we have a pending client event ?  */
            if (pictbridge -> ux_pictbridge_host_client_state_machine & UX_PICTBRIDGE_STATE_MACHINE_CLIENT_REQUEST_PENDING)
            {

                /* Yes, so we need to set the event that advertise the completion of the host request.  */
                _ux_system_event_flags_set(&pictbridge -> ux_pictbridge_event_flags_group, 
                                        UX_PICTBRIDGE_EVENT_FLAG_STATE_MACHINE_READY, 
                                        UX_AND);

            }

            /* Change state machine to idle.  */
            pictbridge -> ux_pictbridge_host_client_state_machine &=  (UINT)~UX_PICTBRIDGE_STATE_MACHINE_HOST_REQUEST;

            /* Return status.  */
            return(status);    
            
        }            
    
        /* We have sent some data, update the length remaining.  */
    object_length -= requested_length;
    }    
    
    /* Close the object.  */
    status = _ux_host_class_pima_object_close(pima, pima_session, object_handle, pima_object);
    
    /* Check the status.  */
    if (status != UX_SUCCESS)
    {
        
        /* Do we have a pending client event ?  */
        if (pictbridge -> ux_pictbridge_host_client_state_machine & UX_PICTBRIDGE_STATE_MACHINE_CLIENT_REQUEST_PENDING)
        {

            /* Yes, so we need to set the event that advertise the completion of the host request.  */
            _ux_system_event_flags_set(&pictbridge -> ux_pictbridge_event_flags_group, 
                                    UX_PICTBRIDGE_EVENT_FLAG_STATE_MACHINE_READY, 
                                    UX_AND);

        }

        /* Change state machine to idle.  */
        pictbridge -> ux_pictbridge_host_client_state_machine &=  (UINT)~UX_PICTBRIDGE_STATE_MACHINE_HOST_REQUEST;

        /* Return status.  */
        return(status);    
    }


    /* Wait for the response from the device.  */
    status = _ux_pictbridge_dpshost_response_get(pictbridge);
    
        
    /* Do we have a pending client event ?  */
    if (pictbridge -> ux_pictbridge_host_client_state_machine & UX_PICTBRIDGE_STATE_MACHINE_CLIENT_REQUEST_PENDING)
    {

        /* Yes, so we need to set the event that advertise the completion of the host request.  */
        _ux_system_event_flags_set(&pictbridge -> ux_pictbridge_event_flags_group, 
                                UX_PICTBRIDGE_EVENT_FLAG_STATE_MACHINE_READY, 
                                UX_AND);

    }

    /* Change state machine to idle.  */
    pictbridge -> ux_pictbridge_host_client_state_machine &=  (UINT)~UX_PICTBRIDGE_STATE_MACHINE_HOST_REQUEST;

    /* Return status.  */
    return(status);    
    
}

