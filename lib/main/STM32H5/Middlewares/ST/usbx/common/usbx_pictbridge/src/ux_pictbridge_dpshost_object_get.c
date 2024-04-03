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
/*    _ux_pictbridge_dpshost_object_get                   PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function obtains an object and execute its script.             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pictbridge                             Pictbridge instance          */ 
/*    object_handle                          Object handle                */ 
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
/*    _ux_pictbridge_dpshost_thread                                       */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed string length check,  */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_dpshost_object_get(UX_PICTBRIDGE *pictbridge, ULONG object_handle)
{
UINT                                status;
UX_HOST_CLASS_PIMA                  *pima;
UX_HOST_CLASS_PIMA_SESSION          *pima_session;
UX_HOST_CLASS_PIMA_OBJECT           *pima_object;
ULONG                               object_length;
ULONG                               requested_length;
ULONG                               actual_length;
UCHAR                               *object_buffer;
UCHAR                               object_file_name[UX_PICTBRIDGE_MAX_FILE_NAME_SIZE + 1]; /* +1 for null-terminator */
UINT                                length, length1;

    /* Get the pima instance from the pictbridge container.  */
    pima = (UX_HOST_CLASS_PIMA *) pictbridge -> ux_pictbridge_pima;
    
    /* And now the session since we use it.  */
    pima_session =  (UX_HOST_CLASS_PIMA_SESSION *) pictbridge -> ux_pictbridge_session;

    /* Get the address of the object container.  */
    pima_object =  (UX_HOST_CLASS_PIMA_OBJECT *) pictbridge -> ux_pictbridge_object_host;

    /* Get the object info structure.  */
    status = _ux_host_class_pima_object_info_get(pima, pima_session, 
                                             object_handle, pima_object);
    if (status != UX_SUCCESS)
    {

        /* Return an error.  */
        return(UX_PICTBRIDGE_ERROR_INVALID_OBJECT_HANDLE);
    }        

    /* Check if this is a script.  If the object is not a script or does not contain the 
       File Name we expect, we ignore it.  */        
    if (pima_object -> ux_host_class_pima_object_format == UX_HOST_CLASS_PIMA_OFC_SCRIPT)
    {

        /* Yes this is a script. We need to search for the DREQUEST.DPS file name.
           Get the file name length (with null-terminator).  */
        length1 = (UINT)(*pima_object -> ux_host_class_pima_object_filename);

        /* Check the file name from this script. It should be in the form DREQUEST.DPS.  */
        if (length1 <= UX_PICTBRIDGE_MAX_FILE_NAME_SIZE)
        {

            /* Invalidate length, on error it's untouched.  */
            length = UX_PICTBRIDGE_MAX_FILE_NAME_SIZE + 1;
            _ux_utility_string_length_check(_ux_pictbridge_drequest_name, &length, UX_PICTBRIDGE_MAX_FILE_NAME_SIZE);
            if ((length + 1) == length1)
            {

                /* Get the file name in a ascii format (with null-terminator). */
                _ux_utility_unicode_to_string(pima_object -> ux_host_class_pima_object_filename, object_file_name);
        
                /* So far, the length of name of the files are the same.
                Compare names now (since length is same just compare without null-terminator). */
                if (_ux_utility_memory_compare(_ux_pictbridge_drequest_name, object_file_name,
                                            length) ==  UX_SUCCESS)
                {

                    /* Yes this is a script. Get the entire object in memory.  */
                    /* Get script length from the object info.  */
                    object_length = pima_object -> ux_host_class_pima_object_compressed_size;
                    
                    /* Check for overflow.  */
                    if (object_length > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
                        return(UX_PICTBRIDGE_ERROR_SCRIPT_BUFFER_OVERFLOW);
                    
                    /* Open the object.  */
                    status = _ux_host_class_pima_object_open(pima, pima_session, object_handle, pima_object);
            
                    /* Check status.  */
                    if (status != UX_SUCCESS)
                        return(status);
                        
                    /* Set the object buffer pointer.  */
                    object_buffer = pima_object -> ux_host_class_pima_object_buffer;
                    
                    /* Obtain all the object data.  */
                    while(object_length != 0)
                    {            
                    
                        /* Calculate what length to request.  */
                        if (object_length > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
                            
                            /* Request maximum length.  */
                            requested_length = UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER;
                            
                        else
                                            
                            /* Request remaining length.  */
                            requested_length = object_length;
                            
                        /* Get the object data.  */
                        status = _ux_host_class_pima_object_get(pima, pima_session, object_handle, pima_object, 
                                                                object_buffer, requested_length, &actual_length);
                        if (status != UX_SUCCESS)
                        {
                    
                            /* We had a problem, abort the transfer.  */
                            _ux_host_class_pima_object_transfer_abort(pima, pima_session, object_handle, pima_object);
            
                            /* And close the object.  */
                            _ux_host_class_pima_object_close(pima, pima_session, object_handle, pima_object);
                    
                            return(status);
                        }            
                    
                        /* We have received some data, update the length remaining.  */
                        object_length -= actual_length;
            
                        /* Update the buffer address.  */
                        object_buffer += actual_length;
                    }    
                    
                    /* Close the object.  */
                    status = _ux_host_class_pima_object_close(pima, pima_session, object_handle, pima_object);
            
                    /* Check the error.  */
                    if (status != UX_SUCCESS)
                        return(UX_PICTBRIDGE_ERROR_INVALID_OBJECT_HANDLE);
            
                    /* We now have the entire script into memory. Parse the object and execute the script.  */
                    status = _ux_pictbridge_object_parse(pictbridge, pima_object -> ux_host_class_pima_object_buffer,
                                                pima_object -> ux_host_class_pima_object_compressed_size);
                    
                    /* Analyze the status from the parsing and determine the result code to be sent back.  */
                    switch (status)
                    {        
                        case UX_SUCCESS                                     :
            
                            /* Set the result code to OK.  */
                            pictbridge -> ux_pictbridge_operation_result =  UX_PICTBRIDGE_ACTION_RESULT_OK;
                            break;
            
                        case UX_PICTBRIDGE_ERROR_SCRIPT_SYNTAX_ERROR        :
            
                            /* Set the result code to Error.  */
                            pictbridge -> ux_pictbridge_operation_result =  UX_PICTBRIDGE_ACTION_RESULT_NOT_SUPPORTED_IP;
                            break;
                        
                        case UX_PICTBRIDGE_ERROR_PARAMETER_UNKNOWN          :
            
                            /* Set the result code to Error.  */
                            pictbridge -> ux_pictbridge_operation_result =  UX_PICTBRIDGE_ACTION_RESULT_NOT_SUPPORTED_UP;
                            break;
                        
                        case UX_PICTBRIDGE_ERROR_PARAMETER_MISSING          :
            
                            /* Set the result code to Error.  */
                            pictbridge -> ux_pictbridge_operation_result =  UX_PICTBRIDGE_ACTION_RESULT_NOT_SUPPORTED_MP;
                            break;
                        
                        default :
            
                            /* Set the result code to Error.  */
                            pictbridge -> ux_pictbridge_operation_result =  UX_PICTBRIDGE_ACTION_RESULT_NOT_SUPPORTED_DEFAULT;
                            break;
                    }            
            
                    /* We have completed the parsing of the XML object.  We will send an object back to the client. 
                    Generate the output report.  */
                    status = _ux_pictbridge_dpshost_output_object_create(pictbridge);
            
                    /* Get the length of the object and store it into the object data set.  */
                    pima_object -> ux_host_class_pima_object_compressed_size = pima_object -> ux_host_class_pima_object_length;

                    /* We need to change the file name to DRSPONSE.DPS. Encode the file name from ascii format into Unicode.  */
                    _ux_utility_string_to_unicode(_ux_pictbridge_hrsponse_name, pima_object -> ux_host_class_pima_object_filename);

                    /* Send a script info.  */
                    status = _ux_host_class_pima_object_info_send(pima, pima_session, 0, 0, pima_object);
                    if (status != UX_SUCCESS)
                        return(status);
            
                    /* Open the object.  */
                    status = _ux_host_class_pima_object_open(pima, pima_session, 
                                                            object_handle,
                                                            pima_object);
                    if (status != UX_SUCCESS)
                        return(status);
                    
                    /* Get the object length.  */
                    object_length =  pima_object -> ux_host_class_pima_object_compressed_size;
                    
                    /* Recall the object buffer address.  */
                    object_buffer = pima_object -> ux_host_class_pima_object_buffer;
            
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
                                                                    object_buffer, requested_length);
                        if (status != UX_SUCCESS)
                        {
                    
                            /* Abort the transfer.  */
                            _ux_host_class_pima_object_transfer_abort(pima, pima_session, object_handle, pima_object);
                    
                            return(status);
                        }            
                    
                        /* We have sent some data, update the length remaining.  */
                    object_length -= requested_length;
                    }    
                    
                    /* Close the object.  */
                    status = _ux_host_class_pima_object_close(pima, pima_session, object_handle, pima_object);
            
                    /* Check the status.  */
                    if (status != UX_SUCCESS)
                        return(status);
                }
            }            
        }
    }
        
    return(UX_SUCCESS);
}

