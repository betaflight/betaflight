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
/**   HID Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_transfer_request_completed       PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called by the completion thread when a transfer    */ 
/*    request has been completed either because the transfer is           */ 
/*    successful or there was an error.                                   */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    transfer_request                      Pointer to transfer request   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    (ux_host_class_hid_report_callback_function)                        */ 
/*                                          Callback function for report  */ 
/*    _ux_host_class_hid_report_decompress  Decompress HID report         */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Release memory block          */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HID Class                                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_hid_transfer_request_completed(UX_TRANSFER *transfer_request)
{

UX_HOST_CLASS_HID                   *hid;
UX_HOST_CLASS_HID_CLIENT            *hid_client;
UX_HOST_CLASS_HID_REPORT            *hid_report;
UINT                                status;
VOID                                *report_buffer;
UX_HOST_CLASS_HID_REPORT_CALLBACK   callback;
UX_HOST_CLASS_HID_CLIENT_REPORT     client_report;
ULONG                               *client_buffer;
UX_HOST_CLASS_HID_FIELD             *hid_field;
ULONG                               field_report_count;

    /* Set Status to success. Optimistic view.  */
    status = UX_SUCCESS;

    /* Get the class instance for this transfer request.  */
    hid =  (UX_HOST_CLASS_HID *) transfer_request -> ux_transfer_request_class_instance;
    
    /* Check the state of the transfer.  If there is an error, we do not proceed with this report.  */
    if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* We have an error. We do not rehook another transfer if the device instance is shutting down or
           if the transfer was aborted by the class.  */
        if ((hid -> ux_host_class_hid_state ==  UX_HOST_CLASS_INSTANCE_SHUTDOWN) || 
            (transfer_request -> ux_transfer_request_completion_code == UX_TRANSFER_STATUS_ABORT))

            /* We do not proceed.  */
            return;

        else
        {            

            /* Reactivate the HID interrupt pipe.  */
            _ux_host_stack_transfer_request(transfer_request);
        
            /* We do not proceed.  */
            return;        
        }            
    }

    /* Get the client instance attached to the HID.  */
    hid_client =  hid -> ux_host_class_hid_client;

    /* Get the pointer to the report buffer in the transfer request.  */
    report_buffer =  transfer_request -> ux_transfer_request_data_pointer;

    /* We know this incoming report is for the Input report.  */
    hid_report =  hid -> ux_host_class_hid_parser.ux_host_class_hid_parser_input_report;

    /* initialize some of the callback structure which are generic to any
       reporting method.  */
    callback.ux_host_class_hid_report_callback_client =  hid_client;
    callback.ux_host_class_hid_report_callback_id =      hid_report -> ux_host_class_hid_report_id;
    
    /* For this report to be used, the HID client must have registered
       the report. We check the call back function.  */
    if (hid_report -> ux_host_class_hid_report_callback_function != UX_NULL)
    {

        /* The report is now in memory in a raw format the application may desire to handle it that way!  */
        if (hid_report -> ux_host_class_hid_report_callback_flags & UX_HOST_CLASS_HID_REPORT_RAW)
        {

            /* Put the length of the report in raw form in the callers callback structure.  */
            callback.ux_host_class_hid_report_callback_actual_length =  transfer_request -> ux_transfer_request_actual_length;
            callback.ux_host_class_hid_report_callback_buffer =         report_buffer;

            /* Build the callback structure status.  */
            callback.ux_host_class_hid_report_callback_status =  status;
    
            /* Set the flags to indicate the type of report.  */
            callback.ux_host_class_hid_report_callback_flags =  hid_report -> ux_host_class_hid_report_callback_flags;
            
            /* Call the report owner.  */
            hid_report -> ux_host_class_hid_report_callback_function(&callback);
        }
        else
        {

            /* The report may be decompressed, buffer length is based on number of items in report.
               Each item is a pair of words (usage and the value itself), so the required decompress memory for each
               item is 4 (word size) * 2 (number of word) = 8 bytes.
               To accelerate we shift number of item by 3 to get the result.  */
            if (UX_OVERFLOW_CHECK_MULC_ULONG(hid_report->ux_host_class_hid_report_number_item, 8))
                client_buffer = UX_NULL;
            else
            {
                client_report.ux_host_class_hid_client_report_length = hid_report->ux_host_class_hid_report_number_item << 3;

                /* We need to allocate some memory to build the decompressed report.  */
                client_buffer =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, client_report.ux_host_class_hid_client_report_length);
            }

            /* Check completion status.  */
            if (client_buffer == UX_NULL)
            {
                /* We have an error of memory, do not proceed */
                status =  UX_MEMORY_INSUFFICIENT;
            }
            else
            {

                /* We need to build a client structure to be used by the decompression engine.  */
                client_report.ux_host_class_hid_client_report_buffer =         client_buffer;
                client_report.ux_host_class_hid_client_report_actual_length =  0;
                client_report.ux_host_class_hid_client_report =                hid_report;

                /* The report buffer must be parsed and decompressed into the local buffer.  */
                _ux_host_class_hid_report_decompress(hid, &client_report, report_buffer, transfer_request -> ux_transfer_request_actual_length);

                /* The report may be decompressed and returned as individual Usages.  */
                if (hid_report -> ux_host_class_hid_report_callback_flags & UX_HOST_CLASS_HID_REPORT_INDIVIDUAL_USAGE)
                {

                    /* Now, we need to call the HID client call back function with a usage/value couple.  */
                    hid_field =  hid_report -> ux_host_class_hid_report_field;

                    /* Set the flags to indicate the type of report (usage/value couple).  */
                    callback.ux_host_class_hid_report_callback_flags =  hid_report -> ux_host_class_hid_report_callback_flags;

                    /* The length of the buffer is irrelevant here so we reset it.  */
                    callback.ux_host_class_hid_report_callback_actual_length =  0;

                    /* Scan all the fields and send each usage/value.   */
                    while(hid_field != UX_NULL)
                    {

                        /* Build each report item.  */
                        for (field_report_count = 0; field_report_count < hid_field -> ux_host_class_hid_field_report_count; field_report_count++)
                        {

                            /* Insert the usage and the report value into the callback structure.  */
                            callback.ux_host_class_hid_report_callback_usage =  *client_report.ux_host_class_hid_client_report_buffer++;
                            callback.ux_host_class_hid_report_callback_value =  *client_report.ux_host_class_hid_client_report_buffer++;

                            /* Build the callback structure status.  */
                            callback.ux_host_class_hid_report_callback_status =  status;

                            /* Call the report owner */
                            hid_report -> ux_host_class_hid_report_callback_function(&callback);
                        }

                        /* Get the next field.  */
                        hid_field =  hid_field -> ux_host_class_hid_field_next_field;
                    }
                }
                else
                {

                    /* Add the length actually valid in the caller's buffer.  */
                    callback.ux_host_class_hid_report_callback_actual_length =  client_report.ux_host_class_hid_client_report_actual_length;
        
                    /* Add the caller's buffer address.  */
                    callback.ux_host_class_hid_report_callback_buffer =  client_report.ux_host_class_hid_client_report_buffer;
            
                    /* Build the callback structure status.  */
                    callback.ux_host_class_hid_report_callback_status =  status;
        
                    /* Set the flags to indicate the type of report.  */
                    callback.ux_host_class_hid_report_callback_flags =  hid_report -> ux_host_class_hid_report_callback_flags;
                
                    /* Call the report owner.  */
                    hid_report -> ux_host_class_hid_report_callback_function(&callback);
                }

                /* Free the memory resource we used.  */
                _ux_utility_memory_free(client_buffer);
            }
        }
    }    

    /* Check latest status.  */
    if (status != UX_SUCCESS)

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);

    /* Reactivate the HID interrupt pipe.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check latest status.  */
    if (status != UX_SUCCESS)

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);

    /* Return to caller.  */
    return;
}

