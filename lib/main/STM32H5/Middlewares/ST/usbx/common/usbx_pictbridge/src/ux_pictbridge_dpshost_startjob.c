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
/*    _ux_pictbridge_dpshost_startjob                     PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function requests one or more files from the camera to be      */ 
/*    printed.                                                            */ 
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
/*    _ux_pictbridge_dpshost_thread                                       */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  04-25-2022     Yajun Xia                Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_dpshost_startjob(UX_PICTBRIDGE *pictbridge)
{
UINT                                status;
UX_PICTBRIDGE_JOBINFO               *jobinfo;
UX_PICTBRIDGE_PRINTINFO             *printinfo;
UX_HOST_CLASS_PIMA                  *pima;
UX_HOST_CLASS_PIMA_OBJECT           pima_object;
UX_HOST_CLASS_PIMA_SESSION          *pima_session;
ULONG                               object_length;
ULONG                               requested_length;
ULONG                               actual_length;
ULONG                               object_offset;
UCHAR                               *object_buffer;

    /* We need the pima instance.  */
    pima = (UX_HOST_CLASS_PIMA *) pictbridge -> ux_pictbridge_pima;

    /* Get the session in a properly casted format.  */
    pima_session = (UX_HOST_CLASS_PIMA_SESSION *) pictbridge -> ux_pictbridge_session;
    
    /* We have 2 structures that tell us about the job(s) to do. The jobinfo which
       applies to all the jobs and one or more instances of the printinfo structure
       which applies to each file\object to be printed.  */
    jobinfo = &pictbridge -> ux_pictbridge_jobinfo;

    /* Get the first printinfo structure.  */
    printinfo =  jobinfo -> ux_pictbridge_jobinfo_printinfo_start;
    
    /* Parse all the job info.  */
    while (printinfo != UX_NULL)
    {
        /* Set the current print job.  */
        jobinfo -> ux_pictbridge_jobinfo_printinfo_current = printinfo;        

        /* Change status from idle to printing.  */
        pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_dpsprintservicestatus =  UX_PICTBRIDGE_DPS_PRINTSERVICE_STATUS_ACTIVE;

        /* Change status from disconnect OK to disconnect NOK.  */
        pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_disconnectenable =  UX_PICTBRIDGE_DISCONNECT_ENABLE_FALSE;

        /* Change NewJob status to cannot print.  */
        pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_newjobok = UX_PICTBRIDGE_NEW_JOB_FALSE;

        /* First issue a DPS_notifyDeviceStatus with state change from idle to printing.  */
        status = _ux_pictbridge_dpshost_input_object_send(pictbridge, UX_PICTBRIDGE_IR_NOTIFY_DEVICE_STATUS);

        /* Check status.  */
        if (status != UX_SUCCESS)
            return(status);

        /* Set the print info values in the notification xml request .  */
        printinfo -> ux_pictbridge_printinfo_current_page =  1;
        printinfo -> ux_pictbridge_printinfo_total_page =  1;
        printinfo -> ux_pictbridge_printinfo_images_printed =  0;

        /* Then issue a DPS_notifyJobStatus.  */
        status = _ux_pictbridge_dpshost_input_object_send(pictbridge, UX_PICTBRIDGE_IR_NOTIFY_JOB_STATUS);

        /* Check status.  */
        if (status != UX_SUCCESS)
            return(status);

        /* Get the object info for this fileID to be printed.  */
        status = _ux_host_class_pima_object_info_get(pima, pima_session, 
                                                printinfo -> ux_pictbridge_printinfo_fileid, &pima_object);
        if (status != UX_SUCCESS)
        {

            /* Change status from printing to idle.  */
            pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_dpsprintservicestatus =  UX_PICTBRIDGE_DPS_PRINTSERVICE_STATUS_IDLE;

            /* Change NewJob status to print ok.  */
            pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_newjobok = UX_PICTBRIDGE_NEW_JOB_TRUE;

            return(status);        
        }

        /* Get the object length from the object info.  */
        object_length = pima_object.ux_host_class_pima_object_compressed_size;
        
        /* Open the object.  */
         status = _ux_host_class_pima_object_open(pima, pima_session, 
                                                printinfo -> ux_pictbridge_printinfo_fileid, &pima_object);
         if (status != UX_SUCCESS)
         {

            /* Change status from printing to idle.  */
            pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_dpsprintservicestatus =  UX_PICTBRIDGE_DPS_PRINTSERVICE_STATUS_IDLE;

            /* Change NewJob status to print ok.  */
            pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_newjobok = UX_PICTBRIDGE_NEW_JOB_TRUE;
            

            return(status);
         }

        /* Allocate memory for the object.  */
        object_buffer =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER);

        /* Check for memory error.  */
        if (object_buffer == UX_NULL)
        {
        
            /* Change status from printing to idle.  */
            pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_dpsprintservicestatus =  UX_PICTBRIDGE_DPS_PRINTSERVICE_STATUS_IDLE;

            /* Change NewJob status to print ok.  */
            pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_newjobok = UX_PICTBRIDGE_NEW_JOB_TRUE;

            
            /* Close the object.  */
            _ux_host_class_pima_object_close(pima, pima_session,
                                                printinfo -> ux_pictbridge_printinfo_fileid, &pima_object);
            /* Return an error.  */
             return(UX_MEMORY_INSUFFICIENT);
        
        }

        /* Reset the current object offset.  */
        object_offset = 0;
        
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
             status = _ux_host_class_pima_object_get(pima, pima_session, printinfo -> ux_pictbridge_printinfo_fileid, &pima_object, 
                                                            object_buffer, requested_length, &actual_length);
             if (status != UX_SUCCESS)
             {
                        
                /* Abort the current transfer.  */                    
                _ux_host_class_pima_object_transfer_abort(pima, pima_session, printinfo -> ux_pictbridge_printinfo_fileid, &pima_object);

                /* Change status from printing to idle.  */
                pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_dpsprintservicestatus =  UX_PICTBRIDGE_DPS_PRINTSERVICE_STATUS_IDLE;

                /* Change NewJob status to print ok.  */
                pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_newjobok = UX_PICTBRIDGE_NEW_JOB_TRUE;


                /* Free memory of the object.  */
                _ux_utility_memory_free(object_buffer);
        
                /* Return error.  */
                return(status);
             }            
        
            /* If there is an application callback function, associated with this object, call it.  */
            if (pictbridge -> ux_pictbridge_application_object_data_write != UX_NULL)
            {
            
                /* Call the callback function.  */
                status = pictbridge -> ux_pictbridge_application_object_data_write(pictbridge, 
                                                                                    object_buffer, 
                                                                                    object_offset, 
                                                                                    pima_object.ux_host_class_pima_object_compressed_size, 
                                                                                    actual_length);

                if (status != UX_SUCCESS)
                {

                    /* Abort the current transfer.  */
                    _ux_host_class_pima_object_transfer_abort(pima, pima_session, printinfo -> ux_pictbridge_printinfo_fileid, &pima_object);

                    /* Change status from printing to idle.  */
                    pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_dpsprintservicestatus =  UX_PICTBRIDGE_DPS_PRINTSERVICE_STATUS_IDLE;

                    /* Change NewJob status to print ok.  */
                    pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_newjobok = UX_PICTBRIDGE_NEW_JOB_TRUE;


                    /* Free memory of the object.  */
                    _ux_utility_memory_free(object_buffer);

                    /* Return error.  */
                    return(status);
                }
            }

             /* We have received some data, update the length remaining.  */
             object_length -= actual_length;
             
             /* Update the offset.  */
             object_offset += actual_length;
         }    
        
         /* Close the object.  */
         status = _ux_host_class_pima_object_close(pima, pima_session,
                                                printinfo -> ux_pictbridge_printinfo_fileid, &pima_object);
        /* Check error code.  */
        if (status != UX_SUCCESS)
        {

            /* Change status from printing to idle.  */
            pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_dpsprintservicestatus =  UX_PICTBRIDGE_DPS_PRINTSERVICE_STATUS_IDLE;

            /* Change NewJob status to print ok.  */
            pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_newjobok = UX_PICTBRIDGE_NEW_JOB_TRUE;

        
            /* Free memory of the object.  */
            _ux_utility_memory_free(object_buffer);

            return(status);
        }
        
        /* Free resources.  */
        _ux_utility_memory_free(object_buffer);

        /* Set the print info values in the notification xml request .  */
        printinfo -> ux_pictbridge_printinfo_current_page =  1;
        printinfo -> ux_pictbridge_printinfo_total_page =  1;
        printinfo -> ux_pictbridge_printinfo_images_printed++;

        /* First issue a DPS_notifyJobStatus.  */
        status = _ux_pictbridge_dpshost_input_object_send(pictbridge, UX_PICTBRIDGE_IR_NOTIFY_JOB_STATUS);

        /* Change status from printing to idle.  */
        pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_dpsprintservicestatus =  UX_PICTBRIDGE_DPS_PRINTSERVICE_STATUS_IDLE;

        /* Change status from disconnect NOK to disconnect OK.  */
        pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_disconnectenable =  UX_PICTBRIDGE_DISCONNECT_ENABLE_TRUE;

        /* Change NewJob status to can print.  */
        pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_newjobok = UX_PICTBRIDGE_NEW_JOB_TRUE;

        /* Change JobEndReason status to Job ended normally.  */
        pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_jobendreason = UX_PICTBRIDGE_JOB_END_REASON_NORMAL;

        /* First issue a DPS_notifyDeviceStatus with state change from idle to printing.  */
        status = _ux_pictbridge_dpshost_input_object_send(pictbridge, UX_PICTBRIDGE_IR_NOTIFY_DEVICE_STATUS);

        /* Check status.  */
        if (status != UX_SUCCESS)
            return(status);

        /* Next printinfo.  */
        printinfo = printinfo -> ux_pictbridge_printinfo_next;
                
    }

    /* Return status successful.  */
    return(UX_SUCCESS);
           
}

