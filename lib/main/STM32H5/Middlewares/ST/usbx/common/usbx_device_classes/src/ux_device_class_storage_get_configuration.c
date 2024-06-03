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
/**   Device Storage Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_storage.h"
#include "ux_device_stack.h"

#define USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_PROFILE_LENGTH                          228
#define USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_HEADER_LENGTH                           8
#define USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_FEATURE_DESCRIPTOR_LENGTH               32
#define USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_FEATURE_HEADER_LENGTH                   4
#define USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_FEATURE_LENGTH (USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_PROFILE_LENGTH - USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_FEATURE_DESCRIPTOR_LENGTH - USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_HEADER_LENGTH)
UCHAR usbx_device_class_storage_configuration_profile[] = { 

    /* Feature Header */
        0x00, 0x00,                     /* Entire length of profile filled by firmware */
        0x00, 0x00,                  
        0x00, 0x00,            
        0x00, 0x00,                     /* Current profile */                           

    /* Feature Descriptor */
        0x00, 0x00,                     /* Feature Code : profile list */
        0x00,                           /* Persistent/current */  
        0x1c,                           /* Additional Length */                           
        
        0x00, 0x12, 0x00, 0x00,         /* DVD-RAM                                  */
        0x00, 0x11, 0x00, 0x00,         /* DVD-R                                    */
        0x00, 0x10, 0x00, 0x00,         /* DVD-ROM                                  */
        0x00, 0x0A, 0x00, 0x00,         /* CD-RW                                    */
        0x00, 0x09, 0x00, 0x00,         /* CD-R                                     */
        0x00, 0x08, 0x00, 0x00,         /* CD-ROM                                   */
        0x00, 0x02, 0x00, 0x00,         /* Writable capable with removable media.   */
        

    /* Feature Descriptor */
        0x00, 0x01,                     /* Feature Code : core feature */
        0x0b,                           /* Persistent/current */  
        0x08,                           /* Additional Length */                           
        
        0x00, 0x00, 0x00, 0x07,         /* Physical Interface Standard              */
        0x01, 0x00, 0x00, 0x00,         /*                                          */

    /* Feature Descriptor */
        0x00, 0x02,                     /* Feature Code : morphing (Ability to notify initiator about 
                                                          operational changes and accept initiator requests to prevent 
                                                          operational changes) */
        0x07,                           /* Persistent/current */  
        0x04,                           /* Additional Length */                           
        
        0x02, 0x00, 0x00, 0x00,         /* Physical Interface Standard              */

    /* Feature Descriptor */
        0x00, 0x03,                     /* Feature Code : Removable Medium (The medium may be removed 
                                                          from the device ) */
        0x0b,                           /* Persistent/current */  
        0x04,                           /* Additional Length */                           
        
        0x2b, 0x00, 0x00, 0x00,         /* Physical Interface Standard              */

    /* Feature Descriptor */
        0x00, 0x10,                     /* Feature Code : Random Readable (Read ability for storage devices 
                                                          with random addressing) */
        0x00,                           /* Persistent/current */  
        0x08,                           /* Additional Length */                           
        
        0x00, 0x00, 0x08, 0x00,         /*                                          */
        0x00, 0x01, 0x01, 0x00,         /*                                          */

    /* Feature Descriptor */
        0x00, 0x1d,                     /* Feature Code : MultiRead (The logical unit can read all CD media types) */
        0x00,                           /* Persistent/current */  
        0x00,                           /* Additional Length */                           


    /* Feature Descriptor */
        0x00, 0x1e,                     /* Feature Code : CD Read (The ability to read CD specific structures) */
        0x08,                           /* Persistent/current */  
        0x04,                           /* Additional Length */                           
        
        0x03, 0x00, 0x00, 0x00,         /*                                          */

    /* Feature Descriptor */
        0x00, 0x1f,                     /* Feature Code : DVD Read (The ability to read DVD specific structures) */
        0x08,                           /* Persistent/current */  
        0x04,                           /* Additional Length */                           
        
        0x01, 0x00, 0x01, 0x00,         /*                                          */

    /* Feature Descriptor */
        0x00, 0x20,                     /* Feature Code : Random Writable (Write support for randomly addressed writes) */
        0x04,                           /* Persistent/current */  
        0x0c,                           /* Additional Length */                           
        
        0x00, 0x00, 0x00, 0x00,         /*                                          */
        0x00, 0x00, 0x08, 0x00,         /*                                          */
        0x00, 0x00, 0x01, 0x00,         /*                                          */


    /* Feature Descriptor */
        0x00, 0x21,                     /* Feature Code : Incremental Streaming Writable (Write support for sequential recording) */
        0x0C,                           /* Persistent/current */  
        0x08,                           /* Additional Length */                           
        
        0x00, 0x00, 0x00, 0x00,         /*                                          */
        0x00, 0x00, 0x00, 0x00,         /*                                          */

    /* Feature Descriptor */
        0x00, 0x23,                     /* Feature Code : Formattable (Support for formatting of media.) */
        0x08,                           /* Persistent/current */  
        0x08,                           /* Additional Length */                           
        
        0x00, 0x00, 0x00, 0x00,         /*                                          */
        0x00, 0x00, 0x00, 0x00,         /*                                          */

    /* Feature Descriptor */
        0x00, 0x24,                     /* Feature Code : Defect Management (Ability of the drive/media system to provide an 
                                                          apparently defect-free space.) */
        0x04,                           /* Persistent/current */  
        0x04,                           /* Additional Length */                           
        
        0x80, 0x00, 0x00, 0x00,         /*                                          */

    /* Feature Descriptor */
        0x00, 0x26,                     /* Feature Code : Restricted Overwrite (Write support for media that must be written 
                                                          in multiples of logical blocks.) */
        0x00,                           /* Persistent/current */  
        0x00,                           /* Additional Length */                           
        

    /* Feature Descriptor */
        0x00, 0x2d,                     /* Feature Code : CD Track at Once (Ability to write CD with Track at Once recording) */
        0x08,                           /* Persistent/current */  
        0x04,                           /* Additional Length */                           
        
        0x46, 0x00, 0x3f, 0x0f,         /*                                          */

    /* Feature Descriptor */
        0x00, 0x2e,                     /* Feature Code : CD Mastering (The ability to write CD with Session at Once or Raw write methods.) */
        0x04,                           /* Persistent/current */  
        0x04,                           /* Additional Length */                           
        
        0x7f, 0x00, 0x0d, 0x00,         /*                                          */

    /* Feature Descriptor */
        0x00, 0x2f,                     /* Feature Code : DVD-R Write (The ability to write DVD specific structures) */
        0x08,                           /* Persistent/current */  
        0x04,                           /* Additional Length */                           
        
        0x4e, 0x00, 0x00, 0x00,         /*                                          */


        0x01, 0x00,                     /* Feature Code : Power Management (Initiator and device directed power management) */
        0x07,                           /* Persistent/current */  
        0x04,                           /* Additional Length */                           
        
        0x00, 0x00, 0x00, 0x00,         /*                                          */


        0x01, 0x01,                     /* Feature Code : S.M.A.R.T. (Self Monitoring Analysis and Reporting Technology (Failure prediction)) */
        0x00,                           /* Persistent/current */  
        0x04,                           /* Additional Length */                           
        
        0x00, 0x00, 0x00, 0x00,         /*                                          */

        0x01, 0x08,                     /* Feature Code : Logical Unit serial number (Logical unit has a unique identifier) */
        0x03,                           /* Persistent/current */  
        0x10,                           /* Additional Length */                           

        0x53, 0x31, 0x33, 0x36,         /* Serial Number */
        0x36, 0x59, 0x42, 0x46, 
        0x37, 0x30, 0x30, 0x39, 
        0x45, 0x48, 0x20, 0x20,         

        0x01, 0x0a,                     /* Feature Code : Not sure : says FDC STC TOC */
        0x00,                           /* Persistent/current */  
        0x0c,                           /* Additional Length */                           

        0x46, 0x44, 0x43, 0x00, 
        0x53, 0x54, 0x43, 0x00, 
        0x54, 0x4F, 0x43, 0x00,
    };


#define USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_ACTIVE_PROFILE_LENGTH                          112
#define USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_ACTIVE_HEADER_LENGTH                           8
#define USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_ACTIVE_FEATURE_DESCRIPTOR_LENGTH               32
#define USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_ACTIVE_FEATURE_HEADER_LENGTH                   4
#define USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_ACTIVE_FEATURE_LENGTH (USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_ACTIVE_PROFILE_LENGTH - USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_ACTIVE_FEATURE_DESCRIPTOR_LENGTH - USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_ACTIVE_HEADER_LENGTH)
UCHAR usbx_device_class_storage_configuration_active_profile[] = { 

    /* Feature Header */
        0x00, 0x00,                     /* Entire length of profile filled by firmware */
        0x00, 0x00,                  
        0x00, 0x00,            
        0x00, 0x08,                     /* Current profile is CD-ROM*/                           

    /* Feature Descriptor */
        0x00, 0x00,                     /* Feature Code : profile list */
        0x00,                           /* Persistent/current */  
        0x1c,                           /* Additional Length */                           
        
        0x00, 0x12, 0x00, 0x00,         /* DVD-RAM                                  */
        0x00, 0x11, 0x00, 0x00,         /* DVD-R                                    */
        0x00, 0x10, 0x00, 0x00,         /* DVD-ROM                                  */
        0x00, 0x0A, 0x00, 0x00,         /* CD-RW                                    */
        0x00, 0x09, 0x00, 0x00,         /* CD-R                                     */
        0x00, 0x08, 0x01, 0x00,         /* CD-ROM : active profile                  */
        0x00, 0x02, 0x00, 0x00,         /* Writable capable with removable media.   */
        

    /* Feature Descriptor */
        0x00, 0x01,                     /* Feature Code : core feature */
        0x0b,                           /* Persistent/current */  
        0x08,                           /* Additional Length */                           
        
        0x00, 0x00, 0x00, 0x07,         /* Physical Interface Standard              */
        0x01, 0x00, 0x00, 0x00,         /*                                          */

    /* Feature Descriptor */
        0x00, 0x02,                     /* Feature Code : morphing (Ability to notify initiator about 
                                                          operational changes and accept initiator requests to prevent 
                                                          operational changes) */
        0x07,                           /* Persistent/current */  
        0x04,                           /* Additional Length */                           
        
        0x02, 0x00, 0x00, 0x00,         /* Physical Interface Standard              */

    /* Feature Descriptor */
        0x00, 0x03,                     /* Feature Code : Removable Medium (The medium may be removed 
                                                          from the device ) */
        0x0b,                           /* Persistent/current */  
        0x04,                           /* Additional Length */                           
        
        0x2b, 0x00, 0x00, 0x00,         /* Physical Interface Standard              */

    /* Feature Descriptor */
        0x00, 0x10,                     /* Feature Code : Random Readable (Read ability for storage devices 
                                                          with random addressing) */
        0x01,                           /* Persistent/current */  
        0x08,                           /* Additional Length */                           
        
        0x00, 0x00, 0x08, 0x00,         /*                                          */
        0x00, 0x01, 0x01, 0x00,         /*                                          */

    /* Feature Descriptor */
        0x00, 0x1d,                     /* Feature Code : MultiRead (The logical unit can read all CD media types) */
        0x01,                           /* Persistent/current */  
        0x00,                           /* Additional Length */                           


    /* Feature Descriptor */
        0x00, 0x1e,                     /* Feature Code : CD Read (The ability to read CD specific structures) */
        0x09,                           /* Persistent/current */  
        0x04,                           /* Additional Length */                           
        
        0x03, 0x00, 0x00, 0x00,         /*                                          */


        0x01, 0x08,                     /* Feature Code : Logical Unit serial number (Logical unit has a unique identifier) */
        0x03,                           /* Persistent/current */  
        0x10,                           /* Additional Length */                           

        0x53, 0x31, 0x33, 0x36,         /* Serial Number */
        0x36, 0x59, 0x42, 0x46, 
        0x37, 0x30, 0x30, 0x39, 
        0x45, 0x48, 0x20, 0x20,         

    };

#if (UX_SLAVE_REQUEST_DATA_MAX_LENGTH < USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_ACTIVE_PROFILE_LENGTH) ||\
    (UX_SLAVE_REQUEST_DATA_MAX_LENGTH < USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_PROFILE_LENGTH)
#error UX_SLAVE_REQUEST_DATA_MAX_LENGTH too small, please check
#endif


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_storage_get_configuration          PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs a GET_CONFIGURATION command.                 */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    storage                               Pointer to storage class      */ 
/*    endpoint_in                           Pointer to IN endpoint        */
/*    endpoint_out                          Pointer to OUT endpoint       */
/*    cbwcb                                 Pointer to CBWCB              */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_class_storage_csw_send     Send CSW                      */ 
/*    _ux_device_stack_transfer_request     Transfer request              */ 
/*    _ux_device_stack_endpoint_stall       Stall endpoint                */
/*    _ux_utility_short_get_big_endian      Get 16-bit big endian         */
/*    _ux_utility_long_put_big_endian       Put 32-bit big endian         */ 
/*    _ux_utility_memory_copy               Copy memory                   */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Device Storage Class                                                */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            optimized command logic,    */
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_storage_get_configuration(UX_SLAVE_CLASS_STORAGE *storage, ULONG lun,
                                            UX_SLAVE_ENDPOINT *endpoint_in,
                                            UX_SLAVE_ENDPOINT *endpoint_out, UCHAR * cbwcb)
{

UINT                    status = 0;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                   starting_feature;
ULONG                   allocation_length;
ULONG                   additional_length;
ULONG                   profile_counter;
UCHAR                   *profile_pointer;
ULONG                   feature;

    UX_PARAMETER_NOT_USED(endpoint_out);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_STORAGE_GET_CONFIGURATION, storage, lun, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)
    
    /* Initialize the length of the configuration profile.  */
    _ux_utility_long_put_big_endian(usbx_device_class_storage_configuration_profile, (USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_PROFILE_LENGTH - 4));
    
    /* Initialize the length of the active configuration profile.  */
    _ux_utility_long_put_big_endian(usbx_device_class_storage_configuration_active_profile, (USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_ACTIVE_PROFILE_LENGTH - 4));
    
    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &endpoint_in -> ux_slave_endpoint_transfer_request;
    
    /* Get the Starting Feature.  */
    starting_feature =  _ux_utility_short_get_big_endian(cbwcb + UX_SLAVE_CLASS_STORAGE_GET_CONFIGURATION_STARTING_FEATURE);

    /* Get the allocation length.  */
    allocation_length =  _ux_utility_short_get_big_endian(cbwcb + UX_SLAVE_CLASS_STORAGE_GET_CONFIGURATION_ALLOCATION_LENGTH);

    /* Default CSW to success.  */
    storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PASSED;

    /* Is the request demanding all the features ? */
    if (starting_feature == 0)
    {

        /* Is the requester demanding the active profile ? */
        if ((*(cbwcb + UX_SLAVE_CLASS_STORAGE_GET_CONFIGURATION_RT) & 3) == 1)
        {

            /* We get the active profile.  */
            /* Can we send all the activeconfiguration profile ? If not, the host may demand the first part of the configuration to get the entire length. 
               In this case, return the length demanded by the host.  */
            if (allocation_length >= USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_ACTIVE_PROFILE_LENGTH)

                /* Adjust allocation length to maximum allowed.  */
                allocation_length = USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_ACTIVE_PROFILE_LENGTH;
                            
            /* Copy the CSW into the transfer request memory.  */
            _ux_utility_memory_copy(transfer_request -> ux_slave_transfer_request_data_pointer, 
                                                usbx_device_class_storage_configuration_active_profile, 
                                                allocation_length); /* Use case of memcpy is verified. */
        }
        else
        {

            /* We get the whole profile.  */
            /* Can we send all the configuration profile ? If not, the host may demand the first part of the configuration to get the entire length. 
               In this case, return the length demanded by the host.  */
            if (allocation_length >= USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_PROFILE_LENGTH)

                /* Adjust allocation length to maximum allowed.  */
                allocation_length = USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_PROFILE_LENGTH;
                            
            /* Copy the CSW into the transfer request memory.  */
            _ux_utility_memory_copy(transfer_request -> ux_slave_transfer_request_data_pointer, 
                                                usbx_device_class_storage_configuration_profile, 
                                                allocation_length); /* Use case of memcpy is verified. */
            
        }

        /* Now success.  */
        status = UX_SUCCESS;
    }
    else
    {

        /* The caller has demanded a specific feature. Scan our configuration profile.  Jump over the beginning sections.  */
        profile_pointer = usbx_device_class_storage_configuration_profile + USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_HEADER_LENGTH +
                                                                            USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_FEATURE_DESCRIPTOR_LENGTH;
        profile_counter = USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_FEATURE_LENGTH;
        
        /* Scan our configuration profile.  */
        while (profile_counter != 0)
        {

            /* Extract the feature from the configuration profile. */
            feature =  _ux_utility_short_get_big_endian(profile_pointer + USBX_DEVICE_CLASS_STORAGE_FEATURE_DESCRIPTOR_FEATURE_CODE);
            
            /* Extract the feature length from the configuration profile. */
            additional_length =  (ULONG ) *(profile_pointer + USBX_DEVICE_CLASS_STORAGE_FEATURE_DESCRIPTOR_FEATURE_ADD_LENGTH);

            /* Compare the Feature extracted with the one demanded.  */
            if (feature == starting_feature)
            {
            
                /* We found the feature, we check if the requester has enough space for us to return it.  */
                if (allocation_length >= (additional_length + USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_HEADER_LENGTH + 
                                                              USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_FEATURE_HEADER_LENGTH))

                    /* Need to adjust the allocation length.  */
                    allocation_length = additional_length + USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_HEADER_LENGTH +
                                                            USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_FEATURE_HEADER_LENGTH;

                /* Copy the CSW into the transfer request memory.  */
                _ux_utility_memory_copy(transfer_request -> ux_slave_transfer_request_data_pointer, 
                                                    profile_pointer, 
                                                    allocation_length); /* Use case of memcpy is verified. */

                /* Now success.  */
                status = UX_SUCCESS;

                /* Get out of the loop.  */
                break;                    
            }
            else            
            {
                
                /* We have not yet found the feature, keep parsing.  */
                if (profile_counter - additional_length - USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_FEATURE_HEADER_LENGTH <= 0)
                {

                    /* We are either at the end of the profile or the profile is corrupted.  */

                    /* And update the REQUEST_SENSE codes.  */
                    storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_request_sense_status =
                        UX_DEVICE_CLASS_STORAGE_SENSE_STATUS(UX_SLAVE_CLASS_STORAGE_SENSE_KEY_ILLEGAL_REQUEST,0x26,0x02);

                    /* Now we set the CSW with failure.  */
                    storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_FAILED;

                    /* Set status to error.  */
                    status = UX_ERROR;

                    /* Get out of the loop.  */
                    break;                    

                }
                else
                {

                    /* Update the profile pointer. */
                    profile_pointer += additional_length + USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_FEATURE_HEADER_LENGTH;

                    /* Update the length remaining to parse in profile.  */
                    profile_counter -= additional_length + USBX_DEVICE_CLASS_STORAGE_CONFIGURATION_FEATURE_HEADER_LENGTH;
                }
            }
        }        
    }

    /* Success, send data.  */
    if (status == UX_SUCCESS)
    {

#if defined(UX_DEVICE_STANDALONE)

        /* Next: Transfer (DATA).  */
        storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_TRANS_START;
        storage -> ux_device_class_storage_cmd_state = UX_DEVICE_CLASS_STORAGE_CMD_READ;

        storage -> ux_device_class_storage_transfer = transfer_request;
        storage -> ux_device_class_storage_device_length = allocation_length;
        storage -> ux_device_class_storage_data_length = allocation_length;
        storage -> ux_device_class_storage_data_count = 0;
        UX_SLAVE_TRANSFER_STATE_RESET(storage -> ux_device_class_storage_transfer);
#else

        /* Send a data payload with the read_capacity response buffer.  */
        _ux_device_stack_transfer_request(transfer_request, 
                                    allocation_length,
                                    allocation_length);
#endif

    }
    else
    {

        /* Error, stall.  */
#if !defined(UX_DEVICE_STANDALONE)
        _ux_device_stack_endpoint_stall(endpoint_in);
#endif
    }

    /* Return completion status.  */
    return(status);
}

