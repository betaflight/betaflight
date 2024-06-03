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


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_storage_partition_read               PORTABLE C      */ 
/*                                                           6.1.2        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will analyze the partition table and parse all the    */
/*    partitions. It may happen that a partition entry points to a        */
/*    secondary partition table.                                          */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    storage                               Pointer to storage class      */ 
/*    sector_memory                         Pointer to memory for sector  */ 
/*    sector                                Sector number                 */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_storage_media_mount    Mount media                   */ 
/*    _ux_host_class_storage_media_open     Open media                    */ 
/*    _ux_utility_long_get                  Get 32-bit word               */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_storage_media_mount    Mount media                   */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added option to disable FX  */
/*                                            media integration,          */
/*                                            resulting in version 6.1    */
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added exFAT support,        */
/*                                            resulting in version 6.1.2  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_partition_read(UX_HOST_CLASS_STORAGE *storage, UCHAR *sector_memory, ULONG sector)
{
#if defined(UX_HOST_CLASS_STORAGE_NO_FILEX)
    UX_PARAMETER_NOT_USED(storage);
    UX_PARAMETER_NOT_USED(sector_memory);
    UX_PARAMETER_NOT_USED(sector);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else
UINT        status =  UX_ERROR;
UINT        partition_index;
    

    /* Point the sector buffer to the first partition entry.  */
    sector_memory +=  UX_HOST_CLASS_STORAGE_PARTITION_TABLE_START;
    
    /* There are 4 partitions in a partition table.  */
    for (partition_index = 0; partition_index < 4; partition_index++)
    {

        /* Check if we recognize this partition entry.  */
        switch(*(sector_memory + UX_HOST_CLASS_STORAGE_PARTITION_TYPE))
        {

        case UX_HOST_CLASS_STORAGE_PARTITION_FAT_12:       
        case UX_HOST_CLASS_STORAGE_PARTITION_FAT_16:   
        case UX_HOST_CLASS_STORAGE_PARTITION_FAT_16L: 
        case UX_HOST_CLASS_STORAGE_PARTITION_FAT_16_LBA_MAPPED: 
        case UX_HOST_CLASS_STORAGE_PARTITION_FAT_32_1:   
        case UX_HOST_CLASS_STORAGE_PARTITION_FAT_32_2:   
        case UX_HOST_CLASS_STORAGE_PARTITION_EXFAT:

            /* We have found a legal partition entry pointing to a potential boot sector.  */
            status =  _ux_host_class_storage_media_open(storage, sector + _ux_utility_long_get(sector_memory + UX_HOST_CLASS_STORAGE_PARTITION_SECTORS_BEFORE));
            break;                              
            
        case UX_HOST_CLASS_STORAGE_PARTITION_EXTENDED:   
        case UX_HOST_CLASS_STORAGE_PARTITION_EXTENDED_LBA_MAPPED:   

            /* We have found an entry to an extended partition. We need to read that partition sector
               and recursively mount all partitions found.  */
            status =  _ux_host_class_storage_media_mount(storage, sector + _ux_utility_long_get(sector_memory + UX_HOST_CLASS_STORAGE_PARTITION_SECTORS_BEFORE));
            break;
                
        default:

            /* We have found something which is not a DOS recognized partition, or an empty entry.
               Ignore it and proceed with the rest.  */
            break;
        }

        /* Move to the next partition entry.  */
        sector_memory +=  UX_HOST_CLASS_STORAGE_PARTITION_TABLE_SIZE;
    }

    /* Return completion status.  */
    return(status);
#endif
}

