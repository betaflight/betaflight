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
/**   System                                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Define UX_SYSTEM_INIT to bring in the USBX version ID string.  */

#define UX_SYSTEM_INIT


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_system.h"

/* Define the USBX system data structure.  */

UX_SYSTEM         *_ux_system;
UX_SYSTEM_OTG     *_ux_system_otg;

/* Define names of all the packed descriptors in USBX.  */

UCHAR _ux_system_endpoint_descriptor_structure[] =                          {1,1,1,1,2,1 };
UCHAR _ux_system_device_descriptor_structure[] =                            {1,1,2,1,1,1,1,2,2,2,1,1,1,1};
UCHAR _ux_system_configuration_descriptor_structure[] =                     {1,1,2,1,1,1,1,1};
UCHAR _ux_system_interface_descriptor_structure[] =                         {1,1,1,1,1,1,1,1,1};
UCHAR _ux_system_interface_association_descriptor_structure[] =             {1,1,1,1,1,1,1,1};
UCHAR _ux_system_string_descriptor_structure[] =                            {1,1,2};
UCHAR _ux_system_dfu_functional_descriptor_structure[] =                    {1,1,1,2,2,2};
UCHAR _ux_system_class_audio_interface_descriptor_structure[] =             {1,1,1,1,1,1,1,1};
UCHAR _ux_system_class_audio_input_terminal_descriptor_structure[] =        {1,1,1,1,2,1,1,2,1,1};
UCHAR _ux_system_class_audio_output_terminal_descriptor_structure[] =       {1,1,1,1,2,1,1,1};
UCHAR _ux_system_class_audio_feature_unit_descriptor_structure[] =          {1,1,1,1,1,1,1};
UCHAR _ux_system_class_audio_streaming_interface_descriptor_structure[] =   {1,1,1,1,1,1};
UCHAR _ux_system_class_audio_streaming_endpoint_descriptor_structure[] =    {1,1,1,1,1,1};
UCHAR _ux_system_hub_descriptor_structure[] =                               {1,1,1,2,1,1,1,1};
UCHAR _ux_system_hid_descriptor_structure[] =                               {1,1,2,1,1,1,2};
UCHAR _ux_system_class_pima_storage_structure[] =                           {2,2,2,4,4,4,4,4};
UCHAR _ux_system_class_pima_object_structure[] =                            {4,2,2,4,2,4,4,4,4,4,4,4,2,4,4};
UCHAR _ux_system_ecm_interface_descriptor_structure[] =                     {1,1,1,1,4,2,2,1};

UCHAR _ux_system_bos_descriptor_structure[] =                               {1,1,2,1};
UCHAR _ux_system_usb_2_0_extension_descriptor_structure[] =                 {1,1,1,4};
UCHAR _ux_system_container_id_descriptor_structure[] =                      {1,1,1,1,4,4,4,4};


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_system_initialize                               PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function initializes the various control data structures for   */ 
/*    the USBX system.                                                    */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    regular_memory_pool_start        Start of non cached memory pool    */
/*    regular_memory_size              Size of non cached memory pool     */ 
/*    cache_safe_memory_pool_start     Start of cached memory pool        */
/*    cache_safe_memory_size           Size of cached memory pool         */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_set                Set memory                    */ 
/*    _ux_utility_mutex_create              Create mutex                  */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  12-31-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added BOS support,          */
/*                                            resulting in version 6.1.3  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_system_initialize(VOID *regular_memory_pool_start, ULONG regular_memory_size, 
                            VOID *cache_safe_memory_pool_start, ULONG cache_safe_memory_size)
{

UX_MEMORY_BLOCK     *memory_block;
ALIGN_TYPE          int_memory_pool_start;
VOID                *regular_memory_pool_end;
ULONG               memory_pool_offset;
#if !defined(UX_STANDALONE)
UINT                status;
#endif


    /* Reset memory block */
    _ux_utility_memory_set(regular_memory_pool_start, 0, regular_memory_size); /* Use case of memset is verified. */

    /* Set the _ux_system structure at the start of our regular memory */
    _ux_system =  (UX_SYSTEM *) regular_memory_pool_start;

    /* Add to the memory offset the size of the allocated block.  */
    memory_pool_offset = sizeof(UX_SYSTEM);

#ifndef UX_DEVICE_SIDE_ONLY

    /* Set the _ux_system_host structure.  */
    _ux_system_host =  (UX_SYSTEM_HOST *) (((UCHAR *) regular_memory_pool_start) + memory_pool_offset);

    /* Add to the memory offset the size of the allocated block.  */
    memory_pool_offset += (ULONG)sizeof(UX_SYSTEM_HOST);
    
#endif 

#ifndef UX_HOST_SIDE_ONLY

    /* Set the _ux_system_slave structure.  */
    _ux_system_slave =  (UX_SYSTEM_SLAVE *) (((UCHAR *) regular_memory_pool_start) + memory_pool_offset);

    /* Add to the memory offset the size of the allocated block.  */
    memory_pool_offset += (ULONG)sizeof(UX_SYSTEM_SLAVE);
    
#endif 


#ifdef UX_OTG_SUPPORT

    /* Set the _ux_system_otg structure.  */
    _ux_system_otg =  (UX_SYSTEM_OTG *) (((UCHAR *) regular_memory_pool_start) + memory_pool_offset);

    /* Add to the memory offset the size of the allocated block.  */
    memory_pool_offset += (ULONG)sizeof(UX_SYSTEM_OTG);
#endif 


    /* Set the cache safe memory for the dynamic pool */
    _ux_system -> ux_system_regular_memory_pool_start =  (UX_MEMORY_BLOCK *) (((UCHAR *) regular_memory_pool_start) 
                                                            + memory_pool_offset);

    /* Make sure the regular memory pool is aligned properly */
    int_memory_pool_start =   (ALIGN_TYPE) _ux_system -> ux_system_regular_memory_pool_start;
    int_memory_pool_start +=  UX_ALIGN_MIN;
    int_memory_pool_start &=  ~((ALIGN_TYPE)UX_ALIGN_MIN);
    
    /* Set the end of the regular memory pool.  */
    regular_memory_pool_end =  (void *) (((UCHAR *) regular_memory_pool_start) + regular_memory_size);

    /* Check if we have memory available.  */
    if (int_memory_pool_start >= (ALIGN_TYPE)regular_memory_pool_end)
    {

        /* No memory available.  */
        return(UX_MEMORY_INSUFFICIENT);
    }

    /* Now, we have a project structure allocated, save the regular memory allocation details */
    _ux_system -> ux_system_regular_memory_pool_size =     (ULONG) (((ALIGN_TYPE) regular_memory_pool_end) - int_memory_pool_start);
    _ux_system -> ux_system_regular_memory_pool_free =     _ux_system -> ux_system_regular_memory_pool_size;
    _ux_system -> ux_system_regular_memory_pool_start =    (UX_MEMORY_BLOCK *) int_memory_pool_start;

    /* Build the first free memory block */
    memory_block =                             _ux_system -> ux_system_regular_memory_pool_start;
    memory_block -> ux_memory_block_size =     _ux_system -> ux_system_regular_memory_pool_size - (ULONG)sizeof(UX_MEMORY_BLOCK);
    memory_block -> ux_memory_block_status =   UX_MEMORY_UNUSED;

    /* Check the definition of the cache safe pool. If the application or controller do not require any cache safe memory,
       define the cached safe memory region as the regular memory region.  */
    if (cache_safe_memory_pool_start == UX_NULL)
    {

        /* Cache safe memory is the same as regular memory.  */
        _ux_system -> ux_system_cache_safe_memory_pool_size =  _ux_system -> ux_system_regular_memory_pool_size;
        _ux_system -> ux_system_cache_safe_memory_pool_free =  _ux_system -> ux_system_regular_memory_pool_free;
        _ux_system -> ux_system_cache_safe_memory_pool_start = _ux_system -> ux_system_regular_memory_pool_start;
    }
    else
    {
    
        /* Make sure the cache safe memory pool is aligned properly */
        int_memory_pool_start =   (ALIGN_TYPE) cache_safe_memory_pool_start;
        int_memory_pool_start +=  UX_ALIGN_MIN;
        int_memory_pool_start &=  ~((ALIGN_TYPE)UX_ALIGN_MIN);
    
        /* Save the cache safe memory allocation details */
        _ux_system -> ux_system_cache_safe_memory_pool_size =     cache_safe_memory_size - UX_ALIGN_MIN;
        _ux_system -> ux_system_cache_safe_memory_pool_free =     _ux_system -> ux_system_cache_safe_memory_pool_size;
        _ux_system -> ux_system_cache_safe_memory_pool_start =    (UX_MEMORY_BLOCK *) int_memory_pool_start;
    
        /* Reset this memory block */
        _ux_utility_memory_set(_ux_system -> ux_system_cache_safe_memory_pool_start, 0, _ux_system -> ux_system_cache_safe_memory_pool_size); /* Use case of memset is verified. */
    
        /* Build the first free memory block */
        memory_block =                             _ux_system -> ux_system_cache_safe_memory_pool_start;
        memory_block -> ux_memory_block_size =     _ux_system -> ux_system_cache_safe_memory_pool_size - (ULONG)sizeof(UX_MEMORY_BLOCK);
        memory_block -> ux_memory_block_status =   UX_MEMORY_UNUSED;
    }

#ifdef UX_ENABLE_MEMORY_STATISTICS
    _ux_system -> ux_system_regular_memory_pool_base = (UCHAR *) _ux_system -> ux_system_regular_memory_pool_start;
    _ux_system -> ux_system_regular_memory_pool_min_free = _ux_system -> ux_system_regular_memory_pool_free;
    _ux_system -> ux_system_cache_safe_memory_pool_base = (UCHAR *) _ux_system -> ux_system_cache_safe_memory_pool_start;
    _ux_system -> ux_system_cache_safe_memory_pool_min_free = _ux_system -> ux_system_cache_safe_memory_pool_free;

    /* Other fields are kept zero.  */
#endif

#ifdef UX_ENABLE_DEBUG_LOG

    /* Obtain memory for storing the debug log.  */
    _ux_system -> ux_system_debug_log_buffer =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_DEBUG_LOG_SIZE);
    if (_ux_system -> ux_system_debug_log_buffer == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Setup the head and tail pointers.  */
    _ux_system -> ux_system_debug_log_head = _ux_system -> ux_system_debug_log_buffer;
    _ux_system -> ux_system_debug_log_tail = _ux_system -> ux_system_debug_log_buffer;
    
    /* Keep the size in system structure variable.  */
    _ux_system -> ux_system_debug_log_size = UX_DEBUG_LOG_SIZE;
    
#endif

#if !defined(UX_STANDALONE)

    /* Create the Mutex object used by USBX to control critical sections.  */
    status =  _ux_system_mutex_create(&_ux_system -> ux_system_mutex, "ux_system_mutex");
    if(status != UX_SUCCESS)
        return(UX_MUTEX_ERROR);
#endif

    return(UX_SUCCESS);
}

