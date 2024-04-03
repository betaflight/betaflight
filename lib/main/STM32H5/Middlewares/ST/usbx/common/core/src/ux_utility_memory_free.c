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
/**   Utility                                                             */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_utility_memory_free                             PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function frees a previously allocated memory block.            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    memory                                Pointer to memory block       */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_mutex_on                  Start system protection       */ 
/*    _ux_utility_mutex_off                 End system protection         */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX Components                                                     */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_utility_memory_free(VOID *memory)
{

UX_MEMORY_BLOCK     *memory_block;
UX_MEMORY_BLOCK     *next_block;
ULONG               memory_size_returned;
UCHAR               *memory_address;
#ifdef UX_ENABLE_MEMORY_POOL_SANITY_CHECK
UCHAR               *regular_start, *regular_end;
UCHAR               *cache_safe_start, *cache_safe_end;
#endif

    /* Get the mutex as this is a critical section.  */
    _ux_system_mutex_on(&_ux_system -> ux_system_mutex);

#ifdef UX_ENABLE_MEMORY_POOL_SANITY_CHECK

    /* Sanity check, check if the memory is in memory pool.  */
    regular_start = (UCHAR *)_ux_system -> ux_system_regular_memory_pool_start;
    regular_end = regular_start + _ux_system -> ux_system_regular_memory_pool_size;
    regular_start += sizeof(UX_MEMORY_BLOCK);
    cache_safe_start = (UCHAR *)_ux_system -> ux_system_cache_safe_memory_pool_start;
    cache_safe_end = cache_safe_start + _ux_system -> ux_system_cache_safe_memory_pool_size;
    cache_safe_start += sizeof(UX_MEMORY_BLOCK);
    memory_address = (UCHAR *)memory;
    if (!((memory_address >= regular_start    && memory_address < regular_end) ||
          (memory_address >= cache_safe_start && memory_address < cache_safe_end)))
    {

        /* Not valid. Release the protection.  */
        _ux_system_mutex_off(&_ux_system -> ux_system_mutex);

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD,
                                UX_SYSTEM_CONTEXT_UTILITY, UX_MEMORY_CORRUPTED);

        /* No action taken.  */
        return;
    }
#endif

    /* The memory block for this memory pointer is located right before the
       memory.  */
    memory_block =  (UX_MEMORY_BLOCK *) (((UCHAR *) memory) - sizeof(UX_MEMORY_BLOCK));
    
    /* Keep track of the memory returned to the pool.  */
    memory_size_returned = memory_block -> ux_memory_block_size + (ULONG)sizeof(UX_MEMORY_BLOCK);

    /* Check this memory block to see if it valid.  */
    if (memory_block -> ux_memory_block_status != (UX_MEMORY_USED | UX_REGULAR_MEMORY) &&
        memory_block -> ux_memory_block_status != (UX_MEMORY_USED | UX_CACHE_SAFE_MEMORY))
    {

        /* Not valid. Release the protection.  */
        _ux_system_mutex_off(&_ux_system -> ux_system_mutex);

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_UTILITY, UX_MEMORY_CORRUPTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_MEMORY_CORRUPTED, memory, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Return to caller.  */
        return;
    }

#ifdef UX_ENABLE_MEMORY_STATISTICS

    /* Update allocate count, total size.  */
    if (memory_block -> ux_memory_block_status == (UX_MEMORY_USED | UX_REGULAR_MEMORY))
    {
        _ux_system -> ux_system_regular_memory_pool_alloc_count --;
        _ux_system -> ux_system_regular_memory_pool_alloc_total -= memory_block -> ux_memory_block_size;
    }
    else
    {
        _ux_system -> ux_system_cache_safe_memory_pool_alloc_count --;
        _ux_system -> ux_system_cache_safe_memory_pool_alloc_total -= memory_block -> ux_memory_block_size;
    }
#endif

    /* We mark this memory block as being unused.  */
    memory_block -> ux_memory_block_status =  UX_MEMORY_UNUSED;
    
    /* Now we must concatenate as many free blocks as possible,
       that include the blocks before and the blocks after the current
       block.  Scan memory backwards.  */

     while (memory_block -> ux_memory_block_previous !=UX_NULL)
     {

        /* Check if the block is free.  */            
        if (memory_block -> ux_memory_block_previous -> ux_memory_block_status == UX_MEMORY_UNUSED)

            /* The memory block before is free. This will be our starting point to 
               concatenate memory.  */
            memory_block =  memory_block -> ux_memory_block_previous;

        else

            /* The previous memory block is not free.  */
            break;
    }

    /* The pointer to the memory block is now our first free block. We use this 
       starting address to concatenate all the contiguous memory block.  */
    next_block =  memory_block -> ux_memory_block_next;
    while (next_block != UX_NULL)
    {

        /* Determine if the memory block is used.  */
        if (next_block -> ux_memory_block_status != UX_MEMORY_UNUSED)
        {

            /* Yes, move to next block.  */
            memory_block -> ux_memory_block_next =  next_block;
            next_block -> ux_memory_block_previous =  memory_block;
            break;
        }

        memory_block -> ux_memory_block_next =  next_block -> ux_memory_block_next;
        memory_block -> ux_memory_block_size +=  next_block -> ux_memory_block_size + (ULONG)sizeof(UX_MEMORY_BLOCK);
        next_block =  next_block -> ux_memory_block_next;                       
    }

    /* Update the memory free in the appropriate pool.  We need to know if this 
       block is in regular memory or cache safe memory.  */
    if(_ux_system -> ux_system_cache_safe_memory_pool_start == _ux_system -> ux_system_regular_memory_pool_start)
    {

        /* There is only one regular memory pool.  */
        _ux_system -> ux_system_regular_memory_pool_free += memory_size_returned;
        
    }
    else
    {

        /* Which pool is this memory in ?  */
        memory_address = (UCHAR *) _ux_system -> ux_system_regular_memory_pool_start;
        
        /* If the memory address is in this range, we are in the regular memory pool.  */
        if ((UCHAR *) memory_block >= memory_address && (UCHAR *) memory_block < (memory_address + _ux_system -> ux_system_regular_memory_pool_size))

            /* Update the regular memory pool.  */
            _ux_system -> ux_system_regular_memory_pool_free += memory_size_returned;

        else
        
            /* Update the cache safe memory pool.  */
            _ux_system -> ux_system_cache_safe_memory_pool_free += memory_size_returned;
        
    }

    /* Release the protection.  */
    _ux_system_mutex_off(&_ux_system -> ux_system_mutex);

    /* Return to caller.  */
    return;
}

