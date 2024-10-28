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
/*    _ux_utility_memory_allocate                         PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function allocates a block of memory for the specified size    */ 
/*    and alignment.                                                      */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    memory_alignment                      Memory alignment required     */ 
/*    memory_cache_flag                     Memory pool source            */ 
/*    memory_size_requested                 Number of bytes required      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Pointer to block of memory                                          */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_free_block_best_get Get best fit block of memory */ 
/*    _ux_utility_memory_set                 Set block of memory          */ 
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
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
VOID  *_ux_utility_memory_allocate(ULONG memory_alignment, ULONG memory_cache_flag,
                                   ULONG memory_size_requested)
{

UX_MEMORY_BLOCK     *memory_block;
UX_MEMORY_BLOCK     *new_memory_block;
UX_MEMORY_BLOCK     *leftover_memory_block;
ULONG               memory_for_alignment;
ULONG               memory_removed_from_pool;
ULONG               leftover;
UCHAR               *memory_buffer;
ALIGN_TYPE          int_memory_buffer;


    /* Get the mutex as this is a critical section.  */
    _ux_system_mutex_on(&_ux_system -> ux_system_mutex);

#ifdef UX_ENFORCE_SAFE_ALIGNMENT

    /* Check if safe alignment requested, in this case switch to UX_NO_ALIGN.  */
    if (memory_alignment == UX_SAFE_ALIGN)
    {

        /* We will use the memory_size_requested for the alignment. 
           But we check to see if we have a minimum or maximum alignment.  */
        if (memory_size_requested < UX_ALIGN_MIN)
        
            /* No need to bother about alignment for small packets sizes.  */
            memory_alignment = UX_NO_ALIGN;
        
        else
        {
    
            /* Check if we are over the maximum.  */
            if (memory_size_requested > UX_MAX_SCATTER_GATHER_ALIGNMENT)
            
                /* We are over the max alignment required. Use the maximum instead.  */
                memory_alignment = UX_MAX_SCATTER_GATHER_ALIGNMENT - 1;

            else
            {
                /* We are not over the maximum, so approximate the alignment according to the size of the memory.  
                   Check range for alignment on 4096 bytes.  */
                if (memory_size_requested >= UX_ALIGN_2048 + 1)
                    memory_alignment = UX_ALIGN_4096;

                else
                {

                       /* Check range for alignment on 2048 bytes.  */
                    if (memory_size_requested >= UX_ALIGN_1024 + 1)
                        memory_alignment = UX_ALIGN_2048;
            
                    else
                    {

                           /* Check range for alignment on 1024 bytes.  */
                        if (memory_size_requested >= UX_ALIGN_512 + 1)
                            memory_alignment = UX_ALIGN_1024;
            
                        else
                        {

                               /* Check range for alignment on 512 bytes.  */
                            if (memory_size_requested >= UX_ALIGN_256 + 1)
                                memory_alignment = UX_ALIGN_512;
                
                            else
                            {

                                   /* Check range for alignment on 256 bytes.  */
                                if (memory_size_requested >= UX_ALIGN_128 + 1)
                                    memory_alignment = UX_ALIGN_256;
                    
                                else
                                {
                            
                                       /* Check range for alignment on 128 bytes.  */
                                    if (memory_size_requested >= UX_ALIGN_64 + 1)
                                        memory_alignment = UX_ALIGN_128;
                    
                                    else
                                    {
                            
                                           /* Check range for alignment on 128 bytes.  */
                                        if (memory_size_requested >= UX_ALIGN_64 + 1)
                                            memory_alignment = UX_ALIGN_128;
                    
                                        else
                                        {
                                
                                               /* Check range for alignment on 64 bytes.  */
                                            if (memory_size_requested >= UX_ALIGN_32 + 1)
                                                memory_alignment = UX_ALIGN_64;
                    
                                            else
                                            {
                                    
                                                   /* Check range for alignment on 32 bytes.  */
                                                if (memory_size_requested >= UX_ALIGN_16 + 1)
                                                    memory_alignment = UX_ALIGN_32;
                    
                                                else
                                                    memory_alignment = UX_ALIGN_MIN;
                                

                                            }                        
                                        }                        
                                    }                        
                                }                        
                            }                        
                        }                        
                    }                        
                }
            }                
        }        
    }

#else

    /* Check if safe alignment requested, in this case switch to UX_NO_ALIGN.  */
    if (memory_alignment == UX_SAFE_ALIGN)
        memory_alignment = UX_NO_ALIGN;
    
#endif

    /* Ensure the alignment meats the minimum.  */
    if (memory_alignment < UX_ALIGN_MIN)
        memory_alignment =  UX_ALIGN_MIN;

    /* Adjust the memory alignment since our macros are one minus the desired alignment.
       Also determine the amount of extra memory we need for the alignment, which is one
       minus the actual alignment.  */
    memory_for_alignment =  memory_alignment;
    memory_alignment++;

    /* We need to make sure that the next memory block buffer is 16-byte aligned too. We
       do this by first adjusting the requested memory to be 16-byte aligned. One problem
       now is that the memory block might not be a size that is a multiple of 16, so we need
       to add the amount of memory required such that the memory buffer after the block has 
       the correct alignment. For example, if the memory block has a size of 24, then we need
       to make sure it is placed on an 8-byte alignment that is after a 16-byte alignment so
       that the memory right after the memory block is 16-byte aligned (8 + 24 = 32).  */
    memory_size_requested =  (memory_size_requested +    UX_ALIGN_MIN) & (~(ULONG)UX_ALIGN_MIN);
    memory_size_requested += (((ULONG)sizeof(UX_MEMORY_BLOCK) + UX_ALIGN_MIN) & (~(ULONG)UX_ALIGN_MIN)) - (ULONG)sizeof(UX_MEMORY_BLOCK);

    /* Try to find the best block for this memory by requesting the maximum amount of
       memory we'll need which is calculated as follows: the amount memory requested by
       the caller plus the maximum amount of memory wasted due to alignment plus 2 memory
       blocks structs - one for the new memory block we'll create for the user block and one
       that we might create if there is extra memory after doing the alignment.  */
    memory_block =  _ux_utility_memory_free_block_best_get(memory_cache_flag, memory_size_requested + memory_for_alignment + (ULONG)sizeof(UX_MEMORY_BLOCK));

    /* If the block returned is NULL, there is no free memory in the pool
       for that size. */
    if (memory_block == UX_NULL)
    {

        /* Release the protection.  */
        _ux_system_mutex_off(&_ux_system -> ux_system_mutex);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_MEMORY_INSUFFICIENT, memory_size_requested, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_UTILITY, UX_MEMORY_INSUFFICIENT);

        /* Return NULL to indicate no block was found.  */
        return(UX_NULL);
    }

    /* Get the memory buffer for this block.  */
    int_memory_buffer = (ALIGN_TYPE) ((UCHAR *) memory_block + sizeof(UX_MEMORY_BLOCK));

    /* Are we already aligned?  */
    if ((int_memory_buffer & (memory_alignment - 1)) == 0)
    {

        /* Setup the new memory block.  */
        new_memory_block =  (UX_MEMORY_BLOCK *) ((UCHAR *) memory_block + sizeof(UX_MEMORY_BLOCK) + memory_size_requested);
        new_memory_block -> ux_memory_block_next =  memory_block -> ux_memory_block_next;
        new_memory_block -> ux_memory_block_previous =  memory_block;
        new_memory_block -> ux_memory_block_size =  memory_block -> ux_memory_block_size - memory_size_requested - (ULONG)sizeof(UX_MEMORY_BLOCK);
        new_memory_block -> ux_memory_block_status =  UX_MEMORY_UNUSED;

        /* Update the current memory block.  */
        memory_block -> ux_memory_block_size =  memory_size_requested;
        memory_block -> ux_memory_block_next =  new_memory_block;
        memory_block -> ux_memory_block_status =  UX_MEMORY_USED | memory_cache_flag;

        /* Declare how much memory we removed from the pool.  */
        memory_removed_from_pool =  memory_block -> ux_memory_block_size + (ULONG)sizeof(UX_MEMORY_BLOCK);
    }
    else
    {

        /* Align the buffer. The first thing we do is increment by the size of a
           memory block because we have to make sure we have enough memory for at
           least that.  */
        int_memory_buffer +=  (ULONG)sizeof(UX_MEMORY_BLOCK);
        int_memory_buffer +=  memory_alignment - 1;
        int_memory_buffer &=  ~(((ALIGN_TYPE) memory_alignment) - 1);

        /* Setup the new memory block. Note that its size is updated again later.  */
        new_memory_block =  (UX_MEMORY_BLOCK *) (int_memory_buffer - (ULONG)sizeof(UX_MEMORY_BLOCK));
        new_memory_block -> ux_memory_block_previous =  memory_block;
        new_memory_block -> ux_memory_block_next =  memory_block -> ux_memory_block_next;
        new_memory_block -> ux_memory_block_size =  memory_block -> ux_memory_block_size;
        new_memory_block -> ux_memory_block_status =  UX_MEMORY_USED | memory_cache_flag;

        /* Update the current memory block.  */
        int_memory_buffer =  (ALIGN_TYPE) ((UCHAR *) memory_block + sizeof(UX_MEMORY_BLOCK));
        memory_block -> ux_memory_block_next =  new_memory_block;
        memory_block -> ux_memory_block_size =  (ULONG) ((ALIGN_TYPE) new_memory_block - int_memory_buffer);

        /* Update the new memory block's size.  */
        new_memory_block -> ux_memory_block_size -=  (memory_block -> ux_memory_block_size + (ULONG)sizeof(UX_MEMORY_BLOCK));

        /* Calculate how much memory is leftover in the new memory block after doing
           the alignment.  */
        leftover =  new_memory_block -> ux_memory_block_size - memory_size_requested;

        /* Can we fit another block after the new block? */
        if (leftover > sizeof(UX_MEMORY_BLOCK))
        {

            /* Setup the leftover memory block.  */
            leftover_memory_block = (UX_MEMORY_BLOCK *) (((ALIGN_TYPE) new_memory_block + sizeof(UX_MEMORY_BLOCK) + memory_size_requested) & 0xFFFFFFFFu);
            leftover_memory_block -> ux_memory_block_next =  new_memory_block -> ux_memory_block_next;
            leftover_memory_block -> ux_memory_block_previous =  new_memory_block;
            leftover_memory_block -> ux_memory_block_size =  leftover - (ULONG)sizeof(UX_MEMORY_BLOCK);
            leftover_memory_block -> ux_memory_block_status =  UX_MEMORY_UNUSED;

            new_memory_block -> ux_memory_block_next =  leftover_memory_block;
            new_memory_block -> ux_memory_block_size -=  leftover;
        }

        /* Declare how much memory we removed from the pool.  */
        memory_removed_from_pool =  new_memory_block -> ux_memory_block_size + (ULONG)sizeof(UX_MEMORY_BLOCK);

        /* The new memory block is the one we give to the user.  */
        memory_block =  new_memory_block;
    }

    /* The memory to be returned is after the block header.  */
    memory_buffer =  ((UCHAR *) memory_block) + sizeof(UX_MEMORY_BLOCK);

    /* Clear the memory block.  */
    _ux_utility_memory_set(memory_buffer, 0, memory_size_requested); /* Use case of memset is verified. */

    /* Update the memory free in the pool.  */
    if (_ux_system -> ux_system_cache_safe_memory_pool_start == _ux_system -> ux_system_regular_memory_pool_start)
    {

        /* There is only one memory pool.  */
        _ux_system -> ux_system_regular_memory_pool_free -= memory_removed_from_pool;
    }
    else
    {

       switch (memory_cache_flag)
       {

            case UX_CACHE_SAFE_MEMORY:
                /* Update the amount of free memory in the cache safe memory pool.  */
                _ux_system -> ux_system_cache_safe_memory_pool_free -= memory_removed_from_pool;

            break;

            default:
                /* Update the amount of free memory in the regular memory pool.  */
                _ux_system -> ux_system_regular_memory_pool_free -= memory_removed_from_pool;
            break;

        }
    }

#ifdef UX_ENABLE_MEMORY_STATISTICS

    /* Update allocate count, total size.  */
    if (memory_cache_flag == UX_REGULAR_MEMORY)
    {
        _ux_system -> ux_system_regular_memory_pool_alloc_count ++;
        _ux_system -> ux_system_regular_memory_pool_alloc_total += memory_size_requested;
        if (_ux_system -> ux_system_regular_memory_pool_alloc_max_count < _ux_system -> ux_system_regular_memory_pool_alloc_count)
            _ux_system -> ux_system_regular_memory_pool_alloc_max_count = _ux_system -> ux_system_regular_memory_pool_alloc_count;
        if (_ux_system -> ux_system_regular_memory_pool_alloc_max_total < _ux_system -> ux_system_regular_memory_pool_alloc_total)
            _ux_system -> ux_system_regular_memory_pool_alloc_max_total = _ux_system -> ux_system_regular_memory_pool_alloc_total;
    }
    else
    {
        _ux_system -> ux_system_cache_safe_memory_pool_alloc_count ++;
        _ux_system -> ux_system_cache_safe_memory_pool_alloc_total += memory_size_requested;
        if (_ux_system -> ux_system_cache_safe_memory_pool_alloc_max_count < _ux_system -> ux_system_cache_safe_memory_pool_alloc_count)
            _ux_system -> ux_system_cache_safe_memory_pool_alloc_max_count = _ux_system -> ux_system_cache_safe_memory_pool_alloc_count;
        if (_ux_system -> ux_system_cache_safe_memory_pool_alloc_max_total < _ux_system -> ux_system_cache_safe_memory_pool_alloc_total)
            _ux_system -> ux_system_cache_safe_memory_pool_alloc_max_total = _ux_system -> ux_system_cache_safe_memory_pool_alloc_total;
    }

    /* Log max usage of regular memory pool.  */
    memory_removed_from_pool = (ALIGN_TYPE)_ux_system -> ux_system_regular_memory_pool_start -
                               (ALIGN_TYPE)_ux_system -> ux_system_regular_memory_pool_base;
    if (memory_removed_from_pool > _ux_system -> ux_system_regular_memory_pool_max_start_offset)
        _ux_system -> ux_system_regular_memory_pool_max_start_offset = memory_removed_from_pool;
    if (_ux_system -> ux_system_regular_memory_pool_min_free > _ux_system -> ux_system_regular_memory_pool_free)
        _ux_system -> ux_system_regular_memory_pool_min_free = _ux_system -> ux_system_regular_memory_pool_free;

    /* Log max usage of cache safe memory pool.  */
    memory_removed_from_pool = (ALIGN_TYPE)_ux_system -> ux_system_cache_safe_memory_pool_start -
                               (ALIGN_TYPE)_ux_system -> ux_system_cache_safe_memory_pool_base;
    if (memory_removed_from_pool > _ux_system -> ux_system_cache_safe_memory_pool_max_start_offset)
        _ux_system -> ux_system_cache_safe_memory_pool_max_start_offset = memory_removed_from_pool;
    if (_ux_system -> ux_system_cache_safe_memory_pool_min_free > _ux_system -> ux_system_cache_safe_memory_pool_free)
        _ux_system -> ux_system_cache_safe_memory_pool_min_free = _ux_system -> ux_system_cache_safe_memory_pool_free;

#endif

    /* Release the protection.  */
    _ux_system_mutex_off(&_ux_system -> ux_system_mutex);

    /* The memory block pointer contains a memory area properly
       aligned.  */
    return(memory_buffer);
}                                
