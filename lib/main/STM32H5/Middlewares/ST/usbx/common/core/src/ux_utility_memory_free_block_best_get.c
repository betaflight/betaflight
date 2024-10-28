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
/**   USBX main stack                                                     */
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
/*    _ux_utility_memory_free_block_best_get              PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function returns the best free memory block.                   */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    memory_cache_flag                     Memory pool source            */ 
/*    memory_size_requested                 Size of memory requested      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Pointer to best free block                                          */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
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
/*                                                                        */
/**************************************************************************/
UX_MEMORY_BLOCK  *_ux_utility_memory_free_block_best_get(ULONG memory_cache_flag, 
                                                        ULONG memory_size_requested)
{

UX_MEMORY_BLOCK     *memory_block;
UX_MEMORY_BLOCK     *best_memory_block;
    

    /* Reset the free memory block.  */
    best_memory_block =  UX_NULL;
    
    /* Check the type of memory we need.  */
    switch (memory_cache_flag)
    {

        case UX_REGULAR_MEMORY            :

            /* Start at the beginning of the regular memory pool.  */
            memory_block =  _ux_system -> ux_system_regular_memory_pool_start;
            break;
            
        case UX_CACHE_SAFE_MEMORY       :

            /* Start at the beginning of the cache safe memory pool.  */
            memory_block =  _ux_system -> ux_system_cache_safe_memory_pool_start;
            break;
        
        default                            :
        
            /* Wrong memory type.  */
            return(UX_NULL);

    }

    /* Loop on all memory blocks from the beginning.  */
    while (memory_block != UX_NULL)
    {

        /* Check the memory block status.  */
        if (memory_block -> ux_memory_block_status == UX_MEMORY_UNUSED)
        {

            /* Check the size of this free block and see if it will 
               fit the memory requirement.  */
            if (memory_block -> ux_memory_block_size > memory_size_requested)
            {
                
                /* This memory block will do. Now see if it is the best.
                   The best memory block is the one whose memory is closest
                   to the memory requested.  */
                if (best_memory_block == UX_NULL)

                    /* Initialize the best block with the first free one.  */
                    best_memory_block =  memory_block;
                else
                {

                    if (memory_block -> ux_memory_block_size < best_memory_block -> ux_memory_block_size)

                        /* We have discovered a better fit block.  */
                        best_memory_block =  memory_block;
                }                    
            }
        }

        /* Search the next free block until the end.  */            
        memory_block =  memory_block -> ux_memory_block_next;
    }

    /* If no free memory block was found, the return value will be NULL.  */
    return(best_memory_block);        
}                                

