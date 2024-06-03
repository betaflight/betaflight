# Add the required subdirectory and register it for linkage
function(add_azrtos_component_dir dirname)
    # Store the current list in a temp
    set(tmp ${azrtos_targets})
    # Add the subdir
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/${dirname})
    # If there is a linker script defined, use it
    if(EXISTS ${LINKER_SCRIPT})
        target_link_options(${dirname} INTERFACE -T ${LINKER_SCRIPT})
    endif()
    # Add this target into the temp
    list(APPEND tmp "azrtos::${dirname}")
    # Copy the temp back up to the parent list
    set(azrtos_targets ${tmp} PARENT_SCOPE)
endfunction()