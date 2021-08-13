# create a hardware_NAME_headers target (see pico_pico_simple_hardware_headers_target)
# create a hardware_NAME target (see pico_pico_simple_hardware_target)
macro(pico_simple_hardware_target NAME)
    pico_simple_hardware_headers_target(${NAME})
    pico_simple_hardware_impl_target(${NAME})
endmacro()

# create an INTERFACE library named target, and define LIB_TARGET=1 (upper case) as a compile option
function(pico_add_impl_library target)
    add_library(${target} INTERFACE)
    string(TOUPPER ${target} TARGET_UPPER)
    target_compile_definitions(${target} INTERFACE LIB_${TARGET_UPPER}=1)
endfunction()

# create an INTERFACE library named hardware_NAME_headers INTERFACE library if it doesn't already exist,
#        and add include/ relative to the calling directory to the includes.
#        and hardware_structs and hardware_claim as dependencies of the library
macro(pico_simple_hardware_headers_target NAME)
    if (NOT TARGET hardware_${NAME}_headers)
        add_library(hardware_${NAME}_headers INTERFACE)

        target_include_directories(hardware_${NAME}_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
        target_link_libraries(hardware_${NAME}_headers INTERFACE pico_base_headers)
        if (NOT PICO_NO_HARDWARE)
            target_link_libraries(hardware_${NAME}_headers INTERFACE hardware_structs hardware_claim)
        endif()
    endif()
endmacro()

# create an INTERFACE library named hardware_NAME if it doesn't exist, along with a hardware_NAME_headers
#        INTERFACE library that it depends on. The hardware_NAME_headers library add include/ relative to
#        and pico_base_headers, and harddware_structs as a dependency of the library
macro(pico_simple_hardware_headers_only_target NAME)
    if (NOT TARGET hardware_${NAME})
        # Choosing not to add LIB_HARDWARE_ defines to avoid command line bloat pending a need (they aren't
        #   super interesting except to determine functionality as they are mostly passive accessors, however
        #   they could be useful to determine if the header is available.
        # pico_add_sdk_impl_library(hardware_${NAME})
        add_library(hardware_${NAME} INTERFACE)

        target_include_directories(hardware_${NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
        target_link_libraries(hardware_${NAME} INTERFACE pico_base_headers)
        if (NOT PICO_NO_HARDWARE)
            target_link_libraries(hardware_${NAME} INTERFACE hardware_structs)
        endif()
    endif()
endmacro()

# create an INTERFACE library named hardware_NAME if it doesn't exist, dependent on a pre-existing  hardware_NAME_headers
#        INTERFACE library and pico_platform. The file NAME.c relative to the caller is added to the C sources for the hardware_NAME
macro(pico_simple_hardware_impl_target NAME)
    if (NOT TARGET hardware_${NAME})
        # Choosing not to add LIB_HARDWARE_ defines to avoid command line bloat pending a need (they aren't
        #   super interesting except to determine functionality as they are mostly passive accessors, however
        #   they could be useful to determine if the header is available.
        # pico_add_sdk_impl_library(hardware_${NAME})
        add_library(hardware_${NAME} INTERFACE)

        target_sources(hardware_${NAME} INTERFACE
                ${CMAKE_CURRENT_LIST_DIR}/${NAME}.c
                )

        target_link_libraries(hardware_${NAME} INTERFACE hardware_${NAME}_headers pico_platform)
    endif()
endmacro()

set(PICO_PROMOTE_COMMON_SCOPE_VARS
            PICO_INCLUDE_DIRS
            PICO_SDK_POST_LIST_DIRS
            PICO_SDK_POST_LIST_FILES
            PICO_CONFIG_HEADER_FILES
            PICO_RP2040_CONFIG_HEADER_FILES
    )

macro(pico_promote_common_scope_vars)
    set(PICO_PROMOTE_COMMON_SCOPE_VARS ${PICO_PROMOTE_COMMON_SCOPE_VARS} PARENT_SCOPE)
    foreach(VAR IN LISTS PICO_PROMOTE_COMMON_SCOPE_VARS)
        SET(${VAR} ${${VAR}} PARENT_SCOPE)
    endforeach()
endmacro()

function(pico_add_subdirectory subdir)
    # todo add option to disable skip flag
    string(TOUPPER ${subdir} subdir_upper)
    set(replace_flag SKIP_${subdir_upper})
    if (NOT ${replace_flag})
        add_subdirectory(${subdir})
    else ()
        message("Not including ${subdir} because ${replace_flag} defined.")
    endif ()
    pico_promote_common_scope_vars()
endfunction()

function(pico_add_dis_output TARGET)
    add_custom_command(TARGET ${TARGET} POST_BUILD
            COMMAND ${CMAKE_OBJDUMP} -h ${CMAKE_BINARY_DIR}/${TARGET}${CMAKE_EXECUTABLE_SUFFIX} >${TARGET}.dis
            COMMAND ${CMAKE_OBJDUMP} -d ${CMAKE_BINARY_DIR}/${TARGET}${CMAKE_EXECUTABLE_SUFFIX} >>${TARGET}.dis
            )
endfunction()

function(pico_wrap_function TARGET FUNCNAME)
    if ("${FUNCNAME}" STREQUAL "malloc")
    MESSAGE("${BoldYellow}Warning: Not using pico malloc${ColourReset}")
    elseif("${FUNCNAME}" STREQUAL "calloc")
        MESSAGE("${BoldYellow}Warning: Not using pico calloc${ColourReset}")
    elseif("${FUNCNAME}" STREQUAL "free")
        MESSAGE("${BoldYellow}Warning: Not using pico free${ColourReset}")
    else()
        target_link_options(${TARGET} INTERFACE "LINKER:--wrap=${FUNCNAME}")
    endif()
endfunction()

# add map file generation for the given target
function(pico_add_map_output TARGET)
    get_target_property(target_type ${TARGET} TYPE)
    if ("EXECUTABLE" STREQUAL "${target_type}")
        target_link_options(${TARGET} PRIVATE "LINKER:-Map=$<TARGET_PROPERTY:NAME>${CMAKE_EXECUTABLE_SUFFIX}.map")
    else ()
        target_link_options(${TARGET} INTERFACE "LINKER:-Map=$<TARGET_PROPERTY:NAME>${CMAKE_EXECUTABLE_SUFFIX}.map")
    endif ()
endfunction()

function(pico_add_doxygen SOURCE_DIR)
    # ignore doxygen
endfunction()

function(pico_add_doxygen_exclude SOURCE_DIR)
    # ignore doxygen
endfunction()

function(pico_message_debug MESSAGE)
    # ignore debug
endfunction()