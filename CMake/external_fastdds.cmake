cmake_minimum_required(VERSION 3.16.3)  # same as in FastDDS (U20)
include(FetchContent)

# We use a function to enforce a scoped variables creation only for FastDDS build (i.e turn off BUILD_SHARED_LIBS which is used on LRS build as well)
function(get_fastdds)

    # Mark new options from FetchContent as advanced options
    mark_as_advanced(FETCHCONTENT_QUIET)
    mark_as_advanced(FETCHCONTENT_BASE_DIR)
    mark_as_advanced(FETCHCONTENT_FULLY_DISCONNECTED)
    mark_as_advanced(FETCHCONTENT_UPDATES_DISCONNECTED)

    message(CHECK_START  "Fetching fastdds...")
    list(APPEND CMAKE_MESSAGE_INDENT "  ")  # Indent outputs

    FetchContent_Declare(
      fastdds
      GIT_REPOSITORY https://github.com/eProsima/Fast-DDS.git
      # 2.10.x is eProsima's last LTS version that still supports U20
      # 2.10.4 has specific modifications based on support provided, but it has some incompatibility
      # with the way we clone (which works with v2.11+), so they made a fix and tagged it for us:
      # Once they have 2.10.5 we should move to it
      GIT_TAG        v2.10.4-realsense
      GIT_SUBMODULES ""     # Submodules will be cloned as part of the FastDDS cmake configure stage
      GIT_SHALLOW ON        # No history needed
      SOURCE_DIR ${CMAKE_BINARY_DIR}/third-party/fastdds
    )

    # Set FastDDS internal variables
    # We use cached variables so the default parameter inside the sub directory will not override the required values
    # We add "FORCE" so that is a previous cached value is set our assignment will override it.
    set(THIRDPARTY_Asio FORCE CACHE INTERNAL "" FORCE)
    set(THIRDPARTY_fastcdr FORCE CACHE INTERNAL "" FORCE)
    set(THIRDPARTY_TinyXML2 FORCE CACHE INTERNAL "" FORCE)
    set(COMPILE_TOOLS OFF CACHE INTERNAL "" FORCE)
    set(BUILD_TESTING OFF CACHE INTERNAL "" FORCE)
    set(SQLITE3_SUPPORT OFF CACHE INTERNAL "" FORCE)
    #set(ENABLE_OLD_LOG_MACROS OFF CACHE INTERNAL "" FORCE)  doesn't work
    set(FASTDDS_STATISTICS OFF CACHE INTERNAL "" FORCE)
    # Enforce NO_TLS to disable SSL: if OpenSSL is found, it will be linked to, and we don't want it!
    set(NO_TLS ON CACHE INTERNAL "" FORCE)

    # Set special values for FastDDS sub directory
    set(BUILD_SHARED_LIBS OFF)
    set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/fastdds/fastdds_install)
    set(CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR}/fastdds/fastdds_install)

    # Get fastdds
    FetchContent_MakeAvailable(fastdds)

    # GCC 14 / libstdc++-15 (Ubuntu 26.04 "resolute") removed transitive <cstdint>
    # includes from many std headers. FastDDS 2.10.4 uses uint8_t (e.g. in
    # DDSFilterCompoundCondition.hpp) without explicitly including <cstdint>,
    # which fails to compile. Force-include <cstdint> ONLY for FastDDS's own
    # targets, and only for their CXX translation units -- <cstdint> is a C++
    # header, so applying the flag directory-wide (via add_compile_options)
    # breaks C compilation elsewhere in the tree (e.g. third-party/glad/glad.c
    # linked into realsense2-gl fatal-errors with "cstdint: No such file").
    #
    # We discover FastDDS's targets by walking BUILDSYSTEM_TARGETS under its
    # source dir instead of hardcoding a list, so a future FastDDS bump that
    # adds new libraries won't silently regress resolute.
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        set(_fastdds_all_targets)
        set(_fastdds_pending "${fastdds_SOURCE_DIR}")
        while(_fastdds_pending)
            list(POP_FRONT _fastdds_pending _dir)
            get_property(_here_targets DIRECTORY "${_dir}" PROPERTY BUILDSYSTEM_TARGETS)
            list(APPEND _fastdds_all_targets ${_here_targets})
            get_property(_here_subdirs DIRECTORY "${_dir}" PROPERTY SUBDIRECTORIES)
            list(APPEND _fastdds_pending ${_here_subdirs})
        endwhile()

        foreach(_fastdds_tgt ${_fastdds_all_targets})
            # Only compilable targets support target_compile_options PRIVATE;
            # INTERFACE / UTILITY targets have no sources of their own and
            # would fatal on the call.
            get_target_property(_fastdds_tgt_type ${_fastdds_tgt} TYPE)
            if(_fastdds_tgt_type MATCHES "^(STATIC|SHARED|MODULE|OBJECT)_LIBRARY$|^EXECUTABLE$")
                target_compile_options(${_fastdds_tgt} PRIVATE
                                       $<$<COMPILE_LANGUAGE:CXX>:-include cstdint>)
            endif()
        endforeach()
    endif()

    # Mark new options from FetchContent as advanced options
    mark_as_advanced(FETCHCONTENT_SOURCE_DIR_FASTDDS)
    mark_as_advanced(FETCHCONTENT_UPDATES_DISCONNECTED_FASTDDS)

    # place FastDDS project with other 3rd-party projects
    set_target_properties(fastcdr fastrtps foonathan_memory PROPERTIES
                          FOLDER "3rd Party/fastdds")

    list(POP_BACK CMAKE_MESSAGE_INDENT) # Unindent outputs

    add_library(dds INTERFACE)
    target_link_libraries( dds INTERFACE fastcdr fastrtps )
    
    disable_third_party_warnings(fastcdr)  
    disable_third_party_warnings(fastrtps)  

    add_definitions(-DBUILD_WITH_DDS)

    install(TARGETS dds fastrtps eProsima_atomic EXPORT realsense2Targets)
    
    # fastcdr is installed separately because it cannot be exported to realsense2Targets - it is already exported in fastdds
    # install in order to set ARCHIVE DESTINATION - to put libfastcdr.a into the x86_64 folder
    install(TARGETS fastcdr 
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
    message(CHECK_PASS "Done")
endfunction()


pop_security_flags()

# Trigger the FastDDS build
get_fastdds()

if(BUILD_WITH_DDS)
   set(REALSENSE2_DDS_DEPENDENCIES
   	 "include(CMakeFindDependencyMacro)\n
	  find_dependency(fastcdr CONFIG REQUIRED)\n
	  find_dependency(foonathan_memory CONFIG REQUIRED)\n"
	  )
else()
  set(REALSENSE2_DDS_DEPENDENCIES "")
endif()

push_security_flags()
