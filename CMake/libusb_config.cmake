if (NOT TARGET usb)
    # Check if local libusb exists first and we're on macOS
    set(LOCAL_LIBUSB_DIR "${CMAKE_SOURCE_DIR}/third-party/libusb")
    
    if(APPLE AND EXISTS "${LOCAL_LIBUSB_DIR}/CMakeLists.txt")
        # Use local libusb from third-party/libusb only on macOS
        message(STATUS "Using local libusb from third-party/libusb on macOS")
        
        # Add libusb as subdirectory
        add_subdirectory(${LOCAL_LIBUSB_DIR} ${CMAKE_BINARY_DIR}/third-party/libusb)
        
        # Create interface target that wraps the local libusb_static
        add_library(usb INTERFACE)
        target_link_libraries(usb INTERFACE libusb_static)
        
        # Add libusb_static to the export set so it can be installed
        install(TARGETS libusb_static EXPORT realsense2Targets
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            RUNTIME DESTINATION bin
        )
        
        set(USE_LOCAL_USB ON)
        
    else()
        # Use system libusb for all other platforms
        find_library(LIBUSB_LIB usb-1.0)
        find_path(LIBUSB_INC libusb.h HINTS PATH_SUFFIXES libusb-1.0)
        include(FindPackageHandleStandardArgs)
        find_package_handle_standard_args(usb "libusb not found; using external version" LIBUSB_LIB LIBUSB_INC)
        if (USB_FOUND AND NOT USE_EXTERNAL_USB)
            add_library(usb INTERFACE)
            target_include_directories(usb INTERFACE ${LIBUSB_INC})
            target_link_libraries(usb INTERFACE ${LIBUSB_LIB})
        else()
            include(CMake/external_libusb.cmake)
        endif()
    endif()
    
    install(TARGETS usb EXPORT realsense2Targets)
endif()
