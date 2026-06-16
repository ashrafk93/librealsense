# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2025 RealSense, Inc. All Rights Reserved.
#
# Built as part of the tree (BUILD_TOOLS=ON) so it links the just-built realsense2,
# which is what makes it pick up whichever CUDA / zero-copy configuration the tree was
# built with.
cmake_minimum_required(VERSION 3.8)
project(zerocopy-benchmark)

add_executable(rs-zerocopy-benchmark zerocopy-benchmark.cpp)
set_property(TARGET rs-zerocopy-benchmark PROPERTY CXX_STANDARD 14)
target_link_libraries(rs-zerocopy-benchmark ${DEPENDENCIES})
include_directories(../../common ../../third-party)

# The --rgb-nn mode needs a CUDA kernel + cudaMemcpy. Compile it only in CUDA builds; the .cpp
# guards all CUDA use behind RGBNN_HAVE_CUDA, so non-CUDA builds link cleanly and --rgb-nn just
# reports that it needs a CUDA build. (CUDA language is already enabled by cuda_config.cmake.)
if(BUILD_WITH_CUDA)
    target_sources(rs-zerocopy-benchmark PRIVATE rgb-nn-cuda.cu)
    target_compile_definitions(rs-zerocopy-benchmark PRIVATE RGBNN_HAVE_CUDA)
    target_link_libraries(rs-zerocopy-benchmark ${CUDA_LIBRARIES})
endif()

set_target_properties(rs-zerocopy-benchmark PROPERTIES FOLDER "Tools")
install(TARGETS rs-zerocopy-benchmark RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
