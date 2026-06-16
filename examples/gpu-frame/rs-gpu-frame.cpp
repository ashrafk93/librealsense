// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

// rs-gpu-frame
// ------------
// Demonstrates rs2::frame::get_gpu_data() — the zero-copy GPU pointer API.
//
// On a build with BUILD_WITH_CUDA_ZEROCOPY running on an integrated GPU (Jetson), a frame's
// pixels live in GPU-mapped memory, so get_gpu_data() returns a CUDA device pointer that
// aliases the frame. You can hand that pointer straight to a CUDA kernel / TensorRT / NPP
// without a host->device copy. In every other configuration it returns null and you fall
// back to get_data() + your own upload — so the same code is correct everywhere.
//
// This example needs no CUDA toolchain to build: it only shows the API and the decision
// pattern. The commented block marks exactly where a GPU consumer would use the pointer.

#include <librealsense2/rs.hpp>
#include <iostream>

int main()
try
{
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream( RS2_STREAM_COLOR );  // RGB is the interesting case for GPU/NN consumers
    pipe.start( cfg );

    // Warm up so streaming/allocation has settled.
    for( int i = 0; i < 30; ++i )
        pipe.wait_for_frames();

    auto frames = pipe.wait_for_frames();
    auto color = frames.get_color_frame();

    const void * host_ptr = color.get_data();      // always valid (CPU pointer)
    const void * gpu_ptr  = color.get_gpu_data();  // device pointer, or null if unavailable

    std::cout << "Color frame " << color.get_width() << "x" << color.get_height()
              << "  host=" << host_ptr << "  gpu=" << gpu_ptr << "\n";

    if( gpu_ptr )
    {
        std::cout << "Zero-copy GPU pointer available — a CUDA/TensorRT consumer can read the\n"
                     "frame in place, no host->device copy.\n";
        // --- where a GPU consumer would use it (pseudo-code) ---
        //   my_cuda_preprocess<<<grid, block>>>( (const uint8_t*)gpu_ptr, width, height );
        //   tensorrt_context->setTensorAddress( "input", (void*)gpu_ptr );
        //   tensorrt_context->enqueueV3( stream );
        // The frame must stay alive (hold `color`) until the GPU work completes.
    }
    else
    {
        std::cout << "No GPU pointer (discrete GPU, or non-zero-copy build) — upload yourself:\n"
                     "  cudaMemcpy(d_input, host_ptr, size, cudaMemcpyHostToDevice);\n";
    }

    pipe.stop();
    return EXIT_SUCCESS;
}
catch( const rs2::error & e )
{
    std::cerr << "RealSense error calling " << e.get_failed_function()
              << "(" << e.get_failed_args() << "):\n    " << e.what() << "\n";
    return EXIT_FAILURE;
}
catch( const std::exception & e )
{
    std::cerr << e.what() << "\n";
    return EXIT_FAILURE;
}
