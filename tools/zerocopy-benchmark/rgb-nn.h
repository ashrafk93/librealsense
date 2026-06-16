// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.
#pragma once

#include <cstddef>

// Tiny CUDA helpers for the --rgb-nn benchmark mode. They model the front of an NN/GPU
// consumer: get an RGB frame onto the GPU and run a per-pixel "preprocess" over it. The
// kernel is a generic grayscale/normalize touch (NO proprietary model) -- its only job is to
// force the frame to actually be read on the GPU so the upload-vs-direct comparison is fair.
//
// Defined in rgb-nn-cuda.cu, which is compiled only in CUDA builds; callers guard use with
// RGBNN_HAVE_CUDA (set by CMake) so non-CUDA builds link without these symbols.

namespace rgbnn {

bool    cuda_available();
void *  dev_alloc( std::size_t bytes );                 // cudaMalloc
float * dev_alloc_float( std::size_t count );           // cudaMalloc for the preprocess output
void    dev_free( void * p );                           // cudaFree

// cudaMemcpy host->device of `bytes`, timed with cudaEvents (GPU-side, immune to CPU
// scheduling jitter). Returns elapsed milliseconds. This is the upload zero-copy eliminates.
double  upload_ms( void * dev, const void * host, std::size_t bytes );

// Stand-in preprocess: read each RGB8 pixel once, write a normalized luma to `out_dev`. Returns
// the kernel time in milliseconds, measured with cudaEvents (GPU-side, not host clock). Models a
// NN reading its input once. `rgb_dev` is the uploaded buffer or the frame's get_gpu_data() ptr.
double  preprocess( const unsigned char * rgb_dev, float * out_dev, int width, int height );

void    sync();  // cudaDeviceSynchronize

}  // namespace rgbnn
