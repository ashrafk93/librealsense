// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include "rgb-nn.h"
#include <cuda_runtime.h>
#include <chrono>

namespace {

// Generic per-pixel preprocess: RGB8 -> normalized luma, reading each pixel once. Not a real
// model; just makes the GPU actually read the frame so the upload-vs-direct comparison measures
// the upload, not a no-op. (Mirrors how a consumer reads its input once into fast memory.)
__global__ void k_preprocess( const unsigned char * __restrict__ rgb,
                              float * __restrict__ out, int n )
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if( i >= n )
        return;
    out[i] = ( 0.299f * rgb[i * 3 + 0] + 0.587f * rgb[i * 3 + 1] + 0.114f * rgb[i * 3 + 2] ) / 255.f;
}

}  // namespace

namespace rgbnn {

bool cuda_available()
{
    int count = 0;
    return cudaGetDeviceCount( &count ) == cudaSuccess && count > 0;
}

void * dev_alloc( std::size_t bytes )
{
    void * p = nullptr;
    if( cudaMalloc( &p, bytes ) != cudaSuccess )
        return nullptr;
    return p;
}

float * dev_alloc_float( std::size_t count )
{
    return static_cast< float * >( dev_alloc( count * sizeof( float ) ) );
}

void dev_free( void * p )
{
    if( p )
        cudaFree( p );
}

// Reused GPU timing events (single-threaded benchmark). cudaEvent timestamps live on the GPU
// stream, so they measure real device duration and are immune to CPU scheduling jitter --
// essential for ~0.1 ms ops that host-side std::chrono timing can't measure reliably.
static cudaEvent_t g_ev_a = nullptr, g_ev_b = nullptr;
static void ensure_events()
{
    if( ! g_ev_a ) cudaEventCreate( &g_ev_a );
    if( ! g_ev_b ) cudaEventCreate( &g_ev_b );
}

double upload_ms( void * dev, const void * host, std::size_t bytes )
{
    ensure_events();
    cudaEventRecord( g_ev_a, 0 );
    cudaMemcpyAsync( dev, host, bytes, cudaMemcpyHostToDevice, 0 );
    cudaEventRecord( g_ev_b, 0 );
    cudaEventSynchronize( g_ev_b );
    float ms = 0.f;
    cudaEventElapsedTime( &ms, g_ev_a, g_ev_b );
    return ms;
}

double preprocess( const unsigned char * rgb_dev, float * out_dev, int width, int height )
{
    const int n = width * height;
    const int block = 256;
    const int grid = ( n + block - 1 ) / block;
    ensure_events();
    cudaEventRecord( g_ev_a, 0 );
    k_preprocess<<< grid, block, 0, 0 >>>( rgb_dev, out_dev, n );
    cudaEventRecord( g_ev_b, 0 );
    cudaEventSynchronize( g_ev_b );
    float ms = 0.f;
    cudaEventElapsedTime( &ms, g_ev_a, g_ev_b );
    return ms;
}

void sync()
{
    cudaDeviceSynchronize();
}

}  // namespace rgbnn
