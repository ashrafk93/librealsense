// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "rggb-converter.h"
#include <src/core/video.h>     // video_stream_profile_interface
#include <src/image.h>          // get_image_bpp
#include <src/stream.h>         // struct rs2_stream_profile (->profile)
#include <librealsense2/hpp/rs_frame.hpp>   // rs2::video_stream_profile
#include <cstdio>
#include <thread>
#include <vector>

#ifdef RS2_USE_CUDA
#include "cuda/cuda-rggb.cuh"
#include "rsutils/accelerators/gpu.h"   // rsutils::rs2_is_cuda_available
#endif

namespace librealsense
{
    void rggb_converter::init_profiles_info( const rs2::frame * f )
    {
        auto p = f->get_profile();
        if( p.get() != _source_stream_profile.get() )
        {
            _source_stream_profile = p;

            // Source dimensions. RAW8 passthrough is 1 byte/pixel and the V4L2 profile width is
            // the padded transport width (e.g. 1612). The real row stride is larger still — the
            // kernel pads each row to 64 bytes (1612 -> 1664) — so process_function() derives the
            // true stride from the frame's raw size rather than trusting width.
            if( auto vsp = p.as< rs2::video_stream_profile >() )
            {
                _src_width  = vsp.width();
                _src_height = vsp.height();
            }
            else
            {
                _src_width = _output_width;
            }

            _target_stream_profile = p.clone( p.stream_type(), p.stream_index(), _target_format );
            _target_bpp = get_image_bpp( _target_format ) / 8;

            // Set the target profile dims to the real (cropped) width, matching the advertised
            // resolution (registration's resolution_transform) and the frame allocated in
            // prepare_frame() -> advertised, profile, and produced frame are all _output_width.
            auto target_spi = (stream_profile_interface *)_target_stream_profile.get()->profile;
            if( auto target_vspi = dynamic_cast< video_stream_profile_interface * >( target_spi ) )
                target_vspi->set_dims( static_cast< uint32_t >( _output_width ),
                                       static_cast< uint32_t >( _src_height ) );
        }
    }

    rs2::frame rggb_converter::prepare_frame( const rs2::frame_source & source, const rs2::frame & f )
    {
        init_profiles_info( &f );
        auto vf = f.as< rs2::video_frame >();
        const int h = vf ? vf.get_height() : _src_height;
        // Allocate the output at the real width (_output_width), NOT the source's padded width
        // (the base prepare_frame uses the source frame width, which keeps the padding).
        return source.allocate_video_frame( _target_stream_profile, f, _target_bpp,
                                            _output_width, h, _output_width * _target_bpp, _extension_type );
    }

    rs2::frame rggb_converter::process_frame( const rs2::frame_source & source, const rs2::frame & f )
    {
        // Capture the source frame's *actual* byte count - the authoritative way to recover the
        // real row stride regardless of whether the backend keeps the padded V4L2 buffer (1664)
        // or repacks to width*bpp (1612).
        // The source frame's actual byte count gives the true row stride (data_size / height),
        // independent of whether the backend keeps 1664-padded rows or hands us a 1612 frame.
        _src_data_size = f.get_data_size();
        return functional_processing_block::process_frame( source, f );
    }

    void rggb_converter::process_function( uint8_t * const dest[], const uint8_t * source,
                                           int width, int height, int /*actual_size*/, int /*input_size*/ )
    {
        // The 'RGGB 8-bit' node actually carries MIPI RAW10 (4 px / 5 bytes). Recover the row
        // stride from the frame's real byte count (fallback: 64-byte-aligned source width), unpack
        // RAW10 -> 8-bit Bayer at the real sensor width, then demosaic into the output frame. The
        // output keeps the advertised width (e.g. 1612); the real image (e.g. 1288) sits on the
        // left and the remaining columns are zeroed (see debayer_rggb8's dst_stride_px).
        int src_stride = ( _src_width + 63 ) & ~63;
        if( _src_data_size > 0 && _src_height > 0 && ( _src_data_size % _src_height ) == 0 )
            src_stride = _src_data_size / _src_height;

        const int real_width = _output_width;          // real sensor width, e.g. 1288 (multiple of 4)

        // White-patch auto white balance: balance the brightest *unclipped* surfaces (the white/
        // light objects) to neutral, so white reads as white. Gray-world (scene average) leaves a
        // cast on white objects that are cooler than the average; white-patch targets them directly.
        // Two sparse passes over the packed source: (1) find the brightest unclipped green, (2)
        // average R/G/B over the top brightness band. EMA-smoothed; feeds CPU + CUDA debayer.
        {
            const int bl = _isp.black_level;
            auto bval = [&]( int x, int y ) -> int {
                int v = (int)source[ (size_t)y * src_stride + (size_t)( x >> 2 ) * 5 + ( x & 3 ) ] - bl;
                return v < 0 ? 0 : v;
            };
            const int step = 16;          // sample one RGGB cell every 16 px
            const int hi = 220;           // clip threshold: a channel at/above this is blown -> skip
            int gmax = 1;                 // brightest unclipped green among the samples
            for( int y = 0; y + 1 < height; y += step )
                for( int x = 0; x + 1 < real_width; x += step )
                {
                    const int g = ( bval( x + 1, y ) + bval( x, y + 1 ) ) >> 1;
                    if( g < hi && g > gmax ) gmax = g;
                }
            const int gthr = ( gmax * 3 ) / 5;   // top ~40% brightness band = light/white surfaces
            double sR = 0, sG = 0, sB = 0;
            long n = 0;
            for( int y = 0; y + 1 < height; y += step )
                for( int x = 0; x + 1 < real_width; x += step )   // x,y even -> land on R sites
                {
                    int rr  = bval( x, y );                                // R site (BGGR: this is B)
                    const int gg2 = ( bval( x + 1, y ) + bval( x, y + 1 ) ) >> 1;  // (Gr + Gb) / 2
                    int bb  = bval( x + 1, y + 1 );                        // B site (BGGR: this is R)
                    if( _isp.swap_rb ) { int t = rr; rr = bb; bb = t; }    // BGGR: real R/B are swapped
                    if( gg2 < gthr )         continue;                     // not a bright surface
                    if( rr >= hi || gg2 >= hi || bb >= hi ) continue;      // any channel clipped
                    sR += rr;  sG += gg2;  sB += bb;  ++n;
                }
            if( n > 20 && sR > 1.0 && sB > 1.0 )   // enough bright-surface samples to trust it
            {
                const double mR = sR / n, mG = sG / n, mB = sB / n;
                auto clampg = []( float g ) { return g < 0.5f ? 0.5f : ( g > 4.f ? 4.f : g ); };
                // No warm bias: validated against the captured raw, the unbiased white-patch gains
                // (gR~2.15, gB~1.72) land the bright surfaces neutral. A bias only re-introduces a tint.
                const float tR = clampg( float( mG / mR ) ), tB = clampg( float( mG / mB ) );
                const float a = 0.1f;                              // EMA: converges in ~30 frames
                _awb_gain_r += a * ( tR - _awb_gain_r );
                _awb_gain_b += a * ( tB - _awb_gain_b );
            }
        }
        rggb::isp_params isp = _isp;     // per-frame ISP with the auto-white-balance gains
        isp.gain_r = _awb_gain_r;
        isp.gain_g = 1.f;
        isp.gain_b = _awb_gain_b;

#ifdef RS2_USE_CUDA
        // GPU path: one fused kernel does RAW10 unpack + RGGB demosaic + gain + tone, writing the
        // output frame in place under zero-copy (no host round-trip). Output is tight (width*3).
        if( rsutils::rs2_is_cuda_available() )
        {
            rscuda::rggb_isp_params ip{};
            ip.black_level = isp.black_level;
            ip.gain_r = isp.gain_r;  ip.gain_g = isp.gain_g;  ip.gain_b = isp.gain_b;
            ip.digital_gain = isp.digital_gain;  ip.gamma = isp.gamma;  ip.s_curve = isp.s_curve;
            ip.saturation = isp.saturation;  ip.contrast = isp.contrast;
            ip.swap_rb = isp.swap_rb ? 1 : 0;
            for( int i = 0; i < 9; ++i ) ip.ccm[i] = isp.ccm[i];
            rscuda::rggb_debayer_raw10_cuda( source, src_stride, real_width, height, dest[0], width * 3, ip );
            return;
        }
#endif

        _bayer.resize( static_cast< size_t >( real_width ) * height );
        rggb::unpack_raw10( source, src_stride, real_width, height, _bayer.data() );
        // Demosaic is the CPU hot path; split it across row bands (the Jetson has spare cores).
        {
            const int nthreads = 4;
            const int band = ( height + nthreads - 1 ) / nthreads;
            std::vector< std::thread > pool;
            for( int t = 1; t < nthreads; ++t )
            {
                const int b0 = t * band, b1 = ( height < b0 + band ) ? height : b0 + band;
                if( b0 >= b1 ) break;
                pool.emplace_back( [&, b0, b1]() {
                    rggb::debayer_rggb8( _bayer.data(), real_width, real_width, height, dest[0], isp, width, b0, b1 );
                } );
            }
            rggb::debayer_rggb8( _bayer.data(), real_width, real_width, height, dest[0], isp, width,
                                 0, ( height < band ) ? height : band );
            for( auto & th : pool ) th.join();
        }
    }
}
