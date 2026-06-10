// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "rggb-converter.h"
#include <src/core/video.h>     // video_stream_profile_interface
#include <src/image.h>          // get_image_bpp
#include <src/stream.h>         // struct rs2_stream_profile (->profile)
#include <librealsense2/hpp/rs_frame.hpp>   // rs2::video_stream_profile
#include <cstdio>

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

#ifdef RS2_USE_CUDA
        // GPU path: one fused kernel does RAW10 unpack + RGGB demosaic + gain + tone, writing the
        // output frame in place under zero-copy (no host round-trip). Output is tight (width*3).
        if( rsutils::rs2_is_cuda_available() )
        {
            const rscuda::rggb_isp_params ip{ _isp.black_level, _isp.gain_r, _isp.gain_g, _isp.gain_b,
                                              _isp.digital_gain, _isp.gamma, _isp.saturation, _isp.contrast };
            rscuda::rggb_debayer_raw10_cuda( source, src_stride, real_width, height, dest[0], width * 3, ip );
            return;
        }
#endif

        _bayer.resize( static_cast< size_t >( real_width ) * height );
        rggb::unpack_raw10( source, src_stride, real_width, height, _bayer.data() );
        rggb::debayer_rggb8( _bayer.data(), real_width, real_width, height, dest[0], _isp, /*dst_stride_px*/ width );
    }
}
