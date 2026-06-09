// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "rggb-converter.h"
#include <src/core/video.h>     // video_stream_profile_interface
#include <src/image.h>          // get_image_bpp
#include <src/stream.h>         // struct rs2_stream_profile (->profile)
#include <librealsense2/hpp/rs_frame.hpp>   // rs2::video_stream_profile
#include <cstdio>

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

            // NOTE: output keeps the source width for now. The ~324 px of transport padding on
            // the right are demosaiced harmlessly; cropping to _output_width (1288) by changing
            // the target dims here would desync from the advertised profile (handled separately).
        }
    }

    rs2::frame rggb_converter::process_frame( const rs2::frame_source & source, const rs2::frame & f )
    {
        // Capture the source frame's *actual* byte count - the authoritative way to recover the
        // real row stride regardless of whether the backend keeps the padded V4L2 buffer (1664)
        // or repacks to width*bpp (1612).
        _src_data_size = f.get_data_size();
        static bool logged = false;
        if( ! logged )
        {
            logged = true;
            auto vf = f.as< rs2::video_frame >();
            std::fprintf( stderr,
                "[RGGB] src w=%d h=%d stride_in_bytes=%d bpp=%d data_size=%d  -> derived row stride=%d\n",
                vf ? vf.get_width() : -1, vf ? vf.get_height() : -1,
                vf ? vf.get_stride_in_bytes() : -1, vf ? vf.get_bytes_per_pixel() : -1,
                _src_data_size, ( _src_height > 0 ) ? _src_data_size / _src_height : -1 );
        }
        return functional_processing_block::process_frame( source, f );
    }

    void rggb_converter::process_function( uint8_t * const dest[], const uint8_t * source,
                                           int width, int height, int /*actual_size*/, int /*input_size*/ )
    {
        // Row stride from the source frame's real size (data_size / height). Falls back to the
        // kernel's 64-byte-aligned width if the size isn't usable. Deriving from RAW_FRAME_SIZE
        // metadata is unreliable, so we use the buffer's true byte count captured in process_frame.
        int stride = ( _src_width + 63 ) & ~63;
        if( _src_data_size > 0 && _src_height > 0 && ( _src_data_size % _src_height ) == 0 )
            stride = _src_data_size / _src_height;
        rggb::debayer_rggb8( source, stride, width, height, dest[0], _isp );
    }
}
