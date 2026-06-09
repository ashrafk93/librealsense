// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "rggb-converter.h"
#include <src/core/video.h>     // video_stream_profile_interface
#include <src/image.h>          // get_image_bpp
#include <src/stream.h>         // struct rs2_stream_profile (->profile)
#include <librealsense2/hpp/rs_frame.hpp>   // rs2::video_stream_profile

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

    void rggb_converter::process_function( uint8_t * const dest[], const uint8_t * source,
                                           int width, int height, int /*actual_size*/, int input_size )
    {
        // Source row stride = the V4L2 bytesperline, which the kernel pads to a 64-byte boundary
        // (1612 -> 1664; confirmed via VIDIOC_TRY_FMT). Do NOT derive it from input_size /
        // RS2_FRAME_METADATA_RAW_FRAME_SIZE: that is the *logical* size (width*height) with the
        // row padding removed, so using it shears the demosaic across the frame.
        (void)input_size;
        const int stride = ( _src_width + 63 ) & ~63;
        rggb::debayer_rggb8( source, stride, width, height, dest[0], _isp );
    }
}
