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

            // Crop the advertised output to the real sensor width (drop the transport padding).
            auto target_spi = (stream_profile_interface *)_target_stream_profile.get()->profile;
            if( auto target_vspi = dynamic_cast< video_stream_profile_interface * >( target_spi ) )
                target_vspi->set_dims( static_cast< uint32_t >( _output_width ),
                                       static_cast< uint32_t >( _src_height ) );
        }
    }

    void rggb_converter::process_function( uint8_t * const dest[], const uint8_t * source,
                                           int width, int height, int /*actual_size*/, int input_size )
    {
        // width/height are the cropped target dims (e.g. 1288x808). The source row stride is the
        // padded transport stride: prefer the exact value from the frame's raw size, else fall
        // back to the kernel's 64-byte row alignment of the source width (e.g. 1612 -> 1664).
        int stride;
        if( input_size > 0 && _src_height > 0 && ( input_size % _src_height ) == 0 )
            stride = input_size / _src_height;
        else
            stride = ( _src_width + 63 ) & ~63;

        rggb::debayer_rggb8( source, stride, width, height, dest[0], _isp );
    }
}
