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

            // Source row width in bytes. RAW8 passthrough is 1 byte/pixel, so the V4L2 profile
            // width (e.g. 1612, padded) is the stride we must step by when reading the mosaic.
            int src_height = 0;
            if( auto vsp = p.as< rs2::video_stream_profile >() )
            {
                _src_stride = vsp.width();
                src_height  = vsp.height();
            }
            else
            {
                _src_stride = _output_width;
            }

            _target_stream_profile = p.clone( p.stream_type(), p.stream_index(), _target_format );
            _target_bpp = get_image_bpp( _target_format ) / 8;

            // Crop the advertised output to the real sensor width (drop the transport padding).
            auto target_spi = (stream_profile_interface *)_target_stream_profile.get()->profile;
            if( auto target_vspi = dynamic_cast< video_stream_profile_interface * >( target_spi ) )
                target_vspi->set_dims( static_cast< uint32_t >( _output_width ),
                                       static_cast< uint32_t >( src_height ) );
        }
    }

    void rggb_converter::process_function( uint8_t * const dest[], const uint8_t * source,
                                           int width, int height, int /*actual_size*/, int /*input_size*/ )
    {
        // width/height are the cropped target dims; _src_stride steps across the padded source.
        rggb::debayer_rggb8( source, _src_stride, width, height, dest[0], _isp );
    }
}
