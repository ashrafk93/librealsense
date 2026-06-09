// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
#pragma once

#include "color-formats-converter.h"   // color_converter
#include "rggb-debayer.h"              // rggb::isp_params, unpack_raw10, debayer_rggb8

#include <vector>

namespace librealsense
{
    // Processing block for the D401 GMSL "dual RGB" mode: an 8-bit RGGB Bayer frame delivered
    // by the FW RAW8 CSI passthrough (V4L2 fourcc 'RGGB', mapped to RS2_FORMAT_RAW8) is cropped
    // from the padded transport width (e.g. 1612) to the real sensor width (e.g. 1288), demosaiced
    // and white-balanced to RGB8. Unlike the YUV color converters this changes resolution, so it
    // overrides init_profiles_info() to set the cropped output dimensions (cf. rotation_transform).
    class LRS_EXTENSION_API rggb_converter : public color_converter
    {
    public:
        rggb_converter( rs2_format target_format,
                        int output_width,
                        rggb::isp_params isp = {},
                        rs2_stream target_stream = RS2_STREAM_COLOR )
            : color_converter( "RGGB Converter", target_format, target_stream )
            , _output_width( output_width )
            , _isp( isp )
        {
        }

    protected:
        void init_profiles_info( const rs2::frame * f ) override;
        // Allocate the output at the real (cropped) width, not the source width (the base uses the
        // source frame's width, which would keep the transport padding).
        rs2::frame prepare_frame( const rs2::frame_source & source, const rs2::frame & f ) override;
        rs2::frame process_frame( const rs2::frame_source & source, const rs2::frame & f ) override;
        void process_function( uint8_t * const dest[], const uint8_t * source,
                               int width, int height, int actual_size, int input_size ) override;

        int              _output_width;     // cropped output width in px (real sensor width, e.g. 1288)
        int              _src_width = 0;    // source profile width in px (e.g. 1612, padded)
        int              _src_height = 0;   // source profile height in px (e.g. 808)
        int              _src_data_size = 0;// source frame's actual byte count (authoritative for stride)
        rggb::isp_params _isp;
        std::vector< uint8_t > _bayer;      // scratch: RAW10 unpacked to 8-bit Bayer (real_width*height)
    };
}
