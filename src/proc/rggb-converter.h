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
        // native_width : real sensor width after cropping the transport padding (e.g. 1288).
        // out_width/out_height : the requested OUTPUT resolution. The native image (native_width x
        //   source_height) is demosaiced, then center-cropped to the output aspect ratio and
        //   bilinear-scaled to out_width x out_height (crop-to-aspect + scale, no stretch). For the
        //   native output (out == native), the scale collapses to a straight copy. One converter
        //   instance is registered per output resolution.
        rggb_converter( rs2_format target_format,
                        int native_width,
                        int out_width,
                        int out_height,
                        rggb::isp_params isp = {},
                        rs2_stream target_stream = RS2_STREAM_COLOR )
            : color_converter( "RGGB Converter", target_format, target_stream )
            , _native_width( native_width )
            , _out_width( out_width )
            , _out_height( out_height )
            , _isp( isp )
        {
        }

    protected:
        void init_profiles_info( const rs2::frame * f ) override;
        // Allocate the output at the requested output resolution (not the source's padded width).
        rs2::frame prepare_frame( const rs2::frame_source & source, const rs2::frame & f ) override;
        rs2::frame process_frame( const rs2::frame_source & source, const rs2::frame & f ) override;
        void process_function( uint8_t * const dest[], const uint8_t * source,
                               int width, int height, int actual_size, int input_size ) override;

        int              _native_width;    // real sensor width in px (e.g. 1288), pre-scale
        int              _out_width;       // requested output width in px
        int              _out_height;      // requested output height in px
        int              _src_width = 0;   // source profile width in px (e.g. 1612, padded)
        int              _src_height = 0;  // source profile height in px (e.g. 808)
        int              _src_data_size = 0;// source frame's actual byte count (authoritative for stride)
        rggb::isp_params _isp;
        float            _awb_gain_r = 1.7f;  // gray-world auto-white-balance gains (EMA per frame)
        float            _awb_gain_b = 1.4f;
        std::vector< uint8_t > _bayer;      // scratch: RAW10 unpacked to 8-bit Bayer (native_width*height)
        std::vector< uint8_t > _rgb_native; // scratch: demosaiced native RGB8 before crop+scale
    };
}
