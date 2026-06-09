// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "dual-rgb-rectify-filter.h"
#include <librealsense2/hpp/rs_processing.hpp>
#include <cstring>

namespace librealsense {

namespace {
const int REAL_W = 1288;   // real OV9782 width (color frames are advertised padded to 1612)
const int REAL_H = 808;
}  // namespace

dual_rgb_rectify_filter::dual_rgb_rectify_filter()
    : stream_filter_processing_block( "RGB Rectification" )
{
    // Only act on color streams; everything else passes through.
    _stream_filter.stream = RS2_STREAM_COLOR;
    _stream_filter.format = RS2_FORMAT_RGB8;
}

void dual_rgb_rectify_filter::ensure_maps()
{
    if( ! _p0 || ! _p1 )
        return;

    rs2_intrinsics inL = _p0.get_intrinsics();
    rs2_intrinsics inR = _p1.get_intrinsics();

    // Color intrinsics are reported at the padded width; rescale to the real image width
    // (fx/fy/ppx by the width ratio, ppy unchanged) -> consistent square, centered intrinsics.
    const float s = float( REAL_W ) / float( inL.width );
    inL.width = REAL_W; inL.fx *= s; inL.fy *= s; inL.ppx *= s;
    inR.width = REAL_W; inR.fx *= s; inR.fy *= s; inR.ppx *= s;

    rs2_extrinsics lr = _p0.get_extrinsics_to( _p1 );   // left -> right (carries the baseline)

    _rc = rect::compute( inL, inR, lr, REAL_W, REAL_H );
    _ready = true;
}

rs2::frame dual_rgb_rectify_filter::process_frame( const rs2::frame_source & source, const rs2::frame & f )
{
    auto vf = f.as< rs2::video_frame >();
    if( ! vf || vf.get_profile().format() != RS2_FORMAT_RGB8 )
        return f;

    const int idx = vf.get_profile().stream_index();
    if( auto vsp = vf.get_profile().as< rs2::video_stream_profile >() )
    {
        if( idx == 0 && ! _p0 ) _p0 = vsp;
        else if( idx == 1 && ! _p1 ) _p1 = vsp;
    }

    if( ! _ready )
    {
        ensure_maps();
        if( ! _ready )
            return f;   // pass through until both eyes' calibration is available
    }

    const rect::remap_table & t = ( idx == 1 ) ? _rc.right : _rc.left;
    const int w = vf.get_width(), h = vf.get_height();
    if( t.w > w || t.h > h )
        return f;   // unexpected geometry; don't touch

    // Rectify the real-width content into scratch, then place it (left-aligned) into a same-size
    // output frame (the padding columns stay zero), so the stream's advertised geometry is unchanged.
    _tmp.resize( (size_t)t.w * t.h * 3 );
    rect::remap_rgb8( static_cast< const uint8_t * >( vf.get_data() ), t.w, h, w * 3, t, _tmp.data() );

    rs2::frame tgt = source.allocate_video_frame( f.get_profile(), f );
    auto tvf = tgt.as< rs2::video_frame >();
    if( ! tvf )
        return f;
    uint8_t * dst = static_cast< uint8_t * >( const_cast< void * >( tvf.get_data() ) );
    const int dstride = tvf.get_stride_in_bytes();
    for( int y = 0; y < h; ++y )
    {
        std::memcpy( dst + (size_t)y * dstride, _tmp.data() + (size_t)y * t.w * 3, (size_t)t.w * 3 );
        if( w > t.w )
            std::memset( dst + (size_t)y * dstride + t.w * 3, 0, (size_t)( w - t.w ) * 3 );
    }
    return tgt;
}

}  // namespace librealsense
