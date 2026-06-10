// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
//
// D401 GMSL dual-RGB stereo rectification using the LIVE SDK calibration + the pure-C++ rectify
// module (src/proc/stereo-rectify). This exercises exactly the code that goes into the SDK
// processing block (no OpenCV in the rectification itself; OpenCV is only used for crop/display).
//
// Build (on the Jetson, against the built fork):
//   g++ -std=c++14 -I <repo>/include -I <repo>/src/proc rs-dual-rgb-rectify.cpp \
//       <repo>/src/proc/stereo-rectify.cpp $(pkg-config --cflags --libs opencv4) \
//       -L <repo>/build-zc/off/Release -lrealsense2 -lpthread -o /tmp/rs-dual-rgb-rectify
//
//   --display  live window (else saves /tmp/rect_before.png + /tmp/rect_after.png)

#include <librealsense2/rs.hpp>
#include "stereo-rectify.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

namespace {
const int SRC_W = 1612, REAL_W = 1288, REAL_H = 808;

// SDK reports color intrinsics at the padded 1612 width; rescale to the real 1288 (fx/fy/ppx by
// the width ratio, ppy unchanged) -> consistent square, centered intrinsics for the real image.
rs2_intrinsics to_real( rs2_intrinsics in )
{
    const float s = float( REAL_W ) / float( SRC_W );
    in.width = REAL_W; in.fx *= s; in.fy *= s; in.ppx *= s;   // ppy, height unchanged
    return in;
}
}  // namespace

int main( int argc, char ** argv )
{
    bool display = false;
    for( int i = 1; i < argc; ++i ) if( std::string( argv[i] ) == "--display" ) display = true;

    try
    {
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream( RS2_STREAM_COLOR, 0, SRC_W, REAL_H, RS2_FORMAT_RGB8, 30 );
        cfg.enable_stream( RS2_STREAM_COLOR, 1, SRC_W, REAL_H, RS2_FORMAT_RGB8, 30 );
        auto prof = pipe.start( cfg );

        // live calibration from the SDK
        rs2::video_stream_profile p0, p1;
        for( auto && sp : prof.get_streams() )
        {
            auto v = sp.as< rs2::video_stream_profile >();
            if( ! v || v.stream_type() != RS2_STREAM_COLOR ) continue;
            ( v.stream_index() == 0 ? p0 : p1 ) = v;
        }
        rs2_intrinsics inL = to_real( p0.get_intrinsics() );
        rs2_intrinsics inR = to_real( p1.get_intrinsics() );
        rs2_extrinsics lr  = p0.get_extrinsics_to( p1 );
        std::cout << "L fx=" << inL.fx << " fy=" << inL.fy << " ppx=" << inL.ppx << " ppy=" << inL.ppy
                  << "  baseline=" << cv::norm( cv::Vec3f( lr.translation[0], lr.translation[1], lr.translation[2] ) ) * 1000 << " mm\n";

        auto rc = librealsense::rect::compute( inL, inR, lr, REAL_W, REAL_H );
        std::cout << "rect new_f=" << rc.new_f << "\n";

        std::vector< uint8_t > rl( (size_t)REAL_W * REAL_H * 3 ), rr( (size_t)REAL_W * REAL_H * 3 );
        int saved = 0;
        for( int frames = 0; frames < ( display ? 1000000 : 40 ); ++frames )
        {
            rs2::frameset fs = pipe.wait_for_frames();
            const uint8_t *srcL = nullptr, *srcR = nullptr;
            for( auto && f : fs )
            {
                auto v = f.as< rs2::video_frame >();
                if( ! v || v.get_profile().stream_type() != RS2_STREAM_COLOR ) continue;
                ( v.get_profile().stream_index() == 0 ? srcL : srcR ) = (const uint8_t *)v.get_data();
            }
            if( ! srcL || ! srcR ) continue;

            // remap directly on the RGB8 frames (src valid width = REAL_W within the SRC_W stride)
            librealsense::rect::remap_rgb8( srcL, REAL_W, REAL_H, SRC_W * 3, rc.left,  rl.data() );
            librealsense::rect::remap_rgb8( srcR, REAL_W, REAL_H, SRC_W * 3, rc.right, rr.data() );

            cv::Mat L( REAL_H, REAL_W, CV_8UC3, rl.data() ), R( REAL_H, REAL_W, CV_8UC3, rr.data() );
            cv::Mat Lb, Rb; cv::cvtColor( L, Lb, cv::COLOR_RGB2BGR ); cv::cvtColor( R, Rb, cv::COLOR_RGB2BGR );
            cv::Mat sbs; cv::hconcat( Lb, Rb, sbs );
            for( int y = 0; y < sbs.rows; y += 40 ) cv::line( sbs, { 0, y }, { sbs.cols, y }, { 0, 255, 0 }, 1 );

            if( display )
            {
                cv::imshow( "dual-RGB rectified (L | R)", sbs );
                if( cv::waitKey( 1 ) == 27 ) break;
            }
            else if( frames > 8 && ! saved )
            {
                // before: raw cropped colors, side by side, same guide lines
                cv::Mat rawL( REAL_H, SRC_W, CV_8UC3, (void*)srcL ), rawR( REAL_H, SRC_W, CV_8UC3, (void*)srcR );
                cv::Mat bL, bR; cv::cvtColor( rawL( cv::Rect(0,0,REAL_W,REAL_H) ), bL, cv::COLOR_RGB2BGR );
                cv::cvtColor( rawR( cv::Rect(0,0,REAL_W,REAL_H) ), bR, cv::COLOR_RGB2BGR );
                cv::Mat before; cv::hconcat( bL, bR, before );
                for( int y = 0; y < before.rows; y += 40 ) cv::line( before, {0,y}, {before.cols,y}, {0,255,0}, 1 );
                cv::imwrite( "/tmp/rect_before.png", before );
                cv::imwrite( "/tmp/rect_after.png", sbs );
                std::cout << "Saved /tmp/rect_before.png + /tmp/rect_after.png\n";
                saved = 1; break;
            }
        }
        pipe.stop();
    }
    catch( const rs2::error & e ) { std::cerr << "RS2 ERROR (" << e.get_failed_function() << "): " << e.what() << "\n"; return 2; }
    catch( const std::exception & e ) { std::cerr << "ERROR: " << e.what() << "\n"; return 2; }
    return 0;
}
