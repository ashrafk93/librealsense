// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
//
// D401 GMSL dual-RGB stereo rectification (CPU).
//
// Streams the two color streams (the two OV9782 imagers exposed by the dual-RGB POC firmware) and
// rectifies them into a scanline-aligned stereo pair using cv::initUndistortRectifyMap + cv::remap.
//
// Calibration: the *raw* per-imager intrinsics + distortion + rectification rotations come from the
// factory stereo calibration (rectify-opencv.cpp values below). NOTE: the SDK only exposes the
// post-rectification (distortion-free) IR/Depth intrinsics, which cannot rectify the raw RGGB
// imager output -- hence the offline-extracted raw calibration here. Replace the constants below
// with the values for this specific unit (these are per-camera). The maps are computed once.
//
// Output: rectified left|right side-by-side with horizontal guide lines (rectified stereo => a
// world point lands on the same row in both images). Saves a PNG; pass --display for a live window.
//
// Build (on the Jetson, against the built fork):
//   g++ -std=c++14 -I <repo>/include rs-dual-rgb-rectify.cpp \
//       $(pkg-config --cflags --libs opencv4) \
//       -L <repo>/build-zc/off/Release -lrealsense2 -lpthread -o /tmp/rs-dual-rgb-rectify
//
// CUDA: cv::remap is the only per-frame cost; a custom CUDA remap kernel can replace it later
// (this OpenCV has no cv::cuda module). The maps are precomputed and would just be uploaded once.

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

namespace {

const int SRC_W  = 1612;   // advertised color width (RAW10 transport padding included)
const int REAL_W = 1288;   // real OV9782 width after dropping the padding
const int REAL_H = 808;

// --- factory stereo calibration (raw intrinsics + distortion + rectification rotations) ---
// Per-unit values; from rectify-opencv.cpp. Calibrated at 1280x720, so we rectify at that size.
struct Calib
{
    cv::Mat K_L, D_L, R_L, K_R, D_R, R_R, P;
    cv::Size size;
} g;

void init_calib()
{
    g.K_L = ( cv::Mat_< double >( 3, 3 ) << 649.6231, 0, 636.1855, 0, 648.288, 402.3958, 0, 0, 1 );
    g.D_L = ( cv::Mat_< double >( 1, 5 ) << -0.052214, 0.060396, 0.00053593, -0.0014044, -0.018184 );
    g.R_L = ( cv::Mat_< double >( 3, 3 ) <<
              0.999949436295546, 0.002088371842696, -0.009836846815234,
              -0.002097318108441, 0.999997396311540, -8.992368395849524e-04,
              0.009834943262254, 9.198223677947176e-04, 0.999951212718821 );
    g.K_R = ( cv::Mat_< double >( 3, 3 ) << 646.5018, 0, 629.5378, 0, 645.279, 403.669, 0, 0, 1 );
    g.D_R = ( cv::Mat_< double >( 1, 5 ) << -0.054867, 0.064802, 0.00026262, 0.00038514, -0.020907 );
    g.R_R = ( cv::Mat_< double >( 3, 3 ) <<
              0.999993118113620, 0.002892431710317, 0.002323265848005,
              -0.002894543623322, 0.999995400216698, 9.061802573074177e-04,
              -0.002320634096974, -9.128988154234657e-04, 0.999996890631736 );
    g.P   = ( cv::Mat_< double >( 3, 3 ) << 650.6063, 0, 638.5314, 0, 650.6063, 363.5143, 0, 0, 1 );
    g.size = cv::Size( 1280, 720 );
}

}  // namespace

int main( int argc, char ** argv )
{
    bool display = false;
    for( int i = 1; i < argc; ++i )
        if( std::string( argv[i] ) == "--display" )
            display = true;

    init_calib();

    // --- rectification maps (computed once) ---
    cv::Mat mapLx, mapLy, mapRx, mapRy;
    cv::initUndistortRectifyMap( g.K_L, g.D_L, g.R_L, g.P, g.size, CV_16SC2, mapLx, mapLy );
    cv::initUndistortRectifyMap( g.K_R, g.D_R, g.R_R, g.P, g.size, CV_16SC2, mapRx, mapRy );

    try
    {
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream( RS2_STREAM_COLOR, 0, SRC_W, REAL_H, RS2_FORMAT_RGB8, 30 );
        cfg.enable_stream( RS2_STREAM_COLOR, 1, SRC_W, REAL_H, RS2_FORMAT_RGB8, 30 );
        pipe.start( cfg );

        cv::Mat rectL, rectR;
        int saved = 0;
        for( int frames = 0; frames < ( display ? 1000000 : 40 ); ++frames )
        {
            rs2::frameset fs = pipe.wait_for_frames();
            cv::Mat leftSrc, rightSrc;
            for( auto && f : fs )
            {
                auto v = f.as< rs2::video_frame >();
                if( ! v || v.get_profile().stream_type() != RS2_STREAM_COLOR )
                    continue;
                cv::Mat rgb( REAL_H, SRC_W, CV_8UC3, (void *)v.get_data() );        // RGB8, padded
                cv::Mat cropped, resized, bgr;
                cv::cvtColor( rgb( cv::Rect( 0, 0, REAL_W, REAL_H ) ), bgr, cv::COLOR_RGB2BGR );  // crop -> BGR
                cv::resize( bgr, resized, g.size );                                  // -> calib resolution
                ( v.get_profile().stream_index() == 0 ? leftSrc : rightSrc ) = resized;
            }
            if( leftSrc.empty() || rightSrc.empty() )
                continue;

            cv::remap( leftSrc,  rectL, mapLx, mapLy, cv::INTER_LINEAR );
            cv::remap( rightSrc, rectR, mapRx, mapRy, cv::INTER_LINEAR );

            cv::Mat sbs;
            cv::hconcat( rectL, rectR, sbs );
            for( int y = 0; y < sbs.rows; y += 40 )
                cv::line( sbs, { 0, y }, { sbs.cols, y }, { 0, 255, 0 }, 1 );

            if( display )
            {
                cv::imshow( "dual-RGB rectified (L | R)", sbs );
                if( cv::waitKey( 1 ) == 27 ) break;
            }
            else if( frames > 8 && saved == 0 )   // let auto-exposure settle, then save one
            {
                cv::Mat raw_sbs;  cv::hconcat( leftSrc, rightSrc, raw_sbs );
                for( int y = 0; y < raw_sbs.rows; y += 40 )
                    cv::line( raw_sbs, { 0, y }, { raw_sbs.cols, y }, { 0, 255, 0 }, 1 );
                cv::imwrite( "/tmp/rect_before.png", raw_sbs );
                cv::imwrite( "/tmp/rect_after.png", sbs );
                std::cout << "Saved /tmp/rect_before.png and /tmp/rect_after.png\n";
                saved = 1;
                break;
            }
        }
        pipe.stop();
    }
    catch( const rs2::error & e )
    {
        std::cerr << "RS2 ERROR (" << e.get_failed_function() << "): " << e.what() << "\n";
        return 2;
    }
    catch( const std::exception & e )
    {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 2;
    }
    return 0;
}
