// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
//
// Standalone unit test for the D401 GMSL dual-RGB RGGB8->RGB8 debayer core.
// No SDK/camera dependency. Build & run:
//   g++ -std=c++14 -I ../../src/proc test-rggb-debayer.cpp ../../src/proc/rggb-debayer.cpp -o /tmp/test_rggb && /tmp/test_rggb

#include "rggb-debayer.h"

#include <cstdio>
#include <cstdint>
#include <vector>
#include <cmath>

using namespace librealsense::rggb;

static int g_failures = 0;
#define CHECK( cond, msg )                                                     \
    do {                                                                       \
        if( !( cond ) ) { std::printf( "  FAIL: %s\n", msg ); ++g_failures; }  \
        else            { std::printf( "  ok  : %s\n", msg ); }                \
    } while( 0 )

// Build a RAW8 RGGB source like the D401 passthrough: real image out_w x out_h, but a wider
// V4L2 stride with garbage padding columns (to prove the crop ignores them).
static std::vector<uint8_t> make_rggb( int stride, int real_w, int real_h,
                                       uint8_t r, uint8_t gr, uint8_t gb, uint8_t b,
                                       uint8_t pad )
{
    std::vector<uint8_t> img( static_cast<size_t>(stride) * real_h, pad );
    for( int y = 0; y < real_h; ++y )
        for( int x = 0; x < real_w; ++x )
        {
            uint8_t v;
            if( !(y&1) && !(x&1) )      v = r;    // R
            else if( !(y&1) &&  (x&1) ) v = gr;   // Gr
            else if(  (y&1) && !(x&1) ) v = gb;   // Gb
            else                        v = b;    // B
            img[ static_cast<size_t>(y)*stride + x ] = v;
        }
    return img;
}

int main()
{
    // --- Test 1: flat color field, crop past padding, no ISP -------------------------------
    // R sites=200, G sites=100, B sites=50. Interior pixels must all demosaic to (200,100,50).
    {
        std::printf( "[1] flat field + crop (stride 1612 -> width 1288)\n" );
        const int stride = 1612, W = 1288, H = 808;
        auto src = make_rggb( stride, W, H, 200, 100, 100, 50, /*pad*/255 );
        std::vector<uint8_t> dst( static_cast<size_t>(W)*H*3, 0 );
        debayer_rggb8( src.data(), stride, W, H, dst.data(), isp_params{ /*black*/0, 1, 1, 1 } );

        bool interior_ok = true;
        int  bad_x = -1, bad_y = -1;
        for( int y = 1; y < H-1 && interior_ok; ++y )
            for( int x = 1; x < W-1; ++x )
            {
                const uint8_t* px = &dst[ (static_cast<size_t>(y)*W + x)*3 ];
                if( px[0] != 200 || px[1] != 100 || px[2] != 50 )
                { interior_ok = false; bad_x = x; bad_y = y; break; }
            }
        if( !interior_ok )
        {
            const uint8_t* px = &dst[ (static_cast<size_t>(bad_y)*W + bad_x)*3 ];
            std::printf( "       first bad pixel (%d,%d) = (%d,%d,%d)\n", bad_x, bad_y, px[0], px[1], px[2] );
        }
        CHECK( interior_ok, "all interior pixels = (200,100,50)" );

        // Padding columns must never leak: a pixel at the right edge should not be 255-ish.
        const uint8_t* edge = &dst[ (static_cast<size_t>(H/2)*W + (W-1))*3 ];
        CHECK( edge[0] <= 200 && edge[1] <= 100 && edge[2] <= 50,
               "right-edge pixel unaffected by 255 padding (crop honored)" );
    }

    // --- Test 2: black-level subtraction ----------------------------------------------------
    // All samples = 80, black_level = 16 -> every channel = 64.
    {
        std::printf( "[2] black-level subtract (80 - 16 = 64)\n" );
        const int W = 64, H = 64, stride = W;
        auto src = make_rggb( stride, W, H, 80, 80, 80, 80, 0 );
        std::vector<uint8_t> dst( static_cast<size_t>(W)*H*3, 0 );
        debayer_rggb8( src.data(), stride, W, H, dst.data(), isp_params{ 16, 1, 1, 1 } );
        const uint8_t* c = &dst[ (static_cast<size_t>(H/2)*W + W/2)*3 ];
        CHECK( c[0]==64 && c[1]==64 && c[2]==64, "center pixel = (64,64,64)" );
    }

    // --- Test 3: white-balance gains + clamp -------------------------------------------------
    // gray 100, gain_r=2 (->200), gain_b=3 (->300 clamps to 255).
    {
        std::printf( "[3] WB gains + saturation clamp\n" );
        const int W = 64, H = 64, stride = W;
        auto src = make_rggb( stride, W, H, 100, 100, 100, 100, 0 );
        std::vector<uint8_t> dst( static_cast<size_t>(W)*H*3, 0 );
        debayer_rggb8( src.data(), stride, W, H, dst.data(), isp_params{ 0, 2.0f, 1.0f, 3.0f } );
        const uint8_t* c = &dst[ (static_cast<size_t>(H/2)*W + W/2)*3 ];
        CHECK( c[0]==200 && c[1]==100 && c[2]==255, "center pixel = (200,100,255) clamped" );
    }

    std::printf( g_failures ? "\nRESULT: %d FAILURE(S)\n" : "\nRESULT: all passed\n", g_failures );
    return g_failures ? 1 : 0;
}
