// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "rggb-debayer.h"

#include <algorithm>

namespace librealsense {
namespace rggb {

namespace {

inline int clampi( int v, int lo, int hi )
{
    return v < lo ? lo : ( v > hi ? hi : v );
}

inline uint8_t to_u8( float v )
{
    int i = static_cast< int >( v + 0.5f );
    return static_cast< uint8_t >( i < 0 ? 0 : ( i > 255 ? 255 : i ) );
}

}  // namespace

void debayer_rggb8( const uint8_t * src,
                    int src_stride,
                    int out_width,
                    int out_height,
                    uint8_t * dst,
                    const isp_params & p )
{
    const int wmax = out_width - 1;
    const int hmax = out_height - 1;
    const int bl   = p.black_level;

    // Black-level-subtracted, edge-clamped Bayer sample at (x,y).
    auto S = [&]( int x, int y ) -> int {
        x = clampi( x, 0, wmax );
        y = clampi( y, 0, hmax );
        int v = static_cast< int >( src[ y * src_stride + x ] ) - bl;
        return v < 0 ? 0 : v;
    };

    for( int y = 0; y < out_height; ++y )
    {
        uint8_t * row = dst + static_cast< size_t >( y ) * out_width * 3;
        const int yodd = y & 1;
        for( int x = 0; x < out_width; ++x )
        {
            const int xodd = x & 1;
            float R, G, B;

            if( !yodd && !xodd )            // R site (red row, red col)
            {
                R = (float)S( x, y );
                G = ( S( x - 1, y ) + S( x + 1, y ) + S( x, y - 1 ) + S( x, y + 1 ) ) * 0.25f;
                B = ( S( x - 1, y - 1 ) + S( x + 1, y - 1 ) + S( x - 1, y + 1 ) + S( x + 1, y + 1 ) ) * 0.25f;
            }
            else if( !yodd && xodd )        // Gr site (red row, green col): H=R, V=B
            {
                G = (float)S( x, y );
                R = ( S( x - 1, y ) + S( x + 1, y ) ) * 0.5f;
                B = ( S( x, y - 1 ) + S( x, y + 1 ) ) * 0.5f;
            }
            else if( yodd && !xodd )        // Gb site (blue row, green col): H=B, V=R
            {
                G = (float)S( x, y );
                R = ( S( x, y - 1 ) + S( x, y + 1 ) ) * 0.5f;
                B = ( S( x - 1, y ) + S( x + 1, y ) ) * 0.5f;
            }
            else                            // B site (blue row, blue col)
            {
                B = (float)S( x, y );
                G = ( S( x - 1, y ) + S( x + 1, y ) + S( x, y - 1 ) + S( x, y + 1 ) ) * 0.25f;
                R = ( S( x - 1, y - 1 ) + S( x + 1, y - 1 ) + S( x - 1, y + 1 ) + S( x + 1, y + 1 ) ) * 0.25f;
            }

            row[ x * 3 + 0 ] = to_u8( R * p.gain_r );
            row[ x * 3 + 1 ] = to_u8( G * p.gain_g );
            row[ x * 3 + 2 ] = to_u8( B * p.gain_b );
        }
    }
}

}  // namespace rggb
}  // namespace librealsense
