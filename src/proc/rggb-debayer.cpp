// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "rggb-debayer.h"

#include <cstddef>   // size_t
#include <cmath>     // std::pow

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

inline float clamp01( float v ) { return v < 0.f ? 0.f : ( v > 1.f ? 1.f : v ); }

}  // namespace

void unpack_raw10( const uint8_t * src, int src_stride, int real_width, int height, uint8_t * bayer8 )
{
    const int groups = real_width / 4;   // 5 source bytes per 4 pixels
    for( int y = 0; y < height; ++y )
    {
        const uint8_t * s = src + static_cast< size_t >( y ) * src_stride;
        uint8_t * d = bayer8 + static_cast< size_t >( y ) * real_width;
        for( int g = 0; g < groups; ++g )
        {
            const uint8_t * q = s + g * 5;          // [m0 m1 m2 m3 lsb]; 8-bit value == MSB byte
            d[ g * 4 + 0 ] = q[ 0 ];
            d[ g * 4 + 1 ] = q[ 1 ];
            d[ g * 4 + 2 ] = q[ 2 ];
            d[ g * 4 + 3 ] = q[ 3 ];
        }
    }
}

void debayer_rggb8( const uint8_t * bayer, int bayer_stride, int width, int height,
                    uint8_t * dst, const isp_params & p, int dst_stride_px )
{
    const int wmax = width - 1;
    const int hmax = height - 1;
    const int bl   = p.black_level;
    const int row_px = ( dst_stride_px > width ) ? dst_stride_px : width;

    // Tone curve: the sensor data is linear; encode with 1/gamma (sRGB-like) so midtones aren't
    // crushed on a display. 1024-entry LUT indexed by the normalized [0,1] post-CCM value.
    uint8_t tone[1024];
    const float inv_g = ( p.gamma > 0.f ) ? 1.f / p.gamma : 1.f;
    const float sc = p.s_curve;
    for( int i = 0; i < 1024; ++i )
    {
        float x = std::pow( i / 1023.f, inv_g );
        x = x + sc * x * ( 1.f - x ) * ( 2.f * x - 1.f );    // S-curve contrast (the "pop")
        tone[i] = to_u8( 255.f * clamp01( x ) );
    }

    const float gr = p.gain_r * p.digital_gain;
    const float gg = p.gain_g * p.digital_gain;
    const float gb = p.gain_b * p.digital_gain;
    const float sat = p.saturation, con = p.contrast;
    const float * m = p.ccm;
    // Normalize the black-subtracted value to [0,1] using the post-black range (255 - black), so a
    // full-scale sensor reading maps to 1.0 (matches the reference raw decode).
    const float inv_range = 1.f / ( 255.f - (float)p.black_level );

    // Black-level-subtracted, edge-clamped Bayer sample at (x,y).
    auto S = [&]( int x, int y ) -> int {
        x = clampi( x, 0, wmax );
        y = clampi( y, 0, hmax );
        int v = static_cast< int >( bayer[ y * bayer_stride + x ] ) - bl;
        return v < 0 ? 0 : v;
    };

    for( int y = 0; y < height; ++y )
    {
        uint8_t * row = dst + static_cast< size_t >( y ) * row_px * 3;
        const int yodd = y & 1;
        for( int x = 0; x < width; ++x )
        {
            const int xodd = x & 1;
            float R, G, B;

            if( !yodd && !xodd )            // R site
            {
                R = (float)S( x, y );
                G = ( S( x - 1, y ) + S( x + 1, y ) + S( x, y - 1 ) + S( x, y + 1 ) ) * 0.25f;
                B = ( S( x - 1, y - 1 ) + S( x + 1, y - 1 ) + S( x - 1, y + 1 ) + S( x + 1, y + 1 ) ) * 0.25f;
            }
            else if( !yodd && xodd )        // Gr site (red row): H=R, V=B
            {
                G = (float)S( x, y );
                R = ( S( x - 1, y ) + S( x + 1, y ) ) * 0.5f;
                B = ( S( x, y - 1 ) + S( x, y + 1 ) ) * 0.5f;
            }
            else if( yodd && !xodd )        // Gb site (blue row): H=B, V=R
            {
                G = (float)S( x, y );
                R = ( S( x, y - 1 ) + S( x, y + 1 ) ) * 0.5f;
                B = ( S( x - 1, y ) + S( x + 1, y ) ) * 0.5f;
            }
            else                            // B site
            {
                B = (float)S( x, y );
                G = ( S( x - 1, y ) + S( x + 1, y ) + S( x, y - 1 ) + S( x, y + 1 ) ) * 0.25f;
                R = ( S( x - 1, y - 1 ) + S( x + 1, y - 1 ) + S( x - 1, y + 1 ) + S( x + 1, y + 1 ) ) * 0.25f;
            }

            if( p.swap_rb ) { float t = R; R = B; B = t; }   // RGGB demosaic -> BGGR (real D401 phase)

            // White-balance + digital gain, normalized to [0,1].
            float r = clamp01( R * gr * inv_range );
            float g = clamp01( G * gg * inv_range );
            float b = clamp01( B * gb * inv_range );
            // Color-correction matrix (sensor RGB -> display primaries).
            float r2 = m[0] * r + m[1] * g + m[2] * b;
            float g2 = m[3] * r + m[4] * g + m[5] * b;
            float b2 = m[6] * r + m[7] * g + m[8] * b;
            // Saturation about luma (linear, Rec.709), then gamma (LUT) + contrast about mid-grey.
            const float yl = 0.2126f * r2 + 0.7152f * g2 + 0.0722f * b2;
            r2 = clamp01( yl + sat * ( r2 - yl ) );
            g2 = clamp01( yl + sat * ( g2 - yl ) );
            b2 = clamp01( yl + sat * ( b2 - yl ) );
            float rd = tone[ static_cast< int >( r2 * 1023.f ) ];
            float gd = tone[ static_cast< int >( g2 * 1023.f ) ];
            float bd = tone[ static_cast< int >( b2 * 1023.f ) ];
            row[ x * 3 + 0 ] = to_u8( ( rd - 128.f ) * con + 128.f );
            row[ x * 3 + 1 ] = to_u8( ( gd - 128.f ) * con + 128.f );
            row[ x * 3 + 2 ] = to_u8( ( bd - 128.f ) * con + 128.f );
        }
        // Zero any padding columns so a narrower image sits cleanly in a wider output frame.
        for( int x = width; x < row_px; ++x )
        {
            row[ x * 3 + 0 ] = 0;
            row[ x * 3 + 1 ] = 0;
            row[ x * 3 + 2 ] = 0;
        }
    }
}

}  // namespace rggb
}  // namespace librealsense
