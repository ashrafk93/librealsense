// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "stereo-rectify.h"
#include <librealsense2/rsutil.h>   // rs2_project_point_to_pixel (handles RS distortion models)

#include <cmath>
#include <cstddef>

namespace librealsense {
namespace rect {

namespace {

// --- minimal 3x3 (row-major) helpers ---
void mat_mul( const float a[9], const float b[9], float o[9] )
{
    for( int r = 0; r < 3; ++r )
        for( int c = 0; c < 3; ++c )
            o[r * 3 + c] = a[r * 3 + 0] * b[0 * 3 + c] + a[r * 3 + 1] * b[1 * 3 + c] + a[r * 3 + 2] * b[2 * 3 + c];
}

void rodrigues( const float v[3], float R[9] )   // rotation vector -> 3x3
{
    float th = std::sqrt( v[0] * v[0] + v[1] * v[1] + v[2] * v[2] );
    if( th < 1e-9f )
    {
        R[0] = R[4] = R[8] = 1.f;
        R[1] = R[2] = R[3] = R[5] = R[6] = R[7] = 0.f;
        return;
    }
    float x = v[0] / th, y = v[1] / th, z = v[2] / th, c = std::cos( th ), s = std::sin( th ), C = 1 - c;
    R[0] = c + x * x * C;     R[1] = x * y * C - z * s; R[2] = x * z * C + y * s;
    R[3] = y * x * C + z * s; R[4] = c + y * y * C;     R[5] = y * z * C - x * s;
    R[6] = z * x * C - y * s; R[7] = z * y * C + x * s; R[8] = c + z * z * C;
}

void inv_rodrigues( const float R[9], float v[3] )   // 3x3 -> rotation vector
{
    float tr = R[0] + R[4] + R[8];
    float th = std::acos( std::max( -1.f, std::min( 1.f, ( tr - 1 ) * 0.5f ) ) );
    if( th < 1e-9f ) { v[0] = v[1] = v[2] = 0.f; return; }
    float s = 2.f * std::sin( th );
    v[0] = ( R[7] - R[5] ) / s * th;
    v[1] = ( R[2] - R[6] ) / s * th;
    v[2] = ( R[3] - R[1] ) / s * th;
}

}  // namespace

remap_table build_table( const rs2_intrinsics & src, const float R[9], float new_f, int out_w, int out_h )
{
    remap_table t;
    t.w = out_w; t.h = out_h;
    t.sx.resize( (size_t)out_w * out_h );
    t.sy.resize( (size_t)out_w * out_h );
    const float cx = out_w * 0.5f, cy = out_h * 0.5f;
    for( int v = 0; v < out_h; ++v )
    {
        for( int u = 0; u < out_w; ++u )
        {
            // rectified pinhole ray
            float x = ( u - cx ) / new_f, y = ( v - cy ) / new_f, z = 1.f;
            // orig = R^T * rect   (R: orig -> rectified, row-major)
            float pt[3] = { R[0] * x + R[3] * y + R[6] * z,
                            R[1] * x + R[4] * y + R[7] * z,
                            R[2] * x + R[5] * y + R[8] * z };
            float px[2];
            rs2_project_point_to_pixel( px, &src, pt );   // applies src distortion (incl. inverse-BC)
            t.sx[(size_t)v * out_w + u] = px[0];
            t.sy[(size_t)v * out_w + u] = px[1];
        }
    }
    return t;
}

rectification compute( const rs2_intrinsics & inL, const rs2_intrinsics & inR,
                       const rs2_extrinsics & lr, int out_w, int out_h )
{
    rectification rc;

    // Bouguet: rotate each camera halfway to a common (coplanar) orientation, then rotate so the
    // baseline lies along x (rows become epipolar lines). lr.rotation is column-major (left->right).
    float R_lr[9];
    for( int r = 0; r < 3; ++r )
        for( int c = 0; c < 3; ++c )
            R_lr[r * 3 + c] = lr.rotation[c * 3 + r];

    float om[3]; inv_rodrigues( R_lr, om );
    float omh[3] = { om[0] * 0.5f, om[1] * 0.5f, om[2] * 0.5f };
    float r_r[9]; rodrigues( omh, r_r );               // right rotated +half
    float omn[3] = { -omh[0], -omh[1], -omh[2] };
    float r_l[9]; rodrigues( omn, r_l );               // left rotated -half

    // baseline in the half-rotated left frame: t = r_l * (lr.translation)
    float T[3] = { lr.translation[0], lr.translation[1], lr.translation[2] };
    float t[3] = { r_l[0] * T[0] + r_l[1] * T[1] + r_l[2] * T[2],
                   r_l[3] * T[0] + r_l[4] * T[1] + r_l[5] * T[2],
                   r_l[6] * T[0] + r_l[7] * T[1] + r_l[8] * T[2] };
    float tn = std::sqrt( t[0] * t[0] + t[1] * t[1] + t[2] * t[2] );
    if( tn < 1e-9f ) tn = 1.f;
    // rectified basis: e1 along baseline, e2 = z x e1, e3 = e1 x e2
    float e1[3] = { t[0] / tn, t[1] / tn, t[2] / tn };
    // Orient the rectified x-axis to +x. The D400 baseline is along -x (T.x < 0), which would make
    // Rrect a 180-degree rotation (flipped image). Keeping x ~ +x yields Rrect ~ identity for these
    // near-parallel imagers (a mirror of the disparity sign, irrelevant for display/rectification).
    if( e1[0] < 0.f ) { e1[0] = -e1[0]; e1[1] = -e1[1]; e1[2] = -e1[2]; }
    float e2[3] = { -e1[1], e1[0], 0.f };
    float e2n = std::sqrt( e2[0] * e2[0] + e2[1] * e2[1] ); if( e2n < 1e-9f ) e2n = 1.f;
    e2[0] /= e2n; e2[1] /= e2n; e2[2] = 0.f;
    float e3[3] = { e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0] };
    float Rrect[9] = { e1[0], e1[1], e1[2], e2[0], e2[1], e2[2], e3[0], e3[1], e3[2] };

    mat_mul( Rrect, r_l, rc.R_left );
    mat_mul( Rrect, r_r, rc.R_right );

    rc.new_f = inL.fx;   // inL is expected already at the output resolution
    rc.left  = build_table( inL, rc.R_left,  rc.new_f, out_w, out_h );
    rc.right = build_table( inR, rc.R_right, rc.new_f, out_w, out_h );
    return rc;
}

void remap_rgb8( const uint8_t * src, int src_w, int src_h, int src_stride,
                 const remap_table & t, uint8_t * dst )
{
    for( int v = 0; v < t.h; ++v )
    {
        uint8_t * drow = dst + (size_t)v * t.w * 3;
        for( int u = 0; u < t.w; ++u )
        {
            float fx = t.sx[(size_t)v * t.w + u], fy = t.sy[(size_t)v * t.w + u];
            int x0 = (int)std::floor( fx ), y0 = (int)std::floor( fy );
            uint8_t * o = drow + u * 3;
            if( x0 < 0 || y0 < 0 || x0 + 1 >= src_w || y0 + 1 >= src_h )
            {
                o[0] = o[1] = o[2] = 0;
                continue;
            }
            float ax = fx - x0, ay = fy - y0;
            const uint8_t * p00 = src + (size_t)y0 * src_stride + x0 * 3;
            const uint8_t * p01 = p00 + 3;
            const uint8_t * p10 = p00 + src_stride;
            const uint8_t * p11 = p10 + 3;
            for( int ch = 0; ch < 3; ++ch )
            {
                float top = p00[ch] * ( 1 - ax ) + p01[ch] * ax;
                float bot = p10[ch] * ( 1 - ax ) + p11[ch] * ax;
                o[ch] = (uint8_t)( top * ( 1 - ay ) + bot * ay + 0.5f );
            }
        }
    }
}

}  // namespace rect
}  // namespace librealsense
