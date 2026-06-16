// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
#pragma once

#include <librealsense2/h/rs_types.h>     // rs2_intrinsics
#include <librealsense2/h/rs_sensor.h>    // rs2_extrinsics
#include <cstdint>
#include <vector>

// Pure-C++ stereo rectification for the D401 GMSL dual-RGB pair. No OpenCV dependency (so it can
// live in an SDK processing block): the remap tables are built with RealSense's own projection
// model (rs2_project_point_to_pixel, which handles RS2_DISTORTION_INVERSE_BROWN_CONRADY used by
// the color imagers), and the per-frame work is a bilinear remap.
//
// Pipeline per eye: for each output (rectified, pinhole) pixel -> back-project to a ray -> apply
// the rectifying rotation -> project through the source intrinsics (with distortion) -> source
// pixel. The table is built once; remap_rgb8() runs per frame.

namespace librealsense {
namespace rect {

// Precomputed source-pixel lookup for one eye.
struct remap_table
{
    int w = 0, h = 0;                 // output (rectified) size
    std::vector< float > sx, sy;      // per output pixel, the source pixel to sample (bilinear)
};

// Rectifying rotations + common focal length for a stereo pair, computed from the two intrinsics
// and the left->right extrinsics (Bouguet). R_left/R_right are 3x3 row-major (orig -> rectified).
struct rectification
{
    float R_left[9];
    float R_right[9];
    float new_f;        // common focal (px) for the rectified images, scaled to out_w
    remap_table left;
    remap_table right;
};

// Compute the full rectification for an output size out_w x out_h. `inL`/`inR` are the (already
// resolution-correct) source intrinsics for left/right; `lr` is the left->right extrinsics.
rectification compute( const rs2_intrinsics & inL,
                       const rs2_intrinsics & inR,
                       const rs2_extrinsics & lr,
                       int out_w, int out_h );

// Build one eye's table directly (used internally / for undistort-only with R = identity).
remap_table build_table( const rs2_intrinsics & src, const float R[9], float new_f, int out_w, int out_h );

// Bilinear remap of an interleaved RGB8 source into dst (out=t.w x t.h, 3 bytes/px, tight).
void remap_rgb8( const uint8_t * src, int src_w, int src_h, int src_stride,
                 const remap_table & t, uint8_t * dst );

}  // namespace rect
}  // namespace librealsense
