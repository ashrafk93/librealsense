// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
#pragma once

#include <cstdint>

// RAW10 RGGB -> RGB8 for the D401 GMSL "dual RGB" mode.
//
// Each OV9782 imager is delivered over GMSL via the FW CSI passthrough. Although the V4L2 node
// advertises 'RGGB' 8-bit, the payload is actually MIPI **RAW10**: 4 pixels packed into 5 bytes
// (4 MSB bytes + 1 byte holding the four 2-bit LSBs). The transport width is 1612 (1610 active
// RAW10 bytes + alignment) while the real image is 1288 px; the row is padded to a 64-byte
// V4L2 stride (1664) on the kernel side, though librealsense hands us a 1612-stride frame.
//
// The host therefore: (1) unpacks RAW10 -> 8-bit Bayer (the 8-bit value is just the MSB byte,
// since (msb<<2 | lsb) >> 2 == msb), (2) demosaics RGGB -> RGB, (3) applies white-balance gains
// (OV9782 is green-dominant). This header has no SDK dependency so it can be unit-tested alone.

namespace librealsense {
namespace rggb {

// ISP knobs. Defaults gray-balance the green-dominant OV9782 (measured R/G/B ~ 41/69/48).
struct isp_params
{
    int   black_level   = 16;     // subtracted per channel before gains, clamped to 0
    float gain_r        = 1.9f;   // white-balance gains (boost R/B relative to green)
    float gain_g        = 1.0f;
    float gain_b        = 1.6f;
    float digital_gain  = 1.8f;   // overall brightness multiplier applied before the tone curve
    float gamma         = 2.2f;   // display gamma; the sensor data is linear, so we encode with
                                  // 1/gamma (sRGB-like) - WITHOUT this the image looks very dark
    float saturation    = 1.40f;  // chroma boost about luma, applied in LINEAR space after the CCM
    float contrast      = 1.18f;  // contrast around mid-grey (1 = neutral); lifts the washed-out look
    // Color-correction matrix (sensor RGB -> display primaries): boosts R/B purity, suppresses
    // green crosstalk. Row-major 3x3, applied to white-balanced, normalized [0,1] linear RGB.
    float ccm[9] = {  1.5f, -0.4f, -0.1f,
                     -0.1f,  1.2f, -0.1f,
                     -0.1f, -0.4f,  1.5f };
};

// Unpack MIPI RAW10 (4 px / 5 bytes) to 8-bit Bayer.
//   src        : RAW10-packed bytes, top-left origin
//   src_stride : bytes per source row (e.g. 1612 from the SDK frame, or 1664 raw V4L2)
//   real_width : real pixel columns to produce, multiple of 4 (e.g. 1288)
//   height     : rows
//   bayer8     : caller buffer of real_width*height bytes (8-bit RGGB Bayer)
void unpack_raw10( const uint8_t * src, int src_stride, int real_width, int height, uint8_t * bayer8 );

// Bilinear RGGB demosaic of 8-bit Bayer -> interleaved RGB8, with black-level + WB gains.
//   bayer        : 8-bit RGGB Bayer (row0: R G R G..., row1: G B G B...)
//   bayer_stride : bytes per Bayer row
//   width,height : pixels to demosaic (even dims assumed)
//   dst          : RGB8 output (R first)
//   p            : ISP params
//   dst_stride_px: output row width in px (0 => contiguous = width). If > width, the extra
//                  columns [width, dst_stride_px) are zeroed - lets a 1288-px image sit inside a
//                  1612-px output frame without a profile-dimension change.
void debayer_rggb8( const uint8_t * bayer, int bayer_stride, int width, int height,
                    uint8_t * dst, const isp_params & p = {}, int dst_stride_px = 0 );

}  // namespace rggb
}  // namespace librealsense
