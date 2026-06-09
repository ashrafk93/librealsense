// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
#pragma once

#include <cstdint>

// RGGB8 -> RGB8 debayer for the D401 GMSL "dual RGB" mode.
//
// On the D401 GMSL dual-RGB POC firmware, each OV9782 imager is delivered over the GMSL
// link as an 8-bit RGGB Bayer image using the FW CSI passthrough (RAW8) path. The V4L2 node
// advertises width = 1612 (the VDF byte-count per line) while the real sensor image is only
// 1288 px wide; the remaining columns are dword-alignment padding. The host therefore has to:
//   1) crop the padding away (1612 -> 1288),
//   2) subtract the OV9782 black level,
//   3) demosaic the RGGB Bayer mosaic to RGB,
//   4) (optionally) apply per-channel white-balance gains.
//
// This header is deliberately free of any librealsense/SDK dependency so the algorithm can be
// unit-tested on its own. The SDK processing-block wrapper (a color_converter) calls into it.

namespace librealsense {
namespace rggb {

// OV9782 / dual-RGB ISP knobs. Defaults are a sane starting point for the D401 OV9782 sensor
// (black level for the 8-bit RAW8 passthrough is the 10-bit floor 64 shifted down by 2 = 16).
struct isp_params
{
    int   black_level = 16;     // subtracted from every channel before gains, then clamped to 0
    float gain_r      = 1.0f;   // white-balance gains applied per channel after black-level
    float gain_g      = 1.0f;
    float gain_b      = 1.0f;
};

// Bilinear demosaic of an 8-bit RGGB Bayer image into interleaved RGB8.
//
//   src        : Bayer8 data, top-left origin, RGGB phase (row0: R G R G..., row1: G B G B...)
//   src_stride : bytes per Bayer row (>= src_width; e.g. 1612 or kernel-padded 1664)
//   out_width  : cropped output width  in pixels (e.g. 1288) — must be even, <= src usable width
//   out_height : cropped output height in pixels (e.g. 808)  — must be even
//   dst        : caller-owned buffer of out_width * out_height * 3 bytes (RGB8, R first)
//   p          : ISP parameters (black level + WB gains)
//
// Edge pixels use clamped neighbours. Both dimensions are assumed even (Bayer tiles are 2x2);
// odd inputs are handled by clamping but the last row/col phase may be approximate.
void debayer_rggb8( const uint8_t * src,
                    int src_stride,
                    int out_width,
                    int out_height,
                    uint8_t * dst,
                    const isp_params & p = {} );

}  // namespace rggb
}  // namespace librealsense
