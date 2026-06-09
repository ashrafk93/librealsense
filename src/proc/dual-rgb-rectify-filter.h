// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
#pragma once

#include "synthetic-stream.h"      // stream_filter_processing_block
#include "stereo-rectify.h"        // rect::rectification
#include <librealsense2/hpp/rs_frame.hpp>
#include <vector>

namespace librealsense {

// D401 GMSL dual-RGB rectification, as a recommended post-processing filter (shows up in the
// viewer's Post-Processing with an on/off toggle, on by default). It self-configures from the two
// color frames' own profiles (intrinsics + the left->right extrinsics, both provided by the SDK):
// once it has seen both color streams it computes the stereo-rectify maps once, then remaps each
// color frame to a rectified image. Pure C++ (no OpenCV) via the stereo-rectify module.
class LRS_EXTENSION_API dual_rgb_rectify_filter : public stream_filter_processing_block
{
public:
    dual_rgb_rectify_filter();

protected:
    rs2::frame process_frame( const rs2::frame_source & source, const rs2::frame & f ) override;

private:
    void ensure_maps();

    rs2::video_stream_profile _p0, _p1;   // captured per-eye color profiles (for calibration)
    bool                      _ready = false;
    rect::rectification       _rc;
    std::vector< uint8_t >    _tmp;        // scratch for the rectified (real-width) image
};

}  // namespace librealsense
