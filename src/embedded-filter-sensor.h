// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#pragma once

#include "core/extension.h"
#include <vector>

namespace librealsense {
    class embedded_filter_sensor
    {
    public:
        virtual ~embedded_filter_sensor() = default;

        void set(rs2_embedded_filter embedded_filter_type, std::vector<uint8_t> params);
        std::vector<uint8_t> get(rs2_embedded_filter embedded_filter_type);
        bool supports(rs2_embedded_filter embedded_filter_type) const;
    };
    MAP_EXTENSION(RS2_EXTENSION_EMBEDDED_FILTER_SENSOR, librealsense::embedded_filter_sensor);
}
