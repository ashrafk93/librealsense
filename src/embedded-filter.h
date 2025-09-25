// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#pragma once

#include "core/extension.h"
#include <vector>
#include <map>
#include <memory>
#include <cstdint>

namespace librealsense {
    // Interface class for embedded filter sensors
    class embedded_filter_sensor_interface
    {
    public:
        virtual ~embedded_filter_sensor_interface() = default;

        // Pure virtual interface methods
        virtual void set_filter(rs2_embedded_filter_type embedded_filter_type, std::vector<uint8_t> params) = 0;
        virtual std::vector<uint8_t> get_filter(rs2_embedded_filter_type embedded_filter_type) = 0;
        virtual bool supports_filter(rs2_embedded_filter_type embedded_filter_type) const = 0;
    };
    MAP_EXTENSION(RS2_EXTENSION_EMBEDDED_FILTER_SENSOR, librealsense::embedded_filter_sensor_interface);

    using embedded_filters = std::vector< std::shared_ptr< embedded_filter_sensor_interface > >;

}
