// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#pragma once

#include "core/extension.h"
#include <vector>
#include <map>
#include <memory>
#include <cstdint>
#include <librealsense2/h/rs_option.h>
#include <src/core/option-interface.h>

namespace librealsense {
    // Interface class for embedded filter sensors
    class embedded_filter_sensor_interface
    {
    public:
        virtual ~embedded_filter_sensor_interface() = default;

        // Pure virtual interface methods
		virtual bool is_filter_enabled(rs2_embedded_filter_type embedded_filter_type) const = 0;
		virtual void enable_filter(rs2_embedded_filter_type embedded_filter_type, bool enable) = 0;
		virtual std::vector<rs2_option> get_filter_supported_options(rs2_embedded_filter_type embedded_filter_type) const = 0;
        virtual option& get_filter_option(rs2_option option_id, rs2_embedded_filter_type embedded_filter_type) const = 0;
    };
    MAP_EXTENSION(RS2_EXTENSION_EMBEDDED_FILTER_SENSOR, librealsense::embedded_filter_sensor_interface);

    using embedded_filters = std::vector< std::shared_ptr< embedded_filter_sensor_interface > >;

}
