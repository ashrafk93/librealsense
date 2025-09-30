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
    class embedded_filter_interface
    {
    public:
        virtual ~embedded_filter_interface() = default;

        // Pure virtual interface methods
		virtual bool is_enabled() const = 0;
		virtual void enable(bool enable) = 0;
		virtual rs2_embedded_filter_type get_type() const = 0;
    };

    using embedded_filters = std::vector< std::shared_ptr< embedded_filter_interface > >;

}
