// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.
#pragma once

#include <vector>
#include <rsutils/json-fwd.h>


namespace librealsense {

    // Decimation filter implementation
    class dds_depth_sensor_decimation_filter
    {
    public:
        dds_depth_sensor_decimation_filter();
        virtual ~dds_depth_sensor_decimation_filter() = default;

        // Override interface methods
        void set(std::vector<uint8_t> params);
        std::vector<uint8_t> get();

        // Decimation-specific methods
        void set_enabled(bool enabled);
        int32_t get_magnitude() const { return _magnitude; }

    private:
        bool _enabled;
        int32_t _magnitude; // Always 2 for decimation
    };

}  // namespace librealsense
