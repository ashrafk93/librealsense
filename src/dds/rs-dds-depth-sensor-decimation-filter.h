// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.
#pragma once

#include <vector>
#include <rsutils/json-fwd.h>
#include <realdds/dds-embedded-filter.h>

namespace realdds {
    class dds_device; // forward declaration
}

namespace librealsense {

    // Class librealsense::dds_depth_sensor_decimation_filter: 
    // handles librealsense depth sensor - specific logic and parameter validation
	// Communication to HW is delegated to realdds::dds_decimation_filter
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
        int32_t get_magnitude() const;

    private:
        std::shared_ptr<realdds::dds_decimation_filter> _dds_decimation_filter;

        // Validates decimation filter parameters for depth sensor
        // Expected parameter format (2 bytes minimum):
        // - Byte 0: enabled flag (0=disabled, 1=enabled)
        // - Byte 1: decimation magnitude as uint8_t (must be 2)
        void validate_depth_decimation_filter_options(const std::vector<uint8_t>& params);
        
        rsutils::json convert_to_json(const std::vector<uint8_t>& params);
        std::vector<uint8_t> convert_from_json(const rsutils::json& json_params);

		const int32_t DECIMATION_MAGNITUDE = 2; // Decimation magnitude must always be 2
    };

}  // namespace librealsense
