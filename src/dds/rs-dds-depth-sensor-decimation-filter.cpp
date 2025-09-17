// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include "rs-dds-depth-sensor-decimation-filter.h"
#include <stdexcept>
#include <string>
#include <cstring>
#include <rsutils/json.h>


namespace librealsense {

    dds_depth_sensor_decimation_filter::dds_depth_sensor_decimation_filter(std::shared_ptr< realdds::dds_device > const& dev)
        : _dds_decimation_filter(std::make_shared<realdds::dds_decimation_filter>(dev))
    {
    }

    void dds_depth_sensor_decimation_filter::set(std::vector<uint8_t> params)
    {
        // Parse and validate depth sensor-specific parameters
        validate_depth_decimation_filter_params(params);

        // Delegate to DDS filter
        _dds_decimation_filter->set_filter_params(convert_to_json(params));
    }

    std::vector<uint8_t> dds_depth_sensor_decimation_filter::get()
    {
        auto json_params = _dds_decimation_filter->get_filter_params();
        return convert_from_json(json_params);
    }

    void dds_depth_sensor_decimation_filter::set_enabled(bool enabled)
    {
        _dds_decimation_filter->set_enabled(enabled);
    }

    int32_t dds_depth_sensor_decimation_filter::get_magnitude() const 
    {
        return _dds_decimation_filter->get_decimation_factor();
    }

    void dds_depth_sensor_decimation_filter::validate_depth_decimation_filter_params(const std::vector<uint8_t>& params)
    {
		// TODO enable partial setting of parameters
        // Check minimum parameter size - need at least 2 bytes (1 for enabled + 1 for magnitude)
        if (params.size() < 2) {
            throw std::invalid_argument("Decimation filter parameters too small. Expected at least 2 bytes (enabled + magnitude)");
        }

        // Extract enabled flag (first byte)
        bool enabled = (params[0] != 0);
        
        // Extract decimation magnitude (next 4 bytes as int32_t, using memcpy)
		int32_t magnitude = static_cast<int32_t>(params[1]);

        // Validate that decimation magnitude is always 2 (as per requirements)
        if (magnitude != DECIMATION_MAGNITUDE) {
            throw std::invalid_argument("Decimation filter magnitude must be " + std::to_string(DECIMATION_MAGNITUDE) + ". Received: " + std::to_string(magnitude));
        }

        // Additional validation: enabled flag should be 0 or 1
        if (params[0] > 1) {
            throw std::invalid_argument("Decimation filter enabled flag must be 0 or 1. Received: " + std::to_string(params[0]));
        }

        // Validation passed - parameters are valid
    }

    rsutils::json dds_depth_sensor_decimation_filter::convert_to_json(const std::vector<uint8_t>& params)
    {
        // Validate parameters first
        validate_depth_decimation_filter_params(params);

        // Extract enabled flag (first byte)
        bool enabled = (params[0] != 0);
        
        // Extract decimation magnitude (next 4 bytes as int32_t, using memcpy)
        int32_t magnitude = static_cast<int32_t>(params[1]);

        // Create JSON object with decimation parameters
        rsutils::json filter_json;
        filter_json["enabled"] = enabled;
        filter_json["magnitude"] = magnitude;
        
        return filter_json;
    }

    std::vector<uint8_t> dds_depth_sensor_decimation_filter::convert_from_json(const rsutils::json& json_params)
    {
        std::vector<uint8_t> params(2); // 2 bytes total: 1 for enabled + 1 for magnitude
        
        // Extract enabled flag from JSON (default to false if not present)
        bool enabled = false;
        if (json_params.contains("enabled") && json_params["enabled"].is_boolean()) {
            enabled = json_params["enabled"].get<bool>();
        }
        
        // Extract magnitude from JSON (default to DECIMATION_MAGNITUDE if not present)
        int32_t magnitude = DECIMATION_MAGNITUDE;
        if (json_params.contains("magnitude") && json_params["magnitude"].is_number_integer()) {
            magnitude = json_params["magnitude"].get<int32_t>();
        }
        
        // Validate magnitude (must be exactly DECIMATION_MAGNITUDE)
        if (magnitude != DECIMATION_MAGNITUDE) {
            throw std::invalid_argument("Decimation filter magnitude must be " + std::to_string(DECIMATION_MAGNITUDE) + ". Received: " + std::to_string(magnitude));
        }

        // Pack enabled flag (first byte)
        params[0] = enabled ? 1 : 0;
        
        // Pack magnitude using memcpy (next byte)
        params[1] = static_cast<uint8_t>(magnitude);
        
        return params;
    }
}  // namespace librealsense
