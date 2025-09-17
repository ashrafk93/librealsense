// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2024 RealSense, Inc. All Rights Reserved.

#include "rs-dds-depth-sensor-temporal-filter.h"
#include <stdexcept>
#include <string>
#include <cstring>
#include <rsutils/json.h>


namespace librealsense {

    dds_depth_sensor_temporal_filter::dds_depth_sensor_temporal_filter(std::shared_ptr< realdds::dds_device > const& dev)
		: _dds_temporal_filter(std::make_shared<realdds::dds_temporal_filter>(dev))
    {
    }

    void dds_depth_sensor_temporal_filter::set(std::vector<uint8_t> params)
    {
        // Parse and validate depth sensor-specific parameters
        validate_depth_temporal_filter_params(params);

        // Delegate to DDS filter
        _dds_temporal_filter->set_filter_params(convert_to_json(params));
    }

    std::vector<uint8_t> dds_depth_sensor_temporal_filter::get()
    {
        auto json_params = _dds_temporal_filter->get_filter_params();
        return convert_from_json(json_params);
    }

    void dds_depth_sensor_temporal_filter::set_enabled(bool enabled)
    {
        _dds_temporal_filter->set_enabled(enabled);
    }

    void dds_depth_sensor_temporal_filter::set_alpha(float alpha)
    {
        _dds_temporal_filter->set_alpha(alpha);
    }

    float dds_depth_sensor_temporal_filter::get_alpha() const
    {
		return _dds_temporal_filter->get_alpha();
    }

    void dds_depth_sensor_temporal_filter::set_delta(int32_t delta)
    {
        _dds_temporal_filter->set_delta(delta);
    }

    int32_t dds_depth_sensor_temporal_filter::get_delta() const
    {
        return _dds_temporal_filter->get_delta();
    }

    void dds_depth_sensor_temporal_filter::set_persistency_index(int32_t persistency)
    {
        _dds_temporal_filter->set_persistency(persistency);
    }

    int32_t dds_depth_sensor_temporal_filter::get_persistency_index() const
    {
        return _dds_temporal_filter->get_persistency();
    }

    void dds_depth_sensor_temporal_filter::validate_depth_temporal_filter_params(const std::vector<uint8_t>& params)
    {
        //TODO enable partial setting of parameters
        // Check minimum parameter size - need at least 13 bytes:
        // 1 byte for enabled + 4 bytes for alpha (float) + 4 bytes for delta (int32_t) + 4 bytes for persistency (int32_t)
        if (params.size() < 13) {
            throw std::invalid_argument("Temporal filter parameters too small. Expected at least 13 bytes (enabled + alpha + delta + persistency)");
        }

        // Extract enabled flag (first byte)
        uint8_t enabled_byte = params[0];
        
        // Extract alpha (next 4 bytes as float, using memcpy)
        float alpha;
        std::memcpy(&alpha, &params[1], sizeof(float));
        
        // Extract delta (next 4 bytes as int32_t, using memcpy)
        int32_t delta = 0;
        std::memcpy(&delta, &params[5], sizeof(int32_t));
        
        // Extract persistency (next 4 bytes as int32_t, using memcpy)
        int32_t persistency = 0;
        std::memcpy(&persistency, &params[9], sizeof(int32_t));

        // Validate enabled flag (must be 0 or 1)
        if (enabled_byte > 1) {
            throw std::invalid_argument("Temporal filter enabled flag must be 0 or 1. Received: " + std::to_string(enabled_byte));
        }

        // Validate alpha range using defined constants
        if (alpha < ALPHA_MIN || alpha > ALPHA_MAX) {
            throw std::invalid_argument("Temporal filter alpha must be in range [" + 
                                       std::to_string(ALPHA_MIN) + ", " + std::to_string(ALPHA_MAX) + 
                                       "]. Received: " + std::to_string(alpha));
        }

        // Validate delta range using defined constants
        if (delta < DELTA_MIN || delta > DELTA_MAX) {
            throw std::invalid_argument("Temporal filter delta must be in range [" + 
                                       std::to_string(DELTA_MIN) + ", " + std::to_string(DELTA_MAX) + 
                                       "]. Received: " + std::to_string(delta));
        }

        // Validate persistency index range using defined constants
        if (persistency < PERSISTENCY_MIN || persistency > PERSISTENCY_MAX) {
            throw std::invalid_argument("Temporal filter persistency index must be in range [" + 
                                       std::to_string(PERSISTENCY_MIN) + ", " + std::to_string(PERSISTENCY_MAX) + 
                                       "]. Received: " + std::to_string(persistency));
        }

        // Validation passed - parameters are valid
    }

    rsutils::json dds_depth_sensor_temporal_filter::convert_to_json(const std::vector<uint8_t>& params)
    {
        // Validate parameters first
        validate_depth_temporal_filter_params(params);

        // Extract enabled flag (first byte)
        bool enabled = (params[0] != 0);
        
        // Extract alpha (next 4 bytes as float, using memcpy)
        float alpha;
        std::memcpy(&alpha, &params[1], sizeof(float));
        
        // Extract delta (next 4 bytes as int32_t, using memcpy)
        int32_t delta;
        std::memcpy(&delta, &params[5], sizeof(int32_t));
        
        // Extract persistency (next 4 bytes as int32_t, using memcpy)
        int32_t persistency;
        std::memcpy(&persistency, &params[9], sizeof(int32_t));

        // Create JSON object with temporal filter parameters
        rsutils::json filter_json;
        filter_json["enabled"] = enabled;
        filter_json["alpha"] = alpha;
        filter_json["delta"] = delta;
        filter_json["persistency"] = persistency;
        
        return filter_json;
    }

    std::vector<uint8_t> dds_depth_sensor_temporal_filter::convert_from_json(const rsutils::json& json_params)
    {
        std::vector<uint8_t> params(13); // 13 bytes total: 1 for enabled + 4 for alpha + 4 for delta + 4 for persistency
        
        // Extract enabled flag from JSON (default to false if not present)
        bool enabled = false;
        if (json_params.contains("enabled") && json_params["enabled"].is_boolean()) {
            enabled = json_params["enabled"].get<bool>();
        }
        
        // Extract alpha from JSON (default to ALPHA_DEFAULT if not present)
        float alpha = ALPHA_DEFAULT;
        if (json_params.contains("alpha") && json_params["alpha"].is_number()) {
            alpha = json_params["alpha"].get<float>();
        }
        
        // Extract delta from JSON (default to DELTA_DEFAULT if not present)
        int32_t delta = DELTA_DEFAULT;
        if (json_params.contains("delta") && json_params["delta"].is_number_integer()) {
            delta = json_params["delta"].get<int32_t>();
        }
        
        // Extract persistency from JSON (default to PERSISTENCY_DEFAULT if not present)
        int32_t persistency = PERSISTENCY_DEFAULT;
        if (json_params.contains("persistency") && json_params["persistency"].is_number_integer()) {
            persistency = json_params["persistency"].get<int32_t>();
        }

        // Validate parameters against defined constants
        if (alpha < ALPHA_MIN || alpha > ALPHA_MAX) {
            throw std::invalid_argument("Temporal filter alpha must be in range [" + 
                                       std::to_string(ALPHA_MIN) + ", " + std::to_string(ALPHA_MAX) + 
                                       "]. Received: " + std::to_string(alpha));
        }

        if (delta < DELTA_MIN || delta > DELTA_MAX) {
            throw std::invalid_argument("Temporal filter delta must be in range [" + 
                                       std::to_string(DELTA_MIN) + ", " + std::to_string(DELTA_MAX) + 
                                       "]. Received: " + std::to_string(delta));
        }

        if (persistency < PERSISTENCY_MIN || persistency > PERSISTENCY_MAX) {
            throw std::invalid_argument("Temporal filter persistency index must be in range [" + 
                                       std::to_string(PERSISTENCY_MIN) + ", " + std::to_string(PERSISTENCY_MAX) + 
                                       "]. Received: " + std::to_string(persistency));
        }

        // Pack enabled flag (first byte)
        params[0] = enabled ? 1 : 0;
        
        // Pack alpha using memcpy (next 4 bytes)
        std::memcpy(&params[1], &alpha, sizeof(float));
        
        // Pack delta using memcpy (next 4 bytes)
        std::memcpy(&params[5], &delta, sizeof(int32_t));
        
        // Pack persistency using memcpy (next 4 bytes)
        std::memcpy(&params[9], &persistency, sizeof(int32_t));
        
        return params;
    }

}  // namespace librealsense
