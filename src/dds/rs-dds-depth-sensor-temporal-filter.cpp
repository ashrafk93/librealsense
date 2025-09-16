// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2024 RealSense, Inc. All Rights Reserved.

#include "rs-dds-depth-sensor-temporal-filter.h"
#include <stdexcept>


namespace librealsense {

    dds_depth_sensor_temporal_filter::dds_depth_sensor_temporal_filter()
        : _enabled(false)
        , _alpha(0.4f)
        , _delta(20)
        , _persistency_index(3)
    {
    }

    void dds_depth_sensor_temporal_filter::set(std::vector<uint8_t> params)
    {
        if (params.empty())
        {
            throw std::invalid_argument("Filter parameters cannot be empty");
        }

        // Parse temporal filter parameters
        // Format: [enabled_flag, alpha(4 bytes), delta(4 bytes), persistency_index]
        _enabled = (params[0] != 0);

        if (params.size() >= 5)
        {
            // Extract alpha (4 bytes float)
            std::memcpy(&_alpha, &params[1], sizeof(float));
        }

        if (params.size() >= 9)
        {
            // Extract delta (4 bytes int32_t)
            std::memcpy(&_delta, &params[5], sizeof(int32_t));
        }

        if (params.size() >= 13)
        {
            // Extract persistency index
            _persistency_index = static_cast<int32_t>(params[13]);
        }
    }

    std::vector<uint8_t> dds_depth_sensor_temporal_filter::get()
    {
        std::vector<uint8_t> data;
        data.push_back(_enabled ? 1 : 0);

        // Pack alpha as 4 bytes
        const uint8_t* alpha_bytes = reinterpret_cast<const uint8_t*>(&_alpha);
        data.insert(data.end(), alpha_bytes, alpha_bytes + sizeof(float));

        // Pack delta as 4 bytes int32_t
        const uint8_t* delta_bytes = reinterpret_cast<const uint8_t*>(&_delta);
        data.insert(data.end(), delta_bytes, delta_bytes + sizeof(int32_t));

        // Pack persistency index
        data.push_back(static_cast<uint8_t>(_persistency_index));

        return data;
    }

    void dds_depth_sensor_temporal_filter::set_enabled(bool enabled)
    {
        _enabled = enabled;
    }

    void dds_depth_sensor_temporal_filter::set_alpha(float alpha)
    {
        if (alpha < 0.0f || alpha > 1.0f)
        {
            throw std::invalid_argument("Alpha must be between 0.0 and 1.0");
        }
        _alpha = alpha;
    }

    void dds_depth_sensor_temporal_filter::set_delta(int32_t delta)
    {
        if (delta < 0)
        {
            throw std::invalid_argument("Delta must be non-negative");
        }
        _delta = delta;
    }

    void dds_depth_sensor_temporal_filter::set_persistency_index(int32_t persistency)
    {
        if (persistency < 0)
        {
            throw std::invalid_argument("Persistency index must be non-negative");
        }
        _persistency_index = persistency;
    }

}  // namespace librealsense
