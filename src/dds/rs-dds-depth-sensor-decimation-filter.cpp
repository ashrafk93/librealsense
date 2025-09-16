// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include "rs-dds-depth-sensor-decimation-filter.h"
#include <stdexcept>


namespace librealsense {

    dds_depth_sensor_decimation_filter::dds_depth_sensor_decimation_filter()
        : _enabled(false)
        , _magnitude(2) // Always 2 for decimation
    {
    }

    void dds_depth_sensor_decimation_filter::set(std::vector<uint8_t> params)
    {
        if (params.empty())
        {
            throw std::invalid_argument("Filter parameters cannot be empty");
        }

        // Parse decimation filter parameters
        // Format: [enabled_flag] (magnitude is always 2)
        _enabled = (params[0] != 0);

        // Magnitude is fixed to 2 for decimation filter; ignore any additional params
        // TODO - set the data in the device instead of caching


    }

    std::vector<uint8_t> dds_depth_sensor_decimation_filter::get()
    {
        // TODO - Must pull the data from the device instead of returning cached values
        std::vector<uint8_t> data;
        data.push_back(_enabled ? 1 : 0);
        data.push_back(static_cast<uint8_t>(_magnitude)); // Always 2
        return data;
    }

    void dds_depth_sensor_decimation_filter::set_enabled(bool enabled)
    {
        _enabled = enabled;
    }
}  // namespace librealsense
