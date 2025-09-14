// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include "embedded-filter.h"
#include <stdexcept>

namespace librealsense {

// ========== Embedded Decimation Filter Implementation ==========

embedded_decimation_filter::embedded_decimation_filter()
    : _enabled(false)
    , _magnitude(2) // Always 2 for decimation
{
}

void embedded_decimation_filter::set(rs2_embedded_filter_type embedded_filter_type, std::vector<uint8_t> params)
{
    if (embedded_filter_type != RS2_EMBEDDED_FILTER_TYPE_DECIMATION)
    {
        throw std::invalid_argument("Decimation filter only supports RS2_EMBEDDED_FILTER_DECIMATION type");
    }

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

std::vector<uint8_t> embedded_decimation_filter::get(rs2_embedded_filter_type embedded_filter_type)
{
    if (embedded_filter_type != RS2_EMBEDDED_FILTER_TYPE_DECIMATION)
    {
        throw std::invalid_argument("Decimation filter only supports RS2_EMBEDDED_FILTER_DECIMATION type");
    }

    // TODO - Must pull the data from the device instead of returning cached values
    std::vector<uint8_t> data;
    data.push_back(_enabled ? 1 : 0);
    data.push_back(static_cast<uint8_t>(_magnitude)); // Always 2
    return data;
}

bool embedded_decimation_filter::supports(rs2_embedded_filter_type embedded_filter_type) const
{
    // TODO - check if the device actually supports decimation filter
    return embedded_filter_type == RS2_EMBEDDED_FILTER_TYPE_DECIMATION;
}

void embedded_decimation_filter::set_enabled(bool enabled)
{
    _enabled = enabled;
}

// ========== Embedded Temporal Filter Implementation ==========

embedded_temporal_filter::embedded_temporal_filter()
    : _enabled(false)
    , _alpha(0.4f)
    , _delta(20)
    , _persistency_index(3)
{
}

void embedded_temporal_filter::set(rs2_embedded_filter_type embedded_filter_type, std::vector<uint8_t> params)
{
    if (embedded_filter_type != RS2_EMBEDDED_FILTER_TYPE_TEMPORAL)
    {
        throw std::invalid_argument("Temporal filter only supports RS2_EMBEDDED_FILTER_TEMPORAL type");
    }

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

std::vector<uint8_t> embedded_temporal_filter::get(rs2_embedded_filter_type embedded_filter_type)
{
    if (embedded_filter_type != RS2_EMBEDDED_FILTER_TYPE_TEMPORAL)
    {
        throw std::invalid_argument("Temporal filter only supports RS2_EMBEDDED_FILTER_TEMPORAL type");
    }

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

bool embedded_temporal_filter::supports(rs2_embedded_filter_type embedded_filter_type) const
{
    return embedded_filter_type == RS2_EMBEDDED_FILTER_TYPE_TEMPORAL;
}

void embedded_temporal_filter::set_enabled(bool enabled)
{
    _enabled = enabled;
}

void embedded_temporal_filter::set_alpha(float alpha)
{
    if (alpha < 0.0f || alpha > 1.0f)
    {
        throw std::invalid_argument("Alpha must be between 0.0 and 1.0");
    }
    _alpha = alpha;
}

void embedded_temporal_filter::set_delta(int32_t delta)
{
    if (delta < 0)
    {
        throw std::invalid_argument("Delta must be non-negative");
    }
    _delta = delta;
}

void embedded_temporal_filter::set_persistency_index(int32_t persistency)
{
    if (persistency < 0)
    {
        throw std::invalid_argument("Persistency index must be non-negative");
    }
    _persistency_index = persistency;
}

} // namespace librealsense