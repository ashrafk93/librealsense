// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include "rs-dds-embedded-decimation-filter.h"
#include <stdexcept>
#include <string>
#include <cstring>
#include <rsutils/json.h>

using rsutils::json;

namespace librealsense {

    rs_dds_embedded_decimation_filter::rs_dds_embedded_decimation_filter(const std::shared_ptr< realdds::dds_embedded_filter >& dds_embedded_filter,
        set_embedded_filter_callback set_embedded_filter_cb,
        query_embedded_filter_callback query_embedded_filter_cb)
		: rs_dds_embedded_filter(dds_embedded_filter, set_embedded_filter_cb, query_embedded_filter_cb)
        , _dds_decimation_filter(std::make_shared<realdds::dds_decimation_filter>())
    {
    }

    void rs_dds_embedded_decimation_filter::set_filter(rs2_embedded_filter_type embedded_filter_type, std::vector<uint8_t> params)
    {
        if (!_set_ef_cb)
            throw std::runtime_error("Set embedded filter callback is not set for filter " + _dds_ef->get_name());

		json j_value = convert_to_json(params);

        validate_filter_options(params);

        _set_ef_cb(j_value);

        // Delegate to DDS filter
        _dds_decimation_filter->set_options(convert_to_json(params));
    }

    std::vector<uint8_t> rs_dds_embedded_decimation_filter::get_filter(rs2_embedded_filter_type embedded_filter_type)
    {
        if (!_query_ef_cb)
            throw std::runtime_error("Query Embedded Filter callback is not set for filter " + _dds_ef->get_name());

        auto const params = _query_ef_cb();

        auto filter_options_j = params.at("options");
		std::vector<uint8_t> ans = convert_from_json(filter_options_j);

        validate_filter_options(ans);

        return ans;
    }

    bool rs_dds_embedded_decimation_filter::supports_filter(rs2_embedded_filter_type embedded_filter_type) const
    {
        return true;
    }

    void rs_dds_embedded_decimation_filter::validate_filter_options(const std::vector<uint8_t>& params)
    {
		// TODO enable partial setting of parameters
        // Check minimum parameter size - need at least 2 bytes (1 for enabled + 1 for magnitude)
        if (params.size() < 2) {
            throw std::invalid_argument("Decimation filter parameters too small. Expected at least 2 bytes (enabled + magnitude)");
        }

        // Extract enabled flag (first byte)
        bool toggle = (params[0] != 0);
        
        // Extract decimation magnitude (next 4 bytes as int32_t, using memcpy)
		int32_t magnitude = static_cast<int32_t>(params[1]);

        // Validate that decimation magnitude is always 2 (as per requirements)
        if (magnitude != DECIMATION_MAGNITUDE) {
            throw std::invalid_argument("Decimation filter magnitude must be " + std::to_string(DECIMATION_MAGNITUDE) + ". Received: " + std::to_string(magnitude));
        }

        // Validation passed - parameters are valid
    }

    rsutils::json rs_dds_embedded_decimation_filter::convert_to_json(const std::vector<uint8_t>& params)
    {
        // Validate parameters first
        validate_filter_options(params);

        // Extract enabled flag (first byte)
        bool toggle = (params[0] != 0);
        
        // Extract decimation magnitude (next 4 bytes as int32_t, using memcpy)
        int32_t magnitude = static_cast<int32_t>(params[1]);

        // Create JSON object with decimation parameters
        rsutils::json filter_json;
        filter_json["Toggle"] = static_cast<int32_t>(toggle);
        filter_json["Magnitude"] = magnitude;
        
        return filter_json;
    }

    std::vector<uint8_t> rs_dds_embedded_decimation_filter::convert_from_json(const rsutils::json& json_params)
    {
        std::vector<uint8_t> params(2); // 2 bytes total: 1 for enabled + 1 for magnitude
        
        // Extract enabled flag from JSON (default to false if not present)
        bool toggle = false;
        if (json_params.contains("Toggle")) {
            toggle = json_params["Toggle"].get<int32_t>();
        }
        
        // Extract magnitude from JSON (default to DECIMATION_MAGNITUDE if not present)
        int32_t magnitude = DECIMATION_MAGNITUDE;
        if (json_params.contains("Magnitude")) {
            magnitude = json_params["Magnitude"].get<int32_t>();
        }
        
        // Validate magnitude (must be exactly DECIMATION_MAGNITUDE)
        if (magnitude != DECIMATION_MAGNITUDE) {
            throw std::invalid_argument("Decimation filter magnitude must be " + std::to_string(DECIMATION_MAGNITUDE) + ". Received: " + std::to_string(magnitude));
        }

        // Pack enabled flag (first byte)
        params[0] = toggle ? 1 : 0;
        
        // Pack magnitude using memcpy (next byte)
        params[1] = static_cast<uint8_t>(magnitude);
        
        return params;
    }
}  // namespace librealsense
