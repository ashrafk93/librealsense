// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.
#pragma once

#include <vector>
#include <rsutils/json-fwd.h>
#include <realdds/dds-embedded-filter.h>


namespace librealsense {

    // Class librealsense::dds_depth_sensor_temporal_filter: 
    // handles librealsense depth sensor - specific logic and parameter validation
    // Communication to HW is delegated to realdds::dds_temporal_filter
    // Temporal filter implementation
    class dds_depth_sensor_temporal_filter
    {
    public:
        dds_depth_sensor_temporal_filter(std::shared_ptr< realdds::dds_device > const& dev);
        virtual ~dds_depth_sensor_temporal_filter() = default;

        // Override interface methods
        void set(std::vector<uint8_t> params);
        std::vector<uint8_t> get();

        // Temporal-specific methods
        void set_enabled(bool enabled);
        void set_alpha(float alpha);
        float get_alpha() const;
        void set_delta(int32_t delta);
        int32_t get_delta() const;
        void set_persistency_index(int32_t persistency);
        int32_t get_persistency_index() const;

    private:
        std::shared_ptr<realdds::dds_temporal_filter> _dds_temporal_filter;

        // Validates temporal filter parameters for depth sensor
        // Expected parameter format (13 bytes minimum):
        // - Byte 0: enabled flag (0=disabled, 1=enabled)
        // - Bytes 1-4: alpha as float little-endian, range [0.0, 1.0], default 0.4, increment 0.01
        // - Bytes 5-8: delta as int32_t little-endian, range [0, 100], default 20, increment 1
        // - Bytes 9-12: persistency index as int32_t little-endian, range [0, 8], default 3, increment 1
        void validate_depth_temporal_filter_params(const std::vector<uint8_t>& params);
        rsutils::json convert_to_json(const std::vector<uint8_t>& params);
        std::vector<uint8_t> convert_from_json(const rsutils::json& json_params);

		const float ALPHA_MIN = 0.0f;
		const float ALPHA_MAX = 1.0f;
		const float ALPHA_STEP = 0.01f;
		const float ALPHA_DEFAULT = 0.4f;
		const int32_t DELTA_MIN = 0;
		const int32_t DELTA_MAX = 100;
		const int32_t DELTA_STEP = 1;
		const int32_t DELTA_DEFAULT = 20;
		const int32_t PERSISTENCY_MIN = 0;
		const int32_t PERSISTENCY_MAX = 8;
		const int32_t PERSISTENCY_STEP = 1;
		const int32_t PERSISTENCY_DEFAULT = 3;        
    };

}  // namespace librealsense
