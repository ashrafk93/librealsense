// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.
#pragma once

#include <vector>
#include <rsutils/json-fwd.h>
#include <realdds/dds-embedded-filter.h>
#include <dds/rs-dds-embedded-filter.h>

namespace realdds {
    class dds_device; // forward declaration
}

namespace librealsense {

    // Class librealsense::rs_dds_embedded_decimation_filter: 
    // handles librealsense embedded decimation filter specific logic and parameter validation
	// Communication to HW is delegated to realdds::dds_decimation_filter
    // Decimation filter implementation
    class rs_dds_embedded_decimation_filter : public rs_dds_embedded_filter
    {
    public:
        rs_dds_embedded_decimation_filter(const std::shared_ptr< realdds::dds_embedded_filter >& dds_embedded_filter,
            set_embedded_filter_callback set_embedded_filter_cb,
            query_embedded_filter_callback query_embedded_filter_cb);
        virtual ~rs_dds_embedded_decimation_filter() = default;

        // Override interface methods
        virtual void set_filter(rs2_embedded_filter_type embedded_filter_type, std::vector<uint8_t> params) override;
        virtual std::vector<uint8_t> get_filter(rs2_embedded_filter_type embedded_filter_type) override;
        virtual bool supports_filter(rs2_embedded_filter_type embedded_filter_type) const override;

    private:
        std::shared_ptr<realdds::dds_decimation_filter> _dds_decimation_filter;

        // Validates decimation filter parameters for depth sensor
        // Expected parameter format (2 bytes minimum):
        // - Byte 0: enabled flag (0=disabled, 1=enabled)
        // - Byte 1: decimation magnitude as uint8_t (must be 2)
        void validate_filter_options(const std::vector<uint8_t>& params);
        
        rsutils::json convert_to_json(const std::vector<uint8_t>& params);
        std::vector<uint8_t> convert_from_json(const rsutils::json& json_params);

		const int32_t DECIMATION_MAGNITUDE = 2; // Decimation magnitude must always be 2
    };

}  // namespace librealsense
