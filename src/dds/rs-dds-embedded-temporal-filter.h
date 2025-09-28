// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.
#pragma once

#include <rsutils/json-fwd.h>
#include <realdds/dds-embedded-filter.h>
#include <dds/rs-dds-embedded-filter.h>


namespace librealsense {

    // Class librealsense::rs_dds_embedded_temporal_filter: 
    // handles librealsense embedded temporal filter specific logic and parameter validation
    // Communication to HW is delegated to realdds::dds_temporal_filter
    // Temporal filter implementation
    class rs_dds_embedded_temporal_filter : public rs_dds_embedded_filter
    {
    public:
        rs_dds_embedded_temporal_filter(const std::shared_ptr< realdds::dds_embedded_filter >& dds_embedded_filter,
            set_embedded_filter_callback set_embedded_filter_cb,
            query_embedded_filter_callback query_embedded_filter_cb);
        virtual ~rs_dds_embedded_temporal_filter() = default;

        // Override interface methods
        inline bool is_filter_enabled(rs2_embedded_filter_type embedded_filter_type) const override { return _enabled; }
        void enable_filter(rs2_embedded_filter_type embedded_filter_type, bool enable) override;

        // Override abstart class methods
        virtual void add_option(std::shared_ptr< realdds::dds_option > option) override;

    private:
        rsutils::json prepare_all_options_json(const rsutils::json& new_value);
        void validate_filter_options(rsutils::json options_j);

    private:
        std::shared_ptr<realdds::dds_temporal_filter> _dds_temporal_filter;
        bool _enabled;
        float _alpha;
        int32_t _delta;
        int32_t _persistency;

		static const float ALPHA_DEFAULT;
		static const int32_t DELTA_DEFAULT;
		static const int32_t PERSISTENCY_DEFAULT;
    };

}  // namespace librealsense
