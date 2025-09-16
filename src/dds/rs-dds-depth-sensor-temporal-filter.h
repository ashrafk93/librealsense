// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.
#pragma once

#include <vector>


namespace librealsense {

    // Temporal filter implementation
    class dds_depth_sensor_temporal_filter
    {
    public:
        dds_depth_sensor_temporal_filter();
        virtual ~dds_depth_sensor_temporal_filter() = default;

        // Override interface methods
        void set(std::vector<uint8_t> params);
        std::vector<uint8_t> get();

        // Temporal-specific methods
        void set_enabled(bool enabled);
        void set_alpha(float alpha);
        float get_alpha() const { return _alpha; }
        void set_delta(int32_t delta);
        int32_t get_delta() const { return _delta; }
        void set_persistency_index(int32_t persistency);
        int32_t get_persistency_index() const { return _persistency_index; }

    private:
        bool _enabled;
        float _alpha;
        int32_t _delta;
        int32_t _persistency_index;
    };

}  // namespace librealsense
