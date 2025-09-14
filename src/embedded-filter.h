// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#pragma once

#include "core/extension.h"
#include <vector>
#include <map>
#include <memory>
#include <cstdint>

namespace librealsense {
    // Interface class for embedded filter sensors
    class embedded_filter_interface
    {
    public:
        virtual ~embedded_filter_interface() = default;

        // Pure virtual interface methods
        virtual void set(rs2_embedded_filter_type embedded_filter_type, std::vector<uint8_t> params) = 0;
        virtual std::vector<uint8_t> get(rs2_embedded_filter_type embedded_filter_type) = 0;
        virtual bool supports(rs2_embedded_filter_type embedded_filter_type) const = 0;
		virtual bool is_enabled() const = 0;
    };
    MAP_EXTENSION(RS2_EXTENSION_EMBEDDED_FILTER_SENSOR, librealsense::embedded_filter_interface);

    using embedded_filters = std::vector< std::shared_ptr< embedded_filter_interface > >;


    // Decimation filter implementation
    class embedded_decimation_filter : public embedded_filter_interface
    {
    public:
        embedded_decimation_filter();
        virtual ~embedded_decimation_filter() = default;

        // Override interface methods
        void set(rs2_embedded_filter_type embedded_filter_type, std::vector<uint8_t> params) override;
        std::vector<uint8_t> get(rs2_embedded_filter_type embedded_filter_type) override;
        bool supports(rs2_embedded_filter_type embedded_filter_type) const override;
        bool is_enabled() const override { return _enabled; }

        // Decimation-specific methods
        void set_enabled(bool enabled);
        int32_t get_magnitude() const { return _magnitude; }

    private:
        bool _enabled;
        int32_t _magnitude; // Always 2 for decimation
    };

    // Temporal filter implementation
    class embedded_temporal_filter : public embedded_filter_interface
    {
    public:
        embedded_temporal_filter();
        virtual ~embedded_temporal_filter() = default;

        // Override interface methods
        void set(rs2_embedded_filter_type embedded_filter_type, std::vector<uint8_t> params) override;
        std::vector<uint8_t> get(rs2_embedded_filter_type embedded_filter_type) override;
        bool supports(rs2_embedded_filter_type embedded_filter_type) const override;
        bool is_enabled() const override { return _enabled; }

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
}
