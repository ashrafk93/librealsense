// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2023 RealSense, Inc. All Rights Reserved.
#pragma once

#include "rs-dds-sensor-proxy.h"
#include <src/depth-sensor.h>
#include <src/dds/rs-dds-depth-sensor-decimation-filter.h>
#include <src/dds/rs-dds-depth-sensor-temporal-filter.h>


namespace librealsense {

// For cases when checking if this is< depth_sensor > or is< depth_stereo_sensor > (like realsense-viewer::subdevice_model and on-chip-calib)
class dds_depth_sensor_proxy
    : public dds_sensor_proxy
    , public depth_stereo_sensor
    , public embedded_filter_sensor_interface
{
    using super = dds_sensor_proxy;

public:
    dds_depth_sensor_proxy(std::string const& sensor_name,
        software_device* owner,
        std::shared_ptr< realdds::dds_device > const& dev);

    // Needed by abstract interfaces
    float get_depth_scale() const override;
    float get_stereo_baseline_mm() const override;

    // Override interface methods
    void set(rs2_embedded_filter_type embedded_filter_type, std::vector<uint8_t> params) override;
    std::vector<uint8_t> get(rs2_embedded_filter_type embedded_filter_type) override;
    bool supports(rs2_embedded_filter_type embedded_filter_type) const override;

    bool extend_to( rs2_extension, void ** ptr ) override;  // extendable_interface

protected:
    void add_no_metadata( frame *, streaming_impl & ) override;
    void add_frame_metadata( frame *, rsutils::json const & md, streaming_impl & ) override;

private:
    std::unique_ptr<dds_depth_sensor_decimation_filter> _decimation_filter;
    std::unique_ptr<dds_depth_sensor_temporal_filter> _temporal_filter;
};

}  // namespace librealsense
