// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.
#pragma once

#include <src/embedded-filter.h>

#include <functional>
#include <memory>
#include <rsutils/json.h>


namespace realdds {
class dds_embedded_filter;
}  // namespace realdds


namespace librealsense {


// A facade for a realdds::dds_embedded_filter exposing librealsense interface
// TODO - check if needed
class rs_dds_embedded_filter : public embedded_filter_sensor_interface
{
    std::shared_ptr< realdds::dds_embedded_filter > _dds_ef;
    rs2_embedded_filter_type const _rs_type;

public:
    typedef std::function< void( rsutils::json value ) > set_embedded_filter_callback;
    typedef std::function< rsutils::json() > query_embedded_filter_callback;

private:
    set_embedded_filter_callback _set_ef_cb;
    query_embedded_filter_callback _query_ef_cb;

public:
    rs_dds_embedded_filter( const std::shared_ptr< realdds::dds_embedded_filter > & dds_embedded_filter,
                   set_embedded_filter_callback set_embedded_filter_cb,
                   query_embedded_filter_callback query_embedded_filter_cb );

    // Override interface methods
    virtual void set(rs2_embedded_filter_type embedded_filter_type, std::vector<uint8_t> params) override;
    virtual std::vector<uint8_t> get(rs2_embedded_filter_type embedded_filter_type) override;
    virtual bool supports(rs2_embedded_filter_type embedded_filter_type) const override;
};



}  // namespace librealsense
