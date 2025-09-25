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
class rs_dds_embedded_filter : public embedded_filter_sensor_interface
{
public:
    typedef std::function< void( rsutils::json value ) > set_embedded_filter_callback;
    typedef std::function< rsutils::json() > query_embedded_filter_callback;

protected:
    std::shared_ptr< realdds::dds_embedded_filter > _dds_ef;
    rs2_embedded_filter_type const _filter_type;
    set_embedded_filter_callback _set_ef_cb;
    query_embedded_filter_callback _query_ef_cb;

public:
    rs_dds_embedded_filter( const std::shared_ptr< realdds::dds_embedded_filter > & dds_embedded_filter,
                   set_embedded_filter_callback set_embedded_filter_cb,
                   query_embedded_filter_callback query_embedded_filter_cb );

    // Not Overriding interface methods - should be done in inheriting classes

    rs2_embedded_filter_type get_type() const { return _filter_type; }
};



}  // namespace librealsense
